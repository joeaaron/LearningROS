#include <ros/package.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include "lighthouse_recognize.h"

using namespace std;

#define RadianToAngle(X) ((X)*180.0/M_PI)
#define AngleToRadian(X) ((X)*M_PI/180.0)
#define SIGN(n) (n <= 0?( n < 0 ?-1:0):1)

LighthouseRecognize::LighthouseRecognize()
    :nh_(),
    laser_scan_sub_(),
    lighthouse_pose_in_map_sub_(),
    lighthouse_fit_display_pub_(),
    lighthouse_pose_pub_(),
    lighthouse_type_("triangle"),
    lighthouse_length_(),
    lighthouse_start_extend_scan_index_(0),
    lighthouse_end_extend_scan_index_(0),
    laser_msg_(),
    pose_quaternion_invalid_(),
    lighthouse_pose2d_in_laser_upright_(),
    frame_id_map_("map"),
    frame_id_agv_("base_footprint"),
    lighthouse_in_laser_upright_coordinate_(),
    get_new_lighthouse_param_(false),
    status_is_run_(false),
    laser_stand_upright_(true),
    get_new_laser_scan_(false),
    get_new_lighthouse_pose_in_map_(false),
    need_trans_lighthouse_pose_from_map_to_laser_(true),
    new_dock_first_recognize_(true),
    lighthouse_pose_stamped_in_map_(),
    Slaser_scan_msg_()
{
    laser_scan_sub_ = nh_.subscribe("/scan_filtered_lighthouse", 1, &LighthouseRecognize::LaserScanCallback, this );
    lighthouse_pose_in_map_sub_ = nh_.subscribe("/lighthouse_pose_in_map", 1, &LighthouseRecognize::LighthousePoseInMapCallback, this );

    lighthouse_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped> ( "/lighthouse_pose", 1 );
    lighthouse_fit_display_pub_ = nh_.advertise<std_msgs::Float32MultiArray> ( "/lighthouse_fit_display", 1 );

    dsrv_ = new dynamic_reconfigure::Server<lighthouse_navigation::LighthouseRecognizeConfig>(ros::NodeHandle("~"));
    dynamic_reconfigure::Server<lighthouse_navigation::LighthouseRecognizeConfig>::CallbackType cb = boost::bind(&LighthouseRecognize::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
}

LighthouseRecognize::~LighthouseRecognize(){}

void LighthouseRecognize::GetParam()
{
    ros::NodeHandle nh_private("~");

    //共同参数
    nh_private.param("lighthouse_extend_angle", lighthouse_extend_angle_, 5);

    //三角板参数
    nh_private.param("triangle_angle_included", triangle_angle_included_, 120.0);
    nh_private.param("max_detecte_distance_triangle_to_laser", max_detecte_distance_triangle_to_laser_, 1.6);
    nh_private.param("max_detecte_angle_triangle_to_laser", max_detecte_angle_triangle_to_laser_, 130.0);
    nh_private.param("num_of_point_first_fit_used", num_of_point_first_fit_used_, 4);
    nh_private.param("max_distance_of_point_to_line", max_distance_of_point_to_line_, 0.013);
    nh_private.param("point_num_of_next_several_point", point_num_of_next_several_point_, 10);
    nh_private.param("min_continuity_point_num_consider_line_far", min_continuity_point_num_consider_line_far_, 15);
    nh_private.param("min_continuity_point_num_consider_line_near", min_continuity_point_num_consider_line_near_, 10);
    nh_private.param("line_cross_tolerate_discontinuity_point", line_cross_tolerate_discontinuity_point_, 15);
    nh_private.param("line_cross_tolerate_offset_angle", line_cross_tolerate_offset_angle_, 6.5);

    //反光胶贴参数
    nh_private.param("min_intensite_consider_reflector_point", min_intensite_consider_reflector_point_, 1000);
    nh_private.param("min_point_num_consider_belt", min_point_num_consider_belt_, 7);
    nh_private.param("max_detecte_angle_belt_to_laser", max_detecte_angle_belt_to_laser_, 130.0);
    nh_private.param("max_detecte_distance_belt_to_laser", max_detecte_distance_belt_to_laser_, 7.0);
    nh_private.param("max_tolerate_diff_belt_length_detected", max_tolerate_diff_belt_length_detected_, 0.1);

    nh_private.param("/tf_base_laser/roll",laser_roll_, 0.0);
    nh_private.param("/tf_base_laser/x",laser_in_agv_x_, 0.0);

    if ( fabs(laser_roll_ - M_PI) < 0.02 )
    {
        laser_stand_upright_ = false;
    }
}

void LighthouseRecognize::InitPoseQuaternionInvalid()
{
    pose_quaternion_invalid_.header.frame_id = "invalid_pose";
    pose_quaternion_invalid_.header.stamp = ros::Time::now();
    pose_quaternion_invalid_.pose.position.x = 0.0;
    pose_quaternion_invalid_.pose.position.y = 0.0;
    pose_quaternion_invalid_.pose.position.z = 0.0;
    pose_quaternion_invalid_.pose.orientation.x = 0.0;
    pose_quaternion_invalid_.pose.orientation.y = 0.0;
    pose_quaternion_invalid_.pose.orientation.z = 0.0;
    pose_quaternion_invalid_.pose.orientation.w = 0.0;
}

void LighthouseRecognize::LaserScanCallback(const sensor_msgs::LaserScanPtr &laser_scan_msg)
{
    if ( !status_is_run_ )
    {
        return;
    }

    laser_msg_ = *laser_scan_msg;
    Slaser_scan_msg_.total_point = laser_scan_msg->ranges.size();
    Slaser_scan_msg_.angle_resolution = RadianToAngle( laser_scan_msg->angle_increment );
    get_new_laser_scan_ = true;
}

void LighthouseRecognize::LighthousePoseInMapCallback(const geometry_msgs::PosePtr &lighthouse_pose_in_map_msg)
{
    try
    {
        tf::assertQuaternionValid(lighthouse_pose_in_map_msg->orientation);
    }
    catch (tf2::InvalidArgumentException &e)//停止识别，进入空闲等待状态
    {
        ROS_ERROR( "assertQuaternionValid: %s", e.what() );//Quaternion malformed
        TurnToIdle();
        return;
    }

    lighthouse_pose_stamped_in_map_.pose = *lighthouse_pose_in_map_msg;
    get_new_lighthouse_pose_in_map_ = true;
    need_trans_lighthouse_pose_from_map_to_laser_ = true;

    return;
}

void LighthouseRecognize::reconfigureCB(lighthouse_navigation::LighthouseRecognizeConfig config, uint32_t level)
{
    ROS_INFO("reconfigureCB config.type_and_one_side_length: %s", config.type_and_one_side_length.c_str());

    std::vector<std::string> splited_str;
    boost::split(splited_str, config.type_and_one_side_length, boost::is_any_of(":"));

    for (size_t i = 0; i < splited_str.size(); i++)
    {
        cout << "splited_str " << i << ": " << splited_str[i] << endl;
    }

    static bool get_reconfigure = false;

    if (!splited_str[0].empty() && !splited_str[1].empty())
    {
        if ( splited_str[0] == "triangle" || splited_str[0] == "reflector" )
        {
            lighthouse_type_ = splited_str[0];
            lighthouse_length_ = boost::lexical_cast<double>(splited_str[1]);

            if ( get_reconfigure )
            {
                get_new_lighthouse_param_ = true;
            }
            get_reconfigure = true;
        }
        else
        {
            ROS_ERROR("Undefined lighthouse type: %s", splited_str[0].c_str() );
        }
    }
    else
    {
        ROS_ERROR("The dynparam should be \"type:one_side_lenght\", recv is: %s", config.type_and_one_side_length.c_str() );
        return;
    }
}

void LighthouseRecognize::TurnToIdle()
{
    get_new_lighthouse_param_ = false;
    get_new_lighthouse_pose_in_map_ = false;

    new_dock_first_recognize_ = true;
    get_new_laser_scan_ = false;
}

//三角在正立的雷达坐标系下的坐标
bool LighthouseRecognize::TransLighthousePoseFromMapToLaserUpright()
{
    lighthouse_pose_stamped_in_map_.header.frame_id = frame_id_map_;
    lighthouse_pose_stamped_in_map_.header.stamp = ros::Time();

    static tf::TransformListener tf_listener;
    geometry_msgs::PoseStamped triangle_pose_stamped_in_agv;
    tf_listener.waitForTransform(frame_id_agv_, frame_id_map_, ros::Time(),ros::Duration(1.0));
    try
    {
        tf_listener.transformPose(frame_id_agv_, lighthouse_pose_stamped_in_map_, triangle_pose_stamped_in_agv);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("trans pose from map to agv error: %s",ex.what());
        return false;
    }

    lighthouse_pose2d_in_laser_upright_.x = triangle_pose_stamped_in_agv.pose.position.x - laser_in_agv_x_;
    lighthouse_pose2d_in_laser_upright_.y = triangle_pose_stamped_in_agv.pose.position.y;
    lighthouse_pose2d_in_laser_upright_.theta = tf::getYaw(triangle_pose_stamped_in_agv.pose.orientation);
//    cout << "lighthouse_pose2d_in_laser_upright_.x: " << lighthouse_pose2d_in_laser_upright_.x << endl;
//    cout << "lighthouse_pose2d_in_laser_upright_.y: " << lighthouse_pose2d_in_laser_upright_.y << endl;
//    cout << "lighthouse_pose2d_in_laser_upright_.theta: " << RadianToAngle(lighthouse_pose2d_in_laser_upright_.theta) << endl;

    return true;
}

void LighthouseRecognize::LighthouseScanPointDisplayPub()
{
    std_msgs::Float32MultiArray lighthouse_points;
    for ( int i = 0; i < lighthouse_in_laser_upright_coordinate_.cols(); ++i )
    {
        lighthouse_points.data.push_back(lighthouse_in_laser_upright_coordinate_(0, i));
        lighthouse_points.data.push_back(lighthouse_in_laser_upright_coordinate_(1, i));
    }
    lighthouse_fit_display_pub_.publish(lighthouse_points);
}


void LighthouseRecognize::LighthouseFitDisplayPub( TriangleFit Striangle_fit )
{
    std_msgs::Float32MultiArray lighthouse_points;
    for ( int i = 0; i < lighthouse_in_laser_upright_coordinate_.cols(); ++i )
    {
        lighthouse_points.data.push_back(lighthouse_in_laser_upright_coordinate_(0, i));
        lighthouse_points.data.push_back(lighthouse_in_laser_upright_coordinate_(1, i));
    }

    lighthouse_points.data.push_back(Striangle_fit.line_first_k);
    lighthouse_points.data.push_back(Striangle_fit.line_first_b);
    lighthouse_points.data.push_back(Striangle_fit.line_seconde_k);
    lighthouse_points.data.push_back(Striangle_fit.line_seconde_b);

    lighthouse_fit_display_pub_.publish(lighthouse_points);
}

void LighthouseRecognize::CheckLighthouseDetectCondition(double distance_triangle_to_laser, double max_distance, double max_angle)
{
    if ( distance_triangle_to_laser > max_distance )
    {
        ROS_WARN( "distance of triangle to laser is out of dock range %f", distance_triangle_to_laser );
    }

    if ( fabs( RadianToAngle(lighthouse_pose2d_in_laser_upright_.theta) ) > max_angle )
    {
        ROS_WARN("triangle middle line angle with axisX out of range %f",RadianToAngle(lighthouse_pose2d_in_laser_upright_.theta));
    }
}

double LighthouseRecognize::CalculateSidecBySideaSidebAnglec(double side_a, double side_b, double angle_c)
{
    double side_c = sqrt( pow(side_a,2) + pow(side_b,2) - 2.0 * side_a * side_b * cos(angle_c) );
    return side_c;
}

double LighthouseRecognize::CalculateAngleaBySideaSidebAnglec(double side_a, double side_b, double angle_c)
{
    double side_c = CalculateSidecBySideaSidebAnglec(side_a, side_b, angle_c);
    double angle_a = acos( ( pow(side_b,2) + pow(side_c,2) - pow(side_a,2) ) / ( 2.0 * side_b * side_c ) );
    return angle_a;
}

bool LighthouseRecognize::CalculateLighthouseExtendScanIndexFromLastPose(double distance_lighthouse_to_laser, double angle_lighthouse_with_laser_axisX)
{
    int middle_laser_scan_point_index = ( Slaser_scan_msg_.total_point + 1 )/2;

    float radian_laser_with_lighthouse_one_side = CalculateAngleaBySideaSidebAnglec(lighthouse_length_, distance_lighthouse_to_laser, angle_lighthouse_with_laser_axisX);
    float angle_laser_with_lighthouse_one_side_extend = RadianToAngle(radian_laser_with_lighthouse_one_side) + lighthouse_extend_angle_;
    //cout << "angle_laser_with_lighthouse_one_side_extend: " << angle_laser_with_lighthouse_one_side_extend << endl;
    double lighthouse_vertex_angle_with_axisX = RadianToAngle( atan2( lighthouse_pose2d_in_laser_upright_.y, lighthouse_pose2d_in_laser_upright_.x ) );
    if ( !laser_stand_upright_ )
    {
        lighthouse_vertex_angle_with_axisX = -lighthouse_vertex_angle_with_axisX;
    }
    //cout << "lighthouse_vertex_angle_with_axisX: " << lighthouse_vertex_angle_with_axisX << endl;

    int index_middle_to_lighthouse = middle_laser_scan_point_index + ceil(lighthouse_vertex_angle_with_axisX/Slaser_scan_msg_.angle_resolution);
    int index_rangle = ceil(angle_laser_with_lighthouse_one_side_extend / Slaser_scan_msg_.angle_resolution);
    lighthouse_start_extend_scan_index_ = index_middle_to_lighthouse - index_rangle;
    lighthouse_end_extend_scan_index_ = index_middle_to_lighthouse + index_rangle;
    //cout << "lighthouse_start_extend_scan_index_: " << lighthouse_start_extend_scan_index_ << "  lighthouse_end_extend_scan_index_: " << lighthouse_end_extend_scan_index_ << endl;

    if ( lighthouse_start_extend_scan_index_ > Slaser_scan_msg_.total_point || lighthouse_end_extend_scan_index_ < 0 )
    {
        ROS_WARN ( "lighthouse extend scan index out range" );
        return false;
    }
    if ( lighthouse_start_extend_scan_index_ < 0 )
    {
        lighthouse_start_extend_scan_index_ = 0;
    }
    if ( lighthouse_end_extend_scan_index_ > Slaser_scan_msg_.total_point )
    {
        lighthouse_end_extend_scan_index_ = Slaser_scan_msg_.total_point;
    }

    if ( lighthouse_start_extend_scan_index_ >= lighthouse_end_extend_scan_index_ )
    {
        ROS_WARN ( "lighthouse point is less than zero" );
        return false;
    }
    return true;
}

void LighthouseRecognize::PubPoseQuaternionInvalid()
{
    lighthouse_pose_pub_.publish(pose_quaternion_invalid_);
}

inline void LighthouseRecognize::TransLighthouseScanPointFromPolarToCart(double distance_lighthouse_to_laser)
{
    int lighthouse_point_fit_step = 1;
    int lighthouse_scan_point_num = lighthouse_end_extend_scan_index_ - lighthouse_start_extend_scan_index_ + 1;

    if ( distance_lighthouse_to_laser < 0.4 && RadianToAngle(Slaser_scan_msg_.angle_resolution) < 0.4 )
    {
        lighthouse_point_fit_step = 2;

        if ( 1 == lighthouse_scan_point_num % 2 )
        {
            lighthouse_scan_point_num += 1;
        }
        lighthouse_scan_point_num = lighthouse_scan_point_num / lighthouse_point_fit_step;
    }

    lighthouse_in_laser_upright_coordinate_.resize(2, lighthouse_scan_point_num);

    if ( laser_stand_upright_ )
    {
        for ( int i = 0; i < lighthouse_scan_point_num; ++i )
        {
            double const &range = laser_msg_.ranges[lighthouse_start_extend_scan_index_ + (i*lighthouse_point_fit_step)];
            double q = laser_msg_.angle_min + laser_msg_.angle_increment * static_cast<double>(lighthouse_start_extend_scan_index_ + (i*lighthouse_point_fit_step));
            lighthouse_in_laser_upright_coordinate_(0, i) = range * cos(q);
            lighthouse_in_laser_upright_coordinate_(1, i) = range * sin(q);
        }
    }
    else//雷达倒立，朝agv前方
    {
        for ( int i = 0; i < lighthouse_scan_point_num; ++i )
        {
            double const &range = laser_msg_.ranges[lighthouse_start_extend_scan_index_ + (i*lighthouse_point_fit_step)];
            double q = laser_msg_.angle_max - (laser_msg_.angle_increment * static_cast<double>(lighthouse_start_extend_scan_index_ + (i*lighthouse_point_fit_step)));
            lighthouse_in_laser_upright_coordinate_(0, lighthouse_scan_point_num-i-1) = range * cos(q);
            lighthouse_in_laser_upright_coordinate_(1, lighthouse_scan_point_num-i-1) = range * sin(q);
        }
    }
    // for ( int i = 0; i < lighthouse_scan_point_num; ++i )
    // {
    //     cout << i << ": " << lighthouse_scan_point_in_laser_coordinate_(0, i) << " : " << lighthouse_scan_point_in_laser_coordinate_(1, i) << endl;
    // }
//    TriangleScanPointDisplayPub();
}

void LighthouseRecognize::PubLighthousePoseInAgv()
{
    geometry_msgs::PoseStamped lighthouse_pose_stamped_in_agv_msg;

    lighthouse_pose_stamped_in_agv_msg.header.frame_id = frame_id_agv_;
    lighthouse_pose_stamped_in_agv_msg.header.stamp = ros::Time::now();
    lighthouse_pose_stamped_in_agv_msg.pose.position.x = lighthouse_pose2d_in_laser_upright_.x + laser_in_agv_x_;
    lighthouse_pose_stamped_in_agv_msg.pose.position.y = lighthouse_pose2d_in_laser_upright_.y;
    lighthouse_pose_stamped_in_agv_msg.pose.position.z = 0.0;
    tf::Quaternion q = tf::createQuaternionFromYaw( lighthouse_pose2d_in_laser_upright_.theta );
    lighthouse_pose_stamped_in_agv_msg.pose.orientation.x = q.getX();
    lighthouse_pose_stamped_in_agv_msg.pose.orientation.y = q.getY();
    lighthouse_pose_stamped_in_agv_msg.pose.orientation.z = q.getZ();
    lighthouse_pose_stamped_in_agv_msg.pose.orientation.w = q.getW();

    lighthouse_pose_pub_.publish(lighthouse_pose_stamped_in_agv_msg);
}

bool LighthouseRecognize::LineFitLeastSquaresABC(int index_start, int index_end, double &fit_line_k, double &fit_line_b)
{
    double s_sumX = 0.0;
    double t_sumY = 0.0;
    double u_sumXX = 0.0;
    double v_sumXY = 0.0;
    double w_sumYY = 0.0;

    double point_x = 0.0;
    double point_y = 0.0;

    bool A_is_zero = false;
    bool B_is_zero = false;

    size_t vector_size = index_end - index_start + 1;
    for ( int i = index_start; i < index_end + 1; ++i )
    {
        point_x = lighthouse_in_laser_upright_coordinate_(0, i);
        point_y = lighthouse_in_laser_upright_coordinate_(1, i);
        s_sumX += point_x;
        t_sumY += point_y;
        u_sumXX += point_x * point_x;
        w_sumYY += point_y * point_y;
        v_sumXY += point_x * point_y;
    }

    double temp1 = vector_size * v_sumXY - s_sumX * t_sumY;
    double temp2 = vector_size * (w_sumYY - u_sumXX) + s_sumX * s_sumX - t_sumY * t_sumY;

    if ( temp1 != 0.0 )
    {
        //A_is_zero = false; B_is_zero = false;
        double temp = temp2 / temp1;
        double A_div_B_small = ( -temp - sqrt( temp * temp + 4 ) ) / 2;
        double A_div_B_big = ( -temp + sqrt( temp * temp + 4 ) ) / 2;
        double C_div_B_small = ( -s_sumX * A_div_B_small - t_sumY ) / vector_size;
        double C_div_B_big = ( -s_sumX * A_div_B_big - t_sumY ) / vector_size;
//        cout << "A_div_B_small: " << A_div_B_small << "  C_div_B_small: " << C_div_B_small << endl;
//        cout << "A_div_B_big: " << A_div_B_big << " C_div_B_big: " << C_div_B_big << endl;

        double equation_value_small = 0.0;
        double equation_value_big = 0.0;
        double temp_x = 0.0;
        double temp_y = 0.0;
        for ( int i = index_start; i < index_end + 1; ++i )
        {
            temp_x = lighthouse_in_laser_upright_coordinate_(0, i);
            temp_y = lighthouse_in_laser_upright_coordinate_(1, i);
            equation_value_small += pow( A_div_B_small * temp_x + temp_y + C_div_B_small , 2 ) / ( pow( A_div_B_small, 2 ) + 1 );
            equation_value_big += pow( A_div_B_big * temp_x + temp_y + C_div_B_big , 2 ) / ( pow( A_div_B_big, 2 ) + 1 );
        }
//        cout << "equation_value_small: " << equation_value_small << endl;
//        cout << "equation_value_big: " << equation_value_big << endl;
        if ( equation_value_small < equation_value_big )
        {
            fit_line_k = -A_div_B_small;
            fit_line_b = -C_div_B_small;
        }
        else
        {
            fit_line_k = -A_div_B_big;
            fit_line_b = -C_div_B_big;
        }
//        cout << "fit_line_k: " << fit_line_k << " fit_line_b: " << fit_line_b << endl;
        return true;
    }
    else
    {
        if ( temp2 != 0.0 )
        {
            ROS_WARN( "Maybe never happen A or B is zero" );
            double C_div_B_A0 = -v_sumXY/s_sumX;
            double C_div_A_B0 = -s_sumX/vector_size;
            double equation_value_A0 = 0.0;
            double equation_value_B0 = 0.0;
            double temp_x = 0.0;
            double temp_y = 0.0;
            for ( int i = index_start; i < index_end + 1; ++i )
            {
                temp_x = lighthouse_in_laser_upright_coordinate_(0, i);
                temp_y = lighthouse_in_laser_upright_coordinate_(1, i);
                equation_value_A0 = pow( temp_y, 2 ) + pow( C_div_B_A0, 2 ) + 2 * C_div_B_A0 * temp_y;
                equation_value_B0 = pow( temp_x, 2 ) + pow( C_div_A_B0, 2 ) + 2 * C_div_A_B0 * temp_x;
            }
            if ( equation_value_A0 < equation_value_B0 )
            {
                A_is_zero = true;//A=0,y=const
            }
            else
            {
                B_is_zero = true;//B=0,x=const
            }
        }
        else
        {
            ROS_WARN( "Maybe never happen" );
        }
        return false;
    }
}

double LighthouseRecognize::DistancePointToLine( double line_k, double line_b, double point_x, double point_y )
{
    double distance_point_to_line = fabs(line_k*point_x - point_y + line_b)/sqrt(line_k*line_k + 1);
    return distance_point_to_line;
}

double LighthouseRecognize::TriangleGetNextSeveralPointDistanceToLine( int start, int total_point_num, double line_k, double line_b )
{
    float average_x = 0.0;
    float average_y = 0.0;
    int point_count = 0;
    for(int k = start; k < start + point_num_of_next_several_point_; ++k)
    {
        if ( k < total_point_num )
        {
            ++point_count;
            average_x += lighthouse_in_laser_upright_coordinate_(0, k);
            average_y += lighthouse_in_laser_upright_coordinate_(1, k);
            //cout << "k: " << k << "average_x: " << average_x << " average_y: " << average_y << endl;
        }
    }
    average_x = average_x/point_count;
    average_y = average_y/point_count;

    //cout << "average_x: " << average_x << " average_y: " << average_y << endl;
    float distance_point_to_line = DistancePointToLine(line_k, line_b, average_x, average_y);
    return distance_point_to_line;
}

bool LighthouseRecognize::TriangleLineFitAndCalculatePose2D(double distance_triangle_to_laser)
{
    double fit_line_k;
    double fit_line_b;
    bool get_first_line = false;

    int index_start_fit_point = 0;
    double gap_between_two_point = 1.8 * tan( AngleToRadian(Slaser_scan_msg_.angle_resolution) );
    int total_point_num_of_scan_triangle =  lighthouse_in_laser_upright_coordinate_.cols();
    //cout << "total_point_num_of_scan_triangle: " << total_point_num_of_scan_triangle << endl;
    while ( index_start_fit_point < total_point_num_of_scan_triangle - num_of_point_first_fit_used_ )
    {
        //1.取几个点，作为拟合起始点，存入待拟合vector，计算点之间的xy间隙
        //cout << "first fit: ";
        double gap_x_average = 0.0;
        double gap_y_average = 0.0;
        for ( int i = 0; i < num_of_point_first_fit_used_; ++i )
        {
            //cout << index_start_fit_point + i << ":" << l_msg_.ranges[triangle_start_extend_scan_index_ + index_start_fit_point + i] << "  ";
            gap_x_average += fabs ( lighthouse_in_laser_upright_coordinate_(0, index_start_fit_point + i) - lighthouse_in_laser_upright_coordinate_(0, index_start_fit_point + i + 1) );
            gap_y_average += fabs ( lighthouse_in_laser_upright_coordinate_(1, index_start_fit_point + i) - lighthouse_in_laser_upright_coordinate_(1, index_start_fit_point + i + 1) );
        }
        //cout << endl;

        gap_x_average = gap_x_average / num_of_point_first_fit_used_;
        gap_y_average = gap_y_average / num_of_point_first_fit_used_;
        //1.1判断间隙是否满足
        if ( gap_x_average > gap_between_two_point || gap_y_average > gap_between_two_point )
        {
            //cout << "fit point no good: " << "  gap_x_average: " << gap_x_average << "  gap_y_average: " << gap_y_average << endl;
            index_start_fit_point += num_of_point_first_fit_used_;
            continue;
        }
        //2.拟合几个起始点
        LineFitLeastSquaresABC ( index_start_fit_point, index_start_fit_point + num_of_point_first_fit_used_, fit_line_k, fit_line_b );

        int count_of_point_fit_ok = num_of_point_first_fit_used_;
        int count_of_point_fit_reference = num_of_point_first_fit_used_;
        int count_of_point_distance_far = 0;

        for (int num_of_point_add_to_fit = index_start_fit_point + num_of_point_first_fit_used_; num_of_point_add_to_fit < total_point_num_of_scan_triangle; ++num_of_point_add_to_fit)
        {
            ++count_of_point_fit_reference;
            bool curren_fit_over = false;

            //计算当前点到直线的距离
            float x_next = lighthouse_in_laser_upright_coordinate_(0, num_of_point_add_to_fit);
            float y_next = lighthouse_in_laser_upright_coordinate_(1, num_of_point_add_to_fit);
            //cout << "num_of_point_add_to_fit: " << num_of_point_add_to_fit << " x_next: " << x_next << " y_next: " << y_next << endl;
            float distance_to_line_next_point = DistancePointToLine(fit_line_k, fit_line_b, x_next, y_next);

            //计算后面几个点的平均点到直线的距离
            float distance_to_line_next_several_point = TriangleGetNextSeveralPointDistanceToLine( num_of_point_add_to_fit, total_point_num_of_scan_triangle, fit_line_k, fit_line_b );

            if ((distance_to_line_next_point < max_distance_of_point_to_line_) && (distance_to_line_next_several_point < max_distance_of_point_to_line_) )
            {
                //cout << "count_of_point_fit_ok: " << count_of_point_fit_ok << endl;
                LineFitLeastSquaresABC ( index_start_fit_point, num_of_point_add_to_fit, fit_line_k, fit_line_b );

                count_of_point_fit_ok += count_of_point_distance_far + 1;
                count_of_point_distance_far = 0;
                //cout << "add point: " << num_of_point_add_to_fit << ": " << l_msg_.ranges[triangle_start_extend_scan_index_ + num_of_point_add_to_fit]
                //<< " curren point distance: " << distance_to_line_next_point << " next several point distance: " << distance_to_line_next_several_point << " ok" << endl;
            }
            else
            {
                ++count_of_point_distance_far;
                //拟合的点数小于8时，出现距离远的点则放弃当前直线拟合，重新开始新的直线拟合
                if ( count_of_point_fit_ok < 8 )
                {
                    curren_fit_over = true;
                }
                //cout << "add point: " << num_of_point_add_to_fit << " : " << l_msg_.ranges[triangle_start_extend_scan_index_ + num_of_point_add_to_fit]
                //<< " distance1: " << distance_to_line_next_point << " distance2: " << distance_to_line_next_several_point << " --------to far: " << count_of_point_distance_far << endl;
            }

            if ((count_of_point_distance_far > 5) || curren_fit_over || (num_of_point_add_to_fit == total_point_num_of_scan_triangle-1))
            {
                if (distance_triangle_to_laser > 0.4)
                {
                    min_continuity_point_num_consider_line_ = min_continuity_point_num_consider_line_far_;
                }
                else
                {
                    min_continuity_point_num_consider_line_ = min_continuity_point_num_consider_line_near_;
                }

                if(count_of_point_fit_ok > min_continuity_point_num_consider_line_)
                {
                    //cout << "get a line, num: " << num_of_point_add_to_fit << endl;
                    TriangleFit Striangle_fit;
                    if (get_first_line)
                    {
                        Striangle_fit.index_seconde_end = num_of_point_add_to_fit - count_of_point_distance_far;
                        Striangle_fit.index_seconde_start = Striangle_fit.index_seconde_end - count_of_point_fit_ok + 1;
                        //cout << "-----------get seconde line: " << Striangle_fit.index_first_start << " to " << Striangle_fit.index_first_end
                        //<< "  " << Striangle_fit.index_seconde_start << " to " << Striangle_fit.index_seconde_end
                        //<< "  total point: " << Striangle_fit.index_seconde_end - Striangle_fit.index_first_start
                        //<< "  gap point: " << Striangle_fit.index_seconde_start - Striangle_fit.index_first_end << endl;
                        if ( Striangle_fit.index_seconde_start - Striangle_fit.index_first_end < line_cross_tolerate_discontinuity_point_)
                        {
                            Striangle_fit.line_seconde_k = fit_line_k;
                            Striangle_fit.line_seconde_b = fit_line_b;

                            if ( TriangleCalculatePose2D(Striangle_fit) )
                            {
                                //TriangleFitDisplayPub( Striangle_fit );
                                return true;
                            }
                        }
                        else
                        {
                            ROS_WARN( "two line discontinuity gap point: %d", Striangle_fit.index_seconde_start - Striangle_fit.index_first_end );
                        }
                        Striangle_fit.line_first_k = fit_line_k;
                        Striangle_fit.line_first_b = fit_line_b;
                        Striangle_fit.index_first_end = Striangle_fit.index_seconde_end;
                    }
                    else
                    {
                        Striangle_fit.line_first_k = fit_line_k;
                        Striangle_fit.line_first_b = fit_line_b;
                        Striangle_fit.index_first_end = num_of_point_add_to_fit - count_of_point_distance_far;
                        Striangle_fit.index_first_start = Striangle_fit.index_first_end - count_of_point_fit_ok + 1;
                        //cout << "----------get first line, line1_start: " << Striangle_fit.index_first_start << " line1_end: " << Striangle_fit.index_first_end << endl;
                        get_first_line = true;
                        //TriangleFitDisplayPub( Striangle_fit );
                    }
                }
                else
                {
                    //cout << "not a line, fit ok point:" << count_of_point_fit_ok << endl;
                }
                break;
            }
        }
        index_start_fit_point = index_start_fit_point + count_of_point_fit_reference - count_of_point_distance_far;
    }
    //ROS_WARN( "There is no point" );
    return false;
}

bool LighthouseRecognize::ReflectorCheckDetectedBeltLength(double index_start, double index_end)
{
    double detected_belt_length = DistanceBetweenTwoPoint(lighthouse_in_laser_upright_coordinate_(0, index_start), lighthouse_in_laser_upright_coordinate_(1, index_start),
                                                          lighthouse_in_laser_upright_coordinate_(0, index_end), lighthouse_in_laser_upright_coordinate_(1, index_end));
    if ( fabs(detected_belt_length - 2.0*lighthouse_length_) < max_tolerate_diff_belt_length_detected_ )
    {
        return true;
    }
    else
    {
        ROS_WARN("detected belt length is not in range, detected belt length: %f", detected_belt_length);
        return false;
    }
}

double LighthouseRecognize::DistanceBetweenTwoPoint(double x1, double y1, double x2, double y2)
{
    double distance = sqrt ( pow(x1-x2, 2) + pow(y1-y2, 2) );
    return distance;
}

bool LighthouseRecognize::CheckDetectedLighthousePoseDiff(double current_x, double current_y)
{
    static int count_pose_changed_much = 0;
    static double last_pose_x = 0.0;
    static double last_pose_y = 0.0;

    if ( new_dock_first_recognize_ )
    {
        count_pose_changed_much = 0;
        last_pose_x = current_x;
        last_pose_y = current_y;
        new_dock_first_recognize_ = false;
        return true;
    }
    else
    {
        double distance_between_two_point = DistanceBetweenTwoPoint(last_pose_x, last_pose_y, current_x, current_y);
        double distance_laser_to_triangle = sqrt(pow(lighthouse_pose2d_in_laser_upright_.x, 2) + pow(lighthouse_pose2d_in_laser_upright_.y, 2));

        if ( distance_between_two_point > (0.06 * distance_laser_to_triangle + 0.06) && count_pose_changed_much < 5 )
        {
            ++count_pose_changed_much;
            return false;
        }
        else
        {
            count_pose_changed_much = 0;
            last_pose_x = current_x;
            last_pose_y = current_y;
            return true;
        }
    }
}

bool LighthouseRecognize::TriangleCalculatePose2D(TriangleFit Striangle_fit)
{
    //计算两边的夹角
    double triangle_vertex_current_x = (Striangle_fit.line_seconde_b - Striangle_fit.line_first_b)/(Striangle_fit.line_first_k - Striangle_fit.line_seconde_k);
    double triangle_vertex_current_y = (Striangle_fit.line_first_k* Striangle_fit.line_seconde_b - Striangle_fit.line_seconde_k*Striangle_fit.line_first_b)/( Striangle_fit.line_first_k- Striangle_fit.line_seconde_k);
    // cout << "triangle_vertex_current_x: " << triangle_vertex_current_x << "  triangle_vertex_current_y: " << triangle_vertex_current_y << endl;

    double line_first_X_intercept = - Striangle_fit.line_first_b / Striangle_fit.line_first_k;
    double line_first_y1 = triangle_vertex_current_y - SIGN(line_first_X_intercept) * fabs(Striangle_fit.line_first_k);
    double line_first_x1 = (line_first_y1 - Striangle_fit.line_first_b) / Striangle_fit.line_first_k;
    // cout << "Striangle_fit.line_first_k: " << Striangle_fit.line_first_k << endl;
    // cout << "line_first_X_intercept: " << line_first_X_intercept << endl;
    // cout << "line_first_x1: " << line_first_x1 << "  line_first_y1: " << line_first_y1 << endl;

    double line_seconde_X_intercept = - Striangle_fit.line_seconde_b / Striangle_fit.line_seconde_k;
    double line_seconde_y2 = triangle_vertex_current_y + SIGN(line_seconde_X_intercept) * fabs(Striangle_fit.line_seconde_k);
    double line_seconde_x2 = (line_seconde_y2 - Striangle_fit.line_seconde_b) / Striangle_fit.line_seconde_k;
    // cout << "Striangle_fit.line_seconde_k: " << Striangle_fit.line_seconde_k << endl;
    // cout << "line_seconde_X_intercept: " << line_seconde_X_intercept << endl;
    // cout << "line_seconde_x2: " << line_seconde_x2 << "  line_seconde_y2: " << line_seconde_y2 << endl;

    double line_seconde_to_line_first = atan2(line_seconde_y2-triangle_vertex_current_y, line_seconde_x2-triangle_vertex_current_x) - atan2(triangle_vertex_current_y-line_first_y1, triangle_vertex_current_x-line_first_x1);
    // cout << "seconde: " << RadianToAngle(atan2(line_seconde_y2-triangle_vertex_current_y, line_seconde_x2-triangle_vertex_current_x)) << endl;
    // cout << "first: " << RadianToAngle(atan2(triangle_vertex_current_y-line_first_y1, triangle_vertex_current_x-line_first_x1)) << endl;
    //cout << "line_seconde_to_line_first: " << RadianToAngle(line_seconde_to_line_first) << endl;
    if ( line_seconde_to_line_first <= -M_PI )
    {
        line_seconde_to_line_first += 2.0 * M_PI;
    }
    else if ( line_seconde_to_line_first > M_PI )
    {
        line_seconde_to_line_first -= 2.0 * M_PI;
    }
//    cout << "line_seconde_to_line_first: " << RadianToAngle(line_seconde_to_line_first) << endl;

    double angle_diff_detected_with_real = RadianToAngle(line_seconde_to_line_first) - (180 - triangle_angle_included_);
    if ( fabs(angle_diff_detected_with_real) < line_cross_tolerate_offset_angle_ )
    {
        //first_line上的两点绕agv坐标系的原点顺时针旋转60度即与三角板中线方向相同
        double theat = -(M_PI - line_seconde_to_line_first) / 2.0;
        double sin_theat = sin(theat);
        double cos_theat = cos(theat);

        double triangle_axisX_x1 = line_first_x1 * cos_theat - line_first_y1 * sin_theat;
        double triangle_axisX_x2 = triangle_vertex_current_x * cos_theat - triangle_vertex_current_y * sin_theat;
        double triangle_axisX_y1 = line_first_x1 * sin_theat + line_first_y1 * cos_theat;
        double triangle_axisX_y2 = triangle_vertex_current_x * sin_theat + triangle_vertex_current_y * cos_theat;

        //计算三角角平分线与雷达坐标系x轴的夹角
        double triangle_axisX_with_laser_axisX = atan2(triangle_axisX_y2 - triangle_axisX_y1, triangle_axisX_x2 - triangle_axisX_x1);
        //cout << "triangle_axisX_with_laser_axisX: " << RadianToAngle(triangle_axisX_with_laser_axisX) << endl;

        if ( CheckDetectedLighthousePoseDiff(triangle_vertex_current_x, triangle_vertex_current_y) )
        {
            lighthouse_pose2d_in_laser_upright_.x = triangle_vertex_current_x;
            lighthouse_pose2d_in_laser_upright_.y = triangle_vertex_current_y;
            lighthouse_pose2d_in_laser_upright_.theta = triangle_axisX_with_laser_axisX;
            return true;
        }
        else
        {
            ROS_WARN("triangle current pose change too much compare to last pose");
            return false;
        }
    }
    else
    {
        ROS_WARN( "angle detected out range, line seconde to line first is: %f", RadianToAngle(line_seconde_to_line_first) );
        return false;
    }
}

double LighthouseRecognize::ReflectorGetBeltXOrYInLaserCoordinate(int X_or_Y, int belt_index_start, int belt_index_end)
{
    int num_point = belt_index_end - belt_index_start + 1;

    double sum_point = 0.0;
    for ( int i = 0; i < num_point; ++i )
    {
        sum_point += lighthouse_in_laser_upright_coordinate_(X_or_Y, i);
    }

    return sum_point / num_point;
}

double LighthouseRecognize::ReflectorGetValueXByEquation( double belt_line_k, double belt_line_b )
{
    double value_x = ( lighthouse_pose2d_in_laser_upright_.y - belt_line_b ) / belt_line_k;
    return value_x;
}

double LighthouseRecognize::ReflectorGetBeltVerticalLineAngleWithLaserAxisX(double belt_fit_line_k, double belt_fit_line_b)
{
    double AxisY_x1 = 0.0;
    double AxisY_y1 = 0.0;
    double AxisY_x2 = 0.0;
    double AxisY_y2 = 0.0;

    double AxisY_X_intercept = - belt_fit_line_b / belt_fit_line_k;
    AxisY_x1 = lighthouse_pose2d_in_laser_upright_.x;
    AxisY_y1 = lighthouse_pose2d_in_laser_upright_.y;
    AxisY_y2 = AxisY_y1 + SIGN(AxisY_X_intercept) * fabs(belt_fit_line_k);
    AxisY_x2 = (AxisY_y2 - belt_fit_line_b) / belt_fit_line_k;

    double theat = -0.5 * M_PI;
    double sin_theat = sin(theat);
    double cos_theat = cos(theat);

    double AxisX_x1 = AxisY_x1 * cos_theat - AxisY_y1 * sin_theat;
    double AxisX_x2 = AxisY_x2 * cos_theat - AxisY_y2 * sin_theat;
    double AxisX_y1 = AxisY_x1 * sin_theat + AxisY_y1 * cos_theat;
    double AxisX_y2 = AxisY_x2 * sin_theat + AxisY_y2 * cos_theat;

    double belt_vertical_line_radian_with_laser_axisX = atan2( AxisX_y2 - AxisX_y1, AxisX_x2 - AxisX_x1 );
//    cout << "belt_vertical_line_radian_with_laser_axisX: " << RadianToAngle( belt_vertical_line_radian_with_laser_axisX ) << endl;
    return belt_vertical_line_radian_with_laser_axisX;
}

bool LighthouseRecognize::ReflectorLineFitAndCalculatePose2D(double distance_reflector_to_laser)
{
    int belt_scan_index_start = 0;
    int belt_scan_index_end = 0;
    bool get_belt_scan_index_start = false;
    bool get_belt_scan_index_end = false;

    int count_continue_reflector_point = 0;

    for ( int i = lighthouse_start_extend_scan_index_; i < lighthouse_end_extend_scan_index_; ++i )
    {
        if ( laser_msg_.intensities[i] > min_intensite_consider_reflector_point_ )
        {
//            cout << "i: " << i << "  intensities: " << laser_msg_.intensities[i] << "  ranges: " << laser_msg_.ranges[i] << endl;
            ++count_continue_reflector_point;
            if ( !get_belt_scan_index_start )
            {
                belt_scan_index_start = i;
                get_belt_scan_index_start = true;
               //cout << "start befor: " << i-1 << "  intensities: " << laser_msg_.intensities[i-1] << "  ranges: " << laser_msg_.ranges[i-1] << endl;
            }

            if ( i == lighthouse_end_extend_scan_index_-1 )
            {
                belt_scan_index_end = i - 1;
                get_belt_scan_index_end = true;
                count_continue_reflector_point = 0;
            }
        }
        else
        {
            if ( get_belt_scan_index_start && count_continue_reflector_point > min_point_num_consider_belt_ )
            {
                belt_scan_index_end = i - 1;
                get_belt_scan_index_end = true;
               //cout << "end next: " << i << "  intensities: " << laser_msg_.intensities[i] << "  ranges: " << laser_msg_.ranges[i] << endl;
               //cout << "index_start: " << belt_scan_index_start << "  index_end: " << belt_scan_index_end << "  total point: " << belt_scan_index_end - belt_scan_index_start + 1 << endl;
            }
            else
            {
                get_belt_scan_index_start = false;
            }
            count_continue_reflector_point = 0;
        }

        if ( get_belt_scan_index_start && get_belt_scan_index_end )
        {
            lighthouse_start_extend_scan_index_ = belt_scan_index_start;
            lighthouse_end_extend_scan_index_ = belt_scan_index_end;

            double fit_line_k = 0.0;
            double fit_line_b = 0.0;

            TransLighthouseScanPointFromPolarToCart(distance_reflector_to_laser);

            int index_start = 0;
            int index_end = lighthouse_in_laser_upright_coordinate_.cols() - 1;

            bool check_detected_length = ReflectorCheckDetectedBeltLength(index_start, index_end);
            if ( !check_detected_length )
            {
                continue;
            }

            bool fit_ok = LineFitLeastSquaresABC(index_start, index_end, fit_line_k, fit_line_b);

            // TriangleFit striangle_fit;
            // striangle_fit.line_first_b = fit_line_b;
            // striangle_fit.line_first_k = fit_line_k;
            // LighthouseFitDisplayPub(striangle_fit);

            double current_x = 0.0;
            double current_y = 0.0;
            current_x = ReflectorGetBeltXOrYInLaserCoordinate(0, index_start, index_end );
            current_y = ReflectorGetBeltXOrYInLaserCoordinate(1, index_start, index_end );
            // if ( fit_ok && fabs(fit_line_k) > 0.5 )
            // {
            //     current_x = ReflectorGetValueXByEquation( fit_line_k, fit_line_b );
            // }
            // else
            // {
            //     current_x = ReflectorGetBeltXOrYInLaserCoordinate(0, index_start, index_end );
            // }

            if ( CheckDetectedLighthousePoseDiff(current_x, current_y) )
            {
                lighthouse_pose2d_in_laser_upright_.x = current_x;
                lighthouse_pose2d_in_laser_upright_.y = current_y;
                lighthouse_pose2d_in_laser_upright_.theta = ReflectorGetBeltVerticalLineAngleWithLaserAxisX(fit_line_k, fit_line_b);
//                printf ( "-------get belt pose ok----------x: %7.5f y: %7.5f angle %7.5f \n", lighthouse_pose2d_in_laser_upright_.x,
//                         lighthouse_pose2d_in_laser_upright_.y, RadianToAngle(lighthouse_pose2d_in_laser_upright_.theta) );
                return true;
            }
            else
            {
                ROS_WARN("triangle current pose change too much compare to last pose");
            }

            ROS_WARN( "--------------------------------not real belt-------------------------------");
            get_belt_scan_index_start = false;
            get_belt_scan_index_end = false;
        }
    }
    ROS_WARN( "--------------------------------get belt pose fail-------------------------------");
    return false;
}

void LighthouseRecognize::Run()
{
    GetParam();
    InitPoseQuaternionInvalid();

    int count_get_lighthouse_failed = 0;

    ros::Rate r(10);

    while( ros::ok() )
    {
        ros::spinOnce();

        status_is_run_ = get_new_lighthouse_pose_in_map_ && get_new_lighthouse_param_;

        if ( !status_is_run_ )
        {
            r.sleep();
            continue;
        }

        if ( !get_new_laser_scan_ )
        {
            r.sleep();
            continue;
        }
        get_new_laser_scan_ = false;

        if ( need_trans_lighthouse_pose_from_map_to_laser_ )
        {
            while ( !TransLighthousePoseFromMapToLaserUpright() )
            {
                ros::Duration(0.1).sleep();
            }

            need_trans_lighthouse_pose_from_map_to_laser_ = false;
        }

        double distance_lighthouse_to_laser = sqrt(pow(lighthouse_pose2d_in_laser_upright_.x, 2) + pow(lighthouse_pose2d_in_laser_upright_.y, 2));
        bool get_lighthouse_pose2D = true;

        if ( lighthouse_type_ == "triangle" )
        {
            CheckLighthouseDetectCondition(distance_lighthouse_to_laser, max_detecte_distance_triangle_to_laser_, max_detecte_angle_triangle_to_laser_);

            if ( !CalculateLighthouseExtendScanIndexFromLastPose(distance_lighthouse_to_laser, AngleToRadian(0.5*triangle_angle_included_)) )
            {
                PubPoseQuaternionInvalid();
                need_trans_lighthouse_pose_from_map_to_laser_ = true;
                r.sleep();
                continue;
            }

            TransLighthouseScanPointFromPolarToCart(distance_lighthouse_to_laser);

            get_lighthouse_pose2D = TriangleLineFitAndCalculatePose2D(distance_lighthouse_to_laser);
        }
        else if ( lighthouse_type_ == "reflector" )
        {
            CheckLighthouseDetectCondition(distance_lighthouse_to_laser, max_detecte_distance_belt_to_laser_, max_detecte_angle_belt_to_laser_);

            if ( !CalculateLighthouseExtendScanIndexFromLastPose(distance_lighthouse_to_laser, 0.5*M_PI) )
            {
                PubPoseQuaternionInvalid();
                need_trans_lighthouse_pose_from_map_to_laser_ = true;
                r.sleep();
                continue;
            }

            get_lighthouse_pose2D = ReflectorLineFitAndCalculatePose2D(distance_lighthouse_to_laser);
        }
        else
        {
            ROS_ERROR("Undefined lighthouse type: %s", lighthouse_type_.c_str());
            continue;
        }

        if ( get_lighthouse_pose2D )
        {
            count_get_lighthouse_failed = 0;
            PubLighthousePoseInAgv();
        }
        else
        {
            PubPoseQuaternionInvalid();
            ++count_get_lighthouse_failed;
            if ( count_get_lighthouse_failed % 10 == 0 )
            {
                ROS_WARN ( "get triangle pose failed times ********************************: %d", count_get_lighthouse_failed);
                need_trans_lighthouse_pose_from_map_to_laser_ = true;
            }
        }

        r.sleep();
    }
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "lighthouse_recognize");
    LighthouseRecognize lighthouse_recognize;
    lighthouse_recognize.Run();
    return 0;
};
