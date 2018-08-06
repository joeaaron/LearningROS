#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include "reflector_recognize.h"

using namespace std;

#define RadianToAngle(X) ((X)*180.0/M_PI)
#define AngleToRadian(X) ((X)*M_PI/180.0)
#define SIGN(n) (n <= 0?( n < 0 ?-1:0):1)

ReflectorRecognize::ReflectorRecognize()
    :nh_(),
    laser_scan_sub_(),
    reflector_pose_in_map_sub_(),
    reflector_pose_pub_(),
    belt_fit_display_pub_(),
    laser_msg_(),
    laser_scan_point_to_coordinate_(),
    frame_id_map_("map"),
    frame_id_robot_("base_footprint"),
    get_new_dynamic_param_(false),
    get_new_laser_scan_(false),
    status_is_run_(false),
    get_new_reflector_pose_in_map_(false),
    need_trans_reflector_pose_from_map_to_laser_(true),
    laser_stand_upright_(true),
    new_dock_first_recognize_(true),
    reflector_start_extend_scan_index_(),
    reflector_end_extend_scan_index_(),
    pose_quaternion_invalid_(),
    reflector_pose_stamped_in_map_(),
    reflector_pose2d_in_laser_upright_(),
    Slaser_scan_msg_()
{
    belt_fit_display_pub_ = nh_.advertise<std_msgs::Float32MultiArray> ( "/lighthouse_fit_display", 1 );
    reflector_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped> ( "/lighthouse_pose", 1 );

    laser_scan_sub_ = nh_.subscribe("/scan_filtered_lighthouse", 1, &ReflectorRecognize::LaserScanCallback, this );
    reflector_pose_in_map_sub_ = nh_.subscribe("/lighthouse_pose_in_map", 1, &ReflectorRecognize::ReflectorPoseInMapCallback, this );

    dsrv_ = new dynamic_reconfigure::Server<lighthouse_navigation::LighthouseRecognizeConfig>(ros::NodeHandle("~"));
    dynamic_reconfigure::Server<lighthouse_navigation::LighthouseRecognizeConfig>::CallbackType cb = boost::bind(&ReflectorRecognize::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
}

ReflectorRecognize::~ReflectorRecognize() {}

void ReflectorRecognize::GetParam()
{
    ros::NodeHandle nh_private("~");
    nh_private.param("belt_scan_index_extend_angle", belt_scan_index_extend_angle_, 5);
    nh_private.param("min_intensite_consider_reflector_point", min_intensite_consider_reflector_point_, 1000);
    nh_private.param("min_point_num_consider_belt", min_point_num_consider_belt_, 7);
    nh_private.param("max_detecte_angle_belt_to_laser", max_detecte_angle_belt_to_laser_, 55.0);
    nh_private.param("max_detecte_distance_belt_to_laser", max_detecte_distance_belt_to_laser_, 7.0);
    nh_private.param("max_tolerate_diff_distance_belt_to_laser", max_tolerate_diff_distance_belt_to_laser_, 0.1);
    nh_private.param("max_tolerate_diff_belt_length_detected", max_tolerate_diff_belt_length_detected_, 0.1);

    nh_private.param("/tf_base_laser/roll",laser_roll_, 0.0);
    nh_private.param("/tf_base_laser/x",laser_in_agv_x_, 0.0);

    if ( fabs(laser_roll_ - M_PI) < 0.02 )
    {
        laser_stand_upright_ = false;
    }
}

void ReflectorRecognize::reconfigureCB(lighthouse_navigation::LighthouseRecognizeConfig config, uint32_t level)
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
//            lighthouse_type_ = splited_str[0];
            belt_half_length_ = boost::lexical_cast<double>(splited_str[1]);

            if ( get_reconfigure )
            {
                get_new_dynamic_param_ = true;
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

void ReflectorRecognize::InitPoseQuaternionInvalid()
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

void ReflectorRecognize::LaserScanCallback(const sensor_msgs::LaserScanPtr& laser_scan_msg)
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

void ReflectorRecognize::ReflectorPoseInMapCallback(const geometry_msgs::PosePtr &reflector_pose_in_map_msg)
{
    try
    {
        tf::assertQuaternionValid(reflector_pose_in_map_msg->orientation);
    }
    catch (tf2::InvalidArgumentException &e)//停止识别，进入空闲等待状态
    {
        ROS_ERROR( "assertQuaternionValid: %s", e.what() );//Quaternion malformed
        TurnToIdle();
        return;
    }

    reflector_pose_stamped_in_map_.pose = *reflector_pose_in_map_msg;
    get_new_reflector_pose_in_map_ = true;
    need_trans_reflector_pose_from_map_to_laser_ = true;

    return;
}

void ReflectorRecognize::TurnToIdle()
{
    get_new_dynamic_param_ = false;
    get_new_reflector_pose_in_map_ = false;

    new_dock_first_recognize_ = true;
    get_new_laser_scan_ = false;
}

//三角在正立的雷达坐标系下的坐标
bool ReflectorRecognize::TransReflectorPoseFromMapToLaserUpright()
{
    reflector_pose_stamped_in_map_.header.frame_id = frame_id_map_;
    reflector_pose_stamped_in_map_.header.stamp = ros::Time();

    static tf::TransformListener tf_listener;
    geometry_msgs::PoseStamped triangle_pose_stamped_in_agv;
    tf_listener.waitForTransform(frame_id_robot_, frame_id_map_, ros::Time(),ros::Duration(1.0));
    try
    {
        tf_listener.transformPose(frame_id_robot_, reflector_pose_stamped_in_map_, triangle_pose_stamped_in_agv);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("trans pose from map to agv error: %s",ex.what());
        return false;
    }

    reflector_pose2d_in_laser_upright_.x = triangle_pose_stamped_in_agv.pose.position.x - laser_in_agv_x_;
    reflector_pose2d_in_laser_upright_.y = triangle_pose_stamped_in_agv.pose.position.y;
    reflector_pose2d_in_laser_upright_.theta = tf::getYaw(triangle_pose_stamped_in_agv.pose.orientation);
//    cout << "reflector_pose2d_in_laser_upright_.x: " << reflector_pose2d_in_laser_upright_.x << endl;
//    cout << "reflector_pose2d_in_laser_upright_.y: " << reflector_pose2d_in_laser_upright_.y << endl;
//    cout << "reflector_pose2d_in_laser_upright_.theta1: " << RadianToAngle(reflector_pose2d_in_laser_upright_.theta) << endl;

    return true;
}

void ReflectorRecognize::TransReflectorScanPointFromPolarToCart(double distance_reflector_to_laser)
{
    int reflector_point_fit_step = 1;
    int reflector_scan_point_num = reflector_end_extend_scan_index_ - reflector_start_extend_scan_index_ + 1;

    if ( distance_reflector_to_laser < 0.4 && RadianToAngle(Slaser_scan_msg_.angle_resolution) < 0.4 )
    {
        reflector_point_fit_step = 2;

        if ( 1 == reflector_scan_point_num % 2 )
        {
            reflector_scan_point_num += 1;
        }
        reflector_scan_point_num = reflector_scan_point_num / reflector_point_fit_step;
    }

    laser_scan_point_to_coordinate_.resize(2, reflector_scan_point_num);

    if ( laser_stand_upright_ )
    {
        for ( int i = 0; i < reflector_scan_point_num; ++i )
        {
            double const &range = laser_msg_.ranges[reflector_start_extend_scan_index_ + (i*reflector_point_fit_step)];
            double q = laser_msg_.angle_min + laser_msg_.angle_increment * static_cast<double>(reflector_start_extend_scan_index_ + (i*reflector_point_fit_step));
            laser_scan_point_to_coordinate_(0, i) = range * cos(q);
            laser_scan_point_to_coordinate_(1, i) = range * sin(q);
        }
    }
    else//雷达倒立，朝agv前方
    {
        for ( int i = 0; i < reflector_scan_point_num; ++i )
        {
            double const &range = laser_msg_.ranges[reflector_start_extend_scan_index_ + (i*reflector_point_fit_step)];
            double q = laser_msg_.angle_max - (laser_msg_.angle_increment * static_cast<double>(reflector_start_extend_scan_index_ + (i*reflector_point_fit_step)));
            laser_scan_point_to_coordinate_(0, reflector_scan_point_num-i-1) = range * cos(q);
            laser_scan_point_to_coordinate_(1, reflector_scan_point_num-i-1) = range * sin(q);
        }
    }
    // for ( int i = 0; i < reflector_scan_point_num; ++i )
    // {
    //     cout << i << ": " << laser_scan_point_to_coordinate_(0, i) << " : " << laser_scan_point_to_coordinate_(1, i) << endl;
    // }
//    TriangleScanPointDisplayPub();
}

double ReflectorRecognize::CalculateSidecBySideaSidebAnglec(double side_a, double side_b, double angle_c)
{
    double side_c = sqrt( pow(side_a,2) + pow(side_b,2) - 2.0 * side_a * side_b * cos(angle_c) );
    return side_c;
}

double ReflectorRecognize::CalculateAngleaBySideaSidebAnglec(double side_a, double side_b, double angle_c)
{
    double side_c = CalculateSidecBySideaSidebAnglec(side_a, side_b, angle_c);
    double angle_a = acos( ( pow(side_b,2) + pow(side_c,2) - pow(side_a,2) ) / ( 2.0 * side_b * side_c ) );
    return angle_a;
}

bool ReflectorRecognize::CheckDetectedBeltLength(double index_start, double index_end)
{
    double detected_belt_length = DistanceBetweenTwoPoint(laser_scan_point_to_coordinate_(0, index_start), laser_scan_point_to_coordinate_(1, index_start),
                                                          laser_scan_point_to_coordinate_(0, index_end), laser_scan_point_to_coordinate_(1, index_end));
    if ( fabs(detected_belt_length - 2.0*belt_half_length_) < max_tolerate_diff_belt_length_detected_ )
    {
        return true;
    }
    else
    {
        ROS_WARN("detected belt length is not in range, detected belt length: %f", detected_belt_length);
        return false;
    }
}

double ReflectorRecognize::DistanceBetweenTwoPoint(double x1, double y1, double x2, double y2)
{
    double distance = sqrt ( pow(x1-x2, 2) + pow(y1-y2, 2) );
    return distance;
}

bool ReflectorRecognize::CalculateReflectorExtendScanIndexFromLastPose(double distance_reflector_to_laser)
{
    int middle_laser_scan_point_index = ( Slaser_scan_msg_.total_point + 1 )/2;

    float radian_laser_with_reflector_one_side = CalculateAngleaBySideaSidebAnglec(belt_half_length_, distance_reflector_to_laser, 0.5*M_PI);
    float angle_laser_with_reflector_one_side_extend = RadianToAngle(radian_laser_with_reflector_one_side) + belt_scan_index_extend_angle_;
    //cout << "angle_laser_with_reflector_one_side_extend: " << angle_laser_with_reflector_one_side_extend << endl;
    double reflector_vertex_angle_with_axisX = RadianToAngle( atan2( reflector_pose2d_in_laser_upright_.y, reflector_pose2d_in_laser_upright_.x ) );
    if ( !laser_stand_upright_ )
    {
        reflector_vertex_angle_with_axisX = -reflector_vertex_angle_with_axisX;
    }
//    cout << "reflector_vertex_angle_with_axisX: " << reflector_vertex_angle_with_axisX << endl;

    int index_middle_to_reflector = middle_laser_scan_point_index + ceil(reflector_vertex_angle_with_axisX/Slaser_scan_msg_.angle_resolution);
    int index_rangle = ceil(angle_laser_with_reflector_one_side_extend / Slaser_scan_msg_.angle_resolution);
    reflector_start_extend_scan_index_ = index_middle_to_reflector - index_rangle;
    reflector_end_extend_scan_index_ = index_middle_to_reflector + index_rangle;
//    cout << "reflector_start_extend_scan_index_: " << reflector_start_extend_scan_index_ << "  reflector_end_extend_scan_index_: " << reflector_end_extend_scan_index_ << endl;

    if ( reflector_start_extend_scan_index_ > Slaser_scan_msg_.total_point || reflector_end_extend_scan_index_ < 0 )
    {
        ROS_WARN ( "reflector extend scan index out range" );
        return false;
    }
    if ( reflector_start_extend_scan_index_ < 0 )
    {
        reflector_start_extend_scan_index_ = 0;
    }
    if ( reflector_end_extend_scan_index_ > Slaser_scan_msg_.total_point )
    {
        reflector_end_extend_scan_index_ = Slaser_scan_msg_.total_point;
    }

    if ( reflector_start_extend_scan_index_ >= reflector_end_extend_scan_index_ )
    {
        ROS_WARN ( "reflector point is less than zero" );
        return false;
    }
    return true;
}

void ReflectorRecognize::CheckReflectorDetectCondition(double distance_triangle_to_laser)
{
    if ( distance_triangle_to_laser > max_detecte_distance_belt_to_laser_ )
    {
        ROS_WARN( "distance of triangle to laser is out of dock range %f", distance_triangle_to_laser );
    }

    if ( fabs( RadianToAngle(reflector_pose2d_in_laser_upright_.theta) ) > max_detecte_angle_belt_to_laser_ )
    {
        ROS_WARN("triangle middle line angle with axisX out of range %f",RadianToAngle(reflector_pose2d_in_laser_upright_.theta));
    }
}

bool ReflectorRecognize::CheckDetectedLighthousePoseDiff(double current_x, double current_y)
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
        double distance_laser_to_triangle = sqrt(pow(reflector_pose2d_in_laser_upright_.x, 2) + pow(reflector_pose2d_in_laser_upright_.y, 2));

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

void ReflectorRecognize::BeltFitDisplayPub( double belt_fit_line_k, double belt_fit_line_b )
{
    double belt_vertical_line_a = tan( atan(belt_fit_line_k) - 0.5*M_PI );
    double belt_vertical_line_b = reflector_pose2d_in_laser_upright_.y - belt_vertical_line_a * reflector_pose2d_in_laser_upright_.x;
    std_msgs::Float32MultiArray belt_points;
    for ( int i = 0; i < laser_scan_point_to_coordinate_.cols(); ++i )
    {
        belt_points.data.push_back(laser_scan_point_to_coordinate_(0, i));
        belt_points.data.push_back(laser_scan_point_to_coordinate_(1, i));
    }
    belt_points.data.push_back(belt_fit_line_k);
    belt_points.data.push_back(belt_fit_line_b);
    belt_points.data.push_back(belt_vertical_line_a);
    belt_points.data.push_back(belt_vertical_line_b);
    belt_fit_display_pub_.publish(belt_points);
}

void ReflectorRecognize::PubPoseQuaternionInvalid()
{
    reflector_pose_pub_.publish(pose_quaternion_invalid_);
}

double ReflectorRecognize::GetBeltVerticalLineAngleWithLaserAxisX(double belt_fit_line_K, double belt_fit_line_B)
{
    double AxisY_x1 = 0.0;
    double AxisY_y1 = 0.0;
    double AxisY_x2 = 0.0;
    double AxisY_y2 = 0.0;

    double AxisY_X_intercept = - belt_fit_line_B / belt_fit_line_K;
    AxisY_x1 = reflector_pose2d_in_laser_upright_.x;
    AxisY_y1 = reflector_pose2d_in_laser_upright_.y;
    AxisY_y2 = AxisY_y1 + SIGN(AxisY_X_intercept) * fabs(belt_fit_line_K);
    AxisY_x2 = (AxisY_y2 - belt_fit_line_B) / belt_fit_line_K;

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

double ReflectorRecognize::GetValueXByEquation( double belt_line_k, double belt_line_b )
{
    double value_x = ( reflector_pose2d_in_laser_upright_.y - belt_line_b ) / belt_line_k;
    return value_x;
}

bool ReflectorRecognize::ReflectorLineFitAndCalculatePose2D(double distance_reflector_to_laser)
{
    int belt_scan_index_start = 0;
    int belt_scan_index_end = 0;
    bool get_belt_scan_index_start = false;
    bool get_belt_scan_index_end = false;

    int count_continue_reflector_point = 0;

    for ( int i = reflector_start_extend_scan_index_; i < reflector_end_extend_scan_index_; ++i )
    {
        if ( laser_msg_.intensities[i] > min_intensite_consider_reflector_point_ )
        {
//            cout << "i: " << i << "  intensities: " << laser_msg_.intensities[i] << "  ranges: " << laser_msg_.ranges[i] << endl;
            ++count_continue_reflector_point;
            if ( !get_belt_scan_index_start )
            {
                belt_scan_index_start = i;
                get_belt_scan_index_start = true;
//                cout << "start befor: " << i-1 << "  intensities: " << laser_msg_.intensities[i-1] << "  ranges: " << laser_msg_.ranges[i-1] << endl;
            }
            if ( i == reflector_end_extend_scan_index_-1 )
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
//                cout << "end next: " << i << "  intensities: " << laser_msg_.intensities[i] << "  ranges: " << laser_msg_.ranges[i] << endl;
//                cout << "index_start: " << belt_scan_index_start << "  index_end: " << belt_scan_index_end << "  total point: " << belt_scan_index_end - belt_scan_index_start + 1 << endl;
            }
            else
            {
                get_belt_scan_index_start = false;
            }
            count_continue_reflector_point = 0;
        }

        if ( get_belt_scan_index_start && get_belt_scan_index_end )
        {
            reflector_start_extend_scan_index_ = belt_scan_index_start;
            reflector_end_extend_scan_index_ = belt_scan_index_end;

            double fit_line_k = 0.0;
            double fit_line_b = 0.0;

            TransReflectorScanPointFromPolarToCart(distance_reflector_to_laser);

            int index_start = 0;
            int index_end = laser_scan_point_to_coordinate_.cols() - 1;

            bool fit_ok = LineFitLeastSquaresABC(index_start, index_end, fit_line_k, fit_line_b);

            bool check_detected_length = CheckDetectedBeltLength(index_start, index_end);
            if ( check_detected_length )
            {
                double current_x = 0.0;
                double current_y = 0.0;
                current_y = GetBeltXOrYInLaserCoordinate(1, index_start, index_end );
                if ( fit_ok )
                {
                    current_x = GetValueXByEquation( fit_line_k, fit_line_b );
                }
                else
                {
                    current_x = GetBeltXOrYInLaserCoordinate(0, index_start, index_end );
                }

                if ( CheckDetectedLighthousePoseDiff(current_x, current_y) )
                {
                    reflector_pose2d_in_laser_upright_.x = current_x;
                    reflector_pose2d_in_laser_upright_.y = current_y;
                    reflector_pose2d_in_laser_upright_.theta = GetBeltVerticalLineAngleWithLaserAxisX(fit_line_k, fit_line_b);
//                    printf ( "-------get belt pose ok----------x: %7.5f y: %7.5f angle %7.5f \n", reflector_pose2d_in_laser_upright_.x,
//                             reflector_pose2d_in_laser_upright_.y, RadianToAngle(reflector_pose2d_in_laser_upright_.theta) );
                    return true;
                }
                else
                {
                    ROS_WARN("triangle current pose change too much compare to last pose");
                }
            }

            ROS_WARN( "--------------------------------not real belt-------------------------------");
            get_belt_scan_index_start = false;
            get_belt_scan_index_end = false;
        }
    }
    ROS_WARN( "--------------------------------get belt pose fail-------------------------------");
    return false;
}

double ReflectorRecognize::GetBeltXOrYInLaserCoordinate(int X_or_Y, int belt_index_start, int belt_index_end)
{
    int num_point = belt_index_end - belt_index_start + 1;

    double sum_point = 0.0;
    for ( int i = 0; i < num_point; ++i )
    {
        sum_point += laser_scan_point_to_coordinate_(X_or_Y, i);
    }

    return sum_point / num_point;
}

bool ReflectorRecognize::LineFitLeastSquaresABC(int index_start, int index_end, double &fit_line_k, double &fit_line_b)
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
        point_x = laser_scan_point_to_coordinate_(0, i);
        point_y = laser_scan_point_to_coordinate_(1, i);
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
            temp_x = laser_scan_point_to_coordinate_(0, i);
            temp_y = laser_scan_point_to_coordinate_(1, i);
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
                temp_x = laser_scan_point_to_coordinate_(0, i);
                temp_y = laser_scan_point_to_coordinate_(1, i);
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

void ReflectorRecognize::PubReflectorPoseInAgv()
{
    geometry_msgs::PoseStamped reflector_pose_stamped_in_agv_msg;

    reflector_pose_stamped_in_agv_msg.header.frame_id = frame_id_robot_;
    reflector_pose_stamped_in_agv_msg.header.stamp = ros::Time::now();
    reflector_pose_stamped_in_agv_msg.pose.position.x = reflector_pose2d_in_laser_upright_.x + laser_in_agv_x_;
    reflector_pose_stamped_in_agv_msg.pose.position.y = reflector_pose2d_in_laser_upright_.y;
    reflector_pose_stamped_in_agv_msg.pose.position.z = 0.0;
    tf::Quaternion q = tf::createQuaternionFromYaw( reflector_pose2d_in_laser_upright_.theta );
    reflector_pose_stamped_in_agv_msg.pose.orientation.x = q.getX();
    reflector_pose_stamped_in_agv_msg.pose.orientation.y = q.getY();
    reflector_pose_stamped_in_agv_msg.pose.orientation.z = q.getZ();
    reflector_pose_stamped_in_agv_msg.pose.orientation.w = q.getW();

    reflector_pose_pub_.publish(reflector_pose_stamped_in_agv_msg);
}

void ReflectorRecognize::Run()
{
    GetParam();
    InitPoseQuaternionInvalid();

    int count_get_reflector_failed = 0;

    ros::Rate r(10);

    while ( ros::ok() )
    {
        ros::spinOnce();

        status_is_run_ = get_new_reflector_pose_in_map_ && get_new_dynamic_param_;

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

        if ( need_trans_reflector_pose_from_map_to_laser_ )
        {
            while ( !TransReflectorPoseFromMapToLaserUpright() )
            {
                ros::Duration(0.1).sleep();
            }

            need_trans_reflector_pose_from_map_to_laser_ = false;
        }

        double distance_reflector_to_laser = sqrt(pow(reflector_pose2d_in_laser_upright_.x, 2) + pow(reflector_pose2d_in_laser_upright_.y, 2));

        CheckReflectorDetectCondition(distance_reflector_to_laser);

        if ( !CalculateReflectorExtendScanIndexFromLastPose(distance_reflector_to_laser) )
        {
            PubPoseQuaternionInvalid();
            need_trans_reflector_pose_from_map_to_laser_ = true;
            r.sleep();
            continue;
        }

        if ( ReflectorLineFitAndCalculatePose2D(distance_reflector_to_laser) )
        {
            count_get_reflector_failed = 0;
            PubReflectorPoseInAgv();
        }
        else
        {
            PubPoseQuaternionInvalid();
            ++count_get_reflector_failed;
            if ( count_get_reflector_failed % 10 == 0 )
            {
                ROS_WARN ( "get triangle pose failed times ********************************: %d", count_get_reflector_failed);
                need_trans_reflector_pose_from_map_to_laser_ = true;
            }
        }

        r.sleep();
    }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "reflector_recognize");
  ReflectorRecognize reflector_recognize;
  reflector_recognize.Run();
  return 0;
};
