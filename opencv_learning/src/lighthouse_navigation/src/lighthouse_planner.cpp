#include <ros/package.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

#include "lighthouse_planner.h"

using namespace std;

#define RadianToAngle(X) ((X)*180.0/M_PI)
#define AngleToRadian(X) ((X)*M_PI/180.0)
#define SIGN(n) (n <= 0?( n < 0 ?-1:0):1)

LighthousePlanner::LighthousePlanner()
    :nh_(),
    lighthouse_pose_sub_(),
    target_offset_object_sub_(),
    dock_stop_sub_(),
    task_finished_pub_(),
    cmd_vel_pub_(),
    get_target_offset_object_(false),
    get_new_target_(false),
    get_new_dock_type_(false),
    continue_get_lighthouse_failed_times_(0),
    target_offset_object_(),
    dock_type_("VerticalEntry"),
    target_in_agv_()
{
    lighthouse_pose_sub_ = nh_.subscribe("/lighthouse_pose", 1, &LighthousePlanner::LighthousePoseCallback, this );//订阅object pose
    target_offset_object_sub_ = nh_.subscribe("/target_offset_lighthouse", 1, &LighthousePlanner::TargetOffsetLighthouseCallback, this );//offset触发
    dock_stop_sub_ = nh_.subscribe("/dock_stop", 1, &LighthousePlanner::DockStopCallback, this );//停止任务

    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel",1);
    task_finished_pub_ = nh_.advertise<std_msgs::String> ( "/waypoint_user_sub", 1 );//task完成
}

LighthousePlanner::~LighthousePlanner()
{
}

void LighthousePlanner::GetParam()
{
    ros::NodeHandle nh_private("~");

    nh_private.param("length_agv_center_to_head", length_agv_center_to_head_, 0.35);

    nh_private.param("p_angle_target", p_angle_target_, 0.0);
    nh_private.param("p_angle_first_turn", p_angle_first_turn_, 0.0);
    nh_private.param("p_angle_speed", p_angle_speed_, 0.0);
    nh_private.param("p_erf_angle_control", p_erf_angle_control_, 0.0);

    nh_private.param("rho_threshold", rho_threshold_, 1.0);
    nh_private.param("erf_rho_threshold", erf_rho_threshold_, 0.3);
    nh_private.param("max_angular", max_angular_, 0.14);
    nh_private.param("max_linear", max_linear_, 0.5);
    nh_private.param("x_tolerate_offset", x_tolerate_offset_, 0.01);
    nh_private.param("y_tolerate_offset", y_tolerate_offset_, 0.01);
    nh_private.param("theat_tolerate_offset", theat_tolerate_offset_, 0.01);
}

void LighthousePlanner::DockStopCallback(const std_msgs::String& task_msg)
{
    if ( task_msg.data == "stop" )
    {
        DockFinish();
    }
}

void LighthousePlanner::TargetOffsetLighthouseCallback(const geometry_msgs::Pose2DPtr& target_offset_object_msg)
{
    if ( target_offset_object_msg->x < 0.0 )
    {
        target_offset_object_.x = target_offset_object_msg->x;
        target_offset_object_.y = target_offset_object_msg->y;
        target_offset_object_.theta = target_offset_object_msg->theta;
        get_target_offset_object_ = true;
    }
    else
    {
        ROS_WARN( "Is a wrong target, the x mast negative" );
    }
}

void LighthousePlanner::LighthousePoseCallback(const geometry_msgs::PoseStampedPtr& lighthouse_pose_stamped_in_agv_msg)
{
    if ( !get_target_offset_object_ )
    {
        return;
    }

    try
    {
        tf::assertQuaternionValid(lighthouse_pose_stamped_in_agv_msg->pose.orientation );
    }
    catch (tf2::InvalidArgumentException& e)//停止识别，进入空闲等待状态
    {
        get_new_target_ = false;
        ++continue_get_lighthouse_failed_times_;
        ROS_ERROR( "assertQuaternionValid: %s", e.what() );//Quaternion malformed
        return;
    }

    get_new_target_ = true;
    continue_get_lighthouse_failed_times_ = 0;

    // tf::Quaternion q_in_agv;
    // tf::quaternionMsgToTF(lighthouse_pose_stamped_in_agv_msg->pose.orientation, q_in_agv);

    // double roll, pitch, yaw_target_with_agv;
    // tf::Matrix3x3(q_in_agv).getEulerYPR(yaw_target_with_agv, pitch, roll);

    // double yaw_cos = cos(yaw_target_with_agv);
    // double yaw_sin = sin(yaw_target_with_agv);

    // target_in_agv_.position.x = lighthouse_pose_stamped_in_agv_msg->pose.position.x + target_offset_object_.x * yaw_cos + target_offset_object_.y * yaw_sin;
    // target_in_agv_.position.y = lighthouse_pose_stamped_in_agv_msg->pose.position.y + target_offset_object_.y * yaw_cos - target_offset_object_.x * yaw_sin;
    // target_in_agv_.orientation = tf::createQuaternionMsgFromYaw(yaw_target_with_agv + target_offset_object_.theta);
    // cout << "target_in_agv_.position.x: " << target_in_agv_.position.x << endl;
    // cout << "target_in_agv_.position.y: " << target_in_agv_.position.y << endl;

    //三角在agv坐标系
    Eigen::Isometry3d lighthouse_in_agv;
    tf::poseMsgToEigen(lighthouse_pose_stamped_in_agv_msg->pose, lighthouse_in_agv);

    //目标在三角坐标系
    geometry_msgs::Pose target_in_lighthouse_pose;
    target_in_lighthouse_pose.position.x = target_offset_object_.x;
    target_in_lighthouse_pose.position.y = target_offset_object_.y;
    target_in_lighthouse_pose.position.z = lighthouse_pose_stamped_in_agv_msg->pose.position.z;
    target_in_lighthouse_pose.orientation = tf::createQuaternionMsgFromYaw(target_offset_object_.theta);
    Eigen::Isometry3d target_in_lighthouse;
    tf::poseMsgToEigen(target_in_lighthouse_pose, target_in_lighthouse);
    // cout << "target_in_lighthouse_pose.orientation: " << target_in_lighthouse_pose.orientation << endl;

    //目标在agv坐标系
    Eigen::Isometry3d target_in_agv;
    target_in_agv = lighthouse_in_agv * target_in_lighthouse;
    // target_in_agv = target_in_lighthouse * lighthouse_in_agv;

    tf::poseEigenToMsg(target_in_agv, target_in_agv_);
    //cout << "target_in_agv_: " << target_in_agv_ << endl;

    return;
}

double LighthousePlanner::CalculateSidecBySideaSidebAnglec(double side_a, double side_b, double angle_c)
{
    double side_c = sqrt( pow(side_a,2) + pow(side_b,2) - 2.0 * side_a * side_b * cos(angle_c) );
    return side_c;
}

double LighthousePlanner::CalculateAngleaBySideaSidebAnglec(double side_a, double side_b, double angle_c)
{
    double side_c = sqrt( pow(side_a,2) + pow(side_b,2) - 2.0 * side_a * side_b * cos(angle_c) );
    double angle_a = acos( ( pow(side_b,2) + pow(side_c,2) - pow(side_a,2) ) / ( 2.0 * side_b * side_c ) );
    return angle_a;
}

void LighthousePlanner::CmdVelPub( double linear, double angular )
{
    geometry_msgs::Twist twist_msg;
    twist_msg.linear.x = linear;
    twist_msg.angular.z = angular;
    cmd_vel_pub_.publish(twist_msg);
}

bool LighthousePlanner::AdjustOnlyOrientation(double yaw_target_with_agv, double s_p123, double angle_target)
{
    double linear = 0.0;
    double angular = 0.0;
    double angular_set = 0.15;
    double angle_range = AngleToRadian(4.0);
    bool adjust_orientation_ok = false;

    static int right_side_times = 0;
    static int left_side_times = 0;

    if ( right_side_times > 5 && left_side_times > 5 )
    {
        if ( fabs(yaw_target_with_agv) < AngleToRadian(0.8) )
        {
            linear = 0.0;
            angular = 0.0;
            adjust_orientation_ok = true;
        }
        else
        {
            angular_set = 0.05;
            angular = SIGN(yaw_target_with_agv) * angular_set;
        }
    }
    else
    {
        if ( s_p123 > 0 )
        {
            ++right_side_times;
            if ( yaw_target_with_agv < angle_target )
            {
                angular = -SIGN(s_p123) * angular_set;
            }
            else if ( yaw_target_with_agv > angle_target + angle_range )
            {
                angular = SIGN(s_p123) * angular_set;
            }
            else
            {
                adjust_orientation_ok = true;
            }
        }
        else
        {
            ++left_side_times;
            if ( yaw_target_with_agv > angle_target )
            {
                angular = -SIGN(s_p123) * angular_set;
            }
            else if ( yaw_target_with_agv < angle_target + SIGN(s_p123) * angle_range )
            {
                angular = SIGN(s_p123) * angular_set;
            }
            else
            {
                adjust_orientation_ok = true;
            }
        }
    }

    if ( adjust_orientation_ok )
    {
        right_side_times = 0;
        left_side_times = 0;
    }

    CmdVelPub ( linear, angular );
    return adjust_orientation_ok;
}

bool LighthousePlanner::AdjustOrientationAndGoToCenter(double yaw_target_with_agv, double s_p123, double distance_agv_coordinate_origin_to_target_middle_line)
{
    double target_pose_in_agv_orientation_with_agv_axisX = RadianToAngle(yaw_target_with_agv);
    double target_pose_in_agv_orientation_with_agv_axisX_fabs = fabs(target_pose_in_agv_orientation_with_agv_axisX);

    double angle_turn_target = 88;

    double linear = 0.0;
    double angular = 0.0;
    static double angular_set = 0.15;

    static bool get_agv_start_side = false;
    static bool arrived_center = false;
    static bool first_trun_ok = false;

    double agv_start_side = 0;
    if ( !get_agv_start_side )
    {
        agv_start_side = s_p123;
        get_agv_start_side = true;
    }

    if ( !arrived_center )
    {
        if ( s_p123 > 0 )
        {
            if ( target_pose_in_agv_orientation_with_agv_axisX < angle_turn_target )
            {
                angular = -SIGN(s_p123) * angular_set;
            }
            else if ( target_pose_in_agv_orientation_with_agv_axisX > angle_turn_target + 4.0 )
            {
                angular = SIGN(s_p123) * angular_set;
            }
            else
            {
                angular_set = 0.08;
                first_trun_ok = true;
            }
        }
        else
        {
            if ( target_pose_in_agv_orientation_with_agv_axisX > -angle_turn_target )
            {
                angular = -SIGN(s_p123) * angular_set;
            }
            else if ( target_pose_in_agv_orientation_with_agv_axisX < -(angle_turn_target + 4.0) )
            {
                angular = SIGN(s_p123) * angular_set;
            }
            else
            {
                angular_set = 0.08;
                first_trun_ok = true;
            }
        }
        if ( first_trun_ok )
        {
            linear = 0.08;
        }
        //agv到达中线，停车
        if ( distance_agv_coordinate_origin_to_target_middle_line < 0.01 )
        {
            linear = 0.0;
            angular = 0.0;
            arrived_center = true;
        }
        //agv靠近中线，减速
        else if ( distance_agv_coordinate_origin_to_target_middle_line < 0.03 )
        {
            linear = 0.03;
        }
        //判断agv是否越过中线，越过调头
        else if ( distance_agv_coordinate_origin_to_target_middle_line > 0.03 )
        {
            if ( s_p123 * agv_start_side < 0 )
            {
                agv_start_side = s_p123;
                if ( (target_pose_in_agv_orientation_with_agv_axisX_fabs > 88.0) && (target_pose_in_agv_orientation_with_agv_axisX_fabs < 92.0) )
                {
                    linear = 0.0;
                    angular = -SIGN(s_p123) * angular_set;
                }
            }
        }
    }
    else//agv已经到达三角中线，将agv方向转到与中线平行
    {
        //agv与中线角度较大，增加角速度
        if ( target_pose_in_agv_orientation_with_agv_axisX_fabs > 5.0 )
        {
            angular_set = 0.15;
        }
        //agv接近与中线平行，减小角速度
        else if ( target_pose_in_agv_orientation_with_agv_axisX_fabs > 2.0 )//head in right,turn left
        {
            angular_set = 0.05;
        }
        //agv满足与中线近似平行，停车，结束该阶段的调整
        else
        {
            CmdVelPub ( 0.0, 0.0 );
            angular_set = 0.15;
            get_agv_start_side = false;
            arrived_center = false;
            first_trun_ok = false;
            return true;
        }
        linear = 0.0;
        angular = SIGN(yaw_target_with_agv) * angular_set;
    }

    CmdVelPub ( linear, angular );
    return false;
}

void LighthousePlanner::DockToTarget()
{
    tf::Quaternion q_in_agv;
    tf::quaternionMsgToTF(target_in_agv_.orientation, q_in_agv);

    double roll, pitch, yaw_target_with_agv;
    tf::Matrix3x3(q_in_agv).getEulerYPR(yaw_target_with_agv, pitch, roll);
    //cout << "yaw_target_with_agv: " << RadianToAngle(yaw_target_with_agv) << endl;

    double distance_agv_center_to_target = sqrt(pow(target_in_agv_.position.x, 2) + pow(target_in_agv_.position.y, 2));
    //cout << "target x: " << target_in_agv_.position.x << " y: " << target_in_agv_.position.y << " theat: " << RadianToAngle(yaw_target_with_agv) << " distance: " << distance_agv_center_to_target << endl;

    double yaw_target_with_agv_fabs = fabs(yaw_target_with_agv);

    if ( !get_new_dock_type_ )
    {
        get_new_dock_type_ = true;
        if ( yaw_target_with_agv_fabs < 0.25 * M_PI  )
        {
            dock_type_ = "VerticalEntry";
        }
        else
        {
            dock_type_ = "ParallelEntry";
        }
    }

    double target_middle_line_k = tan(yaw_target_with_agv);
    double target_middle_line_b = target_in_agv_.position.y - target_middle_line_k * target_in_agv_.position.x;
    double distance_agv_coordinate_origin_to_target_middle_line = fabs(target_middle_line_b) / sqrt(target_middle_line_k*target_middle_line_k + 1);
    double target_vertex_angle_with_agv_coordinate_axisX = atan2( target_in_agv_.position.y, target_in_agv_.position.x);
    double angle_agv_center_target_vertex_target_middle_line = asin(distance_agv_coordinate_origin_to_target_middle_line/distance_agv_center_to_target);
    ROS_DEBUG("angle_agv_center_target_vertex_target_middle_line: %f", RadianToAngle(angle_agv_center_target_vertex_target_middle_line));

    double distance_agv_head_to_target = 0.0;
    if ( fabs(target_vertex_angle_with_agv_coordinate_axisX) < 0.06 )//3.4度
    {
        distance_agv_head_to_target = distance_agv_center_to_target - length_agv_center_to_head_;
    }
    else
    {
        distance_agv_head_to_target = CalculateSidecBySideaSidebAnglec(distance_agv_center_to_target, length_agv_center_to_head_, target_vertex_angle_with_agv_coordinate_axisX);
    }

    double middle_line_x1 = target_in_agv_.position.x;
    double middle_line_y1 = target_in_agv_.position.y;
    double middle_line_x2 = 0.0;
    double middle_line_y2 = 0.0;

    if ( yaw_target_with_agv_fabs < 0.5 * M_PI && yaw_target_with_agv_fabs != 0.0 )
    {
        middle_line_x2 = middle_line_x1 + fabs(target_middle_line_k);
    }
    else if ( yaw_target_with_agv_fabs > 0.5 * M_PI && yaw_target_with_agv_fabs < M_PI )
    {
        middle_line_x2 = middle_line_x1 - fabs(target_middle_line_k);
    }
    else //90.0, 0.0, 180.0
    {
        ROS_WARN("yaw_target_with_agv is zero");
        return;
    }

    middle_line_y2 = target_middle_line_k * middle_line_x2 + target_middle_line_b;
    double s_p123 = (middle_line_x1-0) * (middle_line_y2-0) - (middle_line_y1-0) * (middle_line_x2-0);

    double max_tringle_middle_with_agv_axisX = 5.0 * M_PI / 9.0;//0.5 * M_PI;
    static bool turn_ok = false;
    if ( !turn_ok )
    {
        if ( dock_type_ == "VerticalEntry" )
        {
            double angle_turn = SIGN(s_p123)* p_angle_first_turn_ * angle_agv_center_target_vertex_target_middle_line;
            if ( fabs(angle_turn) > max_tringle_middle_with_agv_axisX )
            {
                angle_turn = SIGN(angle_turn) * max_tringle_middle_with_agv_axisX;
            }
            turn_ok = AdjustOnlyOrientation(yaw_target_with_agv, s_p123, angle_turn);
        }
        else if ( dock_type_ == "ParallelEntry" )
        {
            turn_ok = AdjustOrientationAndGoToCenter(yaw_target_with_agv, s_p123, distance_agv_coordinate_origin_to_target_middle_line);
        }
        else
        {
            ROS_ERROR("Undefined dock type: %s", dock_type_.c_str());
        }
        return;
    }

    double linear = 0.0;
    double angular = 0.0;
    double angle_control = 0.0;

    string dock_status;
    static bool step_three = false;
    static int distance_satisfy_times = 0;
    static int angle_satisfy_times = 0;
    static int angle_unsatisfy_times = 0;

    double angle_target = SIGN(s_p123)* p_angle_target_ * angle_agv_center_target_vertex_target_middle_line;
    //cout << "angle_agv_center_target_vertex_target_middle_line: " << angle_agv_center_target_vertex_target_middle_line << endl;
    //cout << "angle_target: " << RadianToAngle(angle_target) << endl;

    if ( distance_agv_head_to_target > 0.02 )
    {
        dock_status = "StepOne";
        angle_control = p_angle_speed_ * (yaw_target_with_agv - angle_target);

        //double linear_param = 0.3 * erf ( distance_agv_head_to_target + erf_rho_threshold_ - 0.5 * distance_agv_coordinate_origin_to_target_middle_line - k4_erf_ * fabs(angle_control) );
        double linear_param = 0.25 * erf ( 0.8 * distance_agv_head_to_target + erf_rho_threshold_ - p_erf_angle_control_ * fabs(angle_control) );
        if ( linear_param < 0.0 )
        {
            linear_param = 0.0;
        }

        linear = linear_param * distance_agv_head_to_target + 0.002;//0.006
        angular = angle_control;
    }
    else if ( distance_agv_head_to_target > x_tolerate_offset_ )
    {
        distance_satisfy_times = 0;
        angle_satisfy_times = 0;

        dock_status = "StepTwo";
        if ( !step_three )
        {
            angle_control = p_angle_speed_ * (yaw_target_with_agv - 0.5 * angle_target);
        }
        else
        {
            angle_control = yaw_target_with_agv;
        }

        linear = 0.001;//0.005
        angular = angle_control;
    }
    else
    {
        ++distance_satisfy_times;
        step_three = true;

        if ( fabs(yaw_target_with_agv) < theat_tolerate_offset_ )
        {
            dock_status = "------------------StepThree:both ok";
            ++angle_satisfy_times;
        }
        else
        {
            dock_status = "-----StepThree:distance ok";
            angle_satisfy_times = 0;
            ++angle_unsatisfy_times;
        }

        linear = 0.0;
        angular = yaw_target_with_agv;

        if ( (distance_satisfy_times > 6 && angle_satisfy_times > 8) || angle_unsatisfy_times > 35 )
        {
            DockFinish();
            distance_satisfy_times = 0;
            angle_satisfy_times = 0;
            angle_unsatisfy_times = 0;
            step_three = false;
            if ( angle_unsatisfy_times > 35 )
            {
                dock_status = "Finished failed";
            }
            else
            {
                dock_status = "Finished ok";
            }
            printf ( "--------%s %6.4f %6.4f %6.4f | %6.4f %6.4f %6.4f\n",
                     dock_status.c_str(), target_in_agv_.position.x, target_in_agv_.position.y, RadianToAngle(yaw_target_with_agv),
                     distance_agv_head_to_target, linear, angular );
            return;
        }
    }

    if ( fabs(yaw_target_with_agv) > (M_PI / 3.0) )
    {
        if ( SIGN(yaw_target_with_agv) * angular < 0.0 )
        {
            if ( fabs(yaw_target_with_agv) > max_tringle_middle_with_agv_axisX )
            {
                linear = 0.08;
                angular = 0.0;
            }
        }
        else
        {
            if ( linear > 0.1 )
            {
                linear = 0.1;
            }
        }
    }

    if ( fabs ( linear ) > max_linear_ )
    {
        linear = SIGN ( linear ) * max_linear_;
    }
    if ( fabs ( angular ) > max_angular_ )
    {
        angular = SIGN ( angular ) * max_angular_;
    }

    CmdVelPub ( linear, angular );

//    printf ( "--------%s %6.4f %6.4f %6.4f | %6.4f %6.4f %6.4f | %6.4f %6.4f\n",
//             dock_status.c_str(), target_in_agv_.position.x, target_in_agv_.position.y, RadianToAngle(yaw_target_with_agv),
//             distance_agv_head_to_target, RadianToAngle(angle_target), RadianToAngle(angle_control), linear, angular );
    return;
}

void LighthousePlanner::DockFinish()
{
    CmdVelPub( 0.0, 0.0 );

    std_msgs::String task_state_msg;
    task_state_msg.data = "lighthouse_planner_finish";
    task_finished_pub_.publish(task_state_msg);

    get_target_offset_object_ = false;
    get_new_dock_type_ = false;
}

void LighthousePlanner::Run()
{
    GetParam();
    ros::Rate r(20);

    while( ros::ok() )
    {
        ros::spinOnce();
        if ( !get_target_offset_object_ )
        {
            r.sleep();
            continue;
        }

        if ( get_new_target_ )
        {
            get_new_target_ = false;
            DockToTarget();
        }
        else if ( continue_get_lighthouse_failed_times_ > 4 )
        {
            CmdVelPub(0.0, 0.0);
            if ( continue_get_lighthouse_failed_times_ % 10 == 0 )
            {
                ROS_WARN ( "get lighthouse pose failed times ********************************: %d", continue_get_lighthouse_failed_times_);
            }
        }
        r.sleep();
    }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lighthouse_planner");
  LighthousePlanner lighthouse_planner;
  lighthouse_planner.Run();
  return 0;
};
