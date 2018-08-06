#ifndef LIGHTHOUSE_PLANNER
#define LIGHTHOUSE_PLANNER

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
//#include <eigen3/Eigen/Eigen>
#include <sensor_msgs/LaserScan.h>

class LighthousePlanner
{
public:
    LighthousePlanner();
    ~LighthousePlanner();

    void Run();

private:
    ros::NodeHandle nh_;

    ros::Subscriber lighthouse_pose_sub_;
    ros::Subscriber dock_stop_sub_;
    ros::Subscriber target_offset_object_sub_;

    ros::Publisher task_finished_pub_;
    ros::Publisher cmd_vel_pub_;

    double length_agv_center_to_head_;

    double p_angle_target_;
    double p_angle_first_turn_;
    double p_angle_speed_;
    double p_erf_angle_control_;

    double rho_threshold_;
    double erf_rho_threshold_;
    double max_angular_;
    double max_linear_;
    double x_tolerate_offset_;
    double y_tolerate_offset_;
    double theat_tolerate_offset_;

    bool get_target_offset_object_;
    bool get_new_target_;
    bool get_new_dock_type_;

    int continue_get_lighthouse_failed_times_;
    std::string dock_type_;

    geometry_msgs::Pose2D target_offset_object_;
    geometry_msgs::Pose target_in_agv_;

    void GetParam();
    void DockFinish();

    void DockStopCallback(const std_msgs::String& task_msg);
    void TargetOffsetLighthouseCallback(const geometry_msgs::Pose2DPtr& target_offset_object_msg);
    void LighthousePoseCallback(const geometry_msgs::PoseStampedPtr& lighthouse_pose_stamped_in_agv_msg);

    double CalculateSidecBySideaSidebAnglec(double side_a, double side_b, double angle_c);
    double CalculateAngleaBySideaSidebAnglec(double side_a, double side_b, double angle_c);
    bool AdjustOnlyOrientation(double yaw_target_with_agv, double s_p123, double angle_target);
    bool AdjustOrientationAndGoToCenter(double yaw_target_with_agv, double s_p123, double distance_agv_coordinate_origin_to_target_middle_line);
    void DockToTarget();
    void CmdVelPub( double linear, double angular );
};

#endif
