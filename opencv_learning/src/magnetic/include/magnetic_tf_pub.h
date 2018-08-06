#ifndef MAGNETIC_TF_PUB
#define MAGNETIC_TF_PUB

#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/Pose2D.h>

class MagneticTfPub
{
  private:
    ros::NodeHandle nh_;
    ros::Subscriber magnetic_head_sub_;
    ros::Subscriber magnetic_tail_sub_;

    bool magnetic_head_is_forward_;
    bool magnetic_tail_is_forward_;

    double distance_magnetic_goal_to_magnetic_center_;
    double fake_pose_y_out_track_cm_;

    bool get_new_head_magnetic_data_;
    bool get_new_tail_magnetic_data_;

    bool start_magnetic_navigation_;
    bool start_magnetic_navigation_first_time_;
    bool magnetic_head_on_track_;

    geometry_msgs::Pose2D magnetic_head_pose2d_;
    geometry_msgs::Pose2D magnetic_tail_pose2d_;

    void MagneticHeadCallback(const std_msgs::Int32MultiArrayPtr magnetic_head_msg);
    void MagneticTailCallback(const std_msgs::Int32MultiArrayPtr magnetic_tail_msg);
    double GetPoseYInRobotBase(bool magnetic_is_forward, const std_msgs::Int32MultiArrayPtr magnetic_tail_msg);
    double FakeYForControl(double magnetic_y_cm);

  public:
    MagneticTfPub();
    ~MagneticTfPub();

    void Run();
};

#endif
