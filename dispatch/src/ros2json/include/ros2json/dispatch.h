#ifndef MAGNETIC_DRIVER
#define MAGNETIC_DRIVER

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib_msgs/GoalStatusArray.h>

#include <json/json.h>


using namespace std;
class Dispatch
{
public:
    Dispatch();
    //void Run();
    void DeserializedJson(std::string strValue);

private:
    ros::NodeHandle nh_;
    ros::Subscriber robot_pose_sub_;
    ros::Subscriber move_base_status_sub_;
    ros::Publisher move_base_simple_goal_pub_;

    void MoveBaseSimpleGoalPub();
    void RobotPoseCallBack(const geometry_msgs::Pose &robot_pose_msg);
    void MoveBaseStatusCallback(const actionlib_msgs::GoalStatusArray &goal_status_array_msg);

public:
    double agv_x;
    double agv_y;
    double agv_theta;

    vector<double> dispatch_x;
    vector<double> dispatch_y;
    vector<double> dispatch_theta;

};

#endif
