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
#include <fcntl.h>
#include <arpa/inet.h>

#include <iostream>
#include <vector>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <yocs_msgs/Trajectory.h>
#include <yocs_msgs/NavigationControl.h>
#include <yocs_msgs/NavigationControlStatus.h>

#include <json/json.h>

using namespace std;
class Dispatch
{
public:
    Dispatch();
    ~Dispatch();

    void Run();
    void RunTest();

private:
    ros::NodeHandle nh_;

    ros::Subscriber robot_pose_sub_;
    ros::Subscriber move_base_status_sub_;
    ros::Subscriber navigation_control_status_sub_;

    ros::Publisher move_base_simple_goal_pub_;
    ros::Publisher trajectories_add_pub_;
    ros::Publisher trajectories_remove_pub_;
    ros::Publisher navigation_control_pub_;

    std::string server_ip_;
    int server_port_;
    int socket_fd_;

    bool trajectorie_finished_;
    ros::Time trajectories_add_time_;

    int state_machine_;

    bool ConnectServer(string ip_str, int port );
    int SetNonBlock ( const int sockfd, const struct sockaddr* serv_addr,
                      const socklen_t socklen, const int nsec, const int usec );

    string MsgAGVToDispatch(string dev_type, int dev_id, int times);

    void MoveBaseSimpleGoalPub();
    void RobotPoseCallBack(const geometry_msgs::PoseConstPtr &robot_pose_msg);
    void MoveBaseStatusCallback(const actionlib_msgs::GoalStatusArrayConstPtr &goal_status_array_msg);
    void NavigationControlStatusCallback(const yocs_msgs::NavigationControlStatusConstPtr &navigation_control_msg);
    void TrajectorieAddPub();
    void TrajectorieRemovePub();
    void NavigationControlPub();
    void DoMove();

    void DeserializedJson(std::string strValue);
    void DeserializedJsonTest();
    void DeserializedJsonTest1(std::string strValue);

    void ReceiveData(char* phead, int dSize);

public:
    double agv_x;
    double agv_y;
    double agv_theta;

    vector<double> dispatch_x;
    vector<double> dispatch_y;
    vector<double> dispatch_theta;

	std::vector<std::string>    m_InforList;
	std::strig                 m_lastPlus;

    int comID;
};

#endif
