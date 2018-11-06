#ifndef DISPATCH_ROBOT_COMM
#define DISPATCH_ROBOT_COMM

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <yocs_msgs/Trajectory.h>
#include <yocs_msgs/TrajectoryList.h>
#include <yocs_msgs/NavigationControl.h>
#include <yocs_msgs/NavigationControlStatus.h>

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
#include <sys/time.h>

#include <json/json.h>

#define IDLE                    0
#define PUB_TRAJECTORIE_ADD     1
#define PUB_NAVIGATION_CONTROL  2
#define WAIT_MOVE_FINISH        3
#define PUB_TRAJECTORIE_REMOVE  4

#define BUFFERSIZE              1024

using namespace std;
class Dispatch
{
public:
    Dispatch();
    ~Dispatch();
    void Run();

protected:
    ros::NodeHandle nh_;

    ros::Subscriber robot_pose_sub_;
    ros::Subscriber navigation_control_status_sub_;

    ros::Publisher trajectories_add_pub_;
    ros::Publisher trajectories_remove_pub_;
    ros::Publisher navigation_control_pub_;

    std::string server_ip_;
    int server_port_;
    int socket_fd_;

    bool trajectorie_finished_;
    ros::Time trajectories_add_time_;

    int state_machine_;

    std::vector<std::string>    m_InforList;
    std::string                 m_lastPlus;

    int dev_id_;
    int dispatch_comm_id_;
    const string dispatch_trajectorie_base_name_;

    yocs_msgs::TrajectoryList trajectory_list_msg_;

    bool debug_;

    double agv_x_;
    double agv_y_;
    double agv_theta_;

    bool ConnectServer(string ip_str, int port );
    int SetNonBlock ( const int sockfd, const struct sockaddr* serv_addr,
                      const socklen_t socklen, const int nsec, const int usec );

    void RobotPoseCallBack(const geometry_msgs::PoseConstPtr &robot_pose_msg);
    void NavigationControlStatusCallback(const yocs_msgs::NavigationControlStatusConstPtr &navigation_control_msg);

    void TrajectorieAddPub();
    void TrajectorieRemovePub();
    void NavigationControlPub();

    void ReceiveData(char* phead, int dSize);
    void DeserializedJson(std::string strValue);

    void MsgAGVToDispatch(string dev_type, string& msg_agv_to_dispatch);

    void CheckDispatchTaskList();
    void DoDispatchTask();
};

#endif
