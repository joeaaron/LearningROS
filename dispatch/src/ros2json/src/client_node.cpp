/*************************************************************************
 @Author: JoeAaron
 @Email: pant333@163.com
 @Created Time : 2018-10-24 10:00:42
 @Last Modified : 2018-10-24 10:00:42
 @File Name: client_node.cpp
 @Description:
 ************************************************************************/

#include <ros2json/dispatch.h>

#define RadianToAngle(X) ((X)*180.0/M_PI)
#define AngleToRadian(X) ((X)*M_PI/180.0)

#define MESSAGE_FREQ 1

void error(const char *msg) {
    perror(msg);
    exit(0);
}

class Listener {
private:
    char topic_message[256] = { 0 };
public:
    void callback(const std_msgs::String::ConstPtr& msg);
    char* getMessageValue();
};

void Listener::callback(const std_msgs::String::ConstPtr& msg) {
    memset(topic_message, 0, 256);
    strcpy(topic_message, msg->data.c_str());
    ROS_INFO("I heard:[%s]", msg->data.c_str());
}

char* Listener::getMessageValue() {
    return topic_message;
}

Dispatch::Dispatch()
    :nh_()
    ,robot_pose_sub_()
    ,move_base_status_sub_()
    ,move_base_simple_goal_pub_()
{
    robot_pose_sub_ = nh_.subscribe("/robot_pose", 1, &Dispatch::RobotPoseCallBack, this );
    move_base_status_sub_ = nh_.subscribe("/move_base/status", 1, &Dispatch::MoveBaseStatusCallback, this );
    move_base_simple_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1);
}

//todo缓存
void Dispatch::MoveBaseSimpleGoalPub()
{
    geometry_msgs::PoseStamped goal_msg;
    goal_msg.header.frame_id = "map";
    goal_msg.header.stamp = ros::Time();
    goal_msg.pose.position.x = 0.0;//todo
    goal_msg.pose.position.y = 0.0;//todo
    goal_msg.pose.position.z = 0.0;

    double theta = 0.0;//todo
    tf::Quaternion q = tf::createQuaternionFromYaw( theta );
    goal_msg.pose.orientation.x = q.getX();
    goal_msg.pose.orientation.y = q.getY();
    goal_msg.pose.orientation.z = q.getZ();
    goal_msg.pose.orientation.w = q.getW();

    move_base_simple_goal_pub_.publish(goal_msg);
}

void Dispatch::RobotPoseCallBack(const geometry_msgs::Pose &robot_pose_msg)
{
    agv_x = robot_pose_msg.position.x;
    agv_y = robot_pose_msg.position.y;
    agv_theta = RadianToAngle(tf::getYaw(robot_pose_msg.orientation));
    cout << "x= " << agv_x << endl << "y=" << agv_y << endl << "theta=" << agv_theta << endl;
}

void Dispatch::MoveBaseStatusCallback(const actionlib_msgs::GoalStatusArray &goal_status_array_msg)
{
    //1.发两个目标点的情况
    //2.判断导航任务是否完成
    goal_status_array_msg.status_list[0].status;// == 3;
    goal_status_array_msg.status_list[0].goal_id.id;

    //    std_msgs/Header header
    //      uint32 seq
    //      time stamp
    //      string frame_id
    //    actionlib_msgs/GoalStatus[] status_list
    //      uint8 PENDING=0
    //      uint8 ACTIVE=1
    //      uint8 PREEMPTED=2
    //      uint8 SUCCEEDED=3
    //      uint8 ABORTED=4
    //      uint8 REJECTED=5
    //      uint8 PREEMPTING=6
    //      uint8 RECALLING=7
    //      uint8 RECALLED=8
    //      uint8 LOST=9
    //      actionlib_msgs/GoalID goal_id
    //        time stamp
    //        string id
    //      uint8 status
    //      string text


    //            header:
    //              seq: 7922
    //              stamp:
    //                secs: 1540348304
    //                nsecs: 503665089
    //              frame_id: ''
    //            status_list:
    //              -
    //                goal_id:
    //                  stamp:
    //                    secs: 1540348299
    //                    nsecs: 781399327
    //                  id: /move_base-1-1540348299.781399327
    //                status: 4
    //                text: Failed to find a valid control. Even after executing recovery behaviors.
}


void Dispatch::DeserializedJson(std::string strValue)
{
    // used for test
    strValue = "{\"Dev_Type\":\"Core\",\
             \"Dev_ID\":\"1\",\
             \"Process\":\"PT_CAMERAGRAIN\",\
             \"Move\":[{\"Points_Num\":\"10\"},{\"Points_Array\":[{\"x\":\"106.32\"},{\"y\":\"-81.56\"}]}]}";


    Json::Reader reader;
    Json::Value value;

    if(reader.parse(strValue, value))
    {
        std::string action = value["Process"].asString();
        std::cout << action << std::endl;
        const Json::Value arrayObj = value["Move"];
        for(int i = 0; i < arrayObj.size(); i++)
        {
            std::string wayPoints = arrayObj[i]["Points_Array"].asString();
            std::cout << wayPoints;
            if(i != arrayObj.size() - 1)
                std::cout << std::endl;
        }
    }
    MoveBaseSimpleGoalPub();  //need some inputs
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "client_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(MESSAGE_FREQ); // set the rate as defined in the macro MESSAGE_FREQ

    Dispatch dispatch;
    int sockfd, portno, n, choice = 1;
    struct sockaddr_in serv_addr;
    struct hostent *server;
    char buffer[256];
    bool echoMode = false;
    if (argc < 3) {
        fprintf(stderr,"Usage: $ rosrun comm_tcp client_node <hostname> <port> --arguments\nArguments:\n -e : Echo mode\n");
        exit(0);
    }
    if (argc > 3)
        if (strcmp(argv[3], "-e") == 0)
            echoMode = true;
    portno = atoi(argv[2]);
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)
        error("ERROR opening socket");
    server = gethostbyname(argv[1]);
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr,
            (char *)&serv_addr.sin_addr.s_addr,
            server->h_length);
    serv_addr.sin_port = htons(portno);
    if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0)
        error("ERROR connecting");

    while(ros::ok())
    {
        ros::spinOnce();

        bzero(buffer,256);

        Json::Value DeviceInfor;
        DeviceInfor["Dev_Type"] = "AGV";
        DeviceInfor["Dev_ID"] = 1;

        Json::Value AGVInfor;
        AGVInfor["X"] = dispatch.agv_x;
        AGVInfor["Y"] = dispatch.agv_y;
        AGVInfor["a"] = dispatch.agv_theta;
        AGVInfor["ERROR_Code"] = "";
        AGVInfor["Pro_Type"] = "";
        AGVInfor["Pro_State"] = "";
        AGVInfor["Pro_Result"] = "";
        AGVInfor["Time"] = 890;

        /*AGVInfor["Time"] = times;*/
        DeviceInfor["Dev_Data"] = AGVInfor;

        std::string  ctos = DeviceInfor.toStyledString();
        ctos += 0x0d;
        ctos += 0x0a;

        n = write(sockfd,ctos.c_str(),ctos.size());
        cout<<"Client to sever:"<<ctos<<endl;

        if (n < 0)
            error("ERROR writing to socket");
        if (echoMode) {
            bzero(buffer, 256);
            n = read(sockfd,buffer,255);
            string value(buffer);
            dispatch.DeserializedJson(value);
            if (n < 0)
                error("ERROR reading reply");
            printf("%s\n", buffer);
        }

        loop_rate.sleep();
    }
    return 0;
}
