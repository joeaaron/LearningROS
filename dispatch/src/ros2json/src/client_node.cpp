#include <ros2json/dispatch.h>

#define RadianToAngle(X) ((X)*180.0/M_PI)
#define AngleToRadian(X) ((X)*M_PI/180.0)

#define IDLE                    0
#define PUB_TRAJECTORIE_ADD     1
#define PUB_NAVIGATION_CONTROL  2
#define WAIT_MOVE_FINISH        3
#define PUB_TRAJECTORIE_REMOVE  4

// const int BUFFERSIZE = 2048;
const int BUFFERSIZE = 1024;

Dispatch::Dispatch()
    :nh_()
    ,robot_pose_sub_()
    ,move_base_status_sub_()
    ,navigation_control_status_sub_()
    ,move_base_simple_goal_pub_()
    ,trajectories_add_pub_()
    ,trajectories_remove_pub_()
    ,navigation_control_pub_()
    ,trajectories_add_time_()
    ,state_machine_(IDLE)
    ,trajectorie_finished_(false)
    ,comID(0)
{
    robot_pose_sub_ = nh_.subscribe("/robot_pose", 1, &Dispatch::RobotPoseCallBack, this );
    move_base_status_sub_ = nh_.subscribe("/move_base/status", 1, &Dispatch::MoveBaseStatusCallback, this );
    navigation_control_status_sub_ = nh_.subscribe("/nav_ctrl_status", 3, &Dispatch::NavigationControlStatusCallback, this );

    move_base_simple_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1);
    trajectories_add_pub_ = nh_.advertise<yocs_msgs::Trajectory>("/trajectory_add",1);
    trajectories_remove_pub_ = nh_.advertise<yocs_msgs::Trajectory>("/trajectory_remove",1);
    navigation_control_pub_ = nh_.advertise<yocs_msgs::NavigationControl>("/nav_ctrl",1);

    nh_.param( "server_ip", server_ip_, (string)"192.168.1.104" );
    nh_.param( "server_port", server_port_, 10001 );
}

Dispatch::~Dispatch(){}

bool Dispatch::ConnectServer (string ip_str, int port )
{
    struct sockaddr_in server_addr;

    cout<<"ip_: "<<ip_str<<". port: "<<port<<endl;
    if ( ( socket_fd_ = socket ( AF_INET, SOCK_STREAM, 0 ) ) < 0 )
    {
        ROS_ERROR ( "create socket error: %s(errno:%d)\n)", strerror ( errno ), errno );
        return false;
    }

    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    if ( inet_pton ( AF_INET, ip_str.c_str(), &server_addr.sin_addr ) <= 0 )
    {
        ROS_ERROR ( "inet_pton error: %s(errno:%d))\n", strerror ( errno ), errno );
        return false;
    }

    if ( SetNonBlock ( socket_fd_, ( struct sockaddr* ) &server_addr, sizeof ( server_addr ), 0, 1000000 ) == 0 )
    {
        ROS_INFO ( "igv connected!" );
        return true;
    }

    return false;
}

int Dispatch::SetNonBlock ( const int sockfd, const struct sockaddr* serv_addr, const socklen_t socklen,
                                   const int nsec, const int usec )
{
    int flags, n, error = 0;
    socklen_t len;
    fd_set rset, wset;
    struct timeval tval;

    flags = fcntl ( sockfd, F_GETFL, 0 );
    fcntl ( sockfd, F_SETFL, flags | O_NONBLOCK );
    if ( ( n = connect ( sockfd, serv_addr, socklen ) ) < 0 )
    {
        if ( errno != EINPROGRESS )
        {
            return -1;
        }
    }

    if ( n == 0 )
    {
        goto GotoTest;
    }
    FD_ZERO ( &rset );
    FD_SET ( sockfd, &rset );
    wset = rset;
    tval.tv_sec = nsec;
    tval.tv_usec = usec;

    if ( ( n = select ( sockfd + 1, &rset, &wset, NULL, &tval ) ) == 0 )
    {
        close ( sockfd );
        errno = ETIMEDOUT;
        return -1;
    }
    if ( FD_ISSET ( sockfd, &rset ) || FD_ISSET ( sockfd, &wset ) )
    {
        len = sizeof ( error );
        if ( getsockopt ( sockfd, SOL_SOCKET, SO_ERROR, &error, &len ) < 0 )
        {
            return -1;
        }
    }
    else
    {
        ROS_DEBUG ( "select error: sockfd  not set" );
    }
GotoTest:
    fcntl ( sockfd, F_SETFL, flags );
    if ( error )
    {
        close ( sockfd );
        errno = error;
        return -1;
    }
    return 0;
}

void Dispatch::TrajectorieAddPub()
{
    yocs_msgs::Trajectory trajectory_msg;
    trajectory_msg.name = "dispatch_assign";

    int num = dispatch_x.size();
    cout << "num: " << num << endl;

    if ( num > 0 )
    {
        trajectory_msg.waypoints.resize(num);
        for ( int i = 0; i < num; ++i )
        {
            trajectory_msg.waypoints[i].header.frame_id = "{\"close_enough\":\"0.05\",\"failure_mode\":\"LOOP\",\"frame_id\":\"map\",\"goal_timeout\":\"0\",\"mark\":\"false\",\"type\":\"goal\"}";
            trajectory_msg.waypoints[i].name = "name" + boost::lexical_cast<string>(i);// const_cast<string>(i);
            trajectory_msg.waypoints[i].pose.position.x = dispatch_x[i];
            trajectory_msg.waypoints[i].pose.position.y = dispatch_y[i];
            trajectory_msg.waypoints[i].pose.position.z = 0.0;
            tf::Quaternion q = tf::createQuaternionFromYaw( AngleToRadian(dispatch_theta[i]) );
            trajectory_msg.waypoints[i].pose.orientation.x = q.getX();
            trajectory_msg.waypoints[i].pose.orientation.y = q.getY();
            trajectory_msg.waypoints[i].pose.orientation.z = q.getZ();
            trajectory_msg.waypoints[i].pose.orientation.w = q.getW();
            /*rajectory_msg.waypoints[i].close_enough = 0.0;*/
            //trajectory_msg.waypoints[i].goal_timeout = 0.0;
            /*trajectory_msg.waypoints[i].failure_mode = "";*/
        }
        cout << "--------------trajectories_add_pub_.publish(trajectory_msg)-----------" << endl;
        trajectories_add_pub_.publish(trajectory_msg);

        trajectories_add_time_ = ros::Time::now();
    }
}

void Dispatch::TrajectorieRemovePub()
{
    yocs_msgs::Trajectory trajectory_msg;
    trajectory_msg.name = "dispatch_assign";

    trajectories_remove_pub_.publish(trajectory_msg);
}

void Dispatch::NavigationControlPub()
{
    yocs_msgs::NavigationControl navigation_control_msg;

    navigation_control_msg.control = 1;
    navigation_control_msg.goal_name = "dispatch_assign";

    navigation_control_pub_.publish(navigation_control_msg);
}

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

void Dispatch::NavigationControlStatusCallback(const yocs_msgs::NavigationControlStatusConstPtr &navigation_control_msg)
{
    cout << "navigation_control_msg->status: " << static_cast<int>(navigation_control_msg->status) << endl;
    if ( static_cast<int>(navigation_control_msg->status) == 3 )
    {
        cout << "goal ok" << endl;
        trajectorie_finished_ = true;
    }
}

void Dispatch::RobotPoseCallBack(const geometry_msgs::PoseConstPtr &robot_pose_msg)
{
    agv_x = robot_pose_msg->position.x;
    agv_y = robot_pose_msg->position.y;
    agv_theta = RadianToAngle(tf::getYaw(robot_pose_msg->orientation));
    //cout << "x= " << agv_x << endl << "y=" << agv_y << endl << "theta=" << agv_theta << endl;
}

void Dispatch::MoveBaseStatusCallback(const actionlib_msgs::GoalStatusArrayConstPtr &goal_status_array_msg)
{
    if ( !goal_status_array_msg->status_list.empty() )
    {
        if ( static_cast<int>(goal_status_array_msg->status_list[0].status) == 3 )
        {
            //cout << "goal ok" << endl;
            // trajectorie_finished_ = true;
        }
        //goal_status_array_msg->status_list[0].goal_id.id;
    }
}

string Dispatch::MsgAGVToDispatch(string dev_type, int dev_id, int times)//todo
{
    Json::Value DeviceInfor;
    DeviceInfor["Dev_Type"] = dev_type;//"AGV";
    DeviceInfor["Dev_ID"] = dev_id;//1;

    Json::Value AGVInfor;
    AGVInfor["Comm_ID"] = comID;
    AGVInfor["Map_ID"] = 1;
    AGVInfor["X"] = agv_x;
    AGVInfor["Y"] = agv_y;
    AGVInfor["a"] = agv_theta;
    AGVInfor["Cell_Quan"] = 0.98;
    AGVInfor["ERROR_Code"] = 1;
    AGVInfor["Pro_Type"] = "";
    AGVInfor["Pro_State"] = "";
    AGVInfor["Pro_Result"] = 1.0;
    AGVInfor["Time_stamp"] = times;

    //AGVInfor["Time"] = times;
    DeviceInfor["Dev_Data"] = AGVInfor;

    string msg_agv_to_dispatch = DeviceInfor.toStyledString();
    msg_agv_to_dispatch += 0x0d;
    msg_agv_to_dispatch += 0x0a;

    return msg_agv_to_dispatch;
}

void Dispatch::Run()
{
     bool connect_state = false;
     // char buffer[256];
     char buffer[BUFFERSIZE];
     int i = 0;

    ros::Rate loop_rate(1);

    while (ros::ok())
    {
        ++i;
        ros::spinOnce();

        if ( connect_state == false )
        {
            connect_state = ConnectServer ( server_ip_, server_port_ );
            if ( connect_state )
            {
                cout << "socket_fd_: " << socket_fd_ << endl;
                //set timeout
                struct timeval timeout = {3, 0};
                //int ret_send = setsockopt( socket_fd_external_server, SOL_SOCKET, SO_SNDTIMEO, (const char*)&timeout, sizeof(timeout) );
                int ret_recv = setsockopt ( socket_fd_, SOL_SOCKET, SO_RCVTIMEO, ( const char* ) &timeout, sizeof ( timeout ) );
            }
            else
            {
                ROS_ERROR ( "set up connect to server failed! will retry in 1s" );
                ros::Duration ( 1.0 ).sleep();
                continue;
            }
        }

        bzero(buffer, BUFFERSIZE);

        int recv_length = recv(socket_fd_, buffer, BUFFERSIZE, MSG_DONTWAIT);
        // //n = read(sockfd,buffer,BUFFERSIZE);
        // int recv_length = read(socket_fd_, buffer, BUFFERSIZE);
        // cout << "recv_length: " << recv_length << endl;

        if (recv_length > 0)
        {
            cout << "Begin to receive data." << endl;
            //deal string
            ReceiveData(buffer, recv_length);
            string value(buffer);   //char[] --> string
            cout << "recv buffer is :" << buffer << endl;

            //dispatch.DeserializedJson(value);
            DeserializedJsonTest1(value);
        }
        else if ( recv_length == -1 )
        {
            // cout << "no recv data" << endl;
        }
        else if ( recv_length == 0 )
        {
            ROS_ERROR ( "disconnented from server! will retry in 1s" );
            connect_state = false;
            ros::Duration ( 1.0 ).sleep();
            continue;
        }

        string msg_agv_to_dispatch = MsgAGVToDispatch("AGV", 1, i);

        int write_length = write(socket_fd_, msg_agv_to_dispatch.c_str(), msg_agv_to_dispatch.size());
        //cout << "send msg_agv_to_dispatch: " << msg_agv_to_dispatch << endl;

        if (write_length < 0)
        {
            ROS_ERROR("agv write to dispatch ERROR");
            // connect_state = false;
            ros::Duration ( 1.0 ).sleep();
            // continue;
        }

        DoMove();

        loop_rate.sleep();
    }
    close ( socket_fd_ );
}

void Dispatch::RunTest()
{
    // DeserializedJsonTest1();

    ros::Rate r(1);
    while (ros::ok())
    {
        ros::spinOnce();

        DoMove();

        r.sleep();
    }
}

void Dispatch::DoMove()
{
    if ( state_machine_ == IDLE )
    {

    }
    else if ( state_machine_ == PUB_TRAJECTORIE_ADD )
    {
        cout << "state_machine_: " << state_machine_ << endl;
        sleep(2);
        TrajectorieAddPub();
        state_machine_ = PUB_NAVIGATION_CONTROL;
    }
    else if ( state_machine_ == PUB_NAVIGATION_CONTROL )
    {
        cout << "state_machine_1: " << state_machine_ << endl;
        if ( ros::Time::now() - trajectories_add_time_ > ros::Duration(3.0) )
        {
            cout << "state_machine_2: " << state_machine_ << endl;
            NavigationControlPub();
            state_machine_ = WAIT_MOVE_FINISH;
        }
    }
    else if ( state_machine_ == WAIT_MOVE_FINISH )
    {
        cout << "state_machine_: " << state_machine_ << endl;
        if ( trajectorie_finished_ )
        {
            state_machine_ = PUB_TRAJECTORIE_REMOVE;
            trajectorie_finished_ = false;
        }
    }
    else if ( state_machine_ == PUB_TRAJECTORIE_REMOVE )
    {
        cout << "state_machine_: " << state_machine_ << endl;
        TrajectorieRemovePub();
        state_machine_ = IDLE;
        cout << "trajectorie finished" << endl;
    }
    else
    {
        cout << "undefined state_machine_" << endl;
    }
}

void Dispatch::DeserializedJson(std::string strValue)
{
	ROS_INFO("Begin to deserialize");

    Json::Reader reader;
    Json::Value json_object;

    if(reader.parse(strValue, json_object))
    {
        comID = json_object["Comm_ID"].asInt();

        std::string action = json_object["Process"].asString();
        std::cout << action << std::endl;
        const Json::Value arrayObj = json_object["Move"];
        // for(int i = 0; i < arrayObj.size(); i++)
        // {
        //     std::string wayPoints = arrayObj[i]["Points_Array"].asString();
        //     std::cout << wayPoints;
        //     if(i != arrayObj.size() - 1)
        //         std::cout << std::endl;
        // }

        for(int i = 0; i < arrayObj.size(); i++)
        {
            dispatch_x.clear();
            dispatch_y.clear();
            dispatch_theta.clear();
            dispatch_x.push_back( arrayObj[i]["Points_Array"]["X"].asDouble() );
            dispatch_y.push_back( arrayObj[i]["Points_Array"]["Y"].asDouble() );
            dispatch_theta.push_back( arrayObj[i]["Points_Array"]["a"].asDouble() );
            std::cout << "dispatch_x: " << dispatch_x[i] << std::endl;
            std::cout << "dispatch_y: " << dispatch_y[i] << std::endl;
            std::cout << "dispatch_theta: " << dispatch_theta[i] << std::endl;
        }
    }
    if ( !dispatch_x.empty() )
    {
        if ( state_machine_ == IDLE )
        {
            state_machine_ = PUB_TRAJECTORIE_ADD;
        }
        else
        {
            cout << "state_machine_ is not IDLE" << endl;
        }
    }
    //MoveBaseSimpleGoalPub();  //need some inputs
}

void Dispatch::DeserializedJsonTest1(std::string strValue)
{
    ROS_INFO("Begin to deserialize");

    Json::Reader reader;
    Json::Value json_object;

    if(reader.parse(strValue, json_object))
    {
        Json::Value  dev_data = json_object["Dev_Data"];
        comID = dev_data["Comm_ID"].asInt();

        Json::Value   MoveData = dev_data["Move"];
        int  moveposNum = MoveData["Points_Num"].asInt();

        Json::Value arrayObj = MoveData["Points_Array"];

        dispatch_x.clear();
        dispatch_y.clear();
        dispatch_theta.clear();

        dispatch_x.resize(moveposNum);
        dispatch_y.resize(moveposNum);
        dispatch_theta.resize(moveposNum);

        for ( int i = 0; i < arrayObj.size(); ++i )
        {
            int point_index = arrayObj[i]["ID"].asInt() - 1;
            dispatch_x[point_index] = arrayObj[i]["X"].asDouble();
            dispatch_y[point_index] = arrayObj[i]["Y"].asDouble();
            dispatch_theta[point_index] = arrayObj[i]["a"].asDouble();

            // dispatch_x.push_back( arrayObj[i]["X"].asDouble() );
            // dispatch_y.push_back( arrayObj[i]["Y"].asDouble() );
            // dispatch_theta.push_back( arrayObj[i]["a"].asDouble() );

            std::cout << "point_index: " << point_index << std::endl;
            std::cout << "dispatch_x: " << dispatch_x[i] << std::endl;
            std::cout << "dispatch_y: " << dispatch_y[i] << std::endl;
            std::cout << "dispatch_theta: " << dispatch_theta[i] << std::endl;
        }

        Json::Value  procData = dev_data["Process"];
        string  Proc_Name = procData["Proc_Name"].asString();
        float  Para1     = procData["Proc_Para_1"].asFloat();
        float  Para2     = procData["Proc_Para_2"].asFloat();
        float  Para3     = procData["Proc_Para_3"].asFloat();
        float  Para4     = procData["Proc_Para_4"].asFloat();
        float  Para5     = procData["Proc_Para_5"].asFloat();
        float  Para6     = procData["Proc_Para_6"].asFloat();
    }

    if ( !dispatch_x.empty() )
    {
        if ( state_machine_ == IDLE )
        {
            state_machine_ = PUB_TRAJECTORIE_ADD;
        }
        else
        {
            cout << "state_machine_ is not IDLE" << endl;
        }
    }
}

void Dispatch::DeserializedJsonTest()
{
    ROS_INFO("Begin to deserialize");
    // std::string strValue = \
    // "{\"Dev_Type\":\"Core\",\
    //   \"Dev_ID\":\"1\",\
    //   \"Comm_ID\":101,\
    //   \"Process\":\"PT_CAMERAGRAIN\",\
    //   \"Move\":[\
    //             {\"Points_Num\":10},\
    //             {\"Points_Array\":[\
    //                                {\"X\":1},\
    //                                {\"Y\":1.56},\
    //                                {\"a\":90.0}\
    //                               ]\
    //             }\
    //            ]\
    //  }";

    // std::string strValue = \
    // "{\"Dev_Type\":\"Core\",\
    //   \"Dev_ID\":\"1\",\
    //   \"Comm_ID\":101,\
    //   \"Process\":\"PT_CAMERAGRAIN\",\
    //   \"Move\":[\
    //             {\"Points_Num\":10},\
    //             {\"Points_Array\":[\
    //                                {\"X\":1.0,\"Y\":1.5,\"a\":0.0},\
    //                                {\"X\":4.0,\"Y\":-1.5,\"a\":0.0}\
    //                               ]\
    //             }\
    //            ]\
    //  }";
 std::string strValue;// = \
// {
//    "Dev_Data" :
//    {
//       "Comm_ID" : 1,
//       "Move" :
//       {
//          "Points_Array" :
//          [
//             {
//                "ID" : 1,
//                "X" : 1.450000762939453,
//                "Y" : 0.250,
//                "a" : 0.0
//             },
//             {
//                "ID" : 2,
//                "X" : 0.50,
//                "Y" : 0.250,
//                "a" : 206.5650482177734
//             }
//          ],
//          "Points_Num" : 2
//       },
//       "Process" :
//       {
//          "Proc_Name" : ""
//       },
//       "Tar_ID" : 1,
//       "Time_stamp" : ""
//    },
//    "Dev_ID" : 1,
//    "Dev_Type" : "Core"
// }

    Json::Reader reader;
    Json::Value json_object;

    cout << "----------1---------" << endl;
    if(reader.parse(strValue, json_object))
    {
        cout << "----------2---------" << endl;
        comID = json_object["Comm_ID"].asInt();
        std::cout << "Comm_ID: " << comID << std::endl;

        std::string action = json_object["Process"].asString();
        std::cout << "Process: " << action << std::endl;

        const Json::Value arrayObj = json_object["Move"]["Points_Array"];
        cout << "arrayObj.size(): " << arrayObj.size() << endl;

        dispatch_x.clear();
        dispatch_y.clear();
        dispatch_theta.clear();

        for ( int i = 0; i < arrayObj.size(); ++i )
        {
            dispatch_x.push_back( arrayObj[i]["X"].asDouble() );
            dispatch_y.push_back( arrayObj[i]["Y"].asDouble() );
            dispatch_theta.push_back( arrayObj[i]["a"].asDouble() );

            std::cout << "dispatch_x: " << dispatch_x[i] << std::endl;
            std::cout << "dispatch_y: " << dispatch_y[i] << std::endl;
            std::cout << "dispatch_theta: " << dispatch_theta[i] << std::endl;
        }
    }

    if ( !dispatch_x.empty() )
    {
        if ( state_machine_ == IDLE )
        {
            state_machine_ = PUB_TRAJECTORIE_ADD;
        }
        else
        {
            cout << "state_machine_ is not IDLE" << endl;
        }
    }
}

void Dispatch::ReceiveData(char* phead, int dSize)
{
    std::string infor;

    char* pStart = phead;
    char* pOver = NULL;
    char* pTail = NULL;

    for(int i =0; i < (dSize-1); i++)
    {
        char* p1 = (phead + i);
        char* p2 = (phead + i + 1);

        if( (0x0d == *p1) && (0x0a == *p2) )
        {
            pOver = p2;

            char  buf1[BUFFERSIZE];
            memset(buf1, 0, BUFFERSIZE);
            memcpy(buf1, pStart, (pOver - pStart - 1));

            std::string  tempStr = buf1;
            infor = m_lastPlus + tempStr;
            m_lastPlus.clear();
            m_InforList.push_back(infor);

            if(i < (dSize-2))
                pStart = pOver + 1;
        }
        else
        {
            //last char
            if( i == (dSize-2))
            {
                char  buf1[BUFFERSIZE];
                memset(buf1, 0, BUFFERSIZE);
                memcpy(buf1, pStart, ( (phead + dSize -1)  - pStart + 1));

                std::string  tempStr = buf1;
                m_lastPlus = tempStr;
            }
        }

        if( (0x00 == *p1) && (0x00 == *p2) )
        {
            pTail = p1;

            if( pStart != pTail )
            {
                char  buf1[BUFFERSIZE];
                memset(buf1, 0, BUFFERSIZE);
                memcpy(buf1, pStart, (pTail - pStart + 1));

                std::string  tempStr = buf1;
                m_lastPlus = tempStr;
            }

            break;
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dispatch_client_node");
    Dispatch dispatch;

    dispatch.Run();
    // dispatch.RunTest();
    return EXIT_SUCCESS;
}
