#include <dispatch_robot_comm/dispatch_robot_comm.h>

#define BUFFERSIZE              1024

#define RadianToAngle(X) ((X)*180.0/M_PI)
#define AngleToRadian(X) ((X)*M_PI/180.0)

Dispatch::Dispatch()
    :nh_()
    ,robot_pose_sub_()
    ,navigation_control_status_sub_()
    ,trajectories_add_pub_()
    ,trajectories_remove_pub_()
    ,navigation_control_pub_()
    ,trajectories_add_time_()
    ,state_machine_(IDLE)
    ,trajectorie_finished_(false)
    ,dispatch_comm_id_(-1)
    ,debug_(false)
    ,dispatch_trajectorie_base_name_("dispatch_trajectorie_name")
{
    robot_pose_sub_ = nh_.subscribe("/robot_pose", 1, &Dispatch::RobotPoseCallBack, this );
    navigation_control_status_sub_ = nh_.subscribe("/nav_ctrl_status", 3, &Dispatch::NavigationControlStatusCallback, this );

    trajectories_add_pub_ = nh_.advertise<yocs_msgs::Trajectory>("/trajectory_add",1);
    trajectories_remove_pub_ = nh_.advertise<yocs_msgs::Trajectory>("/trajectory_remove",1);
    navigation_control_pub_ = nh_.advertise<yocs_msgs::NavigationControl>("/nav_ctrl",1);

    nh_.param( "server_ip", server_ip_, (string)"192.168.1.104" );
    nh_.param( "server_port", server_port_, 10001 );
    nh_.param( "dev_id", dev_id_, 1 );
}

Dispatch::~Dispatch(){}

void Dispatch::Run()
{
    char buffer[BUFFERSIZE];
    bool connect_state = false;

    ros::Rate loop_rate(5);

    while (ros::ok())
    {
        ros::spinOnce();

        if ( connect_state == false )
        {
            connect_state = ConnectServer ( server_ip_, server_port_ );
            if ( connect_state )
            {
                cout << "socket_fd_: " << socket_fd_ << endl;
                //set timeout
                struct timeval timeout = {1, 0};
                //int ret_send = setsockopt( socket_fd_external_server, SOL_SOCKET, SO_SNDTIMEO, (const char*)&timeout, sizeof(timeout) );
                int ret_recv = setsockopt ( socket_fd_, SOL_SOCKET, SO_RCVTIMEO, ( const char* ) &timeout, sizeof ( timeout ) );
            }
            else
            {
                ROS_ERROR ( "setup connect to server failed! will retry in 1s" );
                close(socket_fd_);
                ros::Duration ( 1.0 ).sleep();
                continue;
            }
        }

        memset(buffer, 0, BUFFERSIZE);

        if (debug_)
            ROS_INFO("befor recv data");
        int recv_length = recv(socket_fd_, buffer, BUFFERSIZE, MSG_DONTWAIT);
        // int recv_length = read(socket_fd_, buffer, BUFFERSIZE);
//         cout << "recv_length: " << recv_length << endl;

        if (recv_length > 0)
        {
            ReceiveData(buffer, recv_length);
            string value(buffer);   //char[] --> string
            if (debug_)
            {
                ROS_INFO("after recv data");
                cout << "recv buffer is :" << buffer << endl;

            DeserializedJson(value);
        }
        else if ( recv_length == -1 )
        {
            // cout << "no recv data" << endl;
        }
        else if ( recv_length == 0 )
        {
            ROS_ERROR ( "disconnented from server! will retry in 1s" );
            connect_state = false;
            close(socket_fd_);
            ros::Duration ( 1.0 ).sleep();
            continue;
        }

        string msg_agv_to_dispatch;
        MsgAGVToDispatch("AGV", msg_agv_to_dispatch);

        int write_length = write(socket_fd_, msg_agv_to_dispatch.c_str(), msg_agv_to_dispatch.size());
        //cout << "send msg_agv_to_dispatch: " << msg_agv_to_dispatch << endl;

        if (write_length < 0)
        {
            ROS_ERROR("agv write to dispatch ERROR");
            connect_state = false;
            close(socket_fd_);
            ros::Duration ( 1.0 ).sleep();
            continue;
        }

        DoDispatchTask();

        loop_rate.sleep();
    }
    close ( socket_fd_ );
}

bool Dispatch::ConnectServer (string ip_str, int port )
{
    cout << "ip_: " << ip_str << " port: " << port << endl;

    struct sockaddr_in server_addr;

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
    trajectories_add_time_ = ros::Time::now();
    trajectories_add_pub_.publish(trajectory_list_msg_.trajectories[0]);
}

void Dispatch::NavigationControlPub()
{
    yocs_msgs::NavigationControl navigation_control_msg;

    navigation_control_msg.control = 1;
    navigation_control_msg.goal_name = trajectory_list_msg_.trajectories[0].name;

    navigation_control_pub_.publish(navigation_control_msg);
}

void Dispatch::TrajectorieRemovePub()
{
    yocs_msgs::Trajectory trajectory_msg;
    trajectory_msg.name = trajectory_list_msg_.trajectories[0].name;

    trajectories_remove_pub_.publish(trajectory_msg);

    trajectory_list_msg_.trajectories.erase(trajectory_list_msg_.trajectories.begin());
}

void Dispatch::NavigationControlStatusCallback(const yocs_msgs::NavigationControlStatusConstPtr &navigation_control_msg)
{
    if ( static_cast<int>(navigation_control_msg->status) == 3 )
    {
        cout << "goal ok" << endl;
        trajectorie_finished_ = true;
    }
}

void Dispatch::RobotPoseCallBack(const geometry_msgs::PoseConstPtr &robot_pose_msg)
{
    agv_x_ = robot_pose_msg->position.x;
    agv_y_ = robot_pose_msg->position.y;
    agv_theta_ = RadianToAngle(tf::getYaw(robot_pose_msg->orientation));
}

void Dispatch::MsgAGVToDispatch(string dev_type, string& msg_agv_to_dispatch)
{
    struct timeval time_value;
    gettimeofday(&time_value,NULL);

    Json::Value DeviceInfor;
    DeviceInfor["Dev_Type"] = dev_type;
    DeviceInfor["Dev_ID"] = dev_id_;

    Json::Value AGVInfor;

    if ( state_machine_ == IDLE && trajectory_list_msg_.trajectories.empty() )
    {
        AGVInfor["Comm_ID"] = dispatch_comm_id_;
    }
    else
    {
        string current_trajectorie_name = trajectory_list_msg_.trajectories[0].name;
        int index = current_trajectorie_name.find_first_not_of(dispatch_trajectorie_base_name_);
        AGVInfor["Comm_ID"] = boost::lexical_cast<int>(current_trajectorie_name.substr(index));
    }
    //cout << "send Comm_ID: " << AGVInfor["Comm_ID"] << endl;
    AGVInfor["Map_ID"] = 1;
    AGVInfor["X"] = agv_x_;
    AGVInfor["Y"] = agv_y_;
    AGVInfor["a"] = agv_theta_;
    AGVInfor["Cell_Quan"] = 0.98;
    AGVInfor["ERROR_Code"] = 1;
    AGVInfor["Pro_Type"] = "";
    AGVInfor["Pro_State"] = "";
    AGVInfor["Pro_Result"] = 1.0;
    AGVInfor["Time_stamp"] = static_cast<double>(time_value.tv_sec);

    DeviceInfor["Dev_Data"] = AGVInfor;

    msg_agv_to_dispatch = DeviceInfor.toStyledString();
    msg_agv_to_dispatch += 0x0d;
    msg_agv_to_dispatch += 0x0a;
}

void Dispatch::CheckDispatchTaskList()
{
    if ( trajectory_list_msg_.trajectories.size() != 0 )
    {
        state_machine_ = PUB_TRAJECTORIE_ADD;
    }
}

void Dispatch::DoDispatchTask()
{
    //cout << "state_machine_: " << state_machine_ << endl;
    if ( state_machine_ == IDLE )
    {
        CheckDispatchTaskList();
    }
    else if ( state_machine_ == PUB_TRAJECTORIE_ADD )
    {
        if (debug_)
        {
            ROS_INFO("state_machine_ = PUB_TRAJECTORIE_ADD");
            debug_ = false;
        }
        TrajectorieAddPub();
        state_machine_ = PUB_NAVIGATION_CONTROL;
    }
    else if ( state_machine_ == PUB_NAVIGATION_CONTROL )
    {
        if ( ros::Time::now() - trajectories_add_time_ > ros::Duration(1.0) )
        {
            NavigationControlPub();
            state_machine_ = WAIT_MOVE_FINISH;
        }
    }
    else if ( state_machine_ == WAIT_MOVE_FINISH )
    {
        if ( trajectorie_finished_ )
        {
            state_machine_ = PUB_TRAJECTORIE_REMOVE;
            trajectorie_finished_ = false;
        }
    }
    else if ( state_machine_ == PUB_TRAJECTORIE_REMOVE )
    {
        TrajectorieRemovePub();
        state_machine_ = IDLE;
//        cout << "trajectorie finished" << endl;
        debug_ = false;
        ROS_INFO("trajectorie finished");
    }
    else
    {
        cout << "undefined state_machine: " << state_machine_ << endl;
    }
}

void Dispatch::DeserializedJson(std::string strValue)
{
    Json::Reader reader;
    Json::Value json_object;

    if(reader.parse(strValue, json_object))
    {
        Json::Value  dev_data = json_object["Dev_Data"];

        if ( dispatch_comm_id_ == dev_data["Comm_ID"].asInt() )
        {
            return;
        }

        dispatch_comm_id_ = dev_data["Comm_ID"].asInt();

        Json::Value   MoveData = dev_data["Move"];
        int point_num = MoveData["Points_Num"].asInt();

        Json::Value  procData = dev_data["Process"];
        string  Proc_Name = procData["Proc_Name"].asString();
//        float  Para1     = procData["Proc_Para_1"].asFloat();
//        float  Para2     = procData["Proc_Para_2"].asFloat();
//        float  Para3     = procData["Proc_Para_3"].asFloat();
//        float  Para4     = procData["Proc_Para_4"].asFloat();
//        float  Para5     = procData["Proc_Para_5"].asFloat();
//        float  Para6     = procData["Proc_Para_6"].asFloat();

        Json::Value arrayObj = MoveData["Points_Array"];

        int points_array_size = arrayObj.size();

        yocs_msgs::Trajectory trajectory_msg;
        trajectory_msg.name = dispatch_trajectorie_base_name_ + boost::lexical_cast<string>(dispatch_comm_id_);

        if ( points_array_size > 0 )
        {
            trajectory_msg.waypoints.resize(points_array_size);
            for ( int i = 0; i < points_array_size; ++i )
            {
                trajectory_msg.waypoints[i].header.frame_id = "{\"close_enough\":\"0.0\",\"failure_mode\":\"LOOP\",\"frame_id\":\"map\",\"goal_timeout\":\"0\",\"mark\":\"false\",\"type\":\"goal\"}";
                trajectory_msg.waypoints[i].name = "dispatch_waypoints_name_" + boost::lexical_cast<string>(arrayObj[i]["ID"].asInt());
                trajectory_msg.waypoints[i].pose.position.x = arrayObj[i]["X"].asDouble();
                trajectory_msg.waypoints[i].pose.position.y = arrayObj[i]["Y"].asDouble();
                trajectory_msg.waypoints[i].pose.position.z = 0.0;
                tf::Quaternion q = tf::createQuaternionFromYaw( AngleToRadian(arrayObj[i]["a"].asDouble()) );
                trajectory_msg.waypoints[i].pose.orientation.x = q.getX();
                trajectory_msg.waypoints[i].pose.orientation.y = q.getY();
                trajectory_msg.waypoints[i].pose.orientation.z = q.getZ();
                trajectory_msg.waypoints[i].pose.orientation.w = q.getW();

                //trajectory_msg.waypoints[i].close_enough = 0.0;
                //trajectory_msg.waypoints[i].goal_timeout = 0.0;
                //trajectory_msg.waypoints[i].failure_mode = "";
            }

            trajectory_list_msg_.trajectories.push_back(trajectory_msg);
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
