#include "barScanner.h"
#include "agv_qr/qrMsg.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "agv_qr");
    ros::NodeHandle nh("~");
    ImageConverter imgConverter(nh, argv[1], argv[2]);

    ros::Publisher chatter_pub = nh.advertise<agv_qr::qrMsg>("robotID",1000);


    ros::Rate loop_rate(11);

    int32_t count = 0;

    while (ros::ok())
    {
        /**
         * This is a message object. You stuff it with data, and then publish it.
         */
        //std_msgs::String msg;

        agv_qr::qrMsg msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "/robot";
        msg.id = count;
        msg.name = "Robot";

        //std::stringstream ss;
        //ss << "hello world " << count;
        //msg.data = ss.str();

        ROS_INFO("I am %s %d", msg.name.c_str(), msg.id);

        /**
         * The publish() function is how you send messages. The parameter
         * is the message object. The type of this object must agree with the type
         * given as a template parameter to the advertise<>() call, as was done
         * in the constructor above.
         */
        chatter_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }
    return 0;
}
