#include "barScanner.h"
#include "qr_navigation/qrMsg.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "qr_navig");
    ros::NodeHandle nh("~");
    ImageConverter imgConverter(nh, argv[1], argv[2]);


    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
