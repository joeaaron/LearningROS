#include "quad_navig/quadScanner.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "quad_navig");
    ros::NodeHandle nh("~");
    QuadScanner quadScanner(nh);


    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
