#include "barScanner.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "agv_qr");
	ros::NodeHandle nh("~");
	ImageConverter imgConverter(nh, argv[1]);

	ros::spin();
	return 0;
}
