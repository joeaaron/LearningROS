#include "zbarScanner.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "zbar_opencv");
	ros::NodeHandle nh("~");
	ImageConverter imgConverter(nh);

	ros::spin();
	return 0;
}
