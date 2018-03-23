#include <message_filters/subscriber.h>  
#include <message_filters/time_synchronizer.h> 
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <dynamic_reconfigure/server.h>
#include "camlasercalib/laser_cutConfig.h"

#include "camLaserCalib.h"

Parameter param;

using namespace pcl;
using namespace sensor_msgs;
//using namespace message_filters;

void paramCallBack(laser_cut::laser_cutConfig &config, uint32_t level)
{
   param.x_min           = config.x_min;
   param.x_max           = config.x_max;
   param.y_min           = config.y_min;
   param.y_max           = config.y_max;
   param.z_min           = config.z_min;
   param.z_max           = config.z_max;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "cam_lidar_calib");
	if(argc != 3)
	{
		cerr<<endl<<"Usage: rosrun cam_laser_calib cam_laser_calib_node path_to_pointsFile path_to_calibFile";
        ros::shutdown();
        return 1;
    }
    ros::NodeHandle nh("~");
	calib::CamLaserCalib CamLaserCalib(nh, argv[1], argv[2]);

	/*************************** message_filter ***********************************/
	message_filters::Subscriber<PointCloud2> lidar16(nh,CamLaserCalib.strSub_pc2.c_str() , 1);  //topic 1
	message_filters::Subscriber<Image> img(nh,CamLaserCalib.strSub_img.c_str() , 1);		//topic 2
	typedef message_filters::sync_policies::ApproximateTime<Image,PointCloud2> MySyncPolicy;
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), img, lidar16);   //syn
	sync.registerCallback(boost::bind(&calib::CamLaserCalib::img_pc2_Callback, &CamLaserCalib, _1, _2));						 //callback
	/*************************** dynamic_reconfigure ***********************************/
	dynamic_reconfigure::Server<laser_cut::laser_cutConfig> srv;
	dynamic_reconfigure::Server<laser_cut::laser_cutConfig>::CallbackType f;
	f = boost::bind(&paramCallBack, _1, _2);
	srv.setCallback(f);
	ros::Rate loop(5);		//5 HZ
	while (ros::ok())
	{
		ros::spinOnce();
		CamLaserCalib.cfgCallback(param);
	}
	
	//ros::Subscriber sub = nh.subscribe("/rslidar_points",1, pointCloudCallBack);

	return 0;
}
