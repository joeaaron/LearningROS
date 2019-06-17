#include "pallet_tf/pallet_tf.h"
#include <dynamic_reconfigure/server.h>
#include "pallet_tf/pallet_cutConfig.h"
Parameter param;

void paramCallBack(pallet_cut::pallet_cutConfig &config, uint32_t level)
{
   param.DISTANCE           = config.DISTANCE;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pallet_tf");
    ros::NodeHandle nh("~");
    ImageConverter imgConverter(nh, argv[1]);
    
    /*************************** dynamic_reconfigure ***********************************/
	dynamic_reconfigure::Server<pallet_cut::pallet_cutConfig> srv;
	dynamic_reconfigure::Server<pallet_cut::pallet_cutConfig>::CallbackType f;
	f = boost::bind(&paramCallBack, _1, _2);
	srv.setCallback(f);

    double rate;
    nh.getParam("rosRate", rate);
    ros::Rate loop_rate(rate);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
        imgConverter.cfgCallback(param);
    }
    return 0;
}