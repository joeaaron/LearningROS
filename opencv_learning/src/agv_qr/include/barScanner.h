#ifndef QR_AGVQR_BARSCANNER_H_
#define QR_AGVQR_BARSCANNER_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <zbar.h>
#include <iostream>
#include <iomanip>

using namespace std;
using namespace cv;
using namespace zbar;


class ImageConverter
{
public:
    ImageConverter(ros::NodeHandle nh);
 
    ~ImageConverter(){}
 
    //订阅回调函数
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
private:
	void ProcessFrame(cv_bridge::CvImagePtr cv_ptr);
	void GetPoints(Mat img, Point2f points[]);
    void QRDecode(Mat img);
private:
    //ros::NodeHandle _nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;
    image_transport::Publisher image_pub;
};

#endif