#ifndef AGV_QUADNAVIG_QUADSCANNER_H_
#define AGV_QUADNAVIG_QUADSCANNER_H__

#include <iostream>
#include <algorithm>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

class QuadScanner
{
public:
    QuadScanner(ros::NodeHandle nh);
    ~QuadScanner(){}
	void imageCb(const sensor_msgs::ImageConstPtr& msg);
private:
	void QuadDetect(cv_bridge::CvImagePtr cv_ptr);
	void FindCandidate(Mat img, Mat& drawing);
	void LineDetection(Mat img, std::vector<Vec4i>& reducedLines);
	void IsQuad(Mat img, std::vector<Vec4i> lines, bool& flag, vector<Point2f>& crossPoints);


private:
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;
    image_transport::Publisher image_pub;
};

#endif
