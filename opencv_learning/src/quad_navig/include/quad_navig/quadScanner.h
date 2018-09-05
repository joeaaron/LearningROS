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
    QuadScanner(ros::NodeHandle nh, const string& calibFile);
    ~QuadScanner(){}
	void imageCb(const sensor_msgs::ImageConstPtr& msg);
	int ReadCalibPara(string filename);
private:
	void QuadDetect(cv_bridge::CvImagePtr cv_ptr);
	void FindCandidate(Mat img, Mat& drawing);
	void LineDetection(Mat img, std::vector<Vec4i>& reducedLines);
	void IsQuad(Mat img, std::vector<Vec4i> lines, bool& flag, vector<Point2f>& crossPoints);
    void GetQuadCamTf(Mat img, vector<Point2f> crossPoints);
	void DrawArrow(cv::Mat& img, cv::Point pStart, cv::Point pEnd, int len, int alpha,
     cv::Scalar& color, int thickness, int lineType);
	void GetTfTrans(double x,double y, double theta);

private:
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;
    image_transport::Publisher image_pub;

   	double unit_x;
    double unit_y;
};

#endif
