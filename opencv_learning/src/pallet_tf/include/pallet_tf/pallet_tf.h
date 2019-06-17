#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Header.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <iostream>
#include <fstream>
#include <iomanip>
#include <math.h>

using namespace std;
using namespace cv;

struct Parameter
{
    double DISTANCE;
};

class ImageConverter
{
public:
    ImageConverter(ros::NodeHandle nh,const string& calibFile);
    ~ImageConverter(){}

    //订阅回调函数
    void colorCb(const sensor_msgs::ImageConstPtr& color_msg);
    void depthCb(const sensor_msgs::ImageConstPtr& depth_msg);
    /*!
	* dynamic param callback function
	*/
	void cfgCallback(const Parameter cfg);
  
private:
	void ProcessFrame(cv_bridge::CvImagePtr cv_ptr);
    void readParameters(ros::NodeHandle nh_);
    int readCalibPara(string filename);
    cv::Mat getNearestPart(cv::Mat img);
    void nearestFiltering(cv::Mat& img);
    vector<vector<cv::Point> > getContours(cv::Mat img, vector<vector<cv::Point> >& _contours);
    bool isAreaQualified(cv::Mat& img, vector<vector<cv::Point> > contours, vector<cv::Point>& center_points);
    void drawArrow(cv::Mat& img, cv::Point pStart, cv::Point pEnd, int len, int alpha,
	cv::Scalar& color, int thickness, int lineType);
private:
    //ros::NodeHandle _nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber color_sub;
    image_transport::Subscriber depth_sub;
    image_transport::Publisher color_pub;
    image_transport::Publisher depth_pub;
    ros::Subscriber task_switch_sub_;
    // dynamic config parameters
   	Parameter _cfg;
    int kernel;
    double minratio, maxratio;
    //tf
    tf::TransformBroadcaster broadcaster;
    tf::Transform transform;
    tf::Quaternion q;

    Mat m_camMat;
    Mat m_distCoeff;
    double unit_x;
    double unit_y;

    string _calibFile;       //cameta calibration file
    Scalar lineColor;
};