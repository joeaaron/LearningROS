#ifndef QR_AGVQR_BARSCANNER_H_
#define QR_AGVQR_BARSCANNER_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <zbar.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <math.h>

using namespace std;
using namespace cv;
using namespace zbar;


class ImageConverter
{
public:
    ImageConverter(ros::NodeHandle nh,const string& calibFile, const string& saveFile);

    ~ImageConverter(){}

    //订阅回调函数
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
private:
	void ProcessFrame(cv_bridge::CvImagePtr cv_ptr);
	void GetPoints(Mat img, Point2f points[]);
    void QRDecode(Mat img);
    double GetDistance (CvPoint pointO,CvPoint pointA);
    void CheckCenter(vector<vector<Point> >c, vector<int>& index);
    void readParameters();
    int readCalibPara(string filename);
    void EstimatePosition(vector<Point2f> points);
    vector<float> rotationMatrixToEulerAngles(Mat& R, vector<float>& angle);
    void DrawArrow(cv::Mat& img, cv::Point pStart, cv::Point pEnd, int len, int alpha,
    Scalar& color, int thickness, int lineType);
    float GetAngelOfTwoVector(Point2f &pt1, Point2f &pt2, Point2f &c);
private:
    //ros::NodeHandle _nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;
    image_transport::Publisher image_pub;
    //tf
    tf::TransformBroadcaster broadcaster;
    tf::Transform transform;
    tf::Quaternion q;

    Size m_markerSize;
    vector<Point2f> m_markerCorners2d;  // marker's 4 corners projection
    vector<Point3f> m_markerCorners3d;
    Mat m_camMat;
    Mat m_distCoeff;
    double unit_x;
    double unit_y;

    string _calibFile;       //cameta calibration file
    ofstream sampleRead;     //save the results
    Scalar lineColor;
};

#endif
