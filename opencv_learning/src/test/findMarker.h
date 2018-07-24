#ifndef QR_QROPENCV_FINDMARKER_H_
#define QR_QROPENCV_FINDMARKER_H_

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>

#include <zbar.h>

#include <iostream>
#include <vector>
#include <iomanip>

using namespace cv;
using namespace std;
using namespace zbar;

class FindMarker
{
public:
    FindMarker(ros::NodeHandle nh);
 
 
    ~FindMarker(){}
 
    //订阅回调函数
    void ProcessFrame(const sensor_msgs::ImageConstPtr& msg);

private:
    void FindMarkerContours(cv_bridge::CvImagePtr cv_ptr);
    void GetPoints(Mat img, Point2f* points);
    void EstimatePosition(Point2f* points);
    vector<float> rotationMatrixToEulerAngles(Mat& R, vector<float>& angle);
    bool IsCorrect(Point point[]);
    void QRDecode(Mat img);
private:
    Size m_markerSize;
    vector<Point2f> m_markerCorners2d;	// marker's 4 corners projection
    vector<Point3f> m_markerCorners3d;	
    Mat m_camMat;
    Mat m_distCoeff;
    vector<float> angles;
    vector<vector<Point> > contours, _contours;
    Mat drawing, _drawing;

    //image_transport
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;
    image_transport::Publisher image_pub;

    //tf
    tf::TransformBroadcaster broadcaster;
    tf::Transform transform;
    tf::Quaternion q;
};

#endif