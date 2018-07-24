#ifndef QR_LOC_FINDMARKER_H
#define QR_LOC_FINDMARKER_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

class FindMarker
{
public:
    FindMarker(void);
    void ProcessFrame(Mat& img);
	
private:
    void FindMarkerContours(Mat& img, vector<vector<Point> > contours,  vector<vector<Point> >& _contours);
    void GetPoints(Mat img, Point2f* points);
    void EstimatePosition(Point2f* points);
    vector<float> rotationMatrixToEulerAngles(Mat& R, vector<float>& angle);
private:
    Size m_markerSize;
    vector<Point2f> m_markerCorners2d;	// marker's 4 corners projection
    vector<Point3f> m_markerCorners3d;	
    Mat m_camMat;
    Mat m_distCoeff;
    vector<float> angles;
};

#endif