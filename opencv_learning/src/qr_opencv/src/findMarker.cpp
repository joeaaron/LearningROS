#include "findMarker.h"

RNG rng(12345);

FindMarker::FindMarker(void)
{
    m_camMat = (Mat_<float>(3,3) << 0,0,0,
  				0,0,0,
  				0,0,0);
    m_distCoeff = (Mat_<float>(4,1) << 0,0,0,0);

    m_markerSize = Size(100, 100);
    // marker default size : 100 * 100; markercorner : 100 * 100 rectangle
    m_markerCorners2d.push_back(Point2f(0, 0));
    m_markerCorners2d.push_back(Point2f(m_markerSize.width-1, 0));
    m_markerCorners2d.push_back(Point2f(m_markerSize.width-1, m_markerSize.height-1));
    m_markerCorners2d.push_back(Point2f(0, m_markerSize.height-1));

    // 3d corner coordinate ---clockwise
    m_markerCorners3d.push_back(cv::Point3f(-0.5f,-0.5f,0));
    m_markerCorners3d.push_back(cv::Point3f(+0.5f,-0.5f,0));
    m_markerCorners3d.push_back(cv::Point3f(+0.5f,+0.5f,0));
    m_markerCorners3d.push_back(cv::Point3f(-0.5f,+0.5f,0));
}

inline Point CalCenter(vector<vector<Point> > contours, int i)
{
    int center_x = 0;
    int center_y = 0;
    int n = contours[i].size();
    center_x = (contours[i][n/4].x + contours[i][n*2/4].x + contours[i][3*n/4].x + contours[i][n-1].x)/4;
    center_y = (contours[i][n/4].y + contours[i][n*2/4].y + contours[i][3*n/4].y + contours[i][n-1].y)/4;
    Point point = Point(center_x, center_y);
    return point;
}

void FindMarker::ProcessFrame(Mat& img)
{
    resize(img, img,  Size(800,600));
    Mat src_gray;
    Mat src_thresh;
    Mat src_contour;
    Mat drawing = Mat::zeros(Size(800,600), CV_8UC3);
    
    vector<vector<Point> > contours, _contours;
    //pre-processing
    src_contour = img.clone();
    cvtColor(img, src_gray, CV_BGR2GRAY);
    blur(src_gray, src_gray, Size(3, 3));
    threshold(src_gray, src_thresh, 100, 255, THRESH_OTSU);
  
    FindMarkerContours(src_thresh, contours, _contours);
    for(int i = 0; i < _contours.size(); i++)
      drawContours(drawing, _contours, i, CV_RGB(rng.uniform(100,255), rng.uniform(100,255), rng.uniform(100,255)), -1);

    
    Point points[3];
    Point2f _points[4];
    for(int i = 0; i < _contours.size(); i++)
      points[i] = CalCenter(_contours, i);
    
   
    //draw the lines
    line(drawing, points[0], points[1], Scalar(rng.uniform(100,255), rng.uniform(100,255), rng.uniform(100,255)), 2);
    line(drawing, points[1], points[2], Scalar(rng.uniform(100,255), rng.uniform(100,255), rng.uniform(100,255)), 2);
    line(drawing, points[0], points[2], Scalar(rng.uniform(100,255), rng.uniform(100,255), rng.uniform(100,255)), 2);
   // imshow("result", drawing);
   // waitKey(0);
    
    GetPoints(drawing, _points);
    
    for(int i = 0; i < 4; i++)
      line(img, _points[i%4], _points[(i+1)%4], Scalar(20, 21, 237), 3);
    
    imshow("result", img);
    waitKey(1);
    
    //EstimatePosition(_points);
}

void FindMarker::EstimatePosition(Point2f* points)
{
    vector<Point2f> marker;
    for (size_t i=0; i < 4; i++)
      marker.push_back(points[i]);
  
    Mat Rvec;
    Vec3f Tvec;
    Mat raux, taux;
    //3d-2d
    solvePnP(m_markerCorners3d, marker, m_camMat, m_distCoeff, raux, taux);

    Rodrigues(raux, Rvec);
    cv::Mat Extrinc(3, 4, Rvec.type()); // Extrinc is 4x4
    Extrinc( cv::Range(0,3), cv::Range(0,3) ) = Rvec * 1; // copies R into Extrinc
    Extrinc( cv::Range(0,3), cv::Range(3,4) ) = taux * 1; // copies tvec into Extrinc
    
    cout<<"R:"<<Rvec<<endl;
    cout<<"t:"<<taux <<endl;
    cout<<"Extrinc: "<<Extrinc << endl;

    angles.clear();
    rotationMatrixToEulerAngles(Rvec, angles);
  
}

inline bool isRotationMatrix(Mat& R)
{
  	cv::Mat Rt;
  	transpose(R,Rt);
  	Mat shouldBeIdentity = Rt * R;
  	Mat I = Mat::eye(3, 3, shouldBeIdentity.type());

  	return norm(I, shouldBeIdentity) < 1e-6;
}

vector<float> FindMarker::rotationMatrixToEulerAngles(Mat& R, vector<float>& angle)
{
  	angle.clear();
   
  	assert(isRotationMatrix(R));
      
  	float sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));
  	bool singular = sy < 1e-6;
  	float x, y, z;
  	float pitch, roll, yaw;

  	if (!singular)
  	{
  		x = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
  		y = atan2(-R.at<double>(2, 0), sy);
  		z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
  	}
  	else
  	{
  		x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
  		y = atan2(-R.at<double>(2, 0), sy);
  		z = 0;
  	}

  	roll = y / CV_PI * 180;  //rad to angle
  	pitch = x / CV_PI * 180;
  	yaw = z / CV_PI * 180;
      
    angle.push_back(x);
    angle.push_back(y);
    angle.push_back(z);

    return angle;
}

void FindMarker::GetPoints(Mat img, Point2f* points)
{
    Mat img_gray;
    Mat img_thresh;
    vector<vector<Point> > img_contour;
    vector<Vec4i> img_hierarchy;
    cvtColor(img, img_gray, CV_BGR2GRAY);
    threshold(img_gray, img_thresh, 45, 255, THRESH_BINARY);
    
    findContours(img_thresh, img_contour, img_hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    RotatedRect rectPoint = minAreaRect(img_contour[0]);
    //Point2f points[4];
    
    rectPoint.points(points);
    
}


void FindMarker::FindMarkerContours(Mat& img, vector<vector<Point> > contours,  vector<vector<Point> >& _contours)
{
    vector<Vec4i> hierarchy;
   
    findContours(img, contours, hierarchy, CV_RETR_TREE, CHAIN_APPROX_NONE, Point(0,0));
    //contour selected
    int c = 0, ic = 0;
    int parentIdx = -1;
    for(int i = 0;i < contours.size(); i++)
    {
      if(hierarchy[i][2] != -1 && ic == 0)
      {
    		parentIdx = i;
    		ic++;
      }
      else if(hierarchy[i][2] != -1 )
		    ic++;
      
      //最外面清零
      else if(hierarchy[i][2] == -1 )
      {
    		ic = 0;
    		parentIdx = -1;
      }
      // 找到定位点信息
      if(ic >= 2)
      {
    		_contours.push_back(contours[parentIdx]);
    		ic = 0;
    		parentIdx = -1;
    		//area = contourArea(contours[i]);
      }
    }

}