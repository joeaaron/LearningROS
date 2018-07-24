#include "barScanner.h"

RNG rng(12345);

ImageConverter::ImageConverter(ros::NodeHandle nh)
	: it(nh)
{
	
    //使用image_transport订阅图像话题“in” 和 发布图像话题“out” /camera/rgb/image_raw
    image_sub=it.subscribe("/usb_cam/image_raw",1,&ImageConverter::imageCb,this);
    image_pub=it.advertise("agv_qr",1);

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


void ImageConverter::ProcessFrame(cv_bridge::CvImagePtr cv_ptr)
{
	Mat img;
    img = cv_ptr->image;
    if (img.empty())
    {
      cout << "no image stream read!Please check the camera first.";
    }
    resize(img, img, Size(500,500));
    Mat src_gray;
    Mat src_thresh;
    Mat src_contour;
     
    Mat drawing = Mat::zeros(Size(800,600), CV_8UC3);
    Mat _drawing = Mat::zeros(Size(800,600), CV_8UC3);
 	vector<vector<Point> > contours, _contours;
    //pre-processing
    src_contour = img.clone();
    cvtColor(src_contour, src_gray, CV_BGR2GRAY);
    blur(src_gray, src_gray, Size(3, 3));
    threshold(src_gray, src_thresh, 100, 255, THRESH_OTSU);
 
    vector<Vec4i> hierarchy;
    findContours(src_thresh, contours, hierarchy, CV_RETR_TREE, CHAIN_APPROX_NONE, Point(0,0));
    //contour selected
    int c = 0, ic = 0;
    int parentIdx = -1;
    int area = 0;
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

    for(int i = 0; i < _contours.size(); i++)
      drawContours(drawing, _contours, i, CV_RGB(rng.uniform(100,255), rng.uniform(100,255), rng.uniform(100,255)), -1);
   
    //Get the central coordinates
    Point points[3];
    Point2f _points[4];

    for(int i = 0; i < _contours.size(); i++)
      points[i] = CalCenter(_contours, i);
    
    //Ellimination of interference points
   

    //draw the lines
    line(drawing, points[0], points[1], Scalar(rng.uniform(100,255), rng.uniform(100,255), rng.uniform(100,255)), 2);
    line(drawing, points[1], points[2], Scalar(rng.uniform(100,255), rng.uniform(100,255), rng.uniform(100,255)), 2);
    line(drawing, points[0], points[2], Scalar(rng.uniform(100,255), rng.uniform(100,255), rng.uniform(100,255)), 2);
    //imshow("result", drawing);
    //waitKey(1);
    GetPoints(drawing, _points);
    for (int i = 0; i < 4; ++i)
    	line(src_contour, _points[i % 4], _points[(i+1)%4], Scalar(20, 21, 237), 3);

    imshow("result", src_contour);
    waitKey(1);
}

//订阅回调函数
void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        //将ROS图像消息转化为适合Opencv的CvImage
        cv_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);

    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s",e.what());
        return;
    }
   
    ProcessFrame(cv_ptr);
    // printf("OK1\n");
    image_pub.publish(cv_ptr->toImageMsg());
}

void ImageConverter::GetPoints(Mat img, Point2f points[])
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