#include "findMarker.h"

RNG rng(12345);

FindMarker::FindMarker(ros::NodeHandle nh)
    : it(nh),
    drawing(Mat::zeros(Size(800,600), CV_8UC3)),
    _drawing(Mat::zeros(Size(800,600), CV_8UC3))
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

    //使用image_transport订阅图像话题“in” 和 发布图像话题“out” /camera/rgb/image_raw
    image_sub=it.subscribe("/usb_cam/image_raw",1,&FindMarker::ProcessFrame,this);
    image_pub=it.advertise("qr_opencv",1);
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

void FindMarker::ProcessFrame(const sensor_msgs::ImageConstPtr& msg)
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
    FindMarkerContours(cv_ptr);
    image_pub.publish(cv_ptr->toImageMsg());
       
}

 
/***************** Mat转vector **********************/
template<typename _Tp>
vector<_Tp> convertMat2Vector(const Mat &mat)
{
  return (vector<_Tp>)(mat.reshape(1, 1));//通道数不变，按行转为一行
}


void FindMarker::EstimatePosition(Point2f* points)
{
    vector<Point2f> marker;
    for (size_t i=0; i < 4; i++)
      marker.push_back(points[i]);
  
    Mat Rvec;
    Mat raux, taux;
    //3d-2d
    solvePnP(m_markerCorners3d, marker, m_camMat, m_distCoeff, raux, taux);

    Rodrigues(raux, Rvec);
    vector<float> Tvec = convertMat2Vector<float>(taux);

    cv::Mat Extrinc(3, 4, Rvec.type()); // Extrinc is 4x4
    Extrinc( cv::Range(0,3), cv::Range(0,3) ) = Rvec * 1; // copies R into Extrinc
    Extrinc( cv::Range(0,3), cv::Range(3,4) ) = taux * 1; // copies tvec into Extrinc
    
    cout<<"R:"<<Rvec<<endl;
    cout<<"T:"<<taux <<endl;
    cout<<"Extrinc: "<<Extrinc << endl;

    angles.clear();
    rotationMatrixToEulerAngles(Rvec, angles);

    ///tf transform
    transform.setOrigin(tf::Vector3(Tvec[0], Tvec[1], Tvec[2]));
    q.setRPY(angles[0], angles[1], angles[2]);
    transform.setRotation(q);

    broadcaster.sendTransform(
      tf::StampedTransform(
        transform, 
        ros::Time::now(), "base_camera", "base_qr"));
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


void FindMarker::FindMarkerContours(cv_bridge::CvImagePtr cv_ptr)
{
    Mat img;
    img = cv_ptr->image;
    if (img.empty())
    {
      cout << "no image stream read!Please check the camera first.";
    }
    resize(img, img, Size(800,600));
    Mat src_gray;
    Mat src_thresh;
    Mat src_contour;
    
    //pre-processing
    src_contour = img.clone();
    cvtColor(img, src_gray, CV_BGR2GRAY);
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
    Point point[3];
    for(int i = 0; i < _contours.size(); i++)
      point[i] = CalCenter(_contours, i);
    

    area = contourArea(_contours[0]);
    int area_side = cvRound(sqrt(double(area)));
    for (int i = 0; i < 3; ++i)
    {
      line(_drawing, point[i], point[(i + 1)%3], CV_RGB(rng.uniform(100,255), rng.uniform(100,255), rng.uniform(100,255)), area_side / 2, 8);
    } 

    if(!IsCorrect(point))
    {
      int start = 0;
    
      for (int i = 0; i < 3; i++)
      {
          double k1, k2,kk;
          k1 = (point[i].y - point[(i + 1) % 3].y) / (point[i].x - point[(i + 1) % 3].x);
          k2 = (point[i].y - point[(i + 2) % 3].y) / (point[i].x - point[(i + 2) % 3].x);
          kk = k1*k2;
          if (k1*k2 <0)
               start = i;
       }
       double ax, ay, bx, by;
       ax = point[(start + 1) % 3].x;
       ay = point[(start + 1) % 3].y;
       bx = point[(start + 2) % 3].x;
       by = point[(start + 2) % 3].y;
       Point2f center(abs(ax - bx) / 2, abs(ay -by)/ 2);
       double dy = ay - by;
       double dx = ax - bx;
       double k3 = dy / dx;
       angle =atan(k3) * 180 / CV_PI;//转化角度
       rot_mat = getRotationMatrix2D(center, angle, 1.0);
       
       warpAffine(src_contour, dst, rot_mat, src_contour.size(), 1, 0, 0);//旋转原图查看
       warpAffine(_drawing, _drawing, rot_mat, src_contour.size(), 1, 0, 0);//旋转连线图
       warpAffine(src_contour, src_contour, rot_mat, src_contour.size(), 1, 0, 0);//旋转原图

       imshow("Dst", dst);
    }

    //draw the lines
    //line(drawing, points[0], points[1], Scalar(rng.uniform(100,255), rng.uniform(100,255), rng.uniform(100,255)), 2);
    //line(drawing, points[1], points[2], Scalar(rng.uniform(100,255), rng.uniform(100,255), rng.uniform(100,255)), 2);
    //line(drawing, points[0], points[2], Scalar(rng.uniform(100,255), rng.uniform(100,255), rng.uniform(100,255)), 2);
   // imshow("result", drawing);
   // waitKey(0);
    Point2f _points[4];
    GetPoints(drawing, _points);
    
    int minx = 0, miny = 0, maxx = 100000, maxy = 100000;
    for(int i = 0; i < 4; i++)
    {
      if (maxx < _points[i].x) maxx = _points[i].x;
      if (maxy < _points[i].y) maxy = _points[i].y;
      if (minx > _points[i].x) minx = _points[i].x;
      if (miny > _points[i].y) miny = _points[i].y;
      line(src_contour, _points[i%4], _points[(i+1)%4], Scalar(20, 21, 237), 3);
    }
    //line(img, _points[i%4], _points[(i+1)%4], Scalar(20, 21, 237), 3);
    
    imshow("result", src_contour);
    waitKey(1);
    
    int set_inter = 5;
    while(true)
    {
      minx -= set_inter;
      miny -= set_inter;
      maxx += set_inter;
      maxy += set_inter;
      if (maxx > img.size().width || maxy > img.size().height || minx < 0 || miny < 0)
      {
         minx += set_inter;
         miny += set_inter;
         maxx -= set_inter;
         maxy -= set_inter;
         set_inter--;
      }
      else
      {
         break;
      }
    }

    Mat fout = src_contour(Rect(minx, miny, maxx - minx, maxy - miny)); //ROI

    imshow("tmp", fout);
    waitKey();

    QRDecode(fout);

    //EstimatePosition(_points);

}

void FindMarker::QRDecode(Mat img)
{
    Mat img_gray;
    //Define a scanner
    ImageScanner scanner;
    scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);

    cvtColor(img, img_gray, CV_BGR2GRAY);

    int width = img_gray.cols;
    int height = img_gray.rows;
    uchar *raw = (uchar*)(img_gray.data);

    //Wrap image data
    Image image(width, height, "Y800", raw, width*height);

    //scan the image for barcodes
    scanner.scan(image);
    //Extract results
    for(Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++ symbol)
    {
      if(image.symbol_begin() == image.symbol_end())
        cout << "Failed to check qr code.Please ensure the image is right!";

      else
      {
        cout << "type:" << endl << symbol->get_type_name() << endl << endl;
        cout << "bar code:" << endl << symbol->get_data() << endl << endl;
      }
    }

    waitKey();

    image.set_data(NULL, 0);
}

bool FindMarker::IsCorrect(Point point[])
{
  for (int i = 0; i < 3; ++i)
  {
    if(point[i].x == point[(i + 1) % 3].x && point[i].y == point[(i + 2) % 3].y);
      return true;
    if(point[i].y == point[(i + 1) % 3].y && point[i].x == point[(i + 2) % 3].x);
      return true;
  }

  return false;
}