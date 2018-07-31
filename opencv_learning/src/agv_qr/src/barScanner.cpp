#include "barScanner.h"

RNG rng(12345);
#define CAMERAMAT "CameraMat"
#define DISTCOEFF "DistCoeff"

ImageConverter::ImageConverter(ros::NodeHandle nh,const string& calibFile)
	: it(nh),
	_calibFile(calibFile),
    lineColor(255, 255, 255)
{
	readParameters();  //when it should be used.
    /*
    m_camMat = (Mat_<float>(3, 3) << 0, 0, 0,
                                    0, 0, 0,
                                    0, 0, 0);
    m_distCoeff = (Mat_<float>(4, 1) << 0, 0, 0, 0);
    */
    m_markerSize = Size(100, 100);
    // marker default size : 100 * 100; markercorner : 100 * 100 rectangle
    m_markerCorners2d.push_back(Point2f(0, 0));
    m_markerCorners2d.push_back(Point2f(m_markerSize.width-1, 0));
    m_markerCorners2d.push_back(Point2f(m_markerSize.width-1, m_markerSize.height-1));
    m_markerCorners2d.push_back(Point2f(0, m_markerSize.height-1));

    // 3d corner coordinate ---anticlockwise
    m_markerCorners3d.push_back(cv::Point3f(-0.5f,-0.5f,0));   //三维坐标的单位是毫米
    m_markerCorners3d.push_back(cv::Point3f(-0.5f,+0.5f,0));
    m_markerCorners3d.push_back(cv::Point3f(+0.5f,+0.5f,0));
    m_markerCorners3d.push_back(cv::Point3f(+0.5f,-0.5f,0));

    //使用image_transport订阅图像话题“in” 和 发布图像话题“out” /camera/rgb/image_raw
    image_sub=it.subscribe("/usb_cam/image_raw",1,&ImageConverter::imageCb,this);
    image_pub=it.advertise("agv_qr",1);

}

void ImageConverter::readParameters()
{
    readCalibPara(_calibFile.c_str());
    //it.param("image_sub", image_sub, string("/usb_cam/image_raw"));      //topic name
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

double ImageConverter::GetDistance (CvPoint pointO,CvPoint pointA )
{
    double distance;
    distance = powf((pointO.x - pointA.x),2) + powf((pointO.y - pointA.y),2);
    distance = sqrtf(distance);
 
    return distance;
}
 
void ImageConverter::CheckCenter(vector<vector<Point> >c, vector<int>& index)
{
    float dmin1=10000;
    float dmin2=10000;
    for(int i=0;i<c.size();i++)
    {
        RotatedRect rect_i = minAreaRect(c[i]);
        for(int j=i+1;j<c.size();j++)
        {
            RotatedRect rect_j = minAreaRect(c[j]);
            float d=GetDistance(rect_i.center,rect_j.center);
            if(d<dmin2 && d>10)
            {
                if(d<dmin1 && d>10)
                {
                    dmin2=dmin1;
                    dmin1=d;
                    index[2]=index[0];
                    index[3]=index[1];
                    index[0]=i;
                    index[1]=j;
 
                }
                else
                {
                   dmin2=d;
                   index[2]=i;
                   index[3]=j;
                }
            }
        }
    }
}

inline bool isRotationMatrix(Mat& R)
{
  	cv::Mat Rt;
  	transpose(R,Rt);
  	Mat shouldBeIdentity = Rt * R;
  	Mat I = Mat::eye(3, 3, shouldBeIdentity.type());

  	return norm(I, shouldBeIdentity) < 1e-6;
}

vector<float> ImageConverter::rotationMatrixToEulerAngles(Mat& R, vector<float>& angle)
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

/***************** Mat转vector **********************/
template<typename _Tp>
vector<_Tp> convertMat2Vector(const Mat &mat)
{
  return (vector<_Tp>)(mat.reshape(1, 1));//通道数不变，按行转为一行
}

void ImageConverter::EstimatePosition(vector<Point2f> points)
{  
    Mat Rvec;
    Mat raux, taux;
    //3d-2d---Sequential correspondence
    solvePnP(m_markerCorners3d, points, m_camMat, m_distCoeff, raux, taux);

    Rodrigues(raux, Rvec);
    vector<float> Tvec = convertMat2Vector<float>(taux);

    cv::Mat Extrinc(3, 4, Rvec.type()); // Extrinc is 4x4
    Extrinc( cv::Range(0,3), cv::Range(0,3) ) = Rvec * 1; // copies R into Extrinc
    Extrinc( cv::Range(0,3), cv::Range(3,4) ) = taux * 1; // copies tvec into Extrinc
    
    //cout<<"R:"<<Rvec<<endl;
    //cout<<"T:"<<taux <<endl;
    //cout<<"Extrinc: "<<Extrinc << endl;
    /*
    string resultName = "result.yml";
    FileStorage calibrate(resultName, FileStorage::WRITE);
    calibrate<< "CameraExtrinsicMat" << Extrinc;
    calibrate.release();
    */

    vector<float> angles;
    rotationMatrixToEulerAngles(Rvec, angles);

    ///tf transform broadcast
    transform.setOrigin(tf::Vector3(Tvec[0], Tvec[1], Tvec[2]));
    q.setRPY(angles[0], angles[1], angles[2]);
    transform.setRotation(q);

    broadcaster.sendTransform(
      tf::StampedTransform(
        transform, 
        ros::Time::now(), "base_camera", "base_qr"));
}

int ImageConverter::readCalibPara(string filename)
{
    cv::FileStorage fs(filename,cv::FileStorage::READ);
    if(!fs.isOpened())
    {
        std::cout<<"Invalid calibration filename."<<std::endl;
        return 0;
    }
    fs[CAMERAMAT]>>m_camMat;
    fs[DISTCOEFF]>>m_distCoeff;

    double unit_x = m_camMat.at<double>(0, 0);
    double unit_y = m_camMat.at<double>(1, 1);
    cout <<  "dx= " << unit_x << endl << "dy= "<< unit_y<<endl;
}

void ImageConverter::ProcessFrame(cv_bridge::CvImagePtr cv_ptr)
{
	Mat img;
    img = cv_ptr->image;
    if (img.empty())
    {
      cout << "no image stream read!Please check the camera first.";
    }
    
    //img coordinate
    lineColor = Scalar(0, 255, 0); 
    DrawArrow(img, Point(img.cols/2, img.rows/2), Point(img.cols/2, img.rows/2 - 200), 25, 30, lineColor, 2, CV_AA); 

    QRDecode(img);
   
}

float ImageConverter::GetAngelOfTwoVector(Point2f &pt1, Point2f &pt2, Point2f &c)
{
    float theta = atan2(pt1.x - c.x, pt1.y - c.y) - atan2(pt2.x - c.x, pt2.y - c.y);
    if (theta > CV_PI)
        theta -= 2 * CV_PI;
    if (theta < -CV_PI)
        theta += 2 * CV_PI;
 
    theta = theta * 180.0 / CV_PI;
    return theta;
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

void ImageConverter::DrawArrow(cv::Mat& img, cv::Point pStart, cv::Point pEnd, int len, int alpha,             
     cv::Scalar& color, int thickness, int lineType)
{    
    const double PI = 3.1415926;    
    Point arrow;    
    //计算 θ 角（最简单的一种情况在下面图示中已经展示，关键在于 atan2 函数，详情见下面）   
    double angle = atan2((double)(pStart.y - pEnd.y), (double)(pStart.x - pEnd.x));  
    line(img, pStart, pEnd, color, thickness, lineType);   
    //计算箭角边的另一端的端点位置（上面的还是下面的要看箭头的指向，也就是pStart和pEnd的位置） 
    arrow.x = pEnd.x + len * cos(angle + PI * alpha / 180);     
    arrow.y = pEnd.y + len * sin(angle + PI * alpha / 180);  
    line(img, pEnd, arrow, color, thickness, lineType);   
    arrow.x = pEnd.x + len * cos(angle - PI * alpha / 180);     
    arrow.y = pEnd.y + len * sin(angle - PI * alpha / 180);    
    line(img, pEnd, arrow, color, thickness, lineType);
}

void ImageConverter::QRDecode(Mat img)
{
    Mat frame, img_gray;
    //Define a scanner
    ImageScanner scanner;
    scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);

    frame = img.clone();
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
        cout << "Faisled to check qr code.Please ensure the image is right!";

      else
      {
        cout << "type:" << symbol->get_type_name() << endl;
        cout << "decoded:" << symbol->get_data() << endl << endl;
      }
	  
	  vector<Point> vp;
      vector<Point2f> pointBuf;
	  int n = symbol->get_location_size();
	  for (int i = 0; i < n; i++)
	  {
	     vp.push_back(Point(symbol->get_location_x(i),symbol->get_location_y(i)));
         pointBuf.push_back(vp[i]);
	  }    
      //cornerSubPix(img, pointBuf, Size(5, 5), Size(-1, -1));
	  //EstimatePosition(pointBuf);
	  
	  for (vector<Point2f>::iterator it = pointBuf.begin(); it != pointBuf.end(); it++)
	  {
	  	 //cout << "Points:" << *it << endl;
         circle(img, *it, 3, Scalar(255, 0, 0), -1, 8);
	  }
	  
      // Draw location of the symbols found
      if (symbol->get_location_size() == 4)
	  {
		  line(img, Point(symbol->get_location_x(0), symbol->get_location_y(0)), Point(symbol->get_location_x(1), symbol->get_location_y(1)), Scalar(0, 255, 0), 2, 8, 0);
		  line(img, Point(symbol->get_location_x(1), symbol->get_location_y(1)), Point(symbol->get_location_x(2), symbol->get_location_y(2)), Scalar(0, 255, 0), 2, 8, 0);
		  line(img, Point(symbol->get_location_x(2), symbol->get_location_y(2)), Point(symbol->get_location_x(3), symbol->get_location_y(3)), Scalar(0, 255, 0), 2, 8, 0);
		  line(img, Point(symbol->get_location_x(3), symbol->get_location_y(3)), Point(symbol->get_location_x(0), symbol->get_location_y(0)), Scalar(0, 255, 0), 2, 8, 0);
          //获得四个点的坐标
          double x0=symbol->get_location_x(0);
          double y0=symbol->get_location_y(0);
          double x1=symbol->get_location_x(1);
          double y1=symbol->get_location_y(1);
          double x2=symbol->get_location_x(2);
          double y2=symbol->get_location_y(2);
          double x3=symbol->get_location_x(3);
          double y3=symbol->get_location_y(3);
            
          //两条对角线的系数和偏移
          double k1=(y2-y0)/(x2-x0);
          double b1=(x2*y0-x0*y2)/(x2-x0);
          double k2=(y3-y1)/(x3-x1);
          double b2=(x3*y1-x1*y3)/(x3-x1);
          //两条对角线交点的X坐标
          double crossX = -(b1-b2)/(k1-k2);
          double crossY = (k1*b2 - k2 *b1)/(k1-k2);

          double centerX = (x0 + x3)/2;
          double centerY = (y0 + y3)/2;

          //qr coordinate
          lineColor = Scalar(0, 0, 255); 
          DrawArrow(img, Point(crossX, crossY), Point(centerX, centerY), 25, 30, lineColor, 2, CV_AA); 
          DrawArrow(img, Point(crossX, crossY), Point(crossX, crossY- 200), 25, 30, lineColor, 2, CV_AA); 
          //L
          lineColor = Scalar(255, 0, 0); 
          DrawArrow(img, Point(crossX, crossY), Point(img.cols/2, img.rows/2), 25, 30, lineColor, 2, CV_AA); 
          double L = sqrt(pow(crossX - img.cols/2, 2) + pow(crossY - img.rows/2, 2));
          //cout << "length = " << L << endl;
          //caluate the angle
          Point2f c(crossX, crossY);
          Point2f pt1(centerX, centerY);
          Point2f pt2(img.cols/2, img.rows/2);
          Point2f pt3(crossX, crossY- 200);

          float a1 = GetAngelOfTwoVector(pt1, pt2, c);
          float a2 = (180 - a1)*CV_PI/180;
          float a3 = GetAngelOfTwoVector(pt1, pt3, c);

          double x = L * cos(a2);
          double y = L * sin(a2);

          cout << "Horizontal Proj: " << x << endl;
          cout << "Vertical Proj:" << y << endl;
          cout << "Angle:" << a3 << endl<< endl;

          //broadcast tf between qr-cam
          static tf::TransformBroadcaster br;
          tf::Transform transform;
          transform.setOrigin( tf::Vector3(x, y, 0.0) );
          tf::Quaternion q;
          q.setRPY(0, 0, a3);
          transform.setRotation(q);
          br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_camera", "base_qr"));
	  }
    
      
    }
    //imshow("final", frame);
    imshow("captured", img);
    waitKey(1);

    image.set_data(NULL, 0);
}