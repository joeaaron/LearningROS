#include "barScanner.h"

RNG rng(12345);
#define CAMERAMAT "CameraMat"
#define DISTCOEFF "DistCoeff"

ImageConverter::ImageConverter(ros::NodeHandle nh,const string& calibFile)
	: it(nh),
	_calibFile(calibFile)
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
    m_markerCorners3d.push_back(cv::Point3f(-0.5f,-0.5f,0));
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
    
    cout<<"R:"<<Rvec<<endl;
    cout<<"T:"<<taux <<endl;
    cout<<"Extrinc: "<<Extrinc << endl;
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
    vector<vector<Point> > contours, _contours;
    Mat drawing = Mat::zeros(Size(500,500), CV_8UC3);
    Mat _drawing = Mat::zeros(Size(500,500), CV_8UC3);

    //pre-processing
    src_contour = img.clone();
    cvtColor(src_contour, src_gray, CV_BGR2GRAY);
    blur(src_gray, src_gray, Size(3, 3));
    threshold(src_gray, src_thresh, 100, 255, THRESH_OTSU);
 
    vector<Vec4i> hierarchy;
    findContours(src_thresh, contours, hierarchy, CV_RETR_TREE, CHAIN_APPROX_NONE, Point(0,0));
   
    vector<int> found;
    for (int i = 0; i < contours.size(); i++)
    {
    	double area = contourArea(contours[i]);
    	if (area < 100)  continue;

    	RotatedRect rect = minAreaRect(contours[i]);

    	//Geometric Analysis based on rectangular features
    	float w = rect.size.width;
    	float h = rect.size.height;
    	float rate = min(w, h) / max(w, h);
    	Point2f fourPoint2f[4];
    	rect.points(fourPoint2f);
    	if (w < 500/4 && h < 500 / 4)
    	{
    		int k = i;
    		int c = 0;

    		while(hierarchy[k][2] != -1)
    		{
    			k = hierarchy[k][2];
    			c = c + 1;
    		}
    		if (c >= 2)
    		{
    			found.push_back(i);
    			_contours.push_back(contours[i]);
    		}

    	}
    }

    if (found.size() >= 3)
    {
    	std::vector<int> indexs(4, -1);
    	CheckCenter(_contours, indexs);
    	std::vector<Point> final;
    	for (int i = 0; i < 4; ++i)
    	{
    		RotatedRect part_rect = minAreaRect(_contours[indexs[i]]);
    		Point2f p[4];
    		part_rect.points(p);
    		for (int j = 0; j < 4; j++)
    		{
    			final.push_back(p[j]);
    		}
    	}
    	
    	//Region of qr
    	Rect ROI=boundingRect(final);
	    Point left_top=ROI.tl();
	    Point right_down=ROI.br();
	    if(left_top.x>=20 || left_top.y>=20 || right_down.x<=500-20 || right_down.y<=500-20 )
	    {
	        ROI=ROI+Point(-20,-20)+Size(40,40);
	    }

	    if (ROI.tl().x >0 && ROI.tl().y>0 && ROI.br().x<500 && ROI.br().y<500)
	    {
	        rectangle( img, ROI.tl(), ROI.br(), Scalar(0, 0, 255));
	        Mat ROI_image;
	        ROI_image=img(ROI);
	        QRDecode(ROI_image);
	     }
    }


    //imshow("result", img);
    //waitKey(1);
   
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
        cout << "Failed to check qr code.Please ensure the image is right!";

      else
      {
        cout << "type:" << endl << symbol->get_type_name() << endl << endl;
        cout << "decoded:" << endl << symbol->get_data() << endl << endl;
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
	  EstimatePosition(pointBuf);
	  
	  for (vector<Point2f>::iterator it = pointBuf.begin(); it != pointBuf.end(); it++)
	  {
	  	 cout << "Points:" << *it << endl;
         circle(img, *it, 3, Scalar(255, 0, 0), -1, 8);
	  }
	  /*
	  RotatedRect r = minAreaRect(vp);
	  Point2f pts[4];
	  r.points(pts);

	  for (int i = 0; i < 4; i++)
	  {
	  	line(frame, pts[i], pts[(i+1) % 4], Scalar(255, 0, 0), 3);
	  }
	  */
      // Draw location of the symbols found
      if (symbol->get_location_size() == 4)
	  {
		line(img, Point(symbol->get_location_x(0), symbol->get_location_y(0)), Point(symbol->get_location_x(1), symbol->get_location_y(1)), Scalar(0, 255, 0), 2, 8, 0);
		line(img, Point(symbol->get_location_x(1), symbol->get_location_y(1)), Point(symbol->get_location_x(2), symbol->get_location_y(2)), Scalar(0, 255, 0), 2, 8, 0);
		line(img, Point(symbol->get_location_x(2), symbol->get_location_y(2)), Point(symbol->get_location_x(3), symbol->get_location_y(3)), Scalar(0, 255, 0), 2, 8, 0);
		line(img, Point(symbol->get_location_x(3), symbol->get_location_y(3)), Point(symbol->get_location_x(0), symbol->get_location_y(0)), Scalar(0, 255, 0), 2, 8, 0);
	  }
           
    }
   // imshow("final", frame);
    imshow("captured", img);
    waitKey(1);

    image.set_data(NULL, 0);
}