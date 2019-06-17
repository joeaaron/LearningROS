#include "pallet_tf/pallet_tf.h"

RNG rng(12345);
#define CAMERAMAT "CameraMat"
#define DISTCOEFF "DistCoeff"
#define debug 0

static const string RGB_WIN = "RGB Image Window";    //定义一个RGB窗口   
static const string DEPTH_WIN = "DEPTH Image Window" ;//定义一个Depeth窗口   

ImageConverter::ImageConverter(ros::NodeHandle nh,const string& calibFile)
	: it(nh),
	_calibFile(calibFile),
    lineColor(255, 255, 255)
{
	readParameters(nh);  //when it scatlhould be used.

    //使用image_transport订阅图像话题“in” 和 发布图像话题“out” /camera/rgb/image_raw
    color_sub=it.subscribe("/colormap",1,&ImageConverter::colorCb,this);
    depth_sub=it.subscribe("/depthmap",1,&ImageConverter::depthCb,this); 
    color_pub=it.advertise("pallet_tf",1);

}

void ImageConverter::cfgCallback(const Parameter cfg)
{
    _cfg = cfg;
}

void ImageConverter::readParameters(ros::NodeHandle nh_)
{
    readCalibPara(_calibFile.c_str());
	nh_.param<int>("KERNALVALUE", kernel, 1);
	nh_.param<double>("RATIOMIN", minratio, 0.05);
	nh_.param<double>("RATIOMAX", maxratio, 0.05);
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

    unit_x = m_camMat.at<double>(0, 0); //if calibrated right, the effect will be good.
    unit_y = m_camMat.at<double>(1, 1);
    //cout <<  "dx= " << unit_x << endl << "dy= "<< unit_y<<endl;
}

void ImageConverter::nearestFiltering(cv::Mat& img)
{
	cv::Mat i_depth(480, 640, CV_16UC1);
	i_depth = img.clone();

	cv::Mat i_before(480, 640, CV_8UC4);				
	cv::Mat i_after(480, 640, CV_8UC4);					
	cv::Mat i_result(480, 640, CV_16UC1);				
	unsigned short maxDepth = 0;
	unsigned short* depthArray = (unsigned short*)i_depth.data;
	unsigned short iZeroCountBefore = 0;
	unsigned short iZeroCountAfter = 0;
	for (int i = 0; i < 640 * 480; i++)
	{
		int row = i / 640;
		int col = i % 640;

		unsigned short depthValue = depthArray[row * 640 + col];
		if (depthValue == 0)
		{
			i_before.data[i * 4] = 255;
			i_before.data[i * 4 + 1] = 0;
			i_before.data[i * 4 + 2] = 0;
			i_before.data[i * 4 + 3] = depthValue / 256;
			iZeroCountBefore++;
		}
		else
		{
			i_before.data[i * 4] = depthValue / 8000.0f * 256;
			i_before.data[i * 4 + 1] = depthValue / 8000.0f * 256;
			i_before.data[i * 4 + 2] = depthValue / 8000.0f * 256;
			i_before.data[i * 4 + 3] = depthValue / 8000.0f * 256;
		}
		maxDepth = depthValue > maxDepth ? depthValue : maxDepth;
	}
	//cout << "max depth value: " << maxDepth << endl;


	unsigned short* smoothDepthArray = (unsigned short*)i_result.data;
	int widthBound = 640 - 1;
	int heightBound = 480 - 1;


	int innerBandThreshold = 3;
	int outerBandThreshold = 7;


	for (int depthArrayRowIndex = 0; depthArrayRowIndex < 480; depthArrayRowIndex++)
	{

		for (int depthArrayColumnIndex = 0; depthArrayColumnIndex < 640; depthArrayColumnIndex++)
		{
			int depthIndex = depthArrayColumnIndex + (depthArrayRowIndex * 640);

			if (depthArray[depthIndex] == 0)
			{

				int x = depthIndex % 640;
				int y = (depthIndex - x) / 640;

				unsigned short filterCollection[24][2] = { 0 };

				int innerBandCount = 0;
				int outerBandCount = 0;

			
				for (int yi = -2; yi < 3; yi++)
				{
					for (int xi = -2; xi < 3; xi++)
					{
					
						if (xi != 0 || yi != 0)
						{
							
							int xSearch = x + xi;
							int ySearch = y + yi;

						
							if (xSearch >= 0 && xSearch <= widthBound &&
								ySearch >= 0 && ySearch <= heightBound)
							{
								int index = xSearch + (ySearch * 640);
	
								if (depthArray[index] != 0)
								{

									for (int i = 0; i < 24; i++)
									{
										if (filterCollection[i][0] == depthArray[index])
										{
											filterCollection[i][1]++;
											break;
										}
										else if (filterCollection[i][0] == 0)
										{

											filterCollection[i][0] = depthArray[index];
											filterCollection[i][1]++;
											break;
										}
									}

									
									if (yi != 2 && yi != -2 && xi != 2 && xi != -2)
										innerBandCount++;
									else
										outerBandCount++;
								}
							}
						}
					}
				}

				
			
				if (innerBandCount >= innerBandThreshold || outerBandCount >= outerBandThreshold)
				{
					short frequency = 0;
					short depth = 0;
				
					for (int i = 0; i < 24; i++)
					{
						if (filterCollection[i][0] == 0)
							break;
						if (filterCollection[i][1] > frequency)
						{
							depth = filterCollection[i][0];
							frequency = filterCollection[i][1];
						}
					}

					smoothDepthArray[depthIndex] = depth;
				}
				else
				{
					smoothDepthArray[depthIndex] = maxDepth;
				}
			}
			else
			{
				smoothDepthArray[depthIndex] = depthArray[depthIndex];
			}
		}
	}

	for (int i = 0; i < 640 * 480; i++)
	{
		int row = i / 640;
		int col = i % 640;

		unsigned short depthValue = smoothDepthArray[row * 640 + col];
		if (depthValue == 0)
		{
			i_after.data[i * 4] = 255;
			i_after.data[i * 4 + 1] = 0;
			i_after.data[i * 4 + 2] = 0;
			i_after.data[i * 4 + 3] = depthValue / 256;
			iZeroCountAfter++;
		}
		else
		{
			i_after.data[i * 4] = depthValue / 8000.0f * 256;
			i_after.data[i * 4 + 1] = depthValue / 8000.0f * 256;
			i_after.data[i * 4 + 2] = depthValue / 8000.0f * 256;
			i_after.data[i * 4 + 3] = depthValue / 8000.0f * 256;
		}
	}

	img = i_result.clone();
}

cv::Mat ImageConverter::getNearestPart(cv::Mat img)
{
	double minv = 0.0, maxv = 0.0;
	double* minp = &minv;
	double* maxp = &maxv;

	cv::minMaxIdx(img, minp, maxp);
	// find the connected component
	cv::MatIterator_<uchar> it, end;
	for (it = img.begin<uchar>(), end = img.end<uchar>(); it != end; ++it)
	{

		if (abs((*it) - minv) < _cfg.DISTANCE)  //find the closest point instead
			(*it) = 255;
		else
			(*it) = 0;
	}

	return img;
}

vector<vector<cv::Point> > ImageConverter::getContours(cv::Mat img, vector<vector<cv::Point> >& _contours)
{

	cv::Mat kernal = cv::Mat::ones(kernel, kernel, CV_8U);
	cv::morphologyEx(img, img, cv::MORPH_CLOSE, kernal, { -1, -1 }, 10);
#if debug
	cv::imshow("morphology",img);  
    cvWaitKey(1);
#endif

	vector<vector<cv::Point> > contours;
	vector<cv::Vec4i> hierarchy, _hierarchy;
	findContours(img, contours, hierarchy, CV_RETR_EXTERNAL, cv::CHAIN_APPROX_NONE, cv::Point(0, 0));
#if debug
	cv::Mat draw = cv::Mat::zeros(img.size(), CV_8U);
	for (size_t i = 0; i < contours.size(); i++){
		drawContours(draw, contours, i, cv::Scalar(255), -1);
	}
#endif
	findContours(img, _contours, _hierarchy, CV_RETR_TREE, cv::CHAIN_APPROX_NONE, cv::Point(0, 0));

	if (contours.size() == _contours.size())
	{
		cv::destroyWindow("pallet");
		return {};
	}

	for (int i = 0; i < contours.size(); i++)
	{
		int contourSize = contours[i].size();
		for (int j = 0; j < _contours.size(); j++)
		{
			int size = _contours[j].size();
			if (contourSize == size)
			{
				swap(_contours[j], _contours[_contours.size() - 1]);
				_contours.pop_back();
				break;
			}
		}
	}
	
#if debug
	vector<vector<Point>>::const_iterator itContours = _contours.begin();
	for (; itContours != _contours.end(); ++itContours) {

		cout << "Size: " << itContours->size() << endl;
	}

	// draw black contours on white image
	cv::Mat drawing = cv::Mat::zeros(img.size(), CV_8U);
	drawContours(drawing, _contours,
		-1,
		cv::Scalar(255),
		2);
	cout << "contour size = " << _contours.size() << endl;
	cv::imshow("contours",drawing);  
    cvWaitKey(1);
#endif
	return _contours;
}

bool ImageConverter::isAreaQualified(cv::Mat& img, vector<vector<cv::Point> > contours, vector<cv::Point>& center_points)
{
	vector<double> area;
	for (int i = 0; i < contours.size(); ++i)
	{
		area.push_back(contourArea(contours[i]));
		cv::Mat tmp_pallet(contours.at(i));
		cv::Moments moment = cv::moments(tmp_pallet, false);
		//assert(moment.m00 != 0);
		if (moment.m00 != 0)
		{
			int x = cvRound(moment.m10 / moment.m00);
			int y = cvRound(moment.m01 / moment.m00);
			if (x > 0 && y > 0)
			{
				cv::circle(img, cv::Point(x, y), 5, cv::Scalar(255, 255, 255));
				center_points.push_back(cv::Point(x, y));
			}
		}
		else
			return false;
	}

	if ((area[0] / area[1]) < minratio || (area[0] / area[1]) > maxratio)        //exclude too small area with similar
		return false;
	else
		return true;
}

void ImageConverter::drawArrow(cv::Mat& img, cv::Point pStart, cv::Point pEnd, int len, int alpha,
	cv::Scalar& color, int thickness, int lineType)
{ 
	const double PI = 3.1415926;
	cv::Point arrow;

	double angle = atan2((double)(pStart.y - pEnd.y), (double)(pStart.x - pEnd.x));
	line(img, pStart, pEnd, color, thickness, lineType);
	
	arrow.x = pEnd.x + len * cos(angle + PI * alpha / 180);
	arrow.y = pEnd.y + len * sin(angle + PI * alpha / 180);
	line(img, pEnd, arrow, color, thickness, lineType);
	arrow.x = pEnd.x + len * cos(angle - PI * alpha / 180);
	arrow.y = pEnd.y + len * sin(angle - PI * alpha / 180);
	line(img, pEnd, arrow, color, thickness, lineType);
}

void ImageConverter::ProcessFrame(cv_bridge::CvImagePtr cv_ptr)
{
    Mat cvRawImg16U, cvDepthImg;
    cv_ptr->image.copyTo(cvRawImg16U);   
    if (cvRawImg16U.empty())
      cout << "no image stream read!Please check the camera first.";

    nearestFiltering(cvRawImg16U);   
    cvRawImg16U.convertTo(cvDepthImg, CV_8U, 255.0 / 10000);
    cv::imshow(DEPTH_WIN,cvDepthImg);  
    cvWaitKey(1);

    //find the closest point 
	cv::Mat palletImg = cvDepthImg.clone();
	getNearestPart(palletImg);
    cv::imshow("Extraction",palletImg);  
    cvWaitKey(1);

    vector<vector<cv::Point> > contours;
	getContours(palletImg, contours);
    if (contours.empty())  cout<< "no coutours detect!" << endl;
	
    else
    {
        if (contours.size() != 2) cout<< "num of contours is wrong"<< endl;
		else
		{
			vector<cv::Point> pallet_center_points;
			if (!isAreaQualified(palletImg, contours, pallet_center_points))
				cout<< "area not qualified" << endl;
			else
			{
				double pallet_x = 0;
				double pallet_y = 0;

				for (auto i : pallet_center_points)
				{
					pallet_x += i.x;
					pallet_y += i.y;
				}
				pallet_x = pallet_x / pallet_center_points.size();
				pallet_y = pallet_y / pallet_center_points.size();

				double k = 0;
				double yaw = 0;
				if (pallet_center_points[0].y == pallet_center_points[1].y)
					cout << "yaw = 0" << endl;
				else
				{
					k = -(pallet_center_points[0].x - pallet_center_points[1].x) / (pallet_center_points[0].y - pallet_center_points[1].y);
					// right:k < 0 ; left:k > 0 
					if (k < 0)
						yaw = 90 + atan(k) / CV_PI * 180;
					else
						yaw = 90 - atan(k) / CV_PI * 180;
					
					cout << "yaw = " << yaw << endl;
				}

		#if debug
				cv::Mat drawing_ = cv::Mat::zeros(palletImg.size(), CV_8UC3);
				cv::circle(drawing_, cv::Point(pallet_x, pallet_y), 3, cv::Scalar(255, 0, 0));
		#endif
				cv::Scalar lineColor = cv::Scalar(255, 0, 0);
				drawArrow(palletImg, cv::Point(pallet_x, pallet_y), cv::Point(palletImg.cols / 2, palletImg.rows / 2), 25, 30, lineColor, 1, CV_AA);
				cv::imshow("pallet", palletImg);
				cv::waitKey(1);
			}
				
		}	
		
	}
	
}


//订阅回调函数
void ImageConverter::colorCb(const sensor_msgs::ImageConstPtr& color_msg)
{
    cv_bridge::CvImagePtr color_ptr;
    Mat rgbImg;
    try
    {
        //将ROS图像消息转化为适合Opencv的CvImage
        color_ptr=cv_bridge::toCvCopy(color_msg,sensor_msgs::image_encodings::BGR8);

    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s",e.what());
        return;
    }
    color_ptr->image.copyTo(rgbImg);  
 
    cv::imshow(RGB_WIN,rgbImg);  
    cvWaitKey(1);  //necessary
    
}

//订阅回调函数
void ImageConverter::depthCb(const sensor_msgs::ImageConstPtr& depth_msg)
{
    cv_bridge::CvImagePtr depth_ptr;
    Mat depthImg;
    try
    { 
        depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1); 

    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s",e.what());
        return;
    }  

    ProcessFrame(depth_ptr);
    depth_pub.publish(depth_ptr->toImageMsg());
}

