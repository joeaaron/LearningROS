/*************************************************************************
  @Author: JoeAaron
  @Email: pant333@163.com
  @Created Time : 2018-09-04 06:25:13PM
  @Last Modified : 2018-09-12 10:46:13AM
  @File Name: quadScanner.cpp
  @Description:
 ************************************************************************/

#include "quad_navig/quadScanner.h"

#define CAMERAMAT "CameraMat"
#define DISTCOEFF "DistCoeff"

const int MAX_QUADLENGTH = 2000;
const int MIN_QUADLENGTH = 200;

struct str{
	bool operator()(Point2f a, Point2f b){
		if(a.y != b.y)
			return a.y < b.y;
		return a.x <= b.x;
	}
}comp;

QuadScanner::QuadScanner(ros::NodeHandle nh, const string& calibFile)
	: it(nh)
{
	ReadCalibPara(calibFile.c_str());

	image_sub=it.subscribe("/usb_cam/image_raw",1,&QuadScanner::imageCb,this);
	image_pub=it.advertise("quad_navig",1);
}

int QuadScanner::ReadCalibPara(string filename)
{
	Mat m_camMat;
	Mat m_distCoeff;

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
	cout <<  "dx= " << unit_x << endl << "dy= "<< unit_y<<endl;
}

Vec2d linearParameters(Vec4i line){
	Mat a = (Mat_<double>(2, 2) <<
			line[0], 1,
			line[2], 1);
	Mat y = (Mat_<double>(2, 1) <<
			line[1],
			line[3]);
	Vec2d mc; solve(a, y, mc);
	return mc;
}

Vec4i extendedLine(Vec4i line, double d){
	// oriented left-t-right
	Vec4d _line = line[2] - line[0] < 0 ? Vec4d(line[2], line[3], line[0], line[1]) : Vec4d(line[0], line[1], line[2], line[3]);
	double m = linearParameters(_line)[0];
	// solution of pythagorean theorem and m = yd/xd
	double xd = sqrt(d * d / (m * m + 1));
	double yd = xd * m;
	return Vec4d(_line[0] - xd, _line[1] - yd , _line[2] + xd, _line[3] + yd);
}

std::vector<Point2i> boundingRectangleContour(Vec4i line, float d){
	// finds coordinates of perpendicular lines with length d in both line points
	// https://math.stackexchange.com/a/2043065/183923

	Vec2f mc = linearParameters(line);
	float m = mc[0];
	float factor = sqrtf(
			(d * d) / (1 + (1 / (m * m)))
			);

	float x3, y3, x4, y4, x5, y5, x6, y6;
	// special case(vertical perpendicular line) when -1/m -> -infinity
	if(m == 0){
		x3 = line[0]; y3 = line[1] + d;
		x4 = line[0]; y4 = line[1] - d;
		x5 = line[2]; y5 = line[3] + d;
		x6 = line[2]; y6 = line[3] - d;
	} else {
		// slope of perpendicular lines
		float m_per = - 1/m;

		// y1 = m_per * x1 + c_per
		float c_per1 = line[1] - m_per * line[0];
		float c_per2 = line[3] - m_per * line[2];

		// coordinates of perpendicular lines
		x3 = line[0] + factor; y3 = m_per * x3 + c_per1;
		x4 = line[0] - factor; y4 = m_per * x4 + c_per1;
		x5 = line[2] + factor; y5 = m_per * x5 + c_per2;
		x6 = line[2] - factor; y6 = m_per * x6 + c_per2;
	}

	return std::vector<Point2i> {
		Point2i(x3, y3),
			Point2i(x4, y4),
			Point2i(x6, y6),
			Point2i(x5, y5)
	};
}

bool extendedBoundingRectangleLineEquivalence(const Vec4i& _l1, const Vec4i& _l2, float extensionLengthFraction, float maxAngleDiff, float boundingRectangleThickness){

	Vec4i l1(_l1), l2(_l2);
	// extend lines by percentage of line width
	float len1 = sqrtf((l1[2] - l1[0])*(l1[2] - l1[0]) + (l1[3] - l1[1])*(l1[3] - l1[1]));
	float len2 = sqrtf((l2[2] - l2[0])*(l2[2] - l2[0]) + (l2[3] - l2[1])*(l2[3] - l2[1]));
	Vec4i el1 = extendedLine(l1, len1 * extensionLengthFraction);
	Vec4i el2 = extendedLine(l2, len2 * extensionLengthFraction);

	// reject the lines that have wide difference in angles
	float a1 = atan(linearParameters(el1)[0]);
	float a2 = atan(linearParameters(el2)[0]);
	if(fabs(a1 - a2) > maxAngleDiff * M_PI / 180.0){
		return false;
	}

	// calculate window around extended line
	// at least one point needs to inside extended bounding rectangle of other line,
	std::vector<Point2i> lineBoundingContour = boundingRectangleContour(el1, boundingRectangleThickness/2);
	return
		pointPolygonTest(lineBoundingContour, cv::Point(el2[0], el2[1]), false) == 1 ||
		pointPolygonTest(lineBoundingContour, cv::Point(el2[2], el2[3]), false) == 1;
}

inline Point2f GetCrossPoint(Vec4i lineA, Vec4i lineB)
{
	double ka, kb;
	ka = (double)(lineA[3] - lineA[1]) / (double)(lineA[2] - lineA[0]);
	kb = (double)(lineB[3] - lineB[1]) / (double)(lineB[2] - lineB[0]);

	Point2f crossPoint;
	crossPoint.x = (ka*lineA[0] - lineA[1] - kb*lineB[0] + lineB[1]) / (ka - kb);
	crossPoint.y = (ka*kb*(lineA[0] - lineB[0]) + ka*lineB[1] - kb*lineA[1]) / (ka - kb);

	return crossPoint;

}

void CalcDstSize(const vector<cv::Point2f>& corners, int& h1, int& h2, int& w1, int& w2)
{
	h1 = sqrt((corners[0].x - corners[3].x)*(corners[0].x - corners[3].x) + (corners[0].y - corners[3].y)*(corners[0].y - corners[3].y));
	h2 = sqrt((corners[1].x - corners[2].x)*(corners[1].x - corners[2].x) + (corners[1].y - corners[2].y)*(corners[1].y - corners[2].y));

	w1 = sqrt((corners[0].x - corners[1].x)*(corners[0].x - corners[1].x) + (corners[0].y - corners[1].y)*(corners[0].y - corners[1].y));
	w2 = sqrt((corners[2].x - corners[3].x)*(corners[2].x - corners[3].x) + (corners[2].y - corners[3].y)*(corners[2].y - corners[3].y));
}

float GetAngleOfTwoVector(Point2f &pt1, Point2f &pt2, Point2f &c)
{
	float theta = atan2(pt1.x - c.x, pt1.y - c.y) - atan2(pt2.x - c.x, pt2.y - c.y);
	if (theta > CV_PI)
		theta -= 2 * CV_PI;
	if (theta < -CV_PI)
		theta += 2 * CV_PI;

	// theta = theta * 180.0 / CV_PI;
	return theta;
}

void QuadScanner::FindCandidate(Mat img, Mat& drawing)
{
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	findContours(img, contours, hierarchy, CV_RETR_EXTERNAL, CHAIN_APPROX_NONE, Point(0,0));
	vector<vector<Point> > polyContours(contours.size());
	int maxArea = 0;
	for(int index = 0; index < contours.size(); index++)
	{
		if(contourArea(contours[index]) > contourArea(contours[maxArea]))
			maxArea = index;
		approxPolyDP(contours[index], polyContours[index],10, true);
	}

	drawContours(drawing, polyContours, maxArea, Scalar(0, 0, 255), 2);
}

void QuadScanner::LineDetection(Mat img, std::vector<Vec4i>& reducedLines)
{
	Mat detectedLinesImg = Mat::zeros(img.rows, img.cols, CV_8UC3);
	Mat reducedLinesImg = detectedLinesImg.clone();

	Ptr<LineSegmentDetector> detector = createLineSegmentDetector(LSD_REFINE_NONE);
	std::vector<Vec4i> lines; detector->detect(img, lines);

	//remove small lines
	std::vector<Vec4i> linesWithoutSmall;
	std::copy_if (lines.begin(), lines.end(), std::back_inserter(linesWithoutSmall), [](Vec4f line){
			float length = sqrtf((line[2] - line[0]) * (line[2] - line[0])
					+ (line[3] - line[1]) * (line[3] - line[1]));
			return length > 30;
			});
	//partion via our partioning function
	std::vector<int> labels;
	int equilavenceClassesCount = cv::partition(linesWithoutSmall, labels, [](const Vec4i l1, const Vec4i l2){
			return extendedBoundingRectangleLineEquivalence(
					l1, l2,
					// line extension length - as fraction of original line width
					0.2,
					// maximum allowed angle difference for lines to be considered in same equivalence class
					2.0,
					// thickness of bounding rectangle around each line
					10);
			});

	// build point clouds out of each equivalence classes
	std::vector<std::vector<Point2i>> pointClouds(equilavenceClassesCount);
	for (int i = 0; i < linesWithoutSmall.size(); i++){
		Vec4i& detectedLine = linesWithoutSmall[i];
		pointClouds[labels[i]].push_back(Point2i(detectedLine[0], detectedLine[1]));
		pointClouds[labels[i]].push_back(Point2i(detectedLine[2], detectedLine[3]));
	}

	// fit line to each equivalence class point cloud
	reducedLines = std::accumulate(pointClouds.begin(), pointClouds.end(), std::vector<Vec4i>{}, [](std::vector<Vec4i> target, const std::vector<Point2i>& _pointCloud){
			std::vector<Point2i> pointCloud = _pointCloud;

			//lineParams: [vx,vy, x0,y0]: (normalized vector, point on our contour)
			// (x,y) = (x0,y0) + t*(vx,vy), t -> (-inf; inf)
			Vec4f lineParams; fitLine(pointCloud, lineParams, CV_DIST_L2, 0, 0.01, 0.01);

			// derive the bounding xs of point cloud
			decltype(pointCloud)::iterator minXP, maxXP;
			std::tie(minXP, maxXP) = std::minmax_element(pointCloud.begin(), pointCloud.end(), [](const Point2i& p1, const Point2i& p2){ return p1.x < p2.x; });

			// derive y coords of fitted line
			float m = lineParams[1] / lineParams[0];
			int y1 = ((minXP->x - lineParams[2]) * m) + lineParams[3];
			int y2 = ((maxXP->x - lineParams[2]) * m) + lineParams[3];

			target.push_back(Vec4i(minXP->x, y1, maxXP->x, y2));
			return target;
			});
	for(Vec4i reduced: reducedLines){
		line(reducedLinesImg, Point(reduced[0], reduced[1]), Point(reduced[2], reduced[3]), Scalar(255, 255, 255), 2);
	}
}

void QuadScanner::IsQuad(Mat img, std::vector<Vec4i> lines, bool& flag, vector<Point2f>& crossPoints)
{
	if(lines.size() != 4)
		flag = false;
	else{
		for(int i = 0;i < lines.size(); i++)
		{
			for(int j = i+ 1; j < lines.size(); j++)
			{
				Point2f pt = GetCrossPoint(lines[i], lines[j]);
				if(pt.x >=0 && pt.y >= 0 && pt.x <= img.cols && pt.y <= img.rows)
					crossPoints.push_back(pt);
			}
		}

		if(crossPoints.size() != 4)
			flag = false;

		bool isGoodPoints = true;
		for(int i = 0;i < crossPoints.size(); i++)
		{
			for(int j = i+ 1; j < crossPoints.size(); j++)
			{
				int distance = sqrt((crossPoints[i].x - crossPoints[j].x)*(crossPoints[i].x - crossPoints[j].x) + (crossPoints[i].y - crossPoints[j].y)*(crossPoints[i].y - crossPoints[j].y));
				if(distance < MIN_QUADLENGTH && distance < MAX_QUADLENGTH)
					isGoodPoints = false;
			}
		}

		if(!isGoodPoints) flag = false;
		else{
			for(int i = -1; i < 4; i++)
				circle(img, crossPoints[i], 9, Scalar(rand() & 255, rand() & 255, rand() & 255), 3);

			flag = true;

			imshow("Result", img);
			waitKey(1);

		}
	}

}

void QuadScanner::DrawArrow(cv::Mat& img, cv::Point pStart, cv::Point pEnd, int len, int alpha,
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

void QuadScanner::GetTfTrans(double x,double y, double theta)
{
	double quad_tf_x = x / unit_x;
	double quad_tf_y = y / unit_y;
	double quad_tf_angle = theta;

	//double rot = theta * 180 /CV_PI;
	//cout << "Horizontal Proj: " << qr_tf_x << endl;
	//cout << "Vertical Proj:" << qr_tf_y << endl;
	//cout << "Angle:" << qr_tf_angle << endl<< endl;
	//cout << "Rotation:" << rot << endl<< endl;

	//broadcast tf between qr-cam
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(-quad_tf_y, -quad_tf_x, 0.0) );
	tf::Quaternion q;
	q.setRPY(0, 0, quad_tf_angle);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_camera", "base_quad"));
}

void QuadScanner::GetQuadCamTf(Mat img, vector<Point2f> crossPoints)
{
	//获得四个点的坐标
	vector<double> coordinates_x;
	vector<double> coordinates_y;
	for(int i = 0; i < crossPoints.size(); i++)
	{
		coordinates_x[i] = crossPoints[i].x;
		coordinates_y[i] = crossPoints[i].y;
	}

	//两条对角线的系数和偏移
	double k1 = (coordinates_y[2] - coordinates_y[0]) / (coordinates_x[2] - coordinates_x[0]);
	double b1 = (coordinates_x[2] * coordinates_y[0] - coordinates_x[0] * coordinates_y[2]) / (coordinates_x[2] - coordinates_x[0]);
	double k2 = (coordinates_y[3] - coordinates_y[1]) / (coordinates_x[3] - coordinates_x[1]);
	double b2 = (coordinates_x[3] * coordinates_y[1] - coordinates_x[1] * coordinates_y[3]) / (coordinates_x[3] - coordinates_x[1]);

	//两条对角线交点的X坐标
	double cross_x = -(b1-b2)/(k1-k2);
	double cross_y = (k1*b2 - k2 *b1)/(k1-k2);

	double center_x = (coordinates_x[0] + coordinates_x[3]) / 2;
	double center_y = (coordinates_y[0] + coordinates_y[3]) / 2;

	//quad coordinate
	Scalar lineColor = Scalar(0, 0, 255);
	DrawArrow(img, Point(cross_x, cross_y), Point(center_x, center_y), 25, 30, lineColor, 2, CV_AA);
	DrawArrow(img, Point(cross_x, cross_y), Point(cross_x, cross_y- 200), 25, 30, lineColor, 2, CV_AA);
	//L
	lineColor = Scalar(255, 0, 0);
	DrawArrow(img, Point(cross_x, cross_y), Point(img.cols/2, img.rows/2), 25, 30, lineColor, 2, CV_AA);
	double L = sqrt(pow(cross_x - img.cols/2, 2) + pow(cross_y - img.rows/2, 2));

	Point2f pt0(cross_x, cross_y);
	Point2f pt1(center_x, center_y);
	Point2f pt2(img.cols/2, img.rows/2);
	Point2f pt3(cross_x, cross_y - 200);

	float a1 = GetAngleOfTwoVector(pt1, pt2, pt0);
	float a2 = CV_PI - a1;
	float a3 = GetAngleOfTwoVector(pt1, pt3, pt0);

	double x = L * cos(a2);
	double y = L * sin(a2);

	GetTfTrans(x, y, a3);
}

/**
 * @brief: Detect the quad that camera could cover
 * @function QuadDetect
 * @param[in] cv_ptr
 * @return none
 **/
void QuadScanner::QuadDetect(cv_bridge::CvImagePtr cv_ptr)
{
	Mat frame, img, bgr, hsv;
	frame = cv_ptr->image;
	img = frame.clone();
	//Image Preprocess
	blur(img, img, Size(3,3));
	img.convertTo(bgr, CV_32FC3, 1.0 / 255, 0);
	cvtColor(bgr, hsv, COLOR_BGR2HSV);

	vector<Mat> mv;
	split(hsv, mv);
	Mat s = mv[1];

	threshold(s, s, 0.3, 255, THRESH_BINARY);
	s.convertTo(s, CV_8U, 1, 0);
	Mat drawing = Mat::zeros(img.size(), CV_8UC3);
	FindCandidate(s, drawing);

	cvtColor(drawing, drawing, CV_BGR2GRAY);
	std::vector<Vec4i> reducedLines;
	LineDetection(drawing, reducedLines);

	bool bQuad = false;
	vector<Point2f> crossPoints;
	IsQuad(img, reducedLines, bQuad, crossPoints);

	if(bQuad)
	{
		//topLeft, topRight, bottomLeft, bottomRight
		sort(crossPoints.begin(), crossPoints.end(), comp);
		//pub tf
		GetQuadCamTf(img, crossPoints);
	}
}

void QuadScanner::imageCb(const sensor_msgs::ImageConstPtr& msg)
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
	QuadDetect(cv_ptr);

	image_pub.publish(cv_ptr->toImageMsg());
}
