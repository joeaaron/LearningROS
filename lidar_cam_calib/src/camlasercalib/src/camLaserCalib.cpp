#include "camLaserCalib.h"

using namespace calib;
using namespace pcl;

#define CAMERAEXTRINSICMAT "CameraExtrinsicMat"
#define CAMERAMAT "CameraMat"
#define DISTCOEFF "DistCoeff"
#define IMAGESIZE "ImageSize"

visualization::CloudViewer viewer("pcd");

void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
	viewer.removeAllShapes();

}

CamLaserCalib::CamLaserCalib(ros::NodeHandle nodehandle, const string& imgCloudPointsFile, const string& calibFile)
    : _nodehandle(nodehandle),
      pc(new Cloud),
      pc_cut(new Cloud),
      cloudPlane(new ColorCloud),
      imgColorCloud(new ColorCloud),
      coefficients(new ModelCoefficients),
      inlierIndices(new PointIndices),
      imageCloudPoints(imgCloudPointsFile.c_str()),
      _calibFile(calibFile)
{
    readParameters();
    pub_plane      = _nodehandle.advertise<sensor_msgs::PointCloud2> ("/plane", 1);
    pub_pc2_cut    = _nodehandle.advertise<sensor_msgs::PointCloud2> ("/cloudCut", 1);
    pub_imgColorPc = _nodehandle.advertise<sensor_msgs::PointCloud2> ("/imgColorCloud", 1);
}

void CamLaserCalib::readParameters()
{
    readCalibPara(_calibFile.c_str());
    _nodehandle.param("strSub_pc2",    strSub_pc2,     string("/velodyne32/velodyne_points"));      //topic name
    _nodehandle.param("strSub_img",    strSub_img,     string("/camera/image_color"));
    _nodehandle.param("onlyDrawPointsColor",    onlyDrawPointsColor,     true);
    _nodehandle.param("DistanceThreshold",      distance_threshold,      double(0.05));                //ransac DistanceThreshold
}

void CamLaserCalib::cfgCallback(const Parameter cfg)
{
    _cfg = cfg;
}

int CamLaserCalib::readCalibPara(string filename)
{
    cv::FileStorage fs(filename,cv::FileStorage::READ);
    if(!fs.isOpened())
    {
        std::cout<<"Invalid calibration filename."<<std::endl;
        return 0;
    }
    fs[CAMERAMAT]>>cameraMat;
    fs[DISTCOEFF]>>distCoeff;
    fs[CAMERAEXTRINSICMAT]>>cameraExtrinsicMat;
    fs[IMAGESIZE]>>_imageSize;
   // R_ = cameraExtrinsicMat_(cv::Rect(0,0,3,3)).t();
   // T_ = -R_*cameraExtrinsicMat_(cv::Rect(3,0,1,3));
    R = cameraExtrinsicMat(cv::Rect(0,0,3,3));
    T = cameraExtrinsicMat(cv::Rect(3,0,1,3));
    cout<<"cameraMat:"<<cameraMat<<endl;
    cout<<"distCoeff:"<<distCoeff<<endl;
    cout<<"cameraExtrinsicMat:"<<cameraExtrinsicMat<<endl;
    cout<<"imageSize:"<<_imageSize<<endl;
}

void CamLaserCalib::grabImg(const sensor_msgs::ImageConstPtr& img)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(img, "bgr8");
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    cv_ptr->image.copyTo(_img);
}

void CamLaserCalib::img_pc2_Callback(const sensor_msgs::ImageConstPtr& img,const sensor_msgs::PointCloud2ConstPtr& pc2)
{
    /************************** get image data **************************/
    grabImg(img);
    markCapture.processFrame(_img);
    /******************** process point cloud *********************/
    pc2Time = pc2->header.stamp;   //time 
    ROS2PCL(pc2, pc);
    //pcl::fromROSMsg(*pc2, ori_cloud);
    imshow("origin", _img);
    drawPointcloudColor(pc,imgColorCloud,_img,R,T,cameraMat, distCoeff, _imageSize);
    imshow("undistort", _img);
    cv::waitKey(5);
    
    if (!onlyDrawPointsColor)
    {
        /************************** process image **************************/
        if(!markCapture.m_markers.size())
            return;
        
        getImgPoint();
        /******************** process point cloud *********************/
        //step 1: Cut Area
        SubRectangle(pc, pc_cut,
                        _cfg.x_max, _cfg.x_min, _cfg.y_max, _cfg.y_min, _cfg.z_max, _cfg.z_min);
        if (!viewer.wasStopped()) 
            viewer.showCloud(pc_cut);
        viewer.runOnVisualizationThreadOnce(viewerOneOff);

        //step 2: Get Plane
        GetPlane(pc_cut, cloudPlane, cloudPoint, coefficients, inlierIndices, distance_threshold);
         /***************************** save QR code board center points in file*****************************************/
        imageCloudPoints<< cloudPoint.x <<" "<< cloudPoint.y <<" "<< cloudPoint.z<<" "<< imgPoint.x <<" "<<imgPoint.y<<endl;
    }
    
    else{
        if (!viewer.wasStopped()) 
            viewer.showCloud(imgColorCloud);
        viewer.runOnVisualizationThreadOnce(viewerOneOff);
    }
    
    publishCloud();
}

void CamLaserCalib::getImgPoint()
{
    //get the cross point from four corner points: http://blog.csdn.net/yangtrees/article/details/7965983
    vector<Point2f> points = markCapture.m_markers[0].m_points;
    double a0 = points[0].y- points[2].y;
    double b0 = points[2].x- points[0].x;
    double c0 = points[0].x * points[2].y - points[2].x * points[0].y;

    double a1 = points[1].y- points[3].y;
    double b1 = points[3].x- points[1].x;
    double c1 = points[1].x * points[3].y - points[3].x * points[1].y;

    double d = a0 * b1 - a1 * b0;
    imgPoint.x = (b0 * c1 - b1 * c0)/d;
    imgPoint.y = (a1 * c0 - a0 * c1)/d;
    circle(_img,imgPoint, 3, Scalar(0,0,255), 2, 8);
    // show marker in image
    for(int i=0; i<markCapture.m_markers.size(); i++)
    {
        int sizeNum = markCapture.m_markers[i].m_points.size();
        for (int j=0; j<sizeNum; j++)
        {
            line(_img, markCapture.m_markers[i].m_points[j], markCapture.m_markers[i].m_points[(j+1)%sizeNum], Scalar(0,0,255), 2, 8);
        }
        circle(_img, markCapture.m_markers[i].m_points[0], 3, Scalar(0,255,255), 2, 8);//clockwise code the points
    }
    imshow("markerDetector", _img);
    cv::waitKey(5);
}

void CamLaserCalib::publishCloud()
{
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::PCLPointCloud2 cloud_pc2;
    if(!onlyDrawPointsColor)
    {
        //punlish estimated plane cloud
        pcl::toPCLPointCloud2(*cloudPlane,cloud_pc2);
        pcl_conversions::fromPCL(cloud_pc2,cloud_msg);
        cloud_msg.header.frame_id = "velodyne16";
        cloud_msg.header.stamp = pc2Time;
        pub_plane.publish(cloud_msg);

        //punlish cut area of point cloud
        pcl::toPCLPointCloud2(*pc_cut,cloud_pc2);
        pcl_conversions::fromPCL(cloud_pc2,cloud_msg);
        cloud_msg.header.frame_id = "velodyne16";
        cloud_msg.header.stamp = pc2Time;
        pub_pc2_cut.publish(cloud_msg);
    }

      //punlish image color cloud
    pcl::toPCLPointCloud2(*imgColorCloud,cloud_pc2);
    pcl_conversions::fromPCL(cloud_pc2,cloud_msg);
    cloud_msg.header.frame_id = "velodyne16";
    cloud_msg.header.stamp = pc2Time;
    pub_imgColorPc.publish(cloud_msg);
}