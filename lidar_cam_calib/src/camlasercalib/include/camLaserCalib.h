#ifndef CAMLASERCALIB_H
#define CAMLASERCALIB_H

#include <ros/ros.h>
#include <string>
#include <cv_bridge/cv_bridge.h>
#include "pcl_util.h"
#include "MarkerDetector.h"

using namespace std;
/*!
 * parameter used to get the rectangle cut area of point cloud
 * laset coordinate:x~left,y~front,z~up
 * camera coordinate:x~left,y~down,z~front
 * so we shoud align the two coordinate to before we get the calibtation matrix
 */
struct Parameter
{
    double x_min;
    double x_max;
    double y_min;
    double y_max;
    double z_min;
    double z_max;
};

namespace calib{

	class CamLaserCalib
	{
	public:
		CamLaserCalib(ros::NodeHandle nodehandle, const string& imgCloudPointsFile, const string& calibFile);
		virtual ~CamLaserCalib(){};
	public:
		/*!
		* image and laser topic callback function using message filter
		*/
		void img_pc2_Callback(const sensor_msgs::ImageConstPtr& img,const sensor_msgs::PointCloud2ConstPtr& pc2);
		/*!
		* dynamic param callback function
		*/
		void cfgCallback(const Parameter cfg);
		/*!
		* get the calibration param from calib.yml file
		*/
		int readCalibPara(string filename);
		 /*!
		* grab cv::mat from image topic
		*/
		void grabImg(const sensor_msgs::ImageConstPtr& img);
		/*!
		* get qrcode center coordinate from image
		*/
		void getImgPoint();
		/*!
		* publish point cloud message
		*/
		void publishCloud();
	public:
		//! ROS Subscribe point cloud message
		string strSub_pc2;
		//! ROS Subscribe image message
		string strSub_img;
	private:
		void readParameters();
	private:
		// ROS nodehandle
		ros::NodeHandle _nodehandle;
		// dynamic config parameters
   	    Parameter _cfg;
		// MarkerDetect
    	MarkerDetector markCapture;
		// ROS Publisher
		ros::Publisher pub_pc2_cut;   //cut area of point cloud
		ros::Publisher pub_plane;     //estimated plane of the cutted point cloud area
		ros::Publisher pub_imgColorPc;//the color point cloud
		ros::Time pc2Time;             //pulish message topic time
		// pcl pointcloud
		Cloud::Ptr pc;                 //transform pointcloud2(pc2) to pointcloud(pc)
		Cloud::Ptr pc_cut;             //rectangle cloud
		ColorCloud::Ptr cloudPlane;    //estamted plane
		ColorCloud::Ptr imgColorCloud; //color cloud points draw with image color
		PointCT cloudPoint;            //cloud point center of the board
		//!image message
   		cv::Mat _img;                   //grab image
    	cv::Point2f imgPoint;              //image point center of the board
		 //!image calibration param
		cv::Mat cameraExtrinsicMat;    //camera laser calibration param
		cv::Mat cameraMat;             //camera internal parameter
		cv::Mat distCoeff;             //camera distortion parameter
		cv::Size _imageSize;            //image size
		cv::Mat R;                     //lasr to image rotation matrix
		cv::Mat T;                     //lasr to image translation matrix
		//! pcl Parameters
		double distance_threshold;     //param used to get the point cloud plane
		pcl::ModelCoefficients::Ptr coefficients;   //pcl plane param
		pcl::PointIndices::Ptr inlierIndices;       //pcl plane inlier indices

 		bool onlyDrawPointsColor;      //change in launch file.flag to judge if just draw points color or estimated ceter points of QR code board in both coordinates
		//!read and write param file
    	string _calibFile;              //cameta laser calibration file
    	ofstream imageCloudPoints;     //QR code board center points in image and laser coordinate
	};
}



#endif // CAMLASERCALIB_H
