#pragma once
//!PCL
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/sample_consensus/method_types.h>   //随机参数估计方法头文件
#include <pcl/sample_consensus/model_types.h>    //模型定义头文件
#include <pcl/segmentation/sac_segmentation.h>   //基于采样一致性分割的类的头文件
//!MSGs
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

//!opencv
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

typedef pcl::PointXYZ    PointT;
typedef pcl::PointXYZRGB PointCT;
typedef pcl::PointCloud<PointT> Cloud;
typedef pcl::PointCloud<PointCT> ColorCloud;

#include "pcl_util.h"

//Message transform
void ROS2PCL(const sensor_msgs::PointCloud2ConstPtr packet, Cloud::Ptr cloud_out);

//! get rectangle area of point clouds
void SubRectangle(Cloud::Ptr cloud_in,
                  Cloud::Ptr cloud_out,
                  double cut_max_x,
                  double cut_min_x,
                  double cut_max_y,
                  double cut_min_y,
                  double cut_max_z,
                  double cut_min_z);

//! Plane Extraction
bool GetPlane(Cloud::Ptr cloud_in,
              ColorCloud::Ptr cloudPlane,
              PointCT &centroid_point,
              pcl::ModelCoefficients::Ptr coeffs,
              pcl::PointIndices::Ptr indices, double threshold);

//!draw point cloud with image color
void drawPointcloudColor(Cloud::Ptr cloud_in,
                         ColorCloud::Ptr cloud_out,
                         cv::Mat& img,
                         const cv::Mat& R,
                         const cv::Mat& T,
                         const cv::Mat& cameraMat,
                         const cv::Mat& distCoeff,
                         const cv::Size& imageSize);
