/*************************************************************************
 @Author: JoeAaron
 @Email: pant333@163.com
 @Created Time : 2019-01-27 10:09:42
 @Last Modified : 2019-01-27 10:09:42
 @File Name: laser_line_extract.cpp
 @Description:
 ************************************************************************/
#include <ros/ros.h>

void cloud_cb(const pcl::PCLPointCloud2ConstPtr& input)
{
    pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2);
    pcl::PassThrough<pcl::PCLPointCloud2> pass_through_x;
    pcl::PassThrough<pcl::PCLPointCloud2> pass_through_y;

    pass_through_x.setInputCloud(input);
    // set fieldname we want to filter over
    pass_through_x.setFilterFieldName ("x");
    // set range for selected field to 1.0 - 1.5 meters
    pass_through_x.setFilterLimits (0.0,1.0);
    // do filtering
    pass_through_x.filter (*cloud_filtered);

    pass_through_y.setInputCloud (cloud_filtered);
    // set fieldname we want to filter over
    pass_through_y.setFilterFieldName ("y");
    // set range for selected field to 1.0 - 1.5 meters
    pass_through_y.setFilterLimits (0.0, 2.0);
    // do filtering
    pass_through_y.filter (*cloud_filtered);

    pub.publish(*cloud_filtered);
    pcl::PointCloud< pcl::PointXYZ> cloud;
    pcl::fromPCLPointCloud2(*cloud_filtered, cloud);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_LINE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.006);
    seg.setInputCloud (cloud.makeShared ());
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a LINE model for the given dataset.");
    }
    angle=2*asin(coefficients->values[4])*180.0/PI;//formulae for quarternions.
    //std::cerr <<"deviation from the wall:"<<angle<<endl;

    tostr << angle;
    text_1.ns = "vehicle_orientation";
    text_1.header.frame_id = "laser";
    text_1.header.stamp = ros::Time::now();
    text_1.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_1.action = visualization_msgs::Marker::ADD;
    text_1.pose.position.x =0.0;// coefficients->values[0];
    text_1.pose.position.y =-0.3;// coefficients->values[1];
    text_1.pose.position.z =0.0;// coefficients->values[2];
    text_1.pose.orientation.x = 0.0;
    text_1.pose.orientation.y = 0.0;
    text_1.pose.orientation.z = 0.0;
    text_1.pose.orientation.w = 1.0;
    text_1.id = 1;
    text_1.text=tostr.str();
    tostr.str("");

    text_1.scale.z = 0.12;
    text_1.color.r = 0.0f;
    text_1.color.g = 1.0f;
    text_1.color.b = 0.0f;
    text_1.color.a = 1.0f;
    text_1.lifetime = ros::Duration();
    marker_pub.publish(text_1);

    marker.ns = "vehicle_orientation";
    marker.header.frame_id = "laser";
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = coefficients->values[0];
    marker.pose.position.y = coefficients->values[1];
    marker.pose.position.z = coefficients->values[2];
    marker.pose.orientation.x = coefficients->values[3];
    marker.pose.orientation.y = coefficients->values[4];
    marker.pose.orientation.z = coefficients->values[5];
    marker.pose.orientation.w = 0.0;
    marker.id = 0;
    marker.scale.x = 0.6;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;
    marker.lifetime = ros::Duration();
    // Publish the marker
    marker_pub.publish(marker);
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "fit_line");
    ros::NodeHandle nh;
    //Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("/scan", 100, cloud_cb);
    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1000);
    //Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2>("/line_filter", 1);

    ros::spin();
}
