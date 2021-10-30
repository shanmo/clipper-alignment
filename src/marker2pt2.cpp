/**
 * @file marker2pt2.cpp
 * @brief convert markerarray to pointcloud2 messages
 * @author Kaveh Fathian <kavehfathian@gmail.com>
 * @date October 2021
 */

#include <ros/ros.h>
#include "aligner/marker2pt2.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

Converter::Converter(const ros::NodeHandle& nh, const ros::NodeHandle& nhp)
: nh_(nh), nhp_(nhp)
{
    sub_markerarray = nh_.subscribe("/robot/marker_array", 1, &Converter::markerarray_cb, this);
    pub_pointcloud2 = nh_.advertise<sensor_msgs::PointCloud2>("/robot/landmarks", 1);

    nhp_.param<double>("converter_dt", converter_dt_, 0.02);
    tim_converter_ = nh_.createTimer(ros::Duration(converter_dt_), &Converter::timer_cb, this);
}

// ---------------------------------------------------------------------------
// convert Marker to PointCloud2 messages
void Converter::convertMarkerToPointCloud2(const visualization_msgs::MarkerArray& msg,
                                                 sensor_msgs::PointCloud2& msg2)
{
  // number of landmarks/objects
  // assumption: we receive landmarks seen across all time
  int num_landmarks = msg.markers.size(); 
//   ROS_INFO_STREAM("number of landmakrs: " << num_landmarks);

  // create point cloud object
  pcl::PointCloud<pcl::PointXYZ> ptcloud;
  for (size_t i=0; i < num_landmarks; ++i) 
  {
    pcl::PointXYZ point;
    point.x = msg.markers[i].pose.position.x;
    point.y = msg.markers[i].pose.position.y;
    point.z = msg.markers[i].pose.position.z;

    ptcloud.push_back(point);
    // ROS_INFO_STREAM("point: " << point);
  }

  // Convert to ROS PointCloud2 data type
  pcl::PCLPointCloud2 ptcloud2;
  pcl::toPCLPointCloud2(ptcloud, ptcloud2);
  pcl_conversions::fromPCL(ptcloud2, msg2);

  // ROS_INFO_STREAM("ptcloud2.width: " << ptcloud2.width);
  // ROS_INFO_STREAM("msg2.width: " << msg2.width);
}

// ----------------------------------------------------------------------------
// ROS Callbacks
// ----------------------------------------------------------------------------
void Converter::markerarray_cb(const visualization_msgs::MarkerArray& msg)
{
    // convert MarkerArray to PointCloud2    
    // ROS_INFO_STREAM("converting message to pointcloud2");
    convertMarkerToPointCloud2(msg, msg2); 

    // check if we have received new data
    // assumption: we receive landmarks seen across all time
    new_data_ = (msg2.width > landmarks_.width);
    
    // ROS_INFO_STREAM("msg2.width: " << msg2.width);
    // ROS_INFO_STREAM("landmarks_.width: " << landmarks_.width);
    // ROS_INFO_STREAM("new_data_: " << new_data_);

    landmarks_ = msg2;
}


// ----------------------------------------------------------------------------
void Converter::timer_cb(const ros::TimerEvent& event)
{
    if (new_data_)
    {
        pub_pointcloud2.publish(msg2);
    }
    
}


// ----------------------------------------------------------------------------
// ROS node
// ----------------------------------------------------------------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "converter");
    ros::NodeHandle nhtopics("");
    ros::NodeHandle nhparams("~");

    Converter converter(nhtopics, nhparams);

    ros::spin();
    return 0;
}

