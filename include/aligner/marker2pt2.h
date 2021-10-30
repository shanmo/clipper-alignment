/**
 * @file marker2pt2.h
 * @brief convert markerarray to pointcloud2 messages
 * @author Kaveh Fathian <kavehfathian@gmail.com>
 * @date October 2021
 */

#pragma once

#include <memory>
#include <vector>
#include <string> 

#include <ros/ros.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Affine3d);

class Converter
{
 public:
 Converter(const ros::NodeHandle& nh, const ros::NodeHandle& nhp);
 ~Converter() = default;

 private:
    ros::NodeHandle nh_, nhp_;
    ros::Timer tim_converter_;    

    ros::Subscriber sub_markerarray;
    ros::Publisher pub_pointcloud2;

    double converter_dt_; // Period of runs
    
    sensor_msgs::PointCloud2 msg2; // landmarks from transforming MarkerArray into PT2
    sensor_msgs::PointCloud2 landmarks_; // currently mapped landmarks 

    bool new_data_ = false; //< true when new data received

    void convertMarkerToPointCloud2(const visualization_msgs::MarkerArray& msg,
                                          sensor_msgs::PointCloud2& msg2);
    void markerarray_cb(const visualization_msgs::MarkerArray& msg);
    void timer_cb(const ros::TimerEvent& event);
                                              

};
