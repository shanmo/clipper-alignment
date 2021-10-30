/**
 * @file aligner.h
 * @brief CLIPPER object-based alignment
 * @author Kaveh Fathian <kavehfathian@gmail.com>
 * @author Parker Lusk <plusk@mit.edu>
 * @date October 2021
 */

#pragma once

#include <memory>
#include <vector>
#include <string> 

#include <ros/ros.h>
#include <tf2_eigen/tf2_eigen.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <clipper/clipper.h>
#include <clipper/invariants/builtins.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Affine3d);

namespace clipperlcd {

  class Aligner
  {
  public:
    Aligner(const ros::NodeHandle& nh, const ros::NodeHandle& nhp);
    ~Aligner() = default;

  //---------- parameters --------------//
  private:
    static constexpr int DIM = 3; // assuming 3D points
    static constexpr int N = 3; // min number of objects required to estimate (R,t)
    
    double clipper_sigma = 3; // sigma for the Guassian weighting
    double clipper_epsilon = 6; // maximum error to consider two associations consistent
    double clipper_mindist = 2; // minimum distance between chosen objects in same map


  private:
    ros::NodeHandle nh_, nhp_;
    ros::Timer tim_aligner_;
    
    std::vector<ros::Subscriber> sub_r_landmarks_;
    std::vector<ros::Subscriber> sub_r_pose_;
    std::vector<ros::Publisher> pubviz_r_map_;
    std::vector<ros::Publisher> pubviz_r_path_;
    std::vector<ros::Publisher> pubviz_r_frame_;
    std::vector<ros::Publisher> pubviz_r_corres_;

    int num_robots = 3; // number of robots
    double aligner_dt_; ///< Period of clipper runs / visualization

    std::vector<bool> r_new_data_; // true when new maps received
    std::vector<bool> r_first_pose_; // has received first pose
    std::vector<std::vector<Eigen::Affine3d>> r_path_wrt_W_; // original paths

    // currently mapped landmarks of each robot
    std::vector<sensor_msgs::PointCloud2> r_landmarks_;

    // brief Problem Geometry
    std::vector<Eigen::Affine3d> T_WRi_; // robots initial position w.r.t world
    std::vector<Eigen::Affine3d> T_WR_; // last received (i.e., current) robot poses
    std::vector<Eigen::Affine3d> T_R1iRii_; // robots' start w.r.t robot1 start : The Thing we want to estimate

    // CLIPPER
    clipper::Association Ain_;
    std::unique_ptr<clipper::CLIPPER<clipper::invariants::EuclideanDistance>> clipper_;    
    
    void align_maps(const uint i);

    // brief ROS callbacks
    void timer_cb(const ros::TimerEvent& event);
  
    // void landmarks_cb(const sensor_msgs::PointCloudConstPtr& msg, const size_t i);
    void landmarks_cb(const sensor_msgs::PointCloud2ConstPtr& msg,  uint i);
    void pose_cb(const geometry_msgs::PoseStampedConstPtr& msg,  uint i);
    
  };

} // ns lcd
