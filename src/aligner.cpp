/**
 * @file aligner.cpp
 * @brief CLIPPER object-based alignment
 * @author Kaveh Fathian <kavehfathian@gmail.com>
 * @author Parker Lusk <plusk@mit.edu>
 * @date October 2021
 */

#include "aligner/aligner.h"

// #include <functional>
#include <chrono>
#include <cmath>

#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/Marker.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace clipperlcd {

Aligner::Aligner(const ros::NodeHandle& nh, const ros::NodeHandle& nhp)
: nh_(nh), nhp_(nhp)
{
  // get number of robots
  nhp_.getParam("num_robots", num_robots);
  ROS_INFO_STREAM("num_robots: " << num_robots);
  if (num_robots==0) return;
  // clipper params
  nhp_.getParam("clipper_sigma", clipper_sigma);
  ROS_INFO_STREAM("clipper_sigma: " << clipper_sigma);
  nhp_.getParam("clipper_epsilon", clipper_epsilon);
  ROS_INFO_STREAM("clipper_epsilon: " << clipper_epsilon);
  nhp_.getParam("clipper_mindist", clipper_mindist);
  ROS_INFO_STREAM("clipper_mindist: " << clipper_mindist);

  // initializers
  sub_r_landmarks_.resize(num_robots);
  sub_r_pose_.resize(num_robots);
  pubviz_r_path_.resize(num_robots);
  pubviz_r_frame_.resize(num_robots);
  pubviz_r_map_.resize(num_robots);
  pubviz_r_corres_.resize(num_robots);
  r_path_wrt_W_.resize(num_robots);
  r_landmarks_.resize(num_robots);

  // initialize
  r_new_data_.resize(num_robots, false);
  r_first_pose_.resize(num_robots, false);
  // std::fill(r_new_data_.begin(), r_new_data_.end(), false);
  // std::fill(r_first_pose_.begin(), r_first_pose_.end(), false);
  // initialize geometry
  T_WRi_.resize(num_robots);
  T_WR_.resize(num_robots);
  for (size_t i=0; i<num_robots; ++i)
  {
    T_WRi_[i].setIdentity();
    T_WR_[i].setIdentity();
  }

  // set initial relative transform to something arbitrary (for visualization)
  T_R1iRii_.resize(num_robots);
  T_R1iRii_[0].setIdentity();
  for (size_t i=1; i<num_robots; ++i)
  {
    T_R1iRii_[i].setIdentity();
    T_R1iRii_[i].translation() = Eigen::Vector3d(i, 0, 0);
    Eigen::Quaterniond q;
    q = Eigen::AngleAxisd(num_robots*M_PI/i, Eigen::Vector3d::UnitZ());
    T_R1iRii_[i].linear() = q.toRotationMatrix();
  }
  

  // CLIPPER Setup
  clipper::Params params;
  clipper::invariants::EuclideanDistance::Params iparams;
  iparams.sigma = clipper_sigma; // sigma for the Guassian weighting
  iparams.epsilon = clipper_epsilon; // maximum error to consider two associations consistent
  iparams.mindist = clipper_mindist; // minimum distance between chosen objects in same map
  clipper_.reset(new clipper::CLIPPER<clipper::invariants::EuclideanDistance>(params, iparams));


  for (size_t i=0; i<num_robots; ++i)
  {
    boost::function<void(const sensor_msgs::PointCloud2ConstPtr&)> cb1 =
      [=](const sensor_msgs::PointCloud2ConstPtr& msg) {landmarks_cb(msg, i);};

    sub_r_landmarks_[i] = nh_.subscribe(
      "/robot"+std::to_string(i+1)+"/landmarks", 1, cb1); 

    // sub_r_landmarks_[i] = nh_.subscribe(
    //   "/robot"+std::to_string(i+1)+"/landmarks", 1, 
    //   boost::bind(&Aligner::landmarks_cb, this, _1, i)); 

    boost::function<void(const geometry_msgs::PoseStampedConstPtr&)> cb2 =
      [=](const geometry_msgs::PoseStampedConstPtr& msg) {pose_cb(msg, i);};

    sub_r_pose_[i] = nh_.subscribe("robot"+std::to_string(i+1)+"/pose", 1, cb2);

    pubviz_r_path_[i] = nh_.advertise<nav_msgs::Path>(
      "robot"+std::to_string(i+1)+"/viz/path", 1);

    pubviz_r_frame_[i] = nh_.advertise<geometry_msgs::PoseStamped>(
      "robot"+std::to_string(i+1)+"/viz/frame", 1);

    pubviz_r_map_[i] = nh_.advertise<sensor_msgs::PointCloud2>(
      "robot"+std::to_string(i+1)+"/landmarks_aligned", 1);
  }

  // correspondence output, numbered as 2,...,num_robots
  for (size_t i=1; i<num_robots; ++i)
  {
    pubviz_r_corres_[i] = nh_.advertise<visualization_msgs::MarkerArray>(
      "viz/correspondences"+std::to_string(i+1), 1);
  }

  // main loop timer for viz and performing loop-closure detection
  nhp_.param<double>("aligner_dt", aligner_dt_, 0.2);
  tim_aligner_ = nh_.createTimer(ros::Duration(aligner_dt_), &Aligner::timer_cb, this);

}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------
void Aligner::align_maps(uint i) 
{
  // nothing to do if maps aren't big enough to estimate (R,t)
  if (r_landmarks_[0].width < N || r_landmarks_[i].width < N) return;

  pcl::PointCloud<pcl::PointXYZ> r1_cloud, ri_cloud;
  pcl::fromROSMsg(r_landmarks_[0], r1_cloud);
  pcl::fromROSMsg(r_landmarks_[i], ri_cloud);

  Eigen::MatrixXd model = r1_cloud.getMatrixXfMap().topRows(DIM).cast<double>();
  Eigen::MatrixXd data = ri_cloud.getMatrixXfMap().topRows(DIM).cast<double>();

  // identify data assocation
  const auto t1 = std::chrono::high_resolution_clock::now();
  Ain_ = clipper_->findCorrespondences(model, data);
  const auto t2 = std::chrono::high_resolution_clock::now();

  const auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1);
  const double elapsed_ms = static_cast<double>(duration.count()) * 1e-6;

  const int m = model.cols() * data.cols(); // assumes all-to-all hypothesis
  ROS_WARN_STREAM("CLIPPER found " << Ain_.rows() << " out of " << m
    << " associations in " << std::round(elapsed_ms) << " ms");

  // we needed at least N correspondences to estimate (R,t)
  if (Ain_.rows() < N) return;

  // rearrange data so that cols correspond
  Eigen::Matrix3Xd p = Eigen::Matrix3Xd(3, Ain_.rows());
  Eigen::Matrix3Xd q = Eigen::Matrix3Xd(3, Ain_.rows());
  for (size_t i=0; i<Ain_.rows(); ++i) {
    p.col(i) = model.col(Ain_(i,0));
    q.col(i) = data.col(Ain_(i,1));
  }
  // ROS_WARN_STREAM("p: " << p);
  // ROS_WARN_STREAM("q: " << q);

  // find transformation that registers q onto p
  Eigen::Matrix4d T_pq = Eigen::umeyama(q, p, false);

  // update our estimate
  T_R1iRii_[i].translation() = T_pq.block<3,1>(0,3);
  T_R1iRii_[i].linear() = T_pq.block<3,3>(0,0);
}



// ----------------------------------------------------------------------------
// ROS Callbacks
// ----------------------------------------------------------------------------

// void Aligner::landmarks_cb(const sensor_msgs::PointCloudConstPtr& msg, const size_t i)
// {
//   // check if we have received new data
//   // assumption: we receive landmarks seen across all time
//   if (!r_new_data_[i]) r_new_data_[i] = (msg->points.size() > r_landmarks_[i].width);
  
//   // CLEAR is using PointCloud instead of PointCloud2
//   sensor_msgs::PointCloud2 msg2;
//   sensor_msgs::convertPointCloudToPointCloud2(*msg, msg2);

//   // Transform {landmarks w.r.t world} to {landmarks w.r.t robot initial frame}
//   pcl_ros::transformPointCloud(T_WRi_[i].inverse().matrix().cast<float>(), msg2, r_landmarks_[i]);
//   r_landmarks_[i].header.frame_id = "R"+std::to_string(i+1);
// }

// ----------------------------------------------------------------------------

void Aligner::landmarks_cb(const sensor_msgs::PointCloud2ConstPtr& msg, const uint i)
{
  // ROS_WARN_STREAM("i: " << i << ", msg->width: " << msg->width);
  // ROS_WARN_STREAM("i: " << i << ", r_landmarks_[i].width: " << r_landmarks_[i].width);
  
  // check if we have received new data
  // assumption: we receive landmarks seen across all time
  if (!r_new_data_[i]) r_new_data_[i] = (msg->width > r_landmarks_[i].width);

  // Transform {landmarks w.r.t world} to {landmarks w.r.t robot initial frame}
  pcl_ros::transformPointCloud(T_WRi_[i].inverse().matrix().cast<float>(), *(msg), r_landmarks_[i]);
  r_landmarks_[i].header.frame_id = "R"+std::to_string(i+1);

  // ROS_WARN_STREAM("i: " << i << ", r_new_data_[i]: " << r_new_data_[i]);
  // ROS_WARN_STREAM("i: " << i << ", r_landmarks_[i].width: " << r_landmarks_[i].width);
}

// ----------------------------------------------------------------------------
void Aligner::pose_cb(const geometry_msgs::PoseStampedConstPtr& msg, const uint i)
{
  tf2::fromMsg(msg->pose, T_WR_[i]);

  if (!r_first_pose_[i]) {
    tf2::fromMsg(msg->pose, T_WRi_[i]);
    r_first_pose_[i] = true;
  }
}

// ----------------------------------------------------------------------------

void Aligner::timer_cb(const ros::TimerEvent& event)
{
  // don't do anything until received initial robot pose
  if (!r_first_pose_[0]) return;
  
  // Publish Robot 1 Trajectory and Map
  {
    // add the current robot pose to the list
    r_path_wrt_W_[0].push_back(T_WR_[0]);

    nav_msgs::Path r1_path;
    r1_path.header.stamp = ros::Time::now();
    r1_path.header.frame_id = "R1";

    // transform each {pose w.r.t world} to {pose w.r.t R1}
    for (const auto& T_WR1 : r_path_wrt_W_[0]) {
      // find current robot pose w.r.t robot's initial frame
      const auto T_R1iR1 = T_WRi_[0].inverse() * T_WR1;

      geometry_msgs::PoseStamped posemsg;
      posemsg.pose = tf2::toMsg(T_R1iR1);
      r1_path.poses.push_back(posemsg);
    }
    pubviz_r_path_[0].publish(r1_path);

    // publish robot 1 start frame w.r.t robot 1 start frame (i.e., identity)
    geometry_msgs::PoseStamped framemsg;
    framemsg.header.stamp = ros::Time::now();
    framemsg.header.frame_id = "R1";
    framemsg.pose.orientation.w = 1; // identity transform
    pubviz_r_frame_[0].publish(framemsg);

    if (!r_landmarks_[0].header.frame_id.empty()) {
      pubviz_r_map_[0].publish(r_landmarks_[0]);
    }
  }

  
  // Publish Robot i Trajectory and Map (w.r.t R1 initial frame)
  for (size_t i=1; i<num_robots; ++i)
  {
    // don't do anything until received initial robot pose
    if (!r_first_pose_[i]) return;

    if (r_new_data_[0] || r_new_data_[i]) {
      if (r_new_data_[i]) r_new_data_[i] = false; //reset flag
      
      align_maps(i); // find relative transform between robots 1 and i using CLIPPER
    }

    // add the current robot pose to the list
    r_path_wrt_W_[i].push_back(T_WR_[i]);

    nav_msgs::Path ri_path;
    ri_path.header.stamp = ros::Time::now();
    ri_path.header.frame_id = "R1";

    // ROS_WARN_STREAM("T_R1iR2i_: \n" << T_R1iR2i_.matrix());

    // transform each {pose w.r.t world} to {pose w.r.t R1i}
    // using current estimate of T_R1iRii_[i]
    for (const auto& T_WRi : r_path_wrt_W_[i]) {
      // find current robot pose w.r.t robot's initial frame
      const auto T_R2iRi = T_WRi_[i].inverse() * T_WRi;
      // now transform to be w.r.t R1i
      const auto T_R1iRi = T_R1iRii_[i] * T_R2iRi;

      geometry_msgs::PoseStamped posemsg;
      posemsg.pose = tf2::toMsg(T_R1iRi);
      ri_path.poses.push_back(posemsg);
    }
    pubviz_r_path_[i].publish(ri_path);

    // publish robot i start frame w.r.t robot 1 start frame (to be estimated)
    geometry_msgs::PoseStamped framemsg;
    framemsg.header.stamp = ros::Time::now();
    framemsg.header.frame_id = "R1";
    framemsg.pose = tf2::toMsg(T_R1iRii_[i]);
    pubviz_r_frame_[i].publish(framemsg);

    if (!r_landmarks_[i].header.frame_id.empty()) {
      // Transform {landmarks w.r.t R2i} to {landmarks w.r.t R1i}
      sensor_msgs::PointCloud2 msg;
      pcl_ros::transformPointCloud(T_R1iRii_[i].matrix().cast<float>(), r_landmarks_[i], msg);
      msg.header.frame_id = "R1";
      pubviz_r_map_[i].publish(msg);
    }


    // Publish Correspondences
    if (Ain_.rows() >= N && !r_landmarks_[i].header.frame_id.empty()) { 
      visualization_msgs::MarkerArray vizmsg;

      // Transform {landmarks w.r.t Rii} to {landmarks w.r.t R1i}
      pcl::PointCloud<pcl::PointXYZ> r1_cloud, ri_cloud, ri_aligned_cloud;
      pcl::fromROSMsg(r_landmarks_[0], r1_cloud);
      pcl::fromROSMsg(r_landmarks_[i], ri_cloud);
      pcl::transformPointCloud(ri_cloud, ri_aligned_cloud, T_R1iRii_[i].cast<float>());
      
      for (size_t j=0; j<Ain_.rows(); j++) {
        visualization_msgs::Marker m;
        m.header.stamp = ros::Time::now();
        m.header.frame_id = "R1";
        m.id = j;
        m.type = visualization_msgs::Marker::LINE_STRIP;
        m.action = visualization_msgs::Marker::ADD;
        m.color.r = 0;
        m.color.g = 1;
        m.color.b = 0;
        m.color.a = 1;
        m.scale.x = 0.05;
        m.pose.orientation.w = 1;
        // gross
        m.points.push_back(tf2::toMsg(static_cast<Eigen::Vector3d>(
          r1_cloud.getMatrixXfMap().topRows(3).cast<double>().col(Ain_(j,0)))));
        m.points.push_back(tf2::toMsg(static_cast<Eigen::Vector3d>(
          ri_aligned_cloud.getMatrixXfMap().topRows(3).cast<double>().col(Ain_(j,1)))));
        vizmsg.markers.push_back(m);
      }
      pubviz_r_corres_[i].publish(vizmsg);
    }
  }

  r_new_data_[0] = false; //reset flag

}


} // ns lcd