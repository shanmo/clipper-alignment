/**
 * @file node.cpp
 * @brief ROS entry point for CLIPPER-based loop-closure detection
 * @author Kaveh Fathian <kavehfathian@gmail.com>
 * @author Parker Lusk <plusk@mit.edu>
 * @date October 2021
 */

#include <ros/ros.h>
#include "aligner/aligner.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "aligner");
  ros::NodeHandle nhtopics("");
  ros::NodeHandle nhparams("~");
  clipperlcd::Aligner aligner(nhtopics, nhparams);
  ros::spin();
  return 0;
}