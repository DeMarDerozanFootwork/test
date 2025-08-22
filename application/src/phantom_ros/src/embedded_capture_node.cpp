/*******************************************************************************
* @file    embedded_capture_node.cpp
* @date    10/30/2019
*
* @attention Copyright (c) 2019
* @attention Phantom AI, Inc.
* @attention All rights reserved.
*******************************************************************************/
#include "ros/ros.h"
#include <phantom_ros/phantom_ros_nodelet_loader.h>

#include <map>
#include <string>
#include <vector>

int main(int argc, char **argv) {
  ros::init(argc, argv, "embedded_capture_node");
  phantom_ros::Loader nodelet;
  std::map<std::string, std::string> remap(ros::names::getRemappings());
  std::vector<std::string> nargv(argv, argv + argc);
  std::string nodelet_name = ros::this_node::getName();
  nodelet.load(nodelet_name, "phantom_ros/embeddedCaptureNodelet", remap, nargv);
  ros::spin();

  return 0;
}
