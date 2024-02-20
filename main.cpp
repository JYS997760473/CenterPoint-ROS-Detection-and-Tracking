#include <fstream>
#include <iostream>
#include <sstream>

#include <ros/ros.h>

#include "centerpoint/centerpoint_detection.h"

#include "cuda_runtime.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "centerpoint_detection");
  std::string model_file = argv[1];
  CenterPointDetection cpd(model_file);
  ros::spin();
  return 0;
}
