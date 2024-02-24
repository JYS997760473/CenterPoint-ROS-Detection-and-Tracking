#include <fstream>
#include <iostream>
#include <sstream>

#include <ros/ros.h>

#include "centerpoint/centerpoint_detection_and_tracking.h"

#include "cuda_runtime.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "centerpoint_detection_and_tracking");
  std::string model_file = argv[1];
  std::string onnx_file = argv[2];
  CenterPointDetectionAndTracking cpd(model_file, false, onnx_file);
  ros::spin();
  return 0;
}
