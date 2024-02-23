#pragma once

#include <fstream>
#include <iostream>
#include <sstream>

#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>

#include <std_msgs/ColorRGBA.h>

#include "centerpoint/centerpoint.h"
#include "centerpoint/common.h"
#include "common/types/type.h"
#include "tracking/base_tracking_worker.h"
#include "tracking/tracking_worker_manager.hpp"

#include "cuda_runtime.h"

class CenterPointDetection {
 public:
  CenterPointDetection(std::string model_file, bool verbose, std::string onnx_file);
  ~CenterPointDetection();

 private:
  ros::NodeHandle nh_;
  ros::Subscriber points_sub_;
  ros::Publisher marker_pub_;

  std::unique_ptr<CenterPoint> centerpoint_ptr_;
  std::string marker_frame_id_;
  std::string lidar_pointcloud_topic_;
  ros::Publisher pub_boxes_;
  ros::Publisher pub_texts_;
  ros::Publisher tracking_objs_pub_;
  std::string tracking_ns_ = "tracking";
  autosense::TrackingWorkerParams tracking_worker_params_;
  std::unique_ptr<autosense::tracking::BaseTrackingWorker> tracking_worker_;

  Params params_;
  cudaStream_t stream_ = NULL;
  float* d_points_ = nullptr;

  std::vector<Bndbox> nms_pred_;
  int id_ = 0;
  float score_thre_;

  constexpr static int s_num_point_feature_{5};
  constexpr static float s_normalize_intensity_value_{255.F};

 private:
  void OnPointCloud(const sensor_msgs::PointCloud2ConstPtr& msg_ptr);
  void GetInfo(void);
  std::unique_ptr<std::vector<float>> pclToArray(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& pc_ptr);
  void publishObjectsMarkers(const std::vector<Bndbox>& bboxes);
  void getTrackingWorkerParams();
  void createObjectFromBndbox(std::vector<autosense::ObjectPtr>* objects);
  void publishTrackingObjects(std::vector<autosense::ObjectPtr>& objs);
  void publishObjectsMarkersLines(const ros::Publisher& publisher, const std_msgs::Header& header,
                                  const std_msgs::ColorRGBA& color, const std::vector<autosense::ObjectPtr>& objects_array);
};
