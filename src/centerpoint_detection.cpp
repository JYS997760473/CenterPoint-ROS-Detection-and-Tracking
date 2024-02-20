#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>

#include "centerpoint/centerpoint_detection.h"

std::map<std::string, std::vector<int>> colormap{{"car", {0, 0, 255}},     // dodger blue
                                                 {"truck", {0, 201, 87}},  // 青色
                                                 {"construction_vehicle", {0, 201, 87}},
                                                 {"bus", {160, 32, 240}},
                                                 {"trailer", {3, 168, 158}},
                                                 {"barrier", {255, 0, 0}},
                                                 {"motorcycle", {255, 97, 0}},
                                                 {"bicycle", {30, 0, 255}},
                                                 {"pedestrian", {255, 0, 0}},
                                                 {"traffic_cone", {0, 0, 255}}};

void CenterPointDetection::GetInfo(void) {
  cudaDeviceProp prop;

  int count = 0;
  cudaGetDeviceCount(&count);
  printf("\nGPU has cuda devices: %d\n", count);
  for (int i = 0; i < count; ++i) {
    cudaGetDeviceProperties(&prop, i);
    printf("----device id: %d info----\n", i);
    printf("  GPU : %s \n", prop.name);
    printf("  Capbility: %d.%d\n", prop.major, prop.minor);
    printf("  Global memory: %luMB\n", prop.totalGlobalMem >> 20);
    printf("  Const memory: %luKB\n", prop.totalConstMem >> 10);
    printf("  SM in a block: %luKB\n", prop.sharedMemPerBlock >> 10);
    printf("  warp size: %d\n", prop.warpSize);
    printf("  threads in a block: %d\n", prop.maxThreadsPerBlock);
    printf("  block dim: (%d,%d,%d)\n", prop.maxThreadsDim[0], prop.maxThreadsDim[1], prop.maxThreadsDim[2]);
    printf("  grid dim: (%d,%d,%d)\n", prop.maxGridSize[0], prop.maxGridSize[1], prop.maxGridSize[2]);
  }
  printf("\n");
}

CenterPointDetection::CenterPointDetection(std::string model_file, bool verbose, std::string onnx_file) {
  // initialize PointPillars
  ROS_INFO("Initializing CenterPoint");
  // GPU Info
  GetInfo();

  checkCudaErrors(cudaStreamCreate(&stream_));

  centerpoint_ptr_ = std::make_unique<CenterPoint>(model_file, verbose, onnx_file);
  centerpoint_ptr_->prepare();
  // checkCudaErrors(cudaMalloc((void**)&d_points_, MAX_POINTS_NUM * params_.feature_num * sizeof(float)));
  ROS_INFO("Finish initializing CenterPoint");

  nh_.getParam("lidar_pointcloud_topic", lidar_pointcloud_topic_);
  nh_.getParam("marker_frame_id", marker_frame_id_);
  nh_.getParam("score_thre", score_thre_);
  ROS_INFO_STREAM("lidar topic: " << lidar_pointcloud_topic_);
  points_sub_ = nh_.subscribe(lidar_pointcloud_topic_, 1, &CenterPointDetection::OnPointCloud, this);
  marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/box", 10);
  pub_boxes_ = nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>("/boxes", 1);
}

void CenterPointDetection::OnPointCloud(const sensor_msgs::PointCloud2ConstPtr& msg_ptr) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg_ptr, *pc_ptr);
  // printf("pc number: %ld, %s \n", pc_ptr->size(), msg_ptr->header.frame_id);
  std_msgs::Header header = msg_ptr->header;

  size_t points_num = pc_ptr->size();
  float* points_data = nullptr;
  // unsigned int points_data_size = points_size * 4 * sizeof(float);
  unsigned int length = points_num * params_.feature_num * sizeof(float);
  std::unique_ptr<std::vector<float>> point_vector_ptr = pclToArray(pc_ptr);
  checkCudaErrors(cudaMallocManaged((void**)&points_data, length));
  checkCudaErrors(cudaMemcpy(points_data, point_vector_ptr->data(), length, cudaMemcpyDefault));
  // checkCudaErrors(cudaMemcpy(d_points_, (void*)point_vector_ptr->data(), length, cudaMemcpyDefault));
  checkCudaErrors(cudaDeviceSynchronize());

  // cudaEventRecord(start_, stream_);

  // centerpoint_ptr_->doinfer((void*)d_points_, points_num, stream_);
  centerpoint_ptr_->doinfer((void*)points_data, points_num, stream_);
  checkCudaErrors(cudaFree(points_data));

  // cudaEventRecord(stop_, stream_);
  // cudaEventSynchronize(stop_);
  // cudaEventElapsedTime(&elapsedTime_, start_, stop_);
  // std::cout << "TIME: pointpillar: " << elapsedTime_ << " ms." << std::endl;

  // checkCudaErrors(cudaFree(points_data));

  // std::cout << "Bndbox objs: " << centerpoint_ptr_->nms_pred_.size() << std::endl;

  publishObjectsMarkers(centerpoint_ptr_->nms_pred_);
  // nms_pred_.clear();
}

std::unique_ptr<std::vector<float>> CenterPointDetection::pclToArray(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& pc_ptr) {
  std::unique_ptr<std::vector<float>> points_array(new std::vector<float>(pc_ptr->size() * s_num_point_feature_));
  const float norm_factor = 1.F / s_normalize_intensity_value_;

  for (size_t i = 0; i < pc_ptr->size(); i++) {
    const pcl::PointXYZI& point = pc_ptr->points[i];
    (*points_array)[i * s_num_point_feature_ + 0] = point.x;
    (*points_array)[i * s_num_point_feature_ + 1] = point.y;
    (*points_array)[i * s_num_point_feature_ + 2] = point.z;
    (*points_array)[i * s_num_point_feature_ + 3] = point.intensity * norm_factor;  // intensity range: 0-255
    (*points_array)[i * s_num_point_feature_ + 4] = 0.0;                            // ring index
  }

  return std::move(points_array);
}

void CenterPointDetection::publishObjectsMarkers(const std::vector<Bndbox>& bboxes) {
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker clear_marker;
  clear_marker.header.frame_id = marker_frame_id_;
  clear_marker.header.stamp = ros::Time::now();
  clear_marker.ns = "/boxes";
  clear_marker.id = 0;
  clear_marker.action = visualization_msgs::Marker::DELETEALL;
  clear_marker.lifetime = ros::Duration();
  marker_array.markers.push_back(clear_marker);
  for (const auto& box : bboxes) {
    if (box.score < score_thre_) {
      continue;
    }
    // Extract box parameters
    double x = box.x;
    double y = box.y;
    double z = box.z;
    double dx = box.w;
    double dy = box.l;
    double dz = box.h;
    double yaw = -box.rt;
    // printf("x: %f, y: %f, z: %f, w: %f, l: %f, h: %f \n", x, y, z, dx, dy, dz);

    // Create a marker for the bounding box
    visualization_msgs::Marker marker;
    marker.header.frame_id = marker_frame_id_;  // Assuming your frame is named "base_link"
    marker.header.stamp = ros::Time::now();
    marker.ns = "/bounding_boxes";
    id_++;
    marker.id = id_;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    tf2::Quaternion quat;
    quat.setRPY(0, 0, yaw);
    marker.pose.orientation = tf2::toMsg(quat);
    marker.scale.x = dx;
    marker.scale.y = dy;
    marker.scale.z = dz;
    std::string label = params_.class_name[box.id];
    marker.color.r = colormap[label][0];
    marker.color.g = colormap[label][1];
    marker.color.b = colormap[label][2];
    marker.color.a = 0.8;  // Set alpha to make the marker semi-transparent

    marker_array.markers.push_back(marker);

    // visualization_msgs::Marker textMarker;
    // textMarker.header.frame_id = "lidar_top";
    // textMarker.header.stamp = ros::Time::now();
    // textMarker.ns = "text_marker";
    // textMarker.id = 0;
    // textMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    // textMarker.action = visualization_msgs::Marker::ADD;
    // textMarker.pose.position.x = marker.pose.position.x;
    // textMarker.pose.position.y = marker.pose.position.y;
    // textMarker.pose.position.z = marker.pose.position.z + marker.scale.z + 0.1;  // Adjust z position
    // textMarker.pose.orientation.w = 1.0;
    // textMarker.scale.z = 0.1;
    // textMarker.color.r = 1.0;
    // textMarker.color.g = 1.0;
    // textMarker.color.b = 1.0;
    // textMarker.color.a = 1.0;
    // textMarker.text = label;  // Your text goes here

    // marker_array.markers.push_back(textMarker);
  }
  marker_pub_.publish(marker_array);

  // jsk_recognition_msgs::BoundingBoxArrayPtr lidar_boxes(new jsk_recognition_msgs::BoundingBoxArray);
  // lidar_boxes->boxes.clear();
  // for (const auto& box : bboxes) {
  //   if (box.score < score_thre_) {
  //     continue;
  //   }
  //   // Extract box parameters
  //   double x = box.x;
  //   double y = box.y;
  //   double z = box.z;
  //   double dx = box.l;
  //   double dy = box.w;
  //   double dz = box.h;
  //   double yaw = -box.rt;
  //   // printf("x: %f, y: %f, z: %f, w: %f, l: %f, h: %f \n", x, y, z, dx, dy, dz);

  //   jsk_recognition_msgs::BoundingBox b;
  //   b.pose.position.x = x;
  //   b.pose.position.y = y;
  //   b.pose.position.z = z;
  //   // b.pose.orientation =
  //   tf2::Quaternion quat;
  //   quat.setRPY(0, 0, yaw);
  //   b.pose.orientation = tf2::toMsg(quat);
  //   b.dimensions.x = box.w;
  //   b.dimensions.y = box.l;
  //   b.dimensions.z = box.h;
  //   b.label = box.id;
  //   b.header.frame_id = "lidar_top";
  //   b.header.stamp = ros::Time::now();
  //   lidar_boxes->boxes.emplace_back(b);
  // }

  // lidar_boxes->header.frame_id = "lidar_top";
  // lidar_boxes->header.stamp = ros::Time::now();
  // pub_boxes_.publish(*lidar_boxes);
}

CenterPointDetection::~CenterPointDetection() {
  checkCudaErrors(cudaFree(d_points_));
  checkCudaErrors(cudaStreamDestroy(stream_));
}
