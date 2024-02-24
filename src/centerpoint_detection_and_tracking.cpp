#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>

#include "centerpoint/centerpoint_detection_and_tracking.h"
#include "common/color.hpp"
#include "common/time.hpp"

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

void CenterPointDetectionAndTracking::GetInfo(void) {
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

CenterPointDetectionAndTracking::CenterPointDetectionAndTracking(std::string model_file, bool verbose, std::string onnx_file) {
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
  nh_.getParam("task", task_);
  ROS_INFO_STREAM("lidar topic: " << lidar_pointcloud_topic_);
  points_sub_ = nh_.subscribe(lidar_pointcloud_topic_, 1, &CenterPointDetectionAndTracking::OnPointCloud, this);
  marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/detection", 10);
  tracking_objs_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/tracking", 10);
  pub_boxes_ = nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>("/boxes", 1);

  if (task_ == "detection_and_tracking") {
    // tracking-----------------------------------
    tracking_worker_ = autosense::tracking::createTrackingWorker(tracking_worker_params_);
    if (nullptr == tracking_worker_) {
      ROS_FATAL("Failed to create tracking_worker_.");
      abort();
    }
  }
}

void CenterPointDetectionAndTracking::OnPointCloud(const sensor_msgs::PointCloud2ConstPtr& msg_ptr) {
  const double kTimeStamp = msg_ptr->header.stamp.toSec();
  pcl::PointCloud<pcl::PointXYZI>::Ptr pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg_ptr, *pc_ptr);
  // printf("pc number: %ld, %s \n", pc_ptr->size(), msg_ptr->header.frame_id);
  // std_msgs::Header header = msg_ptr->header;

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

  // detection markers
  publishObjectsMarkers(centerpoint_ptr_->nms_pred_);

  if (task_ == "detection_and_tracking") {
    // tracking------------------------------------
    // build Object instances
    autosense::tracking::TrackingOptions tracking_options;
    std::vector<autosense::ObjectPtr> obsv_objects;
    createObjectFromBndbox(&obsv_objects);
    std::vector<autosense::ObjectPtr> tracking_objects_velo;
    autosense::common::Clock clock_tracking;
    tracking_worker_->track(obsv_objects, kTimeStamp, tracking_options, &tracking_objects_velo);
    ROS_INFO_STREAM("Finish tracking. " << tracking_objects_velo.size() << " Objects Tracked. Took "
                                        << clock_tracking.takeRealTime() << "ms.");
    publishTrackingObjects(tracking_objects_velo);
    std_msgs::Header header;
    header.frame_id = marker_frame_id_;
    header.stamp = msg_ptr->header.stamp;
    publishObjectsMarkersLines(tracking_objs_pub_, header, autosense::common::DARKGREEN.rgbA, tracking_objects_velo);
  }
  // nms_pred_.clear();
}

std::unique_ptr<std::vector<float>> CenterPointDetectionAndTracking::pclToArray(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& pc_ptr) {
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

void CenterPointDetectionAndTracking::publishObjectsMarkers(const std::vector<Bndbox>& bboxes) {
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker clear_marker;
  clear_marker.header.frame_id = marker_frame_id_;
  clear_marker.header.stamp = ros::Time::now();
  clear_marker.ns = "/detection";
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
    marker.color.a = 0.9;  // Set alpha to make the marker semi-transparent

    marker_array.markers.push_back(marker);
  }
  marker_pub_.publish(marker_array);
}

CenterPointDetectionAndTracking::~CenterPointDetectionAndTracking() {
  checkCudaErrors(cudaFree(d_points_));
  checkCudaErrors(cudaStreamDestroy(stream_));
}

void CenterPointDetectionAndTracking::getTrackingWorkerParams() {
  std::string tracking_worker_ns = tracking_ns_ + "/TrackingWorker";
  nh_.getParam(tracking_worker_ns + "/matcher_method_name", tracking_worker_params_.matcher_method_name);
}

void CenterPointDetectionAndTracking::createObjectFromBndbox(std::vector<autosense::ObjectPtr>* objects) {
  if (objects == nullptr) {
    return;
  }
  (*objects).clear();

  for (auto box : centerpoint_ptr_->nms_pred_) {
    if (box.score < score_thre_) {
      continue;
    }
    autosense::ObjectPtr obj(new autosense::Object);
    obj->anchor_point = Eigen::Vector3d(box.x, box.y, box.z);
    obj->direction = Eigen::Vector3d(cos(box.rt), sin(box.rt), 0);
    obj->ground_center = Eigen::Vector3d(box.x, box.y, 0);
    obj->height = box.h;
    obj->length = box.l;
    obj->score = box.score;
    obj->type = static_cast<autosense::ObjectType>(box.id);
    obj->width = box.w;
    obj->yaw_rad = box.rt;
    objects->push_back(obj);
  }
}

void CenterPointDetectionAndTracking::publishTrackingObjects(std::vector<autosense::ObjectPtr>& objs) {
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker clear_marker;
  clear_marker.header.frame_id = marker_frame_id_;
  clear_marker.header.stamp = ros::Time::now();
  clear_marker.ns = "/tracking";
  clear_marker.id = 0;
  clear_marker.action = visualization_msgs::Marker::DELETEALL;
  clear_marker.lifetime = ros::Duration();
  marker_array.markers.push_back(clear_marker);
  for (const auto& obj : objs) {
    // Extract box parameters
    // double x = box.x;
    // double y = box.y;
    // double z = box.z;
    // double dx = box.w;
    // double dy = box.l;
    // double dz = box.h;
    double yaw = obj->yaw_rad;

    // Create a marker for the bounding box
    visualization_msgs::Marker marker;
    marker.header.frame_id = marker_frame_id_;  // Assuming your frame is named "base_link"
    marker.header.stamp = ros::Time::now();
    marker.ns = "/bounding_boxes";
    id_++;
    marker.id = id_;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = obj->anchor_point(0);
    marker.pose.position.y = obj->anchor_point(1);
    marker.pose.position.z = obj->anchor_point(2);
    tf2::Quaternion quat;
    quat.setRPY(0, 0, yaw);
    marker.pose.orientation = tf2::toMsg(quat);
    marker.scale.x = obj->width;
    marker.scale.y = obj->length;
    marker.scale.z = obj->height;
    std::string label = params_.class_name[static_cast<int>(obj->type)];
    marker.color.r = colormap[label][0];
    marker.color.g = colormap[label][1];
    marker.color.b = colormap[label][2];
    marker.color.a = 0.9;  // Set alpha to make the marker semi-transparent

    marker_array.markers.push_back(marker);
    // printf("x: %f, y: %f, z: %f \n", obj->anchor_point(0), obj->anchor_point(1), obj->anchor_point(2));
  }
  tracking_objs_pub_.publish(marker_array);
}

void CenterPointDetectionAndTracking::publishObjectsMarkersLines(const ros::Publisher& publisher,
                                                                 const std_msgs::Header& header,
                                                                 const std_msgs::ColorRGBA& color,
                                                                 const std::vector<autosense::ObjectPtr>& objects_array) {
  // clear all markers before
  visualization_msgs::MarkerArray empty_markers;
  visualization_msgs::Marker clear_marker;
  clear_marker.header = header;
  clear_marker.ns = "objects";
  clear_marker.id = 0;
  clear_marker.action = clear_marker.DELETEALL;
  clear_marker.lifetime = ros::Duration();
  empty_markers.markers.push_back(clear_marker);
  publisher.publish(empty_markers);

  if (objects_array.size() <= 0) {
    ROS_WARN("Publish empty object marker.");
    return;
  } else {
    ROS_INFO("Publishing %lu objects markers.", objects_array.size());
  }

  visualization_msgs::MarkerArray object_markers;
  for (size_t obj = 0u; obj < objects_array.size(); ++obj) {
    /*
     * @note Apollo's Object Coordinate
     *          |x
     *      C   |   D-----------
     *          |              |
     *  y---------------     length
     *          |              |
     *      B   |   A-----------
     */
    Eigen::Vector3d center = objects_array[obj]->ground_center;
    Eigen::Vector3d dir = objects_array[obj]->direction;

    // object size
    const double& length = objects_array[obj]->length;
    const double& width = objects_array[obj]->width;
    const double& height = objects_array[obj]->height;
    const double yaw = objects_array[obj]->yaw_rad;
    Eigen::Vector3d ldir(cos(yaw), sin(yaw), 0);
    Eigen::Vector3d odir(-ldir[1], ldir[0], 0);
    Eigen::Vector3d bottom_quad[8];
    double half_l = length / 2;
    double half_w = width / 2;
    double h = height;
    // A(-half_l, -half_w)
    bottom_quad[0] = center + ldir * -half_l + odir * -half_w;
    // B(-half_l, half_w)
    bottom_quad[1] = center + ldir * -half_l + odir * half_w;
    // C(half_l, half_w)
    bottom_quad[2] = center + ldir * half_l + odir * half_w;
    // D(half_l, -half_w)
    bottom_quad[3] = center + ldir * half_l + odir * -half_w;
    // top 4 vertices
    bottom_quad[4] = bottom_quad[0];
    bottom_quad[4](2) += h;
    bottom_quad[5] = bottom_quad[1];
    bottom_quad[5](2) += h;
    bottom_quad[6] = bottom_quad[2];
    bottom_quad[6](2) += h;
    bottom_quad[7] = bottom_quad[3];
    bottom_quad[7](2) += h;

    Eigen::MatrixXf OBB(8, 3);
    OBB << bottom_quad[0](0), bottom_quad[0](1), bottom_quad[0](2), bottom_quad[1](0), bottom_quad[1](1), bottom_quad[1](2),
        bottom_quad[2](0), bottom_quad[2](1), bottom_quad[2](2), bottom_quad[3](0), bottom_quad[3](1), bottom_quad[3](2),
        bottom_quad[4](0), bottom_quad[4](1), bottom_quad[4](2), bottom_quad[5](0), bottom_quad[5](1), bottom_quad[5](2),
        bottom_quad[6](0), bottom_quad[6](1), bottom_quad[6](2), bottom_quad[7](0), bottom_quad[7](1), bottom_quad[7](2);

    visualization_msgs::Marker box, dir_arrow;
    box.header = dir_arrow.header = header;
    box.ns = dir_arrow.ns = "objects";
    box.id = obj;
    dir_arrow.id = obj + objects_array.size();
    box.type = visualization_msgs::Marker::LINE_LIST;
    dir_arrow.type = visualization_msgs::Marker::ARROW;
    geometry_msgs::Point p[24];
    // Ground side
    // A->B
    p[0].x = OBB(0, 0);
    p[0].y = OBB(0, 1);
    p[0].z = OBB(0, 2);
    p[1].x = OBB(1, 0);
    p[1].y = OBB(1, 1);
    p[1].z = OBB(1, 2);
    // B->C
    p[2].x = OBB(1, 0);
    p[2].y = OBB(1, 1);
    p[2].z = OBB(1, 2);
    p[3].x = OBB(2, 0);
    p[3].y = OBB(2, 1);
    p[3].z = OBB(2, 2);
    // C->D
    p[4].x = OBB(2, 0);
    p[4].y = OBB(2, 1);
    p[4].z = OBB(2, 2);
    p[5].x = OBB(3, 0);
    p[5].y = OBB(3, 1);
    p[5].z = OBB(3, 2);
    // D->A
    p[6].x = OBB(3, 0);
    p[6].y = OBB(3, 1);
    p[6].z = OBB(3, 2);
    p[7].x = OBB(0, 0);
    p[7].y = OBB(0, 1);
    p[7].z = OBB(0, 2);

    // Top side
    // E->F
    p[8].x = OBB(4, 0);
    p[8].y = OBB(4, 1);
    p[8].z = OBB(4, 2);
    p[9].x = OBB(5, 0);
    p[9].y = OBB(5, 1);
    p[9].z = OBB(5, 2);
    // F->G
    p[10].x = OBB(5, 0);
    p[10].y = OBB(5, 1);
    p[10].z = OBB(5, 2);
    p[11].x = OBB(6, 0);
    p[11].y = OBB(6, 1);
    p[11].z = OBB(6, 2);
    // G->H
    p[12].x = OBB(6, 0);
    p[12].y = OBB(6, 1);
    p[12].z = OBB(6, 2);
    p[13].x = OBB(7, 0);
    p[13].y = OBB(7, 1);
    p[13].z = OBB(7, 2);
    // H->E
    p[14].x = OBB(7, 0);
    p[14].y = OBB(7, 1);
    p[14].z = OBB(7, 2);
    p[15].x = OBB(4, 0);
    p[15].y = OBB(4, 1);
    p[15].z = OBB(4, 2);

    // Around side
    // A->E
    p[16].x = OBB(0, 0);
    p[16].y = OBB(0, 1);
    p[16].z = OBB(0, 2);
    p[17].x = OBB(4, 0);
    p[17].y = OBB(4, 1);
    p[17].z = OBB(4, 2);
    // B->F
    p[18].x = OBB(1, 0);
    p[18].y = OBB(1, 1);
    p[18].z = OBB(1, 2);
    p[19].x = OBB(5, 0);
    p[19].y = OBB(5, 1);
    p[19].z = OBB(5, 2);
    // C->G
    p[20].x = OBB(2, 0);
    p[20].y = OBB(2, 1);
    p[20].z = OBB(2, 2);
    p[21].x = OBB(6, 0);
    p[21].y = OBB(6, 1);
    p[21].z = OBB(6, 2);
    // D->H
    p[22].x = OBB(3, 0);
    p[22].y = OBB(3, 1);
    p[22].z = OBB(3, 2);
    p[23].x = OBB(7, 0);
    p[23].y = OBB(7, 1);
    p[23].z = OBB(7, 2);

    for (size_t pi = 0u; pi < 24; ++pi) {
      box.points.push_back(p[pi]);
    }
    box.scale.x = 0.1;
    box.color = color;
    object_markers.markers.push_back(box);

    // direction
    geometry_msgs::Point start_point, end_point;
    Eigen::Vector3d end = center + dir * (length);
    start_point.x = center[0];
    start_point.y = center[1];
    start_point.z = center[2];
    end_point.x = end[0];
    end_point.y = end[1];
    end_point.z = end[2];
    dir_arrow.points.push_back(start_point);
    dir_arrow.points.push_back(end_point);
    dir_arrow.scale.x = 0.1;
    dir_arrow.scale.y = 0.2;
    dir_arrow.scale.z = length * 0.1;
    dir_arrow.color.a = 1.0;
    dir_arrow.color.r = 1.0;
    dir_arrow.color.g = 0.0;
    dir_arrow.color.b = 0.0;

    object_markers.markers.push_back(dir_arrow);
  }

  publisher.publish(object_markers);
}
