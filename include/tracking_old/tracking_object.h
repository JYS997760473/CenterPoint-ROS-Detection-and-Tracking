#pragma once

#include "tracking/object.h"

struct TrackableObject {
  /* NEED TO NOTICE: All the states of track would be collected mainly based
   * on
   * the states of tracked object. Thus, update tracked object's state when
   * you
   * update the state of track !!! */
  TrackableObject() = default;

  /**
   * @brief init Trackable Object from built Object
   *  inherit built object's ground center/size/direction
   *  compute barycenter/anchor_point
   *  init velocity/acceleration/velocity_uncertainty
   * @param obj_ptr
   */
  explicit TrackableObject(ObjectPtr obj_ptr)
      : object_ptr(obj_ptr) {
    if (object_ptr != nullptr) {
      ground_center = object_ptr->ground_center;
      size = Eigen::Vector3f(object_ptr->length, object_ptr->width, object_ptr->height);
      direction = object_ptr->direction;

      // 初始化重心
      // barycenter = common::geometry::getCloudBarycenter<PointI>(object_ptr->cloud).cast<float>();
      // TODO(gary): need HD Map
      // lane_direction = Eigen::Vector3f::Zero();

      /**
       * @brief initial state
       * @note bary center as anchor point
       */
      // 重心作为锚点
      // anchor_point = barycenter;
      anchor_point = ground_center;
      // velocity = Eigen::Vector3f::Zero();
      velocity = object_ptr->velocity;
      acceleration = Eigen::Vector3f::Zero();

      velocity_uncertainty = Eigen::Matrix3d::Identity() * 5;

      type = object_ptr->type;
    }
  }

  // store transformed object before tracking, cloud...
  ObjectPtr object_ptr;
  // ground center and size
  Eigen::Vector3f ground_center;
  Eigen::Vector3f size;
  Eigen::Vector3f direction;
  // 重心
  // Eigen::Vector3f barycenter;
  // TODO(gary): lane direction needs HD Map
  // Eigen::Vector3f lane_direction;
  // states 每个追踪器追踪的物体状态估计(锚点+速度+加速度)
  Eigen::Vector3f anchor_point;
  Eigen::Vector3f anchor_point_local;
  Eigen::Vector3f orientation;
  Eigen::Vector3f velocity;
  Eigen::Matrix3d velocity_uncertainty;
  Eigen::Vector3f angular_velocity;
  Eigen::Vector3f acceleration;
  // class type
  ObjectType type;
  // association distance, range from 0 to association_score_maximum
  float association_score = 0.0f;
  double orientation_jitter = M_PI;

  // store 4 vertices of 2D bounding box and the flag about if occluded
  // bool set_occl_flag{false};
  // std::array<Eigen::Vector3d, 4> vertices;
};

typedef std::shared_ptr<TrackableObject> TrackableObjectPtr;
typedef std::shared_ptr<const TrackableObject> TrackableObjectConstPtr;
