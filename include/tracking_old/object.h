#pragma once

#include <memory>
#include <string>

#include <Eigen/Core>

#include "tracking/type.h"

using Eigen::Vector3d;

// struct Object;
typedef std::shared_ptr<Object> ObjectPtr;
typedef std::shared_ptr<const Object> ObjectConstPtr;

struct Object {
  /**
   * frame_id: lidar_top
   */
  Object() {
    ground_center = Vector3d::Zero();
    velocity = Vector3d::Zero();
    direction = Vector3d(1, 0, 0);

    type = ObjectType::UNKNOWN;
    // type_probs.resize(MAX_OBJECT_TYPE, 0);

    /*
     *
     * | 0.01  0    0   |
     * |  0   0.01  0   |
     * |  0    0   0.01 |
     */
    position_uncertainty << 0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01;
    velocity_uncertainty << 0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01;
  }

  void setId(uint32_t id) { this->id = id; }

  void invalidate() { is_valid = false; }

  void setValid(bool is_valid) { this->is_valid = is_valid; }

  bool isValid() const { return is_valid; }

  /**
     * @brief deep copy of Object
     * @param rhs
     */
    void clone(const Object& rhs) {
        *this = rhs;
        // TODO(gary): pay attention to point when deep copy
        // this->cloud.reset(new PointICloud);
        // pcl::copyPointCloud<PointI, PointI>(*(rhs.cloud), *cloud);
    }

  //---------------------- basic information
  // object id per frame
  IdType id = -1;

  /*
   * @note Apollo's Object Coordinate
   *          |x
   *      C   |   D-----------
   *          |              |
   *  y---------------     length
   *          |              |
   *      B   |   A-----------
   */
  // oriented boundingbox information: main direction vector(x, y, 0)
  Eigen::Vector3d direction;  // direction of the boundingbox
  // the yaw angle, theta = 0.0 <=> direction = (1, 0, 0)
  double yaw_rad = 0.0;
  // ground center of the object (cx, cy, z_min)
  Eigen::Vector3d ground_center;
  // size of the oriented bbox, length is the size in the main direction
  double length = 0.0;
  double width = 0.0;
  double height = 0.0;

  //---------------------- classification information
  ObjectType type{UNKNOWN};
  // foreground score/probability
  float score = 0.0;
  // fg/bg flag
  bool is_static = false;
  double static_prob = 0.5;
  double orientation_jitter = M_PI;
  // Probability of each type, used for track type.
  // std::vector<float> type_probs;

  //---------------------- tracking information
  /// @note one tracker maintaina tracked trajectory
  uint32_t tracker_id = 0;
  // tracking state
  // stable anchor_point during time, e.g., barycenter
  Eigen::Vector3d anchor_point;
  Eigen::Vector3d orientation;  // velocity's direction
  Eigen::Vector3d velocity;
  Eigen::Vector3d angular_velocity;
  Eigen::Vector3d acceleration;
  // age of the tracked object
  double tracking_time = 0.0;
  double latest_tracked_time = 0.0;
  // noise covariance matrix for uncertainty of position and velocity
  Eigen::Matrix3d position_uncertainty;
  Eigen::Matrix3d velocity_uncertainty;

  float association_score = 0.0;

  bool is_valid{true};  ///< Whether the object is valid or not
  std::string debug_info;
};
