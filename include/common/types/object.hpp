/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef COMMON_INCLUDE_COMMON_TYPES_OBJECT_HPP_
#define COMMON_INCLUDE_COMMON_TYPES_OBJECT_HPP_

#include <memory>

#include <Eigen/Core>
#include <pcl/io/io.h>  // pcl::copyPointCloud

#include "common/types/feature.hpp"
#include "common/types/type.h"

namespace autosense {

using Eigen::Vector3d;

typedef PointCloud PolygonType;
typedef PointDCloud PolygonDType;

struct alignas(16) Object {
  Object() {
    ground_center = Vector3d::Zero();
    velocity = Vector3d::Zero();
    direction = Vector3d(1, 0, 0);

    type = UNKNOWN;
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

  /**
   * @brief deep copy of Object
   * @param rhs
   */
  void clone(const Object& rhs) {
    *this = rhs;
  }

  void setId(IdType id) { this->id = id; }

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
  Eigen::Vector3d direction;
  // the yaw angle, theta = 0.0 <=> direction = (1, 0, 0)
  double yaw_rad = 0.0;
  // ground center of the object (cx, cy, z_min)
  Eigen::Vector3d ground_center;
  // size of the oriented bbox, length is the size in the main direction
  double length = 0.0;
  double width = 0.0;
  double height = 0.0;

  //---------------------- classification information
  ObjectType type;
  // foreground score/probability
  float score = 0.0;
  // fg/bg flag
  bool is_background = false;
  // Object classification type.
  // ObjectType type;
  // Probability of each type, used for track type.
  // std::vector<float> type_probs;

  //---------------------- tracking information
  /// @note one tracker maintaina tracked trajectory
  IdType tracker_id = 0;
  // tracking state
  // stable anchor_point during time, e.g., barycenter
  Eigen::Vector3d anchor_point;
  Eigen::Vector3d velocity;
  // age of the tracked object
  double tracking_time = 0.0;
  double latest_tracked_time = 0.0;
  // noise covariance matrix for uncertainty of position and velocity
  Eigen::Matrix3d position_uncertainty;
  Eigen::Matrix3d velocity_uncertainty;
};

typedef std::shared_ptr<Object> ObjectPtr;
typedef std::shared_ptr<const Object> ObjectConstPtr;

}  // namespace autosense

#endif  // COMMON_INCLUDE_COMMON_TYPES_OBJECT_HPP_
