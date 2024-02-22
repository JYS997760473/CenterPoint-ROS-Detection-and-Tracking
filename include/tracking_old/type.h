#pragma once

#include <map>
#include <string>

typedef uint32_t IdType;
#define ID_MAX UINT_MAX

typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;
typedef PointCloud::ConstPtr PointCloudConstPtr;

typedef pcl::PointXYZI PointI;
typedef pcl::PointCloud<PointI> PointICloud;
typedef PointICloud::Ptr PointICloudPtr;
typedef PointICloud::ConstPtr PointICloudConstPtr;

typedef pcl::PointNormal PointN;
typedef pcl::PointCloud<PointN> PointNCloud;
typedef PointNCloud::Ptr PointNCloudPtr;
typedef PointNCloud::ConstPtr PointNCloudConstPtr;

typedef pcl::Normal Normal;
typedef pcl::PointCloud<Normal> NormalCloud;
typedef NormalCloud::Ptr NormalCloudPtr;
typedef NormalCloud::ConstPtr NormalCloudConstPtr;

/// @brief Object classify type
typedef enum {
  CAR = 0,  ///< Generic objects or objects not having a strong classification
  TRUCK,
  CONSTRUCTION_VEHICLE,
  BUS,
  TRAILER,
  BARRIER,
  MOTORCYCLE,  ///< Noise, false objects or objects that we do not need to care
  BICYCLE,
  PEDESTRIAN,
  TRAFFIC_CONE,
  UNKNOWN,
  NUM_OF_CLASSES
} ObjectType;

static const std::map<std::string, ObjectType> classes_string2type = {{"car", CAR},
                                                                      {"truck", TRUCK},
                                                                      {"construction_vehicle", CONSTRUCTION_VEHICLE},
                                                                      {"bus", BUS},
                                                                      {"trailer", TRAILER},
                                                                      {"barrier", BARRIER},
                                                                      {"motorcycle", MOTORCYCLE},
                                                                      {"bicycle", BICYCLE},
                                                                      {"pedestrian", PEDESTRIAN},
                                                                      {"traffic_cone", TRAFFIC_CONE}};

// {"car", "truck", "construction_vehicle", "bus", "trailer", "barrier", "motorcycle", "bicycle", "pedestrian",
// "traffic_cone"};

struct TrackingWorkerParams {
    //----------------- Matcher: tracker<->observed object association
    std::string matcher_method_name = "hungarian_matcher";
    float matcher_match_distance_maximum = 4.0;
    float matcher_location_distance_weight = 0.6f;
    float matcher_direction_distance_weight = 0.2f;
    float matcher_bbox_size_distance_weight = 0.1f;
    float matcher_point_num_distance_weight = 0.1f;
    float matcher_histogram_distance_weight = 0.5f;

    //----------------- Tracker
    // Tracker Filter setup
    std::string filter_method_name = "robust_kalman_filter";
    bool filter_use_adaptive = false;
    double filter_association_score_maximum = matcher_match_distance_maximum;
    float filter_measurement_noise = 0.4f;
    float filter_initial_velocity_noise = 5.0f;
    float filter_xy_propagation_noise = 10.0f;
    float filter_z_propagation_noise = 10.0f;
    float filter_breakdown_threshold_maximum = 10.0;
    // Basic Tracker setup
    int tracker_cached_history_size_maximum = 5;
    int tracker_consecutive_invisible_maximum = 3;
    float tracker_visible_ratio_minimum = 0.6;
    float tracker_acceleration_noise_maximum = 5;
    float tracker_speed_noise_maximum = 0.4;

    //----------------- Tracking Objects collect conditions
    bool tracking_use_histogram_for_match = false;
    float tracking_histogram_bin_size = 10.;
    int tracking_collect_age_minimum = 1;
    int tracking_collect_consecutive_invisible_maximum = 0;
};  // struct TrackingWorkerParams
