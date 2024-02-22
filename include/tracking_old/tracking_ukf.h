#pragma once

#include <deque>  // std::deque
#include <vector>

#include <Eigen/Core>

#include "tracking/base_filter.h"


class TrackingUKF : public BaseFilter {
 public:
  enum MODEL { STATIONARY = 0, LOW_SPEED = 1, HIGH_SPEED = 2 };

  static int NUM_STATE;
  static int NUM_STATE_AUG;
  static int NUM_SIGMA_PT;
  static int NUM_SIGMA_PT_AUG;
  static double LAMBDA;
  static double STD_A;
  static double STD_YAWD;  // 0.1;
  static int NUM_OBSV;
  static double STD_MEAS_POS;  // 2.0;//3.75;
  static double STD_MEAS_VEL;  // 0.35
  static double STD_MEAS_YAW;

  TrackingUKF();

  ~TrackingUKF() {}

  // @brief init initialize parameters for kalman filter
  // @params[IN] use_precision_tracking: use precision tracker or not
  // @params[IN] initial_position_noise: initial uncertainty of position(x,y)
  // @params[IN] initial_velocity_noise: initial uncertainty of velocity
  // @params[IN] a_propagation_noise: propagation uncertainty of acceleration
  // @params[IN] yawd_propagation_noise: propagation uncertainty of yawd
  // @params[IN] xy_measure_noise: measure uncertainty of xy
  // @params[IN] velocity_measure_noise: measure uncertainty of velocity
  // @params[IN] yaw_measure_noise: measure uncertainty of taw
  // @return true if set successfully, otherwise return false
  static bool initNoiseParams(const bool& use_precision_tracking, const double& initial_position_noise,
                              const double& initial_velocity_noise, const double& a_propagation_noise,
                              const double& yawd_propagation_noise, const double& xy_measure_noise,
                              const double& velocity_measure_noise, const double& yaw_measure_noise);

  // @brief initialize the state of filter
  // @params[IN] anchor_point: initial anchor point for filtering
  // @params[IN] velocity: initial velocity for filtering
  // @return nothing
  void initState(const Eigen::Vector3f& anchor_point, const Eigen::Vector3f& velocity);

  // @brief predict the state of filter
  // @params[IN] time_diff: time interval for predicting
  // @return predicted states of filtering
  Eigen::VectorXf execPredict(const double& time_diff);

  // @brief update filter with object
  // @params[IN] new_object: recently detected object for current updating
  // @params[IN] old_object: last detected object for last updating
  // @params[IN] time_diff: time interval from last updating
  // @return nothing
  void updateWithObservation(TrackableObjectConstPtr new_object, TrackableObjectConstPtr old_object, const double& time_diff);

  // @brief update filter without object
  // @params[IN] time_diff: time interval from last updating
  // @return nothing
  void updateWithoutObservation(const double& time_diff);

  // @brief get state of filter
  // @params[OUT] anchor_point: anchor point of current state
  // @params[OUT] velocity: velocity of current state
  // @return nothing
  void getState(Eigen::Vector3f* anchor_point, Eigen::Vector3f* velocity);

  // @brief get state of filter with accelaration
  // @params[OUT] anchor_point: anchor point of current state
  // @params[OUT] velocity: velocity of current state
  // @params[OUT] velocity_accelaration: accelaration of curret state
  // @return nothing
  void getState(Eigen::Vector3f* anchor_point, Eigen::Vector3f* velocity, Eigen::Vector3f* acceleration);

  void getAngularState(Eigen::Vector3f* orientation, Eigen::Vector3f* angular_velocity);

  void getStateUncertainty(Eigen::Matrix3d* position_uncertainty, Eigen::Matrix3d* velocity_uncertainty);

  void getAccelerationGain(Eigen::Vector3f* acceleration_gain);

  // @brief get online covariance of filter
  // @params[OUT] online_covariance: online covariance
  // @return noting
  void getOnlineCovariance(Eigen::Matrix3d* online_covariance);

  /**
   * @brief Get the Velocity Uncertainty limit, differs based on the current tracking model
   *
   * @return float velocity uncertainty limit, to be used for static object classification
   */
  float getVelocityUncertaintyLimit() const override;

 private:
  void execAccelerationUpdate(const Eigen::VectorXf& measured_acceleration);
  void execVelocityUpdate(const Eigen::VectorXf& measured_velocity, const Eigen::Matrix3d& measured_velocity_cov,
                          const double& time_diff);
  void evaluateOnlineCovariance();
  void calcBreakdownThreshold();
  void execPropagateNoiseUpdate(const double& time_diff);

  // velocity related calculation
  Eigen::VectorXf calcMeasuredVelocity(TrackableObjectConstPtr new_object, TrackableObjectConstPtr old_object,
                                       const double& time_diff);

  Eigen::VectorXf calcMeasuredAnchorPointVelocity(TrackableObjectConstPtr new_object, TrackableObjectConstPtr old_object,
                                                  const double& time_diff);

  Eigen::VectorXf calcMeasuredBboxCenterVelocity(TrackableObjectConstPtr new_object, TrackableObjectConstPtr old_object,
                                                 const double& time_diff);

  Eigen::VectorXf calcMeasuredBboxCornerVelocity(TrackableObjectConstPtr new_object, TrackableObjectConstPtr old_object,
                                                 const double& time_diff);

  Eigen::VectorXf calcMeasuredBboxCenterMeanVelocity(TrackableObjectConstPtr new_object, TrackableObjectConstPtr old_object,
                                                     const double& time_diff);

  Eigen::Vector3f selectMeasuredVelocity(const std::vector<Eigen::Vector3f>& candidates);

  Eigen::Vector3f selectMeasuredVelocityAccordingMotionConsistency(const std::vector<Eigen::Vector3f>& candidates);

  //---------------------- Adaptive KF update quality ----------------------//
  // @brief compute update quality for adaptive filtering
  // @params[IN] new_object: new object for current updating
  // @params[IN] old_object: old object for last updating
  // @reutrn nothing
  void calcUpdateQuality(TrackableObjectConstPtr new_object, TrackableObjectConstPtr old_object);

  // @brief compute update quality by using association score
  // @params[IN] new_object: new object for current updating
  // @return upate quality according association score
  float calcUpdateQualityAccordingAssociationScore(TrackableObjectConstPtr new_object);

  // @brief compute update quality by using association score
  // @params[IN] old_object: old object for last updaitng
  // @params[IN] new_object: new object for current updating
  // @return update quality according point number change
  float calcUpdateQualityAccordingPointNumChange(TrackableObjectConstPtr new_object, TrackableObjectConstPtr old_object);
  //---------------------- Adaptive KF update quality ----------------------//

  /**
   * @brief decides on a suitable speed measurement covariance value based on
   * measurement speed and acceleration between measurement speed and state velocity
   *
   * @param speed measurement speed
   * @param acceleration acceleration between measurement speed and state velocity
   * @return float returns a suitable speed measurement covariance value
   */
  float calculateSpeedMeasCovariance(const float speed, const float acceleration) const;

  const precision_tracking::Params params_;

  // adaptive
  static bool s_use_adaptive_;
  static bool s_use_precision_tracking_;

  static double s_association_score_maximum_;

  // parameters
  // 预测不确定性的噪声协方差矩阵
  static Eigen::Matrix3d s_propagation_noise_;
  // 状态初始化噪声
  static double s_initial_position_noise_;
  static double s_initial_velocity_noise_;
  static double s_breakdown_threshold_maximum_;

  int age_;
  boost::shared_ptr<precision_tracking::MotionModel> motion_model_;
  boost::shared_ptr<precision_tracking::PrecisionTracker> precision_tracker_;

  /// @note Adaptive on accleration "belief_acceleration_gain_"
  double update_quality_;
  /// @note Adaptive on Kalman gain's norm(square root of Euclidean) threshold
  double breakdown_threshold_;

  // filter states
  Eigen::Vector3f belief_anchor_point_;
  Eigen::Vector3f belief_orientation_;
  Eigen::Vector3f belief_velocity_;
  Eigen::Vector3f belief_angular_velocity_;
  Eigen::Vector3f belief_acceleration_;
  Eigen::Vector3f belief_acceleration_gain_;
  Eigen::Matrix3d velocity_covariance_;
  Eigen::Matrix3d online_velocity_covariance_;

  // ukf variables
  Eigen::VectorXd ukf_x_;
  Eigen::MatrixXd ukf_p_;
  Eigen::MatrixXd ukf_x_sigma_pred_;
  Eigen::VectorXd ukf_x_pred_;
  Eigen::VectorXd ukf_z_pred_;
  Eigen::MatrixXd ukf_s_;
  Eigen::MatrixXd ukf_tc_;
  Eigen::VectorXd ukf_weights_;

  bool ukf_ready;

  // filter history
  std::deque<bool> history_measured_is_valid_;
  std::deque<Eigen::Vector3f> history_measured_ground_center_;
  std::deque<Eigen::Vector3f> history_measured_velocity_;
  std::deque<double> history_time_diff_;
  static size_t s_measurement_cached_history_size_minimum_;
  static size_t s_measurement_cached_history_size_maximum_;
  static double s_measurement_noise_;
  size_t measurement_cached_history_size_;

};  // class PrecisionTrackingUKF
