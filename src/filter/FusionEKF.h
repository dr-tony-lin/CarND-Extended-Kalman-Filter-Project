#ifndef FILTER_FUSIONEKF_H_
#define FILTER_FUSIONEKF_H_

#include "../sensor/MeasurementPackage.h"
#include "../sensor/SensorType.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "KalmanFilter.h"

class FusionEKF {
public:
  /**
  * Constructor.
  */
  FusionEKF();

  /**
  * Destructor.
  */
  virtual ~FusionEKF();

  /**
   * Reset the filter
   */
  void Reset() {
    is_initialized = false;
  }

  /**
   * Get the state
   */
  Eigen::VectorXd& x() {
    return ekf_.x();
  };

  /**
   * Get the state covariance matrix
   */
  Eigen::MatrixXd& P() {
    return ekf_.P();
  };

  /**
  * Run the whole flow of the Kalman Filter from here.
  */
  void ProcessMeasurement(const MeasurementPackage<SensorType> &measurement_pack);


private:
  // Kalman Filter update and prediction math lives in here.
  KalmanFilter ekf_;

  // check whether the tracking toolbox was initialized or not (first measurement)
  bool is_initialized;

  // previous timestamp
  long long previous_timestamp;

  // measurement covariance matrix for laser
  Eigen::MatrixXd R_laser;

  // measurement covariance matrix for radar
  Eigen::MatrixXd R_radar;

  // measurement matrix for laser
  Eigen::MatrixXd H_laser;

  // measurement matrix for radar
  Eigen::MatrixXd Hj;

  // acceleration noise x
  float s_ax;

  // acceleration noise y
  float s_ay;
};

#endif /* FusionEKF_H_ */
