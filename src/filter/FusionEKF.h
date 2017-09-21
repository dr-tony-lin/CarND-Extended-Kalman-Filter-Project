#ifndef FILTER_FUSIONEKF_H_
#define FILTER_FUSIONEKF_H_

#include <fstream>
#include <string>
#include <vector>
#include "Eigen/Dense"
#include "KalmanFilter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

class FusionEKF : public KalmanFilter {
 protected:
  /**
   * The minimal time threshold that a measurement and its prior measurement
   * are considered happening at different time.
   */
  const float MINIMAL_TIME_THRESHOLD = 0.000001;

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  void UpdateEKF(const VectorXd &z);

  /**
   * Calculate the Jacobian matrix from the state
   * @param state the state vector of [px, py, vx, vy]
   */
  static MatrixXd CalculateJacobian(const VectorXd &state);

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
   * Reset the filter to start another run
   */
  void Reset() { is_initialized = false; }

  /**
    * Run the Kalman Filter estimation on the given sensor
    * measurement.
    * @param measurement_pack the MeasurementPackage
    * @return true if the estimate was made, false if the initialization was performed.
    */
  bool ProcessMeasurement(
      const MeasurementPackage<SensorType> &measurement_pack);

 private:
  // check whether the tracking toolbox was initialized or not (first
  // measurement)
  bool is_initialized;

  // previous timestamp
  long long previous_timestamp;

  // measurement covariance matrix for laser
  MatrixXd R_laser;

  // measurement covariance matrix for radar
  MatrixXd R_radar;

  // measurement matrix for laser
  MatrixXd H_laser;

  // measurement matrix for radar
  MatrixXd Hj;

  // acceleration noise x
  float s_ax;

  // acceleration noise y
  float s_ay;
};

#endif /* FusionEKF_H_ */
