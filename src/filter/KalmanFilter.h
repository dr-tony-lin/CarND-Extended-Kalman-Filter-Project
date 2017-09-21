#ifndef FILTER_KALMAN_FILTER_H_
#define FILTER_KALMAN_FILTER_H_

#include "../sensor/MeasurementPackage.h"
#include "../sensor/SensorType.h"
#include "Eigen/Dense"

class KalmanFilter {
 private:
  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // state transition matrix
  Eigen::MatrixXd F_;

  // process covariance matrix
  Eigen::MatrixXd Q_;

  // measurement matrix
  Eigen::MatrixXd H_;

  // measurement covariance matrix
  Eigen::MatrixXd R_;

 public:
  /**
   * Reset the filter to start another run
   */
  virtual void Reset() {}

  /**
    * Run the Kalman Filter estimation on the given sensor
    * measurement.
    * @param measurement_pack the MeasurementPackage
    * @return true if the estimate was made, false if the initialization was carried.
    */
  virtual bool ProcessMeasurement(
      const MeasurementPackage<SensorType> &measurement_pack) = 0;

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  virtual void Predict();

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  virtual void Update(const Eigen::VectorXd &z);

  /**
   * Set the state
   * @param value the value to set
   */
  void x(const Eigen::VectorXd value) { x_ = value; };

  /**
   * Get the state
   */
  Eigen::VectorXd &x() { return x_; };

  /**
   * Get the state covariance matrix
   */
  Eigen::MatrixXd &P() { return P_; };

  /**
   * Set the process covariance matrix
   * @param value the value to set
   */
  void Q(const Eigen::MatrixXd value) { Q_ = value; };

  /**
   * Get process covariance matrix
   */
  Eigen::MatrixXd &Q() { return Q_; };

  /**
   * Set the measurement covariance matrix
   * @param value the value to set
   */
  void R(const Eigen::MatrixXd value) { R_ = value; };

  /**
   * Get the measurement covariance matrix
   */
  Eigen::MatrixXd &R() { return R_; };

 protected:
  /**
   * Precomputed identity matrix same size as the state covariance matrix.
   * This is computed when setting the state covariance matrix P in
        P(const Eigen::MatrixXd value)
   */
  Eigen::MatrixXd I_;

  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  /**
   * Set the state covariance matrix
   * @param value the value to set
   */
  void P(const Eigen::MatrixXd value) {
    P_ = value;
    // Also set the identity matrix for the Update operations
    I_ = Eigen::MatrixXd::Identity(P_.rows(), P_.cols());
  };

  /**
   * Set the state transition matrix
   * @param value the value to set
   */
  void F(const Eigen::MatrixXd value) { F_ = value; };

  /**
   * Get the state transition matrix
   */
  Eigen::MatrixXd &F() { return F_; };

  /**
   * Set the measurement matrix
   * @param value the value to set
   */
  void H(const Eigen::MatrixXd value) { H_ = value; };

  /**
   * Get the measurement matrix
   */
  Eigen::MatrixXd &H() { return H_; };
};
#endif /* KALMAN_FILTER_H_ */
