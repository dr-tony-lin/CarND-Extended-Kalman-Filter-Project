#ifndef FILTER_FUSIONEKF_H_
#define FILTER_FUSIONEKF_H_

#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "KalmanFilter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

class FusionEKF: public KalmanFilter {
protected:
  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
   void UpdateEKF(const VectorXd &z);
   static MatrixXd CalculateJacobian(const VectorXd &x_state);
   
public:
  /**
  * Constructor.
  */
  FusionEKF();

  /**
  * Destructor.
  */
  virtual ~FusionEKF();

  void Reset() {
    is_initialized = false;
  }
  
  /**
  * Run the whole flow of the Kalman Filter from here.
  */
  void ProcessMeasurement(const MeasurementPackage<SensorType> &measurement_pack);

private:
  // check whether the tracking toolbox was initialized or not (first measurement)
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
