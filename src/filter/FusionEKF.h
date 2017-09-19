#ifndef FusionEKF_H_
#define FusionEKF_H_

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
  * Run the whole flow of the Kalman Filter from here.
  */
  void ProcessMeasurement(const MeasurementPackage<SensorType> &measurement_pack);

  /**
  * Kalman Filter update and prediction math lives in here.
  */
  KalmanFilter ekf;

private:
  // check whether the tracking toolbox was initialized or not (first measurement)
  bool is_initialized;

  // previous timestamp
  long long previous_timestamp;

  Eigen::MatrixXd R_laser;
  Eigen::MatrixXd R_radar;
  Eigen::MatrixXd H_laser;
  Eigen::MatrixXd Hj;
  float s_ax; // acceleration noise x
  float s_ay; // acceleration noise y
};

#endif /* FusionEKF_H_ */
