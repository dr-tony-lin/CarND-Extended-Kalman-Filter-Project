#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "Tools.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized = false;

  previous_timestamp = 0;

  // initializing matrices
  R_laser = MatrixXd(2, 2);
  R_radar = MatrixXd(3, 3);
  H_laser = MatrixXd(2, 4);
  Hj = MatrixXd(3, 4);

  // measurement covariance matrix - laser
  R_laser << 0.0225, 0, 
             0,      0.0225;

  // measurement covariance matrix - radar
  R_radar << 0.09, 0,      0,
             0,    0.0009, 0,
             0,    0,      0.09;
  H_laser << 1, 0, 0, 0,
             0, 1, 0, 0;
  
  s_ax = 9; // acceleration noise x
  s_ay = 9; // acceleration noise y
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(
    const MeasurementPackage<SensorType> &measurement_pack) {
  if (measurement_pack.timestamp < previous_timestamp) {
    cout << "Restarting ..." << endl;
    previous_timestamp = 0;
    is_initialized = false;
  }

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized) {
    // first measurement
    cout << "EKF: " << endl;
    ekf.x = VectorXd(4);
    previous_timestamp = measurement_pack.timestamp;
    if (measurement_pack.sensor_type == SensorType::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      ekf.x = Tools::getTools().RadarToPV(measurement_pack.measurements);
    } else if (measurement_pack.sensor_type == SensorType::LASER) {
      /**
      Initialize state from laser measurement
      */
      ekf.x << measurement_pack.measurements[0],
          measurement_pack.measurements[1], 0, 0;
    }

    // Initialize P, Q, H, F matrices
    ekf.P = MatrixXd(4, 4);
    ekf.P << 1, 0, 0,    0,
             0, 1, 0,    0,
             0, 0, 1000, 0,
             0, 0, 0,    1000;
    ekf.Q = MatrixXd(4, 4);
    ekf.F = MatrixXd(4, 4);
    ekf.F << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;
    // done initializing, no need to predict or update
    is_initialized = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  float dt = (measurement_pack.timestamp - previous_timestamp) / 1000000.0;
  previous_timestamp = measurement_pack.timestamp;
  if (dt > 0.0001) { // Else, skip prediction as this and the previous one happen at the same time
    // Update the state transition matrix according to dt
    ekf.F(0, 2) = dt;
    ekf.F(1, 3) = dt;
    // Update the Process covariance matrix Q
    float t2 = dt * dt;
    float t3 = t2 * dt / 2;
    float t4 = t2 * t2 / 4;
    float t3os_ax = t3/s_ax;
    float t3os_ay = t3/s_ay;
    ekf.Q = MatrixXd(4, 4);
    ekf.Q << t4/s_ax, 0,               t3os_ax, 0,
             0,               t4/s_ay, 0,               t3os_ay,
             t3os_ax, 0,               t2*s_ax,   0,
             0,               t3os_ay, 0,               t2*s_ay;
    // Do the prediction
    ekf.Predict();
  }

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (measurement_pack.sensor_type == SensorType::RADAR) {
    // Radar updates
    ekf.H = Hj = Tools::getTools().CalculateJacobian(ekf.x);
    ekf.R = R_radar;
    ekf.UpdateEKF(measurement_pack.measurements);
  } else {
    // Laser updates
    ekf.H = H_laser;
    ekf.R = R_laser;
    ekf.Update(measurement_pack.measurements);
  }

#ifdef VERBOSE_OUT
  // print the output
  cout << "x = " << ekf.x << endl;
  cout << "P = " << ekf.P << endl;
#endif
}
