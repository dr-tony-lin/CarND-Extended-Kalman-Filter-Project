#include "FusionEKF.h"
#include "Eigen/Dense"
#include "utils.h"

#ifdef VERBOSE_OUT
#include <iostream>
#endif

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;
using namespace utils;

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
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized) {
    // first measurement
    x(VectorXd(4));
    previous_timestamp = measurement_pack.timestamp();
    if (measurement_pack.sensor_type() == SensorType::RADAR) {
      // Convert radar from polar to cartesian coordinates and initialize state.
      x(RadarToPV(measurement_pack.measurements()));
    } else if (measurement_pack.sensor_type() == SensorType::LASER) {
      // Initialize state from laser measurement
      VectorXd measurements = measurement_pack.measurements();
      x() << measurements[0], measurements[1], 0, 0;
    }

    // Initialize P, Q, H, F matrices
    P(MatrixXd(4, 4));
    P() << 1, 0, 0,    0,
             0, 1, 0,    0,
             0, 0, 1000, 0,
             0, 0, 0,    1000;
    F(MatrixXd(4, 4));
    F() << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;
    // done initializing, no need to predict or update
    is_initialized = true;
    return;
  }

  float dt = (measurement_pack.timestamp() - previous_timestamp) / 1000000.0;
  previous_timestamp = measurement_pack.timestamp();

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  if (dt > 0.0001) { // Else, skip prediction as this and the previous one happen at the same time
    // Update the state transition matrix according to dt
    F()(0, 2) = dt;
    F()(1, 3) = dt;
    // Update the Process covariance matrix Q
    float t2 = dt * dt;
    float t3 = t2 * dt / 2;
    float t4 = t2 * t2 / 4;
    float t3os_ax = t3/s_ax;
    float t3os_ay = t3/s_ay;
    Q(MatrixXd(4, 4));
    Q() << t4/s_ax, 0,               t3os_ax, 0,
               0,               t4/s_ay, 0,               t3os_ay,
               t3os_ax, 0,               t2*s_ax,   0,
               0,               t3os_ay, 0,               t2*s_ay;
    // Perform prediction
    Predict();
  }

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (measurement_pack.sensor_type() == SensorType::RADAR) {
    // Radar updates, set radar's measurement trsnsition matrix and covariance matrix
    H(Hj = CalculateJacobian(x()));
    R(R_radar);
    UpdateEKF(measurement_pack.measurements());
  } else {
    // Laser updates, set laser's measurement trsnsition matrix and covariance matrix
    H(H_laser);
    R(R_laser);
    Update(measurement_pack.measurements());
  }
}

void FusionEKF::UpdateEKF(const VectorXd &z) {
  VectorXd normalized_z = z;
  // Align bearing angle to [0, 2PI), this is important for computing the difference.
  // Without the alignment, invalid state update may occur when crossing x or y axes
  normalized_z(1) = AlignAngle(z(1));
  MatrixXd H_t = H().transpose(); // transpose of H
  // Convert state to radar measurement
  VectorXd rad = PVToRadar(x());
  VectorXd y = normalized_z - rad;
  // align the difference in bearing angles to [PI, -PI), so we have a minimal possible rotation
  y(1) = AlignAngularMovement(y(1));
  MatrixXd S = R() + H() * P() * H_t;
  MatrixXd k = P() * H_t * S.inverse();
  // new state
  x() += k * y;
  // new state covariance matrix
  P() = (I_ - k * H()) * P();
#ifdef VERBOSE_OUT
  // Write verbose information to cout
  std::cout << "Update EKF: " << z.transpose() << ", aligned: " << normalized_z.transpose() << std::endl;
  std::cout << "H = " << H() << std::endl;
  std::cout << "R = " << R() << std::endl;
  std::cout << "Rad = " << rad << std::endl;
  std::cout << "y = " << y.transpose() << std::endl;
  std::cout << "S = " << S << std::endl;
  std::cout << "k = " << k << std::endl;
#endif
}

MatrixXd FusionEKF::CalculateJacobian(const VectorXd &x_state) {
  MatrixXd Hj(3, 4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // pre-compute a set of terms to avoid repeated calculation
  float l2 = px * px + py * py;
  float l = sqrt(l2);
  float vxpy_vypx_o_ll2 = (vx * py - vy * px) / (l * l2);
  float pxol = px / l;
  float pyol = py / l;

  // check division by zero
  if (fabs(l2) < EPSLION) {
#ifdef VERBOSE_OUT
    cout << "CalculateJacobian () - Error - Division by Zero" << endl;
#endif
    return Hj;
  }

  // compute the Jacobian matrix
  Hj << pxol, pyol, 0, 0, 
        -(py / l2), (px / l2), 0, 0,
        py * vxpy_vypx_o_ll2, -px * vxpy_vypx_o_ll2, pxol, pyol;
  return Hj;
}
