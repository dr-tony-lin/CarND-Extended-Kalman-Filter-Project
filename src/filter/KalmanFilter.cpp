#include "KalmanFilter.h"
#include <iostream>
#include "Tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  // Predicated state
  x_ = F_ * x_;
  // predicted state covariance matrix
  P_ = F_ * P_ * F_.transpose() + Q_;
#ifdef VERBOSE_OUT
  // Write verbose information to cout
  std::cout << "Perdict ..." << std::endl;
  std::cout << "F = " << F_ << std::endl;
  std::cout << "P = " << P_ << std::endl;
  std::cout << "Q = " << Q_ << std::endl;
  std::cout << "x = " << x_ << std::endl;
  std::cout << "P = " << P_ << std::endl;
#endif
}

void KalmanFilter::Update(const VectorXd &z) {
  MatrixXd H_t = H_.transpose(); // transpose of H
  VectorXd y = z - H_ * x_;
  MatrixXd S = R_ + H_ * P_ * H_t;
  MatrixXd k = P_ * H_t * S.inverse();
  // new state
  x_ += k * y;
  // new state covariance matrix
  P_ = (I_ - k * H_) * P_;
#ifdef VERBOSE_OUT
  // Write verbose information to cout
  std::cout << "Update: " << z << std::endl;
  std::cout << "H = " << H_ << std::endl;
  std::cout << "R = " << R_ << std::endl;
  std::cout << "y = " << y << std::endl;
  std::cout << "S = " << S << std::endl;
  std::cout << "k = " << k << std::endl;
  std::cout << "x = " << x_ << std::endl;
  std::cout << "P = " << P_ << std::endl;
#endif
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  VectorXd normalized_z = z;
  // Align bearing angle to [0, 2PI), this is important for computing the difference.
  // Without the alignment, invalid state update may occur when crossing x or y axes
  normalized_z(1) = Tools::getTools().AlignAngle(z(1));
  MatrixXd H_t = H_.transpose(); // transpose of H
  // Convert state to radar measurement
  VectorXd rad = Tools::getTools().PVToRadar(x_);
  VectorXd y = normalized_z - rad;
  // align the difference in bearing angles to [PI, -PI), so we have a minimal possible rotation
  y(1) = Tools::getTools().AlignAngularMovement(y(1));
  MatrixXd S = R_ + H_ * P_ * H_t;
  MatrixXd k = P_ * H_t * S.inverse();
  // new state
  x_ += k * y;
  // new state covariance matrix
  P_ = (I_ - k * H_) * P_;
#ifdef VERBOSE_OUT
  // Write verbose information to cout
  std::cout << "Update EKF: " << z << " norm: " << normalized_z << std::endl;
  std::cout << "H = " << H_ << std::endl;
  std::cout << "R = " << R_ << std::endl;
  std::cout << "Rad = " << rad << std::endl;
  std::cout << "y = " << y << std::endl;
  std::cout << "S = " << S << std::endl;
  std::cout << "k = " << k << std::endl;
  std::cout << "x = " << x_ << std::endl;
  std::cout << "P = " << P_ << std::endl;
#endif
}
