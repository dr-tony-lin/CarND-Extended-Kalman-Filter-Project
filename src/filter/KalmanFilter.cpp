#include "KalmanFilter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

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
