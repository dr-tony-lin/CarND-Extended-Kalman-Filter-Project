#include "KalmanFilter.h"
#include <iostream>
#include "Tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x = x_in;
  P = P_in;
  F = F_in;
  H = H_in;
  R = R_in;
  Q = Q_in;
}

void KalmanFilter::Predict() {
  x = F * x;
  P = F * P * F.transpose() + Q;
#ifdef VERBOSE_OUT
  std::cout << "Perdict ..." << std::endl;
  std::cout << "F = " << F << std::endl;
  std::cout << "P = " << P << std::endl;
  std::cout << "Q = " << Q << std::endl;
  std::cout << "x = " << x << std::endl;
  std::cout << "p = " << x << std::endl;
#endif
}

void KalmanFilter::Update(const VectorXd &z) {
  MatrixXd H_t = H.transpose();
  VectorXd y = z - H * x;
  MatrixXd S = R + H * P * H_t;
  MatrixXd k = P * H_t * S.inverse();
  x += k * y;
  P = (MatrixXd::Identity(x.size(), x.size()) - k * H) * P;
#ifdef VERBOSE_OUT
  std::cout << "Update: " << z << std::endl;
  std::cout << "H = " << H << std::endl;
  std::cout << "R = " << R << std::endl;
  std::cout << "y = " << y << std::endl;
  std::cout << "S = " << S << std::endl;
  std::cout << "k = " << k << std::endl;
  std::cout << "x = " << x << std::endl;
  std::cout << "p = " << x << std::endl;
#endif
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  VectorXd normalized_z = z;
  // Normalize paw to 0 and 2PI
  normalized_z(1) = Tools::getTools().AlignRadius(z(1));
  MatrixXd H_t = H.transpose();
  VectorXd rad = Tools::getTools().PVToRadar(x);
  VectorXd y = normalized_z - rad;
  y(1) = Tools::getTools().AlignRadiusDelta(y(1));
  MatrixXd S = R + H * P * H_t;
  MatrixXd k = P * H_t * S.inverse();
  x += k * y;
  P = (MatrixXd::Identity(x.size(), x.size()) - k * H) * P;
#ifdef VERBOSE_OUT
  std::cout << "Update EKF: " << z << " norm: " << normalized_z << std::endl;
  std::cout << "H = " << H << std::endl;
  std::cout << "R = " << R << std::endl;
  std::cout << "Rad = " << rad << std::endl;
  std::cout << "y = " << y << std::endl;
  std::cout << "S = " << S << std::endl;
  std::cout << "k = " << k << std::endl;
  std::cout << "x = " << x << std::endl;
  std::cout << "p = " << x << std::endl;
#endif
}
