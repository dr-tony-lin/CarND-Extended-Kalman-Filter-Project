#include "Tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::ArrayXd;
using std::vector;

static const float EPSLION = 0.00000001;
static const float PI_2 = 2 * M_PI;

Tools Tools::singleton = Tools();

Tools Tools::getTools() {
  return Tools::singleton;
}

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  ArrayXd sum(4);
  sum << 0, 0, 0, 0;
  // Calculate sum of square of errors
  for (int i = 0; i < estimations.size(); ++i) {
    ArrayXd diff = estimations[i] - ground_truth[i];
    sum += diff * diff;
  }
  // calculate the mean
  ArrayXd mean = sum / estimations.size();
  // calculate the squared root
  VectorXd rmse = mean.sqrt();
  // return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd &x_state) {
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

  // check division by zero
  if (fabs(l2) < EPSLION) {
    cout << "CalculateJacobian () - Error - Division by Zero" << endl;
    return Hj;
  }

  // compute the Jacobian matrix
  Hj << (px / l), (py / l), 0, 0, 
        -(py / l2), (px / l2), 0, 0,
        py * vxpy_vypx_o_ll2, -px * vxpy_vypx_o_ll2, px / l, py / l;

  return Hj;
}

VectorXd Tools::RadarToPV(const VectorXd &radar) {
  VectorXd pv(4);
  float c = cos(radar(1));
  float s = sin(radar(1));
	pv << radar(0)*c, radar(0)*s,
        radar(2)*c, radar(2)*s;
	return pv;
}

float Tools::AlignAngle(const float radius) {
  if (radius < 0) {
    return radius + PI_2;
  }
  else if (radius >= PI_2) {
    return radius - PI_2;
  }
  return radius;
}

float Tools::AlignAngularMovement(const float delta) {
  if (delta >= M_PI) {
    return delta - PI_2;
  }
  else if (delta < -M_PI) {
    return delta + PI_2;
  }
  return delta;
}

VectorXd Tools::PVToRadar(const VectorXd &pv) {
	VectorXd radar(3);
	float r = sqrt(pv(0)*pv(0) + (pv(1)*pv(1)));
	if ( r > EPSLION) {
		float paw = abs(pv(0)) > EPSLION? atan(pv(1)/pv(0)): (pv(0) * pv(1) >= 0 ? M_PI/2: -M_PI/2);
    if (pv(0) < 0) { // other side of the hemisphere
      paw = M_PI + paw;
    }
    // Align paw to be within [0  2PI)
    paw = AlignAngle(paw);
    float v = (pv(0)*pv(2) + pv(1)*pv(3)) / r;
		radar << r, paw, v;
	}
	return radar;
}