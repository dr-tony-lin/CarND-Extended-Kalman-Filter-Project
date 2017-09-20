#include "utils.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::ArrayXd;
using std::vector;
using namespace utils;

VectorXd utils::RadarToPV(const VectorXd &radar) {
  VectorXd pv(4);
  float c = cos(radar(1));
  float s = sin(radar(1));
	pv << radar(0)*c, radar(0)*s,
        radar(2)*c, radar(2)*s;
	return pv;
}

float utils::AlignAngle(const float angle) {
  if (angle < 0) {
    return angle + PI_2;
  }
  else if (angle >= PI_2) {
    return angle - PI_2;
  }
  return angle;
}

float utils::AlignAngularMovement(const float delta) {
  if (delta >= M_PI) {
    return delta - PI_2;
  }
  else if (delta < -M_PI) {
    return delta + PI_2;
  }
  return delta;
}

VectorXd utils::PVToRadar(const VectorXd &pv) {
  VectorXd radar(3);
	float r = sqrt(pv(0)*pv(0) + (pv(1)*pv(1)));
	if ( r > EPSLION) {
		float phi = (fabs(pv(0)) > EPSLION)? atan(pv(1)/pv(0)): (pv(0) * pv(1) >= 0 ? M_PI/2: -M_PI/2);
    if (pv(0) < 0) { // other side of the hemisphere
      phi = M_PI + phi;
    }
    // Align phi to be within [0  2PI)
    phi = AlignAngle(phi);
    float v = (pv(0)*pv(2) + pv(1)*pv(3)) / r;
		radar << r, phi, v;
	}
	return radar;
}