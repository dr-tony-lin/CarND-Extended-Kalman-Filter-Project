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
  if (angle > M_PI) {
    return angle - PI_2;
  } else if (angle <= -M_PI) {
    return angle + PI_2;
  }

  return angle;
}

VectorXd utils::PVToRadar(const VectorXd &pv) {
  VectorXd radar(3);
	float r = sqrt(pv(0)*pv(0) + (pv(1)*pv(1)));
	if ( r > EPSLION) {
		float phi = atan2(pv(1), pv(0));
    float v = (pv(0)*pv(2) + pv(1)*pv(3)) / r;
		radar << r, phi, v;
	}
	return radar;
}