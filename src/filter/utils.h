#ifndef FILTER_UTILS_H_
#define FILTER_UTILS_H_

#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace utils {
const float EPSLION = 0.00000001;
const float PI_2 = 2 * M_PI;

/**
 * Helper function to convert radar measurement to P, V state
 */
extern VectorXd RadarToPV(const VectorXd& data);

/**
 * Helper function to convert P, V state to Rador state
 */
extern VectorXd PVToRadar(const VectorXd& pv);

/**
 * Helper function to align an angle so it fall in the range of [PI, -PI)
 * This is important for computing the angular difference for radar sensor data
 * Without the alignment, invalid state update might occur when crossing x or y
 * axes
 */
extern float AlignAngle(const float radius);
}
#endif /* TOOLS_H_ */
