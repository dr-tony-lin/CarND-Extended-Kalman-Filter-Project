#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include "Eigen/Dense"

template<typename T> class MeasurementPackage {
public:
  long long timestamp;
  T sensor_type;
  Eigen::VectorXd measurements;
};

#endif /* MEASUREMENT_PACKAGE_H_ */
