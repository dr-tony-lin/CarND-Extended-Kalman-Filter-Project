#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include "Eigen/Dense"

template<typename T> class MeasurementPackage {
private:
  long long timestamp_;
  T sensor_type_;
  Eigen::VectorXd measurements_;

public:
  MeasurementPackage(long long timestamp, T sensor_type, Eigen::VectorXd measurements) {
    timestamp_ = timestamp;
    sensor_type_ = sensor_type;
    measurements_ = measurements;
  };

  /**
   * Get the timestamp
   */
  long long timestamp() const {
    return timestamp_;
  };

  /**
   * Get the sensor type
   */
  T sensor_type() const {
    return sensor_type_;
  };

  /**
   * Get the measurements
   */
  Eigen::VectorXd measurements() const {
    return measurements_;
  };
};

#endif /* MEASUREMENT_PACKAGE_H_ */
