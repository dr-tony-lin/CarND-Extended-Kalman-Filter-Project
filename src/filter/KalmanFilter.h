#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"

class KalmanFilter {
 private:
  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // Identity matrix same size as the state covariance matrix
  Eigen::MatrixXd I_;

  // state transition matrix
  Eigen::MatrixXd F_;

  // process covariance matrix
  Eigen::MatrixXd Q_;

  // measurement matrix
  Eigen::MatrixXd H_;

  // measurement covariance matrix
  Eigen::MatrixXd R_;

 public:
  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  /**
   * Init Initializes Kalman filter
   * @param x_in Initial state
   * @param P_in Initial state covariance
   * @param F_in Transition matrix
   * @param H_in Measurement matrix
   * @param R_in Measurement covariance matrix
   * @param Q_in Process covariance matrix
   */
  void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
            Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in,
            Eigen::MatrixXd &Q_in);

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  void Predict();

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  void Update(const Eigen::VectorXd &z);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  void UpdateEKF(const Eigen::VectorXd &z);

  /**
   * Set the state
   * @param value the value to set
   */
  void x(const Eigen::VectorXd value) {
    x_ = value;
  };

  /**
   * Get the state
   */
   Eigen::VectorXd& x() {
    return x_;
  };

  /**
   * Set the state covariance matrix
   * @param value the value to set
   */
  void P(const Eigen::MatrixXd value) {
    P_ = value;
    // Also set the identity matrix for the Update operations
    I_ = Eigen::MatrixXd::Identity(P_.rows(), P_.cols());
  };
  
  /**
   * Get the state covariance matrix
   */
  Eigen::MatrixXd& P() {
    return P_;
  };

  /**
   * Set the state transition matrix
   * @param value the value to set
   */
  void F(const Eigen::MatrixXd value) {
    F_ = value;
  };
  
  /**
   * Get the state transition matrix
   */
   Eigen::MatrixXd& F() {
    return F_;
  };

  /**
   * Set the process covariance matrix
   * @param value the value to set
   */
  void Q(const Eigen::MatrixXd value) {
    Q_ = value;
  };
  
  /**
   * Get process covariance matrix
   */
   Eigen::MatrixXd& Q() {
    return Q_;
  };

  /**
   * Set the measurement matrix
   * @param value the value to set
   */
  void H(const Eigen::MatrixXd value) {
    H_ = value;
  };
  
  /**
   * Get the measurement matrix
   */
   Eigen::MatrixXd& H() {
    return H_;
  };

  /**
   * Set the measurement covariance matrix
   * @param value the value to set
   */
  void R(const Eigen::MatrixXd value) {
    R_ = value;
  };
  
  /**
   * Get the measurement covariance matrix
   */
   Eigen::MatrixXd& R() {
    return R_;
  };
};
#endif /* KALMAN_FILTER_H_ */
