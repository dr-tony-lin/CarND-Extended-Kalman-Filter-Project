#ifndef FILTER_TOOLS_H_
#define FILTER_TOOLS_H_
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

/*
 * A helper class. It is a singleton where the singleton is accessible through getTools() method
 */
class Tools {
private:
  static Tools singleton;

public:
  /**
   * Return the singleton
   */
  static Tools getTools();

  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

  /**
  * A helper method to calculate RMSE.
  */
  VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

  /**
  * A helper method to calculate Jacobians.
  */
  MatrixXd CalculateJacobian(const VectorXd& x_state);

  /**
   * Helper function to convert radar measurement to P, V state
   */
  VectorXd RadarToPV(const VectorXd& data);

  /**
   * Helper function to convert P, V state to Rador state
   */
  VectorXd PVToRadar(const VectorXd &pv);

  /**
   * Helper function to align an angle so it fall in the range of [0, 2PI)
   * This is important for computing the angular difference for radar sensor data
   * Without the alignment, invalid state update might occur when crossing x or y axes
   */
  float AlignAngle(const float radius);

  /**
   * Helper function to align angular movement so it fall in the range of [PI, -PI).
   * This will result in a minimal angle to rotate between two angles
   */
  float AlignAngularMovement(const float delta);
};

#endif /* TOOLS_H_ */
