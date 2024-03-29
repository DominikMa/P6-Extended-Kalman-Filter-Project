#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"
#include "tools.h"

class KalmanFilter {
 public:

  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // state transition matrix
  Eigen::MatrixXd F_;

  // process covariance matrix
  Eigen::MatrixXd Q_;

  // measurement matrix
  Eigen::MatrixXd H_laser_;

  // measurement jacobi matrix
  Eigen::MatrixXd Hj_;

  // measurement covariance matrix laser
  Eigen::MatrixXd R_laser_;

  // measurement covariance matrix radar
  Eigen::MatrixXd R_radar_;

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
   * @param H_laser_in Measurement matrix
   * @param Hj_in measurement jacobi matrix
   * @param R_laser_in Measurement covariance matrix laser
   * @param R_radar_in Measurement covariance matrix radar
   * @param Q_in Process covariance matrix
   */
  void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
            Eigen::MatrixXd &H_laser_in, Eigen::MatrixXd &Hj_in, Eigen::MatrixXd &R_laser_in,
            Eigen::MatrixXd &R_radar_in, Eigen::MatrixXd &Q_in);

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

 private:
  // tool object used to compute Jacobian and RMSE
  Tools tools;
};

#endif /* KALMAN_FILTER_H_ */
