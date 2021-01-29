#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * lidar data (from laser) - uses basic Kalman filter equations
   * 
   * z is the measurement vector. For a lidar sensor, the 
   * z vector contains the position − x and position - y measurements.
   * 
   * H_ is the matrix that projects our belief about the object's current
   * state into the measurement space of the sensor.
   * 
   * R_ is a covariance matrix, which represents the uncertainty in our 
   * sensor measurements. The dimensions of the R matrix is square and each
   * side of its matrix is the same length as the number of measurements parameters.
   */
  VectorXd y = z - H_ * x_;
  CalculateGlobal(y);

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * radar data - uses non-linear equations,
   * so the update step involves linearizing the equations with the Jacobian matrix. 
   */

  // range
  double rho = std::sqrt(x_(0) * x_(0) + x_(1) * x_(1));
  double phi = 0.0;       // bearing
  double rho_dot = 0.0;   // range rate

  // Calculate phi = bearing
  // We have to avoid division by zero
  if (std::fabs(x_(0)) > 0.0001)
    phi = std::atan2(x_(1), x_(0));
  else
    phi = 0.0001;

  // Calculate rho_dot = range rate
  // We have to avoid division by zero
  rho_dot = (x_(0) * x_(2) + x_(1) * x_(3)) / std::max(rho, 0.0001);

  VectorXd h(3);
  h << rho, phi, rho_dot;

  VectorXd y = z - h;

  // Normalizing the angle phi
  while (y(1) > M_PI || y(1) < -M_PI) {
    // If y(1) (phi) > Pi then sub Pi; otherwise add Pi
    y(1) = y(1) > M_PI ? y(1) - M_PI : y(1) + M_PI;
  }

  CalculateGlobal(y);
}

void KalmanFilter::CalculateGlobal(const VectorXd &y) {
  /**
   * The Code in Update and UpdateEKF are identical.
   * So I have refactored the code by extracting this method.
   * 
   * It calculates the update for both, the Kalman Filter
   * and the Extended Kalman Filter
   */

  MatrixXd H_transposed = H_.transpose();
  MatrixXd PHt = P_ * H_transposed;
  MatrixXd S = H_ * PHt + R_;
  MatrixXd K = PHt * S.inverse();

  // new estimate
  x_ = x_ + K * y;
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
