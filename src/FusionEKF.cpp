#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */

  // Initialize matrix for laser (H_laser)
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  // Initialize P (object covariance matrix)
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1000, 0,
              0, 0, 0, 1000;

  // Initialize F (state transition matrix)
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;

  // Initialize Q (noise covariance matrix)
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    double px, py, vx, vy, sinPhi, cosPhi;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      double rho = measurement_pack.raw_measurements_[0]; // range
  	  double phi = measurement_pack.raw_measurements_[1]; // bearing
  	  double rho_dot = measurement_pack.raw_measurements_[2]; // range rate

  	  // convert polar coordinates to cartesian
      sinPhi = sin(phi);
      cosPhi = cos(phi);

  	  px = rho * cosPhi;
      py = rho * sinPhi;
  	  vx = rho_dot * cosPhi;
  	  vy = rho_dot * sinPhi;

      // Avoid values too near to zero for px and py
      if (px >= 0 && px < 0.0001){
        px = 0.0001;
        cout << "px positive, but too near to zero" << endl;
      }

      if (px < 0 && px > -0.0001) {
        px = -0.0001;
        cout << "px negative, but too near to zero" << endl;
      }

      if (py >= 0 && py < 0.0001){
        py = 0.0001;
        cout << "py positive, but too near to zero" << endl;
      }

      if (py < 0 && py > -0.0001) {
        py = -0.0001;
        cout << "py negative, but too near to zero" << endl;
      }
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
      px = measurement_pack.raw_measurements_[0];
      py = measurement_pack.raw_measurements_[1];
    }

    ekf_.x_ << px, py, vx, vy;
    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  // dt = delta time in seconds
  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
   
  // put dt in F matrix
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  // noise values for Q matrix
  double noise_ax = 9.0;
  double noise_ay = 9.0;

  // for better readability and less computation when setting the values of the Q matrix
  double dt_2 = dt * dt;
  double dt_3 = dt_2 * dt;
  double dt_4 = dt_3 * dt;
  double dt_4div4 = dt_4 / 4;
  double dt_3div2 = dt_3 / 2;
  
  // Update the process covariance matrix Q
  ekf_.Q_ <<  dt_4div4 * noise_ax, 0.0, dt_3div2 * noise_ax, 0.0,
              0.0, dt_4div4 * noise_ay, 0, dt_3div2 * noise_ay,
              dt_3div2 * noise_ax, 0.0, dt_2 * noise_ax, 0.0,
              0.0, dt_3div2 * noise_ay, 0.0, dt_2 * noise_ay;

  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates

  } else {
    // TODO: Laser updates

  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
