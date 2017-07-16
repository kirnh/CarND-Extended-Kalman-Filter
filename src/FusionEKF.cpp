#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <math.h>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
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

  H_laser_ << 1, 0, 0, 0,
            0, 1, 0, 0;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  ekf_.x_ = VectorXd(4);
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.Q_ = MatrixXd(4, 4);

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float ro = measurement_pack.raw_measurements_[0];
      float theta = measurement_pack.raw_measurements_[1];

      float px = ro * cos(theta);
      float py = ro * sin(theta);

      ekf_.x_[0] = px;
      ekf_.x_[1] = py;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      ekf_.x_[0] = measurement_pack.raw_measurements_[0];
      ekf_.x_[1] = measurement_pack.raw_measurements_[1];
    }

    // Initialize velocities to 0
    ekf_.x_[2] = 0;
    ekf_.x_[3] = 0;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    
    // updating timestamp
    previous_timestamp_ = measurement_pack.timestamp_;
  }

  // Delta time using timestamp and previous_timestamp_
  float delta_t = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
    ekf_.F_ << 1, 0, delta_t, 0,
                0, 1, 0, delta_t,
                0, 0, 1, 0,
                0, 0, 0, 1;

    float delta_t4 = pow(delta_t, 4);
    float delta_t3 = pow(delta_t, 3);
    float delta_t2 = pow(delta_t, 2);

    ekf_.Q_ << delta_t4/4*9, 0, delta_t3/2*9, 0,
                0, delta_t4/4*9, 0, delta_t3/2*9,
                delta_t3/2*9, 0 , delta_t2*9, 0,
                0, delta_t3/2*9, 0, delta_t2*9;
    
    // Call the predict function
    ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {

    // Radar updates
    // Radar measurement covariance matrix
    ekf_.R_ = R_radar_;
    // Jacobian matrix H for radar measurement function linearlization
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;

    // Call the EKF update function
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } 

  else {
    // Laser updates
    // Laser measurement covariance matrix
    ekf_.R_ = R_laser_;
    // Laser measurement matrix
    ekf_.H_ = H_laser_;

    // Call the KF update function
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
