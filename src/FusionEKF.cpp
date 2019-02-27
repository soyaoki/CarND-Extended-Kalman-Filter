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
  H_laser_ << 1, 0, 0, 0,
                    0, 1, 0, 0;

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
    
    MatrixXd P_in = MatrixXd(4, 4);
    P_in << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;
    MatrixXd F_in = MatrixXd(4, 4); 
    MatrixXd Q_in = MatrixXd(4, 4); 

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      VectorXd x_in = ekf_.x_;
      float ro = measurement_pack.raw_measurements_(0);
      float phi = measurement_pack.raw_measurements_(1);
      float ro_dot = measurement_pack.raw_measurements_(2);
      x_in(0) = ro * cos(phi);
      x_in(1) = ro * sin(phi);      
      x_in(2) = ro_dot * cos(phi);
      x_in(3) = ro_dot * sin(phi);
      ekf_.Init(x_in, P_in, F_in, H_laser_, R_laser_, R_radar_, Q_in);
      std::cout << "Initialization is Done! (RADAR)" << std::endl;
      
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
      VectorXd x_in = ekf_.x_;
      x_in(0) = measurement_pack.raw_measurements_(0);
      x_in(1) = measurement_pack.raw_measurements_(1);
      ekf_.Init(x_in, P_in, F_in, H_laser_, R_laser_, R_radar_, Q_in);
      std::cout << "Initialization is Done! (LASER)" << std::endl;
      
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    previous_timestamp_ = measurement_pack.timestamp_;
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
  
  float dt = ( measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  std::cout << "dt=" << dt << std::endl;  
  
  ekf_.F_ << 1, 0, dt, 0,
                 0, 1, 0, dt,
                 0, 0, 1, 0,
                 0, 0, 0, 1;
  
//    std::cout << ekf_.F_ << std::endl;
  
  int nax = 9;
  int nay = 9;
  
  ekf_.Q_ << pow(dt,4.0)*nax/4, 0, pow(dt,3.0)*nax/2, 0,
                  0, pow(dt,4.0)*nay/4, 0, pow(dt,3.0)*nay/2,
                  pow(dt,3.0)*nax/2, 0, pow(dt,2.0)*nax, 0,
                  0, pow(dt,3.0)*nay/2, 0, pow(dt,2.0)*nay;
  
//  std::cout << ekf_.Q_ << std::endl;

  ekf_.Predict();
//  std::cout << "Predict is Done!" << std::endl;
  
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
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.UpdateEKF(measurement_pack.raw_measurements_, Hj_);
    std::cout << "Update by EKF is Done!" << std::endl;

  } else {
    // TODO: Laser updates
    ekf_.Update(measurement_pack.raw_measurements_);
    std::cout << "Update by KF is Done!" << std::endl;

  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
