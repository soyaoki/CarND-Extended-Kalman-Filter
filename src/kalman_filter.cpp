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

void KalmanFilter::Init(const VectorXd &x_in, const MatrixXd &P_in, const MatrixXd &F_in,
                        const MatrixXd &H_laser_in, const MatrixXd &R_laser_in, const MatrixXd &R_radar_in, const MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_laser_ = H_laser_in;
  R_laser_ = R_laser_in;
  R_radar_ = R_radar_in;
  Q_ = Q_in;
  
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
//  std::cout << "Predicting..." << std::endl;
  x_= F_* x_;
  P_= F_*P_*F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
//  std::cout << "Updating by KF..." << std::endl;
  VectorXd y = z - H_laser_* x_;
//  std::cout << y << std::endl;
  MatrixXd S = H_laser_*P_*H_laser_.transpose() + R_laser_;
//  std::cout << S << std::endl;
  MatrixXd K = P_*H_laser_.transpose()*S.inverse();
//  std::cout << K << std::endl;
  MatrixXd I = MatrixXd::Identity(4, 4);
  
  x_= x_+ ( K * y );
  P_= ( I - K * H_laser_) * P_;
  
}

void KalmanFilter::UpdateEKF(const VectorXd &z, const MatrixXd &Hj_) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
//  std::cout << "Updating by EKF..." << std::endl;
  float rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
  float phi = atan2(x_(1), x_(0));
  float rho_dot;
  if (fabs(rho) < 0.0001) {
    rho_dot = 0;
  } else {
    rho_dot = (x_(0)*x_(2) + x_(1)*x_(3))/rho;
  }
  VectorXd z_pred(3);
  z_pred << rho, phi, rho_dot;
  VectorXd y = z - z_pred;
//  std::cout << y << std::endl;
//  std::cout << Hj_ << std::endl;
  MatrixXd S = Hj_*P_*Hj_.transpose() + R_radar_;
//  std::cout << S << std::endl;
  MatrixXd K = P_*Hj_.transpose()*S.inverse();
//  std::cout << K << std::endl;
  MatrixXd I = MatrixXd::Identity(4, 4);
  
  x_= x_+ ( K * y );
  P_= ( I - K * Hj_ ) * P_;
  
}
