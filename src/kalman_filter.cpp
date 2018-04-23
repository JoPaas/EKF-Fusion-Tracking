#include "kalman_filter.h"
#include "tools.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
#include <math.h>

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

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
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  //cout << "updating lidar..." << endl;
  VectorXd y = z - H_ * x_;
  UpdateStep(y);
  //cout << "updated lidar" << endl;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  //cout << "updating radar..." << endl;
  
  double p_x = x_(0);
  double p_y = x_(1);
  double v_x = x_(2);
  double v_y = x_(3);
  //norm of x/y
  double p_norm = sqrt(pow(p_x, 2) + pow(p_y, 2));
  if (p_norm < 1){
    p_norm = 1;
  }
  VectorXd h_x(3);
  //measurement vector
  h_x << p_norm,
	atan2(p_y, p_x),
	(p_x * v_x + p_y * v_y)/p_norm;
  
  VectorXd y = z - h_x;
  //check for large angles (0->360/360->0)
  if (y(1) > M_PI){
    y(1) -= 2*M_PI;
  }
  else if (y(1) < -M_PI){
    y(1) += 2*M_PI;
  }
  //cout << "angle z: " << z(1)*180/3.1415 << ", angle h_x: " << h_x(1)*180/3.1415 << "angle y: " << y(1)*180/3.1415 << endl;
  UpdateStep(y);
  //cout << "updated radar" << endl;
}

void KalmanFilter::UpdateStep(const VectorXd &y){
  // KF Measurement update step
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();
  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  // new state
  x_ = x_ + K * y;
  P_ = (I - K * H_) * P_;
}
