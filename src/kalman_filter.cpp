#include "kalman_filter.h"
#include <math.h>
#include <iostream>
using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

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
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  // Mapping the predicted state variables into the measurement space of radar
  VectorXd z_pred(3);
  double ro_pred = pow((pow(x_(0), 2)+pow(x_(1), 2)), 0.5); // sqrt(px^2+py^2)
  if (x_(0) == 0){
  	cout << "Division by zero error while calculating theta_pred!" << endl;
  }
  double theta_pred = atan2(x_(1),x_(0)); // arctan(py/px)
  if (ro_pred == 0){
  	cout << "Division by zero error while calculating ro_dot_pred!" << endl;
  }
  double ro_dot_pred = ((x_(0)*x_(2)+x_(1)*x_(3))/ro_pred); // px*vx+py*vy/sqrt(px^2+py^2)
  z_pred << ro_pred, theta_pred, ro_dot_pred;
	
	VectorXd y = z - z_pred;
	// normalize ϕ in the y vector so that its angle is between −pi and pi;
	while (y(1) < -M_PI){
		y(1) += 2*M_PI;
	}
	while (y(1) > M_PI){
		y(1) -= 2*M_PI;
	}

	// All the H matrix used here are Jacobian matrices used in lineariation of the non linear measurement function
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);

	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}
