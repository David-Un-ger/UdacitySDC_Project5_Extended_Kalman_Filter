#include "kalman_filter.h"
#include <iostream>
using Eigen::MatrixXd;
using Eigen::VectorXd;
#include <math.h>

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
   * Kalman Prediction step is predicting the mean state vector x and the state covariance Matrix P
   */	
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}


void KalmanFilter::Update(const VectorXd &z) {
  /**
   * Update is called in case Lidar measurements -> use of the "normal" Kalman Filter
   * The equations from the lectures are used.
   */
   	VectorXd y = z - H_ * x_;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd K = P_ * Ht * Si;

	// next state calculation
	x_ = x_ + K*y;
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K*H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * The Extended Kalman Filter equations are needed for the Radar measurements
   * The Radar state transition function is nonlinear
   */
   
  VectorXd h = VectorXd(3);
  float h0 = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
  float h1 = atan2(x_(1), x_(0));
  float h2 = (x_(0)*x_(2) + x_(1)*x_(3)) / h0;
  
  h << h0, h1, h2;
  
  VectorXd y = z - h;
  
  
  // if the measured angle is bigger than pi or smaller than -pi, the KF doesn´t work anymore
  if(y(1)>= M_PI){
	  y(1) -= 2*M_PI;
  }
  else if(y(1)< -M_PI){
  	  y(1) += 2*M_PI;
  }

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;
  
  // next state
  x_ = x_ + K*y;
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K*H_) * P_;
}
