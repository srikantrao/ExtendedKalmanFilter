#include "kalman_filter.h"
#include<math.h>

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
  // Code based on lecture exercises
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {

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
  // Get the position and velocity
	double px = x_(0);  // x_pos
	double py = x_(1);  // y_pos
	double vx = x_(2);  // x_vel
	double vy = x_(3);  // y_vel

	//Measurement Update  -- The measurement function will still use the non-linear function
	VectorXd h_of_x(3);
	float rad_len = sqrt(px*px + py*py) ;
	h_of_x(0) = rad_len ;

	h_of_x(1) = atan2(py,px);

	if (fabs(h_of_x(1)) < 0.001) {
	    h_of_x(1) = 0.001;
	  }

	if(rad_len < 0.00001){
		h_of_x(2) = (px*vx + py*vy)/0.00001;
	}
	else {
		h_of_x(2) = (px*vx + py*vy) / rad_len;
	}

	VectorXd z_pred = h_of_x;
	VectorXd y = z - z_pred;
	y(1) = fmod(y(1),M_PI);
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
