#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
	// Based on my solution the the lecture quiz

	VectorXd rmse(4);
		rmse << 0,0,0,0;

		if(estimations.size() != ground_truth.size() || estimations.size() == 0){
				cout << "Invalid estimation or ground_truth data" << endl;
				return rmse;
		}

		//accumulate squared residuals
			for(unsigned int i=0; i < estimations.size(); i++){

				VectorXd residual = estimations[i] - ground_truth[i];

				//coefficient-wise multiplication
				residual = residual.array()*residual.array();
				rmse += residual;
			}

			//calculate the mean
			rmse = rmse/estimations.size();

			//calculate the squared root
			rmse = rmse.array().sqrt();

			//return the result
			return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {

	// Based on solution to lecture quiz

	MatrixXd Hj(3,4);

		//recover state parameters
		double px = x_state(0);
		double py = x_state(1);
		double vx = x_state(2);
		double vy = x_state(3);

		double denom1 = px*px + py*py;
		//check division by zero
		if(denom1 < 0.0001 ) { cout << " Divide by zero error" << endl; return Hj; }

		double sqr_denom = sqrt(denom1);
		//Compute the Jacobian matrix
		Hj(0,0) = px/sqr_denom;
		Hj(0,1) = py/sqr_denom;
		Hj(0,2) = 0;
		Hj(0,3) = 0;
		Hj(1,0) = -py/denom1;
		Hj(1,1) = px/denom1;
		Hj(1,2) = 0;
		Hj(1,3) = 0;
		Hj(2,0) = py*(py*vx-px*vy)/pow(sqr_denom, 3);
		Hj(2,1) = px*(px*vy-py*vx)/pow(sqr_denom, 3);
		Hj(2,2) = Hj(0,0);
		Hj(2,3) = Hj(0,1);

		// return the Jacobian

		return Hj;
}
