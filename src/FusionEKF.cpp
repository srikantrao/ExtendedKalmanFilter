#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include<math.h>

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
  H_laser_ = MatrixXd(2, 4);  //
  Hj_ = MatrixXd(3, 4);      // Jacobian Matrix

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  // measurement matrix - laser
  H_laser_ << 1, 0, 0, 0,
		  	  0, 1, 0, 0;

  //Jacobian Matrix - Radar
  Hj_ << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, 0;

  // Also need to initialize conditions for Kalman Filter Object ekf

  //State covariance matrix - P
   ekf_.P_ = MatrixXd(4, 4);
   ekf_.P_ << 1, 0, 0, 0,
    		  0, 1, 0, 0,
    		  0, 0, 1000, 0,
    		  0, 0, 0, 1000;

      // State Transition Matrix - F
    ekf_.F_ = MatrixXd(4, 4);
    ekf_.F_ << 1, 0, 1, 0,
    		   0, 1, 0, 1,
    		   0, 0, 1, 0,
    		   0, 0, 0, 1;

     ekf_.Q_ = MatrixXd(4, 4);
     ekf_.Q_ << 1,0,0,0,
                0,1,0,0,
                0,0,1,0,
                0,0,0,1;

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
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {

    	// extract rho, phi and rho_dot
    	double rho = measurement_pack.raw_measurements_(0);
    	double phi = measurement_pack.raw_measurements_(1);
    	double rho_dot = measurement_pack.raw_measurements_(2);

    	// Convert from Polar to Cartesian Co-ordinates
    	ekf_.x_(0) = rho*cos(phi);
    	ekf_.x_(1) = rho*sin(phi);
    	ekf_.x_(2) = rho_dot*cos(phi);
    	ekf_.x_(3) = rho_dot*sin(phi);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    	ekf_.x_(0) =measurement_pack.raw_measurements_(0);
    	ekf_.x_(1) =measurement_pack.raw_measurements_(1);
    	ekf_.x_(2) = 0;
    	ekf_.x_(3) = 0;
    }

    // done initializing, no need to predict or update
    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  // Measure Elapsed time based on previous lecture quiz

    double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;     //dt - expressed in seconds
    previous_timestamp_ = measurement_pack.timestamp_;
    cout << "dt = " << dt << endl;  // Print time interval for Debug

    // Update the state transition Matrix F using dt
    ekf_.F_(0,2) = dt;
    ekf_.F_(1,3) = dt;

    // Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
    double noise_ax = 9;
    double noise_ay = 9;

    // Update the process noise covariance matrix.
    ekf_.Q_ = MatrixXd(4,4);

    double dt_4 = pow(dt,4)/4; // Includes the coefficient
    double dt_3 = pow(dt,3)/2; // Includes the coefficient
    double dt_2 = pow(dt,2);

    ekf_.Q_ << dt_4*noise_ax, 0, dt_3*noise_ax,0,
    			  0, dt_4*noise_ay, 0, dt_3*noise_ay,
    			  dt_3*noise_ax, 0, dt_2*noise_ax, 0,
    			  0, dt_3*noise_ay, 0, dt_2*noise_ay;

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

	  // Use Jacobian to calculate H
	  ekf_.H_ = tools.CalculateJacobian(ekf_.x_);

	  ekf_.R_ = R_radar_;

	  // perform an Update using the Extended kalman Filter Update mechanism
	  ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // Laser updates - Everything is linear
	  ekf_.H_ = H_laser_;
	  ekf_.R_ = R_laser_;
	  ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
