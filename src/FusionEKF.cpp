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
			  
  H_laser_ << 1, 0, 0, 0,
            0, 1, 0, 0;
			
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1,0,0,0,
            0,1,0,0,
            0,0,1,0,
            0,0,0,1;
			
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
     * Initialize the state ekf_.x_ with the first measurement.
     * Creating the covariance matrix.
     */

    // first measurement
    cout << "EKF: Starting initialization" << endl;
    
    
	double px;
	double py;
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // is executed if the first measurement is radar.
	  // Convert radar from polar to cartesian coordinates and stores the x and y position. 
	  // The velocity is not stored because from a single measurement it is not accurate enought.	  
	  px = measurement_pack.raw_measurements_(0)  * cos(measurement_pack.raw_measurements_(1));
	  py = measurement_pack.raw_measurements_(0)  * sin(measurement_pack.raw_measurements_(1));
	    

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // is executed if the first measurement is lidar.
	  px = measurement_pack.raw_measurements_(0);
	  py = measurement_pack.raw_measurements_(0);
	  
    }
	
	// stores the measured position in the state variable
	ekf_.x_ = VectorXd(4);
	ekf_.x_ << px,
			py,
			0.0,
			0.0;
			
	// initialize the state covariance matrix with small uncertainty for the position and high uncertainty for the velocity.		
    ekf_.P_ = MatrixXd(4, 4);
	ekf_.P_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;

	
	previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

   
  // compute the time elapsed between the current and previous measurements
  // dt - expressed in seconds
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  
  // Modify the F matrix so that the time is integrated
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  // calculation that are needed for Q
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  // set the acceleration noise components
  float noise_ax = 9;
  float noise_ay = 9;
  
  // set the process covariance matrix Q
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
         0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
         dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
         0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

  ekf_.Predict();

  /**
   * Update
   */


  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates 
	Hj_= tools.CalculateJacobian(ekf_.x_);
	ekf_.H_ = Hj_;
	ekf_.R_ = R_radar_;
	ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // Laser updates
	ekf_.H_ = H_laser_;
	ekf_.R_ = R_laser_;
	ekf_.Update(measurement_pack.raw_measurements_);

  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
