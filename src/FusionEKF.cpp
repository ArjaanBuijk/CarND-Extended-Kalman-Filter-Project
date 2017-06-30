#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = 0;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_      = MatrixXd(3, 4);

  /*
  * R represents the uncertainty in the position measurement
  * Generally, the parameters for the random noise measurement matrix will be provided by the sensor manufacturer. 
  * For this extended Kalman filter project, we have provided R matrices values for both the radar sensor and the lidar sensor.
  */
  
  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0     ,
              0     , 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0     , 0,
              0   , 0.0009, 0,
              0   , 0     , 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  
  //measurement matrix - laser  (5.13)
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;
  
  //Note: Hj_, the measurement matrix for radar does not require intialization
  //      It is calculated during each update step
  
  //the initial state covariance matrix P_  (Lesson 5.13)
  //NOTE: where the 1000 comes from is still a mystery. It is just a large number to start with...
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1000, 0,
             0, 0, 0, 1000;
  
  //the initial transition matrix F_ (Lesson 5.13)
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;
       
  //acceleration noise components (Lesson 5.9)
  //used to calculate each time the process covariance matrix Q
  noise_ax = 9; // variance on acceleration in x-direction (sigma^2_ax)
  noise_ay = 9; // variance on acceleration in y-direction (sigma^2_ay)
  
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (is_initialized_<2) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first two lidar measurements.
      * Create the covariance matrix. ... AB: Why does it say this ??? We do not do this here...
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // still initializing
  
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar is not as accurate, so always initialize from Lidar
    return;
  }
  
  ekf_.x_ = VectorXd(4); // px, py, vx, vy  (locations & velocities)
  ekf_.x_ << 1, 1, 1, 1;

    //if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
  //  // We should not get here.
  //  cerr<<"ERROR: FusionEKF::ProcessMeasurement(): should not get here\n";
    //  /**
    //  Convert radar from polar to cartesian coordinates and initialize state.
    //  */
    //}
    //else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
  if (is_initialized_==0){
      /**
      Initialize state.
      */
    ekf_.x_[0] = measurement_pack.raw_measurements_[0]; // px
    ekf_.x_[1] = measurement_pack.raw_measurements_[1]; // py
    previous_timestamp_ = measurement_pack.timestamp_;  
    }
  else{
    float previous_px = ekf_.x_[0]; 
    float previous_py = ekf_.x_[1];
    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;  //dt - expressed in seconds

    if (dt<1.e-10) { // If time difference to small, wait for next measurement. do not yet initialize velocities.
      return;
    }
    
    ekf_.x_[0] = measurement_pack.raw_measurements_[0]; // px
    ekf_.x_[1] = measurement_pack.raw_measurements_[1]; // py
    ekf_.x_[2] =(measurement_pack.raw_measurements_[0] - previous_px) / dt; // vx
    ekf_.x_[3] =(measurement_pack.raw_measurements_[1] - previous_py) / dt; // vy
    previous_timestamp_ = measurement_pack.timestamp_;
  }
  
    // Note: no need to predict or update as part of initialization
    ++is_initialized_;
    return;
  }
  

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
    - AB: Skip this step if elapsed time is zero. Just go do the Update.
     * Update the process noise covariance matrix Q.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
   
  // Lesson 5.13
  // compute the time elapsed between the current and previous measurements
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;  //dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;
  
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;
  
  // Modify the state transition matrix F so that the time is integrated
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;
  
  // calculate the process covariance matrix Q, 
  // representing the increase in uncertainty of this new measurement due to accelarations that we do not measure
  // Lesson 5.9
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ <<  dt_4/4.0*noise_ax, 0.0, dt_3/2.0*noise_ax, 0.0,
              0.0, dt_4/4.0*noise_ay, 0.0, dt_3/2.0*noise_ay,
              dt_3/2.0*noise_ax, 0.0, dt_2*noise_ax, 0.0,
              0.0, dt_3/2.0*noise_ay, 0.0, dt_2*noise_ay;

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
    Hj_ = tools_.CalculateJacobian(ekf_.x_);
    
    ekf_.R_ = R_radar_;
    ekf_.H_ = Hj_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    ekf_.R_ = R_laser_;
    ekf_.H_ = H_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  //cout << "x_ = " << ekf_.x_ << endl;
  //cout << "P_ = " << ekf_.P_ << endl;
}
