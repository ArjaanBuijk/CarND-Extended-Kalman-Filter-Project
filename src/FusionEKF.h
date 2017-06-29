#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
#include "tools.h"

class FusionEKF {
public:
  /**
  * Constructor.
  */
  FusionEKF();

  /**
  * Destructor.
  */
  virtual ~FusionEKF();

  /**
  * Run the whole flow of the Kalman Filter from here.
  */
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  /**
  * Kalman Filter update and prediction math lives in here.
  */
  KalmanFilter ekf_;

private:
  // check whether the tracking toolbox was initialized or not (first measurement)
  int is_initialized_; // 0: no , 
                       // 1: got one measurement, and initialized px, py
					   // 2: got two measurements, also initialized vx, vy 

  // previous timestamp
  long long previous_timestamp_;

  // tool object used to compute Jacobian and RMSE
  Tools tools;
  Eigen::MatrixXd R_laser_; //measurement covariance matrix - laser
  Eigen::MatrixXd R_radar_; //measurement covariance matrix - radar
  Eigen::MatrixXd H_laser_; //measurement matrix - laser
  Eigen::MatrixXd Hj_;      //measurement matrix (Jacobian) - radar
  
  // AB
  // acceleration noise components
  float noise_ax;
  float noise_ay;
  
  // to call helper functions
  Tools tools_;
};

#endif /* FusionEKF_H_ */
