#include "kalman_filter.h"
#include <iostream>
#include <math.h>
#include "tools.h"

using namespace std;

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  //AB - We are not using this function...
  /*AB
  Definition of variables:
  See: https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/3612b91d-9c33-47ad-8067-a572a6c93837/concepts/81d536e6-4f6f-4970-94a2-eec7f0a20595
  
  Prediction update, using motion --> total probability; Convolution; Additions:
  new location:         x' = Fx+u  (u=motion vector)
  new uncertainty covariance: P' = F.P.F_transpose
  
  Measurement update --> Bayes; Product
  y = z - H.x
  S = H.P.H_transpose + R
  K = P.H_transpose.S_inverse
  new location:         x' = x + (K.y)
  new uncertainty covariance: P' = (I-K.H).P
  */
  
  x_ = x_in; // object state
  P_ = P_in; // object covariance matrix, which contains information about the uncertainty of the object's position and velocity
  F_ = F_in; // state transition matrix -> how does the position change as function of velocity
  H_ = H_in; // measurement matrix, 
  R_ = R_in; // measurement covariance matrix, representing measurement noise
  Q_ = Q_in; // process covariance matrix. representing increase in uncertainty of this new measurement
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * use information we have to predict the state of the pedestrian until the next measurement arrives
  * Note that the prediction step is identical for Lidar and Radar measurements. It just uses the
  * available information at time(k).

  */
  // See Lesson 5.13
  x_ = F_ * x_;              // this is just x(k+1) = x + vx*dt ; vx(k+1)=vx(k)
  MatrixXd Ft = F_.transpose(); 
  P_ = F_ * P_ * Ft + Q_;    // this updates the covariance (uncertainty in state) due to unknown 
                             // accelerations that happened during dt
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
    * Update state with lidar (=laser) measurement
    * Lidar Measurement Vector z = (px, py)  
  TODO:
    * update the state by using standard Kalman Filter equations
    * See Lessons: 3.11, 3.24 & 5.10-5.13
    * Code is copied from Lesson 5.13
   
  */
  // This function is for Laser, See Lesson 3.11, 3.24 & Lesson 5.13
  VectorXd z_pred = H_ * x_;      // (Lesson 5.10)
                                  // For laser, H_ is
                                  //    1, 0, 0, 0
                                  //    0, 1, 0, 0
                                  // So, this just transforms the predicted state vector x_ (px', py', vx', vy')
                                  // into the measurement space of the sensor z = (px, py)
                                  // This can be done simpler: z_pred[0] = x_[0]; z_pred[1] = x_[1];

  VectorXd y = z - z_pred;        // This calculates the error between measured and predicted state
  MatrixXd Ht = H_.transpose();    
  MatrixXd S = H_ * P_ * Ht + R_; // (Lesson 3.24)
                                  // "Project system uncertainty error into the measurement space..."
                                  // Basically, just transforming the covariance matrices to work
                                  // on (px, py) data only, ignoring (vx,vy) because those are not
                                  // measured by laser.
                                  // S is a (2x2) matrix, like R_laser_
                                  // This can also be done much quicker, without matrix multiplications,
                                  // which are mostly multiplication with 0s and thus a waste.
                                  
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;          // (Lesson 3.24)
                                  // K is often called the 'Kalman gain' 
  
  //new estimate                  // (Lesson 3.24)
  x_ = x_ + (K * y);              // update both position & velocity estimate
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;         // update covariance 
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
    * Update state with radar measurement
    * Radar Measurement Vector z = (ro, theta, ro_dot)  
  TODO:
    * update the state by using Extended Kalman Filter equations
    * See Lessons: 5.14-5.23
     
  */
  VectorXd y = z - tools_.Calculate_h(x_); // Lesson 5.14 & 5.20
  
  // Normalize the angle to be within -pi, pi
  float angle = y[1];
  y[1] = atan2(sin(angle), cos(angle));
  
  //NOTE: 
  //rest of this code is identical to code used in Update()
  //consider to refactor this into another function, but for now, I just copy/pasted.
  
  MatrixXd Ht = H_.transpose();    
  MatrixXd S = H_ * P_ * Ht + R_;  
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;      
  x_ = x_ + (K * y);        
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;       
  
}














