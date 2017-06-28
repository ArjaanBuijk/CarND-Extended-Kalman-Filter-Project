#include "ab_debug.h"

#include "kalman_filter.h"
#include <iostream>

using namespace std;

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  /*AB
  See: https://classroom.udacity.com/courses/cs373/lessons/48723604/concepts/486836600923
  x = estimate of current state [location and velocity)
  P = uncertainty covariance
  F = state transition matrix -> how does the position change as function of velocity
  H = measurement function -> to update state
  R = measurement noise
  
  Prediction update, using motion --> total probability; Convolution; Additions:
  new location: 			  x' = Fx+u  (u=motion vector)
  new uncertainty covariance: P' = F.P.F_transpose
  
  Measurement update --> Bayes; Product
  y = z - H.x
  S = H.P.H_transpose + R
  K = P.H_transpose.S_inverse
  new location: 			  x' = x + (K.y)
  new uncertainty covariance: P' = (I-K.H).P
  */
  if (AB_DEBUG)
	  debug_message("Entered KalmanFilter::Init()");

  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  if (AB_DEBUG)
	  debug_message("Entered KalmanFilter::Predict()");
	
  /**
  TODO:
    * predict the state
  */
}

void KalmanFilter::Update(const VectorXd &z) {
  if (AB_DEBUG)
	  debug_message("Entered KalmanFilter::Update()");

  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  if (AB_DEBUG)
	  debug_message("Entered KalmanFilter::UpdateEKF()");
	
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
}
