#include "ab_debug.h"

#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  if (AB_DEBUG)
    debug_message("Entered Tools::CalculateRMSE()");
  /**
  TODO:
    * Calculate the RMSE here.
	* See Lesson 5.23
  */
  VectorXd rmse(4);
	rmse << 0,0,0,0;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	if(estimations.size() != ground_truth.size()
			|| estimations.size() == 0){
		cout << "Invalid estimation or ground_truth data" << endl;
		return rmse;
	}

	//accumulate squared residuals
	for(unsigned int i=0; i < estimations.size(); ++i){

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
  if (AB_DEBUG)
    debug_message("Entered Tools::CalculateJacobian()");
  /**
  TODO:
    * Calculate a Jacobian here.
	* See Lesson 5.19
  */
  MatrixXd Hj(3,4);
  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  
  //pre-compute a set of terms to avoid repeated calculation
  float c1 = px*px+py*py;
  float c2 = sqrt(c1);
  float c3 = (c1*c2);
  
  //check division by zero
  if(fabs(c1) < 0.0001){
  	cout << "CalculateJacobian () - Error - Division by Zero" << endl;
  	return Hj;
  }
  
  //compute the Jacobian matrix
  Hj << (px/c2), (py/c2), 0, 0,
  	  -(py/c1), (px/c1), 0, 0,
  	  py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;
  
  return Hj;
}

VectorXd Tools::Calculate_h(const VectorXd& x_state){
  if (AB_DEBUG)
    debug_message("Entered Tools::Calculate_h()");
  /**
   * Calculate the h function to transform state (px, py, vx, vy) into 
   * radar measurement space (ro, theta, ro_dot)
   */
  VectorXd h(3);
  
  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
   
  if (AB_DEBUG){
    cout<<"px="<<px<<'\n';
    cout<<"py="<<py<<'\n';
    cout<<"vx="<<vx<<'\n';
    cout<<"vy="<<vy<<'\n';
  }
  // h function (Lesson 5.14 & 5.20)
  h[0] = sqrt(px*px+py*py);
  if (AB_DEBUG)
    cout<<"h[0]="<<h[0]<<'\n';
    
  h[1] = atan2(py, px);
  if (AB_DEBUG)
    cout<<"h[1]="<<h[1]<<'\n';
  if (h[1]>M_PI) {
    h[1]= h[1]-2*M_PI;
    if (AB_DEBUG)
      cout<<"NORMALIZED h[1]="<<h[1]<<'\n';
  }
  else if (h[1]< -M_PI) {
    h[1]= h[1]+2*M_PI;
    if (AB_DEBUG)
      cout<<"NORMALIZED h[1]="<<h[1]<<'\n';
  }
  
  if (h[0]<1.e-10){
    h[2] = 0.0;
    if (AB_DEBUG)
      cout<<"SMALL VALUE of h[0], setting h[2] to zero\n";
  }
  else{
    h[2] = (px*vx+py*vy) / h[0];
    if (AB_DEBUG)
      cout<<"h[2]="<<h[2]<<'\n';
  }
  
  if (AB_DEBUG){
	 ostringstream s1;
	 s1<<"\nh(0)="+std::to_string(h[0]);
   s1<<"\nh(1)="+std::to_string(h[1]);
   s1<<"\nh(2)="+std::to_string(h[2]);
	 
	 debug_message("Tools::Calculate_h: ", s1.str());
	}
   
   return h;  
}
