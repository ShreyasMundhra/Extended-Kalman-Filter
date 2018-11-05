#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  
  if(estimations.size() == 0){
    cout << "Zero estimations" << endl;
    return rmse;
  }
  else if(estimations.size() != ground_truth.size()){
    cout << "Estimations and ground truths do not have equal sizes" << endl;
    return rmse;
  }
    
	//accumulate squared residuals
	for(unsigned int i=0; i < estimations.size(); ++i){
        // ... your code here
        VectorXd diff = estimations[i] - ground_truth[i];
        VectorXd diff2 = diff.array()*diff.array();
		rmse = rmse + diff2;
	}

	//calculate the mean
	// ... your code here
	rmse /= estimations.size();

	//calculate the squared root
	// ... your code here
	rmse = rmse.array().sqrt();

	//return the result
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  double px = x_state[0];
  double py = x_state[1];
  double vx = x_state[2];
  double vy = x_state[3];
  
  double rho2 = px*px + py*py;
  double rho = sqrt(rho2);
  double rho32 = rho2 * rho;
  
  MatrixXd Hj = MatrixXd(3, 4);
  if(rho <= 0.0001){
    return Hj;
  }
  
  Hj << px/rho, py/rho, 0.0, 0.0,
        -py/rho2, px/rho2, 0.0, 0.0,
        py*(vx*py - vy*px)/rho32, px*(vy*px - vx*py)/rho32, px/rho, py/rho;
  
  return Hj;
}
