#include <iostream>
#include "tools.h"

#define PI 3.14159265

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  //Validate the estimations vector
  if(estimations.size() == 0 || estimations.size() != ground_truth.size()){
    cout<<"Error in size of Estimations vector or size mismatch with Ground Truth vector";
    return rmse;
  }
  
  //Accumulate the residual
  for(int i = 0; i < estimations.size(); ++i){
  VectorXd residual = estimations[i] - ground_truth[i];
  rmse = rmse + (residual.array() * residual.array()).matrix();
  }

  //Mean and Sqrt the error
  rmse = rmse / estimations.size();
  rmse = rmse.array().sqrt();

  return rmse;
}

void Tools::Normalize(double& angle){
  while (angle > PI) {angle -= 2.*PI;}
  while (angle < -PI) {angle += 2.*PI;}
}

void Tools::CalculateNIS(double& NIS, const VectorXd& y, const MatrixXd& S){
	
	NIS = y.transpose()*S.inverse()*y;
}