#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using std::vector;


Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
   VectorXd RMSE(4);
  if (estimations.size() != ground_truth.size() || estimations.size() == 0 ){
     std::cout << "Invalid estimation or ground_truth data" << std::endl;
     return RMSE;
  }
  // calculate sum
   for(int i = 0;i < estimations.size();i++){
      VectorXd residual = estimations[i] - ground_truth[i];
      VectorXd residual_sq = residual.array() * residual.array();
      RMSE += residual_sq; 
   }

   //calculate mean
   RMSE = RMSE / estimations.size();

   //calculate square

   RMSE = RMSE.array().square();
   
   
   return RMSE;
}