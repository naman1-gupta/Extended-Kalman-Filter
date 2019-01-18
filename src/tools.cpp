#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  VectorXd rmse(4);

   rmse << 0,0,0,0;

   if(estimations.size() != ground_truth.size() || estimations.size() == 0){
      cout << "Something is wrong. Either the estimations are empty or estimations and ground truth are not equal.";
      return rmse;
   }

   else {
      for(int i = 0; i < estimations.size(); i++){

         VectorXd residual = estimations[i] - ground_truth[i];

         residual = residual.array() * residual.array();
         rmse += residual;

      }
      rmse = rmse/estimations.size();

      rmse = rmse.array().sqrt();

      return rmse;
   }
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */

}
