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
  float px = x_state(0);
   float py = x_state(1);
   float vx = x_state(2);
   float vy = x_state(3);

   float px2 = px*px;
   float py2 = py*py;
   float vx2 = vx*vx;
   float vy2 = vy*vy;
   float s1 = px2 + py2;

   MatrixXd Hj(3,4);

   Hj << px / sqrt(s1), py/sqrt(s1), 0, 0,
      (-1 * py)/s1,px / s1, 0, 0,
      py*(vx*py - vy*px) / (sqrt(s1) * s1), px*(vy*px - vx*py) / (sqrt(s1) * s1), px / sqrt(s1), py / sqrt(s1);


   return Hj;

}
