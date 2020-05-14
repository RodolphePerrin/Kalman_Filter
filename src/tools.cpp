#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  
    VectorXd RMSE(4);
    RMSE<< 0,0,0,0;
    int n = estimations.size();
    
    if(n != ground_truth.size())
    {
        cout<<"Error in CalculateRMSE: sizes of estimations and ground truth vectors differs."<<endl;
        return RMSE;
    }
    
    if (n==0)
    {
       cout<<"Error in CalculateRMSE: sizes of estimations and ground truth are null."<<endl;
        return RMSE;
    }
    
    //Compute RMSE
    
    for (int t= 0; t<n; t++ )
    {
        VectorXd diff = estimations[t]-ground_truth[t];
        diff= diff.array()*diff.array();
        
        RMSE += diff;
    }
    RMSE = RMSE/n;
    RMSE = RMSE.array().sqrt();
    
    return RMSE;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  
    //define the state variables
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);
    
    //define Jacobian matrix Hj
    MatrixXd Hj(3,4);
    
    float rho = sqrt(px*px+py*py);
    float rho_2 =rho*rho;
    float rho_3 = rho_2*rho;
    float diff1 = vx*py - vy*px;
    float diff2 = vy*px - vx*py;
    
    if (rho==0)
    {
        //cannot divide by zero
        return Hj;
    }
    
    //Compute Hj
    Hj <<     px/rho,            py/rho,           0,        0,
            - py/rho_2,          px/rho_2,         0,        0,
    py*(diff1)/rho_3,   px*(diff2)/rho_3,     px/rho,    py/rho;
    
    return Hj;
    
}
