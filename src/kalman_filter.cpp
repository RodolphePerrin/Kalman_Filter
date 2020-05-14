#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
                        Eigen::MatrixXd &H_in,Eigen::MatrixXd &Hj_in, Eigen::MatrixXd &R_laser_in,
                        Eigen::MatrixXd &R_radar_in, Eigen::MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  Hj_ = Hj_in;
  R_ = R_laser_in;
  R_ekf = R_radar_in;
  Q_ = Q_in;
  I_ = Eigen::MatrixXd::Identity(4,4);
    
}


void KalmanFilter::Predict() {
    
    //Reuse Kalman filter prediction equation.
    x_=F_*x_;
    MatrixXd F_t = F_.transpose();
    P_=F_*P_*F_t + Q_;
    
}

void KalmanFilter::Update(const VectorXd &z) {
    
    //Update the state with the help of Kalman equations.
    
    VectorXd y_ = z - H_*x_;
    MatrixXd S_ = H_*P_*H_.transpose() + R_;
    MatrixXd K_ = P_*H_.transpose()*S_.inverse();
    
    //State and Covariance updated beliefs
    x_ = x_ + K_*y_;
    P_ = (I_ - K_*H_)*P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {

    //Define state
    
    float px = x_[0];
    float py = x_[1];
    float vx = x_[2];
    float vy = x_[3];
    
    float rho = sqrt(px*px + py*py);
     if (rho ==0)
     {
         cout<<" Error in Update EKF : rho is null and this leads to a forbidden division by zero."<<endl;
         return;
     }
    
    //Calculate Jacobian H and linearized function hofx
   
    MatrixXd Hj = tools.CalculateJacobian(x_);
   
    
    
    VectorXd hofx(3);
    hofx << rho, atan2(py,px), (px*vx + py*vy) /rho;
    
    //Update the state using Extended Kalman Filter equations
    
    VectorXd y = z - hofx;
    MatrixXd S = Hj*P_*Hj.transpose() + R_ekf;
    MatrixXd K = P_*Hj.transpose()*S.inverse();
    x_ = x_ + K*y;
    P_ = (I_ - K*Hj)*P_;
}
