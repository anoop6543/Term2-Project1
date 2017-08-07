#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
//using std::vector;
using namespace std;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;

}

void KalmanFilter::Predict() 
{
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;

  //cout << "KalmanFilter::Predict()" << endl;
  //cout << "x_: " << x_ << endl;
  //cout << "F_: " << F_ << endl;
  //cout << "P_: " << P_ << endl;
  //cout << "Q_: " << Q_ << endl;
}

void KalmanFilter::Update(const VectorXd &z) 
{
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) 
{
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  
  VectorXd z_pred = hx_;
  VectorXd y = z - z_pred;
  bool in_pi = false;
  while (in_pi == false) 
  {
    if (y(1) > 3.14159) 
    {
      //cout << "phi > pi" << endl;
      y(1) = y(1) - 6.2831;
    }
    else if (y(1) < -3.14159) 
    {
      //cout << "phi < -pi" << endl;
      y(1) = y(1) + 6.2831;
    } 
    else 
    {
      in_pi = true;
    }
  }

    //cout << "y: " << y << endl;
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;
    //cout << "K_: " << K << endl;
    
    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    //cout << "I: " << I << endl;
    P_ = (I - K * H_) * P_;
}
