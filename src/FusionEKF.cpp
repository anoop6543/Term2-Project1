#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  // Initialize P (object covariance) as zero matrix
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1000, 0,
                0, 0, 0, 1000;

    // Initialize F with dt = 0
    ekf_.F_ = MatrixXd(4, 4);
    ekf_.F_ << 1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;
    
    // Initialize H_laser
    H_laser_ << 1, 0, 0, 0,
                0, 1, 0, 0;
    
    // Initialize H
    ekf_.H_ = MatrixXd(4, 4);
    ekf_.H_ << 1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;

    cout << "FusionEKF::FusionEKF() initialized" << endl;

  int noise_ax = 9;
  int noise_ay = 9; 

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) 
{


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) 
  {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    //set the state with the initial location and zero velocity
    //ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    // Create process covariance matrix
    ekf_.Q_ = Eigen::MatrixXd(4,4);

    float px;
    float py;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) 
    {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float rho = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
        
      px = rho*cos(phi);
      py = rho*sin(phi);

      cout <<"Initialize Radar parameters and convert co ords to cartesian from polar "<< endl;

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) 
    {
      /**
      Initialize state.
      */
      cout << "initialize laser parameters" << endl;

      px = measurement_pack.raw_measurements_[0];
      py = measurement_pack.raw_measurements_[1];

    }

    // Handle small px, py
    if(fabs(px) < 0.0001)
    {
        px = 0.1;
        cout << "initialized px very low" << endl;
    }

    if(fabs(py) < 0.0001)
    {
        py = 0.1;
        cout << "initialized py very low" << endl;
    }

    ekf_.x_ << px, py, 0, 0;
    cout << "ekf_.x_: " << ekf_.x_ << endl;

    previous_timestamp_ = measurement_pack.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
    cout << "Start predicting" << endl;
    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0; //dt - expressed in seconds
    cout << "dt: " << dt << endl;
    previous_timestamp_ = measurement_pack.timestamp_;

    float dt_2 = dt * dt;
    float dt_3 = dt_2 * dt;
    float dt_4 = dt_3 * dt;

     cout << "Modifying F matrix" << endl;
    
    //Modify the F matrix so that the time is included
    ekf_.F_(0, 2) = dt;
    ekf_.F_(1, 3) = dt;
    
    cout << "F_: " << ekf_.F_ << endl;
    
    // noise values
    float noise_ax = 9;
    float noise_ay = 9;
    
    // Update the process covariance matrix Q
    ekf_.Q_ <<  dt_4/4 * noise_ax, 0, dt_3/2 * noise_ax, 0,
                0, dt_4/4 * noise_ay, 0, dt_3/2 * noise_ay,
                dt_3/2 * noise_ax, 0, dt_2 * noise_ax, 0,
                0, dt_3/2 * noise_ay, 0, dt_2 * noise_ay;
    cout << "Done updating Q" << endl;


  ekf_.Predict();
  cout << "Prediction completed" << endl;
  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    cout << "Radar Updates" << endl;
    ekf_.hx_ = VectorXd(3);

    float px = ekf_.x_[0];
    float py = ekf_.x_[1];
    float vx = ekf_.x_[2];
    float vy = ekf_.x_[3];

    float rho;
    float phi;
    float rhdot;

    if(fabs(px) < 0.0001 or fabs(py) < 0.0001)
    {
      if(fabs(px) < 0.0001)
      {
        px = 0.0001;
        cout << "px very low" << endl;
      }

      if(fabs(py) < 0.0001)
      {
        py = 0.0001;
        cout << "py very low" << endl;
      }

      rho = sqrt(px*px + py*py);
      phi = 0;
      rhdot = 0;
    }
    else
    {
      rho = sqrt(px*px + py*py);
      phi = atan2(py,px); //  arc tangent of y/x, in the interval [-pi,+pi] radians.
      rhdot = (px*vx + py*vy) /rho;
    }

    ekf_.hx_ << rho, phi, rhdot;

    //set H_ to Hj when updating with radar measurement

    Hj_ = tools.CalculateJacobian(ekf_.x_);

    //don't update measuremenmt if we can't compute jacobian 
    if(Hj_.isZero(0))
    {
      cout << "Hj is zero" << endl;
      return;
    }

    ekf_.H_ = Hj_;

    ekf_.R_ = R_radar_;

    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // Laser updates
    cout << "Laser update" << endl;
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
