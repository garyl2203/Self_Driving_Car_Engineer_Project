//#include "FusionEKF.h"
#include <iostream>
#include "kalman_filter.h"

#include "tools.h"
#include "measurement_package.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;




//here we set up our intialized matrices
KalmanFilter::KalmanFilter() {
   //stating matrix
   x_ = VectorXd(4);
   x_ << 1,1,1,1;

   //uncertainty covariance matrix
   P_ = MatrixXd(4,4);
   P_ << 1,0,0,0,
          0,1,0,0,
          0,0,1000,0,
          0,0,0,1000;

   //initial transition matrix F_
   F_ = MatrixXd(4,4);
   F_ << 1,0,1,0,
          0,1,0,1,
          0,0,1,0,
          0,0,0,1;

   // prediction uncertainty covariance 
   Q_ = MatrixXd(4,4);
   Q_ << 0,0,0,0,
           0,0,0,0,
           0,0,0,0,
           0,0,0,0;

   // identity matrix
   I_ = MatrixXd::Identity(4,4);       

   // sensor measurement noise
   u_ = VectorXd(4);
   u_ << 0,0,0,0;

   // measurement covariance matrix LASER and RADAR  
   previous_timestamp_ = 0.0;

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
 
   noise_ax_ = 9;
   noise_ay_ = 9;
   is_initialized_= false;
}




KalmanFilter::~KalmanFilter() {}


bool KalmanFilter::isInitialized() {
    return is_initialized_;
}


// here we take our Radar data and convert to cartesian coordinates! We also take in our Lidar data,
void KalmanFilter::Init(const MeasurementPackage &measurement_pack) {
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    float x = measurement_pack.raw_measurements_[0]*cos(measurement_pack.raw_measurements_[1]);
    float y = measurement_pack.raw_measurements_[0]*sin(measurement_pack.raw_measurements_[1]);
    x_ << x, y, 0, 0;
  } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
     x_<< measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
  }

  //done initiallizing, no need to predict or update
  is_initialized_ = true;
}



void KalmanFilter::InitMatricesTimestamp(long long timestamp) {
    float dt = (timestamp_ - previous_timestamp_) / 1000000.0; //dt - expressed in seconds
    previous_timestamp_ = timestamp_;

    //Modify the F matrix so that the time is integrated
    F_(0, 2) = dt;
    F_(1, 3) = dt;

    float dt_2 = dt * dt;
    float dt_3 = dt_2 * dt;
    float dt_4 = dt_3 * dt;


    //set the process covariance matrix Q
    Q_ = MatrixXd(4, 4);
    Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
             0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
             dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
             0, dt_3/2*noise_ay, 0, dt_2*noise_ay;
}






void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state  (u_ is the noise)
  */
   x_ = F_ * x_ + u_;
   MatrixXd Ft = F_.transpose();
   P_ = F_ * P_ * Ft + Q_;
}


void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
   
   MatrixrXd y = z - H_laser_ * x_;

   MatrixXd S = H_laser_ * P_ * H_laser_.transpose() + R_laser_;
   
   MatrixXd K = P_ *  H_laser_.transpose() * S.inverse();

   //new estimate
   x_ = x_ + (K * y);
   long x_size = x_.size();
   MatrixXd I = MatrixXd::Identity(x_size, x_size);
   P_ = (I - K * H_laser_) * P_;
}



void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
   if (z(0) == 0 && z(1) == 0 && z(2) == 0) {
       return;
  }

  //I need the jacobian here
  Tools::CalculateJacobian(x_, Hj_);

  MatrixXd y = z - Tools::h(x_);
  MatrixXd S = Hj_ * P_ * Hj_.transpose() + R_radar_;
  MatrixXd K = P_ * Hj_.transpose() * S.inverse();
  x_ = x_ + K * y;
  P_ = (I_ - K * Hj_) * P_; 

}
