#include "FusionEKF.h"
//#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
//#include <sstream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  //is_initialized_ = false;

  //previous_timestamp_ = 0.0;

  // initializing matrices
  //R_laser_ = MatrixXd(2, 2);
  //R_radar_ = MatrixXd(3, 3);
 // H_laser_ = MatrixXd(2, 4);
  //Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  //R_laser_ << 0.0225, 0,
       // 0, 0.0225;

  //measurement covariance matrix - radar
  //R_radar_ << 0.09, 0, 0,
       // 0, 0.0009, 0,
       // 0, 0, 0.09;

  //noise_ax_ = 9;
  //noise_ay_ = 9;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */


}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
   
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    if (!ekf_.isInitialized()) {
    	ekf_.Init(measurement_pack);
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
  //the elapsed time in seconds!
  

  ekf_.InitMatricesTimestamp(measurement_pack.timestamp_);
  //predict
  ekf_.Predict();

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
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
