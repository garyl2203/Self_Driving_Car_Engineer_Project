#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"
#include "measurement_package.h"

class KalmanFilter {
public:

  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // state transistion matrix
  Eigen::MatrixXd F_;

  // process covariance matrix
  Eigen::MatrixXd Q_;

  //measurement noise
  Eigen::MatrixXd R_laser_;
  //measurement function
  Eigen::MatrixXd H_laser_;

  //measurement noise
  Eigen::MatrixXd R_radar_;
  //measurement function
  //Eigen::MatrixXd H_radar_;
  //measurment function
  Eigen::MatrixXd Hj_;



  //identity matrix
  Eigen::MatrixXd I_;

  //noise error
  Eigen::VectorXd u_;

  long long previous_timestamp_;
  int noise_ax_;
  int noise_ay_;
  bool is_initialized_;


  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  /**
   * Init Initializes Kalman filter
   * @param x_in Initial state
   * @param P_in Initial state covariance
   * @param F_in Transition matrix
   * @param H_in Measurement matrix
   * @param R_in Measurement covariance matrix
   * @param Q_in Process covariance matrix
   */
  bool isInitialized();


  //feed in our data
  void Init(const MeasurementPackage &measurement_pack);

  void InitMatricesTimestamp(long long timestamp);

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  void Predict();

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  void Update(const Eigen::VectorXd &z);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  void UpdateEKF(const Eigen::VectorXd &z);

};

#endif /* KALMAN_FILTER_H_ */
