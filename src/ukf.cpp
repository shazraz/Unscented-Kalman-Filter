#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

#define EPS 0.00007
#define RHO_EPS 0.0001
#define PSIDOT_EPS 0.0001

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1; //need to tune

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.3; //need to tune

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  //Set initialization to false
  is_initialized_ = false;

  //Set state dimensions
  n_x_ = 5;
  n_aug_ = 7;
  n_sig_ = 2 * n_aug_ + 1;

  // Set Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  //Initialize and set the process noise matrix
  Q_ = MatrixXd(2,2);
  Q_ << std_a_*std_a_, 0, 0, std_yawdd_*std_yawdd_;

  //Initialize the matrix of predicted sigma points 
  Xsig_pred_ = MatrixXd::Zero(n_x_, n_sig_);

  //Initialize the weights vector
  weights_ = VectorXd::Zero(n_sig_);

  //Initialize the laser sensor transformation matrix
  H_laser_ = MatrixXd::Zero(2,5);
  H_laser_ << 1,0,0,0,0,
              0,1,0,0,0;

  //Initialize the laser measurement uncertainty matrix
  R_laser_ = MatrixXd::Zero(2,2);
  R_laser_ << std_laspx_*std_laspx_, 0,
              0, std_laspy_*std_laspy_;

  //Initialize the radar measurement uncertainty matrix
  R_radar_ = MatrixXd::Zero(3,3);
  R_radar_ << std_radr_*std_radr_, 0, 0,
              0, std_radphi_*std_radphi_, 0,
              0, 0, std_radrd_*std_radrd_;

  //Initialize NIS values to zero
  NIS_radar_ = 0;
  NIS_laser_ = 0;

}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {

  if (!is_initialized_) {
   
    cout << "Initializing UKF.. \n";

    // Set initial state values
    x_ << 1, 1, 0, 0, 0; //px, py, v, psi, psi_dot

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {

      double rho = meas_package.raw_measurements_[0];
      double theta = meas_package.raw_measurements_[1];
      double rho_dot = meas_package.raw_measurements_[2];

      //Account for small values in intial measurements  
      x_(0) = (rho*cos(theta) > EPS) ? rho*cos(theta) : EPS;
      x_(1) = (rho*sin(theta) > EPS) ? rho*sin(theta) : EPS;
       
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {

      double px = meas_package.raw_measurements_[0];
      double py = meas_package.raw_measurements_[1];
      
      //Account for small values in intial measurements
      x_(0) = (px > EPS) ? px : EPS;
      x_(1) = (py > EPS) ? px : EPS;

    }

    //Capture the timestamp
    time_us_ = meas_package.timestamp_;
   
    //assign initial values to the covariance matrix, P. Adjust the variance values to reflect uncertainty in initial state
    P_ << 0.1,0,0,0,0,
          0,0.1,0,0,0,
          0,0,50,0,0,
          0,0,0,50,0,
          0,0,0,0,50;    
    
    // Set weights
    double weight_0 = lambda_/(lambda_ + n_aug_);
    double weight = 0.5/(n_aug_+lambda_);
    weights_.fill(weight);
    weights_(0) = weight_0;
    
    is_initialized_ = true;
    cout << "Completed initialization of UKF.\n";
    return;
  }

  //Calculate the time difference between the current and previous measurement
  long long delta_t_us = meas_package.timestamp_ - time_us_;
  double delta_t = static_cast<double>(delta_t_us) / static_cast<double>(1e6);
  time_us_ = meas_package.timestamp_;

  //Perform the prediction step
  Prediction(delta_t);
  //cout << "Prediction step complete...\n";
  //cout << "x_: " << x_ << "\n";
  //cout << "P_: " << P_ << "\n"; 

  //Perform the update step 
  //LIDAR update - check to make sure the use_laser_ flag is on and that a non-zero measurement is recieved for px and py
  if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_ 
  	&& !(fabs(meas_package.raw_measurements_[0]) < EPS && fabs(meas_package.raw_measurements_[1]) < EPS)) {
    
    
    UpdateLidar(meas_package);
    //cout << "Using LIDAR measurement to update...\n";
    //cout << "x_: " << x_ << "\n";
    //cout << "P_: " << P_ << "\n";  

  } 
  //RADAR update - check to make sure the use_radar_ flag is on and a non-zero measurement is recieved for rho
  else if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_ && fabs(meas_package.raw_measurements_[0]) > RHO_EPS) {
   
    UpdateRadar(meas_package);
    //cout << "Using RADAR measurement to update...\n";
    //cout << "x_: " << x_ << "\n";
    //cout << "P_: " << P_ << "\n";   
  }
  else {
  	cout << "Skipping update step for Sensor: " << meas_package.sensor_type_ <<"\n";
  }

}

void UKF::Prediction(double delta_t) {

  /* ====== CREATE AUGMENTED SIGMA POINTS ====== */
  VectorXd x_aug = VectorXd::Zero(7);
  MatrixXd P_aug = MatrixXd::Zero(7, 7);
  MatrixXd Xsig_aug = MatrixXd::Zero(n_aug_, n_sig_);

  //Create augmented mean state
  x_aug << x_, 0, 0;
  //Create augment covariance matrix
  P_aug.block(0, 0, n_x_, n_x_) = P_;
  P_aug.block(n_x_, n_x_, 2, 2 ) = Q_;

  //Create square root matrix
  MatrixXd P_sqrt = P_aug.llt().matrixL();
  //create augmented sigma points
  Xsig_aug.col(0) = x_aug;
  for(int i = 0; i < n_aug_; i++){
      Xsig_aug.col(i + 1) =          x_aug + sqrt(lambda_ + n_aug_) * P_sqrt.col(i);
      Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * P_sqrt.col(i);    
  }

  
  /* ====== TRANSFORM THE SIGMA POINTS USING THE PROCESS MODEL, F(X) ====== */
  //Fill the predicted sigma points matrix with zeros in every prediction step just to be safe
  Xsig_pred_.fill(0.0);
  double delta_t2 = delta_t * delta_t;

  for (int i = 0; i < Xsig_aug.cols(); i++){
    //Initalize the posterior
    VectorXd posterior_aug = Xsig_aug.col(i);
    VectorXd posterior = posterior_aug.head(5);
    
    //Initialize the vector to hold the output of the procecss model
    VectorXd fx = VectorXd::Zero(5);
    
    //Initialize the noise vector to zero
    VectorXd noise = VectorXd::Zero(5);
    
    //Calculate the noise vector
    noise << 0.5 * (delta_t2) * cos(posterior_aug(3)) * posterior_aug(5),
             0.5 * (delta_t2) * sin(posterior_aug(3)) * posterior_aug(5),
             delta_t * posterior_aug(5),
             0.5 * delta_t2 * posterior_aug(6),
             delta_t * posterior_aug(6);
             
    //check if yaw rate is close to zero
    if (fabs(posterior(4)) < PSIDOT_EPS){
        fx(0) = posterior_aug(2)*cos(posterior_aug(3))*delta_t;
        fx(1) = posterior_aug(2)*sin(posterior_aug(3))*delta_t;   
    }
    else {
        fx(0) = (posterior_aug(2)/posterior_aug(4)) 
                * (sin(posterior_aug(3) + delta_t*posterior_aug(4)) 
                  - sin(posterior_aug(3)));
        fx(1) = (posterior_aug(2)/posterior_aug(4)) 
               * (cos(posterior_aug(3)) 
                  - cos(posterior_aug(3) + delta_t*posterior_aug(4)));
        fx(3) = delta_t * posterior_aug(4);
    }
    
    //Populate the columns of the predicted sigma points matrix
    Xsig_pred_.col(i) = posterior + fx + noise;
  }

  /* ====== GENERATE THE NEW PREDICTED MEAN AND COVARIANCE ====== */
  //Calculate the new mean using the predicted sigma points

  //Vectorized implementation of predicted mean
  x_ = Xsig_pred_ * weights_; //5x15 matrix * 15x1 matrix = 5x1 matrix

  //Reset the value of P_
  P_.fill(0.0);
  for (int i = 0; i < n_sig_; i++) {  
      // state difference
      VectorXd x_diff = Xsig_pred_.col(i) - x_;
      //Normalize the yaw angle
      tool_.Normalize(x_diff(3));
      //Set the value of P_
      P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
  }

}

void UKF::UpdateLidar(MeasurementPackage meas_package) {

  MatrixXd Ht = H_laser_.transpose();
  MatrixXd PHt = P_ * Ht;
  VectorXd z = meas_package.raw_measurements_;

  VectorXd y = z - H_laser_ * x_;
  MatrixXd S = H_laser_ * PHt + R_laser_;
  MatrixXd K = PHt * S.inverse();

  //Calculate NIS
  tool_.CalculateNIS(NIS_laser_, y, S);
  //cout << "Laser NIS is: " << NIS_laser_ << "\n";

  //Update state
  x_ = x_ + (K * y);

  //Update covariance
  P_ = P_ - (K * H_laser_ * P_);
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {

  //Declare matrices to be used in this function
  MatrixXd Zsig = MatrixXd::Zero(3,n_sig_);
  VectorXd z_pred = VectorXd::Zero(3);
  VectorXd z = meas_package.raw_measurements_;
  MatrixXd S = MatrixXd::Zero(3,3);
  MatrixXd Tc = MatrixXd::Zero(n_x_, 3);

  /* ====== TRANSFORM THE PREDICTED SIGMA POINTS INTO MEASUREMENT SPACE, H(X) ====== */
  for (int i=0; i<Xsig_pred_.cols(); i++){
    
    //Extract the predicted sigma points
    VectorXd sigmapoints_x = Xsig_pred_.col(i);
    double px = sigmapoints_x(0);
    double py = sigmapoints_x(1);
    double v = sigmapoints_x(2);
    double psi = sigmapoints_x(3);
    double psi_dot = sigmapoints_x(4);
    
    //Convert sigma points to measurement space
    double rho = sqrt(px*px + py*py);
    //Account for division by zero by thresholding value of rho
    rho = (rho > RHO_EPS) ? rho : RHO_EPS;
    double phi = atan2(py,px);
    double phi_dot = (px*v*cos(psi) + py*v*sin(psi))/rho;    
    
    Zsig(0, i) = rho;
    Zsig(1, i) = phi;
    Zsig(2, i) = phi_dot;
    
    //calculate mean predicted measurement
    z_pred = z_pred + (weights_(i) * Zsig.col(i));
  }

  /* ====== CALCULATE THE UPDATED MEASUREMENT COVARIANCE MATRIX ====== */

  //Combine the calculation of the cross correlation matrix, Tc
  for (int i=0; i<Xsig_pred_.cols(); i++){
      //Calculate the residuals
      VectorXd x_diff = Xsig_pred_.col(i) - x_;  
      MatrixXd z_diff = Zsig.col(i) - z_pred;
      
      //Make sure yaw is normalized
      tool_.Normalize(x_diff(3));
      //Make sure phi is normalized
      tool_.Normalize(z_diff(1));
      
      //Accumulate the measurement covariance matrix
      S += weights_(i)*z_diff*z_diff.transpose();
      //Accumulate the cross correlation matrix
      Tc += weights_(i)*x_diff*z_diff.transpose();
  }
  
  S += R_radar_;

  //Calculate NIS
  VectorXd y = meas_package.raw_measurements_ - z_pred;
  tool_.CalculateNIS(NIS_radar_, y, S);
  //cout << "Radar NIS is: " << NIS_radar_ << "\n";

  /* ====== PERFORM THE UPDATE ====== */

  //calculate Kalman gain K;
  MatrixXd K = Tc*S.inverse();

  //Update the mean state
  x_ = x_ + K*(z - z_pred);
  //Update the state covariance matrix
  P_ = P_ - K*S*K.transpose();  

}