#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
# define PI 3.14159265358979323846

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
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
  std_a_ = 0.9;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.9;
  
  n_x_=5;
  n_aug_=7;
  lambda_=3-n_aug_;
  
  NIS_LASER_ = 0.0;
  NIS_RADAR_ = 0.0;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
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
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.  
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
	// first measurement
	cout << "UKF: " << endl;
	if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
	  x_(0)=meas_package.raw_measurements_[0]*cos(meas_package.raw_measurements_[1]);
	  x_(1)=meas_package.raw_measurements_[0]*sin(meas_package.raw_measurements_[1]);
	  x_(2)=5;
	  x_(3)=0;
	  x_(4)=0;
	}
	else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
	  x_(0)=meas_package.raw_measurements_[0];
	  x_(1)=meas_package.raw_measurements_[1];
	  x_(2)=5;
	  x_(3)=0;
	  x_(4)=0;
	}
	
	P_ << 1, 0, 0, 0, 0,
			  0, 1, 0, 0, 0,
			  0, 0, 1, 0, 0,
			  0, 0, 0, PI, 0,
			  0,0,0,0,PI/9;
			  
	previous_timestamp_=meas_package.timestamp_;
	
	// done initializing, no need to predict or update
	is_initialized_ = true;
	return;
	}
	/*****************************************************************************/
	float dt = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0;
	Prediction(dt);
	
	if ((meas_package.sensor_type_ == MeasurementPackage::LASER)&&(use_laser_==true)){ // if LASER is the incoming msmt
	UpdateLidar(meas_package);
	}
	else if ((meas_package.sensor_type_ == MeasurementPackage::RADAR)&&(use_radar_==true)){ // if radar is the incoming msmt
	UpdateRadar(meas_package);
	}	
	previous_timestamp_ = meas_package.timestamp_; // latch the previous time stamp before exiting loop
	
	// ----------- PRINT OUT THE STATE AND COVARIANCE MATRIX ----------//
	cout << "x_ = " << x_ << endl;
	cout << "P_ = " << P_ << endl;
}


void UKF::Prediction(double delta_t) {
  
  // ********** GENERATING SIGMA POINTS (state plus noise augmented) *********** //
  VectorXd x_aug = VectorXd(n_aug_);
  //create augmented mean state
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  //create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;

  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  Xsig_aug.fill(0.0);
  //create augmented sigma points
  Xsig_aug.col(0)  = x_aug;
  for (int i = 0; i< n_aug_; i++){
    Xsig_aug.col(i+1)       = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
  }
  
  //*********** PREDICT SIGMA POINTS ******************* //
  
  Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1); // Declare a matrix to store predicted sigma points after processing through model
  Xsig_pred.fill(0.0); // Initialize with 0 for the sigma state prediction matrix
  
  for (int i = 0; i< 2*n_aug_+1; i++)
  {
    //extract values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    }
    else {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    //add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    //write predicted sigma point into right column
    Xsig_pred(0,i) = px_p;
    Xsig_pred(1,i) = py_p;
    Xsig_pred(2,i) = v_p;
    Xsig_pred(3,i) = yaw_p;
    Xsig_pred(4,i) = yawd_p;
  }
  
  // ************* CALCULATE MEAN STATE & COVARIANCE *********** //
  
  //----------- WEIGHTS -------------------------------------------//
  weights_ = VectorXd(2*n_aug_+1); // declare a weights vector
  
  // set weights
  double weight_0 = lambda_/(lambda_+n_aug_);
  weights_(0) = weight_0;
  for (int i=1; i<2*n_aug_+1; i++) {  //2n+1 weights_
    double weight = 0.5/(n_aug_+lambda_);
    weights_(i) = weight;
  }  
  
  //----------- PREDICT MEAN STATE --------------------------------//

  //create vector for predicted state
  VectorXd x = VectorXd(n_x_);

  //predicted state mean
  x.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    x = x+ weights_(i) * Xsig_pred.col(i);
  }
  
  //----------- PREDICT MEAN COVARIANCE ---------------------------//

  //create covariance matrix for prediction
  MatrixXd P = MatrixXd(n_x_, n_x_);
  
  //predicted state covariance matrix
  P.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points

    // state difference
    VectorXd x_diff = Xsig_pred.col(i) - x;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P = P + weights_(i) * x_diff * x_diff.transpose() ;
	
	x_=x;
	P_=P;
  }   
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  	MatrixXd H_laser_ = MatrixXd(2, 5);
	H_laser_<< 1,0,0,0,0,
			0,1,0,0,0;
	
	VectorXd z_pred = H_laser_ * x_;
	VectorXd z = meas_package.raw_measurements_;
	
	MatrixXd R_laser_ = MatrixXd(2, 2);
	R_laser_ << std_laspx_*std_laspx_, 0,
        0, std_laspy_*std_laspy_;
	
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_laser_.transpose();
	MatrixXd S = H_laser_ * P_ * Ht + R_laser_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_laser_) * P_;
	
	NIS_LASER_ = y.transpose() * Si * y;
	NIS_RADAR_ = -1.0;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  
  //transform sigma points into measurement space
  MatrixXd Zsig = MatrixXd(3, 2 * n_aug_ + 1);
  
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    // extract values for better readibility
    double p_x = Xsig_pred(0,i);
    double p_y = Xsig_pred(1,i);
    double v  = Xsig_pred(2,i);
    double yaw = Xsig_pred(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;
	
    // measurement model
	Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //rho
    
	if (fabs(p_x) > 0.001) {
	Zsig(1,i) = atan2(p_y,p_x);                                 //phi
    }
	else{
	Zsig(1,i)=0.0;
	}
	
	if(Zsig(0,i)>0.001){
	Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //rho_dot
	}
	else{
	Zsig(2,i)=0.001;
	}	
  }
  

  //mean predicted measurement
  VectorXd z_pred = VectorXd(3);
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //innovation covariance matrix S
  MatrixXd S = MatrixXd(3,3);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(3,3);
  R <<    std_radr_*std_radr_, 0, 0,
          0, std_radphi_*std_radphi_, 0,
          0, 0,std_radrd_*std_radrd_;
  S = S + R; 
  
  //calculate cross correlation matrix
  MatrixXd Tc = MatrixXd(n_x_, 3);
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred.col(i) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z = meas_package.raw_measurements_;
  VectorXd z_diff = z - z_pred;

  //angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();
  
  // NIS calculation
  NIS_RADAR_ = z_diff.transpose() * S.inverse() * z_diff;
  NIS_LASER_ = -1.0; 

}


  

