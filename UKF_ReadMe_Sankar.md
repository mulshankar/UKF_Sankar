# Unscented Kalman Filter for sensor fusion in a self driving car
---

The goals of this project are as follows:

* Implement an Unscented Kalman Filter to predict a bicycle position and velocity by fusing LIDAR and RADAR sensor data
* The position estimates must be within 0.09 m accuracy and velocity estimates must be within 0.3 m/s (accuracy is quantified via Root Mean Square Estimates)

[//]: # (Image References)
[image1]: ./figs/RadarData.jpg
[image2]: ./figs/OnlyLaser.JPG
[image3]: ./figs/OnlyRadar.JPG
[image4]: ./figs/BothSensors.JPG
[image5]: ./figs/Dataset2.JPG

**Description of files**

* FusionUFK.cpp is the core file that contains the UKF algorithm
* main.cpp calls the FusionUKF.cpp with the measurements and communicates with the simulator
* tools. cpp contains the RMSE calculation for easy access

Remaining files are used "as-is" to interface with simulator and doing background tasks. 

**Algorithm**
---

The motion model was captured using a Constant Turn Rate and Velocity magnitude (CTRV) model. The model captures non-linearity in motion better than a constant velocity model. Five primary states were chosen as described below.

* States=		[Position_x,
		Position_y,
		Velocity,
		yaw
		yaw_rate]

* Two sets of measurement data is available:

LIDAR measurements = [ position_x, position_y]
RADAR measurements = [rho, phi, rho_dot];
		
* The extended Kalman filter uses the Jacobian matrix to linearize non-linear functions. The unscented Kalman filter, on the other hand, does not need to linearize non-linear functions. Instead, the Unscented Kalman filter takes representative points from a Gaussian distribution.

```sh
void KalmanFilter::Predict() {
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}
```
* Call measurement update function based on LIDAR or RADAR data. The key difference is in the measurement model between the two sensors. As shown above, LIDAR provides a 3-D point cloud that can be parsed into a (x,y) position. In this case the measurement model,  

```sh
	Z=H*x;
	H_laser_<< 1,0,0,0,
			0,1,0,0;
```

But in the case of radar, measurement model is non-linear since we need to convert the data from polar to cartesian coordinates

```sh
  	float rho=sqrt(x_(0)*x_(0)+x_(1)*x_(1));
	
	float phi=0;// check for division by zero
	if (fabs(x_(0)) > 0.001) {
		phi=atan2(x_(1),x_(0));
	}
	
	float rho_dot=0.001; // check for division by zero
	if (rho > 0.001) {
		rho_dot=(x_(0)*x_(2)+x_(1)*x_(3))/rho;
	}
	
	VectorXd z_pred=VectorXd(3);
	z_pred << rho,phi,rho_dot;
	
	VectorXd y = z - z_pred;
	
	// normalize to keep phi between pi and -pi
	if (y(1)<-PI){
	y(1)=y(1)+2*PI;
	}
	else if (y(1)>PI){
	y(1)=y(1)-2*PI;
	}	
```
As shown above, it becomes necessary to check for division by zero and also ensure that all angles are normalized between +pi and -pi with 0 being center of viewing angle. It is illustrated in the figure below. The angle marked by the star can be represented either as + pi/2 or -3/2 * pi. 

![alt text][image1]

* The non-linear measurement model of the radar necessitates calculation of the Jacobian to be used in the kalman filter equation. The jacobian calculation is embedded in the tools.cpp function

* Once the Jacobian is calculated, the measurement update equations remain identical between the LIDAR and RADAR data

```sh
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
```

**Testing**

As the first step, only LIDAR measurement was used to predict the states. The results of simulator testing is shown in image below. While the performance is OK, the RMSE values are above the target requirement. 

![alt text][image2]

The next step was to test only with RADAR data to quantify performance. Simulator testing results is shown below. The RADAR measurement is inherently noisy compared to LIDAR. The state estimates are worse than the pure LIDAR case.

![alt text][image3]

The next gradual step was to use both LIDAR and RADAR data. Simulator testing shows that the state estimation results are well within the project requirement. The covariance matrix P also indicates high confidence on the prediction. 

![alt text][image4]

**Closure**

An Extended Kalman Filter was implemented in C++ to estimate position and velocities for a self driving car application. Results of simulator testing are shown by fusing RADAR and LIDAR data. Algorithm works great on dataset 1 but there is scope for some improvement to generalize. Algorithm performance on dataset 2 is shown below. 

![alt text][image5]





















