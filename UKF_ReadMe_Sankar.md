# Unscented Kalman Filter for sensor fusion in a self driving car
---

The goals of this project are as follows:

* Implement an Unscented Kalman Filter (UKF) to predict vehicle position and velocity by fusing LIDAR and RADAR sensor data
* The position estimates must be within 0.09 m accuracy and velocity estimates must be within 0.4 m/s (accuracy is quantified via Root Mean Square Estimates)

[//]: # (Image References)
[image1]: ./figs/CTRV.png
[image2]: ./figs/RMSE.png
[image3]: ./figs/SigmaPoints.png
[image4]: ./figs/Weights.png
[image5]: ./figs/MeasurementPrediction.png
[image6]: ./figs/UKFupdate1.png
[image7]: ./figs/NIS.png
[image8]: ./figs/ChiSquare.png

**Description of files**

* FusionUFK.cpp is the core file that contains the UKF algorithm - process model and measurement updates
* main.cpp calls the FusionUKF.cpp with incoming measurements and communicates with the simulator
* tools. cpp contains the RMSE calculation for easy access

Remaining files are used "as-is" to interface with simulator and doing background tasks. 

**Algorithm**
---

The motion model was captured using a Constant Turn Rate and Velocity Magnitude (CTRV) model. The model captures non-linearity in motion better than a constant velocity model. Five primary states were chosen as described below.

* States=		[Position_x,
		Position_y,
		Velocity,
		yaw
		yaw_rate]

![alt text][image1]
		
* Two sets of measurement data is available:

```sh
LIDAR measurements = [ position_x, position_y]
RADAR measurements = [rho, phi, rho_dot]
```	

* The Extended Kalman filter uses the Jacobian matrix to linearize non-linear functions. The UKF algorithm does not linearize the process or measurement model. The UKF takes representative points from a Gaussian distribution. These points are called "sigma" points. The first step is to create the sigma points as shown below. Lambda is a tuning parameter that distributes the sigma points from the mean state. 


![alt text][image3]

* Once the sigma points are generated, they go through the process model for the prediction step. The noise terms - linear acceleration and yaw acceleration are augmented to the state vector and corresponding sigma points generated. Once all the sigma points go through the prediction step via the CTRV model, the mean state and covariance of a gaussian that approximates all the points is computed. In order to compute the mean state and covaraince, a vector of weights is generated using the formulation shown below. 


![alt text][image4]


* Once the mean state and covariance is predicted, the next step is to perform measurement update

* LIDAR with its linear measurement model does not require the UKF measurement update equations. Therefore measurement update for LIDAR is done via linear kalman filter.

* The non-linear measurement model of the radar necessitates the UKF algorithm. The first step for the radar measurement update is to map the predicted sigma points into the measurement space. This is simply done via the radar measurement model as shown below. Similar to the mean state and covariance calculation in the prediction step, a mean predicted measurement and covariance is calculated. 

![alt text][image5]

* The next step is to perform the UKF update using equations shown below.

![alt text][image6]

* Normalized Innovation Squared (NIS) was calculated for both radar and laser measurements. The NIS term is a useful term to tune process noise parameters. The calculation is shown below:

![alt text][image7]

* The NIS values at each step is compared with the 5% and 95% term in a chi-square distribution. The table is shown below with degrees of freedom being the first column. 

![alt text][image8]

**Closure**

An Unscented Kalman Filter was implemented to fuse LIDAR and radar data in conjunction with a non-linear CTRV motion model to estimate position and velocities for a self driving car. The final RMSE values of the positions and velocity estimates of simulator testing are shown below. The blue and red dots represent the radar and LIDAR estimates. The green dots are the output of the UKF algorithm.

![alt text][image2]


















