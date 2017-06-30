# Extended Kalman Filters

---

Extended Kalman Filter Project.

---

The goals / steps of this project are the following:

* Implement Lidar and Radar sensor fusion in C++
* Use standard Kalman filter for Lidar and Extended Kalman filter for Radar.
* Test implementation by tracking a moving object.
* Validate implementation by comparing predicted trajectory to provided ground truth trajectory.

---

# 1. Files

My project includes the following files:

- [<b>C++</b> - The source code](https://github.com/ArjaanBuijk/CarND-Extended-Kalman-Filter-Project/tree/master/src)
- [<b>writeup_report.md</b> - A summary of the project](https://github.com/ArjaanBuijk/CarND-Extended-Kalman-Filter-Project/blob/master/writeup_report.md)
- [<b>videoProject.mp4</b> - A video showing prediction (green) compared to sensor data (red=Lidar, blue=Radar)](https://github.com/ArjaanBuijk/CarND-Extended-Kalman-Filter-Project/blob/master/videoProject.mp4)

    ![track1](https://github.com/ArjaanBuijk/CarND-Extended-Kalman-Filter-Project/blob/master/videoProject.gif?raw=true)

---

# 2. helper functions

In tools.cpp helper functions are defined used in the radar measurement update:

| Helper Function   | Description |
|-------------------|-------------|
| Calculate_h		| To calculate the non-linear mapping function, known as the 'h function'.<br><br>This function transforms the object's location & velocity from a 2D cartesian coordinate system (px, py, vx, vy), into a 2D polar coordinate system (ro, theta, ro_dot). The 2D polar system is what the Radar sensor uses, and the measurement update step of the Extended Kalman filter is done in the 2D polar coordinate system: <br> <img src="https://github.com/ArjaanBuijk/CarND-Extended-Kalman-Filter-Project/blob/master/images/Radar_Measurement_Space.png?raw=true" alt="Radar_Measurement_Space" style="width: 400px;"/>|
| CalculateJacobian | To calculate the Jacobian Matrix of the 'h function'.<br><br>To maintain gaussian shapes of the covariances (=uncertainties) during the measurement update, a linear mapping function must be used. For this reason, the h function is linearized with a first order Taylor series expansion:<br>![Image](https://github.com/ArjaanBuijk/CarND-Extended-Kalman-Filter-Project/blob/master/images/MultiDimensionTaylorSeries.PNG?raw=true)<br>The Jacobian Matrix contains the first derivates of the h function:<br>    ![Image](https://github.com/ArjaanBuijk/CarND-Extended-Kalman-Filter-Project/blob/master/images/Jacobian.PNG?raw=true)|
| CalculateRMSE     | To calculate the Root Mean Squared Error between predicted and ground truth trajectories|

---

# 3. Implementation

The implementation is done in these files:

| File | Description |
|-------|-------------|
|main.cpp| Implements a web-server that receives sensor measurments. When it receives a measurment from the simulator, the following is done in this function:<br>- extract the data<br>- create a 'measurement package'<br>- call FusionEKF to process the measurement<br>- calculates RSME<br>- sends updated location and RSME back to simulator
|FusionEKF.cpp|Processes the measurements, as follows:<br>- initialize location (px,py) from first Lidar measurement<br>- initialize velocity (vx,vy) from second Lidar measurement<br>- does predict and update step for next Lidar and Radar measurements<br><br>The initialization of the object's state is using the first two Lidar measurements, and not a Radar measurement, because Lidar is more accurate in position detection.<br><br>The initial estimate of the velocity is calculated as (dx/dt, dy/dt) between the first two Lidar measurements.|
|kalman_filter.cpp|Implements the predict and measurement update steps, called from FusionEKF:<br>- Predict: predict new location assuming constant velocity.<br>- Update: applies Kalman Filter update step for Lidar<br>- UpdateEKF: applies Extended Kalman Filter update for Radar|

The implementation was done in straight-forward manner by using VectorXd and MatrixXd of the provided Eigen library. This keeps the implementation simple and easily readable since the code is nearly identical as the actual equations in Matrix notation. It not really optimal though for speed, since there are a lot of multiplications by zero that are not necessary. However, the code was not optimized for this.

During the update for the radar measurement (UpdateEKF), the state (ro, theta, ro_dot) is first updated by subtracting the h function from the predicted state. This can potentially lead to a value for theta that falls outside [-pi, pi]. A normalization is applied to ensure that theta falls within [-pi, pi], using:

y[1] = atan2(sin(angle), cos(angle));

---

# 4. Results

Three trajectory predictions were tested:

1. Using Lidar measurements only
2. Using Radar measurements only
3. Using both Lidar and Radar measurements (sensor fusion)

The result is summarized in this table:

|Method        |RMSE<br>px|RMSE<br>py|RMSE<br>vx|RMSE<br>vy|Trajectory Prediction|
|--------------|----------|----------|----------|----------|----------|
|Lidar only    |
|Radar only (*)|
|Lidar & Radar |

(*) In all cases, the first two Lidar measurements were used to initialize the estimate of location and velocity. After initialization was completed, the Radar only case ignored the Lidar measurements.

# 5. Summary

The end result can be summarized as follows:

- The classifier is able to detect both the white & black car very good in individual frames.
- The classifier does detect a fair amount of false positives, sometimes with high confidence.
- An elaborate heatmap thresholding over 14-17 frames was needed to eliminate the false positives while keeping the true positives.


Even though it works well for this video, there is definitely a lot of room for improvement. I feel there are way too many false positives and rely too heavily on an elaborate heatmap thresholding which is brittle. I do not expect the current logic to work well on other cases without having to do significant work in fine-tuning the classifier and the thresholding.

Concretely, to improve it, I would focus on:

- Optimize the classifier. 
- Use more or better training data, especially for non-vehicles.
