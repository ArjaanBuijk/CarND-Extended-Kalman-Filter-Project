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
| Calculate_h		| To calculate the non-linear mapping function, known as the 'h function'.<br><br>The measurement update step of the Extended Kalman filter is done in a 2D polar coordinate system because that is what Radar sensors use. The 'h function' transforms the object's location & velocity from a 2D cartesian coordinate system into a 2D polar coordinate system: <br> <div style="text-align:center"><img src="https://github.com/ArjaanBuijk/CarND-Extended-Kalman-Filter-Project/blob/master/images/Radar_Measurement_Space.png?raw=true" style="width: 300px;"/></div>|
| CalculateJacobian | To calculate the Jacobian Matrix of the 'h function'.<br><br>To maintain gaussian shapes of the covariances (=uncertainties) during a measurement update step, a linear mapping function must be used. For this reason, the non-linear h function is linearized with a first order Taylor series expansion:<br> <div style="text-align:center"><img src="https://github.com/ArjaanBuijk/CarND-Extended-Kalman-Filter-Project/blob/master/images/MultiDimensionTaylorSeries.PNG?raw=true" style="width: 400px;"/></div><br>The Jacobian Matrix contains the first derivates of the h function:<br> <div style="text-align:center"><img src="https://github.com/ArjaanBuijk/CarND-Extended-Kalman-Filter-Project/blob/master/images/Jacobian.PNG?raw=true" style="width: 300px;"/></div>|
| CalculateRMSE     | To calculate the Root Mean Squared Error between predicted and ground truth trajectories|

---

# 3. Implementation

The implementation is done in these files:

| File | Description |
|-------|-------------|
|main.cpp| Implements a web-server that receives sensor measurments. When it receives a measurment from the simulator, the following is done in this function:<br>- extract the data<br>- create a 'measurement package'<br>- call FusionEKF to process the measurement<br>- calculates RSME<br>- sends updated location and RSME back to simulator
|FusionEKF.cpp|Processes the measurements, as follows:<br>- initialize location (px,py) from first Lidar measurement<br>- initialize velocity (vx,vy) from second Lidar measurement<br>- does predict and update step for next Lidar and Radar measurements<br>- returns prediction of new location and velocity<br><br>The initialization of the object's state is using the first two Lidar measurements, and not a Radar measurement, because Lidar is more accurate in position detection. Also, the velocity measurement of a Radar sensor is not very accurate and not suitable to initialize the velocity of the object.<br><br>The initial estimate of the velocity is calculated as (dx/dt, dy/dt) between the first two Lidar measurements.|
|kalman_filter.cpp|Implements the predict and update steps in these functions:<br>- Predict: predict new location assuming constant velocity.<br>- Update: applies Kalman Filter update step for Lidar<br>- UpdateEKF: applies Extended Kalman Filter update for Radar|

The implementation was done in a straight-forward manner by using VectorXd and MatrixXd from the provided Eigen library. This keeps the implementation simple and easily readable since the code is nearly identical as the actual equations in Matrix notation. It not really optimal though for speed, since there are a lot of multiplications by zero that are not necessary.

During the update for the radar measurement (UpdateEKF), the state (ro, theta, ro_dot) is first updated by subtracting the h function from the predicted state. This can potentially lead to a value for theta that falls outside [-pi, pi]. A normalization is applied to ensure that theta falls within [-pi, pi], using:

    y[1] = atan2(sin(angle), cos(angle));  // Normalize angle to value between [-pi, pi]

---

# 4. Results

Three trajectory predictions were calculated:

1. Using Lidar measurements only
2. Using Radar measurements only
3. Using both Lidar and Radar measurements (sensor fusion)

The result is summarized in this table:

|Method        |RMSE<br>px|RMSE<br>py|RMSE<br>vx|RMSE<br>vy|Trajectory Prediction|
|--------------|----------|----------|----------|----------|----------|
|Lidar only    |0.1839|0.1544|0.5931|0.5833|<div style="text-align:center"><img src="https://github.com/ArjaanBuijk/CarND-Extended-Kalman-Filter-Project/blob/master/images/1.RMSE-Lidar-Only.PNG?raw=true" style="width: 300px;"/>|
|Radar only (*)|0.2327|0.3348|0.5607|0.7178|<div style="text-align:center"><img src="https://github.com/ArjaanBuijk/CarND-Extended-Kalman-Filter-Project/blob/master/images/2.RMSE-Radar-Only.PNG?raw=true" style="width: 300px;"/>|
|Lidar & Radar |0.0988|0.0850|0.4328|0.4751|<div style="text-align:center"><img src="https://github.com/ArjaanBuijk/CarND-Extended-Kalman-Filter-Project/blob/master/images/3.RMSE-Lidar-and-Radar.PNG?raw=true" style="width: 300px;"/>|

(*) In all cases, the first two Lidar measurements were used to initialize the estimate of location and velocity. After initialization was completed, the Radar only case ignored the rest of the Lidar measurements.

# 5. Summary

The result can be summarized as follows:

- When using Lidar only, the result is more accurate than using Radar only.
- Even though Radar-only is giving a less accurate result than Lidar-only, when fusing the Radar sensor measurements with the Lidar sensor measurements, the prediction becomes more accurate.
- Only with sensor fusion is the RMSE [0.0988, 0.0850, 0.4328, 0.4751] below the required target [0.11, 0.11, 0.52, 0.52].

Even though it works well for this example and in the simulator environment, there is definitely a lot of room for improvement. Concretely, I would focus on improving the following C++ implementation issues:

1. Refactoring the code:
   - The state variables are public. It would be better to encapsulate them into private variables.
   - Duplicate code is used in Update() and UpdateEKF().
2. Speed up:
   - Instead of using matrix multiplacations, it will be advantageous to eliminate all the multiplications by 0.
3. Error handling:
   - The code does not nicely handle errors. It just quits or core dumps. This would be unacceptable for production code, and a good error handling must be implemented.

