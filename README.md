# Extended Kalman Filter Project
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

---

## Note

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

# <span style = "color: #2F766D"> Project submission </span>

## 1. Starting with the result:

The result is generated based on the given sample data `obj_pose-laser-radar-synthetic-input.txt`.

For data set 1:
<center>

<img src = "./pictures/data1.png" alt = "data set 1 result" width = "400px">

| `Data Set` | `RMSE X` | `RMSE Y` | `RMSE vx` | `RMSE vy` |
|------------|----------|----------|-----------|-----------|
| <center>`1`</center>| `0.0973` | `0.0855` | `0.4513`  | `0.4399`  |
</center>

For data set 2:

<center>
<img src = "./pictures/data2.png" alt = "data set 2 result" width = "400px">

| `Data Set` | `RMSE X` | `RMSE Y` | `RMSE vx` | `RMSE vy` |
|------------|----------|----------|-----------|-----------|
| <center>`2`</center>| `0.0726` | `0.0965` | `0.4219`  | `0.4937`  |
</center>

## 2. Sensor Fusion Workflow:

<!-- <center><img src = "./pictures/flow.png" alt = "Overview of process measurement" width = "600px"></center> -->

Every Lidar and radar measurement data are fed from the given txt file `obj_pose-laser-radar-synthetic-input.txt`, sent by the term 2 simulator through `uWebSocketIO`, which then received by `main.cpp`, and passed to `FusionEKF.ProcessMeasurement()`

If the input data are the first set of measurement, an intialization process will be conducted (line 78 of `FusionEKF.cpp`):
<pre><code>if (!is_initialized_ || ekf_.x_[0] == NULL)
</code></pre>

Depending on which measurement type, the initialization process is handled differently:

<pre><code>if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        /**
        Convert radar from polar to cartesian coordinates and initialize state.
        */
            cout << "EKF : First measurement RADAR" << endl;
            ekf_.Init(x_init, P_init, F_init, Hj_, R_radar_, Q_init);
			double rho = measurement_pack.raw_measurements_[0]; // range
  	  		double phi = measurement_pack.raw_measurements_[1]; // bearing
  	  		double rho_dot = measurement_pack.raw_measurements_[2]; // velocity of rho
  	  		// Coordinates convertion from polar to cartesian
  	  		px = rho * cos(phi);
      		if ( px < 0.0001 ) {
        		px = 0.0001;
      		}
  	  		py = rho * sin(phi);
      		if ( py < 0.0001 ) {
        		py = 0.0001;
      		}
  	  		double vx = rho_dot * cos(phi);
  	  		double vy = rho_dot * sin(phi);
      		ekf_.x_ << px, py, vx , vy;

            if(fabs(px) < 0.0001) {
                px = 1;
                ekf_.P_(0,0) = 1000;
            }

            if(fabs(py) < 0.0001) {
                py = 1;
                ekf_.P_(1,1) = 1000;
            }
        } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
        /**
        Initialize state.
        */
            cout << "EKF : First measurement LASER" << endl;
            ekf_.Init(x_init, P_init, F_init, H_laser_, R_laser_, Q_init);

            px = measurement_pack.raw_measurements_[0];
            py = measurement_pack.raw_measurements_[1];
      	    ekf_.x_ << px, py, 0, 0;
        }
</code></pre>