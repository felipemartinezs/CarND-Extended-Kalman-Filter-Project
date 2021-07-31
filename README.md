
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)



# Extended Kalman Filter Project 
Self-Driving Car Engineer Nanodegree Program

This project consists of implementing the extended Kalman filter in C ++. You should provide simulated radar and lidar measurements that detect a bicycle moving around your vehicle. It will use a Kalman filter, lidar measurements, and radar measurements to track the bike's position and speed.

Will identify Lidar measurements with red circles and radar measurements with blue circles with an arrow pointing in the direction of the observed angle. The estimation markers are green triangles.


<br>

# Overview of a Kalman Filter

A Kalman filter is an optimal estimator  - i.e., it infers parameters of interest from indirect,  inaccurate, and uncertain observations.    It is recursive so that new measurements can be processed as they arrive.    (cf batch processing where all data must be present).

If all noise is  Gaussian,  the  Kalman filter minimizes the mean square error of the estimated parameters.


The three main steps for programming a Kalman filter:

- initializing Kalman filter variables
- predicting where our object is going to be after a time step $\Delta{t}$
- updating where our object is based on sensor measurements





![Arquitecture Kalman Filter](screenshot-from-2017-02-27-19-56-58.png)


## Dependencies & environment

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [Eigen library](src/Eigen)


## Project files

(Note: the hyperlinks **only** works if you are on the homepage of this GitHub reop,
and if you are viewing it in "github.io" you can be redirected by clicking the **View the Project on GitHub** on the top)

* [CMakeLists.txt](CMakeLists.txt) is the cmake file.

* [data](data) folder contains test lidar and radar measurements.

* [Docs](Docs) folder contains docments which describe the data.

* [src](src) folder contains the source code.


## Summary of What Needs to Be Done

* In **tools.cpp**, fill in the functions that calculate root mean squared error (RMSE) and the Jacobian matrix.

* Fill in the code in **FusionEKF.cpp**. You'll need to initialize the Kalman Filter, prepare the Q and F matrices for the prediction step, and call the radar and lidar update functions.

* In **kalman_filter.cpp**, fill out the Predict(), Update(), and UpdateEKF() functions.


## Code Style

* [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).


## Compile and Run on Unity Simulator the code

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it by either of the following commands: 
   * `./ExtendedKF  ../data/obj_pose-laser-radar-synthetic-input.txt ./output.txt`
   * `./ExtendedKF  ../data/sample-laser-radar-measurement-data-1.txt ./output.txt`


## Results

The objective in this project is to reach values of the output coordinates px, py, vx, v and they must have an RMSE <= [.11, .11, 0.52, 0.52] for Dataset 1

#### RMSE values

The folowing table lists the results of both datasets:

| RMSE | Dataset 1 | Dataset 2 |
|------|-----------|-----------|
| P x  |  0.0973   |  0.0726   |
| P y  |  0.0855   |  0.0965   |
| V x  |  0.4513   |  0.4216   |
| V y  |  1.4399   |  0.4932   |


<img src="./Dataset 1.JPG " alt="dataset1" width="600"/>

<img src="./Dataset 2.JPG " alt="dataset2" width="600"/>

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)



