# C++ Extended Kalman Filter Project 
Self-Driving Car Engineer Nanodegree Program - Udacity - Extended Kalman Filter Implementation

# Overview
This project consists of implementing an [Extended Kalman Filter](https://en.wikipedia.org/wiki/Extended_Kalman_filter) with C++. A simulator provided by Udacity ([can be downloaded here](https://github.com/udacity/self-driving-car-sim/releases)) generates noisy RADAR and LIDAR measurements of the position and velocity of an object, and the Extended Kalman Filter[EKF] must fusion those measurements to predict the position of the object. The communication between the simulator and the EKF is done using [WebSocket](https://en.wikipedia.org/wiki/WebSocket) using the [uWebSockets](https://github.com/uNetworking/uWebSockets) implementation on the EKF side.
To get this project started, Udacity provides a seed project that could be found (here)(https://github.com/udacity/CarND-Extended-Kalman-Filter-Project).


# Prerequisites

The project has the following dependencies (from Udacity's seed project):

- cmake >= 3.5
- make >= 4.1
- gcc/g++ >= 5.4
- Udacity's simulator.

In order to install the necessary libraries, use the [install-mac.sh](./install-mac.sh).

# Compiling and executing the project

These are the suggested steps:

- Clone the repo and cd to it on a Terminal.
- Create the build directory: `mkdir build`
- `cd build`
- `cmake ..`
- `./ExtendedKF` (to run the EKF implementation code)

---

## Running the Filter

From the build directory, execute `./ExtendedKF`. The output should be:

```
Listening to port 4567
Connected!!!
```

As you can see, the simulator connect to it right away.

The following is an image of the simulator:

![Simulator: Dataset2](images/Dataset2.png)

The simulator provides two datasets. The difference between them are:

- The direction the car (the object) is moving.
- The order the first measurement is sent to the EKF. On dataset 1, the LIDAR measurement is sent first. On the dataset 2, the RADAR measurement is sent first.

## Accuracy

px, py, vx, vy output coordinates must have an RMSE <= [.11, .11, 0.52, 0.52] when using the file: "obj_pose-laser-radar-synthetic-input.txt".


## Following the Correct Algorithm

Udacity instructions :

Your Sensor Fusion algorithm follows the general processing flow as taught in the preceding lessons.
The Kalman filter implementation can be found src/kalman_filter.cpp and it is used to predict at src/FusionEKF.cpp line 147 and to update line 159 to 169.

Your Kalman Filter algorithm handles the first measurements appropriately.
The first measurement is handled at src/FusionEKF.cpp from line 61 to line 107.

Your Kalman Filter algorithm first predicts then updates.
The predict operation could be found at src/FusionEKF.cpp line 147 and the update operation from line 159 to 169 of the same file.

Your Kalman Filter can handle radar and lidar measurements.
Different type of measurements are handled in two places in src/FusionEKF.cpp:

For the first measurement from line 61 to line 107.
For the update part from line 159 to 169.
Code Efficiency
Your algorithm should avoid unnecessary calculations.
An example of this calculation optimization is when the Q matrix is calculated src/FusionEKF.cpp line 135 to line 144.

## References

- [ Extended Kalman Filter - Equations and References for this example](https://d17h27t6h515a5.cloudfront.net/topher/2017/February/58b461d5_sensor-fusion-ekf-reference/sensor-fusion-ekf-reference.pdf)

 - [ Extended Kalman Filter - How does it work ?](http://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/)

