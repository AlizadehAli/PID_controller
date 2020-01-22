# PID Controller Design Project
Self-Driving Car Engineer Nanodegree Program
---
## Overview
In this project a Proportional-Integral-Differential (PID) controller that is used to maneuver the vehicle around the lake city track in the simulator is desinged. Cross track error (CTE) and the velocity (mph) are provided by the simulator, whereupon the objective is to develop a PID controller in c++ that successfully drives the vehicle around the track. 

The goals/steps of this project are as follows:

* Implement a PID controller for steering angle 
* Optimize init parameters for each PID coefficient

## Dependencies

* cmake >= 3.5
* make >= 4.1(mac, linux), 3.81(Windows)
* gcc/g++ >= 5.4
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
* [Simulator](https://github.com/udacity/self-driving-car-sim/releases); Udacity's term 2 simulator

## Basic Build Instructions

1. Clone the repository
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`

## Project Instructions and [Rubric points](https://review.udacity.com/#!/rubrics/1972/view)

#### Compilation

The submitted code compiles without error with `cmake` and `make`. It generated the `pid` binary code that will be excutable with no error.

#### Implementation

The implementation steps in this project closely follows the steps taught in the lecture videos. Hyperparameter tuning was done heuristically to find the P, I and D control gains. The PID initialization and update are implemented in `./src/PID.cpp`.

#### Reflection

##### Effect of the P, I, D component of the PID algorithm

P or "proportional" component has the highest effect on the response of the controller. t drives the car by steering it proportional (opposite) to the car's CTE (distance to the center line). the magnitude of the steering is directly proportional to distance of the car to the center line. A controller solely designed with P component, P controller, cannot converge to the reference trajectory in the project which is shown in [this video](./src/P_controller.mp4).

D, or "differential", component counteracts the P component's tendency to oscilate acound the reference line. A PD controller incorporating properly tuned D component drives the car smoothly to the center line. The video of designed PD controller performance is shown [here](./src/PD_controller.mp4)

I, or "integral", component counteracts a bias in the CTE which intrinsically prevents the car to converge to the reference trajectory. This bias can stem from the steering drift, or any other undesirable forces/noises. A controller incorporating all P, I, D components drives the car smoother and more efficient. A video my PID controller performance against UDacity simulator is shown [here](./videos/PID_controller.mp4).

##### Hyperparameters (P, I, D coefficients) optimization

in order to capture the effects of the P, I, and D controler parameters, I designed three controllers (P controller, PD contrller and PID controller). P, I, D coefficients are fixed for all controllers to provide a fair baseline for their performance comparison. the following cofficient values are achieved by heiristics which drive the car smooth and efficient.

| **Parameter** | **value** | **Definition** |
|:---------:|:---------:|:---------:|
| P | 0.05 | Proportinal coefficient |
| I | 0.004 | Integral coefficient |
| D | 3.0 | Differential coefficient |

#### Simulation

Three different controllers are designed and investigated in this project; P, PD, and PID controllers.
The initial gains for the controllers are defined as follows:

| **Controller** | **(K<sub>P</sub>, K<sub>I</sub>, K<sub>D</sub>)** |
|:---------:|:---------:|
| P | (0.05, 0.0, 0.0) | 
| PD | (0.05, 0.0, 3.0) | 
| PID | (0.05, 0.004, 0.0) | 

Amogst the above controllers, PD and PID contrllers meet the simulation requirement. They both drive the car in the drivable road while the PID controller perform better in terms of smoothness of motion and CTE.

The videos of [PD](./videos) and [PID](./videos) controllers are in videos folder.
 
<video src="src/PD_controller.mp4" width="320" height="200" controls preload></video>


