# PID control Project

## Introduction:
Main objective of this project was to design a PID controller & tune the gains so that car follows the track in the provided simulator. The simulator provides CTE (cross track error = current car positition - set point), speed, steering angle data as inputs. 

### Effect of Proportional / Derivative & Integral gains..!!

The proportional gain (P) - 'Kp' in the code has an apparent affect on the car in the simulator. Steering angle adjustment scales linearly with the gain and system or the car reaches marginal stability - hence oscillating heavily. 

Differential gain (kd) reduces the oscillations or overshoot as it exerts control action looking at the rate of change of error or cte in our case. The greater the cte error change more is the affect of Kd.

Integral component tries to reduce the steady state error to zero. The bias in CTE which restricts the car from reaching the set point or the middle of the road is compensated using Integral component.

### Describe how the final hyperparameters were chosen:
I started off with manual tuning.
First with a Kp = 0.1, Kd & ki = 0. The car in the simulator oscillates heavily about the center of the the track as CTE error starts to increase. 
Next step was tuning Kd to eliminate overshoot and oscillations - using kd = 3.0 worked best. Just using PD controller the car could easily navigate on the track without oscillations and overshoot. 
Just to make sure the car follows middle of the track and hence to reduce CTE during turns a small Ki gain of 0.0003 was applied. 
Also the PID controller gains can be tuned using optimization techniques like twiddling, SGD. 

Apart from PID controller that was implemented to compensate CTE and adjust steering angle, a throttle controller was implemented. Car needs to slow down and acceralate as the steering angle input is large and respectively small. Hence the input to throttle controller is (1 - absolute(steering angle)) and I used a gain and offset to scale the throttle input.

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) 

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

