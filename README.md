# CarND - PID Controller

Udacity Self-Driving Car Engineer - Project8: PID Controller


This is the project repo for the PID project of the Udacity Self-Driving Car Nanodegree. Here are some outputs of the project:

![output1](./output/output1.gif)
![output2](./output/output2.gif)
![output3](./output/output3.gif)


# PID Controller

A PID controller (Proportional Integral Derivative controller) is a control loop mechanism employing feedback that continuously calculates an error value as the difference between a desired setpoint (SP) and a measured process variable (PV) and applies a correction based on proportional, integral, and derivative terms (denoted P, I, and D respectively). The contol function is:

![PID](./images/pid.svg)

where Kp, Ki and Kd are the coefficients for the proportional, integral, and derivative terms respectively.

The following describes the effect each of the P, I, D components had on the current steering controller implementation. In this case, the measured error is the cross track error (cte), which is the distance between the car position and the desired position the car should be driving through:

**Proportional component (P):** 
This component set the steer in a value proportional to the cte. The bigger the value of the cte, the greater the value of the P component. Keeping Ki and Kd at 0 and choosing a suitable value of Kp, the vehicle is able to drive following the road. However, this simple control usually has overshooting of the setpoint and that is shown in the oscillations that the vehicle has when driving on the road. Even more, if the value of Kp is very large the vehicle tends to lose control and roll off to the side of the road. On the other hand, if the value of Kp is small, the vehicle does not turn well enough. 

**Derivative component (D):**
This component is proportional to the difference between the current error and the error in the previous instant (Δcte). It eliminates the overshooting caused by P component and the value converges faster to the desired setpoint. This is shown in the vehicle's path as a decrease in oscillations, tracing a smoother path.

**Integral component (I):** 
This component is proportional to the cumulative sum of all previous cte values (the integral of cte). It eliminates the error known as systematic bias (for example steering drift).

### Implementation

The PID controller is implemented in `PID` class. The function `GetValue` updates the error and returns the new value of the controlled variable. PID initialization takes as parameter the maximum and minimum output value. If the controller output exceeds the limits, the control saturates at that limit.

Two PID controller were implemented in `main.cpp` to control the steering and the speed. The parameters Kp, Ki and Kd were manually selected as follows:
- Set the throttle value in 0.25 and tune steering PID parameters in.
- First, set Ki and Kd to 0 and find the Kp value that let the vehicle navigate (probably with oscillations).
- Keeping Ki in 0, find the value of Kd that eliminates (or reduces) the oscillations.
- Adjust the Ki value to eliminate the drift error (systematic bias).
- Repeat the process with speed PID parameters.


## Future improvements

The twiddle algorithm (or another) could be implemented in order to get the optimal parameters for PID controller. 

# Installation

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
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.
