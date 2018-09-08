# CarND-Controls-PID
Udacity Self-Driving Car Engineer Nanodegree Program: Term 2
[Master project repo](https://github.com/udacity/CarND-Kidnapped-Vehicle-Project)

This project implements a PID controller for a simulated autonomous vehicle, using the cross-track error (CTE). After tuning the three parameters for the controller, the car is able to drive autonomously around the track continously at speeds over 50 mph (maximum speed ~ 65 mph).

This project uses the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

## Project structure

The project consists of the following primary components:
* src/main.cpp - Responsible for interfacing with the simulator and reading (simulated) CTE data
* src/PID.cpp, .h - Responsible for implementing the PID controller and holding all values associated with it

## PID controller concepts

One approach to autonomously controlling the steering of a car is to steer in proportion to the cross-track error (CTE), which is how far the car is to the left or right of the center of the lane. By tuning a parameter known as Kp, this method steers back to the center of the lane either sharply or gradually. However, for all values of Kp, the track of the vehicle overshoots, and the vehicle oscillates back and forth across the lane center as it travels forward.

To mitigate this, it's possible to add a term to the steering value that's proportional to the derivative of CTE, i.e. how quickly CTE is changing. This mitigates fast turns and tends to reduce "wobbling". A final consideration is whether the vehicle has a constant "drift" in the steering, which occurs in real-world cars. With proportional and derivative-based steering only, this leads to a vehicle trajectory with a constant offset from the lane center. Adding a term to the steering value proportional to the integral of the CTE with time serves to prevent this.

The three terms are labeled "P" (proportional), "I" (integral), and "D" (differential), hence "PID" controller.

### Tuning PID parameters 

While implementing a PID controller in code is relatively easy, the primary challenge is tuning the parameters to find values that will result in as smooth a ride as possible. This is non-trivial, and there are many combinations of Kp, Ki and Kd that will accomplish this. 

While it's possible to implement auto-tuning of the parameters (such as using the "Twiddle" algorithm), this was difficult to implement in this project because the code interfacing with the simulator was confusing. Also, the simulator appears to be somewhat stochastic, meaning that the exact same PID parameters will result in slightly different trajectories and cumulative CTE run-to-run. 

Instead, I manually tuned the parameters, arriving at successful values after about an hour. One remarkable point about this is that they differ by several orders of magnitude. The largest is Kd, with Kp two orders of magnitude smaller, and Ki another two orders of magnitude smaller again. This is probably because the optimum controller has roughly equal contributions from these three terms, and the values of CTE, its derivative, and its integral vary by at least an order of magnitude (the integral is likely the largest, followed by the direct, instantaneous value, and finally the derivative as the smallest). 

### Speed

I increased the throtle from 0.3 to 0.6 (before tuning the PID parameters). The car completes most of the lap at or above 60 mph.

---

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

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

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

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

