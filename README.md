# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## The Model

Global Kinematic model embodies the the dynamics of a vehicle in a model that emulates a vehicle controls dynamics as closely as possible. This model ultimately calculates the state of a vehicle at a given time based on various forces that are modeled to be acting on the vehicle.  A state is represented by its x & y position in its local coordinate system, the steering angle and velocity.  The vehicle state in the near future time is predicted based on the current state and previous historic values.  Similarly, the actuator values are also calculated using the predictions of future state with this model.  Actuators are the steering angle and the acceleration that needs to be sent to the vehicle controllers to move the vehicle to the next state.  In this project, the model is developed in MPC.cpp file.  The operator() function of FG_eval class embodies the global kinematic model and thereby calculates the state and actuator values at t+1.

## N & dt

This model predits the future state of a vehicle for the next time period mentioned.  Even though we need the model to predict the state values continuously, it operates as a micro batches of predictions for a small time horizon of few seconds based on the environment & vehicle sensor inputs.  In each cycle, the model is executed at regular internal of 'dt' seconds for N times.  For example, in order to predict the state for the next 5 seconds, the dt can be set as 0.5 seconds and N can be 10.  The model uses these values in iterate and produces a prediction.  Having a big N or dt will make the model work faster as number of cycles to be done are lesser.  Similarly, having smaller N will make the model to perform much slower. 

I tried the model with N values ranging from 5 to 25, where the speed of the vehicle in simulator is varying significantly.  The optimum value is coming in the range of 10 to 15.  Similarly, dt value tried between 0.1 seconds to 0.05 seconds, the 0.05 seconds gave a more accurate results.

## Polynomial Fitting and MPC Preprocessing

In the simulator localization, the vehicle state data is given in global coordinates as if it is coming from a Map.  However, our works in the local coordinate syste (or vehicle coordinate system).  In pre-processing step, the conversion of coordinates to local system is important and is done as the first step in main.cpp file.  (lines 106 to 111).  

After the preprocessing, the coefficients are calculated based on the fitting the polynominals (using polyfit function provided).  And the same is evaluated using polyevals function.

## MPC Latency

Often times, in real life scenario, there will be a few milli seconds delay between the time the command is sent from model and the same got received by CAN Bus for execution, it is called as latency.  This latency can affect the performance of our vehicle.  For example, if the model predicted the state of the vehicle for next 0.5 seconds, if there is a latency of 0.1 second, then the vehicle would have moved and the prediction will be put to execution at 0.6th second instead of 0.5th second.  By that time, the environment could have changed to some extent and model's out doesn't appear to be accurate.  So, if there is a latency in the vehicle physicals, it is important to measure that latency and incorporate it in the model itself.  In this project, it is indicated that the simulator is having a latency of 100 milli seconds, hence that latency is also incorporated before the actuators values are arrived and sent to vehicle.  The code is in main.cpp file at line number 135.

---

## Dependencies

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
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

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
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
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
