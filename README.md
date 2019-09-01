[//]: # (Image References)
[image1]: twiddle_after14Iter_failed.gif  "twiddle"
[image2]: manual_tune_pid.gif "Manual Tuned PID results"


# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---
## Implementations
1. Rewrite python code from classroom to C++.:) 
   * `Init()` initialize P, I, D parameters (PID.cpp line 14-25)
   * `UpdateError()`, based on PID error calculation formula, get error for each parameters(PID.cpp line 27-35)
   * `GetSteerValue()`, output steer values (PID.cpp line 46-58)
2. Manual PID parameters turning 
   * Adjust values by modify P, all others to 0; then P,I; then PID.
   * it looks like P= .2, I = 1e-4, D = 3.0 let the car finish the full lap
3. Using Twiddle Algorithm to Tune PID Paramters
   * I imitated the python code algorithm and rewrite it in C++
   * Added a few conditions to make it better works in the simulation enviroment
   *  if cte > 5 , which means the car is out of lane, should rerun simulation
   *  if time steps (no of h messages) > 8000 ( about one lap), restart simulation for another iteraton
   *  However, it seems my implementation doesn't work well. first of all, it takes much longer time to run more iterations in simulator.
   * Besides, PID values are not moving in the right directions and  stay at a range then do not change anymore.
   * Due to limited time, I am not be able to figure out the reason and will revisit this after finishing the final project.
  
 Below is a short GIF shows the animation after using manual tuned PID parameters.
 
 ![Manual Tuned PID results][image2]
 
 Below is a short GIF shows the animation after using twiddle for 14 iterations(Failed).
 
 ![Twiddle][image1]
## Project Dependencies

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

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

