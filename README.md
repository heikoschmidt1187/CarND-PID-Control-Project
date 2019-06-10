# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

# Project Writeup

The following sections describe the particular Rubric points.

## Your code should compile.

The project code compiles through the use of cmake like described in the *Basic Build Instructions* in the Udacity section of this README below.

## The PID procedure follows what was taught in the lessons.

The basic filter implementation is based on the implementation taught in the lesson. An auto-tuner class has been added to be able to use twiddle to find the PID parameters.

## Describe the effect each of the P, I, D components had in your implementation.

The P component stands for the proportional part of the PID controller. The more the car is off the desired path, the bigger the cross track error (CTE) is. For the P component, the CTE is multiplied with the Kp constant. Bigger errors mean a harder steering in this case. Changing the Kp in the project leads to great direct visual results. Higher values lead to faster reactions, but also the car can nerver reach the ideal line, so it starts to meander around the target position.

The D componentent as differential part of the PID controller counteracts the P component's overshooting of the target line. If chosen properly, the car will smoothly go back to the desired target line. The higher the Kp constant, the higher the Kd needed to be. Value around Kd ~ 10 * Kp turned out to work not bad.

While P-D controllers tend to bias at some point, the I or integral part of the PID controller takes care to smooth back to the target value.

## Describe how the final hyperparameters were chosen.

I first started to manually tune the parameters step by step, starting with a P of 0.2, I and D 0. With this the car did leave the track. Next, I tuned D to make the car drive a full round - which was possible with a value of 2.3. After about two rounds the car left the track again, which has been compensated through an I value of 0.0003. I did play with higher speeds, but the car was instable quite fast. For the project I decided to keep a constant throttle of 0.3.

With the manual found values if (p=0.2 i=0.0003 d=2.3) I startet the Twiddle algorithm I implemented for around 800 cycles of 700 timesteps. First I set fixed distances and restartet the simulator automatically after each round, but in the end I modified the autotune algorithm to have a stepcount of 700 steps and drive continously around the track, except for the cases the car leaves the track or hanging at the bridge spart with a speed of 0. With this I was able to "train" the parameters on their own and stop and continue another day with setting the start p's and dp's according to the last values.

The Twiddle algorithm soon seem to hang in local minimums so there wasn't improvement in the overall simulator performance. I then did some steps back and startet the autotuning with lightly manual modified p-values.

In the end, the following parameters were chosen: (p=0.257847 i=0.00930742 d=4.34883).

## The vehicle must successfully drive a lap around the track.

I let the simulator drive the car around the track for more than four hours before submitting the project without any major incident.

## Possible improvements

While the performance should be ok for the project Rubric points, the car drives far from perfect. The following points may improve the handling further:

* Use a PID controller for the throttle, based either on a target speed or on the current CTE so it slows down the bigger the CTE is.
* Other functions to find the hyperparameters like SGD may be more efficient in finding optimal parameters, so it's worth a try to check them out.
* Maybe machine learning via CNNs can automate the auto-tuning even better.




# Original Udacity README

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

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

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
