# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

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

## Effects of the P, I, D components of the PID algorithm
The input to the PID controller is Cross Track Error (cte). This is the distance from the center of the vehicle
to the expected location on the path, which in this case is the center of the lane.

### Proportional (P) Gain
The proportional term is simply the cross track error times the proportional gain value P.

When tuning the controller I was surprised that the proportional term alone was not able to create a stable controller.
With just a P controller the controller either could not steer enough on turns to keep the car on the road, or would
become unstable after turns. In order to get a stable system that would at least be able to drive the entire track
I had to include derivative gain. I think this is because the target of the controller is constantly changing as the
vehicle is driving which the proportional gain cannot compensate for.

### Integral (I) Gain
The integral term is the sum of all the previous errors. So the integral term of the controller will be the sum of all
the previous cross track errors multiplied by the I gain.

I did not feel that the Integral portion of the controller added a lot of performance gain to the system. It had to
remain very small or else it would make the system unstable. I was able to get better performance by resetting the
Integral term whenever the cross track error changed directions

### Derivative (D) Gain
The derivative term is the change in the cross track error multiplied by the D gain.

The derivative term for this controller gave stability to the system 

## Tuning PID gains
I started tuning the PID gains by manual trial and error. I also created a twiddle algorithm in my PID controller to
adjust the gains automatically. My twiddle algorithm ran for a specified duration on the track and took the average
cross track error over that duration to decide whether the change in parameters was better or worse than the previous
parameters.

I found that minimizing the cross track error this way did not necessarily create the best steering performance. For
Example the car could steer very poorly on a turn but sense this is a small portion of the overall section it would have
a better average cross track error than parameters where the car smoothly turns with worse cross track error.

I mainly used manual trial and error to get the tuning parameters I desired. I used twiddle to see if
improvements could be made, but this took a long time to run and often created tuning parameters that were too high, 
causing the car to go slightly off the track on tight turns, and to hunt around the center on straight paths.