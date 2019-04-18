# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Goals
In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

### Result
Please read the video 'test_video1.mp4'.

The car drives according to the speed limit In order to achieve this, the algorithm discards trajectories that are going to exceed the speed limit in the future, so it picks a trajectory with less acceleration or overall speed.

Max Acceleration and Jerk are not Exceeded The acceleration increments implemented by the behavior model were designed carefully so that neither the acceleration nor the jerk are exceeded of 10 m/s^2 and 10 m/s^3 respectively.

Car does not have collisions. The 'prediction' function predicts the surrounding cars for both front and rear in the ego/left/right lanes.

The car stays in its lane, except for the time between changing lanes.

The car is able to change lanes as you can see in the video that the ego vehicle changes lanes very well when there is a vehicle in front of him going slower and there is no traffic in the next lane

### Implementation
The project consists of the following main components:
- Prediction 
It contains the predictions for the surrounding obstacles, located in vehicle.cpp.
The inputs of this function are the sensor fusion for surroundings, the localization data of the ego vehicle and the previous path. The outpus are the ID, position (Fernet coordinates), and flags which indicate if the obstacles exist or not.
It scan the inputs from the sensor fusion, trying to find the closest surrounding vehicles for front and rear in all lanes.

- Behavior Planning
It contains the finite state machine including 4 states: Lane Keep, Prepare Lane Change Left, Prepare Lane Change Right, Change Lane, located in vehicle.cpp.
The inputs of this function are the current state in the finite state machine, current lane where the ego car is. The outpus are the next state and the lane where the ego car should be.
When the speed of the ego car is larger than the front car, it will try to lower down the speed to avoid collision. When the speed is below the reference speed (around 50 mph) and there is no detected front car, it will try to accelerate in order to reach the max speed limit.
Some timers (counters) have been set up in order to control the time staying in the PLCL/PLCR states.

- Trajectory Generation
It contains reading waypoint map data, coordinate transformation, generate trajectory and smooth trajectory, located in trajectory_generator.cpp.
The inputs of this function are the current lane from the finite state machine and the previous path. The outpus is the generated trajectory.
It uses the current and previous ego car position or the left-over waypoints from the previous path returned by the simulator. It also uses 3 more control points that are cp_inc ahead of the last path waypoint. These 5 points determine the spline input. It then use the spline function to smooth out the trajectory in order to obey the acceleration and jert constraints.

The simulator sends sensor fusion data of the ego vehicle and the surrounding environment. A series of waypoints are also provided which allow as to create a global map of the road when the program started.

Each waypoint in the list contains [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop. 

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 


## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

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


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
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

