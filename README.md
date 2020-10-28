# **Path Planning**
 
##  Safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit

### Using the process of prediction, behavior, and trajectory
---

**Path Planning Project**

The goals / steps of this project are the following:
* Implement path planning algorithms to drive a car around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit
* The car's localization and sensor fusion data is provided, there is also a sparse map list of waypoints around the highway
* The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too
* The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another
* The car should be able to make one complete loop around the 6946m highway
* Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3

[//]: # (Image References)

[image1]: ./Images_forReadMe/highwaylefttoright1.gif "highwaylefttoright1"
[image2]: ./Images_forReadMe/highwaylefttoright2.gif "highwaylefttoright2"
[image3]: ./Images_forReadMe/200mph.gif "200mph"
[image4]: ./Images_forReadMe/highwaylefttoright1.mp4 "highwaylefttoright1"
[image5]: ./Images_forReadMe/highwaylefttoright2.mp4 "highwaylefttoright2"
[image6]: ./Images_forReadMe/highway.mp4 "highway"

---

### README

- A path planning algorithm was implented to navigate around a virtual highway which can be found in the [src folder](./src)

- Below is the result for the path planning algorithm on two different examples:

| Example 1 | Example 2 |
| ------------- | ------------- |
| ![alt text][image1]| ![alt text][image2] |
| [Full Example 1 Video](./Images_forReadMe/highwaylefttoright1.mp4) | [Full Example 2 Video](./Images_forReadMe/highwaylefttoright2.mp4) |
| [Full video for 15 miles](./Images_forReadMe/highway.mp4)|

- The car is able to drive at least 15 miles without collisions, could be further but simulator was stopped.

- The car obeyed the 50mph speed limit as no speed limit red message was seen.

- Max acceleration and jerk red message were not seen.

- The car sucessfuly stays in its lane as it is programmed to stay in the middle lane for most of the time but it changes lane due to traffic.

- The car change lanes when the there is a slow car in front of it, and it is safe to change lanes (no other cars around) or when it is safe to return the center lane.

### 1. Basic Build Instructions
* The project could be executed directly using `./build/path_planning`

```
> cd build
> ./path_planning
Listening to port 4567
```

Now the path planner is running and listening on port 4567 for messages from the simulator.

* No changes were made in the cmake configuration. A new file was added [src/spline.h](./scr/spline.h). It is the [Cubic Spline interpolation implementation](http://kluge.in-chemnitz.de/opensource/spline/): a single .h file you can use splines instead of polynomials. It was a great suggestion from the classroom QA video.

### 2. Path Planning Algorithm

* The path planning algorithms start at [src/main.cpp](./src/main.cpp#L33) line 33 to 122.
* The code was divided into three main functions: prediction, behavior, and trajectory from line 123 to line 301.

#### Prediction [line 123 to line 163](./src/main.cpp#L123)
This part of the code deal with the telemetry and sensor fusion data. It makes the environment logical to its system. In the case, we want to know three aspects of it:

- Is there a car in front of us blocking the traffic.
- Is there a car to the right of us making a lane change not safe.
- Is there a car to the left of us making a lane change not safe.

These questions are answered by calculating the lane each other car is and the position it will be at the end of the last plan trajectory. A car is considered "dangerous" when its distance to our car is less than 30 meters in front or behind us.

#### Behavior [line 164 to line 195](./src/main.cpp#L164)
This part determines the possible actions:
  - If theres a car in front, be prepare to switch lane
  - Speed up or slow down?

Based on the prediction of the situation we are in, this code increases the speed, decrease speed, or make a lane change when it is safe. Instead of increasing the speed at this part of the code, a `speed_diff` is created to be used for speed changes when generating the trajectory in the last part of the code. This approach makes the car more responsive acting faster to changing situations like a car in front of it trying to apply breaks to cause a collision.

#### Trajectory [line 197 to line 301](./src/main.cpp#L197)
This code does the calculation of the trajectory based on the speed and lane output from the behavior, car coordinates and past path points.

First, the last two points of the previous trajectory (or the car position if there are no previous trajectory) are used in conjunction three points at a far distance to initialize the spline calculation. To make the work less complicated to the spline calculation based on those points, the coordinates are transformed (shift and rotation) to local car coordinates.

In order to ensure more continuity on the trajectory (in addition to adding the last two point of the pass trajectory to the spline adjustment), the pass trajectory points are copied to the new trajectory. The rest of the points are calculated by evaluating the spline and transforming the output coordinates to not local coordinates. Worth noticing the change in the velocity of the car from line 393 to 398. The speed change is decided on the behavior part of the code, but it is used in that part to increase/decrease speed on every trajectory points instead of doing it for the complete trajectory.

### 3. Data from Simulator

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

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

#### Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

### 4. Dependencies

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

### Conclusion

* The project's difficulty level can definitely escalate quickly. 
* A more sophisticated path planner should be used with specific cost functions. 
* In this case, for a quick fix, a 'hard code' method was approach where in summary, "Stay in the middle lane, overtake if necessary". A re-visit is what I intend later on but currently satisfy the rubrics.
* Also, was curious what happens if i were to increase speed from 50mph to 200mph:
![alt text][image3]
### Additional resources/ information

#### Further Improvement Suggestions

* [Introduction to Robotics #4: Path-Planning](http://correll.cs.colorado.edu/?p=965)
* [The path planning problem in depth](https://www.cs.cmu.edu/afs/cs/project/jair/pub/volume9/mazer98a-html/node2.html)
* [Roborealm](http://www.roborealm.com/help/Path_Planning.php)
* A discussion on [What is the difference between path planning and motion planning?](https://robotics.stackexchange.com/questions/8302/what-is-the-difference-between-path-planning-and-motion-planning)
* [Excellent Tutorial on A* Robot Path Planning](https://www.robotshop.com/community/forum/t/excellent-tutorial-on-a-robot-path-planning/13170)
* [Path Planning in Environments of Different Complexity](https://www.mathworks.com/help/robotics/ug/path-planning-in-environments-of-difference-complexity.html;jsessionid=d957f0f3b079d18368e234a6997f)
* [Introduction to robot motion: Robot Motion Planning](http://ais.informatik.uni-freiburg.de/teaching/ss11/robotics/slides/18-robot-motion-planning.pdf)
* [Introduction to robot motion: Path Planning and Collision Avoidance](http://ais.informatik.uni-freiburg.de/teaching/ss10/robotics/slides/16-pathplanning.pdf)
* [Path Planning for Collision Avoidance Maneuver](https://www.researchgate.net/publication/267596342_Path_Planning_for_Collision_Avoidance_Maneuver)
* [Optimal Trajectory Planning for Glass-Handing Robot Based on Execution Time Acceleration and Jerk](https://www.hindawi.com/journals/jr/2016/9329131/)
* [This discussion on StackExchange can be of interest Which trajectory planning algorithm for minimizing jerk](https://robotics.stackexchange.com/questions/8555/which-trajectory-planning-algorithm-for-minimizing-jerk)
* [Here is another great paper](http://ai.stanford.edu/~ddolgov/papers/dolgov_gpp_stair08.pdf) written by Sebastian Thrun which discusses the practical search techniques in path planning.
* This is another great resource based on ["A path planning and obstacle avoidance algorithm for an autonomous robotic vehicle"](https://webpages.uncc.edu/~jmconrad/GradStudents/Thesis_Ghangrekar.pdf)

#### Indoors
* [Intention-Net: Integrating Planning and Deep Learning for Goal-Directed Autonomous Navigation by S. W. Gao, et. al.](https://arxiv.org/abs/1710.05627)

#### City Navigation
* [Learning to Navigate in Cities Without a Map by P. Mirowski, et. al.](https://arxiv.org/abs/1804.00168)

#### Intersections
* [A Look at Motion Planning for Autonomous Vehicles at an Intersection by S. Krishnan, et. al.](https://arxiv.org/abs/1806.07834)

#### Planning in Traffic with Deep Reinforcement Learning
* [DeepTraffic: Crowdsourced Hyperparameter Tuning of Deep Reinforcement Learning Systems for Multi-Agent Dense Traffic Navigation by L. Fridman, J. Terwilliger and B. Jenik](https://arxiv.org/abs/1801.02805)


