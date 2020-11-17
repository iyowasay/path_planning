# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program  

### Intro

In this project our goal is to safely navigate around a virtual highway in the Udacity simulator, with other traffic that is driving ± 10 MPH of the 50 MPH speed limit. The localization of ego vehicle and sensor fusion data(only the surrounding vehicles on the same side of the road) are provided. Note that other cars will try to change lanes too. In addition, a sparse map list of waypoints around the highway is given in data/highway_map.txt. With these data, one should complete the project by using motion planning methods and weighted cost functions. 

### Limitations

* The speed should go as close as possible to the 50 MPH speed limit.
* The total acceleration and jerk should not exceed 10 m/s^2 and 10 m/s^3 respectively. These cause uncomfortable passanger experience.
* The car should avoid collision at all cost and drive inside the lane at all times except during lane changing.

### Given data

#### Map data of the highway [x,y,s,dx,dy]

- x and y are the waypoint's map coordinate position
- the s value is the distance along the road to get to that waypoint in meters
- the dx and dy values define the unit normal vector pointing outward of the highway loop.

#### Localization data of ego car (No Noise)

- x, y: the car's x, y position in map(global) coordinates

- s, d: the car's s, d position in frenet coordinates

- yaw: the car's yaw angle in the map

- speed: the car's speed in MPH

#### Previous path data given to the Planner

The simulator returns the previous list but with processed points removed(remaining and unused points). They can be a nice tool to show how far along the path has processed since last time(`previous_path_x, previous_path_y`). Besides, the frenet values of the last processed point from previous list are provided by the simulator(`end_path_s`, `end_path_d`).

#### Sensor fusion data, 

This contains a list(a 2D vector) of all other car's attributes on the same side of the road without noise.

`"sensor_fusion"`: 
[0] - car's unique ID
[1],[2] - car's x, y position in map coordinates
[3],[4] - car's x, y velocity in m/s
[5],[6] - car's s and d position in frenet coordinates

### Concepts

Path planning is an important and complicated task. It depends on the environment of different situations.
For cases in the parking lot or maze-like spaces, it is reasonable to use Hybrid A star algorithm, which gives continuous and drivable solutions. On the other hand, a finite state machine with transition function would work for the highway scenario in this project. In general, it can be divided into three main steps: behavior control, prediction, and trajectory generation. 

#### Behavior control(Planner)

Behavior planning is a high level decision making process. It has the lowest update rate and need to take all the possible variables, such as static objects, dynamic objects, and , into account. Finite state machine is one of the most common methods. 

Hierarchical state machine

Can't handle unencoutered scenarios
Rule-based behaviour planner
Reinforcement learning

Finite state machine: keep lane, change lane left, change lane right, prepare lane change left, prepare lane change right.

#### Prediction

In order to safely traverse the road, we must be able to perceive and predict the motion of other dynamic objects, such as pedestrians, other vehicles, and follow traffic rules at the same time. Incorporating uncertainty in this part is critical for generalization. This means not only the noise of data but also how much we trust the process model. One can use model-based or data-driven method or implement hybrid approaches(a combination of both methods). For model-based approach, we define process models mathematically, and then implement Autonomous Multiple Model algorithm(probabilistical classification). As for data-driven method, we first acquire data with sensors, define a measure of similarity(of trajectories), and then use trajectory clustering to compare and predict possible movements.

<img src="/predict.png" alt="table" width="700" height="300"/>

#### Trajectory generation

The next step is to generate drivable trajectories.
Motion planning algorithm
Problem with Hybrid A star algorithm

### The use of Frenet coordinate and previous waypoints

If one use too many previous points, the system would not be able to react when there's a sudden change cause it's mostly following the previous path.

two sets of points: 
points that are going to be passed into the simulator and used by the car.(future path points)
points which construct the spline segment 

tradeoff between hybrid a star and finite state machine, when should one use hybrid a star
Different environments: Highway(sparse space) v.s. parking lot(dense and discretized) or intersection

### Path generation

#### Spline 

http://kluge.in-chemnitz.de/opensource/spline/

Linearization: to get number N of equally spaced points in a segment from starting point to target point, we need to linearize the actual path(the curve we got using spline) and coordinate transformation in order to make the math easier.

Need to incorporate time T(3 dimensional problem)

Feasibility checking for the generated trajectories
logitudinal acceleration 
lateral acceleration

#### Polynomial 






## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.





