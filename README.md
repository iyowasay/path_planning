# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program  

Here's [the link to the video](https://www.youtube.com/watch?v=PO55TqNiEn4). 

## Intro

In this project our goal is to safely navigate around a virtual highway in the Udacity simulator, with other traffics that are driving ± 10 MPH of the 50 MPH speed limit. The localization of ego vehicle and sensor fusion data(only the surrounding vehicles on the same side of the road) are provided. Note that other cars will try to change lanes too. In addition, a sparse map list of waypoints around the highway is given in `data/highway_map.csv`. With these data, one should complete the project by using motion planning methods and weighted cost functions. 

## Limitations

* The speed should go as close as possible to the speed limit of 50 MPH.
* The total acceleration and jerk should not exceed 10 m/s^2 and 10 m/s^3 respectively, since these movements result in uncomfortable passanger experiences.
* The car should avoid collision at all cost and drive inside the lane at all times except during lane changing.

## Given data

#### 1. Map data of the highway 

The data from `highway_map.csv` is a vector of different parameters`[x,y,s,dx,dy]`.

- x and y are the waypoint's map coordinate position
- the s value is the distance along the road to get to that waypoint in meters
- the dx and dy values define the unit normal vector pointing outward of the highway loop.

#### 2. Localization data of ego car (No Noise)

- x, y: the car's x, y position in map(global) coordinates
- s, d: the car's s, d position in frenet coordinates
- yaw: the car's yaw angle in the map
- speed: the car's speed in MPH

#### 3. Previous path data given to the Planner

The simulator returns the previous list but with processed points removed(remaining or unused points). They can work as a useful tool to show how far along the path has processed since last time(`previous_path_x, previous_path_y`). Besides, the frenet values of the last processed point from previous list are also provided by the simulator(`end_path_s`, `end_path_d`).

#### 4. Sensor fusion data, 

This contains a list(a 2D vector) of all other car's attributes on the same side of the road without noises.

- `sensor_fusion[0]`: unique ID
- `sensor_fusion[1], [2]`: x and y position in map coordinates
- `sensor_fusion[3], [4]`: x and y velocity in m/s
- `sensor_fusion[5], [6]`: s and d position in frenet coordinates

## Concepts

Path planning is an important and complicated task. It depends on the environment of different situations.
For cases in the parking lot or maze-like spaces, it is reasonable to use Hybrid A star algorithm, which gives continuous and drivable solutions. On the other hand, a finite state machine with transition function would work in the highway scenario for this project. In general, it can be divided into three main steps: behavior control, prediction, and trajectory generation. 

#### 1. Behavior control(Planner)

Behavior planning is a high level decision making process. It has the lowest update rate and need to take all the possible variables, such as static objects and dynamic objects, into account. Finite state machine is one of the most common methods and hierarchical state machine is the advanced version of FSM. However, it still has some disadvantages. For instance, FSM is not able to handle unencoutered scenarios. Futher developments of planning methods include rule-based behavior planner or reinforcement learning.

In this project, only four states are implemented. They are keep lane, change lane left, change lane right and prepare lane change(when `collision_warn` is true). 

#### 2. Prediction

In order to safely traverse the environment, we must be able to perceive and predict the motion of other dynamic objects, such as pedestrians, other vehicles, and follow traffic rules at the same time. Incorporating uncertainty in this part is critical for generalization. This means not only the noise of data but also how much we trust the process model. One can use model-based or data-driven method or implement hybrid approaches(a combination of both methods). 

For model-based approach, we define process models mathematically, and then implement Autonomous Multiple Model algorithm(probabilistical classification). As for data-driven method, we first acquire data with sensors, define a measure of similarity(of trajectories), and then use trajectory clustering to compare and predict possible movements.

<img src="/predict.png" alt="table" width="700" height="400"/>

#### 3. Trajectory generation

The next step is to generate drivable trajectories. As an engineer, one needs to consider not only safty issues but also passenger comfort. When it comes to motion planning problems, the Completeness and Optimality are also important. In reality, most self driving cars have several trajectory planners they can use depending on the situation. One common sampling-based algorithm is called Polynomial Trajectory Generation, which produces jerk minimizing trajectories. 

In this project an [opensource library called Spline](http://kluge.in-chemnitz.de/opensource/spline/) is suggested. It takes a set of x,y coordinates and generates a function of the fitting curve. 

<!-- Linearization: to get number N of equally spaced points in a segment from starting point to target point, we need to linearize the actual path(the curve we got using spline) and coordinate transformation in order to make the math easier.

Need to incorporate time T(3 dimensional problem)

Feasibility checking for the generated trajectories
logitudinal acceleration 
lateral acceleration -->

## Flow chart

<img src="/chart.png" alt="table" width="900" height="350"/>

#### The use of Frenet coordinate and previous waypoints

In this project, we mainly work with two sets of points. One is the points that are going to be passed into the simulator and used by the car(future trajectory points). The other one corresponds the input points of Spline function. Here we also introduce a new coordiante system called the Frenet coordinate frame. It describes the position of a with respect to the road and makes our work easier. As for the previous point, the simulator returns a set of points that are not processed or executed by the ego car. We then can take advantage of these points and add them into the new set of predicted path points. This makes the trajectory smoother. However, if one use too many previous points, the system would not be able to react when there's a sudden change in the surroundings. Thus, one should be aware of this tradoff when designing the planner.

<!--tradeoff between hybrid a star and finite state machine, when should one use hybrid a star
Different environments: Highway(sparse space) v.s. parking lot(dense and discretized) or intersection -->

## Code explanation

#### Constant definition in main.cpp

| Variable name        | Value     | Remarks|
| ------------- |:-------------:| :-------------:|
| `lane_width`      | 4 |  single lane width   | 
| `SPEED_LIMIT`    |    49.5  | slightly lower than actual speed limit |
| `MPH2MPS`|  0.44704  | convert from miles per hour to m/s|
| `TOTAL_POINTS `    |    50  | total numper of points to pass into the simulator|
| `NUM_LANES`|  3  | number of lanes|
| `UPDATE_RATE`|  0.02 | unit:second|
| `WAIT_AFTER_CHANGE `|  60  | avoid changing lane immediately after a lane change |

#### `decide_better_lane()` in helpers.h

This function takes current lane, current distance to the car ahead of the ego vehicle, and sensor fusion data as inputs. First we define suitable distances(`front_gap`, `back_gap`) between current s coordinate of ego car and approaching vehicles. These distances should allow us to make a safe lane change. Then we loop through the sensor data of adjacent lanes and keep track of the closest distances between the cars in front and behind and the ego car. If we are in the middle lane(lane 1), choose the left or right lane change that has larger space(larger gap to the car ahead of you). If in lane 0 or lane 2, make a lane change when the gap is large enough. Otherwise the output should be the same as the current lane when all the safety criteria are not satisfied. 

#### Speed planner in main.cpp

```c++
iter_count++;
if(collision_warn)
{
	if(iter_count > WAIT_AFTER_CHANGE)
	{
		int curLane = lane;
		lane = decide_better_lane(curLane, car_s, NUM_LANES, cur_gap, sensor_fusion);
		if(lane != curLane) // lane changed
	    	iter_count=0;
	}

	if(cur_speed >= max_speed)
	{
		cur_speed -= 0.2*(1+0.28*(safe_gap-cur_gap)/safe_gap);
		if(cur_speed < 0)
			cur_speed = 0.0;
	}
	else if(cur_speed < max_speed)
		cur_speed += 0.224;
}
else if(cur_speed < max_speed)
	cur_speed += 0.224;
```

The variable `iter_count` is used to avoid constant lane change. After a lane change is performed, the car should wait for a short period of time and not make another lane change immediately. This makes sure that the limitation of acceleration is not violated. Besides, we keep track of the speed of the car in front of the ego car. If we are in the state of preparing lane change, the ego adjusts it's speed to match the speed of the preceding vehicle. By doing so, the ego car can maintain a safe gap to the car in front of it while looking for a better chance of lane change.


<!-- ## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.
 -->




