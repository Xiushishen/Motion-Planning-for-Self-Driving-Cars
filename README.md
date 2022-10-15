Motion Planning for Self-Driving Car
===
[behavioural planner](#Behavioural-Planner)//[local planner](#local-planner)//[static obstacle avoidance](#Static-Obstacle-avoidance)//[velocity profile generation](#Velocity-Profile-generation)//[running carla simulator](#Running-Carla-Simulator)//[Simulation](#Simulation)

This folder includes all files from the final project of the [Motion Planning for Self-Driving Cars](https://www.coursera.org/learn/motion-planning-self-driving-cars?specialization=self-driving-cars) course by University of Toronto.
This project involves implementation of a motion planner in CARLA simulator in python 3.6.

In order to implement a fully functional motion planner for autonomous car in the CARLA simulator the following modules were implemented:</br>
* Behavioural Planner</br>
* Local planner
* Static Obstacle avoidance
* Velocity Profile generation

The control module used in this simulation is similar to the one implemented in [vehicle control](https://github.com/JagtapSagar/Vehicle-Control/) project.

### Behavioural Planner
Implemented a state machine that transitions between lane following, deceleration to the stop sign, staying stopped, and back to lane following, when it encounters a stop sign. All the code for the behavioural planner is contained in behavioural_planner.py. Before the state machine transitions are implemented it is important to define parts of a intersection.

An intersection is divided in three discrete zones represented in the picture below.</br>
* Approaching zone of intersections (red) - area of the road in which the vehicle should begin breaking.
* At zone (green) - area at intersection where the vehicle should stop or wait.
* On the intersection (orange) - crossing intersection zone
<p align="center"><img src='https://github.com/JagtapSagar/Motion-Planning-for-Self-Driving-Car/blob/main/Images/discrete%20intersection%20zones.png' width="300" height="300"></p>

The images below illustrate workflow diagram of the state transitions.

Track speed | Follow lead vehicle
:-------------------------:|:-------------------------:
<img src='https://github.com/JagtapSagar/Motion-Planning-for-Self-Driving-Car/blob/main/Images/state_transitions_track_speed.png'> | <img src='https://github.com/JagtapSagar/Motion-Planning-for-Self-Driving-Car/blob/main/Images/state_transitions_follow_lead.png'>
**Decelerate to stop** | **Stay stopped**
<img src='https://github.com/JagtapSagar/Motion-Planning-for-Self-Driving-Car/blob/main/Images/state_transitions_decel_to_stop.png'> | <img src='https://github.com/JagtapSagar/Motion-Planning-for-Self-Driving-Car/blob/main/Images/state_transitions_stay_stopped.png'>


  
### Local Planner
The local planner generates a set of short paths that are safe for the self-driving car to traverse for it to get a step closer to its navigational goal. The following steps explains how the implementation works.
  * Compute the goal state set (the set of goal points to plan paths to before path selection) which includes the goal state about the path and alternate goal states at near it.
    
    In the illustration below the left most endpoint on the blue line is the vehicles current position and the yellow line represents the desired vehicle trajectory. The point at the other end of the blue line is a goal state a certain lookup distance ahead which the vehicle must plan path to. The blue dots next to it are alternate goal state that we calculate so that we have alternate paths that the vehicle can take in case of any obstacles in its way.
    
    <p align="center"><img src='https://github.com/JagtapSagar/Motion-Planning-for-Self-Driving-Car/blob/main/Images/Goal_horizon.png' width="300" height="300"></p>
  * Compute the yaw of the car at a set of arc length points for a given spiral which sets up the optimization problem for a given path. Once the optimization is complete, the resulting spiral will be sampled to generate the path. This is implemented in path_optimizer.py and integrated in local_planner.py.
    
    In the example below the red rectangle obstructs the red paths. All the green lines are obstacle free paths to alternate goal states. And the blue line is the obstacle free path that leads the self-driving car to a goal state nearest to the center goal state. To find which paths are free of obstacles an obstacle detection method is implemented (brief in the next section).
  
    <p align="center"><img src='https://github.com/JagtapSagar/Motion-Planning-for-Self-Driving-Car/blob/main/Images/collision_checking_and_path_selection.png' width="300" height="300"></p>

### Static Obstacle Avoidance</br>
To check whether a path is void of obstacles circle-based collision checking method for obstacle detection was implemented. The collision checking computes path set using the collision_check() function in collision_checker.py. Vehicle footprint is calculated with circles at various points on a path creating a swath of vehicle footprints along the path. One or more circles can be used to form the footprint of an object depending on its shape and size. If collision is detected between the circles covering the vehicle and the obstacle at any point on a path, then the path is not collision free.
  
  The image below presents the algorithm for collision detection for circle-based collision checking.
  
  <p align="center"><img src='https://github.com/JagtapSagar/Motion-Planning-for-Self-Driving-Car/blob/main/Images/circle_collision_checking.png' width="500" height="150"></p>

### Velocity Profile Generation
Now that we have the path selected, a velocity profile must be generated for the controls to execute. In this project a Linear Ramp Velocity profile was used. This is computed using the vehicle’s current velocity, its desired final velocity (Eg, 0 mph for stop line, lead vehicle speed or the road speed limit for lane following), as well as the path length to next goal state. Alternative methods like Trapezoidal velocity profile also could have be used.
This velocity planner does not handle all edge cases, but handles stop signs, lead dynamic obstacles, as well as nominal lane maintenance. The implementation of this can be found in velocity_planner.py.

The images below illustrate both the output velocity profile (in yellow) generated during this simulation and the controller outputs. These plots pertain to the simulation described in the Simulation section below.

 <p align="center"><img src='https://github.com/JagtapSagar/Motion-Planning-for-Self-Driving-Car/blob/main/Images/control_feedback.gif' width="300" height="600"></p>

Running Carla Simulator
---
To run this project:
1. Install Carla Simulator
2. Download this directory with all its files into a folder in the PythonClient directory within the Carla simulator directory.
3. Execute following command in the simulator directory to open the simulator.
   * For Ubuntu: `./CarlaUE4.sh /Game/Maps/Course4 -windowed -carla-server -benchmark -fps=30`
   * For Windows: `CarlaUE4.exe /Game/Maps/Course4 -windowed -carla-server -benchmark -fps=30`
4. Execute module_7.py from project directory.
   * For Ubuntu: `python3 module_7.py`
   * For Windows: `python module_7.py`

NOTE: 
* CARLA requires python version 3.5 or 3.6
* matplotlib version 3.0.0 was used for this project. Newer version might cause errors with the use of deprecated matplotlib.backends.tkagg import.



Simulation
---
The simulation begins with the self-driving car following the lead car while avoiding obstacles like parked cars in its lane. The animation below illustrates this part of the simulation. The vehicle avoids the parked car but continues to follow its lane.
<img src='https://github.com/JagtapSagar/Motion-Planning-for-Self-Driving-Car/blob/main/Images/CARLA_obstacle-avoidance_and_follow_lane.gif'>

The car stays in the FOLLOW_LANE state keeping a safe distance from the lead vehicle if any until the stop line at the T-intersection approaches. The vehicle enters DECELERATE_TO_STOP state as it enters the *Approaching zone of intersection* and starts slowing down. When the vehicle enters the *At zone* of the intersection at the stop line it enters the STAY_STOPPED state and remains stopped for a few seconds. Once the stop time is up and the path is clear of obstacles the vehicle then enters the FOLLOW_LANE state and exits right at the intersection travelling along the desired trajectory. The simulation comes to an end when the self-driving car stops at the next stop line.
<img src='https://github.com/JagtapSagar/Motion-Planning-for-Self-Driving-Car/blob/main/Images/CARLA_intersection.gif'>



