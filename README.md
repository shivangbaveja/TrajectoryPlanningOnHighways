# TrajectoryPlanningOnHighways
Planning and Decision Making 16-721 Course Project

#Code modules:
1. Trajectory generation module written in python3. Path optimization is done using iterative numerical method described in https://www.learnopencv.com/install-opencv3-on-ubuntu/.
The curvature is assumed to be cubic polynomial of curve length. Here is a brief description of the optimization algo:
	1. parameters: curvarture at (1/3) curve length k1, curvarture at (2/3) curve length k2, final curvature k3, curve length s
	2. cost: euclidean distance of goal configuration and current state
	3. parameters are tweaked and delta cost is computed by projecting the motion model forward.    
	4. gradient of cost wrt parameter is computed and used to update the parameter using gradient descent algorithm
	5. optimization is continued until is cost is below a certain threshhold
	6. The paths generated are converted to trajectories by fitting a acceleration profile to them.

This module generates a bunch of trajectories at different initial speeds within the set {3m/s, 6m/s, ... 21m/s}. For each state there are 3 actions, left lane change, right lane change and straight path. For each of these actions, there are three associated trajectories one with acceleration, one with deceleration and at constant speed. So for each state 9 trajectories are computed. All the pre-computed trajectories are stored in a json file called the actions.json.

2. Planning-cpp module: At runtime, it first loads actions.json and then runs the specific test case. Planner is using openCV for visualization and also collision checking. Before running planner, distance transform is computed in x,y, and t which is used for collision checking and also for assigning a cost to the trajectory.


#Install the following:
1. OpenCV 3: https://www.learnopencv.com/install-opencv3-on-ubuntu/
2. python -mpip install matplotlib


#To build and run:
1. cd cpp
2. mkdir build
3. cd build
4. cmake ..
5. make
6.  ./planner 0


#Different test cases are implemented:
0: No obstacles: The environment has no other vehicles
1:Fast  obstacles: The  environment  has  other  vehicles that travel faster than the ego vehicle
2: Slow middle obstacles: The environment has other vehicles in the middle lane that travel slower than the ego vehicle
3: Change lane: The environment forces the ego vehicle to choose a lane
4: Wait and change: The environment forces the ego vehicle to wait a significant distance before performing a lane change.
5: Multiple lane change: The environment has a lot of static obstacles.









 
