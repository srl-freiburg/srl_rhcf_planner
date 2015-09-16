# SRL_RHCF_PLANNER 
###### Randomized Homotopy Classes Finder for Social Navigation developed within the context of the EU FP7 project [SPENCER](http://www.spencer.eu).


![alt tag](http://www2.informatik.uni-freiburg.de/~palmieri/images/cover_girl.png)

**Description of the figure**: The **red** Voronoi diagram, which implicitly encodes homotopy classes, describes the possible ways to go through a crowd in a room. Our approach rapidly selected two possible paths (in **yellow** and **green**).

### Motivation 
To deal with unexpected obstacles and quickly react to dynamic world's changes in social settings we utilize a fast random walk approach (RHCF) to generate a set of K distinct paths, belonging to different homotopy classes. The best path among those found by RHCF is then used to generate a kinodynamic trajectory.


### Description of the Package
SRL_RHCF_PLANNER is a ROS move_base package that implements a fast hierarchical motion
planning framework, which reactively generates an optimal kinodynamic
trajectory in the best homotopy class found by the algorithm Randomized Homotopy
Class Finder (RHCF).

The package is a global_planner plug-in for move_base. It adhers to the specifics of nav_core::BaseGlobalPlanner, please check for further details on move_base refer to http://wiki.ros.org/move_base.

In this global planner, firstly a set of homotopy classes is generated using RHCF from a socially-informed Voronoi diagram, then a nonholonomic RRT* based algorithm generates a smooth kinodynamic trajectory in the best homotopy
class among those found by RHCF.


![alt tag](https://docs.google.com/drawings/d/1pJRteOwdmayL_jTUQHmQ1CtwFKKFfWJJI9OUvlbb_wc/pub?w=480&h=360)

### Requirements
* ROS (including visualization rools -> rviz), tested on Indigo and Hydro
* ros-hydro-navigation or ros-indigo-navigation
* Eigen3
* Boost >= 1.46
* C++11 compiler
* spencer_people_tracking package (state-of-the-art people tracker) from the EU-Project Spencer github repository, https://github.com/spencer-project/spencer_people_tracking

### Installation

Clone the package into you catkin workspace
- `cd [workspace]/src`
- `git clone https://github.com/srl-freiburg/srl_rhcf_planner.git`
- `cd ../`
- `catkin_make` or `catkin build`

### Usage
- roslaunch srl_rhcf_planner move_base_global_srl.launch will launch the global planner node. You can launch the planner with different configurations, by varying some parameters:
  - `useVoronoiRandWalk` set to true if want to use the random walk, otherwise other planning methods could be choosed (see afterwards);  
  - `type_weight`, defined the Distance used to generate the Voronoi Diagram: 
    -   0, Euclidean Distance;
    -   1, Socially informed Distance;
  - `weighting_gain`, gain associated to the humans direction, standard value equal to 10
  - `social_gain`, gain associated to the social force in the scene, standard value equal to 2
  - `border_gain`, gain used to push away the voronoi edges from the humans, standard value equal to 7
  - `discounting_factor`, used to tune the speed of the random walk. The number of random walks needed to generate K distinct paths, and consequently RHCF runtime, decreases monotonically as α goes from 1 to 0.5. Standard value equal to 0.8
  - `K_homotopy_classes`, number of homotopy classes to generates, among those select the best one and 
  - `SHOW_LOG_INFO`, if set to true extra info on the random walk are displayed
  - `TYPE_PLANNER`, set to:
    - 0, use `RRT`
    - 1, use `RRT*` only partial rewiring
    - 2, use `RRT*`
  - `NUMBER_UPDATE_TRAJ`, set to:
    - Choose after how many cost improvements the planner could stop, currently set at 2. Minimum value is 1. Higher the value, higher the computaion time required to generate a trajectory
  - `BOX` :
    - if it is set to 1, the nearest vertex is selected from a weighted box according to the Ball-Box Theorem.
  - `RADIUS` :
    - the size of the radius where the near neighbor set is generated, in case you use `RRT*` select -1 so to have the `RRT*` shrinking ball.
  - `RHO` :
    - end condition for the POSQ steer function, should be set to a value of few cm.
  - `DT` :
    - integration time step of the POSQ steer function, maximum value 0.5s
  - `TYPE_SAMPLING` :
    - if `TYPE_SAMPLING` == 0 support set as uniform over a strips following a discrete path generate by a `Theta*` algorithm
    - If `TYPE_SAMPLING` == 1 support set as Gaussian Mixture over the `Theta*` path
    - if `TYPE_SAMPLING` == 2 support set as gaussians over a spline fitting the `Theta*` waypoints
    - if `TYPE_SAMPLING` == 3 support for `Theta*-RRT`, if set need to specify the range where to set orientations `OR_RANGE` and the width of the strip along the `Theta*` path `WIDTH_STRIP`
    - if `TYPE_SAMPLING` == 4 support set as the entire state space, the dimension of the state space are read from the grid generate by the move_base framework
    - if `TYPE_SAMPLING` == 5 Path Biasing along the current available trajectory. If used need to set also the biasing probability `BIAS_PROB` and the `DISPERSION`
  - `GOAL_BIASING`
    - if set to 1 activate goal biasing.
  - `GOAL_BIASING_THS`
    - set the probability to select a state not in the state space
  - `ADD_COST_FROM_COSTMAP`, set to true if you want to add cost from global cost map
  - `ADD_COST_PATHLENGTH`, set to true if you want to add the cost associated to path length and changes of heading
  - `ADD_COST_THETASTAR`, set to true if you want to add cost resembling closeness to thetastar path
  - Params related to the distance metric, only one of them shoul be set to 1. `LEARNED` and `NOTLEARNED` select the best vertex from the spherical neighborhood with radius equal to the parameter `RADIUS`:
    - `LEARNED`, set to 1, if you want to find the nearest vertex according to the learned cost
    - `FINDNEAREST`, set to 1 if you want to find the nearest vertex according to the Kd Tree Euclidean Distance
    - `NOTLEARNED`, set to 1 if you want to find the nearest vertex according to the cost computed over extensions of POSQ path
  - `TIMECOUNTER`, set to 1 if you want to specify the maximum amount of seconds your planner should work.
  - `MAXTIME`, max number of seconds allowed to find a path
  - `max_iterations`, if `TIMECOUNTER` is 0, this is the maximum number of iterations the planner will execute to find a path,

### Developers
Any contribution to the software is welcome. Contact the current developers for any info: 
* Luigi Palmieri (https://github.com/palmieri, palmieri(at)informatik.uni-freiburg.de)
* Andrey Rudenko (andrey.rudenko(at)saturn.uni-freiburg.de)

### TODOs:
* Rewrite the code to be conform to the ROS Cpp style guide, see http://wiki.ros.org/CppStyleGuide
* Use dynamic reconfiguration of the parameters, see http://wiki.ros.org/dynamic_reconfigure
* Improve documentation
