# SRL_RHCF_PLANNER 

SRL_RHCF_PLANNER is a ROS move_base package that implements a fast hierarchical motion
planning framework, which reactively generates an optimal kinodynamic
trajectory in the best homotopy class found by the algorithm Randomized Homotopy
Class Finder (RHCF).

The package is a global_planner plug-in for move_base. It adhers to the specifics of nav_core::BaseGlobalPlanner, please check for further details on move_base refer to http://wiki.ros.org/move_base.


In this global planner, firstly a set of homotopy classes is generated using RHCF from a socially-informed Voronoi diagram, then a nonholonomic RRT* based algorithm generates a smooth kinodynamic trajectory in the best homotopy
class among those found by RHCF.

![alt tag](http://www2.informatik.uni-freiburg.de/~palmieri/images/cover_girl.png)

Description of the figure: The **red** Voronoi diagram, which implicitly encodes homotopy classes, describes the possible ways to go through a crowd in a room. Our approach rapidly selected two possible paths (in **yellow** and **green**).

## Requirements
* ROS (including visualization rools -> rviz), tested on Indigo and Hydro
* ros-hydro-navigation or ros-indigo-navigation
* Eigen3
* Boost >= 1.46
* C++11 compiler

## Installation

Clone the package into you catkin workspace
- `cd [workspace]/src`
- `git clone https://github.com/srl-freiburg/srl_rhcf_planner.git`
- `cd ../`
- `catkin_make` or `catkin build`


