# SRL_RHCF_PLANNER 

SRL_RHCF_PLANNER is a ROS move_base package that implements a fast hierarchical motion
planning framework, which reactively generates an optimal kinodynamic
trajectory in the best homotopy class found by the algorithm Randomized Homotopy
Class Finder (RHCF).

The package is a global_planner plug-in for move_base. It adhers to the specifics of nav_core::BaseGlobalPlanner, please check for further details on move_base refer to http://wiki.ros.org/move_base.

![alt tag](http://www2.informatik.uni-freiburg.de/~palmieri/images/cover_girl.png)


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


