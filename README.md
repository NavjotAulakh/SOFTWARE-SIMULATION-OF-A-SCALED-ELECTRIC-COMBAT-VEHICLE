# Software Simulation of a Scaled Electric Combat Vehicle

#### The project goal was to deliver a simulation package for the scaled electric combat vehicle providing these specific functionalities: Autonomous capabilities, Object Detection, Avoidance & Classification. This scaled electric combat vehicle (SECV) has been under development at the University of Ontario Institute of Technology since 2016.

This project consists of two Separate teams working in parallel, Namely, the Software development team working on the autonomous driving simulation and the mechanical exterior design team working on vehicle dynamics.

The development of the simulation package involved the following:
#### SECV & Environment Model Design
- Design and development of universal robot description format file (URDF).
- Create/Develop Environment Model of SIRC 3RD Floor.
#### SECV Autonomous Navigation Design	
- Development of Ackermann Drive Controller for Scaled electric combat vehicle.
- Navigation & Planning of Scaled electric combat vehicle
- Configuration files of the AMCL localization package.
- Configuration files of the move_base package.
- Configuration of the robot interface for the purposes of relaying messages.
- Localization of TEB local planner parameters to provide ackerman steering support .

### SECV in Simulation Packages

#### ROS Environment - RVIZ
- Middleware for our Scaled electric combat vehicle.
- Distributed system of interconnected nodes running roscore which houses our core functionality.
![Image of RVIZ Simulation](https://lh4.googleusercontent.com/3bObty9X2w24tXEhdZSEYutz-CcMIyz1PGMnpgru0Zf8yfugCHNAq26PlWMlNE5NQlxy7OagiUd_vJwSJDM3Uzts9BVP0K6OJ16OUcx5QHDxMo4sfl8JmVpIhhG6HkAV6JBzMr7DFPiylA)
Following the installation Guide file above will allow users to launch the 8x8 Electronic combat vehicle in its environment ready for navigation 

#### Gazebo
- Provides module support and interface for simulation of robots in a simulated environment.
- Vehicle Simulation inlined with project constraints.
![Image of Gazebo Simulation](https://lh5.googleusercontent.com/IS0EMlulMqbV79KztjArsWNq4ZVpTh-oJwugvul-hMWqVMrh2rCvoJiGTdDEvxIBU_PINtzjIs1jJT9FqRdR4F7bofUb4_LnUokegdOd)

### Simulation Package Architecture 
![Image of SECV Package Architecture](https://lh3.googleusercontent.com/itG8StFvQImQhD89g9sphxT4X7uTK1CzA_k7yLbefZIgdHmXKwJkDuzbeF57w7qNOBBIS4iXfIgHOH2ZnvXMHrLx3MeZNiOi1nNsgmDokKPuWEESdPiA1LAs0qi--uMK__hbwGWmXGV1xQ)

The figure above shows the overall architecture of our simulation package. Starting with the top left, we have the robot’s data simulated in gazebo, a physics simulation software package. We configured to the simulation to include variables such as mass, friction, and inertia of the robot. This data is then transmitted to the navigation stack. 

The navigation stack is a software package that plans a path for the robot to reach its desired destination. RVIZ is short for robot visualizer which is the main interface a user has for setting goals and viewing the results. Finally, the Ackermann Steering Controller is a software program that calculates the actual velocities and angles of each wheel to create the desired movement.

## Installation Guide
Can be found in the files above - https://github.com/NavjotAulakh/SOFTWARE-SIMULATION-OF-A-SCALED-ELECTRIC-COMBAT-VEHICLE/blob/master/Installation%20Guide

## Authors
Navjot Aulakh
Jeffrey Zhang
Nico Zarfino
Musadiq Soso
