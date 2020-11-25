# C++ Capstone Project Description: Ball Chaser OOP

For completion of the C++ capstone project I have chosen to apply object oriented principles (OOP) to the 'GoChaseIt!' project completed as a part of the 'Robotics Software Engineer' Nanodegree program. The 'GoChaseIt!' project called for the implementation of a ball chaser application in which a custom robot placed in a custom gazebo world detects and follows a white ball in it's field of view. The app employs 2 ROS nodes and 1 service (server and client) to achieve the necessary communication between the main process, the robot motor controller, and camera input used for detection. The manner in which these elements are implemented in the course project does not make use of object-oriented concepts that allow for abstraction and encapsulation, which is my capstone submission will rectify.

## What is different
The basic functionality of the application is unchanged, however node/service handling and callbacks have been structured in the class BallChaser. 
The previously submitted 'GoChaseIt!' project can be found here: https://github.com/kyle-stanhouse/GoChaseIt

The following criteria/specifications from the Capstone rubric are met.

**Criteria:** _The project uses Object Oriented Programming techniques._\
**Meets Specifications:** _The project code is organized into classes with class attributes to hold the data, and class methods to perform tasks._

**Criteria:** Classes use appropriate access specifiers for class members.
**Meets Specifications:** All class data members are explicitly specified as public, protected, or private.

**Criteria:** Class constructors utilize member initialization lists.
**Meets Specifications:** All class members that are set to argument values are initialized through member initialization lists.

**Criteria:** Classes abstract implementation details from their interfaces.
**Meets Specifications:** All class member functions document their effects, either through function names, comments, or formal documentation. Member functions do not change program state in undocumented ways.

**Criteria:** Classes encapsulate behavior.
**Meets Specifications:** Appropriate data and functions are grouped into classes. Member data that is subject to an invariant is hidden from the user. State is accessed via member functions.

**Criteria:** The project makes use of references in function declarations.
**Meets Specifications:** At least two variables are defined as references, or two functions use pass-by-reference in the project code.

Possibly
**Criteria:** The project uses destructors appropriately.
**Meets Specifications:** At least one class that uses unmanaged dynamically allocated memory, along with any class that otherwise needs to modify state upon the termination of an object, uses a destructor.

**Criteria:** The project uses smart pointers instead of raw pointers.
**Meets Specifications:** The project uses at least one smart pointer: unique_ptr, shared_ptr, or weak_ptr. The project does not use raw pointers.

**Criteria:** The project uses multithreading.
**Meets Specifications:** The project uses multiple threads in the execution.


## Directory Structure

```
    .cpp_capstone                      # Capstone Project
    ├── my_robot                       # my_robot package                   
    │   ├── launch                     # launch folder for launch files   
    │   │   ├── robot_description.launch
    │   │   ├── world.launch
    │   ├── meshes                     # meshes folder for sensors (lidar)
    │   │   ├── hokuyo.dae
    │   ├── urdf                       # urdf folder for xacro description files
    │   │   ├── my_robot.gazebo
    │   │   ├── my_robot.xacro
    │   ├── world                      # world folder for world files
    │   │   ├── ball_chaser_world.world
    │   ├── CMakeLists.txt             # compiler instructions
    │   ├── package.xml                # package info
    ├── ball_chaser                    # ball_chaser package                   
    │   ├── launch                     # launch folder for launch files   
    │   │   ├── ball_chaser_OOP.launch
    │   ├── src                        # source folder for C++ scripts
    │   │   ├── drive_bot.cpp
    │   │   ├── process_images_OOP.cpp
    │   ├── srv                        # service folder for ROS services
    │   │   ├── DriveToTarget.srv
    │   ├── CMakeLists.txt             # compiler instructions
    │   ├── package.xml                # package info                  
    └──        
```

# Basic Build Instructions

## Dependencies 
* Gazebo >= 7.0  
* ROS Kinetic  
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
  
  ## Tips  
1. It's recommended to update and upgrade your environment before running the code.  
```bash
sudo apt-get update && sudo apt-get upgrade -y
```
  
## Install and run 

* Create a `catkin_ws`
`$ mkdir -p /home/workspace/catkin_ws/`
`$ cd /home/workspace/catkin_ws/src`
`$ catkin_init_workspace`
 
 test this in project workspace 

* Clone repository
`git clone https://github.com/kyle-stanhouse/cpp_capstone.git`

* Build application 
`catkin_make`

* Run project
Open up terminal 1
`roslaunch my_robot world.launch`
Open up terminal 2
`roslaunch ball_chaser_OOP ball_chaser.launch`

Observe the robot follow the ball as you move it around the gazebo world. 
The robot will stop when the white ball is not in it's field of view.




