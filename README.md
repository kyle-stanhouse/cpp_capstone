## C++ Capstone Project Description: Ball Chaser OOP

For completion of my C++ capstone project I have chosen to apply object oriented principles (OOP) to the 'GoChaseIt!' project completed as a part of the 'Robotics Software Engineer' Nanodegree program. The 'GoChaseIt!' project called for the implementation of a ball chaser application in which a custom robot placed in a custom gazebo world detects and follows a white ball in it's field of view. There are 2 nodes initialized and 1 service (w/client) that are used to achieve the communication between the main process, the robot motor controller, and camera input used for detection. These nodes are 
 
My previously submitted 'GoChaseIt!' project can be found here: https://github.com/kyle-stanhouse/GoChaseIt


# What is different
The basic functionality of the application is unchanged, however the processes and node handling have been added to classes such that abstraction is achieved.

# Directory Structure

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

## Basic Build Instructions

# Dependencies 
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
  
# Install and run 

* Create a `catkin_ws
`$ mkdir -p /home/workspace/catkin_ws/
`$ cd /home/workspace/catkin_ws/src
`$ catkin_init_workspace

`catkin_make



* Clone repository
`git clone https://github.com/kyle-stanhouse/cpp_capstone.git


  

## Tips  
1. It's recommended to update and upgrade your environment before running the code.  
```bash
sudo apt-get update && sudo apt-get upgrade -y
```
  
  


