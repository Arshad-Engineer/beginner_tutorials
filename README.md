[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
---
# Overview:
 - Modify Talker node to broadcast a tf frame called /talk with parent /world. The transform should have non-zero translation and rotation. It may be static or time-variant.

## Personnel:
 - Arshad Shaik, UID: 118438832

## Dependencies/Requirements: 
  - Ubuntu 20.04 or higher
 - VS Code/Terminal
 - ROS 2 Humble

## Creating Workspace & Cloning Repo: 
 - Open terminal and source ros2
 ```
 source <ros2_humble_installation_directory>/install/setup.bash
 ```

 - Create a workspace (if not created before)
 ```
 mkdir -p ~/ros2_ws/src
 cd ~/ros2_ws/src
 ```

 - Navigate to your current workspace (if created before)
 - Clone the repo into src directory of your workspace
 ```
 git clone git@github.com:Arshad-Engineer/beginner_tutorials.git
 ```

## Build Instructions:
 - Open new terminal & navigate to root of workspace
 ```
 cd <ros2_workspace_name>
 ```
 - or
 ```
 cd ros2_ws
 ```

 - Check for missing dependencies before building the package
 ```
 rosdep install -i --from-path src --rosdistro humble -y
 ```

 - Build the package
 ```
 colcon build --packages-select beginner_tutorials
 ```

## Basic Run Instructions:
  - Running server node:
      - Open new terminal & navigate to root of workspace
      ```
      cd ros2_ws
      ```
      - or
      ```
      cd <ros2_workspace_name>
      ```
      - Source ros2
      ```
      . install/setup.bash
      ```

      - Run server node
      ```
      ros2 run beginner_tutorials publisher_member_function
      ```
      - Run server node with custom logger level
      ```
      ros2 run beginner_tutorials subscriber_member_function --ros-args --log-level WARN
      ```

  - Running client node (server node should be running):
      - Open new terminal & navigate to root of workspace
      ```
      cd ros2_ws
      ```
      - or
      ```
      cd <ros2_workspace_name>
      ```
      - Source ros2
      ```
      . install/setup.bash
      ```

      - Run client node by changing "string_to_modify" which will be received by client for modification
      ```
      ros2 run beginner_tutorials subscriber_member_function <string_to_modify>
      ```
      - Change one parameter in "publisher_member_function.cpp" from command line
      ```
      ros2 param set /minimal_param_node my_parameter earth
      ```

## Run from Launch File:
 - Run launch file which launches all the nodes at once & modifies one parameter in publisher_member_function.cpp
 ```
 ros2 launch beginner_tutorials tf_launch.py
 ```

## View logs in rqt console:
 - open new terminal, navigate to ROS 2 workspace and source ROS2
 - open rqt console
 ```
 ros2 run rqt_console rqt_console
 ```

 - open another terminal, navigate to ROS 2 workspace and source ROS2
 - Run server node
 ```
 ros2 run beginner_tutorials publisher_member_function
 ```

 - open another terminal, navigate to ROS 2 workspace and source ROS2
 - Run client node by changing "string_to_modify" which will be received by client for modification
 ```
 ros2 run beginner_tutorials subscriber_member_function <string_to_modify>
 ```

## Command to run static code analysis:
 - Navigate to src folder in package
 ```
 cd <ros2_workspace>/src/beginner_tutorials/src
 ```
 - run the following command
 ```
 cppcheck --enable=all --std=c++17 *.cpp --suppress=missingIncludeSystem --suppress=missingInclude --suppress=unmatchedSuppression > ./../results/cppcheckreport
 ```

## Command to check Google Style:
 - Navigate to src folder in package
 ```
 cd <ros2_workspace>/src/beginner_tutorials/src
 ```
 - run the following command
 ```
 cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order publisher_member_function.cpp subscriber_member_function.cpp > ./../results/cpplintreport
 ```

## Dependency Installation: 
- srv package:
  - A custom written srv file is used in this project which is needed for runing this program
  - It's called "tutorial_interfaces/srv/modify_string.hpp". 
  - This folder called "tutorial_interfaces" is a ROS package which has to be placed inside src directory of ros2 workspace along with other packages
  - Then execute the following command:
  ```
  colcon build --packages-select tutorial_interfaces
  ```
  - This will build the dependency of custom service required