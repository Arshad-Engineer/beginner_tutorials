# beginner_tutorials
# Overview:
 - This is 'beginner_tutorials' - "First Publisher/Subscriber" ROS package on GitHub after working through the tutorials:
 - The publisher publishes a custom string "Hi, Arshad " followed by message count
 - The subscriber receives this data and prints "I heard: Hello, Arshad: " followed by the received data
 - Google Style Guide was followed in all the cpp files
 - CPPCHECK and CPPLINT was run to perform static code analysis
 - 'results' folder contains the output of CPPCHECK and CPPLINT in a text file

## Author:
 - Arshad Shaik
    - UID: 118438832

## Dependencies/Requirements: 
 - Laptop
 - Ubuntu 20.04 or higher
 - VS Code/Terminal
 - ROS 2 Humble

## Build & Run Instructions:
 - open terminal and source ros2
 ```
 source ~/ros2_humble/install/setup.bash
 printenv | grep -i ROS
 ```

 - create a workspace (if not created before)
 ```
 mkdir -p ~/ros2_ws/src
 cd ~/ros2_ws/src
 ```

 - Navigate to root of workspace
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
 colcon build --packages-select cpp_pubsub
 ```

 - open new terminal
 - Navigate to root of workspace
 ```
 cd ros2_ws
 ```
   - or
 ```
 cd <ros2_workspace_name>
 ```

 - source ros2
 ```
 . install/setup.bash
 ```

 - run publisher node
 ```
 ros2 run cpp_pubsub talker
 ```

 - open new terminal
 - Navigate to root of workspace
 ```
 cd ros2_ws
 ```
   - or
 ```
 cd <ros2_workspace_name>
 ```

 - source ros2
 ```
 . install/setup.bash
 ```

 - run subscriber node
 ``` 
 ros2 run cpp_pubsub listener
 ```

## Command to run static code analysis:
 - Navigate to src folder in package
 ```
 cd ros2_ws/src/cpp_pubsub/src
 ```
 - run the following command from src folder in package
 ```
 cppcheck --enable=all --std=c++17 *.cpp --suppress=missingIncludeSystem --suppress=missingInclude --suppress=unmatchedSuppression > ./../results/cppcheckreport
 ```

## Command to check Google Style:
 - Navigate to src folder in package
 ```
 cd ros2_ws/src/cpp_pubsub/src
 ```
 - run the following command from src folder in package
 ```
 cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order subscriber_member_function.cpp publisher_member_function.cpp > ./../results/cpplintreport
 ```
 - For reference puposes - clang formating command is listed here:
 ```
 clang-format -style=Google -i subscriber_member_function.cpp
 clang-format -style=Google -i publisher_member_function.cpp
 ```
## Dependency Installation: 
- ROS 2 Humble:
- Follow the below website instructions to install ROS 2 Humble based on your Ubuntu version
  - Ubuntu 20.04 (build from source):
    - https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html
- cpplint
```
sudo pip install cpplint
```
## Issues faced:
- VSCode is earlier configured to 'Intellisense" instead of 'Clang'.
- This created auto-building everytime the change is made and piled up the commits. Git is removed from source coontrol from VSCode
- Then C/C++ intellisense extensions are unistalled.
- Due to above, .basrc is accidentally deleted, resulting in losing of text color in the terminal
- cpplint installation is also deleted
- Ran into this issue when I accidentally deleted my ~/.bashrc file. Changing the Profile Preferences as stated in a comment above did not work. Since I completely lost my ~/.bashrc file, I simply copied /etc/skel/.bashrc to ~/.bashrc
