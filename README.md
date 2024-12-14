# Robot Controller with GUI 
A simple robot controller with GUI using ROS2. 

### Concept
Our robot can be turtle from turtlesim packages. You can control the robot in 4 directions: left/right and up/down. This project has a two nodes:
- robot_controller -> allows you to control the robot
- turtlesim -> a turtle simulation node that handles motion and visualization in the simulation

When program works, you can see actual robots coordinates. 

### Before use:
- install turtlesim packages:
```
sudo apt install ros-[ROS2_DISTRO]-turtlesim
```
- in each new window terminal you must activate ros2 environment with commands below:
```
source /opt/ros/humble/setup.bash
```
```
source ~/ros2_ws/install/setup.bash
```
  
ros2_ws is name of my workspace which contains all files of the project.

### Screens
![Controller](https://github.com/user-attachments/assets/0938fd50-f56b-4ce9-9f69-78a0562ce0e3)
![turtle](https://github.com/user-attachments/assets/c0c5dcc2-82b9-4a8f-ac96-3cb23d1b5978)

### How does it work
![robot](https://github.com/user-attachments/assets/a99c911a-4a49-4e50-9f06-44164a7f59fb)
