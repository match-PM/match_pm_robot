# match Precision Assembly Robot 
ROS Framework for the Precision Assembly Robot
## 1. Repository overview
* `pm_robot_control`: Contains the control of the robot via OPC UA
* `pm_robot_control_test`: Contains a publisher to control the in gazebo
* `pm_robot_description`: Contains the description of the robot and launch files for gazebo and rviz
* `pm_robot_submodules`: Contains submodules for e.g. Basler cameras

## 2. Installation 
Start by changing directory to your workspace!

### Clone package
```
git clone https://github.com/match-PM/match_pm_robot.git
```
## 3. Install packages
Make sure that you have installed the following packages:

```
sudo apt install ros-humble-desktop-full
```
```
sudo aptinstall ros-humble-xacro
```
```
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gazebo-ros2-control
```
```
sudo apt install ros-humble-rqt*
```
```
ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller --force-discover
```
Add these commands to the .bashrc file: 
```
source /opt/ros/humble/setup.bash
```
```
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
```
```
source ~/ros2_ws/install/setup.bash
```

### 4. Build packages
Use your standard build tools to build the downloaded packages e.g. : 
```
colcon build  --symlink-install
```


## 4. Usage
To use the robot in gazebo or rviz, you can launch the model

```
ros2 launch pm_robot_description pm_robot_gazebo.launch.py 
```
or
```
ros2 launch pm_robot_description pm_robot_gazebo.rviz.py 
```

## 5. External documentation
[ROS 2 - Humble - Documentation and Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)
[Building a robot with ROS2](https://www.youtube.com/@ArticulatedRobotics/playlists)


