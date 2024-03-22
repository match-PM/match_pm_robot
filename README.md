# match Precision Assembly Robot 
ROS Framework for the Precision Assembly Robot
## 1. Repository overview
* `pm_hardware_interface`: Contains the hardware interface for the precision assembly robot
* `pm_robot_control`: Contains the control of the robot via OPC UA
* `pm_robot_bringup`: Contains the launch files to bringup the robot in real or simulation
* `pm_robot_control_test`: Contains a nodes to sample nodes to control the robot
* `pm_robot_description`: Contains the description of the robot 
* `pm_robot_moveit_config`: Contains the moveit config 
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
sudo apt install ros-humble-xacro
```
```
sudo apt install ros-humble-moveit
```
```
sudo apt install ros-humble-moveit
```
```
sudo apt install ros-humble-moveit-visual-tools
```
```
sudo apt install ros-humble-graph-msgs
```
```
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
```
```
ros-humble-gazebo-ros2-control
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
In the package pm_robot_bringup, you can find two launch files. 
You can launch the robot in a simulation environment or use the real robot.
You can define the robot configuration (Gonimeter, Gripper, etc.) into the launch files. 

### Real Hardware:
To use the real hardware (robot), the robot has to be switched on and the function Debug7 has to be running. You can control the robot axis via forward_position_controller. 

```
ros2 launch pm_robot_bringup pm_robot_real_HW.launch.py 
```
### Simulation:
Using the simualtion, the robot is spawned in gazebo. You can control the robot axis via joint_trajectory_controller. To plan trajectories, you can use Moveit. 
```
ros2 launch pm_robot_bringup pm_robot_sim_HW.launch.py 
```

## 5. External documentation
[ROS 2 - Humble - Documentation and Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)
[Building a robot with ROS2](https://www.youtube.com/@ArticulatedRobotics/playlists)


