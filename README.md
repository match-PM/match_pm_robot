# match Precision Assembly Robot 
ROS Framework for the Precision Assembly Robot
## 1. Repository overview
* `pm_robot_control`: Contains the control of the robot via OPC UA
* `pm_robot_control_test`: Contains a publisher to control the in gazebo
* `pm_robot_description`: Contains the description of the robot and launch files for gazebo and rviz
* `pm_robot_submodules`: Contains submodules for e.g. Basler cameras

## 2. Installation 
Start by changing directory to your catkin workspace!

### Clone package
```
git clone https://github.com/match-PM/match_pm_robot.git
```
### Install dependencies
Browse to `your_catkin_workspace/src/match_pm_robot` and execute
```
git submodule update --init --recursive
cd ../..
rosdep install -i --from-path src --rosdistro humble -y
```
If the last `rosdep install` command is not working, please try the following:
```
rosdep install --from-paths src --ignore-src -r -y
```

### Build packages
Use your standard build tools to build the downloaded packages e.g. : 
```
colcon build  --symlink-install
```


## 3. Usage
To use the robot in gazebo or rviz, you can launch the model

```
ros2 launch pm_robot_description pm_robot_gazebo.launch.py 
```
or
```
ros2 launch pm_robot_description pm_robot_gazebo.rviz.py 
```

## 4. External documentation
[ROS2 Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)
