# match Precision Assembly Robot 
ROS Framework for the Precision Assembly Robot
# 1. Repository overview
* `pm_hardware_interface`: Contains the hardware interface for the precision assembly robot
* `pm_robot_control`: Contains the control of the robot via OPC UA
* `pm_robot_bringup`: Contains the launch files to bringup the robot in real or simulation
* `pm_robot_control_test`: Contains a nodes to sample nodes to control the robot
* `pm_robot_description`: Contains the description of the robot 
* `pm_robot_moveit_config`: Contains the moveit config 
* `pm_robot_submodules`: Contains submodules for e.g. Basler cameras


# 2. Installation 
Start by changing directory to your workspace!

## Clone package
```
git clone https://github.com/match-PM/match_pm_robot.git
```
# 3. Install packages
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
sudo apt install ros-humble-gazebo-ros2-control
```
```
sudo apt install ros-humble-gazebo-ros-pkgs
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

# 4. Build packages
Use your standard build tools to build the downloaded packages e.g. : 
```
colcon build  --symlink-install
```


# 4. Usage
In the package pm_robot_bringup, you can find two launch files. 
You can launch the robot in a simulation environment or use the real robot.
You can define the robot configuration (Gonimeter, Gripper, etc.) into the launch files. 

## Real Hardware:
To use the real hardware (robot), the robot has to be switched on.
```
ros2 launch pm_robot_bringup pm_robot_real_HW.launch.py 
```
## Simulation:
There are two simulations. One in Unity and one in Gazebo.
The Unity Simulation encompasses all functions of the real robot, including sensors (camera, laser, force sensor). It requires you to install unity.

The Gazebo simulation is easier to use, but does not have every functionality.

### Gazebo
Using the gazebo simualtion, the robot is spawned in gazebo. You can control the robot axis via joint_trajectory_controller. To plan trajectories, you can use Moveit. 
```
ros2 launch pm_robot_bringup pm_robot_sim_HW.launch.py 
```

### Unity
For the unity simulation you need the install the repro https://github.com/match-PM/match_pm_robot_unity.git

To start the simulation, do the following:

1. Start the opcua-server
```
ros2 run opcua_server opcua_server
```

2. Start the unity simulation by enter the play mode

3. Start ros2
```
ros2 launch pm_robot_bringup pm_robot_unity_HW.launch.py 
```

# 5. External documentation
[ROS 2 - Humble - Documentation and Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)
[Building a robot with ROS2](https://www.youtube.com/@ArticulatedRobotics/playlists)




### Change the configured tool

`/pm_robot_primitive_skills/change_tool` uses `pm_msgs/srv/ChangeTool` with
`tool_name` and `attachment_name`. It validates the pair, activates its gripper,
and saves the bringup configuration selected by the running node's launch-time
`use_real_config` setting (real hardware: true; Gazebo/Unity: false). The response
contains `success`, `message`, and the saved file path within the message.

```bash
ros2 service call /pm_robot_primitive_skills/change_tool pm_msgs/srv/ChangeTool "{tool_name: 'Siemens_Vacuum_Array_angled', attachment_name: 'Tip_Siemens_Vacuum_Array'}"
```

This service only changes the configuration file. Restart the robot separately
to load the tool. It does not restart launches, wait, or reconfigure Unity.
The published launch-time configuration snapshot remains unchanged.

### Check and set robot configuration

The primitive skills node provides two services with identical request fields
(`rsap_path`, `file_path_override`) and response fields (`success`, `message`):

- `/pm_robot_primitive_skills/check_pm_robot_configuration`
  (`pm_msgs/srv/CheckPmRobotConfiguration`) checks the supplied YAML requirements
  against both the configuration captured at robot launch and the current bringup
  file. It also returns `success: false` if the complete bringup file differs from
  the launch snapshot, even outside the requested fields.
- `/pm_robot_primitive_skills/set_pm_robot_configuration`
  (`pm_msgs/srv/SetPmRobotConfiguration`) merges only the supplied fields into the
  active real-hardware or simulation bringup file. Unknown keys and incompatible
  value types are rejected before writing. `success: true` means the file was
  saved; the response message states whether a robot restart is required.

Both services default to `pm_robot_configuration.yaml` inside `rsap_path`.
`file_path_override` may name another relative file inside that directory or an
absolute path. For example, a requirement file can contain only:

```yaml
pm_robot_1K_dispenser_tip:
  use_dispenser_tip: Dis_Tip_Red_I_250_A520_L12700
```

```bash
ros2 service call /pm_robot_primitive_skills/check_pm_robot_configuration pm_msgs/srv/CheckPmRobotConfiguration "{rsap_path: '/path/to/process', file_path_override: ''}"
ros2 service call /pm_robot_primitive_skills/set_pm_robot_configuration pm_msgs/srv/SetPmRobotConfiguration "{rsap_path: '/path/to/process', file_path_override: ''}"
```

Setting configuration does not restart the robot or replace its running URDF,
controllers, or MoveIt model. Restart the complete robot bringup to load changed
configuration, then check again. The `current_pm_robot_configuration` topic
publishes the launch snapshot, so editing the file does not change that topic.
Real, Unity, and simulation launch files supply the
`loaded_pm_robot_configuration_yaml` parameter. Custom launch files must pass the
same YAML snapshot used to launch their robot; without it the check service fails
because the loaded robot configuration cannot be verified.

After changing these interfaces, rebuild and source the workspace:

```bash
colcon build --packages-select pm_msgs pm_robot_modules pm_robot_primitive_skills pm_robot_bringup
source install/setup.bash
```
