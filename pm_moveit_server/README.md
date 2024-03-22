# pm_moveit_server

## 1. Overview
Specifically, the packages have been designed to be used for precision assembly tasks.
The repository contains the following packages:
* `pm_moveit_server`: This package can be used to interact with the move_groups (see Moveit2 doc for more information) that are launched from the pm_robot_bringup package (see match_pm_robot repository). The pm_moveit_server provides multiple services to control the pm_robot using the inverse kinematics. A more detailed description can be found in the package.readme.
* `pm_moveit_interfaces`: Contains the message and service definitions used by the pm_moveit_server.

## 2. Installation 
To run all the packages in this repository, the installation of ROS2 (tested on Humble) is mandatory! 

* Make sure you have the following packages installed (only mandatory to run the pm_moveit_server)!
```
sudo apt install ros-humble-moveit
```
```
sudo apt install ros-humble-moveit-visual-tools
```
```
sudo apt install ros-humble-graph-msgs
```

The purpose of the pm_moveit_server is to offer services to move pre-defined end-effectors to a given frame (TF). In order to do so, the moviet_server offers different services:

* `MoveCam1TcpTo.srv`: Commands the control, to move the frame "Cam1_Toolhead_TCP" to the given frame.
* `MoveLaserTcpTo.srv`: Commands the control, to move the frame "Laser_Toolhead_TCP" to the given frame.
* `MoveToolTcpTo.srv`: Commands the control, to move the frame "PM_Robot_Tool_TCP" to the given frame.
* `ExecutePlan.srv`: Currently not supported. Will later be used to execute a planed motion, if planing has been done in a seperate step.

Currently a change of endeffectors is not implemented (but desired in the future). All service calls (for movements) have the same messages as in and output:

Inputs:
* `frame_name`: Name of the frame, the end-effector should move to. If specified the "move_to_pose" is ignored. If left empty, target pose is the "move_to_pose".
* `move_to_pose`: Pose, the end-effector should move to in world coordinates. If a "frame_name" is given, the "move_to_pose" is ignored. If "frame_name" is empty and "move_to_pose" position is (0,0,0) the target frame will become the current end-effector pose. Using the following "translation" and "rotation" inputs, relative robot motion is thus possible. This also means that the world origin (0,0,0) can never be an input for the target pose (movement to that pose is not possible anyway).
* `translation`: Additional translation that is added to the target frame specified by "frame_name" or "move_to_pose" (in target coordinate system). 
* `rotation`: Additional rotation that is added to the target frame specified by "frame_name" or "move_to_pose" (in target coordinate system). 
* `exec_wait_for_user_input`: This is currently not implmented. If later set to true, rviz will ask you to click a button before a movement will be executed.
* `execute`: Movement will only be executed if set to true. If set to false, movement is only planed. Thus it is possible to check if a movement is possible.

Outputs:
* `success`: Returns true if a movement could be planned successfully.
* `joint_names`: Returns the joint names involved in the movement.
* `joint_values`: Returns the targed joint values of the planned pose. These are also returned when motion is not executed. 



To-Do's:
* Rotations are currently not supported. Regardless what is inserted into rotation and pose.orientation the target pose will always have x: 0, y: 0, z: 0, w: 1.
* Vizualization in Rviz
* Wait for user input in Rviz before execution
* Change planning algorithms to change planning accuracy, planners, and constraints
