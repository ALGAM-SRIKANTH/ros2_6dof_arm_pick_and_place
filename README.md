# ROS 2 Robot Arm Pick-and-Place System

## Overview
A full-stack ROS 2 robotic manipulator project implementing motion planning,
control, and task-level autonomy using MoveIt 2, ros2_control, and Gazebo.

The system performs collision-aware pick-and-place tasks in a physics-based
simulation using an industry-style robot software architecture.


## Key Concepts Covered
- ROS 2 Jazzy node architecture
- URDF / Xacro robot modeling
- ros2_control integration
- JointTrajectoryController configuration
- MoveIt 2 motion planning
- Planning Scene & Collision Objects
- Gripper control using FollowJointTrajectory
- C++ MoveGroupInterface usage
- Gazebo + RViz2 visualization


## System Architecture

Task Layer (Pick & Place) ->
MoveIt 2 (Planning) ->
ros2_control (Execution) ->
Gazebo (Physics Simulation)


## Demo – ROS 2 Pick & Place

[Pick and Place Demo](demo/pick_place_preview.gif)

[Full demo video](demo/pick_place_demo.mp4)


## Technology Stack
- ROS Version -  ROS 2 Jazzy
- Simulator -  Gazebo
- Motion Planning -  MoveIt 2
- Controllers -  ros2_contro
- Programming Language -  python, C++
- Robot -  Custom 6-DOF Arm + Gripper 


## Robot Description
- 6 revolute joints (position-controlled)
- Parallel gripper with a single actuated finger joint
- Proper joint limits and dynamics
- Transmissions configured for ros2_control
- Full TF tree published


## How to Build and Run
```bash
# Build workspace
colcon build
source install/setup.bash
```
```bash
# Start Gazebo simulation, controllers, Rviz and MoveIt
ros2 launch my_robot_bringup my_robot.launch.xml
```
```bash
# Run pick-and-place task
ros2 launch my_robot_pick_place pick_place.launch.py
```

## Pick-and-Place Logic (C++)
- High-level execution flow:
- Move robot to predefined home pose
- Add table and object to planning scene
- Move arm to pre-grasp pose
- Open gripper
- Approach object and close gripper
- Attach object to end-effector
- Move to target place pose
- Detach object
- Retreat to safe pose


## Tested Environment
- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo
- Rviz2
- MoveIt 2


## Future Improvements
- Vision-based object detection
- Automatic grasp pose generation
- Behavior Tree / State Machine integration
- Multi-object manipulation
- Real robot hardware deployment

## License
This project is licensed under the MIT License.
See the [LICENSE](LICENSE) file for details.

## Author
- Algam Srikanth |
- Mechanical Engineer → Robotics Software Developer
- ROS 2 | MoveIt 2 | Gazebo | C++
