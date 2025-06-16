# RB1 ROS2 Description Package

This package provides the complete ROS 2 model and simulation setup for the RB1 mobile base robot developed by Robotnik. 
This package is focused on how to generate ROS2 control to move robot and lift the elevator.

## Installation Instructions

### Prerequisites

Before installing this package, ensure your system has:

- Ubuntu 22.04
- ROS 2 Galactic or newer
- `gazebo_ros2_control` and `ros2_control` packages

There are important files you need to create ROS2 control.
config/rb1_controller.yaml : This contains ROS controller managers
launch/rb1_ros2_xacro.launch.py : This file in charge of launching follow items
        gazebo,
        rsp_robot1,
        spawn_robot1,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        position_controller_spawner

### Clone and Build
First you need to visit my GitHub page and download pacakges

bash
cd ~/ros2_ws/src
git clone https://github.com/bhmoon713/rb1_ros2_description.git
cd ~/ros2_ws
colcon build --packages-select rb1_ros2_description
source install/setup.bash


## Getting Started

To start the simulation with the RB1 robot in Gazebo:

```bash
ros2 launch rb1_ros2_description rb1_ros2_xacro.launch.py

There is considerable delay at Gazebo while it spawns robot, and need to wait it complete for launching ROS2 control nodes
This info massage will be shown while waiting for robot spawning completed. Please do not intruppt but just wait. Robot will show up at Gazebo and move to next launches

[wait_for_service_node-5] [INFO] [1750051453.582879189] [rclcpp]: service not available, waiting again...
[wait_for_service_node-5] [INFO] [1750051454.583005859] [rclcpp]: service not available, waiting again...

I have create several launch files for your study purpose.
rb1_ros2_xacro_method1.launch.py  : Without delaying logic 
rb1_ros2_xacro_method2.launch.py  : With delay
rb1_ros2_xacro.launch.py  : With delay



## Controller Activation

If controllers do not start automatically or need to be respawned, use the following commands:

```bash
ros2 run controller_manager spawner joint_state_broadcaster --controller-manager /rb1_robot/controller_manager
ros2 run controller_manager spawner rb1_base_controller --controller-manager /rb1_robot/controller_manager
ros2 run controller_manager spawner position_controller --controller-manager /rb1_robot/controller_manager
```

This will activate:
- Joint state publisher
- Diff drive controller
- Lifting unit position controller

if everything goes well. You can check by follow commands
ros2 control list_hardware_interfaces
command interfaces
        robot_elevator_platform_joint/position [claimed]
        robot_left_wheel_joint/velocity [claimed]
        robot_right_wheel_joint/velocity [claimed]
state interfaces
         robot_elevator_platform_joint/effort
         robot_elevator_platform_joint/position
         robot_elevator_platform_joint/velocity
         robot_left_wheel_joint/position
         robot_left_wheel_joint/velocity
         robot_right_wheel_joint/position
         robot_right_wheel_joint/velocity
ros2 control list_controllers
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
rb1_base_controller [diff_drive_controller/DiffDriveController] active
joint_trajectory_controller[joint_trajectory_controller/JointTrajectoryController] active


## You can control the robot move by a velocity command:

```bash
ros2 topic pub --rate 10 /rb1_base_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0, z: 0.0}, angular: {x: 0.0,y: 0.0, z: 0.2}}"
```
You can control the lift up and down by publishing a position command:
```bash
 ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: ['robot_elevator_platform_joint'], points: [{positions: [0.05], time_from_start: {sec: 1, nanosec: 0}}]}}"
```
```bash
 ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: ['robot_elevator_platform_joint'], points: [{positions: [0.00], time_from_start: {sec: 1, nanosec: 0}}]}}"
```

## Additional Notes
you can visualize robot in Rviz with following command
rviz2 -d ~/ros2_ws/src/rb1_ros2_description/rviz/setup.rviz

