# RB1 ROS2 Description Package

This package provides the complete ROS 2 model and simulation setup for the RB1 mobile base robot developed by Robotnik. It includes all components necessary for visualization, control, and simulation in Gazebo, including drive wheels, sensors, and a lifting unit.

## Installation Instructions

### Prerequisites

Before installing this package, ensure your system has:

- Ubuntu 22.04
- ROS 2 Galactic or newer
- `gazebo_ros2_control` and `ros2_control` packages

Install ROS-related dependencies (if not already installed):

```bash
sudo apt update
sudo apt install ros-${ROS_DISTRO}-ros2-control                  ros-${ROS_DISTRO}-ros2-controllers                  ros-${ROS_DISTRO}-gazebo-ros2-control
```

### Clone and Build

```bash
cd ~/ros2_ws/src
git clone https://github.com/bhmoon713/rb1_ros2_description.git
cd ~/ros2_ws
colcon build --packages-select rb1_ros2_description
source install/setup.bash
```

## Getting Started

To start the simulation with the RB1 robot in Gazebo:

```bash
ros2 launch rb1_ros2_description rb1_ros2_xacro.launch.py

There is considerable delay at Gazebo while it spawns robot, and need to wait it complete for launching ROS2 control nodes

[wait_for_service_node-5] [INFO] [1750051453.582879189] [rclcpp]: service not available, waiting again...
[wait_for_service_node-5] [INFO] [1750051454.583005859] [rclcpp]: service not available, waiting again...



Several launch files
rb1_ros2_xacro_method1.launch.py  : Without delaying logic 
rb1_ros2_xacro_method2.launch.py  : With delay
rb1_ros2_xacro.launch.py  : With delay
```

This will:
- Launch Gazebo
- Spawn the RB1 robot model
- Start the robot_state_publisher and controller manager

---

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




user:~$ ros2 control list_hardware_interfaces
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
user:~$ ros2 control list_controllers
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
rb1_base_controller [diff_drive_controller/DiffDriveController] active
joint_trajectory_controller[joint_trajectory_controller/JointTrajectoryController] active


---

## Moving the Robot's Lifting Unit

You can control the lift by publishing a position command:

```bash
ros2 topic pub --rate 10 /rb1_base_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0, z: 0.0}, angular: {x: 0.0,y: 0.0, z: 0.2}}"
```

 ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: ['robot_elevator_platform_joint'], points: [{positions: [0.05], time_from_start: {sec: 1, nanosec: 0}}]}}"

 ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: ['robot_elevator_platform_joint'], points: [{positions: [0.00], time_from_start: {sec: 1, nanosec: 0}}]}}"


---

## Additional Notes


