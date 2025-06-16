# RB1 ROS2 Description Package

This package provides the complete ROS 2 model and simulation setup for the RB1 mobile base robot developed by Robotnik. It includes all components necessary for visualization, control, and simulation in Gazebo, including drive wheels, sensors, and a lifting unit.

---

## 📦 Installation Instructions

### ✅ Prerequisites

Before installing this package, ensure your system has:

- Ubuntu 22.04
- ROS 2 Galactic or newer
- `gazebo_ros2_control` and `ros2_control` packages

Install ROS-related dependencies (if not already installed):

```bash
sudo apt update
sudo apt install ros-${ROS_DISTRO}-ros2-control                  ros-${ROS_DISTRO}-ros2-controllers                  ros-${ROS_DISTRO}-gazebo-ros2-control
```

### 📥 Clone and Build

```bash
cd ~/ros2_ws/src
git clone <your-repo-url> rb1_ros2_description
cd ~/ros2_ws
colcon build --packages-select rb1_ros2_description
source install/setup.bash
```

---

## 🚀 Getting Started

To start the simulation with the RB1 robot in Gazebo:

```bash
ros2 launch rb1_ros2_description rb1_ros2_xacro.launch.py
```

This will:
- Launch Gazebo
- Spawn the RB1 robot model
- Start the robot_state_publisher and controller manager

---

## 🕹️ Controller Activation

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

---

## ⬆️⬇️ Moving the Robot's Lifting Unit

You can control the lift by publishing a position command:

```bash
ros2 topic pub /rb1_robot/position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.2]"
```

- Replace `0.2` with the desired vertical position (meters)
- Make sure `robot_elevator_platform_joint` is correctly configured in your controller YAML

---

## 📚 Additional Notes

- Always source your workspace before launching:
  ```bash
  source ~/ros2_ws/install/setup.bash
  ```

- View the TF tree:
  ```bash
  ros2 run tf2_tools view_frames
  xdg-open frames.pdf
  ```

---

## 🧑‍💻 Maintainer

**RB1 Simulation Team**  
[https://www.robotnik.eu](https://www.robotnik.eu)  
Feel free to fork, contribute, and file issues.
