Negative Obstacle Detection
=============
![Docker CI Build](https://github.com/KumarRobotics/dcist_master/actions/workflows/docker-build.yaml/badge.svg?branch=master)

This is the base image used for negative obstacle detection at KR, for the Jackals to run ROS2-Jazzy on Ubuntu-24.04

TL;DR:
 - It installs the dependencies required by many user packages (see
   `negative-obstacle-detection/Dockerfile`).
 - It copies the `ws` main dcist workspace and builds it.
 - It provides helper scripts `build.bash`, `run.bash` and `join.bash`

### Architectures
Three architectures are provided:
 - `kumarrobotics/negative-obstacle-detection-bare` - **x86_64 CPU**: To run in CPU-only platforms, such as the Intel NUC for the high-altitude quads. Based on `ubuntu` Docker images.
 - `kumarrobotics/negative-obstacle-detection-nvda` - **x86_64 CUDA**: To run in x86_64 GPU-accelerated platforms, such as the computers on the Jackals. Based on `nvidia/cuda` Docker images.
 - `kumarrobotics/negative-obstacle-detection-nvda` - **arm64 CUDA**: To run in Nvidia Jetson platforms.

### How to build and use?
```
git clone https://github.com/KumarRobotics/negative_obstacle_detection.git
cd negative_obstacle_detection && git submodule update --init --recursive
./build.bash negative-obstacle-detection x86_64_nvda
./run.bash negative-obstacle-detection-nvda:latest
```

### Notes on how to start up the Jackal
To launch the base hardware (no sensors), use the following commands:

```
ros2 launch jackal_robot bringup.launch.py
ros2 run twist_stamper twist_stamper --ros-args -r  cmd_vel_in:=/jackal_velocity_controller/cmd_vel_unstamped -r cmd_vel_out:=/jackal_velocity_controller/cmd_vel
```

### Notes on how to start the realsense and ouster
To launch the camera and LIDAR, use the following commands:

```
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
ros2 launch ouster_ros sensor.launch.xml sensor_hostname:=192.168.100.12 udp_dest:=192.168.100.1 viz:=false
```

To use ouster drivers with default QoS (needed for DLIO currently)
```
ros2 launch ouster_ros sensor.launch.xml sensor_hostname:=192.168.100.12 udp_dest:=192.168.100.1 viz:=false use_system_default_qos:=true
```