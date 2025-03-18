Negative Obstacle Detection
=============
![Docker CI Build](https://github.com/KumarRobotics/dcist_master/actions/workflows/docker-build.yaml/badge.svg?branch=master)

This is the base image used for negative obstacle detection at KR, for the Jackals to run ROS2-Jazzy on Ubuntu-24.04

TL;DR:
 - It installs the dependencies required by many user packages (see
   `risk-mpc/Dockerfile`).
 - It copies the `ws` main dcist workspace and builds it.
 - It provides helper scripts `build.bash`, `run.bash` and `join.bash`

### Architectures
Three architectures are provided:
 - `kumarrobotics/risk-mpc-bare` - **x86_64 CPU**: To run in CPU-only platforms, such as the Intel NUC for the high-altitude quads. Based on `ubuntu` Docker images.
 - `kumarrobotics/risk-mpc-nvda` - **x86_64 CUDA**: To run in x86_64 GPU-accelerated platforms, such as the computers on the Jackals. Based on `nvidia/cuda` Docker images.
 - `kumarrobotics/risk-mpc-nvda` - **arm64 CUDA**: To run in Nvidia Jetson platforms.

### How to build and use?
```
git clone https://github.com/KumarRobotics/risk_mpc.git
cd risk_mpc && git submodule update --init --recursive
./build.bash risk-mpc x86_64_nvda
./run.bash risk-mpc-nvda:latest
```

### Notes on how to start up the Jackal
To launch the base hardware (no sensors), use the following commands:

```
ros2 launch jackal_robot bringup.launch.py
ros2 run twist_stamper twist_stamper --ros-args -r  cmd_vel_in:=/jackal_velocity_controller/cmd_vel_unstamped -r cmd_vel_out:=/jackal_velocity_controller/cmd_vel
```

### Notes on how to start the zed and ouster
To launch the camera and LIDAR, use the following commands:

```
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
ros2 launch ouster_ros sensor.launch.xml sensor_hostname:=192.168.100.12 udp_dest:=192.168.100.1 viz:=false
```

<!-- To use ouster drivers with default QoS (needed for DLIO currently)
```
ros2 launch ouster_ros sensor.launch.xml sensor_hostname:=192.168.100.12 udp_dest:=192.168.100.1 viz:=false use_system_default_qos:=true
``` -->

### Notes on how to get dometry estimates
To receive odometry, use the following commands:

```
ros2 launch direct_lidar_inertial_odometry dlio.launch.py rviz:=false pointcloud_topic:=/ouster/points imu_topic:=/ouster/imu
```

### Additional relevant launch scripts

To get static and dynamic transforms

```
ros2 run transforms transform_pub
```

To get obstacle detection

```
ros2 launch groundgrid ground_grid.launch.py
ros2 run positive_obstacle_detection positive_obstacle_detection
ros2 run negative_obstacle_detection noor_update
```

To get the 2-DOF MPC

```
ros2 launch planners planners.launch.py
ros2 run mpc_controller mpc_llc
```