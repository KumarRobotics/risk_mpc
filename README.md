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
ros2 run safety_controller safety_controller
```

### Notes on how to start the zed and ouster
To launch the camera and LIDAR, use the following commands:

```
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
ros2 launch ouster_ros sensor.composite.launch.py viz:=false
```

### Notes on how to get odometry estimates
To receive odometry, use the following commands:

```
ros2 launch direct_lidar_inertial_odometry dlio.launch.py rviz:=false
```

### Additional relevant launch scripts

To get static and dynamic transforms

```
ros2 run transforms transform_pub
```

To get obstacle detection

```
ros2 launch groundgrid ground_grid.launch.py
ros2 launch obstacle_detection obstacle_detection.launch.py
```

To get the trajectory planning and control framework

```
ros2 launch planners planners.launch.py
ros2 run mpc_controller mpc_llc
```