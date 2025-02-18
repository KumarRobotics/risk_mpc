# FASTER-LIO-ROS2

Migration [Faster-LIO](https://github.com/gaoxiang12/faster-lio) to ROS2 

## Environment
* Ubuntu 20.04 LTS
* ROS galactic version

## Dependency
1. ROS galactic
2. glog: ```sudo apt-get install libgoogle-glog-dev```
3. eigen: ```sudo apt-get install libeigen3-dev```
4. pcl: ```sudo apt-get install libpcl-dev```
5. yaml-cpp: ```sudo apt-get install libyaml-cpp-dev```
6. [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2)
7. [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)

## Build

```
cd ~/<work_space>/src
git clone https://github.com/frankgon1627/Faster-LIO-ROS2.git
cd ~/<work_space>
colcon build --symlink-install
```

## Run
```
source install/local_setup.sh
ros2 launch faster_lio mapping_ouster64.launch.py
```

## Acknowledgments
Thanks to [Faster-LIO](https://github.com/gaoxiang12/faster-lio) for their research