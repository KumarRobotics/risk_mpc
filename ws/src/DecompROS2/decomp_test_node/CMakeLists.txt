################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(decomp_test_node)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(decomp_util REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)


find_package(rosbag2_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(decomp_ros_msgs REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(pcl_conversions REQUIRED)

set(decomp_rviz_plugins_FOUND 1 )
find_package(decomp_rviz_plugins REQUIRED)

################################################################################
# Build
################################################################################
include_directories(
    include 
    ${EIGEN3_INCLUDE_DIRS} 
    ${DECOMP_UTIL_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS})

set(DEPENDENCIES
  "rclcpp"
  "rosbag2_cpp"
  "nav_msgs"
  "decomp_ros_msgs"
  "sensor_msgs"
  "rcutils"
  "std_msgs"
  "tf2_geometry_msgs"
  "rcl_interfaces"
  "pcl_conversions"
  "decomp_rviz_plugins"
)

# add_executable(bag_reader src/bag_reader.cpp)
# ament_target_dependencies(bag_reader ${DEPENDENCIES})

# add_executable(test_path_decomp_2d src/test_path_decomp_2d.cpp)
# ament_target_dependencies(test_path_decomp_2d ${DEPENDENCIES})

add_executable(test_path_decomp_3d src/test_path_decomp_3d.cpp)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
target_link_libraries(test_path_decomp_3d ${Boost_SYSTEM_LIBRARY} 
      ${PCL_LIBRARIES}
)
ament_target_dependencies(test_path_decomp_3d ${DEPENDENCIES})


install(
  DIRECTORY launch config 
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
    test_path_decomp_3d
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
