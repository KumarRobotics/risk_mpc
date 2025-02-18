cmake_minimum_required(VERSION 3.8)
project(faster_lio)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions -g -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS}")
set(CMAKE_BUILD_TYPE "Release")

option(WITH_IVOX_NODE_TYPE_PHC "Use PHC instead of default ivox node" OFF)
if (WITH_IVOX_NODE_TYPE_PHC)
    message("USING_IVOX_NODE_TYPE_PHC")
    add_definitions(-DIVOX_NODE_TYPE_PHC)
else ()
    message("USING_IVOX_NODE_TYPE_DEFAULT")
endif()

set(Eigen3_DIR "/usr/lib/cmake/eigen3")

###  find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Pose6D.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Glog REQUIRED)
include_directories(${Glog_INCLUDE_DIRS})
ament_export_include_directories(${Glog_INCLUDE_DIRS})


set(dependencies
geometry_msgs 
nav_msgs 
rclcpp
rclpy 
std_msgs 
tf2
tf2_ros
tf2_geometry_msgs
pcl_conversions
EIGEN3
PCL
pcl_ros
livox_ros_driver2

)


option(WITH_IVOX_NODE_TYPE_PHC "Use PHC instead of default ivox node" OFF)

if (WITH_IVOX_NODE_TYPE_PHC)
    message("USING_IVOX_NODE_TYPE_PHC")
    add_definitions(-DIVOX_NODE_TYPE_PHC)
else ()
    message("USING_IVOX_NODE_TYPE_DEFAULT")
endif()


include_directories(
        ${EIGEN3_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
        ${PYTHON_INCLUDE_DIRS}
        ${yaml-cpp_INCLUDE_DIRS}
        include

)
ament_export_include_directories(        
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  ${yaml-cpp_INCLUDE_DIRS}
  include
)

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

add_library(${PROJECT_NAME}_lib SHARED
        src/laser_mapping.cc
        src/pointcloud_preprocess.cc
        src/options.cc
        src/utils.cc
        )


target_link_libraries(${PROJECT_NAME}_lib
        ${PCL_LIBRARIES}
        ${PYTHON_LIBRARIES}
        tbb
        glog
        yaml-cpp
        )

ament_target_dependencies(${PROJECT_NAME}_lib
        ${dependencies}
        )

        
rosidl_target_interfaces(${PROJECT_NAME}_lib
${PROJECT_NAME}
"rosidl_typesupport_cpp"
)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
)
target_include_directories(${PROJECT_NAME}_lib PRIVATE ${PYTHON_INCLUDE_DIRS})

    
install(
  TARGETS ${PROJECT_NAME}_lib 
  EXPORT ${PROJECT_NAME}_lib_export
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  DESTINATION lib/${PROJECT_NAME}
)


# add_subdirectory(app)
add_executable(run_mapping_online app/run_mapping_online.cc)
target_link_libraries(run_mapping_online
        ${PROJECT_NAME}_lib gflags
        )

install(TARGETS
        run_mapping_online
        # run_mapping_offline
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  rviz_cfg
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
