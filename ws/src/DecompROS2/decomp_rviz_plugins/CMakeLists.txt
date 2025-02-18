################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(decomp_rviz_plugins)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Wno-deprecated-declarations")
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)
find_package(decomp_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core)
find_package(decomp_ros_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rcpputils)
find_package(pluginlib)
find_package(decomp_ros_util REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS} ${DECOMP_UTIL_INCLUDE_DIRS}  ${PCL_INCLUDE_DIRS})

add_definitions(-DQT_NO_KEYWORDS)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
## This setting causes Qt's "MOC" generation to happen automatically.
## this does not moc things in include!!!!!!! only in src
set(CMAKE_AUTOMOC ON)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${rviz_common_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
        ${DECOMP_ROS_UTIL_INCLUDE_DIRS}
)

message(STATUS "decomp_ros_util_INCLUDE_DIRS: ${DECOMP_ROS_UTIL_INCLUDE_DIRS}")

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(decomp_rviz_plugins_headers_to_moc
  include/${PROJECT_NAME}/bound_visual.hpp
  include/${PROJECT_NAME}/ellipsoid_array_display.hpp
  include/${PROJECT_NAME}/mesh_visual.hpp
  include/${PROJECT_NAME}/mesh_shape.hpp
  include/${PROJECT_NAME}/vector_visual.hpp
  include/${PROJECT_NAME}/data_ros_utils.hpp
  include/${PROJECT_NAME}/ellipsoid_array_visual.hpp
  include/${PROJECT_NAME}/polyhedron_array_display.hpp
)

foreach(header "${vision_msgs_rviz_plugins_headers_to_moc}")
  qt5_wrap_cpp(decomp_rviz_plugins_moc_files "${header}")
endforeach()


set(decomp_rviz_plugins_source_files
  src/bound_visual.cpp
  src/mesh_shape.cpp 
  src/mesh_visual.cpp
  src/vector_visual.cpp
  src/ellipsoid_array_visual.cpp
  src/ellipsoid_array_display.cpp
  src/polyhedron_array_display.cpp)

add_library(${PROJECT_NAME} SHARED ${decomp_rviz_plugins_source_files} 
                      ${decomp_rviz_plugins_moc_files})

target_include_directories(${PROJECT_NAME} PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
            ${Qt5Widgets_INCLUDE_DIRS} )

target_link_libraries(${PROJECT_NAME} PUBLIC
            rviz_ogre_vendor::OgreMain
            rviz_ogre_vendor::OgreOverlay
            rviz_common::rviz_common ${Boost_SYSTEM_LIBRARY}  ${PCL_LIBRARIES}
)

target_compile_definitions(${PROJECT_NAME}
                 PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")

ament_target_dependencies(${PROJECT_NAME}
        PUBLIC
        rclcpp
        rclcpp_components
        rviz_common
        rviz_default_plugins
        rviz_ogre_vendor
        rviz_rendering
        decomp_ros_msgs
        decomp_ros_util
        tf2 
        tf2_ros 
        sensor_msgs
        pcl_conversions
        nav_msgs)

## Here we export the plugins and meshes so they can be found by RViz at runtime.
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

ament_export_include_directories("include/${PROJECT_NAME}")

# Export modern CMake targets
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

## Install
ament_export_dependencies(${PROJECT_NAME}
  geometry_msgs
  rclcpp
  rviz_common
  sensor_msgs
  rviz_default_plugins
  nav_msgs
  rviz_rendering
  pcl_conversions
  )

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/icons"
  DESTINATION "share/${PROJECT_NAME}"
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)


ament_package()

