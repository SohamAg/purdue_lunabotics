cmake_minimum_required(VERSION 3.0.2)
project(lunabot_perception)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  image_proc
  realsense2_camera
  message_filters
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  laser_geometry 
  tf
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

find_path(YAML_CPP_INCLUDE_DIR NAMES yaml_cpp.h PATHS ${YAML_CPP_INCLUDE_DIRS})
find_library(YAML_CPP_LIBRARY NAMES YAML_CPP PATHS ${YAML_CPP_LIBRARY_DIRS})

link_directories(${YAML_CPP_LIBRARY_DIRS})

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_scan_to_pc_node src/scan_to_pc.cpp)
set_target_properties(${PROJECT_NAME}_scan_to_pc_node PROPERTIES OUTPUT_NAME scan_to_pc PREFIX "")
add_dependencies(${PROJECT_NAME}_scan_to_pc_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_scan_to_pc_node
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_concat src/points_concat.cpp)
set_target_properties(${PROJECT_NAME}_concat PROPERTIES OUTPUT_NAME points_concat PREFIX "")
add_dependencies(${PROJECT_NAME}_concat ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_concat
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)
