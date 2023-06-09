cmake_minimum_required(VERSION 3.0.2)
project(models_plugins)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  gazebo_ros 
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros 
)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

# For model_push_plugin plugin
add_library(model_move_plugin src/forward.cc)
target_link_libraries(model_move_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

