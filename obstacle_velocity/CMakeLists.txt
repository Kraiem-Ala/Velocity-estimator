cmake_minimum_required(VERSION 3.0.2)
project(obstacle_velocity)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs 
  visualization_msgs
  teb_local_planner
  tf2 
  roscpp
  message_generation

)
## Messages_Files
add_message_files(
   FILES
   obstacle.msg
   obstacles_msgs.msg
 )
generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   
 )
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp geometry_msgs visualization_msgs teb_local_planner tf2 
#  DEPENDS other non-ROS libs
)

include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${GSTREAMER_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/obstacle_velocity.cpp
)
add_executable(obstacles src/obstacle_velocity.cpp)
add_executable(filter src/Data_filter.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(obstacles ${catkin_LIBRARIES})
target_link_libraries(filter ${catkin_LIBRARIES})





install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
