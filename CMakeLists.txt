cmake_minimum_required(VERSION 2.8.3)
project(step_ctrl)

find_package( Gazebo REQUIRED)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  geometry_msgs
  roscpp
  roslib
	shared_control_msgs
)

include_directories(
 ${GAZEBO_INCLUDE_DIRS}
 ${GAZEBO_PROTO_INCLUDE_DIRS}
 ${SDF_INCLUDE_DIRS}
)


catkin_package(
  DEPENDS 
  roscpp 
	gazebo_ros 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(step_ctrl src/step_ctrl.cpp)
target_link_libraries(step_ctrl ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})




