cmake_minimum_required(VERSION 2.8.3)
project(gimbal_gazebo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  gazebo_ros
)
find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gimbal_pointing
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
  DEPENDS
   roscpp
   gazebo_ros
)
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/test_plugin.cc)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

