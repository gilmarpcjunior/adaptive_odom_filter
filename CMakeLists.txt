cmake_minimum_required(VERSION 2.8.3)
project(adaptive_odom_filter)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  rtabmap_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES adaptive_filter
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs tf rtabmap_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# add_executable(EKFAdaptiveFilter src/EKFAdaptiveFilter.cpp)
# target_link_libraries(EKFAdaptiveFilter ${catkin_LIBRARIES})

# add_executable(adaptive_covariance src/adaptive_covariance.cpp)
# target_link_libraries(adaptive_covariance ${catkin_LIBRARIES})


