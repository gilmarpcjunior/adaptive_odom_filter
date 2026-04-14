cmake_minimum_required(VERSION 2.8.3)
project(adaptive_odom_filter)

# Adicionei o Eigen explicitamente, já que você usa muito
find_package(Eigen3 REQUIRED)
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
  cloud_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES adaptive_odom_filter
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs tf cloud_msgs rtabmap_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS} # Garante que o Eigen seja encontrado
)

# 1. Crie a biblioteca com TODO o código compartilhado
add_library(adaptive_odom_filter
    src/ekf_adaptive_tools.cpp
    src/adaptive_robust_ekf.cpp
    src/alphaBetaFilter.cpp
)

target_link_libraries(adaptive_odom_filter
    ${catkin_LIBRARIES}
)

# 2. Executável 1 - Apenas o arquivo do nó
add_executable(EKFAdaptiveFilter src/EKFAdaptiveFilter.cpp)
# Linka com a biblioteca local que criamos acima + catkin
target_link_libraries(EKFAdaptiveFilter 
    adaptive_odom_filter 
    ${catkin_LIBRARIES}
)

# 3. Executável 2 - Apenas o arquivo do nó
add_executable(EKFRobustAdaptiveFilter src/EKFRobustAdaptiveFilter.cpp)
# Linka com a biblioteca local que criamos acima + catkin
target_link_libraries(EKFRobustAdaptiveFilter 
    adaptive_odom_filter 
    ${catkin_LIBRARIES}
)