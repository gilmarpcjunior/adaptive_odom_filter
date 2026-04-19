cmake_minimum_required(VERSION 3.8)
project(adaptive_odom_filter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rtabmap_msgs REQUIRED)
find_package(cloud_msgs QUIET)

set(adaptive_odom_filter_dependencies
  rclcpp
  nav_msgs
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  rtabmap_msgs
)

if(cloud_msgs_FOUND)
  list(APPEND adaptive_odom_filter_dependencies cloud_msgs)
endif()

add_library(adaptive_odom_filter SHARED
  src/ekf_adaptive_tools.cpp
  src/adaptive_robust_ekf.cpp
  src/alphaBetaFilter.cpp
)

target_include_directories(adaptive_odom_filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(adaptive_odom_filter ${adaptive_odom_filter_dependencies})

add_executable(EKFAdaptiveFilter src/EKFAdaptiveFilter.cpp)
target_link_libraries(EKFAdaptiveFilter adaptive_odom_filter)
ament_target_dependencies(EKFAdaptiveFilter ${adaptive_odom_filter_dependencies})

add_executable(adaptive_odom_filter_node src/EKFAdaptiveFilter.cpp)
target_link_libraries(adaptive_odom_filter_node adaptive_odom_filter)
ament_target_dependencies(adaptive_odom_filter_node ${adaptive_odom_filter_dependencies})

add_executable(EKFRobustAdaptiveFilter src/EKFRobustAdaptiveFilter.cpp)
target_link_libraries(EKFRobustAdaptiveFilter adaptive_odom_filter)
ament_target_dependencies(EKFRobustAdaptiveFilter ${adaptive_odom_filter_dependencies})

install(PROGRAMS
  scripts/fake_odometry_path.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  adaptive_odom_filter
  EKFAdaptiveFilter
  adaptive_odom_filter_node
  EKFRobustAdaptiveFilter
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${adaptive_odom_filter_dependencies})
ament_export_include_directories(include)

ament_package()
