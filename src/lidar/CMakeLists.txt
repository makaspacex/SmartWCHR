cmake_minimum_required(VERSION 3.5)
project(lidar)
# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#=======================================
# Compile setup (CATKIN, COLCON)
#=======================================

set(COMPILE_METHOD COLCON)

set(CMAKE_BUILD_TYPE RELEASE)

add_definitions(-O3)

#ROS2#
find_package(rclcpp QUIET)
if(rclcpp_FOUND AND ${COMPILE_METHOD} STREQUAL "COLCON")
  # message(=============================================================)
  # message("-- ROS2 Found. ROS2 Support is turned On!")
  # message(=============================================================)
  
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/package.xml ${CMAKE_CURRENT_SOURCE_DIR}/package.xml COPYONLY)
  add_definitions(-DROS2_FOUND)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/package.xml ${CMAKE_CURRENT_SOURCE_DIR}/package.xml COPYONLY)
  # find dependencies
  find_package(ament_cmake REQUIRED)
  #find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED) 
  include_directories(${rclcpp_INCLUDE_DIRS})

endif(rclcpp_FOUND AND ${COMPILE_METHOD} STREQUAL "COLCON")
# Bin and Install
set(ORADAR_SDK_DIR "sdk")
add_subdirectory(${ORADAR_SDK_DIR})

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${ORADAR_SDK_DIR}
)

add_executable(scan  ${CMAKE_CURRENT_SOURCE_DIR}/src/scan_node.cpp)

#Ros2#
if(rclcpp_FOUND AND ${COMPILE_METHOD} STREQUAL "COLCON")
  ament_target_dependencies(scan rclcpp sensor_msgs)
  target_link_libraries(scan oradar_sdk pthread)
  
  install(TARGETS scan
  DESTINATION lib/${PROJECT_NAME}
  )

  install(DIRECTORY launch rviz2
  DESTINATION share/${PROJECT_NAME}/
  )
  
  # install(
  #   DIRECTORY config launch rviz
  #   DESTINATION share/${PROJECT_NAME}
  # )  
  ament_package()

endif(rclcpp_FOUND AND ${COMPILE_METHOD} STREQUAL "COLCON")
