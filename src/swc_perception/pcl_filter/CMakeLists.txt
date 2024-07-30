cmake_minimum_required(VERSION 3.8)
project(pcl_filter)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)


set(dependencise
  rclcpp
  sensor_msgs
  PCL
  pcl_conversions
  tf2_ros
)

include_directories(include)

add_executable(${PROJECT_NAME}_node src/main.cpp src/pointcloud_node.cpp)
ament_target_dependencies(${PROJECT_NAME}_node ${dependencise})
target_link_libraries(${PROJECT_NAME}_node ${PCL_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
