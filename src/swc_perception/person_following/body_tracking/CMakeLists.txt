cmake_minimum_required(VERSION 3.5)
project(body_tracking)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# add_compile_definitions(-D_GLIBCXX_USE_CXX11_ABI=0)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
# for msg generation
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED) 
# others 20230828
find_package(Eigen3 REQUIRED)
find_package(ccf_person_identification REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(person_tracking_msgs REQUIRED)


find_package(dlib REQUIRED)
include("/home/jetson/dlib-19.22/dlib/cmake")

ament_export_dependencies(rosidl_default_runtime)

include_directories(
    ${EIGEN3_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/
    ${PROJECT_SOURCE_DIR}/include
    ${CCF_PERSON_IDENTIFICATION_INCLUDE_DIRS}
    ${MESSAGE_FILTERS_INCLUDE_DIRS}
    ${cv_bridge_INCLUDE_DIRS}
    ${AMENT_INDEX_CPP_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    )
    
message(STATUS "EIGEN3_INCLUDE_DIRS: ${EIGEN3_INCLUDE_DIRS}")
message(STATUS "PROJECT_SOURCE_DIR: ${PROJECT_SOURCE_DIR}")
# message(STATUS "VISION_OPENCV_INCLUDE_DIRS: ${vision_opencv_INCLUDE_DIRS}")
message(STATUS "CCF_PERSON_IDENTIFICATION_INCLUDE_DIRS: ${CCF_PERSON_IDENTIFICATION_INCLUDE_DIRS}")
message(STATUS "ccf_person_identification_INCLUDE_DIRS: ${ccf_person_identification_INCLUDE_DIRS}")
message(STATUS "MESSAGE_FILTERS_INCLUDE_DIRS: ${MESSAGE_FILTERS_INCLUDE_DIRS}")
message(STATUS "cv_bridge_INCLUDE_DIRS: ${cv_bridge_INCLUDE_DIRS}")
message(STATUS "cv_bridge_LIBRARY_DIRS: ${CV_BRIDGE_LIBRARY_DIRS}")
message(STATUS "AMENT_INDEX_CPP_INCLUDE_DIRS: ${AMENT_INDEX_CPP_INCLUDE_DIRS}")
# message(STATUS "STD_MSGS_INCLUDE_DIRS: ${std_msgs_INCLUDE_DIRS}")



# 手动设置包含ccf_person_identification功能包的include路径
# include_directories("/home/jetson/Documents/person_tracking_ws/src/ccf_person_identification/include")
include_directories("/home/jetson/Desktop/SmartWCHR/src/swc_perception/person_following/ccf_person_identification/include")



file(GLOB_RECURSE HEADERFILES
  ${PROJECT_SOURCE_DIR}/include/*.h
  )

link_directories(
  )

# link_directories("/opt/tros/lib/")

add_executable(${PROJECT_NAME}
  src/main.cpp
  src/body_tracking.cpp
  src/util.cpp
  src/context.cpp
  #   src/sub_topic.cpp
)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})


# target_link_libraries(${PROJECT_NAME}
#   opencv_world
  # cv_bridge
# )


ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_action
  message_filters
  cv_bridge
  ament_index_cpp
  person_tracking_msgs
  # Eigen3
  )


target_link_libraries(${PROJECT_NAME} dlib::dlib)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

ament_package()
