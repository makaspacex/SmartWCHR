cmake_minimum_required(VERSION 3.5)
project(ccf_person_identification)

# Set C++ standard to C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

# Find dependencies
# set(OpenCV_DIR /usr/local/share/OpenCV)
# set(Flann_DIR /usr/include/flann)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Flann REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ament_index_cpp REQUIRED)


find_package(dlib REQUIRED)

include($ENV{DLIB_ROOT}/dlib/cmake)

# include_directories($ENV{DLIB_ROOT})
# link_directories($ENV{DLIB_ROOT}/build)

# set(CMAKE_PREFIX_PATH "/home/gawain/root/dlib-19.22" ${CMAKE_PREFIX_PATH})
# set(dlib_DIR /home/gawain/root/dlib-19.22/dlib)
# set(dlib_DIR "$ENV{DLIB_ROOT}/dlib")
# find_package(dlib REQUIRED)
# include_directories(${dlib_INCLUDE_DIRS})

# include_directories(${dlib_INCLUDE_DIRS})
# include_directories(${ament_index_cpp_INCLUDE_DIRS})
# Add include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${FLANN_INCLUDE_DIRS}
  ${ament_index_cpp_INCLUDE_DIRS}
)

# Create the library target
add_library(${PROJECT_NAME} SHARED
  src/ccf_person_identification.cpp
)


ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  cv_bridge
  Eigen3
  OpenCV
  ament_index_cpp
  Flann
  # dlib
)

target_link_libraries(${PROJECT_NAME} dlib::dlib)


target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_definitions(${PROJECT_NAME}
  PRIVATE "CCF_PERSON_IDENTIFICATION_BUILDING_DLL"
)

# Create the executable target
add_executable(${PROJECT_NAME}_test
  src/ccf_person_identification_test.cpp
)
ament_target_dependencies(${PROJECT_NAME}_test
  rclcpp
  cv_bridge
)
target_link_libraries(${PROJECT_NAME}_test
  ${PROJECT_NAME}
)

# Install library and executables
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS ${PROJECT_NAME}_test
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

# 将 data 目录复制到 build 目录
file(COPY data DESTINATION ${CMAKE_BINARY_DIR}/${PROJECT_NAME})

# 安装 data 目录到 share/${PROJECT_NAME}
install(DIRECTORY ${CMAKE_BINARY_DIR}/${PROJECT_NAME}/data
  DESTINATION share/${PROJECT_NAME}
  PATTERN "CMakeFiles" EXCLUDE)

# Install launch files
# install(DIRECTORY launch/
#   DESTINATION share/${PROJECT_NAME}/launch
# )

# Install configuration files
ament_package()
