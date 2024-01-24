cmake_minimum_required(VERSION 3.0.2)
project(mrflocalization_detector)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries



find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  cv_bridge
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

find_package(OpenCV REQUIRED)


catkin_package()


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  "${PROJECT_SOURCE_DIR}/include"
)


add_executable(mrf_detector src/node.cpp src/mrf.cpp)
target_link_libraries(mrf_detector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


install(
  TARGETS
    mrf_detector
  RUNTIME DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)