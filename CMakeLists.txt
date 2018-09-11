cmake_minimum_required(VERSION 2.8.3)
project(TeliCam_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
  tf
  tf_conversions
)

add_definitions(-std=c++11 -O2 -g -DNDEBUG)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

add_executable(cam_driver src/cam_driver.cpp)
target_link_libraries(cam_driver
${catkin_LIBRARIES}
${libpointmatcher_LIBRARIES}
)


