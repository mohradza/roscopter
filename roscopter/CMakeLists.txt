cmake_minimum_required(VERSION 2.8.3)
project(roscopter)

set(CMAKE_CXX_STANDARD 11)
if (NOT CMAKE_BUILD_TYPE)
    # Options: Debug, Release, MinSizeRel, RelWithDebInfo
    message(STATUS "No build type selected, default to Release")
    set(CMAKE_BUILD_TYPE "Release")
endif()

set(CMAKE_CXX_FLAGS "-fopenmp")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  dynamic_reconfigure
  eigen_conversions
  rosflight_msgs
  rosflight_utils
  std_msgs
  nav_msgs
  sensor_msgs
  roslib
  rosbag
)

find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/Controller.cfg
)

catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES roscopter_eigen
  CATKIN_DEPENDS roscpp rospy dynamic_reconfigure eigen_conversions rosflight_msgs rosflight_utils std_msgs nav_msgs sensor_msgs
  DEPENDS EIGEN3
)

###########
## Build ##
###########

include_directories(include ${EIGEN3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

## Declare cpp libraries
add_library(roscopter_eigen include/lib/eigen.h)
set_target_properties(roscopter_eigen PROPERTIES LINKER_LANGUAGE CXX)
add_dependencies(roscopter_eigen ${catkin_EXPORTED_TARGETS})
target_link_libraries(roscopter_eigen ${catkin_LIBRARIES})


##########################################################
## Controller
add_executable(controller
  src/controller/controller_node.cpp
  src/controller/controller.cpp
  include/controller/controller.h)
add_dependencies(controller ${catkin_EXPORTED_TARGETS})
target_link_libraries(controller ${catkin_LIBRARIES})

##########################################################
## EKF Library - main library code
add_library(ekf
  src/ekf/ekf.cpp
  include/ekf/ekf.h
  include/ekf/math_helper.h
  include/ekf/quat.h
)
## ROS EKF Library - A ROS-wrapper for the EKF
add_library(ekf_ros src/ekf/ekf_ros.cpp include/ekf/ekf_ros.h)
target_link_libraries(ekf_ros ${catkin_LIBRARIES} ekf)

## EKF Test - Used to check Jacobians and other Functionality
add_executable(ekf_test src/ekf/test.cpp)
target_link_libraries(ekf_test ${GTEST_LIBRARIES} pthread ekf)

## ROS node - A simple node wrapper for the ROS EKF library
add_executable(ekf_node  src/ekf/ekf_node.cpp )
target_link_libraries(ekf_node ${catkin_LIBRARIES} ekf ekf_ros)

## ROSbag parser node - used to post-process a ROS bag
add_executable(ekf_rosbag src/ekf/ekf_rosbag.cpp)
target_link_libraries(ekf_rosbag ${catkin_LIBRARIES} ekf mocap_ekf_ros)
