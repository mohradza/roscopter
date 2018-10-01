cmake_minimum_required(VERSION 2.8.3)
project(roscopter)

set(CMAKE_CXX_STANDARD 17)
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
  rosbag
  image_transport
  cv_bridge
)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Eigen3 REQUIRED)
find_package(GTest REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

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

set(VI_EKF_DIR lib/vi_ekf)

include_directories(include 
  lib/vi_ekf/include
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${GTEST_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

## Declare cpp libraries
add_library(roscopter_eigen include/lib/eigen.h)
set_target_properties(roscopter_eigen PROPERTIES LINKER_LANGUAGE CXX)
add_dependencies(roscopter_eigen ${catkin_EXPORTED_TARGETS})
target_link_libraries(roscopter_eigen ${catkin_LIBRARIES})


## Declare cpp executables
add_executable(controller
  src/controller/controller_node.cpp
  src/controller/controller.cpp
  include/controller/controller.h)
add_dependencies(controller ${catkin_EXPORTED_TARGETS})
target_link_libraries(controller ${catkin_LIBRARIES})

add_library(math_helper
        ${VI_EKF_DIR}/src/math_helper.cpp
        ${VI_EKF_DIR}/include/math_helper.h
        ${VI_EKF_DIR}/include/quat.h
        ${VI_EKF_DIR}/include/xform.h
)

add_library(vi_ekf
  ${VI_EKF_DIR}/src/vi_ekf/vi_ekf.cpp
  ${VI_EKF_DIR}/src/vi_ekf/vi_ekf_helper.cpp
  ${VI_EKF_DIR}/src/vi_ekf/vi_ekf_feat.cpp
  ${VI_EKF_DIR}/src/vi_ekf/vi_ekf_log.cpp
  ${VI_EKF_DIR}/src/vi_ekf/vi_ekf_meas.cpp
  ${VI_EKF_DIR}/src/vi_ekf/vi_ekf_error.cpp
  ${VI_EKF_DIR}/src/vi_ekf/vi_ekf_kfr.cpp
  ${VI_EKF_DIR}/src/vi_ekf/vi_ekf_dyn.cpp
  ${VI_EKF_DIR}/include/vi_ekf.h
)
target_link_libraries(vi_ekf math_helper ${YAML_CPP_LIBRARIES})

add_library(klt_tracker
  ${VI_EKF_DIR}/src/klt_tracker.cpp
  ${VI_EKF_DIR}/include/klt_tracker.h
)

add_library(vi_ekf_ros
  ${VI_EKF_DIR}/src/vi_ekf_ros.cpp
  ${VI_EKF_DIR}/include/vi_ekf_ros.h
  ${VI_EKF_DIR}/include/eigen_helpers.h
)
target_link_libraries(vi_ekf_ros
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  vi_ekf
  klt_tracker
)

add_executable(vi_ekf_node ${VI_EKF_DIR}/src/vi_ekf_node.cpp)
target_link_libraries(vi_ekf_node ${catkin_LIBRARIES} vi_ekf_ros)

add_executable(vi_ekf_rosbag ${VI_EKF_DIR}/src/rosbag_parser.cpp)
target_link_libraries(vi_ekf_rosbag ${catkin_LIBRARIES} vi_ekf_ros)

