cmake_minimum_required(VERSION 2.8.3)
project(pid)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
#  dynamic_reconfigure
)

#generate_dynamic_reconfigure_options(
#  cfg/MyParams.cfg
#)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
#  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake 
)

add_executable(pid 
    src/pid.cpp
    src/node_pid.cpp
    src/myPoint.cpp
)
#add_dependencies(pid ${PROJECT_NAME}_gencfg)
target_link_libraries(pid
   ${catkin_LIBRARIES}
)
