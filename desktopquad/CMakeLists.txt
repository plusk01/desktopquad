cmake_minimum_required(VERSION 2.8.3)
project(desktopquad)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  cmake_modules
  sensor_msgs
  roscpp
  rospy
  eigen_conversions
  std_msgs
  fcu_common
  dynamic_reconfigure
)
find_package(Eigen REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/Controller.cfg
)

add_service_files(
  FILES
  AddWaypoint.srv
  RemoveWaypoint.srv
  SetWaypointsFromFile.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES desktopquad
  CATKIN_DEPENDS roscpp rospy nav_msgs std_msgs
)

###########
## Build ##
###########
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare cpp libraries
add_library(desktopquad include/lib/eigen.h)
add_dependencies(desktopquad fcu_common_generate_messages_cpp)
target_link_libraries(desktopquad ${catkin_LIBRARIES})
set_target_properties(desktopquad PROPERTIES LINKER_LANGUAGE CXX)

## Declare cpp executables
# PID SLC controller
add_executable(controller
  src/controller/controller_node.cpp
  src/controller/controller.cpp
  include/controller/controller.h)
add_dependencies(controller fcu_common_generate_messages_cpp desktopquad_gencfg)
target_link_libraries(controller ${catkin_LIBRARIES})

# differential flatness controller
add_executable(diff_flat
  src/diff_flat/node.cpp
  src/diff_flat/controller.cpp
  include/diff_flat/controller.h)
add_dependencies(diff_flat fcu_common_generate_messages_cpp desktopquad_gencfg)
target_link_libraries(diff_flat ${catkin_LIBRARIES})