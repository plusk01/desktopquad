cmake_minimum_required(VERSION 2.8.3)
project(desktopquad_sim)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  tf
)

# Depend on system install of Gazebo
find_package(
  gazebo REQUIRED
)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

catkin_package(
  DEPENDS
    roscpp
    gazebo_ros
)

## Build

add_executable(tf_sim_frames src/tf_sim_frames.cpp)
target_link_libraries(tf_sim_frames ${catkin_LIBRARIES})