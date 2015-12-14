cmake_minimum_required(VERSION 2.8.3)
project(moveit_boilerplate)

# C++ 11
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_visual_tools
  cmake_modules
  controller_manager_msgs
  rosparam_shortcuts
  dashboard_msgs
)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)
find_package(Gflags REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    controller_manager_msgs
    rosparam_shortcuts
    dashboard_msgs
  DEPENDS
    Eigen
  INCLUDE_DIRS
    include
  LIBRARIES
    debug_interface
    fix_state_bounds
    execution_interface
    #planning_interface
    trajectory_io
    ${PROJECT_NAME}_moveit_base
    boilerplate
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Gflags_INCLUDE_DIRS}
)

include_directories(SYSTEM
  ${Boost_INCLUDE_DIR}
  ${EIGEN_INCLUDE_DIRS}
)

# debug_interface library
add_library(debug_interface
  src/debug_interface.cpp
)
target_link_libraries(debug_interface
  ${Gflags_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Fix_state_bounds library
add_library(fix_state_bounds
  src/fix_state_bounds.cpp
)
target_link_libraries(fix_state_bounds
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# execution interfaceX library
add_library(execution_interface
  src/execution_interface.cpp
)
target_link_libraries(execution_interface
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Joint-space path planners
add_library(planning_interface
  src/planning_interface.cpp
)
target_link_libraries(planning_interface
  execution_interface
  debug_interface
  fix_state_bounds
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Trajectory input/output
add_library(trajectory_io
  src/trajectory_io.cpp
)
target_link_libraries(trajectory_io
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Simplified reusable class for MoveIt!
add_library(${PROJECT_NAME}_moveit_base
  src/moveit_base.cpp
)
target_link_libraries(${PROJECT_NAME}_moveit_base
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Reusable class for MoveIt!
# TODO(davetcoleman) depend on ${PROJECT_NAME}_moveit_base
add_library(boilerplate
  src/boilerplate.cpp
)
target_link_libraries(boilerplate
  debug_interface
  execution_interface
  planning_interface
  ${Gflags_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(${PROJECT_NAME}_example src/example.cpp)
target_link_libraries(${PROJECT_NAME}_example
  ${PROJECT_NAME}_moveit_base
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)