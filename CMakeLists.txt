cmake_minimum_required(VERSION 2.8.3)
project(moveit_boilerplate)

# C++ 11
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_visual_tools
  cmake_modules
  controller_manager_msgs
  ros_param_shortcuts
  dashboard_msgs
)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED)

# Gflags
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Gflags REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    controller_manager_msgs
    ros_param_shortcuts
    dashboard_msgs
  DEPENDS
    Eigen
  INCLUDE_DIRS 
    include
  LIBRARIES
    remote_control
    fix_state_bounds
    execution_interface
    #planning_interface
    trajectory_io
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

# Manipulation Data Library
# add_library(manipulation_data
#    src/manipulation_data.cpp
# )
# target_link_libraries(manipulation_data
#   ${catkin_LIBRARIES} 
#   ${Boost_LIBRARIES}
# )

# remote_control library
add_library(remote_control
  src/remote_control.cpp
)
target_link_libraries(remote_control
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

# execution interface library
add_library(execution_interface
  src/execution_interface.cpp
)
target_link_libraries(execution_interface
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES}
)

# Planning_interface library
# add_library(planning_interface
#   src/planning_interface.cpp
# )
# add_dependencies(planning_interface moveit_boilerplate_generate_messages_cpp)
# target_link_libraries(planning_interface
#   manipulation_data
#   execution_interface
#   remote_control  
#   fix_state_bounds
#   ${catkin_LIBRARIES} 
#   ${Boost_LIBRARIES}
# )

# trajectory input/output
add_library(trajectory_io
  src/trajectory_io.cpp
)
target_link_libraries(trajectory_io
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES}
)

# Reusable class for MoveIt!
add_library(boilerplate
  src/boilerplate.cpp
)
target_link_libraries(boilerplate
  remote_control
  execution_interface
  trajectory_io
  gflags
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES}
)
