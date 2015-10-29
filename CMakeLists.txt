cmake_minimum_required(VERSION 2.8.3)
project(moveit_manipulation)

# C++ 11
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_grasps
  moveit_visual_tools
  cmake_modules
  controller_manager_msgs
  ros_param_utilities
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
    moveit_grasps
    moveit_visual_tools
    controller_manager_msgs
    ros_param_utilities
    dashboard_msgs
  DEPENDS
    Eigen
  INCLUDE_DIRS 
    include
  LIBRARIES
    fix_state_bounds
    manipulation    
    moveit_boilerplate
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
add_library(manipulation_data
  src/manipulation_data.cpp
)
target_link_libraries(manipulation_data
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES}
)

# Remote_control library
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

# Manipulation pipeline library
add_library(manipulation
  src/manipulation.cpp
)
add_dependencies(manipulation moveit_manipulation_generate_messages_cpp)
target_link_libraries(manipulation
  manipulation_data
  execution_interface
  remote_control  
  fix_state_bounds
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES}
)

# trajectory input/output
add_library(trajectory_io
  src/trajectory_io.cpp
)
target_link_libraries(trajectory_io
  manipulation
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES}
)

# Reusable class for manipulation
add_library(moveit_boilerplate
  src/moveit_boilerplate.cpp
)
target_link_libraries(moveit_boilerplate
  trajectory_io
  manipulation
  gflags
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES}
)

# Teleop-specific functionality
add_library(teleoperation
  src/behaviors/teleoperation.cpp
)
target_link_libraries(teleoperation  
  moveit_boilerplate
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES}
)

# Teleop-specific functionality
add_library(arm_testing
  src/behaviors/arm_testing.cpp
)
target_link_libraries(arm_testing  
  moveit_boilerplate
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES}
)

## Declare a C++ executable
add_executable(teleoperation_main src/behaviors/teleoperation_main.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(teleoperation_main
  teleoperation
  gflags
  ${catkin_LIBRARIES}
)
