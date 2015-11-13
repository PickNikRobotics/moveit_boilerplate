/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, PickNik LLC
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the PickNik LLC nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman <dave@dav.ee>
   Desc:   Interface between MoveIt! execution tools and MoveItManipulation
*/

#ifndef MOVEIT_BOILERPLATE__EXECUTION_INTERFACE_
#define MOVEIT_BOILERPLATE__EXECUTION_INTERFACE_

// ROS
#include <ros/ros.h>
#include <cartesian_msgs/CartesianCommand.h>

// Visual tools
#include <moveit_visual_tools/moveit_visual_tools.h>

// MoveItManipulation
#include <moveit_boilerplate/namespaces.h>
#include <moveit_boilerplate/debug_interface.h>

// MoveIt
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

namespace trajectory_execution_manager
{
MOVEIT_CLASS_FORWARD(TrajectoryExecutionManager);
}

namespace moveit_boilerplate
{
MOVEIT_CLASS_FORWARD(ExecutionInterface);

enum CommandMode { 
  JOINT_EXECUTION_MANAGER, // use the default MoveIt! method for sending trajectories using actionlib
  JOINT_PUBLISHER,         // send trajectories direct to ros_control using ROS messages
  CARTESIAN_PUBLISHER      // send cartesian poses direclty to your controller using ROS messages
};

class ExecutionInterface
{
public:
  /**
   * \brief Constructor
   */
  ExecutionInterface(CommandMode mode, DebugInterfacePtr debug_interface,
                     psm::PlanningSceneMonitorPtr planning_scene_monitor);

  /**
   * \brief Execute a desired cartesian end effector pose
   * \param pose
   * \return true on success
   */
  bool executePose(const Eigen::Affine3d &pose, JointModelGroup *arm_jmg,
                   const double &duration = 0.1);

  /**
   * \brief Do a bunch of checks and send to low level controllers
   * \return true on success
   */
  bool executeTrajectory(moveit_msgs::RobotTrajectory &trajectory_msg, JointModelGroup *jmg,
                         bool wait_for_execution = true);

  /**
   * \brief Wait for trajectory to finish being executed
   * \return true on success
   */
  bool waitForExecution();

  /** \brief Pass through accessor function */
  trajectory_execution_manager::TrajectoryExecutionManagerPtr getTrajectoryExecutionManager()
  {
    return trajectory_execution_manager_;
  }

private:

  /**
   * \brief Ensure that execution manager has been loaded
   * \return true on success
   */
  bool checkExecutionManager();

  /**
   * \brief Get the current state of the robot
   */
  moveit::core::RobotStatePtr getCurrentState();

  /** \brief Debug tools for visualizing in Rviz */
  void loadVisualTools();

  /** \brief Check for potential errors in the trajectory been sent */
  void checkForWaypointJumps(const trajectory_msgs::JointTrajectory &trajectory);

  /** \brief Check if correct controllers are loaded */
  bool checkTrajectoryController(ros::ServiceClient &service_client,
                                 const std::string &hardware_name, bool has_ee = false);

  /** \brief Save a trajectory that is about to be executed to file, for later debugging */
  bool saveTrajectory(const moveit_msgs::RobotTrajectory &trajectory_msg,
                      const std::string &file_name);

  // A shared node handle
  ros::NodeHandle nh_;

  // Configuration settings
  CommandMode mode_; // how to publish
  bool save_traj_to_file_ = false;
  std::string save_traj_to_file_path_;
  bool visualize_trajectory_line_ = false;
  bool visualize_trajectory_path_ = false;
  bool check_for_waypoint_jumps_ = false;

  std::size_t trajectory_filename_count_ = 0; // iterate file names

  DebugInterfacePtr debug_interface_;
  mvt::MoveItVisualToolsPtr visual_tools_;
 
  // Track collision objects in the environment
  psm::PlanningSceneMonitorPtr planning_scene_monitor_;

  // Allocated memory for robot state
  moveit::core::RobotStatePtr current_state_;

  // Trajectory execution
  trajectory_execution_manager::TrajectoryExecutionManagerPtr trajectory_execution_manager_;

  // Alternative method to sending trajectories than trajectory_execution_manager
  ros::Publisher joint_trajectory_pub_;

  // Cartesian execution
  cartesian_msgs::CartesianCommand cartesian_command_msg_;
  ros::Publisher cartesian_command_pub_;

};  // end class

}  // end namespace

#endif
