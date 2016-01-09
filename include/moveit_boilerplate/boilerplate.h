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
 *   * Neither the name of PickNik LLC nor the names of its
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
   Desc:   Base class for using MoveIt! in C++
*/

#ifndef MOVEIT_BOILERPLATE__MOVEIT_BOILERPLATE
#define MOVEIT_BOILERPLATE__MOVEIT_BOILERPLATE

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>

// MoveIt
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/macros/console_colors.h>

// Visual tools
#include <moveit_visual_tools/moveit_visual_tools.h>

// moveit_boilerplate
#include <moveit_boilerplate/namespaces.h>
#include <moveit_boilerplate/debug_interface.h>
#include <moveit_boilerplate/execution_interface.h>
#include <moveit_boilerplate/planning_interface.h>
#include <moveit_boilerplate/get_planning_scene_service.h>

namespace moveit_boilerplate
{
static const std::string ROBOT_DESCRIPTION = "robot_description";

class Boilerplate
{
public:
  /**
   * \brief Constructor
   */
  Boilerplate();

  /**
   * \brief Connect to the MoveIt! planning scene messages
   */
  bool loadPlanningSceneMonitor(const std::string &joint_state_topic, const std::string &planning_scene_topic);

  /**
   * \brief Load visual tools
   */
  void loadVisualTools();

  /** \brief Output to console the current state of the robot's joint limits */
  bool showJointLimits(JointModelGroup *jmg);

  /**
   * \brief Use the planning scene to get the robot's current state
   */
  moveit::core::RobotStatePtr getCurrentState();

  /**
   * \brief Get pose of the end effector
   */
  const Eigen::Affine3d &getCurrentPose();

protected:
  // Name of this class
  std::string name_;

  // A shared node handle
  ros::NodeHandle nh_;

  // Transform
  boost::shared_ptr<tf::TransformListener> tf_;

  // For visualizing things in rviz
  mvt::MoveItVisualToolsPtr visual_tools_;

  // Core MoveIt components
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  robot_model::RobotModelPtr robot_model_;
  planning_scene::PlanningScenePtr planning_scene_;
  psm::PlanningSceneMonitorPtr planning_scene_monitor_;

  // Service for sharing the planning scene
  moveit_boilerplate::GetPlanningSceneService get_planning_scene_service_;

  // For executing joint and cartesian trajectories
  ExecutionInterfacePtr execution_interface_;

  // Allocated memory for robot state
  moveit::core::RobotStatePtr current_state_;

  // Debug interface for dealing with GUIs
  DebugInterfacePtr debug_interface_;

  // For generating joint trajectories
  PlanningInterfacePtr planning_interface_;

  // Desired planning group to work with
  JointModelGroup *arm_jmg_;

};  // end class

}  // end namespace

#endif
