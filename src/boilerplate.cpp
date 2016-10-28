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

// C++
#include <string>
#include <vector>

// MoveItManipulation
#include <moveit_boilerplate/boilerplate.h>

// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace moveit_boilerplate
{

Boilerplate::Boilerplate() : name_("boilerplate"), nh_("~")
{
  std::string joint_state_topic;
  std::string arm_joint_model_group;
  std::string execution_command_mode;
  std::string robot_description = "robot_description";

  // Load rosparams
  ros::NodeHandle rpnh(nh_, name_);
  int error = 0;
  if (!rosparam_shortcuts::get(name_, rpnh, "robot_description", robot_description))
  {
    ROS_WARN_STREAM_NAMED(name_, "Please specify robot_description in your config file");
  }
  error += !rosparam_shortcuts::get(name_, rpnh, "joint_state_topic", joint_state_topic);
  error += !rosparam_shortcuts::get(name_, rpnh, "arm_joint_model_group", arm_joint_model_group);
  error += !rosparam_shortcuts::get(name_, rpnh, "planning_scene_topic", planning_scene_topic_);
  error += !rosparam_shortcuts::get(name_, rpnh, "planning_scene_name", planning_scene_name_);
  rosparam_shortcuts::shutdownIfError(name_, error);

  // Load the loader
  robot_model_loader_.reset(new robot_model_loader::RobotModelLoader(robot_description));

  // Load the robot model
  robot_model_ = robot_model_loader_->getModel();  // Get a shared pointer to the robot
  if (!robot_model_)
  {
    ROS_ERROR_STREAM_NAMED(name_, "Unable to load robot model from '" << robot_description << "'");
    exit(-1);
  }

  // Choose planning group
  arm_jmg_ = robot_model_->getJointModelGroup(arm_joint_model_group);

  // Create the planning scene
  planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));

  // Create tf transformer
  tf_.reset(new tf::TransformListener(nh_));
  // TODO(davetcoleman): remove these lines, only an attempt to fix loadPlanningSceneMonitor bug
  ros::spinOnce();

  // Load planning scene monitor
  if (!loadPlanningSceneMonitor(joint_state_topic))
  {
    ROS_ERROR_STREAM_NAMED("boilerplate", "Unable to load planning scene monitor");
  }

  // Service for sharing the planning scene
  get_planning_scene_service_.initialize(nh_, "/get_planning_scene", planning_scene_monitor_);

  // Create initial robot state
  {
    psm::LockedPlanningSceneRO scene(planning_scene_monitor_);  // Lock planning scene
    current_state_.reset(new moveit::core::RobotState(scene->getCurrentState()));
  }  // end scoped pointer of locked planning scene

  // Load the Robot Viz Tools for publishing to Rviz
  loadVisualTools();

  // Load the debug interface for dealing with GUIs
  remote_control_.reset(new rviz_visual_tools::RemoteControl(nh_));

  // Load execution interface
  execution_interface_.reset(new ExecutionInterface(planning_scene_monitor_, visual_tools_));

  // Load planning interface
  planning_interface_.reset(new PlanningInterface(planning_scene_monitor_, visual_tools_, arm_jmg_, execution_interface_));

  ROS_INFO_STREAM_NAMED("boilerplate", "Boilerplate Ready.");
}

bool Boilerplate::loadPlanningSceneMonitor(const std::string& joint_state_topic)
{
  // Allows us to sycronize to Rviz and also publish collision objects to ourselves
  ROS_DEBUG_STREAM_NAMED("boilerplate", "Loading Planning Scene Monitor");
  planning_scene_monitor_.reset(
      new psm::PlanningSceneMonitor(planning_scene_, robot_model_loader_, tf_, planning_scene_name_));
  ros::spinOnce();

  if (planning_scene_monitor_->getPlanningScene())
  {
    // Optional monitors to start:
    planning_scene_monitor_->startStateMonitor(joint_state_topic, "");
    planning_scene_monitor_->startPublishingPlanningScene(psm::PlanningSceneMonitor::UPDATE_SCENE,
                                                          planning_scene_topic_);
    planning_scene_monitor_->getPlanningScene()->setName("planning_scene");
  }
  else
  {
    ROS_ERROR_STREAM_NAMED("boilerplate", "Planning scene not configured");
    return false;
  }
  ros::spinOnce();
  ros::Duration(0.5).sleep();  // when at 0.1, i believe sometimes vjoint not properly loaded

  // Wait for complete state to be recieved
  bool wait_for_complete_state = false;
  // Break early
  if (!wait_for_complete_state)
    return true;

  std::vector<std::string> missing_joints;
  std::size_t counter = 0;
  while (!planning_scene_monitor_->getStateMonitor()->haveCompleteState() && ros::ok())
  {
    ROS_INFO_STREAM_THROTTLE_NAMED(1, "", "Waiting for complete state from topic " << joint_state_topic);
    ros::Duration(0.1).sleep();
    ros::spinOnce();

    // Show unpublished joints
    if (counter % 10 == 0)
    {
      planning_scene_monitor_->getStateMonitor()->haveCompleteState(missing_joints);
      for (std::size_t i = 0; i < missing_joints.size(); ++i)
        ROS_WARN_STREAM_NAMED("boilerplate", "Unpublished joints: " << missing_joints[i]);
    }
    counter++;
  }
  ros::spinOnce();

  return true;
}

void Boilerplate::loadVisualTools()
{
  visual_tools_.reset(new mvt::MoveItVisualTools(robot_model_->getModelFrame(),
                                                 nh_.getNamespace() + "/markers",
                                                 planning_scene_monitor_));
  visual_tools_->loadRobotStatePub(nh_.getNamespace() + "/robot_state");
  visual_tools_->loadTrajectoryPub(nh_.getNamespace() + "/display_trajectory");
  visual_tools_->loadMarkerPub();
  visual_tools_->setAlpha(0.8);
  visual_tools_->deleteAllMarkers();  // clear all old markers
  visual_tools_->setManualSceneUpdating(true);
  visual_tools_->hideRobot();  // show that things have been reset
}

bool Boilerplate::showJointLimits(JointModelGroup* jmg)
{
  const std::vector<const moveit::core::JointModel*>& joints = jmg->getActiveJointModels();

  std::cout << std::endl;

  // Loop through joints
  for (std::size_t i = 0; i < joints.size(); ++i)
  {
    // Assume all joints have only one variable
    if (joints[i]->getVariableCount() > 1)
    {
      ROS_ERROR_STREAM_NAMED("manipulation", "Unable to handle joints with more than one var");
      return false;
    }
    getCurrentState();
    double current_value = current_state_->getVariablePosition(joints[i]->getName());

    // check if bad position
    bool out_of_bounds = !current_state_->satisfiesBounds(joints[i]);

    const moveit::core::VariableBounds& bound = joints[i]->getVariableBounds()[0];

    if (out_of_bounds)
      std::cout << MOVEIT_CONSOLE_COLOR_RED;

    std::cout << "   " << std::fixed << std::setprecision(5) << bound.min_position_ << "\t";
    double delta = bound.max_position_ - bound.min_position_;
    // std::cout << "delta: " << delta << " ";
    double step = delta / 20.0;

    bool marker_shown = false;
    for (double value = bound.min_position_; value < bound.max_position_; value += step)
    {
      // show marker of current value
      if (!marker_shown && current_value < value)
      {
        std::cout << "|";
        marker_shown = true;
      }
      else
        std::cout << "-";
    }
    // show max position
    std::cout << " \t" << std::fixed << std::setprecision(5) << bound.max_position_ << "  \t" << joints[i]->getName()
              << " current: " << std::fixed << std::setprecision(5) << current_value << std::endl;

    if (out_of_bounds)
      std::cout << MOVEIT_CONSOLE_COLOR_RESET;
  }

  return true;
}

moveit::core::RobotStatePtr Boilerplate::getCurrentState()
{
  // Get the real current state
  psm::LockedPlanningSceneRO scene(planning_scene_monitor_);  // Lock planning scene
  (*current_state_) = scene->getCurrentState();
  return current_state_;
}

const Eigen::Affine3d& Boilerplate::getCurrentPose()
{
  return getCurrentState()->getGlobalLinkTransform(arm_jmg_->getOnlyOneEndEffectorTip());
}

}  // namespace moveit_boilerplate
