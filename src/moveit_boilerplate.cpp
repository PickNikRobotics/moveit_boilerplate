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

// Command line arguments
#include <gflags/gflags.h>

// MoveItManipulation
#include <moveit_manipulation/moveit_boilerplate.h>

// MoveIt
#include <moveit/macros/console_colors.h>

namespace moveit_manipulation
{
DEFINE_bool(fake_execution, false, "Fake execution of motions");
DEFINE_int32(id, 0, "Identification number for various component modes");

MoveItBoilerplate::MoveItBoilerplate()
  : nh_("~")
{
  // Warn of fake modes
  if (FLAGS_fake_execution)
    ROS_WARN_STREAM_NAMED("moveit_boilerplate", "In fake execution mode");

  // Load the loader
  robot_model_loader_.reset(new robot_model_loader::RobotModelLoader(ROBOT_DESCRIPTION));

  // Load the robot model
  robot_model_ = robot_model_loader_->getModel();  // Get a shared pointer to the robot

  // Create the planning scene
  planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));

  // Get package path
  package_path_ = ros::package::getPath(package_name_);
  if (package_path_.empty())
    ROS_FATAL_STREAM_NAMED("product", "Unable to get " << package_name_ << " package path");

  // Load manipulation data for our robot
  config_.reset(new ManipulationData());
  config_->load(robot_model_, FLAGS_fake_execution, package_path_);

  // Create tf transformer
  tf_.reset(new tf::TransformListener(nh_));
  // TODO: remove these lines, only an attempt to fix loadPlanningSceneMonitor bug
  ros::spinOnce();

  // Load planning scene monitor
  if (!loadPlanningSceneMonitor())
  {
    ROS_ERROR_STREAM_NAMED("moveit_boilerplate", "Unable to load planning scene monitor");
  }

  // Load the Robot Viz Tools for publishing to Rviz
  loadVisualTools();

  // Load the remote control for dealing with GUIs
  remote_control_.reset(new RemoteControl(nh_));

  // Load grasp data specific to our robot
  grasp_datas_[config_->right_arm_].reset(
      new moveit_grasps::GraspData(nh_, config_->right_hand_name_, robot_model_));

  if (config_->dual_arm_)
    grasp_datas_[config_->left_arm_].reset(
        new moveit_grasps::GraspData(nh_, config_->left_hand_name_, robot_model_));

  // Create helper class for planning and manipulation
  planning_interface_.reset(new PlanningInterface(planning_scene_monitor_, config_,
                                       grasp_datas_, remote_control_, FLAGS_fake_execution));

  // Show interactive marker
  //setupInteractiveMarker();

  ROS_INFO_STREAM_NAMED("moveit_boilerplate", "MoveItBoilerplate Ready.");
}

bool MoveItBoilerplate::loadPlanningSceneMonitor()
{
  // Allows us to sycronize to Rviz and also publish collision objects to ourselves
  ROS_DEBUG_STREAM_NAMED("moveit_boilerplate", "Loading Planning Scene Monitor");
  static const std::string PLANNING_SCENE_MONITOR_NAME = "AmazonShelfWorld";
  planning_scene_monitor_.reset(new psm::PlanningSceneMonitor(
      planning_scene_, robot_model_loader_, tf_, PLANNING_SCENE_MONITOR_NAME));
  ros::spinOnce();

  if (planning_scene_monitor_->getPlanningScene())
  {
    // Optional monitors to start:
    planning_scene_monitor_->startStateMonitor(config_->joint_state_topic_, "");
    planning_scene_monitor_->startPublishingPlanningScene(
        psm::PlanningSceneMonitor::UPDATE_SCENE, "picknik_planning_scene");
    planning_scene_monitor_->getPlanningScene()->setName("picknik_planning_scene");
  }
  else
  {
    ROS_ERROR_STREAM_NAMED("moveit_boilerplate", "Planning scene not configured");
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
    ROS_INFO_STREAM_THROTTLE_NAMED(1, "", "Waiting for complete state from topic "
                                                          << config_->joint_state_topic_);
    ros::Duration(0.1).sleep();
    ros::spinOnce();

    // Show unpublished joints
    if (counter % 10 == 0)
    {
      planning_scene_monitor_->getStateMonitor()->haveCompleteState(missing_joints);
      for (std::size_t i = 0; i < missing_joints.size(); ++i)
        ROS_WARN_STREAM_NAMED("moveit_boilerplate", "Unpublished joints: " << missing_joints[i]);
    }
    counter++;
  }
  ros::spinOnce();

  return true;
}

void MoveItBoilerplate::loadVisualTools()
{
  visual_tools_.reset(new mvt::MoveItVisualTools(robot_model_->getModelFrame(),
                                                 "/moveit_manipulation/markers", planning_scene_monitor_));

  visual_tools_->loadRobotStatePub("/moveit_manipulation/robot_state");
  visual_tools_->loadTrajectoryPub("/moveit_manipulation/display_trajectory");
  visual_tools_->loadMarkerPub();
  visual_tools_->setAlpha(0.8);
  visual_tools_->deleteAllMarkers();  // clear all old markers
  visual_tools_->setManualSceneUpdating(true);
  visual_tools_->hideRobot();  // show that things have been reset
}

bool MoveItBoilerplate::allowCollisions(JointModelGroup* arm_jmg)
{
  // Allow collisions between frame of robot and floor
  {
    psm::LockedPlanningSceneRW scene(planning_scene_monitor_);  // Lock planning
    collision_detection::AllowedCollisionMatrix& collision_matrix =
        scene->getAllowedCollisionMatrixNonConst();

    // Get links of end effector
    const std::vector<std::string>& ee_link_names =
        grasp_datas_[arm_jmg]->ee_jmg_->getLinkModelNames();
    for (std::size_t i = 0; i < ee_link_names.size(); ++i)
    {
      for (std::size_t j = i + 1; j < ee_link_names.size(); ++j)
      {
        // std::cout << "disabling collsion between " << ee_link_names[i] << " and " <<
        // ee_link_names[j] << std::endl;
        collision_matrix.setEntry(ee_link_names[i], ee_link_names[j], true);
      }
    }
  }

  return true;
}

moveit::core::RobotStatePtr MoveItBoilerplate::getCurrentState()
{
  // Pass down to the exection interface layer so that we can catch the getCurrentState with a fake
  // one if we are unit testing  
  return planning_interface_->getExecutionInterface()->getCurrentState();
}

}  // end namespace
