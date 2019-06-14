/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, PickNik LLC
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

/* Author: Dave Coleman <dave@picknik.ai>, Henning Kayser <henningkayser@picknik.ai>
   Desc:   Collection of planning problems for experimenting with the R2 robot
*/

// C++
#include <string>
#include <vector>

// this package
#include <moveit_base/moveit_base.h>

// MoveIt
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

// Conversions
#include <tf_conversions/tf_eigen.h>
#include <moveit/robot_state/conversions.h>
#include <eigen_conversions/eigen_msg.h>

namespace moveit_base
{
MoveItBase::MoveItBase()
{
}

bool MoveItBase::init(ros::NodeHandle& nh)
{
  nh_ = nh;
  std::string joint_state_topic;
  std::string rviz_markers_topic, rviz_robot_state_topic, rviz_trajectory_topic;

  // Load rosparams
  ros::NodeHandle rpnh(nh_, name_);
  int error = 0;
  error += !rosparam_shortcuts::get(name_, rpnh, "joint_state_topic", joint_state_topic);
  error += !rosparam_shortcuts::get(name_, rpnh, "planning_scene_topic", planning_scene_topic_);
  error += !rosparam_shortcuts::get(name_, rpnh, "rviz/markers_topic", rviz_markers_topic);
  error += !rosparam_shortcuts::get(name_, rpnh, "rviz/robot_state_topic", rviz_robot_state_topic);
  error += !rosparam_shortcuts::get(name_, rpnh, "rviz/trajectory_topic", rviz_trajectory_topic);
  rosparam_shortcuts::shutdownIfError(name_, error);

  // Load the loader
  robot_model_loader_.reset(new robot_model_loader::RobotModelLoader(ROBOT_DESCRIPTION, false));
  ros::Duration(5.0).sleep();

  // Load the robot model
  robot_model_ = robot_model_loader_->getModel();  // Get a shared pointer to the robot

  //// Create the planning scene
  planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));

  // Load planning scene monitor
  if (!loadPlanningSceneMonitor(joint_state_topic))
  {
    ROS_ERROR_STREAM_NAMED(name_, "Unable to load planning scene monitor");
  }

  // Create initial robot state
  {
    planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_);  // Lock planning scene
    current_state_.reset(new moveit::core::RobotState(scene->getCurrentState()));
  }  // end scoped pointer of locked planning scene

  // Init planning pipeline
  planning_pipeline_.reset(new planning_pipeline::PlanningPipeline(robot_model_, nh, "planning_plugin",
        "request_adapters"));

  // Init Trajectory Execution Manager
  trajectory_execution_manager_.reset(new trajectory_execution_manager::TrajectoryExecutionManager(robot_model_,
        planning_scene_monitor_->getStateMonitor()));

  // Load the Robot Viz Tools for publishing to Rviz
  loadVisualTools(rviz_markers_topic, rviz_robot_state_topic, rviz_trajectory_topic);

  ROS_INFO_STREAM_NAMED(name_, "MoveItBase Ready.");

  return true;
}

bool MoveItBase::enableExecutionController(const std::string& group_name)
{
  if (!trajectory_execution_manager_->ensureActiveControllersForGroup(group_name))
  {
    ROS_ERROR_STREAM("Group " << group_name << " does not have controllers loaded");
    return false;
  }

  // Get the controllers List
  std::vector<std::string> controller_list;
  trajectory_execution_manager_->getControllerManager()->getControllersList(controller_list);

  // Check active controllers are running
  if (!trajectory_execution_manager_->ensureActiveControllers(controller_list))
  {
    ROS_ERROR("Robot does not have the desired controllers active");
    return false;
  }
  return true;
}

bool MoveItBase::loadPlanningSceneMonitor(const std::string& joint_state_topic)
{
  // Create tf transformer
  tf_buffer_.reset(new tf2_ros::Buffer());

  // TODO(davetcoleman): remove these lines, only an attempt to fix loadPlanningSceneMonitor bug
  ros::spinOnce();

  // Allows us to sycronize to Rviz and also publish collision objects to ourselves
  ROS_DEBUG_STREAM_NAMED(name_, "Loading Planning Scene Monitor");
  static const std::string PLANNING_SCENE_MONITOR_NAME = "MoveItBasePlanningScene";
  planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(planning_scene_, robot_model_loader_,
                                                                                 tf_buffer_, PLANNING_SCENE_MONITOR_NAME));
  ros::spinOnce();

  if (planning_scene_monitor_->getPlanningScene())
  {
    // Optional monitors to start:
    planning_scene_monitor_->startStateMonitor(joint_state_topic, "");
    planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
                                                          planning_scene_topic_);
    // planning_scene_monitor_->getPlanningScene()->setName("planning_scene");
    planning_scene_monitor_->startSceneMonitor(planning_scene_topic_);
  }
  else
  {
    ROS_ERROR_STREAM_NAMED(name_, "Planning scene not configured");
    return false;
  }
  // ros::spinOnce();
  // ros::Duration(0.5).sleep();  // when at 0.1, i believe sometimes vjoint not properly loaded

  // Wait for complete state to be recieved
  bool wait_for_complete_state = false;
  // Break early
  if (!wait_for_complete_state)
    return true;

  std::vector<std::string> missing_joints;
  std::size_t counter = 0;
  while (!planning_scene_monitor_->getStateMonitor()->haveCompleteState() && ros::ok())
  {
    ROS_INFO_STREAM_THROTTLE_NAMED(1, name_, "Waiting for complete state from topic " << joint_state_topic);
    ros::Duration(0.1).sleep();
    ros::spinOnce();

    // Show unpublished joints
    if (counter % 10 == 0)
    {
      planning_scene_monitor_->getStateMonitor()->haveCompleteState(missing_joints);
      for (std::size_t i = 0; i < missing_joints.size(); ++i)
        ROS_WARN_STREAM_NAMED(name_, "Unpublished joints: " << missing_joints[i]);
    }
    counter++;
  }
  ros::spinOnce();

  return true;
}

void MoveItBase::loadVisualTools(const std::string& rviz_markers_topic, const std::string& rviz_robot_state_topic,
                                 const std::string& rviz_trajectory_topic)
{
  visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools(robot_model_->getModelFrame(), rviz_markers_topic,
                                                                 planning_scene_monitor_));

  visual_tools_->loadRobotStatePub(rviz_robot_state_topic);
  visual_tools_->loadTrajectoryPub(rviz_trajectory_topic);
  visual_tools_->loadMarkerPub();
  visual_tools_->setAlpha(0.8);
  visual_tools_->deleteAllMarkers();  // clear all old markers
  visual_tools_->setManualSceneUpdating(true);
  visual_tools_->hideRobot();  // show that things have been reset
}

bool MoveItBase::showJointLimits(const moveit::core::JointModelGroup* jmg)
{
  const std::vector<const moveit::core::JointModel*>& joints = jmg->getActiveJointModels();

  std::cout << std::endl;

  // Loop through joints
  for (std::size_t i = 0; i < joints.size(); ++i)
  {
    // Assume all joints have only one variable
    if (joints[i]->getVariableCount() > 1)
    {
      ROS_ERROR_STREAM_NAMED(name_, "Unable to handle joints with more than one var");
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

moveit::core::RobotStatePtr MoveItBase::getCurrentState()
{
  // Get the real current state
  planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_);  // Lock planning scene
  (*current_state_) = scene->getCurrentState();
  return current_state_;
}

bool MoveItBase::getTFTransform(const std::string& from_frame, const std::string& to_frame, Eigen::Affine3d& pose)
{
  // TODO(davetcoleman): fix this function for TF2, from TF1

  ROS_ERROR_STREAM_NAMED("moveit_base", "NOT IMPLEMENTED");
  // tf::StampedTransform tf_transform;
  // try
  // {
  //   tf_buffer_->lookupTransform(from_frame, to_frame, ros::Time(0), tf_transform);
  // }
  // catch (tf::TransformException ex)
  // {
  //   ROS_ERROR("%s", ex.what());
  //   return false;
  // }

  // // Convert to eigen
  // tf::transformTFToEigen(tf_transform, pose);
  return true;
}

bool MoveItBase::planTo(const moveit::core::RobotState& robot_state, std::string planning_group)
{
  if (planning_group.empty())
  {
    if (default_planning_group_.empty())
    {
      ROS_ERROR("Failed to plan - no planning group specified");
      return false;
    }
    planning_group = default_planning_group_;
  }

  // retrieve joint model group
  const auto& joint_model_group = robot_state.getJointModelGroup(planning_group);
  if (!joint_model_group)
  {
    ROS_ERROR_STREAM("Robot model has no joint model group named '" << planning_group << "'");
    return false;
  }

  // Clone current planning scene
  planning_scene_monitor_->lockSceneRead();  // LOCK planning scene
  planning_scene::PlanningScenePtr planning_scene =
    planning_scene::PlanningScene::clone(planning_scene_monitor_->getPlanningScene());
  planning_scene_monitor_->unlockSceneRead();  // UNLOCK planning scene

  // Init MotionPlanRequest
  planning_interface::MotionPlanRequest req;
  req.group_name = planning_group;
  req.planner_id = default_planner_id_;
  req.allowed_planning_time = allowed_planning_time_;
  req.max_velocity_scaling_factor = max_velocity_scaling_factor_;
  req.max_acceleration_scaling_factor = max_acceleration_scaling_factor_;

  // Set start state
  moveit::core::RobotStatePtr start_state = start_state_;
  if (!start_state)
    start_state = getCurrentState();
  moveit::core::robotStateToRobotStateMsg(*start_state, req.start_state);
  planning_scene->setCurrentState(*start_state);

  // Set goal state
  req.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(robot_state, joint_model_group));

  // Run planning attempt
  planning_interface::MotionPlanResponse res;
  planning_pipeline_->generatePlan(planning_scene, req, res);
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR("Could not compute plan successfully");
    last_solution_trajectory_ = NULL;
    return false;
  }
  last_solution_trajectory_ = res.trajectory_;
  std::vector<const moveit::core::LinkModel*> eef_links;
  if (joint_model_group->getEndEffectorTips(eef_links))
  {
    for (const auto& eef_link : eef_links)
    {
      ROS_INFO_STREAM("Publishing trajectory for end effector " << eef_link->getName());
      visual_tools_->publishTrajectoryLine(last_solution_trajectory_, eef_link);
      visual_tools_->publishTrajectoryPath(last_solution_trajectory_, false);
      visual_tools_->publishRobotState(last_solution_trajectory_->getLastWayPoint(), rviz_visual_tools::TRANSLUCENT);
    }
  }
  return true;
}

bool MoveItBase::execute(bool blocking)
{
  if (!last_solution_trajectory_)
  {
    ROS_ERROR("There is no successfull trajectory to execute");
    return false;
  }

  trajectory_processing::TimeOptimalTrajectoryGeneration totg;
  if (!totg.computeTimeStamps(*last_solution_trajectory_, max_velocity_scaling_factor_, max_acceleration_scaling_factor_))
  {
    ROS_ERROR("Failed to parameterize trajectory");
    return false;
  }

  trajectory_execution_manager_->clear();
  moveit_msgs::RobotTrajectory robot_trajectory;
  last_solution_trajectory_->getRobotTrajectoryMsg(robot_trajectory);
  trajectory_execution_manager_->push(robot_trajectory);
  trajectory_execution_manager_->execute();
  if (blocking)
    return trajectory_execution_manager_->waitForExecution();
  else
    return true;
}


}  // namespace moveit_base
