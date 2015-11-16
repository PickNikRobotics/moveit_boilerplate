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

   **Developer Notes**

   Methods for publishing commands:

   TrajectoryExecutionInterface using MoveItControllerMananger:
     moveit_simple_controller_manager average: 0.00450083
     moveit_ros_control_interface     average: 0.222877
   Direct publishing on ROS topic:    average: 0.00184441  (59% faster)

*/

// MoveItManipulation
#include <moveit_boilerplate/execution_interface.h>

// MoveIt
#include <moveit/trajectory_execution_manager/trajectory_execution_manager.h>

// Parameter loading
#include <ros_param_shortcuts/ros_param_shortcuts.h>

// ros_control
#include <controller_manager_msgs/ListControllers.h>

namespace moveit_boilerplate
{
ExecutionInterface::ExecutionInterface(CommandMode mode, DebugInterfacePtr debug_interface,
                                       psm::PlanningSceneMonitorPtr planning_scene_monitor)
  : nh_("~")
  , mode_(mode)
  , debug_interface_(debug_interface)
  , planning_scene_monitor_(planning_scene_monitor)
{
  // Create initial robot state
  {
    psm::LockedPlanningSceneRO scene(planning_scene_monitor_);  // Lock planning scene
    current_state_.reset(new moveit::core::RobotState(scene->getCurrentState()));
  }  // end scoped pointer of locked planning scene

  // Debug tools for visualizing in Rviz
  loadVisualTools();

  std::string joint_trajectory_topic;
  std::string cartesian_command_topic;

  // Load rosparams
  {
    const std::string parent_name = "execution_interface";  // for namespacing logging messages
    ros::NodeHandle rosparam_nh(nh_, parent_name);
    using namespace ros_param_shortcuts;
    std::size_t error = 0;
    error += !getStringParam(parent_name, rosparam_nh, "joint_trajectory_topic", joint_trajectory_topic);
    error += !getStringParam(parent_name, rosparam_nh, "cartesian_command_topic", cartesian_command_topic);
    error += !getStringParam(parent_name, rosparam_nh, "save_traj_to_file_path", save_traj_to_file_path_);
    error += !getBoolParam(parent_name, rosparam_nh, "save_traj_to_file", save_traj_to_file_);
    error += !getBoolParam(parent_name, rosparam_nh, "visualize_trajectory_line", visualize_trajectory_line_);
    error += !getBoolParam(parent_name, rosparam_nh, "visualize_trajectory_path", visualize_trajectory_path_);
    error += !getBoolParam(parent_name, rosparam_nh, "check_for_waypoint_jumps", check_for_waypoint_jumps_);
    shutdownIfParamErrors(parent_name, error);
  }

  // Load the proper execution method
  switch (mode_)
  {
    case JOINT_EXECUTION_MANAGER:
      ROS_DEBUG_STREAM_NAMED("execution_interface","Connecting to trajectory execution manager");
      if (!trajectory_execution_manager_)
      {
        using namespace trajectory_execution_manager;
        trajectory_execution_manager_.reset(
            new TrajectoryExecutionManager(planning_scene_monitor_->getRobotModel()));
      }
      break;
    case JOINT_PUBLISHER:
      ROS_DEBUG_STREAM_NAMED("execution_interface","Connecting to joint publisher on topic " << joint_trajectory_topic);
      // Alternative method to sending trajectories than trajectory_execution_manager
      joint_trajectory_pub_ =
          nh_.advertise<trajectory_msgs::JointTrajectory>(joint_trajectory_topic, 1000);
      break;
    case CARTESIAN_PUBLISHER:
      ROS_DEBUG_STREAM_NAMED("execution_interface","Connecting to cartesian publisher on topic" << cartesian_command_topic);
      cartesian_command_pub_ =
          nh_.advertise<cartesian_msgs::CartesianCommand>(cartesian_command_topic, 1000);
      break;
    default:
      ROS_ERROR_STREAM_NAMED("execution_interface", "Unknown control mode");
  }

  ROS_INFO_STREAM_NAMED("execution_interface", "ExecutionInterface Ready.");
}

bool ExecutionInterface::executePose(const Eigen::Affine3d &pose, JointModelGroup *arm_jmg,
                                     const double &duration)
{
  if (mode_ != CARTESIAN_PUBLISHER)
  {
    ROS_ERROR_STREAM_NAMED("execution_interface", "Unable to execute pose because in different mode");
    return false;
  }

  visual_tools_->convertPoseSafe(pose, cartesian_command_msg_.desired_pose.pose);
  cartesian_command_msg_.duration = duration;
  cartesian_command_pub_.publish(cartesian_command_msg_);
  return true;
}

bool ExecutionInterface::executeTrajectory(moveit_msgs::RobotTrajectory &trajectory_msg,
                                           JointModelGroup *jmg, bool wait_for_execution)
{
  trajectory_msgs::JointTrajectory &trajectory = trajectory_msg.joint_trajectory;

  // Debug
  ROS_DEBUG_STREAM_NAMED("execution_interface.summary",
                         "Executing trajectory with " << trajectory.points.size() << " waypoints");
  ROS_DEBUG_STREAM_NAMED("execution_interface.trajectory", "Publishing:\n" << trajectory_msg);

  // Error check
  if (trajectory.points.empty())
  {
    ROS_ERROR_STREAM_NAMED("execution_interface","No points to execute, aborting trajectory execution");
    return false;
  }

  // Optionally save to file
  if (save_traj_to_file_)
    saveTrajectory(trajectory_msg,
                   jmg->getName() + "_moveit_trajectory_" +
                       boost::lexical_cast<std::string>(trajectory_filename_count_++) + ".csv");

  // Optionally visualize the hand/wrist path in Rviz
  if (visualize_trajectory_line_)
  {
    if (trajectory.points.size() > 1 && !jmg->isEndEffector())
    {
      visual_tools_->deleteAllMarkers();
      ros::spinOnce();  // TODO remove?

      // TODO get parent_link using native moveit tools
      // visual_tools_->publishTrajectoryLine(
      // trajectory_msg, grasp_datas_[jmg]->parent_link_, config_->right_arm_, rvt::LIME_GREEN);
    }
    else
      ROS_WARN_STREAM_NAMED("execution_interface",
                            "Not visualizing path because trajectory only has "
                                << trajectory.points.size()
                                << " points or because is end effector");
  }

  // Optionally visualize trajectory in Rviz
  if (visualize_trajectory_path_)
  {
    const bool wait_for_trajetory = false;
    visual_tools_->publishTrajectoryPath(trajectory_msg, getCurrentState(), wait_for_trajetory);
  }

  // Optionally check for errors in trajectory
  if (check_for_waypoint_jumps_)
    checkForWaypointJumps(trajectory);

  // Confirm trajectory before continuing
  if (!debug_interface_->getFullAutonomous())
  {
    debug_interface_->waitForNextFullStep("execute trajectory");
    ROS_INFO_STREAM_NAMED("execution_interface", "Remote confirmed trajectory execution.");
  }

  // Send new trajectory
  switch (mode_)
  {
    case JOINT_EXECUTION_MANAGER:
      // Reset trajectory manager
      trajectory_execution_manager_->clear();

      if (trajectory_execution_manager_->pushAndExecute(trajectory_msg))
      {
        // Optionally wait for completion
        if (wait_for_execution)
          waitForExecution();
        else
          ROS_DEBUG_STREAM_NAMED("exceution_interface", "Not waiting for execution to finish");
      }
      else
      {
        ROS_ERROR_STREAM_NAMED("execution_interface", "Failed to execute trajectory");
        return false;
      }
      break;
    case JOINT_PUBLISHER:
      joint_trajectory_pub_.publish(trajectory);

      if (wait_for_execution)
      {
        ROS_INFO_STREAM_NAMED("execution_interface","Sleeping while trajectory executes");
        ros::Duration(trajectory.points.back().time_from_start).sleep();
      }
      break;
    case CARTESIAN_PUBLISHER:
      ROS_ERROR_STREAM_NAMED("execution_interface", "Incorrect control mode: CARTESIAN");
      break;
    default:
      ROS_ERROR_STREAM_NAMED("execution_interface", "Unknown control mode");
  }

  return true;
}

bool ExecutionInterface::stopExecution()
{
  ROS_WARN_STREAM_NAMED("temp","Execution stop requested");

  trajectory_msgs::JointTrajectory blank_trajectory;

  switch (mode_)
  {
    case JOINT_EXECUTION_MANAGER:
      ROS_ERROR_STREAM_NAMED("temp","Not implemented");
      // TODO Just send a blank trajectory
      break;
    case JOINT_PUBLISHER:
      // Just send a blank trajectory
      joint_trajectory_pub_.publish(blank_trajectory);
      return true;
      break;
    case CARTESIAN_PUBLISHER:
      ROS_ERROR_STREAM_NAMED("temp","Not implemented");
      break;
    default:
      ROS_ERROR_STREAM_NAMED("execution_interface", "Unknown control mode");
  }
  return false;
}

bool ExecutionInterface::waitForExecution()
{
  if (mode_ != JOINT_EXECUTION_MANAGER)
  {
    ROS_WARN_STREAM_NAMED("execution_interface", "Not waiting for execution because not in execution_manager "
                                  "mode");
    return true;
  }

  ROS_DEBUG_STREAM_NAMED("execution_interface", "Waiting for executing trajectory to finish");

  // wait for the trajectory to complete
  moveit_controller_manager::ExecutionStatus execution_status =
      trajectory_execution_manager_->waitForExecution();
  if (execution_status == moveit_controller_manager::ExecutionStatus::SUCCEEDED)
  {
    ROS_DEBUG_STREAM_NAMED("execution_interface", "Trajectory execution succeeded");
    return true;
  }

  if (execution_status == moveit_controller_manager::ExecutionStatus::PREEMPTED)
    ROS_INFO_STREAM_NAMED("execution_interface", "Trajectory execution preempted");
  else if (execution_status == moveit_controller_manager::ExecutionStatus::TIMED_OUT)
    ROS_ERROR_STREAM_NAMED("execution_interface", "Trajectory execution timed out");
  else
    ROS_ERROR_STREAM_NAMED("execution_interface", "Trajectory execution control failed");

  return false;
}

void ExecutionInterface::checkForWaypointJumps(const trajectory_msgs::JointTrajectory &trajectory)
{
  // Debug: check for errors in trajectory
  static const double MAX_TIME_STEP_SEC = 4.0;
  ros::Duration max_time_step(MAX_TIME_STEP_SEC);
  static const double WARN_TIME_STEP_SEC = 3.0;
  ros::Duration warn_time_step(WARN_TIME_STEP_SEC);
  ros::Duration diff;
  for (std::size_t i = 0; i < trajectory.points.size() - 1; ++i)
  {
    diff = (trajectory.points[i + 1].time_from_start - trajectory.points[i].time_from_start);
    if (diff > max_time_step)
    {
      ROS_ERROR_STREAM_NAMED(
          "execution_interface",
          "Max time step between points exceeded, likely because of wrap around/IK bug. Point " << i);
      std::cout << "First time: " << trajectory.points[i].time_from_start.toSec() << std::endl;
      std::cout << "Next time: " << trajectory.points[i + 1].time_from_start.toSec() << std::endl;
      std::cout << "Diff time: " << diff.toSec() << std::endl;
      std::cout << "-------------------------------------------------------" << std::endl;
      std::cout << std::endl;

      debug_interface_->setAutonomous(false);
      debug_interface_->setFullAutonomous(false);

      // return false;
    }
    else if (diff > warn_time_step)
    {
      ROS_WARN_STREAM_NAMED(
          "execution_interface",
          "Warn time step between points exceeded, likely because of wrap around/IK bug. Point "
              << i);
      std::cout << "First time: " << trajectory.points[i].time_from_start.toSec() << std::endl;
      std::cout << "Next time: " << trajectory.points[i + 1].time_from_start.toSec() << std::endl;
      std::cout << "Diff time: " << diff.toSec() << std::endl;
      std::cout << "-------------------------------------------------------" << std::endl;
      std::cout << std::endl;
    }
  }
}

bool ExecutionInterface::checkExecutionManager()
{
  ROS_INFO_STREAM_NAMED("execution_interface", "Checking that execution manager is loaded.");

  /*
  JointModelGroup *arm_jmg = config_->dual_arm_ ? config_->both_arms_ : config_->right_arm_;

  // Get the controllers List
  std::vector<std::string> controller_list;
  trajectory_execution_manager_->getControllerManager()->getControllersList(controller_list);

  // Check active controllers are running
  if (!trajectory_execution_manager_->ensureActiveControllersForGroup(arm_jmg->getName()))
  {
    ROS_ERROR_STREAM_NAMED("execution_interface",
                           "Group '" << arm_jmg->getName() << "' does not have controllers loaded");
    std::cout << "Available controllers: " << std::endl;
    std::copy(controller_list.begin(), controller_list.end(),
              std::ostream_iterator<std::string>(std::cout, "\n"));
    return false;
  }

  // Check active controllers are running
  if (!trajectory_execution_manager_->ensureActiveControllers(controller_list))
  {
    ROS_ERROR_STREAM_NAMED("execution_interface", "Robot does not have the desired controllers "
                                                  "active");
    return false;
  }
  */

  return true;
}

bool ExecutionInterface::checkTrajectoryController(ros::ServiceClient &service_client,
                                                   const std::string &hardware_name, bool has_ee)
{
  // Try to communicate with controller manager
  controller_manager_msgs::ListControllers service;
  ROS_DEBUG_STREAM_NAMED("execution_interface", "Calling list controllers service client");
  if (!service_client.call(service))
  {
    ROS_ERROR_STREAM_THROTTLE_NAMED(2, "execution_interface",
                                    "Unable to check if controllers for "
                                        << hardware_name << " are loaded, failing. Using nh "
                                                            "namespace " << nh_.getNamespace()
                                        << ". Service response: " << service.response);
    return false;
  }

  std::string control_type = "position";  // fake_execution_ ? "position" : "velocity";

  // Check if proper controller is running
  bool found_main_controller = false;
  bool found_ee_controller = false;

  for (std::size_t i = 0; i < service.response.controller.size(); ++i)
  {
    if (service.response.controller[i].name == control_type + "_trajectory_controller")
    {
      found_main_controller = true;
      if (service.response.controller[i].state != "running")
      {
        ROS_WARN_STREAM_THROTTLE_NAMED(2, "execution_interface",
                                       "Controller for " << hardware_name << " is in manual mode");
        return false;
      }
    }
    if (service.response.controller[i].name == "ee_" + control_type + "_trajectory_controller")
    {
      found_ee_controller = true;
      if (service.response.controller[i].state != "running")
      {
        ROS_WARN_STREAM_THROTTLE_NAMED(2, "execution_interface",
                                       "Controller for " << hardware_name << " is in manual mode");
        return false;
      }
    }
  }

  if (has_ee && !found_ee_controller)
  {
    ROS_ERROR_STREAM_THROTTLE_NAMED(
        2, "execution_interface", "No end effector controller found for "
                                      << hardware_name << ". Controllers are: " << service.response);
    return false;
  }
  if (!found_main_controller)
  {
    ROS_ERROR_STREAM_THROTTLE_NAMED(
        2, "execution_interface", "No main controller found for "
                                      << hardware_name << ". Controllers are: " << service.response);
    return false;
  }

  return true;
}

bool ExecutionInterface::saveTrajectory(const moveit_msgs::RobotTrajectory &trajectory_msg,
                                        const std::string &file_name)
{
  const trajectory_msgs::JointTrajectory &joint_trajectory = trajectory_msg.joint_trajectory;

  // Error check
  if (!joint_trajectory.points.size() || !joint_trajectory.points[0].positions.size())
  {
    ROS_ERROR_STREAM_NAMED("execution_interface", "No trajectory points available to save");
    return false;
  }
  bool has_accelerations = true;
  if (joint_trajectory.points[0].accelerations.size() == 0)
  {
    has_accelerations = false;
  }

  std::string file_path = save_traj_to_file_path_ + "/" + file_name;

  std::ofstream output_file;
  output_file.open(file_path.c_str());

  // Output header -------------------------------------------------------
  output_file << "time_from_start,";
  for (std::size_t j = 0; j < joint_trajectory.joint_names.size(); ++j)
  {
    output_file << joint_trajectory.joint_names[j] << "_pos," << joint_trajectory.joint_names[j]
                << "_vel,";
    if (has_accelerations)
      output_file << joint_trajectory.joint_names[j] << "_acc,";
  }
  output_file << std::endl;

  // Output data ------------------------------------------------------

  // Subtract starting time

  for (std::size_t i = 0; i < joint_trajectory.points.size(); ++i)
  {
    // Timestamp
    output_file.precision(20);
    output_file << joint_trajectory.points[i].time_from_start.toSec() << ",";
    output_file.precision(5);
    // Output entire trajectory to single line
    for (std::size_t j = 0; j < joint_trajectory.points[i].positions.size(); ++j)
    {
      // Output State
      output_file << joint_trajectory.points[i].positions[j] << ",";
      if (joint_trajectory.points[i].velocities.size())
        output_file << joint_trajectory.points[i].velocities[j] << ",";
      if (joint_trajectory.points[i].accelerations.size())
        output_file << joint_trajectory.points[i].accelerations[j] << ",";
    }

    output_file << std::endl;
  }
  output_file.close();
  ROS_INFO_STREAM_NAMED("execution_interface", "Saved trajectory to file " << file_name);
  return true;
}

moveit::core::RobotStatePtr ExecutionInterface::getCurrentState()
{
  // Get the real current state
  psm::LockedPlanningSceneRO scene(planning_scene_monitor_);  // Lock planning scene
  (*current_state_) = scene->getCurrentState();
  return current_state_;
}

void ExecutionInterface::loadVisualTools()
{
  visual_tools_.reset(
      new mvt::MoveItVisualTools(planning_scene_monitor_->getRobotModel()->getModelFrame(),
                                 "/moveit_boilerplate/markers", planning_scene_monitor_));

  visual_tools_->loadRobotStatePub("/moveit_boilerplate/robot_state");
  visual_tools_->loadTrajectoryPub("/moveit_boilerplate/display_trajectory");
  visual_tools_->loadMarkerPub();
  visual_tools_->setAlpha(0.8);
  visual_tools_->deleteAllMarkers();  // clear all old markers
  visual_tools_->setManualSceneUpdating(true);
  visual_tools_->hideRobot();  // show that things have been reset
}

}  // end namespace
