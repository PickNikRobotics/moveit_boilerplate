/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
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
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
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

// MoveItManipulation
#include <moveit_manipulation/execution_interface.h>

// MoveIt
#include <moveit/trajectory_execution_manager/trajectory_execution_manager.h>

// ros_control
#include <controller_manager_msgs/ListControllers.h>

namespace moveit_manipulation
{
ExecutionInterface::ExecutionInterface(
    RemoteControlPtr remote_control,
    moveit_grasps::GraspDatas grasp_datas,
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor,
    ManipulationDataPtr config, moveit::core::RobotStatePtr current_state, bool fake_execution)
  : remote_control_(remote_control)
  , grasp_datas_(grasp_datas)
  , planning_scene_monitor_(planning_scene_monitor)
  , config_(config)
  , current_state_(current_state)
  , nh_("~")
  , unit_testing_enabled_(false)
  , fake_execution_(fake_execution)
{
  // Debug tools for visualizing in Rviz
  loadVisualTools();

  // Check that controllers are ready
  zaber_list_controllers_client_ = nh_.serviceClient<controller_manager_msgs::ListControllers>(
      "/jacob/zaber/controller_manager/list_controllers");
  kinova_list_controllers_client_ = nh_.serviceClient<controller_manager_msgs::ListControllers>(
      "/jacob/kinova/controller_manager/list_controllers");

  cartesian_command_pub_ =
      nh_.advertise<cartesian_msgs::CartesianCommand>("/r3/cartesian_command", 1000);

  ROS_INFO_STREAM_NAMED("execution_interface", "ExecutionInterface Ready.");
}

bool ExecutionInterface::executePose(const Eigen::Affine3d &pose, JointModelGroup *arm_jmg,
                                     const double &duration)
{
  // Convert the parent link location to that which Blue expects (URDFs are off). This value was
  // manually calibrated
  Eigen::Affine3d converted_pose = pose * grasp_datas_[arm_jmg]->grasp_pose_to_eef_pose_;

  visual_tools_->convertPoseSafe(converted_pose,
                                           cartesian_command_msg_.desired_pose.pose);
  cartesian_command_msg_.duration = duration;
  cartesian_command_pub_.publish(cartesian_command_msg_);
  return true;
}

bool ExecutionInterface::executeTrajectory(moveit_msgs::RobotTrajectory &trajectory_msg,
                                           JointModelGroup *jmg, bool wait_for_execution)
{
  trajectory_msgs::JointTrajectory &trajectory = trajectory_msg.joint_trajectory;

  bool run_fast = true;

  // Debug
  if (!run_fast)
    ROS_INFO_STREAM_NAMED("execution_interface", "Executing trajectory with "
                                                     << trajectory.points.size() << " waypoints");

  // Debug
  if (!run_fast)
    ROS_DEBUG_STREAM_NAMED("execution_interface.trajectory", "Publishing:\n" << trajectory_msg);

  // Save to file
  if (!run_fast)
  {
    // Only save non-finger trajectories
    if (trajectory.joint_names.size() > 3)
    {
      static std::size_t trajectory_count = 0;
      saveTrajectory(trajectory_msg, jmg->getName() + "_trajectory_" +
                                         boost::lexical_cast<std::string>(trajectory_count++) +
                                         ".csv");
    }
  }

  // Visualize the hand/wrist path in Rviz
  if (!run_fast)
  {
    if (trajectory.points.size() > 1 && !jmg->isEndEffector())
    {
      visual_tools_->deleteAllMarkers();
      ros::spinOnce();  // TODO remove?

      visual_tools_->publishTrajectoryLine(
          trajectory_msg, grasp_datas_[jmg]->parent_link_, config_->right_arm_, rvt::LIME_GREEN);
    }
    else
      ROS_WARN_STREAM_NAMED("execution_interface",
                            "Not visualizing path because trajectory only has "
                                << trajectory.points.size()
                                << " points or because is end effector");

    // Visualize trajectory in Rviz
    bool wait_for_trajetory = false;
    visual_tools_->publishTrajectoryPath(trajectory_msg, getCurrentState(),
                                                   wait_for_trajetory);
  }

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
          "Max time step between points exceeded, likely because of wrap around/IK bug. Point "
              << i);
      std::cout << "First time: " << trajectory.points[i].time_from_start.toSec() << std::endl;
      std::cout << "Next time: " << trajectory.points[i + 1].time_from_start.toSec() << std::endl;
      std::cout << "Diff time: " << diff.toSec() << std::endl;
      std::cout << "-------------------------------------------------------" << std::endl;
      std::cout << std::endl;

      remote_control_->setAutonomous(false);
      remote_control_->setFullAutonomous(false);

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

  // Ensure that execution manager has been loaded
  loadExecutionManager();

  // Check if in unit testing mode
  if (unit_testing_enabled_)
  {
    robot_trajectory::RobotTrajectoryPtr robot_trajectory(
        new robot_trajectory::RobotTrajectory(planning_scene_monitor_->getRobotModel(), jmg));

    robot_trajectory->setRobotTrajectoryMsg(*current_state_, trajectory_msg);
    *current_state_ = robot_trajectory->getLastWayPoint();
    return true;
  }

  // Confirm trajectory before continuing
  if (!remote_control_->getFullAutonomous() &&
      // Only wait for non-finger trajectories
      trajectory.joint_names.size() > 3)
  {
    remote_control_->waitForNextFullStep("execute trajectory");
    ROS_INFO_STREAM_NAMED("execution_interface", "Executing trajectory....");
  }

  // Reset trajectory manager
  trajectory_execution_manager_->clear();

  // Send new trajectory
  //if (trajectory_execution_manager_->push(trajectory_msg))
  if (trajectory_execution_manager_->pushAndExecute(trajectory_msg))
  {
    //trajectory_execution_manager_->execute();

    // Optionally wait for completion
    if (wait_for_execution)
    {
      waitForExecution();
    }
    else if (!run_fast)
    {
      ROS_INFO_STREAM_NAMED("exceution_interface", "Not waiting for execution to finish");
    }
  }
  else
  {
    ROS_ERROR_STREAM_NAMED("execution_interface", "Failed to execute trajectory");
    return false;
  }

  return true;
}

bool ExecutionInterface::waitForExecution()
{
  ROS_DEBUG_STREAM_NAMED("execution_interface", "Waiting for executing trajectory to finish");

  // Ensure that execution manager has been loaded
  loadExecutionManager();

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

bool ExecutionInterface::loadExecutionManager()
{
  if (!trajectory_execution_manager_)
  {
    ROS_DEBUG_STREAM_NAMED("execution_interface", "Loading trajectory execution manager");
    trajectory_execution_manager_.reset(
        new trajectory_execution_manager::TrajectoryExecutionManager(
            planning_scene_monitor_->getRobotModel()));
  }
  return true;
}

bool ExecutionInterface::checkExecutionManager()
{
  ROS_INFO_STREAM_NAMED("execution_interface", "Checking that execution manager is loaded.");

  // Ensure that execution manager has been loaded
  loadExecutionManager();

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
    ROS_ERROR_STREAM_NAMED("execution_interface",
                           "Robot does not have the desired controllers active");
    return false;
  }

  // Check that correct controllers are running
  // TODO - make this not jacob-specific
  if (false)
  {
    bool has_error = true;

    while (has_error && ros::ok())
    {
      has_error = false;
      if (!checkTrajectoryController(zaber_list_controllers_client_, "zaber"))
      {
        has_error = true;
      }

      bool has_ee = true;
      if (!checkTrajectoryController(kinova_list_controllers_client_, "kinova", has_ee))
      {
        has_error = true;
      }

      ros::Duration(0.5).sleep();
    }
  }

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
    ROS_ERROR_STREAM_THROTTLE_NAMED(
        2, "execution_interface",
        "Unable to check if controllers for "
            << hardware_name << " are loaded, failing. Using nh namespace " << nh_.getNamespace()
            << ". Service response: " << service.response);
    return false;
  }

  std::string control_type = fake_execution_ ? "position" : "velocity";

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
    ROS_ERROR_STREAM_THROTTLE_NAMED(2, "execution_interface",
                                    "No end effector controller found for "
                                        << hardware_name
                                        << ". Controllers are: " << service.response);
    return false;
  }
  if (!found_main_controller)
  {
    ROS_ERROR_STREAM_THROTTLE_NAMED(2, "execution_interface",
                                    "No main controller found for "
                                        << hardware_name
                                        << ". Controllers are: " << service.response);
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

  std::string file_path;
  getFilePath(file_path, file_name);
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
  ROS_DEBUG_STREAM_NAMED("execution_interface", "Saved trajectory to file " << file_name);
  return true;
}

bool ExecutionInterface::getFilePath(std::string &file_path, const std::string &file_name) const
{
  namespace fs = boost::filesystem;

  // Check that the directory exists, if not, create it
  fs::path path;
  path = fs::path(config_->package_path_ + "/trajectories/analysis/");

  boost::system::error_code returnedError;
  fs::create_directories(path, returnedError);

  // Error check
  if (returnedError)
  {
    ROS_ERROR_STREAM_NAMED("execution_interface", "Unable to create directory " << path.string());
    return false;
  }

  // Directories successfully created, append the group name as the file name
  path = path / fs::path(file_name);
  file_path = path.string();

  ROS_DEBUG_STREAM_NAMED("execution_interface.file_path", "Using full file path" << file_path);
  return true;
}

moveit::core::RobotStatePtr ExecutionInterface::getCurrentState()
{
  // Get the fake current state
  if (unit_testing_enabled_)
  {
    // ROS_WARN_STREAM_NAMED("manipulation","Unit testing enabled, get current state is skipping
    // planning scene");
    return current_state_;
  }

  // Get the real current state
  planning_scene_monitor::LockedPlanningSceneRO scene(
      planning_scene_monitor_);  // Lock planning scene
  (*current_state_) = scene->getCurrentState();
  return current_state_;
}

bool ExecutionInterface::enableUnitTesting(bool enable)
{
  unit_testing_enabled_ = enable;
  return true;
}

void ExecutionInterface::loadVisualTools()
{
  visual_tools_.reset(new mvt::MoveItVisualTools(planning_scene_monitor_->getRobotModel()->getModelFrame(),
                                                 "/moveit_manipulation/markers", planning_scene_monitor_));

  visual_tools_->loadRobotStatePub("/moveit_manipulation/robot_state");
  visual_tools_->loadTrajectoryPub("/moveit_manipulation/display_trajectory");
  visual_tools_->loadMarkerPub();
  visual_tools_->setAlpha(0.8);
  visual_tools_->deleteAllMarkers();  // clear all old markers
  visual_tools_->setManualSceneUpdating(true);
  visual_tools_->hideRobot();  // show that things have been reset
}

}  // end namespace
