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
   Desc:   Helper functions for using the planning and manipulation
           facilities in MoveIt! with better introspection and feedback
           tools
*/

// MoveItManipulation
#include <moveit_boilerplate/planning_interface.h>

// MoveIt
#include <moveit/collision_detection/world.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/macros/console_colors.h>
#include <moveit/robot_state/conversions.h>

namespace moveit_boilerplate
{
PlanningInterface::PlanningInterface(psm::PlanningSceneMonitorPtr planning_scene_monitor,
                                     ManipulationDataPtr config,
                                     ExecutionInterface execution_interface,
                                     moveit_grasps::GraspDatas grasp_datas,
                                     RemoteControlPtr remote_control, bool fake_execution)
  : nh_("~")
  , planning_scene_monitor_(planning_scene_monitor)
  , config_(config)
  , execution_interface_(execution_interface)
  , grasp_datas_(grasp_datas)
  , remote_control_(remote_control)
{
  // Create initial robot state
  {
    psm::LockedPlanningSceneRO scene(planning_scene_monitor_);  // Lock planning scene
    current_state_.reset(new moveit::core::RobotState(scene->getCurrentState()));
  }  // end scoped pointer of locked planning scene

  // Set shared robot states for all visual tools
  // visual_tools_->setSharedRobotState(current_state_);

  // Set robot model
  robot_model_ = current_state_->getRobotModel();

  // Debug tools for visualizing in Rviz
  loadVisualTools();


  /*
  // Load grasp generator
  grasp_generator_.reset(new moveit_grasps::GraspGenerator(visual_tools_));
  // setStateWithOpenEE(true, current_state_); // so that grasp filter is started up with EE open
  grasp_filter_.reset(new moveit_grasps::GraspFilter(current_state_, visual_tools_));
  grasp_planner_.reset(new moveit_grasps::GraspPlanner(visual_tools_));
  grasp_planner_->setWaitForNextStepCallback(
      boost::bind(&moveit_boilerplate::RemoteControl::waitForNextStep, remote_control_, _1));
  */

  // Done
  ROS_INFO_STREAM_NAMED("manipulation", "PlanningInterface Ready.");
}

bool PlanningInterface::moveToSRDFPose(JointModelGroup* arm_jmg, const std::string& pose_name,
                                       double velocity_scaling_factor, bool check_validity)
{
  ROS_DEBUG_STREAM_NAMED("manipulation.superdebug", "moveToSRDFPose()");

  // Set new state to current state
  getCurrentState();

  // Set goal state to initial pose
  moveit::core::RobotStatePtr goal_state(
      new moveit::core::RobotState(*current_state_));  // Allocate robot states
  if (!goal_state->setToDefaultValues(arm_jmg, pose_name))
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Failed to set pose '" << pose_name
                                                                  << "' for planning group '"
                                                                  << arm_jmg->getName() << "'");
    return false;
  }

  // Plan
  bool execute_trajectory = true;
  bool verbose = false;
  if (!move(current_state_, goal_state, arm_jmg, velocity_scaling_factor, verbose,
            execute_trajectory, check_validity))
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Unable to move to new position");
    return false;
  }

  return true;
}

bool PlanningInterface::moveToSRDFPoseNoPlan(JointModelGroup* arm_jmg, const std::string& pose_name,
                                             double duration)
{
  ROS_DEBUG_STREAM_NAMED("manipulation.superdebug", "moveToSRDFPose()");

  // Set new state to current state
  getCurrentState();

  // Set goal state to initial pose
  moveit::core::RobotStatePtr goal_state(
      new moveit::core::RobotState(*current_state_));  // Allocate robot states
  if (!goal_state->setToDefaultValues(arm_jmg, pose_name))
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Failed to set pose '" << pose_name
                                                                  << "' for planning group '"
                                                                  << arm_jmg->getName() << "'");
    return false;
  }

  // Get EE pose of goal stae
  Eigen::Affine3d goal_world_to_ee =
      goal_state->getGlobalLinkTransform(grasp_datas_[arm_jmg]->parent_link_);

  Eigen::Affine3d goal_base_to_ee;
  transformWorldToBase(goal_world_to_ee, goal_base_to_ee);

  // Move robot
  execution_interface_->executePose(goal_base_to_ee, arm_jmg, duration);

  return true;
}

bool PlanningInterface::moveToEEPose(const Eigen::Affine3d& ee_pose, double velocity_scaling_factor,
                                     JointModelGroup* arm_jmg)
{
  // Create start and goal
  getCurrentState();
  moveit::core::RobotStatePtr goal_state(new moveit::core::RobotState(*current_state_));

  if (!getRobotStateFromPose(ee_pose, goal_state, arm_jmg))
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Unable to get robot state from pose");
    return false;
  }

  // Plan to this position
  bool verbose = true;
  bool execute_trajectory = true;
  bool check_validity = true;
  if (!move(current_state_, goal_state, arm_jmg, velocity_scaling_factor, verbose,
            execute_trajectory, check_validity))
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Failed to move EE to desired pose");
    return false;
  }

  ROS_INFO_STREAM_NAMED("manipulation", "Moved EE to desired pose successfullly");

  return true;
}

bool PlanningInterface::move(const moveit::core::RobotStatePtr& start,
                             const moveit::core::RobotStatePtr& goal, JointModelGroup* arm_jmg,
                             double velocity_scaling_factor, bool verbose, bool execute_trajectory,
                             bool check_validity)
{
  ROS_INFO_STREAM_NAMED("manipulation.move", "Planning to new pose with velocity scale "
                                                 << velocity_scaling_factor);

  // Check validity of start and goal
  if (check_validity && !checkCollisionAndBounds(start, goal))
  {
    ROS_ERROR_STREAM_NAMED("manipulation.move", "Potential issue with start and goal state, but "
                                                "perhaps this should not fail in the future");
    return false;
  }
  else if (!check_validity)
    ROS_WARN_STREAM_NAMED("manipulation.move", "Start/goal state collision checking for move() was "
                                               "disabled");

  // Visualize start and goal
  if (verbose)
  {
    visual_start_state_->publishRobotState(start, rvt::GREEN);
    visual_goal_state_->publishRobotState(goal, rvt::ORANGE);
  }

  // Check if already in new position
  if (statesEqual(*start, *goal, arm_jmg))
  {
    ROS_INFO_STREAM_NAMED("manipulation", "Not planning motion because current state and goal "
                                          "state are close enough.");
    return true;
  }

  // Do motion plan
  moveit_msgs::RobotTrajectory trajectory_msg;
  std::size_t plan_attempts = 0;
  while (ros::ok())
  {
    if (plan_attempts > 0)
      ROS_WARN_STREAM_NAMED("manipulation", "Previous plan attempt failed, trying again on attempt "
                                                << plan_attempts);

    if (plan(start, goal, arm_jmg, velocity_scaling_factor, verbose, trajectory_msg))
    {
      // Plan succeeded
      break;
    }
    plan_attempts++;
    if (plan_attempts > 5)
    {
      ROS_ERROR_STREAM_NAMED("manipulation", "Max number of plan attempts reached, giving up");
      return false;
    }
  }

  // Hack: do not allow a two point trajectory to be executed because there is no velcity?
  if (trajectory_msg.joint_trajectory.points.size() < 3)
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Trajectory only has "
                                               << trajectory_msg.joint_trajectory.points.size()
                                               << " points");

    if (trajectory_msg.joint_trajectory.points.size() == 2)
    {
      // Remove previous parameterization
      for (std::size_t i = 0; i < trajectory_msg.joint_trajectory.points.size(); ++i)
      {
        trajectory_msg.joint_trajectory.points[i].velocities.clear();
        trajectory_msg.joint_trajectory.points[i].accelerations.clear();
      }

      // Add more waypoints
      robot_trajectory::RobotTrajectoryPtr robot_traj(
          new robot_trajectory::RobotTrajectory(robot_model_, arm_jmg));
      robot_traj->setRobotTrajectoryMsg(*current_state_, trajectory_msg);

      // Interpolate
      double discretization = 0.25;
      interpolate(robot_traj, discretization);

      // Convert trajectory back to a message
      robot_traj->getRobotTrajectoryMsg(trajectory_msg);

      std::cout << "BEFORE PARAM: \n" << trajectory_msg << std::endl;

      // Perform iterative parabolic smoothing
      iterative_smoother_.computeTimeStamps(*robot_traj, config_->main_velocity_scaling_factor_);

      // Convert trajectory back to a message
      robot_traj->getRobotTrajectoryMsg(trajectory_msg);
    }
  }

  // Execute trajectory
  bool wait_for_execution = false;
  if (execute_trajectory)  // TODO remove this feature and replace with the unit testing ability?
  {
    if (!execution_interface_->executeTrajectory(trajectory_msg, arm_jmg, wait_for_execution))
    {
      ROS_ERROR_STREAM_NAMED("manipulation", "Failed to execute trajectory");
      return false;
    }
  }
  else
  {
    ROS_WARN_STREAM_NAMED("manipulation", "Trajectory not executed as because was requested not "
                                          "to");
  }

  // Do processing while trajectory is execute
  planPostProcessing();

  // Wait for trajectory execution to finish
  if (execute_trajectory && !wait_for_execution)
  {
    if (!execution_interface_->waitForExecution())
    {
      ROS_ERROR_STREAM_NAMED("manipulation", "Failed to execute trajectory");
      return false;
    }
  }

  return true;
}

bool PlanningInterface::createPlanningRequest(planning_interface::MotionPlanRequest& request,
                                              const moveit::core::RobotStatePtr& start,
                                              const moveit::core::RobotStatePtr& goal,
                                              JointModelGroup* arm_jmg,
                                              double velocity_scaling_factor)
{
  moveit::core::robotStateToRobotStateMsg(*start, request.start_state);

  // Create Goal constraint
  double tolerance_pose = 0.0001;
  moveit_msgs::Constraints goal_constraint =
      kinematic_constraints::constructGoalConstraints(*goal, arm_jmg, tolerance_pose, tolerance_pose);
  request.goal_constraints.push_back(goal_constraint);

  // Other settings e.g. OMPL
  request.planner_id = "RRTConnectkConfigDefault";
  // request.planner_id = "RRTstarkConfigDefault";
  request.group_name = arm_jmg->getName();
  if (config_->use_experience_setup_)
    request.num_planning_attempts =
        1;  // this must be one else it threads and doesn't use lightning/thunder correctly
  else
    request.num_planning_attempts = 3;  // this is also the number of threads to use
  request.allowed_planning_time = config_->planning_time_;  // seconds
  request.use_experience = config_->use_experience_setup_;
  request.experience_method = config_->experience_type_;
  request.max_velocity_scaling_factor = velocity_scaling_factor;

  // Parameters for the workspace that the planner should work inside relative to center of robot
  double workspace_size = 1;
  request.workspace_parameters.header.frame_id = robot_model_->getModelFrame();
  request.workspace_parameters.min_corner.x =
      start->getVariablePosition("virtual_joint/trans_x") - workspace_size;
  request.workspace_parameters.min_corner.y =
      start->getVariablePosition("virtual_joint/trans_y") - workspace_size;
  request.workspace_parameters.min_corner.z =
      0;  // floor start->getVariablePosition("virtual_joint/trans_z") - workspace_size;
  request.workspace_parameters.max_corner.x =
      start->getVariablePosition("virtual_joint/trans_x") + workspace_size;
  request.workspace_parameters.max_corner.y =
      start->getVariablePosition("virtual_joint/trans_y") + workspace_size;
  request.workspace_parameters.max_corner.z =
      start->getVariablePosition("virtual_joint/trans_z") + workspace_size;
  // visual_tools_->publishWorkspaceParameters(request.workspace_parameters);

  return true;
}

bool PlanningInterface::plan(const moveit::core::RobotStatePtr& start,
                             const moveit::core::RobotStatePtr& goal, JointModelGroup* arm_jmg,
                             double velocity_scaling_factor, bool verbose,
                             moveit_msgs::RobotTrajectory& trajectory_msg)
{
  // Create motion planning request
  planning_interface::MotionPlanRequest request;
  planning_interface::MotionPlanResponse result;

  createPlanningRequest(request, start, goal, arm_jmg, config_->main_velocity_scaling_factor_);

  // Call pipeline
  std::vector<std::size_t> dummy;

  // SOLVE
  loadPlanningPipeline();  // always call before using planning_pipeline_
  planning_scene::PlanningScenePtr cloned_scene;
  {
    psm::LockedPlanningSceneRO scene(planning_scene_monitor_);  // Lock planning scene
    cloned_scene = planning_scene::PlanningScene::clone(scene);
  }  // end scoped pointer of locked planning scene

  planning_pipeline_->generatePlan(cloned_scene, request, result, dummy, planning_context_handle_);

  // Get the trajectory
  moveit_msgs::MotionPlanResponse response;
  response.trajectory = moveit_msgs::RobotTrajectory();
  result.getMessage(response);
  trajectory_msg = response.trajectory;

  // Check that the planning was successful
  bool error = (result.error_code_.val != result.error_code_.SUCCESS);
  if (error)
  {
    ROS_ERROR_STREAM_NAMED("manipulation",
                           "Planning failed:: " << getActionResultString(
                               result.error_code_, trajectory_msg.joint_trajectory.points.empty()));
    return false;
  }

  return true;
}

bool PlanningInterface::planPostProcessing() { return true; }

bool PlanningInterface::interpolate(robot_trajectory::RobotTrajectoryPtr robot_traj,
                                    const double& discretization)
{
  double dummy_dt = 1;  // dummy value until parameterization

  robot_trajectory::RobotTrajectoryPtr new_robot_traj(
      new robot_trajectory::RobotTrajectory(robot_model_, robot_traj->getGroup()));
  std::size_t original_num_waypoints = robot_traj->getWayPointCount();

  // Error check
  if (robot_traj->getWayPointCount() < 2)
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Unable to interpolate between less than two states");
    return false;
  }

  // Debug
  // for (std::size_t i = 0; i < robot_traj->getWayPointCount(); ++i)
  // {
  //   moveit::core::robotStateToStream(robot_traj->getWayPoint(i), std::cout, false);
  // }
  // std::cout << "-------------------------------------------------------" << std::endl;

  // For each set of points (A,B) in the original trajectory
  for (std::size_t i = 0; i < robot_traj->getWayPointCount() - 1; ++i)
  {
    // Add point A to final trajectory
    new_robot_traj->addSuffixWayPoint(robot_traj->getWayPoint(i), dummy_dt);

    for (double t = discretization; t < 1; t += discretization)
    {
      // Create new state
      moveit::core::RobotStatePtr interpolated_state(
          new moveit::core::RobotState(robot_traj->getFirstWayPoint()));
      // Fill in new values
      robot_traj->getWayPoint(i).interpolate(robot_traj->getWayPoint(i + 1), t, *interpolated_state);
      // Add to trajectory
      new_robot_traj->addSuffixWayPoint(interpolated_state, dummy_dt);
      // std::cout << "inserting " << t << " at " << new_robot_traj->getWayPointCount() <<
      // std::endl;
    }
  }

  // Add final waypoint
  new_robot_traj->addSuffixWayPoint(robot_traj->getLastWayPoint(), dummy_dt);

  // Debug
  // for (std::size_t i = 0; i < new_robot_traj->getWayPointCount(); ++i)
  // {
  //   moveit::core::robotStateToStream(new_robot_traj->getWayPoint(i), std::cout, false);
  // }

  std::size_t modified_num_waypoints = new_robot_traj->getWayPointCount();
  ROS_DEBUG_STREAM_NAMED("manipulation.interpolation", "Interpolated trajectory from "
                                                           << original_num_waypoints << " to "
                                                           << modified_num_waypoints);

  // Copy back to original datastructure
  *robot_traj = *new_robot_traj;

  return true;
}

std::string PlanningInterface::getActionResultString(const moveit_msgs::MoveItErrorCodes& error_code,
                                                     bool planned_trajectory_empty)
{
  if (error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    if (planned_trajectory_empty)
      return "Requested path and goal constraints are already met.";
    else
    {
      return "Solution was found and executed.";
    }
  }
  else if (error_code.val == moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME)
    return "Must specify group in motion plan request";
  else if (error_code.val == moveit_msgs::MoveItErrorCodes::PLANNING_FAILED ||
           error_code.val == moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN)
  {
    if (planned_trajectory_empty)
      return "No motion plan found. No execution attempted.";
    else
      return "Motion plan was found but it seems to be invalid (possibly due to postprocessing). "
             "Not executing.";
  }
  else if (error_code.val == moveit_msgs::MoveItErrorCodes::UNABLE_TO_AQUIRE_SENSOR_DATA)
    return "Motion plan was found but it seems to be too costly and looking around did not help.";
  else if (error_code.val ==
           moveit_msgs::MoveItErrorCodes::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE)
    return "Solution found but the environment changed during execution and the path was aborted";
  else if (error_code.val == moveit_msgs::MoveItErrorCodes::CONTROL_FAILED)
    return "Solution found but controller failed during execution";
  else if (error_code.val == moveit_msgs::MoveItErrorCodes::TIMED_OUT)
    return "Timeout reached";
  else if (error_code.val == moveit_msgs::MoveItErrorCodes::PREEMPTED)
    return "Preempted";
  else if (error_code.val == moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS)
    return "Invalid goal constraints";
  else if (error_code.val == moveit_msgs::MoveItErrorCodes::INVALID_OBJECT_NAME)
    return "Invalid object name";
  else if (error_code.val == moveit_msgs::MoveItErrorCodes::FAILURE)
    return "Catastrophic failure";
  return "Unknown event";
}

bool PlanningInterface::executeState(const moveit::core::RobotStatePtr goal_state,
                                     JointModelGroup* jmg, double velocity_scaling_factor)
{
  // Get the start state
  getCurrentState();

  bool go_fast = true;  // reduce debug output

  // Visualize start/goal
  if (go_fast)
    visual_start_state_->publishRobotState(current_state_, rvt::GREEN);
  visual_goal_state_->publishRobotState(goal_state, rvt::ORANGE);

  // Check if already in new position
  if (statesEqual(*current_state_, *goal_state, jmg))
  {
    ROS_INFO_STREAM_NAMED("manipulation", "Not executing because current state and goal state are "
                                          "close enough.");
    return true;
  }

  // Create trajectory
  std::vector<moveit::core::RobotStatePtr> robot_state_trajectory;
  robot_state_trajectory.push_back(current_state_);

  // Add goal state
  robot_state_trajectory.push_back(goal_state);

  // Get trajectory message
  moveit_msgs::RobotTrajectory trajectory_msg;

  // Convert trajectory to a message
  bool interpolate = false;
  if (!convertRobotStatesToTrajectory(robot_state_trajectory, trajectory_msg, jmg,
                                      velocity_scaling_factor, interpolate))
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Failed to convert to parameterized trajectory");
    return false;
  }

  // Execute
  const bool wait_for_execution = true;
  if (!execution_interface_->executeTrajectory(trajectory_msg, jmg, wait_for_execution))
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Failed to execute trajectory");
    return false;
  }
  return true;
}

bool PlanningInterface::moveDirectToState(const moveit::core::RobotStatePtr goal_state,
                                          JointModelGroup* jmg, double velocity_scaling_factor)
{
  // Visualize goal
  visual_goal_state_->publishRobotState(goal_state, rvt::ORANGE);

  // Get trajectory message
  moveit_msgs::RobotTrajectory trajectory_msg;

  // Assign joint names
  trajectory_msg.joint_trajectory.joint_names = jmg->getActiveJointModelNames();

  // Assign joint values
  trajectory_msg.joint_trajectory.points.resize(1);
  trajectory_msg.joint_trajectory.points[0].positions.resize(jmg->getActiveJointModels().size());
  goal_state->copyJointGroupPositions(jmg, trajectory_msg.joint_trajectory.points[0].positions);

  // Assign duration
  trajectory_msg.joint_trajectory.points[0].time_from_start = ros::Duration(0.05);

  // Debug
  // std::copy(trajectory_msg.joint_trajectory.joint_names.begin(),
  // trajectory_msg.joint_trajectory.joint_names.end(),
  // std::ostream_iterator<std::string>(std::cout, "\n"));
  // std::copy(  trajectory_msg.joint_trajectory.points[0].positions.begin(),
  // trajectory_msg.joint_trajectory.points[0].positions.end(),
  // std::ostream_iterator<double>(std::cout, "\n"));

  // Wait for previous trajectory to finish?
  execution_interface_->waitForExecution();

  // Execute
  const bool wait_for_execution = false;
  if (!execution_interface_->executeTrajectory(trajectory_msg, jmg, wait_for_execution))
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Failed to execute trajectory");
    return false;
  }
  return true;
}

bool PlanningInterface::executeVerticlePathWithIK(JointModelGroup* arm_jmg,
                                                  const double& desired_lift_distance, bool up,
                                                  bool ignore_collision)
{
  Eigen::Vector3d approach_direction;
  approach_direction << 0, 0, (up ? 1 : -1);  // 1 is up, -1 is down
  bool reverse_path = false;

  if (!executeCartesianPath(arm_jmg, approach_direction, desired_lift_distance,
                            config_->lift_velocity_scaling_factor_, reverse_path, ignore_collision))
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Failed to execute horizontal path");
    return false;
  }
  return true;
}

bool PlanningInterface::executeHorizontalPath(JointModelGroup* arm_jmg,
                                              const double& desired_lift_distance, bool left,
                                              bool ignore_collision)
{
  Eigen::Vector3d approach_direction;
  approach_direction << 0, (left ? 1 : -1), 0;  // 1 is left, -1 is right
  bool reverse_path = false;

  if (!executeCartesianPath(arm_jmg, approach_direction, desired_lift_distance,
                            config_->lift_velocity_scaling_factor_, reverse_path, ignore_collision))
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Failed to execute horizontal path");
    return false;
  }
  return true;
}

bool PlanningInterface::executeRetreatPath(JointModelGroup* arm_jmg, double desired_retreat_distance,
                                           bool retreat, bool ignore_collision)
{
  // Compute straight line in reverse from grasp
  Eigen::Vector3d approach_direction;
  approach_direction << (retreat ? -1 : 1), 0, 0;  // backwards towards robot body
  bool reverse_path = false;

  if (!executeCartesianPath(arm_jmg, approach_direction, desired_retreat_distance,
                            config_->retreat_velocity_scaling_factor_, reverse_path, ignore_collision))
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Failed to execute retreat path");
    return false;
  }
  return true;
}

bool PlanningInterface::executeCartesianPath(JointModelGroup* arm_jmg,
                                             const Eigen::Vector3d& direction,
                                             double desired_distance, double velocity_scaling_factor,
                                             bool reverse_path, bool ignore_collision)
{
  getCurrentState();

  // Debug
  // visual_tools_->publishRobotState( current_state_, rvt::PURPLE );
  // visual_start_state_->hideRobot();
  // visual_goal_state_->hideRobot();

  double path_length;
  std::vector<moveit::core::RobotStatePtr> robot_state_trajectory;
  if (!computeStraightLinePath(direction, desired_distance, robot_state_trajectory, current_state_,
                               arm_jmg, reverse_path, path_length, ignore_collision))

  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Error occured while computing straight line path");
    return false;
  }

  // Get approach trajectory message
  moveit_msgs::RobotTrajectory cartesian_trajectory_msg;
  if (!convertRobotStatesToTrajectory(robot_state_trajectory, cartesian_trajectory_msg, arm_jmg,
                                      velocity_scaling_factor))
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Failed to convert to parameterized trajectory");
    return false;
  }

  // Execute
  if (!execution_interface_->executeTrajectory(cartesian_trajectory_msg, arm_jmg))
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Failed to execute trajectory");
    return false;
  }

  return true;
}

bool PlanningInterface::computeStraightLinePath(
    Eigen::Vector3d direction, double desired_distance,
    std::vector<moveit::core::RobotStatePtr>& robot_state_trajectory,
    moveit::core::RobotStatePtr robot_state, JointModelGroup* arm_jmg, bool reverse_trajectory,
    double& last_valid_percentage, bool ignore_collision)
{
  // End effector parent link (arm tip for ik solving)
  const moveit::core::LinkModel* ik_tip_link = grasp_datas_[arm_jmg]->parent_link_;

  // ---------------------------------------------------------------------------------------------
  // Show desired trajectory in BLACK
  Eigen::Affine3d tip_pose_start = robot_state->getGlobalLinkTransform(ik_tip_link);

  // Debug
  if (false)
  {
    std::cout << "Tip Pose Start \n" << tip_pose_start.translation().x() << "\t"
              << tip_pose_start.translation().y() << "\t" << tip_pose_start.translation().z()
              << std::end;l
  }

  // Visualize start and goal state
  bool verbose = false;
  if (verbose)
  {
    visual_tools_->publishSphere(tip_pose_start, rvt::RED, rvt::LARGE);

    // Get desired end pose
    Eigen::Affine3d tip_pose_end;
    straightProjectPose(tip_pose_start, tip_pose_end, direction, desired_distance);

    visual_tools_->publishLine(tip_pose_start, tip_pose_end, rvt::BLACK, rvt::REGULAR);

    // Show start and goal states of cartesian path
    if (reverse_trajectory)
    {
      // The passed in robot state is the goal
      visual_start_state_->hideRobot();
      visual_goal_state_->publishRobotState(robot_state, rvt::ORANGE);
    }
    else
    {
      // The passed in robot state is the start (retreat)
      visual_goal_state_->hideRobot();
      visual_start_state_->publishRobotState(robot_state, rvt::GREEN);
    }
  }

  // ---------------------------------------------------------------------------------------------
  // Settings for computeCartesianPath

  // Resolution of trajectory
  double max_step = 0.01;  // 0.01 // The maximum distance in Cartesian space between consecutive
                           // points on the resulting path

  // Error check
  if (desired_distance < max_step)
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Not enough: desired_distance ("
                                               << desired_distance << ")  < max_step (" << max_step
                                               << ")");
    return false;
  }

  // Jump threshold for preventing consequtive joint values from 'jumping' by a large amount in
  // joint space
  double jump_threshold = config_->jump_threshold_;  // aka jump factor

  bool collision_checking_verbose = false;

  // Reference frame setting
  bool global_reference_frame = true;

  // Check for kinematic solver
  if (!arm_jmg->canSetStateFromIK(ik_tip_link->getName()))
    ROS_ERROR_STREAM_NAMED("manipulation", "No IK Solver loaded - make sure "
                                           "moveit_config/kinamatics.yaml is loaded in this "
                                           "namespace");

  std::size_t attempts = 0;
  static const std::size_t MAX_IK_ATTEMPTS = 4;
  while (attempts < MAX_IK_ATTEMPTS)
  {
    if (attempts > 0)
    {
      std::cout << std::endl;
      ROS_INFO_STREAM_NAMED("manipulation", "Attempting IK solution, attempts # " << attempts);
    }
    if (attempts > MAX_IK_ATTEMPTS - 2)
    {
      ROS_WARN_STREAM_NAMED("manipulation", "Enabling collision check verbose");
      collision_checking_verbose = true;
    }
    attempts++;

    bool only_check_self_collision = false;
    if (ignore_collision)
    {
      only_check_self_collision = true;
      ROS_INFO_STREAM_NAMED("manipulation", "computeStraightLinePath() is ignoring collisions with "
                                            "world objects (but not robot links)");
    }

    // Collision check
    boost::scoped_ptr<psm::LockedPlanningSceneRO> ls;
    ls.reset(new psm::LockedPlanningSceneRO(planning_scene_monitor_));
    moveit::core::GroupStateValidityCallbackFn constraint_fn = boost::bind(
        &isStateValid, static_cast<const planning_scene::PlanningSceneConstPtr&>(*ls).get(),
        collision_checking_verbose, only_check_self_collision, visual_tools_, _1, _2, _3);

    // Compute Cartesian Path
    // this is the Cartesian pose we start from, and have to move in the direction indicated
    const Eigen::Affine3d& start_pose = robot_state->getGlobalLinkTransform(ik_tip_link);

    // the direction can be in the local reference frame (in which case we rotate it)
    const Eigen::Vector3d rotated_direction =
        global_reference_frame ? direction : start_pose.rotation() * direction;

    // The target pose is built by applying a translation to the start pose for the desired
    // direction and distance
    Eigen::Affine3d target_pose = start_pose;
    target_pose.translation() += rotated_direction * desired_distance;

    robot_state_trajectory.clear();
    last_valid_percentage =
        robot_state->computeCartesianPath(arm_jmg, robot_state_trajectory, ik_tip_link, target_pose,
                                          true, max_step, jump_threshold, constraint_fn);

    ROS_DEBUG_STREAM_NAMED("manipulation", "Cartesian last_valid_percentage: "
                                               << last_valid_percentage
                                               << ", number of states in trajectory: "
                                               << robot_state_trajectory.size());

    double min_allowed_valid_percentage = 0.9;
    if (last_valid_percentage == 0)
    {
      ROS_WARN_STREAM_NAMED("manipulation", "Failed to computer cartesian path: "
                                            "last_valid_percentage is 0");
    }
    else if (last_valid_percentage < min_allowed_valid_percentage)
    {
      ROS_DEBUG_STREAM_NAMED("manipulation.waypoints", "Resulting cartesian path is less than "
                                                           << min_allowed_valid_percentage
                                                           << " % of the desired distance, % "
                                                              "valid: " << last_valid_percentage);
    }
    else
    {
      ROS_INFO_STREAM_NAMED("manipulation", "Found valid cartesian path");
      break;
    }
  }  // end while AND scoped pointer of locked planning scene

  // Check if we never found a path
  if (attempts >= MAX_IK_ATTEMPTS)
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Never found a valid cartesian path, aborting");
    return false;
  }

  // Reverse the trajectory if neeeded
  if (reverse_trajectory)
  {
    std::reverse(robot_state_trajectory.begin(), robot_state_trajectory.end());

    // Also, save the first state so that generateApproachPath() can use it
    first_state_in_trajectory_ = robot_state_trajectory.front();
  }

  // Debug
  if (verbose)
  {
    // Super debug
    if (false)
    {
      std::cout << "Tip Pose Result: \n";
      for (std::size_t i = 0; i < robot_state_trajectory.size(); ++i)
      {
        const Eigen::Affine3d tip_pose_start =
            robot_state_trajectory[i]->getGlobalLinkTransform(ik_tip_link);
        std::cout << tip_pose_start.translation().x() << "\t" << tip_pose_start.translation().y()
                  << "\t" << tip_pose_start.translation().z() << std::endl;
      }
    }

    // Show actual trajectory in GREEN
    ROS_INFO_STREAM_NAMED("manipulation", "Displaying cartesian trajectory in green");
    const Eigen::Affine3d& tip_pose_end =
        robot_state_trajectory.back()->getGlobalLinkTransform(ik_tip_link);
    visual_tools_->publishLine(tip_pose_start, tip_pose_end, rvt::LIME_GREEN, rvt::LARGE);
    visual_tools_->publishSphere(tip_pose_end, rvt::ORANGE, rvt::LARGE);

    // Visualize end effector position of cartesian path
    ROS_INFO_STREAM_NAMED("manipulation", "Visualize end effector position of cartesian path");
    visual_tools_->publishTrajectoryPoints(robot_state_trajectory, ik_tip_link);

    // Show start and goal states of cartesian path
    if (reverse_trajectory)
    {
      // The passed in robot state is the goal
      visual_start_state_->publishRobotState(robot_state_trajectory.front(), rvt::GREEN);
    }
    else
    {
      // The passed in robot state is the start (retreat)
      visual_goal_state_->publishRobotState(robot_state_trajectory.back(), rvt::ORANGE);
    }
  }

  return true;
}

JointModelGroup* PlanningInterface::chooseArm(const Eigen::Affine3d& ee_pose)
{
  // Single Arm
  if (!config_->dual_arm_)
  {
    return config_->right_arm_;  // right is always the default arm for single arm robots
  }
  // Dual Arm
  else if (ee_pose.translation().y() < 0)
  {
    ROS_DEBUG_STREAM_NAMED("manipulation", "Using right arm for task");
    return config_->right_arm_;
  }
  else
  {
    ROS_DEBUG_STREAM_NAMED("manipulation", "Using left arm for task");
    return config_->left_arm_;
  }
}

bool PlanningInterface::getRobotStateFromPose(const Eigen::Affine3d& ee_pose,
                                              moveit::core::RobotStatePtr& robot_state,
                                              JointModelGroup* arm_jmg, double consistency_limit)
{
  // Setup collision checking with a locked planning scene
  {
    static bool collision_checking_verbose =
        config_->isEnabled("get_robot_state_from_pose__collision_checking_verbose");
    if (collision_checking_verbose)
      ROS_WARN_STREAM_NAMED("manipulation.getRobotStateFromPose", "collision_checking_verbose "
                                                                  "turned on");

    boost::scoped_ptr<psm::LockedPlanningSceneRO> ls;
    ls.reset(new psm::LockedPlanningSceneRO(planning_scene_monitor_));
    bool only_check_self_collision = true;
    moveit::core::GroupStateValidityCallbackFn constraint_fn = boost::bind(
        &isStateValid, static_cast<const planning_scene::PlanningSceneConstPtr&>(*ls).get(),
        collision_checking_verbose, only_check_self_collision, visual_tools_, _1, _2, _3);

    // Solve IK problem for arm
    std::size_t attempts = 0;  // use default
    double timeout = 0;        // use default

    // Create consistency limits TODO cache
    // This specifies the desired distance between the solution and the seed state
    std::vector<double> consistency_limits_vector;
    if (consistency_limit)
      // TODO hard coded njoints
      for (std::size_t i = 0; i < arm_jmg->getActiveJointModels().size(); ++i)
        consistency_limits_vector.push_back(consistency_limit);

    const moveit::core::LinkModel* ik_tip_link = grasp_datas_[arm_jmg]->parent_link_;
    if (!robot_state->setFromIK(arm_jmg, ee_pose, ik_tip_link->getName(), consistency_limits_vector,
                                attempts, timeout, constraint_fn))
    {
      // visual_tools_->publishZArrow(ee_pose, rvt::RED);
      static std::size_t warning_counter = 0;
      ROS_WARN_STREAM_NAMED("manipulation", "Unable to find arm solution for desired pose "
                                                << warning_counter++);
      return false;
    }
  }  // end scoped pointer of locked planning scene

  // ROS_DEBUG_STREAM_NAMED("manipulation","Found solution to pose request");
  return true;
}

bool PlanningInterface::straightProjectPose(const Eigen::Affine3d& original_pose,
                                            Eigen::Affine3d& new_pose,
                                            const Eigen::Vector3d direction, double distance)
{
  ROS_DEBUG_STREAM_NAMED("manipulation.superdebug", "straightProjectPose()");

  // Assume everything is in world coordinates
  new_pose = original_pose;
  Eigen::Vector3d longer_direction = direction * distance;
  new_pose.translation() += longer_direction;

  return true;
}

bool PlanningInterface::convertRobotStatesToTrajectory(
    const std::vector<moveit::core::RobotStatePtr>& robot_state_traj,
    moveit_msgs::RobotTrajectory& trajectory_msg, JointModelGroup* jmg,
    const double& velocity_scaling_factor, bool use_interpolation)
{
  ROS_DEBUG_STREAM_NAMED("manipulation.superdebug", "convertRobotStatesToTrajectory()");

  // Copy the vector of RobotStates to a RobotTrajectory
  robot_trajectory::RobotTrajectoryPtr robot_traj(
      new robot_trajectory::RobotTrajectory(robot_model_, jmg));

  // -----------------------------------------------------------------------------------------------
  // Convert to RobotTrajectory datatype
  for (std::size_t k = 0; k < robot_state_traj.size(); ++k)
  {
    double duration_from_previous = 1;  // this is overwritten and unimportant
    robot_traj->addSuffixWayPoint(robot_state_traj[k], duration_from_previous);
  }

  if (robot_traj->getFirstWayPoint().hasVelocities())
    ROS_DEBUG_STREAM_NAMED("manipulation.convert", "First waypoint has velocity");

  // Interpolate any path with two few points
  if (use_interpolation)
  {
    static const std::size_t MIN_TRAJECTORY_POINTS = 20;
    if (robot_traj->getWayPointCount() < MIN_TRAJECTORY_POINTS)
    {
      ROS_INFO_STREAM_NAMED("manipulation", "Interpolating trajectory because two few points ("
                                                << robot_traj->getWayPointCount() << ")");

      // Interpolate between each point
      double discretization = 0.25;
      interpolate(robot_traj, discretization);
    }
  }

  bool debug = false;
  if (debug)
  {
    // Convert trajectory to a message
    robot_traj->getRobotTrajectoryMsg(trajectory_msg);
    std::cout << "Before Iterative smoother: " << trajectory_msg << std::endl;
  }

  // Perform iterative parabolic smoothing
  iterative_smoother_.computeTimeStamps(*robot_traj, velocity_scaling_factor);

  // Convert trajectory to a message
  robot_traj->getRobotTrajectoryMsg(trajectory_msg);

  // std::cout << "After Iterative smoother: " << trajectory_msg << std::endl;

  return true;
}

bool PlanningInterface::openEEs(bool open)
{
  ROS_DEBUG_STREAM_NAMED("manipulation.superdebug", "openEEs()");

  // First arm
  if (!openEE(open, config_->right_arm_))
    return false;

  // Second arm if applicable
  if (config_->dual_arm_)
    if (!openEE(open, config_->left_arm_))
      return false;

  return true;
}

bool PlanningInterface::openEE(bool open, JointModelGroup* arm_jmg)
{
  if (open)
  {
    return setEEGraspPosture(grasp_datas_[arm_jmg]->pre_grasp_posture_, arm_jmg);
  }
  else
  {
    return setEEGraspPosture(grasp_datas_[arm_jmg]->grasp_posture_, arm_jmg);
  }
}

bool PlanningInterface::setEEJointPosition(double joint_position, JointModelGroup* arm_jmg)
{
  std::vector<double> joint_positions;
  for (std::size_t i = 0; i < grasp_datas_[arm_jmg]->ee_jmg_->getVariableCount(); ++i)
  {
    joint_positions.push_back(joint_position);
    // TODO ignore tip joints
  }

  trajectory_msgs::JointTrajectory grasp_posture;
  grasp_datas_[arm_jmg]->jointPositionsToGraspPosture(joint_positions, grasp_posture);

  return setEEGraspPosture(grasp_posture, arm_jmg);
}

bool PlanningInterface::setEEGraspPosture(trajectory_msgs::JointTrajectory grasp_posture,
                                          JointModelGroup* arm_jmg)
{
  ROS_DEBUG_STREAM_NAMED("manipulation.end_effector", "Moving to grasp posture:\n" << grasp_posture);

  // Check status
  if (!config_->isEnabled("end_effector_enabled"))
  {
    ROS_WARN_STREAM_NAMED("manipulation", "Gripping is disabled");
    return true;
  }

  getCurrentState();
  robot_trajectory::RobotTrajectoryPtr ee_trajectory(
      new robot_trajectory::RobotTrajectory(robot_model_, grasp_datas_[arm_jmg]->ee_jmg_));

  ROS_INFO_STREAM_NAMED("manipulation", "Sending command to end effector "
                                            << grasp_datas_[arm_jmg]->ee_jmg_->getName());

  // Add goal state
  ee_trajectory->setRobotTrajectoryMsg(*current_state_, grasp_posture);

  // Add start state to trajectory
  double dummy_dt = 1;
  ee_trajectory->addPrefixWayPoint(current_state_, dummy_dt);

  // Check if already in new position
  if (statesEqual(ee_trajectory->getFirstWayPoint(), ee_trajectory->getLastWayPoint(),
                  grasp_datas_[arm_jmg]->ee_jmg_))
  {
    ROS_INFO_STREAM_NAMED(
        "manipulation",
        "Not executing motion because current state and goal state are close enough for group "
            << grasp_datas_[arm_jmg]->ee_jmg_->getName());

    return true;
  }

  // Interpolate between each point
  double discretization = 0.1;
  interpolate(ee_trajectory, discretization);

  // Perform iterative parabolic smoothing
  double ee_velocity_scaling_factor = 0.1;
  iterative_smoother_.computeTimeStamps(*ee_trajectory, ee_velocity_scaling_factor);

  // Show the change in end effector
  bool verbose = false;
  if (verbose)
  {
    visual_start_state_->publishRobotState(current_state_, rvt::GREEN);
    visual_goal_state_->publishRobotState(ee_trajectory->getLastWayPoint(), rvt::ORANGE);
  }

  // Convert trajectory to a message
  moveit_msgs::RobotTrajectory trajectory_msg;
  ee_trajectory->getRobotTrajectoryMsg(trajectory_msg);

  // Execute trajectory
  if (!execution_interface_->executeTrajectory(trajectory_msg, arm_jmg))
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Failed to execute grasp trajectory");
    return false;
  }

  return true;
}

ExecutionInterfacePtr PlanningInterface::getExecutionInterface() { return execution_interface_; }

bool PlanningInterface::fixCollidingState(planning_scene::PlanningScenePtr cloned_scene)
{
  ROS_DEBUG_STREAM_NAMED("manipulation.superdebug", "fixCollidingState()");

  // Turn off auto mode
  // remote_control_->setFullAutonomous(false);

  // Open hand to ensure we aren't holding anything anymore
  if (!openEEs(true))
  {
    ROS_WARN_STREAM_NAMED("manipulation", "Unable to open end effectors");
    // return false;
  }

  JointModelGroup* arm_jmg = config_->dual_arm_ ? config_->left_arm_ : config_->right_arm_;

  // Decide what direction is needed to fix colliding state, using the cloned scene
  collision_detection::CollisionResult::ContactMap contacts;
  cloned_scene->getCollidingPairs(contacts);

  std::string colliding_world_object = "";
  for (collision_detection::CollisionResult::ContactMap::const_iterator contact_it = contacts.begin();
       contact_it != contacts.end(); contact_it++)
  {
    // const std::string& body_id_1 = contact_it->first.first;
    // const std::string& body_id_2 = contact_it->first.second;
    // std::cout << "body_id_1: " << body_id_1 << std::endl;
    // std::cout << "body_id_2: " << body_id_2 << std::endl;

    const std::vector<collision_detection::Contact>& contacts = contact_it->second;

    for (std::size_t i = 0; i < contacts.size(); ++i)
    {
      const collision_detection::Contact& contact = contacts[i];

      // Find the world object that is the problem
      if (contact.body_type_1 == collision_detection::BodyTypes::WORLD_OBJECT)
      {
        colliding_world_object = contact.body_name_1;
        break;
      }
      if (contact.body_type_2 == collision_detection::BodyTypes::WORLD_OBJECT)
      {
        colliding_world_object = contact.body_name_2;
        break;
      }
    }
    if (!colliding_world_object.empty())
      break;
  }

  if (colliding_world_object.empty())
  {
    ROS_WARN_STREAM_NAMED("manipulation", "Did not find any world objects in collision. Attempting "
                                          "to move home");
    bool check_validity = false;
    return moveToStartPosition(NULL, check_validity);
  }

  ROS_INFO_STREAM_NAMED("manipulation", "World object " << colliding_world_object << " in "
                                                                                     "collision");

  // Categorize this world object
  bool reverse_out = false;
  bool raise_up = false;
  bool move_in_right = false;
  bool move_in_left = false;
  std::cout << "substring is: " << colliding_world_object.substr(0, 7) << std::endl;

  // if shelf or product, reverse out
  if (colliding_world_object.substr(0, 7) == "product")
  {
    reverse_out = true;
  }
  else if (colliding_world_object.substr(0, 7) == "front_w")  // front_wall
  {
    reverse_out = true;
  }
  else if (colliding_world_object.substr(0, 7) == "shelf")  // TODO string name
  {
    reverse_out = true;
  }
  // if goal bin, raise up
  else if (colliding_world_object.substr(0, 7) == "goal_bin")  // TODO string name
  {
    raise_up = true;
  }
  // Right wall
  else if (colliding_world_object.substr(0, 7) == "right_w")
  {
    move_in_left = true;
  }
  // Left wall
  else if (colliding_world_object.substr(0, 7) == "left_w")
  {
    move_in_right = true;
  }
  else
  {
    int mode = visual_tools_->iRand(0, 3);
    ROS_WARN_STREAM_NAMED(
        "manipulation",
        "Unknown object, not sure how to handle. Performing random action using mode " << mode);

    if (mode == 0)
      reverse_out = true;
    else if (mode == 1)
      raise_up = true;
    else if (mode == 2)
      move_in_left = true;
    else  // mode = 3
      move_in_right = true;
  }

  if (raise_up)
  {
    ROS_INFO_STREAM_NAMED("manipulation", "Raising up");
    double desired_distance = 0.2;
    bool up = true;
    bool ignore_collision = true;
    if (!executeVerticlePathWithIK(arm_jmg, desired_distance, up, ignore_collision))
    {
      return false;
    }
  }
  else if (reverse_out)
  {
    ROS_INFO_STREAM_NAMED("manipulation", "Reversing out");
    double desired_distance = 0.1;
    bool restreat = true;
    bool ignore_collision = true;
    if (!executeRetreatPath(arm_jmg, desired_distance, restreat, ignore_collision))
    {
      return false;
    }
  }
  else if (move_in_left)
  {
    ROS_INFO_STREAM_NAMED("manipulation", "Moving in left");
    double desired_distance = 0.2;
    bool left = true;
    bool ignore_collision = true;
    if (!executeHorizontalPath(arm_jmg, desired_distance, left, ignore_collision))
    {
      return false;
    }
  }
  else if (move_in_right)
  {
    ROS_INFO_STREAM_NAMED("manipulation", "Moving in right");
    double desired_distance = 0.2;
    bool left = false;
    bool ignore_collision = true;
    if (!executeHorizontalPath(arm_jmg, desired_distance, left, ignore_collision))
    {
      return false;
    }
  }

  return true;
}

bool PlanningInterface::moveToStartPosition(JointModelGroup* arm_jmg, bool check_validity)
{
  // Choose which planning group to use
  if (arm_jmg == NULL)
    arm_jmg = config_->dual_arm_ ? config_->both_arms_ : config_->right_arm_;
  return moveToSRDFPose(arm_jmg, config_->start_pose_, config_->main_velocity_scaling_factor_,
                        check_validity);
}

void PlanningInterface::loadPlanningPipeline()
{
  ROS_DEBUG_STREAM_NAMED("manipulation.superdebug", "loadPlanningPipeline()");

  if (!planning_pipeline_)
  {
    // Setup planning pipeline
    planning_pipeline_.reset(new planning_pipeline::PlanningPipeline(
        robot_model_, nh_, "planning_plugin", "request_adapters"));
  }
}

bool PlanningInterface::statesEqual(const moveit::core::RobotState& s1,
                                    const moveit::core::RobotState& s2, JointModelGroup* jmg)
{
  static const double STATES_EQUAL_THRESHOLD = 0.01;

  double s1_vars[jmg->getVariableCount()];
  double s2_vars[jmg->getVariableCount()];
  s1.copyJointGroupPositions(jmg, s1_vars);
  s2.copyJointGroupPositions(jmg, s2_vars);

  for (std::size_t i = 0; i < jmg->getVariableCount(); ++i)
  {
    const moveit::core::JointModel* this_joint = jmg->getJointModel(jmg->getVariableNames()[i]);

    std::vector<const moveit::core::JointModel*>::const_iterator joint_it;
    joint_it = std::find(jmg->getActiveJointModels().begin(), jmg->getActiveJointModels().end(),
                         this_joint);

    // Make sure joint is active
    if (joint_it != jmg->getActiveJointModels().end())
    {
      if (fabs(s1_vars[i] - s2_vars[i]) > STATES_EQUAL_THRESHOLD)
      {
        // std::cout << "    statesEqual: Variable " << jmg->getVariableNames()[i] << " beyond
        // threshold, diff: "
        // << fabs(s1_vars[i] - s2_vars[i]) << std::endl;
        return false;
      }
    }
  }

  return true;
}

moveit::core::RobotStatePtr PlanningInterface::getCurrentState()
{
  // Get the real current state
  psm::LockedPlanningSceneRO scene(planning_scene_monitor_);  // Lock planning scene
  (*current_state_) = scene->getCurrentState();
  return current_state_;
}

bool PlanningInterface::waitForRobotToStop(const double& timeout)
{
  ROS_INFO_STREAM_NAMED("manipulation", "Waiting for robot to stop moving");
  ros::Time when_to_stop = ros::Time::now() + ros::Duration(timeout);

  static const double UPDATE_RATE = 0.1;  // how often to check if robot is stopped
  static const double POSITION_ERROR_THRESHOLD = 0.002;
  // static const std::size_t REQUIRED_STABILITY_PASSES = 4; // how many times it must be within
  // threshold in a row
  std::size_t stability_passes = 0;
  double error;
  // Get the current position
  moveit::core::RobotState previous_position = *getCurrentState();  // copy the memory
  moveit::core::RobotState current_position = previous_position;    // copy the memory

  while (ros::ok())
  {
    ros::Duration(UPDATE_RATE).sleep();
    ros::spinOnce();

    current_position = *getCurrentState();  // copy the memory

    // Check if all positions are near zero
    bool stopped = true;
    for (std::size_t i = 0; i < current_position.getVariableCount(); ++i)
    {
      error = fabs(previous_position.getVariablePositions()[i] -
                   current_position.getVariablePositions()[i]);

      if (error > POSITION_ERROR_THRESHOLD)
      {
        ROS_DEBUG_STREAM_NAMED("manipulation", "Robot is still moving with error "
                                                   << error << " on variable " << i);
        stopped = false;
      }
    }
    if (stopped)
    {
      stability_passes++;
      // ROS_INFO_STREAM_NAMED("manipulation","On stability pass " << stability_passes);
    }
    else
    {
      // Reset the stability passes because this round didn't pass
      stability_passes = 0;
    }

    if (stability_passes > 2)
      return true;

    // Check timeout
    if (ros::Time::now() > when_to_stop)
    {
      ROS_WARN_STREAM_NAMED("manipulation", "Timed out while waiting for robot to stop");
      return false;
    }

    // Copy newest positions
    previous_position = current_position;  // copy the memory
  }

  return false;
}

bool PlanningInterface::fixCurrentCollisionAndBounds(JointModelGroup* arm_jmg)
{
  // ROS_INFO_STREAM_NAMED("manipulation","Checking current collision and bounds");

  bool result = true;

  // Copy planning scene that is locked
  planning_scene::PlanningScenePtr cloned_scene;
  {
    psm::LockedPlanningSceneRO scene(planning_scene_monitor_);
    cloned_scene = planning_scene::PlanningScene::clone(scene);
    (*current_state_) = scene->getCurrentState();
  }

  // Check for collisions
  bool verbose = false;
  if (cloned_scene->isStateColliding(*current_state_, arm_jmg->getName(), verbose))
  {
    result = false;

    // std::cout << std::endl;
    // std::cout << "-------------------------------------------------------" << std::endl;
    ROS_WARN_STREAM_NAMED("manipulation", "State is colliding, attempting to fix...");

    // Show collisions
    visual_tools_->publishContactPoints(*current_state_, cloned_scene.get());
    visual_tools_->publishRobotState(current_state_, rvt::RED);

    // Attempt to fix collision state
    if (!fixCollidingState(cloned_scene))
    {
      ROS_ERROR_STREAM_NAMED("manipulation", "Unable to fix colliding state");
    }
  }
  else
  {
    ROS_INFO_STREAM_NAMED("manipulation", "State is not colliding");
  }

  // Check if satisfies bounds
  if (!current_state_->satisfiesBounds(arm_jmg, fix_state_bounds_.getMaxBoundsError()))
  {
    std::cout << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;
    ROS_WARN_STREAM_NAMED("manipulation", "State does not satisfy bounds, attempting to fix...");
    std::cout << "-------------------------------------------------------" << std::endl;

    moveit::core::RobotStatePtr new_state(new moveit::core::RobotState(*current_state_));

    if (!fix_state_bounds_.fixBounds(*new_state, arm_jmg))
    {
      ROS_WARN_STREAM_NAMED("manipulation", "Unable to fix state bounds or change not required");
    }
    else
    {
      // Alert human to error
      remote_control_->setAutonomous(false);

      // State was modified, send to robot
      if (!executeState(new_state, arm_jmg, config_->main_velocity_scaling_factor_))
      {
        ROS_ERROR_STREAM_NAMED("manipulation", "Unable to exceute state bounds fix");
      }
      result = false;
    }
  }
  else
  {
    ROS_INFO_STREAM_NAMED("manipulation", "State satisfies bounds");
  }
  return result;
}

bool PlanningInterface::checkCollisionAndBounds(const moveit::core::RobotStatePtr& start_state,
                                                const moveit::core::RobotStatePtr& goal_state,
                                                bool verbose)
{
  bool result = true;

  // Check if satisfies bounds  --------------------------------------------------------

  // Start
  if (!start_state->satisfiesBounds(fix_state_bounds_.getMaxBoundsError()))
  {
    if (verbose)
    {
      ROS_WARN_STREAM_NAMED("manipulation", "Start state does not satisfy bounds");

      // For debugging in console
      showJointLimits(config_->right_arm_);
    }
    result = false;
  }

  // Goal
  if (goal_state && !goal_state->satisfiesBounds(fix_state_bounds_.getMaxBoundsError()))
  {
    if (verbose)
    {
      ROS_WARN_STREAM_NAMED("manipulation", "Goal state does not satisfy bounds");

      // For debugging in console
      showJointLimits(config_->right_arm_);
    }

    result = false;
  }

  // Check for collisions --------------------------------------------------------
  JointModelGroup* arm_jmg = config_->dual_arm_ ? config_->both_arms_ : config_->right_arm_;

  // Get planning scene lock
  {
    psm::LockedPlanningSceneRO scene(planning_scene_monitor_);
    // Start
    if (scene->isStateColliding(*start_state, arm_jmg->getName(), verbose))
    {
      if (verbose)
      {
        ROS_WARN_STREAM_NAMED("manipulation.checkCollisionAndBounds", "Start state is colliding");
        // Show collisions
        visual_tools_->publishContactPoints(*start_state,
                                            planning_scene_monitor_->getPlanningScene().get());
        visual_tools_->publishRobotState(*start_state, rvt::RED);
      }
      result = false;
    }

    // Goal
    if (goal_state)
    {
      goal_state->update();

      if (scene->isStateColliding(*goal_state, arm_jmg->getName(), verbose))
      {
        if (verbose)
        {
          ROS_WARN_STREAM_NAMED("manipulation.checkCollisionAndBounds", "Goal state is colliding");
          // Show collisions
          visual_tools_->publishContactPoints(*goal_state,
                                              planning_scene_monitor_->getPlanningScene().get());
          visual_tools_->publishRobotState(*goal_state, rvt::RED);
        }
        result = false;
      }
    }
  }

  return result;
}

double PlanningInterface::getMaxJointLimit(const moveit::core::JointModel* joint)
{
  // Assume all joints have only one variable
  if (joint->getVariableCount() > 1)
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Unable to handle joints with more than one variable");
    return 0;
  }

  const moveit::core::VariableBounds& bound = joint->getVariableBounds()[0];

  return bound.max_position_;
}

double PlanningInterface::getMinJointLimit(const moveit::core::JointModel* joint)
{
  // Assume all joints have only one variable
  if (joint->getVariableCount() > 1)
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Unable to handle joints with more than one variable");
    return 0;
  }

  const moveit::core::VariableBounds& bound = joint->getVariableBounds()[0];

  return bound.min_position_;
}

bool PlanningInterface::showJointLimits(JointModelGroup* jmg)
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
    std::cout << " \t" << std::fixed << std::setprecision(5) << bound.max_position_ << "  \t"
              << joints[i]->getName() << " current: " << std::fixed << std::setprecision(5)
              << current_value << std::endl;

    if (out_of_bounds)
      std::cout << MOVEIT_CONSOLE_COLOR_RESET;
  }

  return true;
}

void PlanningInterface::transformWorldToBase(Eigen::Affine3d& pose_world, Eigen::Affine3d& pose_base)
{
  const Eigen::Affine3d& base_to_world =
      getCurrentState()->getGlobalLinkTransform("base_link").inverse();
  pose_base = base_to_world * pose_world;
}

void PlanningInterface::loadVisualTools()
{
  visual_tools_.reset(new mvt::MoveItVisualTools(
      robot_model_->getModelFrame(), "/moveit_boilerplate/markers", planning_scene_monitor_));
  visual_tools_->loadTrajectoryPub("/moveit_boilerplate/display_trajectory");
  visual_tools_->loadMarkerPub();
  visual_tools_->setAlpha(0.8);
  visual_tools_->deleteAllMarkers();  // clear all old markers
  visual_tools_->setManualSceneUpdating(true);

  // Robot Start State
  visual_start_state_.reset(new mvt::MoveItVisualTools(
      robot_model_->getModelFrame(), "/moveit_boilerplate/markers2", planning_scene_monitor_));
  visual_start_state_->loadRobotStatePub("/moveit_boilerplate/robot_start_state");
  visual_start_state_->hideRobot();

  // Robot Goal State
  visual_goal_state_.reset(new mvt::MoveItVisualTools(
      robot_model_->getModelFrame(), "/moveit_boilerplate/markers3", planning_scene_monitor_));
  visual_goal_state_->loadRobotStatePub("/moveit_boilerplate/robot_goal_state");
  visual_goal_state_->hideRobot();
}

}  // end namespace

namespace
{
bool isStateValid(const planning_scene::PlanningScene* planning_scene, bool verbose,
                  bool only_check_self_collision, mvt::MoveItVisualToolsPtr visual_tools,
                  moveit::core::RobotState* robot_state, JointModelGroup* group,
                  const double* ik_solution)
{
  robot_state->setJointGroupPositions(group, ik_solution);
  robot_state->update();

  if (!planning_scene)
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "No planning scene provided");
    return false;
  }
  if (only_check_self_collision)
  {
    // No easy API exists for only checking self-collision, so we do it here. TODO: move this big
    // into planning_scene.cpp
    collision_detection::CollisionRequest req;
    req.verbose = false;
    req.group_name = group->getName();
    collision_detection::CollisionResult res;
    planning_scene->checkSelfCollision(req, res, *robot_state);
    if (!res.collision)
      return true;  // not in collision
  }
  else if (!planning_scene->isStateColliding(*robot_state, group->getName()))
    return true;  // not in collision

  // Display more info about the collision
  if (verbose)
  {
    visual_tools->publishRobotState(*robot_state, rvt::RED);
    planning_scene->isStateColliding(*robot_state, group->getName(), true);
    visual_tools->publishContactPoints(*robot_state, planning_scene);
    ros::Duration(0.4).sleep();
  }
  return false;
}

}  // end annonymous namespace
