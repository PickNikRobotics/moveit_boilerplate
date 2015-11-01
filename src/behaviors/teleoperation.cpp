/* Author: Dave Coleman <dave@dav.ee>
   Desc:   Teleoperation
*/

// Command line arguments
//#include <gflags/gflags.h>

// moveit_manipulation
#include <moveit_manipulation/behaviors/teleoperation.h>

// Parameter loading
#include <ros_param_utilities/ros_param_utilities.h>

// MoveIt
#include <moveit/robot_state/conversions.h>

namespace moveit_manipulation
{
Teleoperation::Teleoperation() : MoveItBoilerplate()
{
  // Load rosparams
  const std::string parent_name = "teleoperation";  // for namespacing logging messages
  ros::NodeHandle teleop_nh("~/teleoperation");
  using namespace ros_param_utilities;
  getAffine3dParameter(parent_name, teleop_nh, "ee_offset", ee_offset_);
  getDoubleParameter(parent_name, teleop_nh, "ik_consistency_limit", ik_consistency_limit_);
  getDoubleParameter(parent_name, teleop_nh, "ik_timeout", ik_timeout_);
  getDoubleParameter(parent_name, teleop_nh, "ik_attempts", ik_attempts_);
  getDoubleParameter(parent_name, teleop_nh, "ik_cartesian_max_step", ik_cartesian_max_step_);
  getDoubleParameter(parent_name, teleop_nh, "ik_cartesian_jump_threshold",
                     ik_cartesian_jump_threshold_);
  getDoubleParameter(parent_name, teleop_nh, "visualization_rate", visualization_rate_);
  getDurationParameter(parent_name, teleop_nh, "execution_delay", execution_delay_);
  getBoolParameter(parent_name, teleop_nh, "debug/ik_rate", debug_ik_rate_);
  getBoolParameter(parent_name, teleop_nh, "debug/command_rate", debug_command_rate_);

  // Create consistency limits for IK solving. Pre-build this vector for improved speed
  // This specifies the desired distance between the solution and the seed state
  if (ik_consistency_limit_)
    for (std::size_t i = 0; i < config_->right_arm_->getActiveJointModels().size(); ++i)
      ik_consistency_limits_vector_.push_back(ik_consistency_limit_);

  // Reset stats
  total_ik_duration_ = ros::Duration(0.0);

  // Choose planning group
  arm_jmg_ = config_->right_arm_;

  // End effector parent link (arm tip for ik solving)
  ik_tip_link_ = grasp_datas_[arm_jmg_]->parent_link_;
  
  // Compute cartesian command input
  cartesian_desired_waypoints_.resize(1);

  // Check for kinematic solver
  if (!arm_jmg_->canSetStateFromIK(ik_tip_link_->getName()))
    ROS_ERROR_STREAM_NAMED("teloperation", "No IK Solver loaded - make sure "
                                           "moveit_config/kinamatics.yaml is loaded in this "
                                           "namespace");

  // In separate thread from imarker & Ik solver, send joint commands
  ROS_INFO_STREAM_NAMED("teleoperation", "Teleoperation state command loop ready");

  // Load robot states
  start_planning_state_.reset(new moveit::core::RobotState(*getCurrentState()));
  //ik_teleop_state_.reset(new moveit::core::RobotState(*start_planning_state_));
  visualize_goal_state_.reset(new moveit::core::RobotState(*start_planning_state_));

  // Show interactive marker
  remote_control_->initializeInteractiveMarkers(chooseNewIMarkerPose());

  // Set a callback function
  remote_control_->setInteractiveMarkerCallback(
      std::bind(&Teleoperation::processIMarkerPose, this, std::placeholders::_1));

  // Create thread for publishing to rviz
  non_realtime_loop_ = nh_.createTimer(ros::Duration(1.0 / visualization_rate_),
                                       &Teleoperation::visualizationThread, this);

  // Create threads for IK solving and commanding joints
  ik_thread_ = std::thread(&Teleoperation::solveIKThread, this);
  command_joints_thread_ = std::thread(&Teleoperation::commandJointsThread, this);
}

Teleoperation::~Teleoperation()
{
  std::cout << "DESTRUCTOR " << std::endl;
  ik_thread_.join();
  command_joints_thread_.join();
}

void Teleoperation::commandJointsThread()
{
  while (ros::ok())
  {
    if (!has_state_to_command_)
    {
      usleep(10);  // 10000 = 1 sec
      continue;
    }
    has_state_to_command_ = false;

    // Optionally start timer
    ros::Time begin_time;
    if (debug_command_rate_)
      begin_time = ros::Time::now();

    // Do computation
    commandJointsThreadHelper();

    // Optionally end timer
    if (debug_command_rate_)
    {
      ros::Duration duration = (ros::Time::now() - begin_time);
      total_command_duration_ += duration;
      total_commands_++;

      ROS_INFO_STREAM_NAMED("teleoperation","Command duration: " << duration.toSec()
                            << ", average duration: " << total_command_duration_.toSec() / total_commands_);
    }
  }
}

void Teleoperation::commandJointsThreadHelper()
{
  // std::cout << "Output traj:\n " << trajectory_msg_ << std::endl;

  // Get read-only mutex lock
  {
    boost::shared_lock<boost::shared_mutex> lock(trajectory_msg_mutex_);
    trajectory_msg_copy_ = trajectory_msg_;  // Make a copy of datastructure
  }

  // Execute
  const bool wait_for_execution = false;
  trajectory_msg_timestamp_ = ros::Time::now();
  if (!planning_interface_->getExecutionInterface()->executeTrajectory(trajectory_msg_copy_,
                                                                       arm_jmg_, wait_for_execution))
  {
    ROS_ERROR_STREAM_NAMED("teleoperation", "Failed to execute trajectory");
  }
}

/*
void Teleoperation::commandJointsThreadHelper2()
{
  // Wait for previous trajectory to finish
  // planning_interface_->getExecutionInterface()->waitForExecution();

  // Assign joint names
  //trajectory_msg_.joint_trajectory.joint_names = arm_jmg_->getActiveJointModelNames();

  // Initialize trajectory message
  // TODO move this into constructor??
  trajectory_msg_.joint_trajectory.points.resize(1);
  trajectory_msg_.joint_trajectory.points[0].positions.resize(
                                                              arm_jmg_->getActiveJointModels().size());
  trajectory_msg_.joint_trajectory.points[0].velocities.clear();
  trajectory_msg_.joint_trajectory.points[0].accelerations.clear();

  // Assign duration (some small number)
  trajectory_msg_.joint_trajectory.points[0].time_from_start = ros::Duration(0.00001);

  // Get read-only mutex lock
  {
    // boost::shared_lock<boost::shared_mutex> lock(trajectory_msg_mutex_);
    // Assign joint values
    visualize_goal_state_->copyJointGroupPositions(arm_jmg_,
                                            trajectory_msg_.joint_trajectory.points[0].positions);
  }

  // Add more points
  if (false)
  {
    // Add more waypoints
    robot_trajectory::RobotTrajectoryPtr robot_traj(
                                                    new
robot_trajectory::RobotTrajectory(robot_model_, arm_jmg_));
    robot_traj->setRobotTrajectoryMsg(*getCurrentState(), trajectory_msg_);

    // Add current state to trajectory
    double dummy_dt = 1.0;
    robot_traj->addPrefixWayPoint(getCurrentState(), dummy_dt); // TODO getCurrentState() is
dangerous

    // Debug
    if (false)
    {
      // Convert trajectory back to a message
      robot_traj->getRobotTrajectoryMsg(trajectory_msg_);

      std::cout << "BEFORE INTERPOLATE: \n" << trajectory_msg_ << std::endl;
    }

    // Interpolate
    double discretization = 0.1;
    planning_interface_->interpolate(robot_traj, discretization);

    // Debug
    if (false)
    {
      // Convert trajectory back to a message
      robot_traj->getRobotTrajectoryMsg(trajectory_msg_);

      std::cout << "BEFORE PARAM: \n" << trajectory_msg_ << std::endl;
    }

    // Perform iterative parabolic smoothing
    const double max_vel_scaling_factor = 0.8;
    planning_interface_->getIterativeSmoother().computeTimeStamps(*robot_traj,
                                                            max_vel_scaling_factor);

    // Convert trajectory back to a message
    robot_traj->getRobotTrajectoryMsg(trajectory_msg_);

    // First time_from_start cannnot be zero, otherwise controller throws error TODO why?
    trajectory_msg_.joint_trajectory.points.front().time_from_start = ros::Duration(0.00000001);

    // std::cout << "AFTER PARAM: \n" << trajectory_msg_ << std::endl;

    // Clear all points that do not have increasing time
    if (trajectory_msg_.joint_trajectory.points.size() > 0)  // Make sure we have at least one point
      for (std::size_t i = 1; i < trajectory_msg_.joint_trajectory.points.size(); ++i)
      {
        if (trajectory_msg_.joint_trajectory.points[i - 1].time_from_start ==
            trajectory_msg_.joint_trajectory.points[i].time_from_start)
        {
          // std::cout << "found where time_from_start repeats: " << i << std::endl;
          // remove all following points
          trajectory_msg_.joint_trajectory.points.resize(i);
          break;
        }
      }

    std::cout << "AFTER CLIP: \n" << trajectory_msg_ << std::endl;
  }

  // Execute
  const bool wait_for_execution = false;
  if (!planning_interface_->getExecutionInterface()->executeTrajectory(trajectory_msg_, arm_jmg_,
                                                                 wait_for_execution))
  {
    ROS_ERROR_STREAM_NAMED("teleoperation", "Failed to execute trajectory");
  }
}
*/

void Teleoperation::solveIKThread()
{
  while (ros::ok())
  {
    if (!has_pose_to_ik_solve_)
    {
      usleep(10);  // 10000 = 1 sec
      continue;
    }
    has_pose_to_ik_solve_ = false;

    // Optionally start timer
    ros::Time begin_time;
    if (debug_ik_rate_)
      begin_time = ros::Time::now();

    // Do computation
    solveIKThreadHelper();

    // Optionally end timer
    if (debug_ik_rate_)
    {
      ros::Duration duration = (ros::Time::now() - begin_time);
      total_ik_duration_ += duration;
      total_ik_attempts_++;

      ROS_INFO_STREAM_NAMED("teleoperation","IK duration: " << duration.toSec()
                            << ", average duration: " << total_ik_duration_.toSec() / total_ik_attempts_
                            << ", attempts: " << ik_attempts_ << ", timeout: " << ik_timeout_);
    }
  }
}

void Teleoperation::solveIKThreadHelper()
{
  // Get predicted state of robot in near future
  bool found_point = false;
  if (trajectory_msg_.joint_trajectory.points.size() > 1)  // has previous trajectory
  {
    // How far in the future to grab a trajectory point
    ros::Duration time_from_last_trajectory =
        ros::Time::now() - trajectory_msg_timestamp_ + execution_delay_;

    // Loop through previous points until the predicted current one is found
    for (std::size_t i = 0; i < trajectory_msg_.joint_trajectory.points.size(); ++i)
    {
      if (trajectory_msg_.joint_trajectory.points[i].time_from_start > time_from_last_trajectory)
      {
        // const trajectory_msgs::JointTrajectoryPoint &point =
        // trajectory_msg_.joint_trajectory.points[i];
        // std::cout << "Point: \n" << point << std::endl;
        ROS_INFO_STREAM_NAMED("teleoperation", "Predicted point is "
                                                   << i << " out of "
                                                   << trajectory_msg_.joint_trajectory.points.size());

        // std::cout << "sending joint trajectory: \n" << trajectory_msg_.joint_trajectory <<
        // std::endl;

        if (!moveit::core::jointTrajPointToRobotState(trajectory_msg_.joint_trajectory, i,
                                                      *start_planning_state_))
        {
          ROS_ERROR_STREAM_NAMED("teleoperation", "Error converting trajectory point to robot "
                                                  "state");
          return;
        }

        // planning_interface_->getVisualGoalState()->publishRobotState(getCurrentState(),
        // rvt::ORANGE);

        found_point = true;
        break;
      }
    }  // for loop
    if (!found_point)
    {
      ROS_WARN_STREAM_NAMED("teleoperation", "Did not find an predicted current trajectory point out of " << trajectory_msg_.joint_trajectory.points.size() << " points." <<
                            "Using current state");
      *start_planning_state_ = *getCurrentState();  // deep copy
    }
  } 
  else
  {
    ROS_WARN_STREAM_NAMED("teleoperation", "No previous trajectory points exist (first iteration?). "
                                           "Using current state");    
    *start_planning_state_ = *getCurrentState();  // deep copy
  }


  // Get read-only mutex lock
  {
    boost::shared_lock<boost::shared_mutex> lock(desired_ee_pose_mutex_);    
    // Command input from the imarkers
    cartesian_desired_waypoints_.front() = desired_ee_pose_;  
  }

  std::vector<moveit::core::RobotStatePtr> cartesian_traj;

  // Plan smooth cartesian path using IK only
  if (!computeCartesianWaypointPath(start_planning_state_, cartesian_desired_waypoints_, cartesian_traj))
  {
    ROS_WARN_STREAM_NAMED("teleoperation", "Unable to plan cartesian path TODO");
    // TODO - still execute as much as possible
    return;
  }

  // ROS_INFO_STREAM_NAMED("teleoperation", "Created " << cartesian_traj.size() << "
  // cartesian_traj");

  // Replace first RobotState with one that has current velocity and accelerations
  cartesian_traj.front() = start_planning_state_;

  // Get trajectory message
  const double vel_scaling_factor = 0.8;
  if (!convertRobotStatesToTrajectory(cartesian_traj, trajectory_msg_, vel_scaling_factor))
  {
    ROS_ERROR_STREAM_NAMED("teleoperation", "Failed to convert to parameterized trajectory");
    return;
  }

  // Copy solution to command RobotState
  *visualize_goal_state_ = *cartesian_traj.back();  // deep copy

  // Tell other threads new state is ready to be used
  has_state_to_command_ = true;
  has_state_to_visualize_ = true;
}

/*
void Teleoperation::solveIKThreadHelper2() // old version
{
  // Debug settings
  static bool collision_checking_verbose = false;
  if (collision_checking_verbose)
    ROS_WARN_STREAM_NAMED("teleoperation", "collision_checking_verbose turned on");

  bool found_solution = true;

  // Setup collision checking with a locked planning scene
  {
    // TODO add collision checking
    // boost::scoped_ptr<planning_scene_monitor::LockedPlanningSceneRO> ls;
    // ls.reset(new planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_));
    // bool only_check_self_collision = true;
    // moveit::core::GroupStateValidityCallbackFn constraint_fn = boost::bind(
    //     &isStateValid, static_cast<const planning_scene::PlanningSceneConstPtr&>(*ls).get(),
    //     collision_checking_verbose, only_check_self_collision, visual_tools_, _1, _2, _3);

    // Solve IK problem for arm
    // TODO desired_ee_pose_ may not be thread safe?
    if (!ik_teleop_state_->setFromIK(arm_jmg_, desired_ee_pose_, ik_tip_link_->getName(),
                                     ik_consistency_limits_vector_, ik_attempts_,
                                     ik_timeout_))  //, constraint_fn))
    {
      static std::size_t warning_counter = 0;
      ROS_WARN_STREAM_NAMED("manipulation", "Unable to find arm solution for desired pose "
                            << warning_counter++);
      found_solution = false;
    }
  }  // end scoped pointer of locked planning scene

  // Exit if no solution found
  if (!found_solution)
    return;

  // Get write mutex lock
  {
    // boost::lock_guard<boost::shared_mutex> lock(trajectory_msg_mutex_);
    // Copy solution to command RobotState
    *visualize_goal_state_ = *ik_teleop_state_;  // deep copy
  }

  // Tell other threads new state is ready to be used
  has_state_to_command_ = true;
  has_state_to_visualize_ = true;
}
*/

bool Teleoperation::computeCartesianWaypointPath(const moveit::core::RobotStatePtr start_state,
                                                 const EigenSTL::vector_Affine3d& waypoints,
                                                 std::vector<moveit::core::RobotStatePtr>& cartesian_traj)
{
  // Cartesian settings
  // const bool collision_checking_verbose = false;
  // const bool only_check_self_collision = false;
  const bool global_reference_frame = true;  // Reference frame setting

  // Create a temp state that doesn't have velocity or accelerations
  moveit::core::RobotState cartesian_state(*start_state);

  // Clear both the velocities and accelerations because this state is copied for the cartesian path
  // but the vel & accel are left the original (incorrect) values
  std::vector<double> zero_variables(cartesian_state.getVariableCount(), 0.0);
  cartesian_state.setVariableVelocities(zero_variables);
  cartesian_state.setVariableAccelerations(zero_variables);

  // Results
  double last_valid_percentage;

  std::size_t attempts = 0;
  static const std::size_t MAX_IK_ATTEMPTS = ik_attempts_;
  bool valid_path_found = false;
  while (attempts < MAX_IK_ATTEMPTS)
  {
    if (attempts > 0)
      ROS_DEBUG_STREAM_NAMED("teleoperation", "Attempting IK solution, attempt # " << attempts + 1);
    attempts++;

    // Collision check
    /*
    boost::scoped_ptr<planning_scene_monitor::LockedPlanningSceneRO> ls;
    ls.reset(new planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_));
    moveit::core::GroupStateValidityCallbackFn constraint_fn = boost::bind(
        &isStateValid, static_cast<const planning_scene::PlanningSceneConstPtr&>(*ls).get(),
        collision_checking_verbose, only_check_self_collision, visual_tools_, _1, _2, _3);
    */
    // moveit::core::GroupStateValidityCallbackFn constraint_fn;

    // Compute Cartesian Path
    cartesian_traj.clear();
    last_valid_percentage = cartesian_state.computeCartesianPath(
        arm_jmg_, cartesian_traj, ik_tip_link_, waypoints, global_reference_frame,
        ik_cartesian_max_step_,
        ik_cartesian_jump_threshold_);  //, constraint_fn, kinematics::KinematicsQueryOptions());

    // clang-format off
    ROS_DEBUG_STREAM_NAMED("teleoperation", "Cartesian last_valid_percentage: " << last_valid_percentage 
                           << " number of points in trajectory: " << cartesian_traj.size());  // clang-format on

    double min_allowed_valid_percentage = 0.9;
    if (last_valid_percentage == 0)
    {
      ROS_DEBUG_STREAM_NAMED("teleoperation", "Failed to computer cartesian path: "
                                              "last_valid_percentage is 0");
    }
    else if (last_valid_percentage < min_allowed_valid_percentage)
    {
      ROS_DEBUG_STREAM_NAMED("teleoperation", "Resulting cartesian path is less than "
                                                  << min_allowed_valid_percentage
                                                  << " % of the desired distance, % valid: "
                                                  << last_valid_percentage);
    }
    else
    {
      ROS_DEBUG_STREAM_NAMED("teleoperation", "Found valid cartesian path");
      valid_path_found = true;
      return true;
    }
  }  // end while AND scoped pointer of locked planning scenep

  if (!valid_path_found)
  {
    ROS_INFO_STREAM_NAMED("teleoperation", "UNABLE to find valid waypoint cartesian path after "
                                               << MAX_IK_ATTEMPTS << " attempts");
    return false;
  }
  return true;
}

bool Teleoperation::convertRobotStatesToTrajectory(
    const std::vector<moveit::core::RobotStatePtr>& robot_state_traj,
    moveit_msgs::RobotTrajectory& trajectory_msg, const double& vel_scaling_factor,
    bool use_interpolation)
{
  ROS_DEBUG_STREAM_NAMED("teleoperation.superdebug", "convertRobotStatesToTrajectory()");

  // Copy the vector of RobotStates to a RobotTrajectory
  robot_trajectory::RobotTrajectoryPtr robot_traj(
      new robot_trajectory::RobotTrajectory(robot_model_, arm_jmg_));

  // -----------------------------------------------------------------------------------------------
  // Convert to RobotTrajectory datatype
  for (std::size_t k = 0; k < robot_state_traj.size(); ++k)
  {
    double duration_from_previous = 1;  // this is overwritten and unimportant
    robot_traj->addSuffixWayPoint(robot_state_traj[k], duration_from_previous);
  }

  if (robot_traj->getFirstWayPoint().hasVelocities())
    ROS_DEBUG_STREAM_NAMED("teleoperation.convert", "First waypoint has velocity");

  static const std::size_t MIN_TRAJECTORY_POINTS = 5;
  if (robot_traj->getWayPointCount() < MIN_TRAJECTORY_POINTS)
  {
    ROS_INFO_STREAM_NAMED("teleoperation", "Too few points (" << robot_traj->getWayPointCount()
                                                              << ")");
  }

  // Interpolate any path with two few points
  if (use_interpolation)
  {
    // Interpolate between each point
    // double discretization = 0.25;
    // interpolate(robot_traj, discretization);
  }

  bool debug = false;
  if (debug)
  {
    // Convert trajectory to a message
    moveit_msgs::RobotTrajectory trajectory_msg_debug;
    robot_traj->getRobotTrajectoryMsg(trajectory_msg_debug);
    std::cout << "Before Iterative smoother: " << trajectory_msg_debug << std::endl;
  }

  // Perform iterative parabolic smoothing
  planning_interface_->getIterativeSmoother().computeTimeStamps(*robot_traj, vel_scaling_factor);

  // Get write mutex lock
  {
    boost::lock_guard<boost::shared_mutex> lock(trajectory_msg_mutex_);

    // Convert trajectory to a message
    robot_traj->getRobotTrajectoryMsg(trajectory_msg);
  }

  // std::cout << "After Iterative smoother: " << trajectory_msg << std::endl;

  return true;
}

void Teleoperation::visualizationThread(const ros::TimerEvent& e)
{
  // Check if there is anything to visualize
  if (!has_state_to_visualize_)
    return;

  has_state_to_visualize_ = false;

  visual_tools_->publishRobotState(visualize_goal_state_, rvt::BLUE);
  planning_interface_->getVisualStartState()->publishRobotState(start_planning_state_, rvt::GREEN);
}

geometry_msgs::Pose Teleoperation::chooseNewIMarkerPose()
{
  // Check that we have a current state
  if (!getCurrentState())
    ROS_ERROR_STREAM_NAMED("teleoperation", "No current state");

  Eigen::Affine3d imarker_start_pose =
      getCurrentState()->getGlobalLinkTransform(grasp_datas_[arm_jmg_]->parent_link_);

  // Move marker to tip of fingers
  imarker_start_pose = imarker_start_pose * ee_offset_.inverse();

  return visual_tools_->convertPose(imarker_start_pose);  // Not thread safe!
}

void Teleoperation::processIMarkerPose(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  using namespace visualization_msgs;

  switch (feedback->event_type)
  {
    case InteractiveMarkerFeedback::BUTTON_CLICK:
      break;
    // --------------------------------------------------------------------------

    case InteractiveMarkerFeedback::MENU_SELECT:
      switch (feedback->menu_entry_id)
      {
        case 1:  // Reset
          ROS_INFO_STREAM_NAMED("teleoperation", "Reset imaker");
          // User has requested to reset location of interactive marker
          remote_control_->updateMarkerPose(chooseNewIMarkerPose());
          break;
        case 2:  // Start
          ROS_INFO_STREAM_NAMED("teleoperation", "Set start state");
          // start_ee_pose_ = offsetEEPose(feedback->pose);
          // visual_tools_->publishZArrow(start_ee_pose_, rvt::GREEN);

          // // Save start state // TODO thread safety?
          // *start_planning_state_ = *ik_teleop_state_;  // deep copy

          // // Show Robot State
          // planning_interface_->getVisualStartState()->publishRobotState(start_planning_state_, rvt::GREEN);
          break;
        case 3:  // Goal
          ROS_INFO_STREAM_NAMED("teleoperation", "Set goal state");
          // goal_ee_pose_ = offsetEEPose(feedback->pose);
          // visual_tools_->publishZArrow(goal_ee_pose_, rvt::ORANGE);

          // // Show Robot State // TODO thread safety?
          // planning_interface_->getVisualGoalState()->publishRobotState(ik_teleop_state_, rvt::ORANGE);
          break;
        case 4:  // Plan cartesian
          ROS_INFO_STREAM_NAMED("teleoperation", "Plan cartesian");
          //planCartesianPath();
          break;
        default:
          ROS_WARN_STREAM_NAMED("teleoperation", "Unknown menu id");
      }
      break;
    // --------------------------------------------------------------------------

    case InteractiveMarkerFeedback::MOUSE_DOWN:
      break;

    // --------------------------------------------------------------------------
    case InteractiveMarkerFeedback::POSE_UPDATE:
      // Get write mutex lock
      {
        boost::lock_guard<boost::shared_mutex> lock(desired_ee_pose_mutex_);
        desired_ee_pose_ = offsetEEPose(feedback->pose);
      }

      // Mark pose as ready for ik solving
      has_pose_to_ik_solve_ = true;
      break;

    case InteractiveMarkerFeedback::MOUSE_UP:
      break;
  }
}

// void Teleoperation::planCartesianPath()
// {
//   // Plan from start to goal
//   EigenSTL::vector_Affine3d waypoints;
//   waypoints.push_back(goal_ee_pose_);

//   std::vector<moveit::core::RobotStatePtr> cartesian_traj;

//   if (computeCartesianWaypointPath(start_planning_state_, waypoints, cartesian_traj))
//   {
//     ROS_WARN_STREAM_NAMED("teleoperation", "Unable to plan cartesian path");
//   }

//   ROS_INFO_STREAM_NAMED("teleoperation", "Created " << cartesian_traj.size() << " cartesian_traj");

//   // Publish trajectory
//   // for (std::size_t i = 0; i < cartesian_traj.size(); ++i)
//   // {
//   //   visual_tools_->publishRobotState(cartesian_traj[i]);
//   //   ros::Duration(0.01).sleep();
//   // }

//   // Visualize trajectory in Rviz display
//   const bool wait_for_trajetory = false;
//   const double speed = 0.01;
//   visual_tools_->publishTrajectoryPath(cartesian_traj, arm_jmg_, speed, wait_for_trajetory);
// }

Eigen::Affine3d Teleoperation::offsetEEPose(const geometry_msgs::Pose& pose) const
{
  // TODO convertPose is not thread safe
  Eigen::Affine3d marker_pose = visual_tools_->convertPose(pose);

  // Offset ee pose forward, because interactive marker is a special thing in front of hand
  return marker_pose * ee_offset_;
}

}  // end namespace
