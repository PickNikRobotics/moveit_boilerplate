/* Author: Dave Coleman <dave@dav.ee>
   Desc:   Teleoperation
*/

// Command line arguments
//#include <gflags/gflags.h>

// moveit_manipulation
#include <moveit_manipulation/behaviors/teleoperation.h>

// Parameter loading
#include <ros_param_utilities/ros_param_utilities.h>

// C++

namespace moveit_manipulation
{
Teleoperation::Teleoperation()
  : MoveItBoilerplate()
{
  // Load rosparams
  const std::string parent_name = "teleoperation";  // for namespacing logging messages
  std::vector<double> offset_doubles;

  ros::NodeHandle teleop_nh("~/teleoperation");

  using namespace ros_param_utilities;
  getDoubleParameters(parent_name, teleop_nh, "ee_offset", offset_doubles);
  convertDoublesToEigen(parent_name, offset_doubles, ee_offset_);
  getDoubleParameter(parent_name, teleop_nh, "ik_consistency_limit", ik_consistency_limit_);
  getDoubleParameter(parent_name, teleop_nh, "ik_timeout", ik_timeout_);
  getDoubleParameter(parent_name, teleop_nh, "ik_attempts", ik_attempts_);
  getDoubleParameter(parent_name, teleop_nh, "visualization_rate", visualization_rate_);
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

  // Assign joint names
  trajectory_msg_.joint_trajectory.joint_names = arm_jmg_->getActiveJointModelNames();

  // In separate thread from imarker & Ik solver, send joint commands
  ROS_INFO_STREAM_NAMED("teleoperation", "Teleoperation state command loop ready");

  // Load robot states
  start_state_.reset(new moveit::core::RobotState(*getCurrentState()));
  ik_teleop_state_.reset(new moveit::core::RobotState(*start_state_));
  command_state_.reset(new moveit::core::RobotState(*start_state_));

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
      usleep(10); // 10000 = 1 sec
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
      ros::Time end_time = ros::Time::now();
      ros::Duration duration = (end_time - begin_time);
      total_command_duration_ += duration;
      total_commands_++;

      std::cout << "Command duration: " << duration.toSec()
                << ", average duration: " << total_command_duration_.toSec() / total_commands_
                << std::endl;
    }
  }
}

void Teleoperation::commandJointsThreadHelper()
{
  // Plan from start to goal
  EigenSTL::vector_Affine3d waypoints;
  waypoints.push_back(desired_ee_pose_);

  std::vector<moveit::core::RobotStatePtr> cartesian_traj;

  // TODO make current state be in the future
  if (!computeCartesianWaypointPath(arm_jmg_, getCurrentState(), waypoints, cartesian_traj))
  {
    ROS_WARN_STREAM_NAMED("teleoperation", "Unable to plan cartesian path");
    return;
  }

  //ROS_INFO_STREAM_NAMED("teleoperation", "Created " << cartesian_traj.size() << " cartesian_traj"); 

  // Get trajectory message
  double velocity_scaling_factor = 0.9;
  if (!manipulation_->convertRobotStatesToTrajectory(cartesian_traj, trajectory_msg_, arm_jmg_,
                                                     velocity_scaling_factor))
  {
    ROS_ERROR_STREAM_NAMED("teleoperation", "Failed to convert to parameterized trajectory");
    return;
  }

  //std::cout << "Output traj:\n " << trajectory_msg_ << std::endl;

  // Execute
  const bool wait_for_execution = false;
  if (!manipulation_->getExecutionInterface()->executeTrajectory(trajectory_msg_, arm_jmg_,
                                                                 wait_for_execution))
  {
    ROS_ERROR_STREAM_NAMED("teleoperation", "Failed to execute trajectory");
  }
}

void Teleoperation::commandJointsThreadHelper2()
{
  // Wait for previous trajectory to finish
  // manipulation_->getExecutionInterface()->waitForExecution();

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
    // boost::shared_lock<boost::shared_mutex> lock(ik_state_mutex_);
    // Assign joint values
    command_state_->copyJointGroupPositions(arm_jmg_,
                                            trajectory_msg_.joint_trajectory.points[0].positions);
  }

  // Add more points
  if (false)
  {
    // Add more waypoints
    robot_trajectory::RobotTrajectoryPtr robot_traj(
                                                    new robot_trajectory::RobotTrajectory(robot_model_, arm_jmg_));
    robot_traj->setRobotTrajectoryMsg(*getCurrentState(), trajectory_msg_);

    // Add current state to trajectory
    double dummy_dt = 1.0;
    robot_traj->addPrefixWayPoint(getCurrentState(), dummy_dt);

    // Debug
    if (false)
    {
      // Convert trajectory back to a message
      robot_traj->getRobotTrajectoryMsg(trajectory_msg_);

      std::cout << "BEFORE INTERPOLATE: \n" << trajectory_msg_ << std::endl;
    }

    // Interpolate
    double discretization = 0.1;
    manipulation_->interpolate(robot_traj, discretization);

    // Debug
    if (false)
    {
      // Convert trajectory back to a message
      robot_traj->getRobotTrajectoryMsg(trajectory_msg_);

      std::cout << "BEFORE PARAM: \n" << trajectory_msg_ << std::endl;
    }

    // Perform iterative parabolic smoothing
    const double max_velocity_scaling_factor = 0.8;
    manipulation_->getIterativeSmoother().computeTimeStamps(*robot_traj,
                                                            max_velocity_scaling_factor);

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
  if (!manipulation_->getExecutionInterface()->executeTrajectory(trajectory_msg_, arm_jmg_,
                                                                 wait_for_execution))
  {
    ROS_ERROR_STREAM_NAMED("teleoperation", "Failed to execute trajectory");
  }
}

void Teleoperation::solveIKThread()
{
  while (ros::ok())
  {
    if (!has_pose_to_ik_solve_)
    {
      usleep(10); // 10000 = 1 sec
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
      ros::Time end_time = ros::Time::now();
      ros::Duration duration = (end_time - begin_time);
      total_ik_duration_ += duration;
      total_ik_attempts_++;

      std::cout << "IK duration: " << duration.toSec()
                << ", average duration: " << total_ik_duration_.toSec() / total_ik_attempts_
                << ", attempts: " << ik_attempts_ << ", timeout: " << ik_timeout_ << std::endl;
    }
  }
}

void Teleoperation::solveIKThreadHelper()
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
    const moveit::core::LinkModel* ik_tip_link = grasp_datas_[arm_jmg_]->parent_link_;
    if (!ik_teleop_state_->setFromIK(arm_jmg_, desired_ee_pose_, ik_tip_link->getName(),
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
    // boost::lock_guard<boost::shared_mutex> lock(ik_state_mutex_);
    // Copy solution to command RobotState
    *command_state_ = *ik_teleop_state_;  // deep copy
  }

  // Tell other threads new state is ready to be used
  has_state_to_command_ = true;
  has_state_to_visualize_ = true;
}

bool Teleoperation::computeCartesianWaypointPath(
    JointModelGroup* arm_jmg, const moveit::core::RobotStatePtr start_state,
    const EigenSTL::vector_Affine3d& waypoints, std::vector<moveit::core::RobotStatePtr> &cartesian_traj)    
{
  // End effector parent link (arm tip for ik solving)
  const moveit::core::LinkModel* ik_tip_link = grasp_datas_[arm_jmg]->parent_link_;

  // Resolution of trajectory
  const double max_step = 0.01;  // The maximum distance in Cartesian space between consecutive
                                 // points on the resulting path

  // Jump threshold for preventing consequtive joint values from 'jumping' by a large amount in
  // joint space
  const double jump_threshold = config_->jump_threshold_;  // aka jump factor

  // Collision setting
  //const bool collision_checking_verbose = false;
  //const bool only_check_self_collision = false;

  // Reference frame setting
  const bool global_reference_frame = true;

  // Check for kinematic solver
  if (!arm_jmg->canSetStateFromIK(ik_tip_link->getName()))
  {
    ROS_ERROR_STREAM_NAMED("manipulation.waypoints", "No IK Solver loaded - make sure "
                                                     "moveit_config/kinamatics.yaml is loaded in "
                                                     "this namespace");
    return false;
  }

  // Results
  double last_valid_percentage;

  std::size_t attempts = 0;
  static const std::size_t MAX_IK_ATTEMPTS = 5;
  bool valid_path_found = false;
  while (attempts < MAX_IK_ATTEMPTS)
  {
    if (attempts > 0)
    {
      // std::cout << std::endl;
      // std::cout << "-------------------------------------------------------" << std::endl;
      ROS_DEBUG_STREAM_NAMED("manipulation.waypoints", "Attempting IK solution, attempt # "
                                                           << attempts + 1);
    }
    attempts++;

    // Collision check
    /*
    boost::scoped_ptr<planning_scene_monitor::LockedPlanningSceneRO> ls;
    ls.reset(new planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_));
    moveit::core::GroupStateValidityCallbackFn constraint_fn = boost::bind(
        &isStateValid, static_cast<const planning_scene::PlanningSceneConstPtr&>(*ls).get(),
        collision_checking_verbose, only_check_self_collision, visual_tools_, _1, _2, _3);
    */
    //moveit::core::GroupStateValidityCallbackFn constraint_fn;

    // Test
    moveit::core::RobotState temp_state(*start_state);

    // Compute Cartesian Path
    cartesian_traj.clear();
    last_valid_percentage = temp_state.computeCartesianPath(
        arm_jmg, cartesian_traj, ik_tip_link, waypoints, global_reference_frame, max_step,
        jump_threshold); //, constraint_fn, kinematics::KinematicsQueryOptions());

    ROS_DEBUG_STREAM_NAMED("manipulation.waypoints", "Cartesian last_valid_percentage: "
                                                         << last_valid_percentage
                                                         << " number of points in trajectory: "
                                                         << cartesian_traj.size());

    double min_allowed_valid_percentage = 0.9;
    if (last_valid_percentage == 0)
    {
      ROS_DEBUG_STREAM_NAMED("manipulation.waypoints",
                             "Failed to computer cartesian path: last_valid_percentage is 0");
    }
    else if (last_valid_percentage < min_allowed_valid_percentage)
    {
      ROS_DEBUG_STREAM_NAMED("manipulation.waypoints",
                             "Resulting cartesian path is less than "
                                 << min_allowed_valid_percentage
                                 << " % of the desired distance, % valid: "
                                 << last_valid_percentage);
    }
    else
    {
      ROS_DEBUG_STREAM_NAMED("manipulation.waypoints", "Found valid cartesian path");
      valid_path_found = true;
      return true;
    }
  }  // end while AND scoped pointer of locked planning scenep

  if (!valid_path_found)
  {
    ROS_INFO_STREAM_NAMED("manipulation.waypoints",
                          "UNABLE to find valid waypoint cartesian path after " << MAX_IK_ATTEMPTS
                                                                                << " attempts");
    return false;
  }
  return true;
}

void Teleoperation::visualizationThread(const ros::TimerEvent& e)
{
  // Check if there is anything to visualize
  if (!has_state_to_visualize_)
    return;

  has_state_to_visualize_ = false;

  // Get read-only mutex lock
  {
    // boost::shared_lock<boost::shared_mutex> lock(ik_state_mutex_);
    visual_tools_->publishRobotState(command_state_, rvt::BLUE);
  }
}

geometry_msgs::Pose Teleoperation::chooseNewIMarkerPose()
{
  // Check that we have a current state
  if (!getCurrentState())
    ROS_ERROR_STREAM_NAMED("teleoperation", "No current state");

  Eigen::Affine3d imarker_start_pose = getCurrentState()->getGlobalLinkTransform(grasp_datas_[arm_jmg_]->parent_link_);                                                                                 

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
          start_ee_pose_ = offsetEEPose(feedback->pose);
          visual_tools_->publishZArrow(start_ee_pose_, rvt::GREEN);

          // Save start state // TODO thread safety?
          *start_state_ = *ik_teleop_state_;  // deep copy

          // Show Robot State
          manipulation_->getVisualStartState()->publishRobotState(start_state_, rvt::GREEN);
          break;
        case 3:  // Goal
          ROS_INFO_STREAM_NAMED("teleoperation", "Set goal state");
          goal_ee_pose_ = offsetEEPose(feedback->pose);
          visual_tools_->publishZArrow(goal_ee_pose_, rvt::ORANGE);

          // Show Robot State // TODO thread safety?
          manipulation_->getVisualGoalState()->publishRobotState(ik_teleop_state_, rvt::ORANGE);
          break;
        case 4:  // Plan cartesian
          ROS_INFO_STREAM_NAMED("teleoperation", "Plan cartesian");
          planCartesianPath();
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
    case InteractiveMarkerFeedback::MOUSE_UP:

      // Get pose
      desired_ee_pose_ = offsetEEPose(feedback->pose);

      // Mark pose as ready for ik solving
      has_pose_to_ik_solve_ = true;

      break;
  }
}

void Teleoperation::planCartesianPath()
{
  // Plan from start to goal
  EigenSTL::vector_Affine3d waypoints;
  waypoints.push_back(goal_ee_pose_);

  std::vector<moveit::core::RobotStatePtr> cartesian_traj;

  if (computeCartesianWaypointPath(arm_jmg_, start_state_, waypoints, cartesian_traj))
  {
    ROS_WARN_STREAM_NAMED("teleoperation", "Unable to plan cartesian path");
  }

  ROS_INFO_STREAM_NAMED("teleoperation", "Created " << cartesian_traj.size() << " cartesian_traj");
                        

  // Publish trajectory
  // for (std::size_t i = 0; i < cartesian_traj.size(); ++i)
  // {
  //   visual_tools_->publishRobotState(cartesian_traj[i]);
  //   ros::Duration(0.01).sleep();
  // }

  // Visualize trajectory in Rviz display
  const bool wait_for_trajetory = false;
  const double speed = 0.01;
  visual_tools_->publishTrajectoryPath(cartesian_traj, arm_jmg_, speed, wait_for_trajetory);                                       
}

Eigen::Affine3d Teleoperation::offsetEEPose(const geometry_msgs::Pose& pose) const
{
  Eigen::Affine3d marker_pose =
    visual_tools_->convertPose(pose);  // TODO convertPose is not thread safe

  // Offset ee pose forward, because interactive marker is a special thing in front of hand
  return marker_pose * ee_offset_;
}

}  // end namespace
