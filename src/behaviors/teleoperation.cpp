/* Author: Dave Coleman <dave@dav.ee>
   Desc:   Teleoperation
*/

// Command line arguments
//#include <gflags/gflags.h>

// moveit_manipulation
#include <moveit_manipulation/behaviors/teleoperation.h>

// Parameter loading
#include <ros_param_shortcuts/ros_param_shortcuts.h>

// MoveIt
#include <moveit/robot_state/conversions.h>

namespace moveit_manipulation
{
Teleoperation::Teleoperation() : MoveItBoilerplate()
{
  // Load rosparams
  const std::string parent_name = "teleoperation";  // for namespacing logging messages
  ros::NodeHandle teleop_nh("~/teleoperation");
  using namespace ros_param_shortcuts;
  double command_freq;
  double compute_ik_freq;
  getAffine3dParam(parent_name, teleop_nh, "ee_offset", ee_offset_);
  getDoubleParam(parent_name, teleop_nh, "compute_ik_freq", compute_ik_freq);                     
  getDoubleParam(parent_name, teleop_nh, "command_freq", command_freq);                     
  getDoubleParam(parent_name, teleop_nh, "ik_consistency_limit", ik_consistency_limit_);
  getDoubleParam(parent_name, teleop_nh, "ik_timeout", ik_timeout_);
  getDoubleParam(parent_name, teleop_nh, "ik_attempts", ik_attempts_);
  getDoubleParam(parent_name, teleop_nh, "ik_cart_max_step", ik_cart_max_step_);
  getDoubleParam(parent_name, teleop_nh, "ik_cart_jump_threshold", ik_cart_jump_threshold_);
  getDoubleParam(parent_name, teleop_nh, "vel_scaling_factor", vel_scaling_factor_);
  getDoubleParam(parent_name, teleop_nh, "visualization_rate", visualization_rate_);
  getDurationParam(parent_name, teleop_nh, "execution_delay", execution_delay_);
  getBoolParam(parent_name, teleop_nh, "debug/ik_rate", debug_ik_rate_);
  getBoolParam(parent_name, teleop_nh, "debug/command_rate", debug_command_rate_);
  getBoolParam(parent_name, teleop_nh, "debug/generated_traj_rate", debug_generated_traj_rate_);

  // Create consistency limits for IK solving. Pre-build this vector for improved speed
  // This specifies the desired distance between the solution and the seed state
  if (ik_consistency_limit_)
    for (std::size_t i = 0; i < config_->right_arm_->getActiveJointModels().size(); ++i)
      ik_consistency_limits_vector_.push_back(ik_consistency_limit_);

  // Calculate how often to check for a new input pose, in microseconds
  static const double SECOND_TO_MICOSECOND = 1000000.0;
  compute_ik_delay_ = (1.0 / compute_ik_freq) * SECOND_TO_MICOSECOND;    
  ROS_INFO_STREAM_NAMED("teleoperation", "Checking for new IK solution at frequency "
                         << 1.0 / (compute_ik_delay_ / SECOND_TO_MICOSECOND)
                         << " hz");

  // How often to send new joint commands, in microseconds
  command_new_trajectory_delay_ = (1.0 / command_freq) * SECOND_TO_MICOSECOND;
  ROS_INFO_STREAM_NAMED("teleoperation", "Sending joint command trajectories at frequency "
                         << 1.0 / (command_new_trajectory_delay_ / SECOND_TO_MICOSECOND)
                         << " hz");  

  // Set the planning scene monitor to listen to velocity and effort, also
  //planning_scene_monitor_->getStateMonitorNonConst()->copyDynamicsEnabled(true);    

  // Choose planning group
  arm_jmg_ = config_->right_arm_;

  // End effector parent link (arm tip for ik solving)
  ik_tip_link_ = grasp_datas_[arm_jmg_]->parent_link_;

  // Compute cartesian command input
  cart_desired_waypoints_.resize(1);

  // Check for kinematic solver
  if (!arm_jmg_->canSetStateFromIK(ik_tip_link_->getName()))
    ROS_ERROR_STREAM_NAMED("teloperation", "No IK Solver loaded - make sure "
                           "moveit_config/kinamatics.yaml is loaded in this "
                           "namespace");

  // In separate thread from imarker & Ik solver, send joint commands
  ROS_INFO_STREAM_NAMED("teleoperation", "Teleoperation state command loop ready");

  // Load robot states
  start_planning_state_.reset(new moveit::core::RobotState(*getCurrentState()));
  visualize_goal_state_.reset(new moveit::core::RobotState(*start_planning_state_));

  // Show interactive marker
  remote_control_->initializeInteractiveMarkers(chooseNewIMarkerPose());

  // Set a callback function
  remote_control_->setInteractiveMarkerCallback(
                                                std::bind(&Teleoperation::processIMarkerPose, this, std::placeholders::_1));

  // Subscribe to controller state so we can get the desired state
  std::string topic = "/iiwa_7_r800/position_trajectory_controller/state";
  // State subscriber
  state_sub_ = nh_.subscribe<ControllerState>(topic, 1, &Teleoperation::stateCB, this);                                              

  // Create thread for publishing to rviz
  non_realtime_loop_ = nh_.createTimer(ros::Duration(1.0 / visualization_rate_),
                                       &Teleoperation::visualizationThread, this);

  // Create threads for IK solving and commanding joints
  //ik_thread_ = std::thread(&Teleoperation::solveIKThread, this);
  //command_joints_thread_ = std::thread(&Teleoperation::commandJointsThread, this);
}

Teleoperation::~Teleoperation()
{
  ik_thread_.join();
  command_joints_thread_.join();
}

void Teleoperation::solveIKThread()
{
  while (ros::ok())
  {
    if (!has_pose_to_ik_solve_)
    {
      usleep(compute_ik_delay_);  // microseconds
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

      double average_duration = total_ik_duration_.toSec() / total_ik_attempts_;
      ROS_INFO_STREAM_NAMED("teleoperation",
                            "PLAN_IK duration: " << std::fixed << duration.toSec()
                            << "\t avg duration: " << std::fixed << average_duration
                            << "\t avg freq: " << std::fixed << (1.0 / average_duration) << " hz");
    }
  }
}

bool Teleoperation::solveIKThreadHelper()
{
  // Get predicted state of robot in near future
  bool found_point = false;
  if (false && trajectory_msg_.joint_trajectory.points.size() > 1)  // has previous trajectory
  {
    ROS_WARN_STREAM_NAMED("temp","trajectory_msg_timestamp_ is disabled!");
    // How far in the future to grab a trajectory point
    ros::Duration time_from_last_trajectory =
      ros::Time::now() - trajectory_msg_timestamp_;

    // Loop through previous points until the predicted current one is found
    for (std::size_t i = 0; i < trajectory_msg_.joint_trajectory.points.size(); ++i)
    {
      if (trajectory_msg_.joint_trajectory.points[i].time_from_start > time_from_last_trajectory)
      {
        ROS_WARN_STREAM_NAMED("teleoperation", "Predicted point is "
                              << i << " out of "
                              << trajectory_msg_.joint_trajectory.points.size());

        if (!moveit::core::jointTrajPointToRobotState(trajectory_msg_.joint_trajectory, i,
                                                      *start_planning_state_))
        {
          ROS_ERROR_STREAM_NAMED("teleoperation", "Error converting trajectory point to robot "
                                 "state");
          return false;
        }

        // planning_interface_->getVisualGoalState()->publishRobotState(getCurrentState(),
        // rvt::ORANGE);

        found_point = true;
        break;
      }
    }  // for loop
    if (!found_point)
    {
      ROS_WARN_STREAM_NAMED("teleoperation",
                            "Did not find an predicted current trajectory point out of "
                            << trajectory_msg_.joint_trajectory.points.size() << " points. "
                            << "Using current state");
      getDesiredState(start_planning_state_);
    }
  }
  else
  {
    // ROS_WARN_STREAM_NAMED("teleoperation",
    //                       "No previous trajectory points exist (first iteration?). "
    //                       "Using current state");
    getDesiredState(start_planning_state_);
  }

  //std::cout << "START PLANNING STATE:" << std::endl;
  //start_planning_state_->printStateInfo();

  // Get read-only mutex lock
  {
    boost::shared_lock<boost::shared_mutex> lock(desired_ee_pose_mutex_);
    // Command input from the imarkers
    cart_desired_waypoints_.front() = desired_ee_pose_;
  }

  // Check if the desired pose is the same as our current pose
  if (posesEqual(start_planning_state_->getGlobalLinkTransform(ik_tip_link_),
                 cart_desired_waypoints_.front()))
  {
    ROS_WARN_STREAM_NAMED("teleoperation", "Robot already at desired pose, ignoring command");
    return false;
  }

  std::vector<moveit::core::RobotStatePtr> cart_traj;

  // Plan smooth cartesian path using IK only
  if (!computeCartWaypointPath(start_planning_state_, cart_desired_waypoints_, cart_traj))
  {
    ROS_WARN_STREAM_NAMED("teleoperation", "Unable to plan cartesian path TODO");
    // TODO - still execute as much as possible
    return false;
  }

  // ROS_INFO_STREAM_NAMED("teleoperation", "Created " << cart_traj.size() << "
  // cart_traj");

  // Replace first RobotState with one that has current velocity and accelerations
  cart_traj.front() = start_planning_state_;

  // Get trajectory message
  if (!convertRobotStatesToTrajectory(cart_traj, trajectory_msg_))
  {
    ROS_ERROR_STREAM_NAMED("teleoperation", "Failed to convert to parameterized trajectory");
    return false;
  }

  // Statistics on generated trajectory
  if (debug_generated_traj_rate_)
  {
    const trajectory_msgs::JointTrajectory &traj = trajectory_msg_.joint_trajectory;
    //std::cout << "Trajectory:\n " << traj << std::endl;
    // for (std::size_t i = 1; i < traj.points.size() - 1; ++i)
    // {
    //   double delta_t = (traj.points[i].time_from_start - traj.points[i - 1].time_from_start).toSec();
    //   std::cout << "delta t = " << delta_t << " s" << std::endl;
    // }
    double avg_freq = 1.0 / (traj.points.back().time_from_start.toSec() / double(traj.points.size() - 2));
    ROS_INFO_STREAM_NAMED("teleoperation", "Average frequency of generated trajectory waypoints: " << avg_freq << " hz");
  }

  // Copy final solved state to visualization RobotState for Rviz
  *visualize_goal_state_ = *cart_traj.back();  // deep copy

  // Tell other threads new state is ready to be used
  has_state_to_command_ = true;
  has_state_to_visualize_ = true;
  return true;
}

void Teleoperation::commandJointsThread()
{
  while (ros::ok())
  {
    if (!has_state_to_command_)
    {
      usleep(command_new_trajectory_delay_);  // microseconds
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

      double average_duration = total_command_duration_.toSec() / total_commands_;
      ROS_ERROR_STREAM_NAMED("teleoperation",
                            "COMMAND duration: " << std::fixed << duration.toSec()
                            << "\t avg duration: " << std::fixed << average_duration
                            << "\t avg freq: " << std::fixed << (1.0 / average_duration) << " hz");
    }
  }
}

void Teleoperation::commandJointsThreadHelper()
{
  //std::cout << "Sending commanded trajectory:\n " << trajectory_msg_ << std::endl;

  // Get read-only mutex lock
  // {
  //   boost::shared_lock<boost::shared_mutex> lock(trajectory_msg_mutex_);
  //   trajectory_msg_copy_ = trajectory_msg_;  // Make a copy of datastructure
  // }

  // Fix message 'Dropping first 1 trajectory point(s) out of 35, as they occur before the current time.' from ros_control
  bool use_first_waypoint_offset = false; // prevent first waypoint from being skipped
  trajectory_msgs::JointTrajectory &traj = trajectory_msg_.joint_trajectory;
  if (traj.points.size() > 1)
    ROS_INFO_STREAM_NAMED("temp","second point (point 1) is at time offset " << traj.points[1].time_from_start.toSec());
  if (use_first_waypoint_offset)
  {
    static const ros::Duration SLIGHT_TIME_OFFSET(0.00000001);
    for (std::size_t i = 0; i < traj.points.size(); ++i)
      traj.points[i].time_from_start += SLIGHT_TIME_OFFSET;
  }
  else
  {
    traj.points.erase(traj.points.begin()); // remove first element
  }


  trajectory_msg_.joint_trajectory.header.stamp = controller_state2_->header.stamp; //ros::Time::now();

  // Debug
  //std::cout << "Sending commanded trajectory: \n" << trajectory_msg_ << std::endl;
  // std::cout << "Sending commanded trajectory:" << std::endl;
  // std::cout << trajectory_msg_.joint_trajectory.header;
  // if (trajectory_msg_.joint_trajectory.points.size() > 0)
  //   std::cout << trajectory_msg_.joint_trajectory.points[0];
  // if (trajectory_msg_.joint_trajectory.points.size() > 1)
  //   std::cout << trajectory_msg_.joint_trajectory.points[1];

  // Execute
  const bool wait_for_execution = false;
  //trajectory_msg_timestamp_ = ros::Time::now();
  if (!planning_interface_->getExecutionInterface()->executeTrajectory(trajectory_msg_,
                                                                       arm_jmg_, wait_for_execution))
  {
    ROS_ERROR_STREAM_NAMED("teleoperation", "Failed to execute trajectory");
  }
}

bool Teleoperation::computeCartWaypointPath(const moveit::core::RobotStatePtr start_state,
                                            const EigenSTL::vector_Affine3d& waypoints,
                                            std::vector<moveit::core::RobotStatePtr>& cart_traj)
{
  // Cartesian settings
  // const bool collision_checking_verbose = false;
  // const bool only_check_self_collision = false;
  const bool global_reference_frame = true;  // Reference frame setting

  // Results
  double last_valid_percentage;

  std::size_t attempts = 0;
  static const std::size_t MAX_IK_ATTEMPTS = ik_attempts_;

  while (attempts < MAX_IK_ATTEMPTS)
  {
    if (attempts > 0)
      ROS_DEBUG_STREAM_NAMED("teleoperation", "Attempting IK solution, attempt # " << attempts + 1);
    attempts++;

    // Create a temp state that doesn't have velocity or accelerations
    moveit::core::RobotState cart_state(*start_state);

    // Clear both the velocities and accelerations because this state is copied for the cartesian path
    // but the vel & accel are left the original (incorrect) values
    std::vector<double> zero_variables(cart_state.getVariableCount(), 0.0);
    cart_state.setVariableVelocities(zero_variables);
    cart_state.setVariableAccelerations(zero_variables);


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
    cart_traj.clear();
    last_valid_percentage = cart_state.computeCartesianPath(
                                                            arm_jmg_, cart_traj, ik_tip_link_, waypoints, global_reference_frame, ik_cart_max_step_,
                                                            ik_cart_jump_threshold_);  //, constraint_fn, kinematics::KinematicsQueryOptions());

    // clang-format off
    ROS_DEBUG_STREAM_NAMED("teleoperation", "Cartesian last_valid_percentage: " << last_valid_percentage
                           << " number of points in trajectory: " << cart_traj.size());  // clang-format on

    double min_allowed_valid_percentage = 0.9;
    if (last_valid_percentage == 0)
    {
      ROS_WARN_STREAM_NAMED("teleoperation", "Failed to computer cartesian path: "
                             "last_valid_percentage is 0");
    }
    else if (last_valid_percentage < min_allowed_valid_percentage)
    {
      ROS_WARN_STREAM_NAMED("teleoperation", "Resulting cartesian path is less than "
                             << min_allowed_valid_percentage
                             << " % of the desired distance, % valid: "
                             << last_valid_percentage);
    }
    else
    {
      ROS_DEBUG_STREAM_NAMED("teleoperation", "Found valid cartesian path");
      return true;
    }
  }  // end while AND scoped pointer of locked planning scenep

  ROS_INFO_STREAM_NAMED("teleoperation", "UNABLE to find valid waypoint cartesian path after "
                        << MAX_IK_ATTEMPTS << " attempts"); 
  return false;
}

bool Teleoperation::convertRobotStatesToTrajectory(
                                                   const std::vector<moveit::core::RobotStatePtr>& robot_state_traj,
                                                   moveit_msgs::RobotTrajectory& trajectory_msg,
                                                   bool use_interpolation)
{
  // Copy the vector of RobotStates to a RobotTrajectory
  robot_trajectory::RobotTrajectoryPtr robot_traj(
                                                  new robot_trajectory::RobotTrajectory(robot_model_, arm_jmg_));

  // Convert to RobotTrajectory datatype
  for (std::size_t k = 0; k < robot_state_traj.size(); ++k)
  {
    double duration_from_previous = 1;  // this is overwritten and unimportant
    robot_traj->addSuffixWayPoint(robot_state_traj[k], duration_from_previous);
  }

  //if (robot_traj->getFirstWayPoint().hasVelocities())
  //ROS_DEBUG_STREAM_NAMED("teleoperation.convert", "First waypoint has velocity");

  // Check trajectory length
  static const std::size_t MIN_TRAJECTORY_POINTS = 5;
  if (robot_traj->getWayPointCount() < MIN_TRAJECTORY_POINTS)
  {
    ROS_INFO_STREAM_NAMED("teleoperation", "Too few points (" << robot_traj->getWayPointCount()
                          << ")");
  }

  // Interpolate any path with two few points
  if (use_interpolation && false)
  {
    // Interpolate between each point
    double discretization = 0.25;
    planning_interface_->interpolate(robot_traj, discretization);
  }

  bool debug = false;
  if (debug)
  {
    // Convert trajectory to a message
    moveit_msgs::RobotTrajectory trajectory_msg_debug;
    robot_traj->getRobotTrajectoryMsg(trajectory_msg_debug);
    std::cout << "Before Iterative smoother: \n" << trajectory_msg_debug << std::endl;
  }

  // Perform iterative parabolic smoothing
  planning_interface_->getIterativeSmoother().computeTimeStamps(*robot_traj, vel_scaling_factor_);

  // Get write mutex lock
  {
    boost::lock_guard<boost::shared_mutex> lock(trajectory_msg_mutex_);

    // Convert trajectory to a message
    robot_traj->getRobotTrajectoryMsg(trajectory_msg);
  }

  if (debug)
    std::cout << "After Iterative smoother: " << trajectory_msg << std::endl;

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
        case 2:  // Save Pose
          ROS_INFO_STREAM_NAMED("teleoperation", "Saving Pose");

          Eigen::Affine3d pose = offsetEEPose(feedback->pose);
          visual_tools_->publishZArrow(pose, rvt::GREEN);

          // // Save start state // TODO thread safety?

          // // Show Robot State
          // planning_interface_->getVisualStartState()->publishRobotState(start_planning_state_,
          // rvt::GREEN);

          std::vector<double> xyzrpy;
          visual_tools_->convertToXYZRPY(pose, xyzrpy);
          std::cout << "pose: [" 
                    << xyzrpy[0] << ", "
                    << xyzrpy[1] << ", "
                    << xyzrpy[2] << ", "
                    << xyzrpy[3] << ", "
                    << xyzrpy[4] << ", "
                    << xyzrpy[5] << "]"
                    << std::endl;

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

void Teleoperation::stateCB(const ControllerState::ConstPtr& state)
{  
  // Get write mutex lock
  {
    boost::lock_guard<boost::shared_mutex> lock(controller_state_mutex_);
    if (is_processing_)
    {
      //ROS_WARN_STREAM_NAMED("temp","SKIPPING a controller state callback");
      return;
    }
    is_processing_ = true;
  }

  // Copy pointer
  controller_state2_ = state;

  // Run pipeline
  stateCBHelper();

  // Get write mutex lock
  {
    boost::lock_guard<boost::shared_mutex> lock(controller_state_mutex_);
    is_processing_ = false;
  }
}

void Teleoperation::stateCBHelper()
{
  // TODO thread safe?
  if (!has_pose_to_ik_solve_)
  {
    //ROS_WARN_STREAM_NAMED("temp","SKIPPING because no ik pose to solve");
    return;
  }
  has_pose_to_ik_solve_ = false;

  // Optionally start timer
  ros::Time begin_time;
  if (debug_command_rate_)
    begin_time = ros::Time::now();

  // Plan IK Path
  if (!solveIKThreadHelper())
  {
    ROS_WARN_STREAM_NAMED("temp","aborting because no path found");
    return;
  }

  // Throttle how many to process
  static std::size_t count = 0;
  count++;
  if (count % std::size_t(execution_delay_.toSec()) == 0)
  {
    // Send trajectory for execution
    commandJointsThreadHelper();
  }

  // Optionally end timer
  if (debug_command_rate_)
  {
    ros::Duration duration = (ros::Time::now() - begin_time);
    total_command_duration_ += duration;
    total_commands_++;

    double average_duration = total_command_duration_.toSec() / total_commands_;
    ROS_WARN_STREAM_NAMED("teleoperation",
                           "COMMAND duration: " << std::fixed << duration.toSec()
                           << "\t avg duration: " << std::fixed << average_duration
                           << "\t avg freq: " << std::fixed << (1.0 / average_duration) << " hz");
  }

}

void Teleoperation::getDesiredState(moveit::core::RobotStatePtr& robot_state)
{
  // TODO controller_state2_ check that exists

  // Get read-only mutex lock
  //boost::shared_lock<boost::shared_mutex> lock(controller_state_mutex_);

  // NOTE: THE FOLLOWING CODE IS MODIFIED FROM moveit_ros/planning/planning_scene_monitor/src/current_state_monitor.cpp
  // For each joint
  const double error = std::numeric_limits<double>::epsilon();
  const std::size_t num_joints = controller_state2_->joint_names.size();
  for (std::size_t i = 0; i < num_joints; ++i)
  {
    const robot_model::JointModel* jm = robot_model_->getJointModel(controller_state2_->joint_names[i]);
    if (!jm)
    {
      ROS_WARN_STREAM_ONCE("Unrecognized joint in joint_state message " << controller_state2_->joint_names[i]);
      continue;
    }
    // ignore fixed joints, multi-dof joints (they should not even be in the message)
    if (jm->getVariableCount() != 1)
    {
      ROS_WARN_ONCE("Multi-dof joint attempted to be updated from /joint_state message, skipping");
      continue;
    }

    // Do not update joint if position is the same
    if (robot_state->getJointPositions(jm)[0] != controller_state2_->desired.positions[i])
    {
      // Copy
      robot_state->setJointPositions(jm, &(controller_state2_->desired.positions[i]));

      // check if velocities exist
      if (num_joints == controller_state2_->desired.velocities.size())
      {
        // Copy
        robot_state->setJointVelocities(jm, &(controller_state2_->desired.velocities[i]));

        // check if effort exist. assume they are not useful if no velocities were passed in
        if (num_joints == controller_state2_->desired.effort.size())
        {
          // Copy
          robot_state->setJointEfforts(jm, &(controller_state2_->desired.effort[i]));
        }
      }

      // continuous joints wrap, so we don't modify them (even if they are outside bounds!)
      if (jm->getType() == robot_model::JointModel::REVOLUTE)
        if (static_cast<const robot_model::RevoluteJointModel*>(jm)->isContinuous())
          continue;
        
      const robot_model::VariableBounds &b = jm->getVariableBounds()[0]; // only one variable in the joint, so we get its bounds
        
      // if the read variable is 'almost' within bounds (up to error_ difference), then consider it to be within bounds
      if (controller_state2_->desired.positions[i] < b.min_position_ && controller_state2_->desired.positions[i] >= b.min_position_ - error)
        robot_state->setJointPositions(jm, &b.min_position_);
      else
        if (controller_state2_->desired.positions[i] > b.max_position_ && controller_state2_->desired.positions[i] <= b.max_position_ + error)
          robot_state->setJointPositions(jm, &b.max_position_);
    }
  }

}

Eigen::Affine3d Teleoperation::offsetEEPose(const geometry_msgs::Pose& pose) const
{
  // TODO convertPose is not thread safe
  Eigen::Affine3d marker_pose = visual_tools_->convertPose(pose);

  // Offset ee pose forward, because interactive marker is a special thing in front of hand
  return marker_pose * ee_offset_;
}

bool Teleoperation::posesEqual(const Eigen::Affine3d& pose1, const Eigen::Affine3d& pose2)
{
  static const double POSE_EQUAL_THRESHOLD = 0.001;
  static const std::size_t NUM_VARS = 16;

  for (std::size_t i = 0; i < NUM_VARS; ++i)
  {
    //std::cout << "var " << i << ", values: " << pose1.data()[i] << ", " << pose2.data()[i] << std::endl;
    if (fabs(pose1.data()[i] - pose2.data()[i]) > POSE_EQUAL_THRESHOLD)
    {
      //std::cout << "Not equal at var  " << i << std::endl;
      return false;
    }
  }

  return true;
}

}  // end namespace
