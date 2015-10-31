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

  // Show interactive marker
  setupInteractiveMarker();

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
  // Wait for previous trajectory to finish
  //manipulation_->getExecutionInterface()->waitForExecution();

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
    robot_traj->setRobotTrajectoryMsg(*manipulation_->getCurrentState(), trajectory_msg_);

    // Add current state to trajectory
    double dummy_dt = 1.0;
    robot_traj->addPrefixWayPoint(manipulation_->getCurrentState(), dummy_dt);

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
    manipulation_->getIterativeSmoother().computeTimeStamps(*robot_traj, max_velocity_scaling_factor);

    // Convert trajectory back to a message
    robot_traj->getRobotTrajectoryMsg(trajectory_msg_);

    // First time_from_start cannnot be zero, otherwise controller throws error TODO why?
    trajectory_msg_.joint_trajectory.points.front().time_from_start = ros::Duration(0.00000001);

    //std::cout << "AFTER PARAM: \n" << trajectory_msg_ << std::endl;

    // Clear all points that do not have increasing time
    if (trajectory_msg_.joint_trajectory.points.size() > 0) // Make sure we have at least one point
      for (std::size_t i = 1; i < trajectory_msg_.joint_trajectory.points.size(); ++i)
      {
        if (trajectory_msg_.joint_trajectory.points[i-1].time_from_start == 
            trajectory_msg_.joint_trajectory.points[i].time_from_start)
        {
          //std::cout << "found where time_from_start repeats: " << i << std::endl;
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
    ROS_ERROR_STREAM_NAMED("teleopertion", "Failed to execute trajectory");
  }
}

void Teleoperation::solveIKThread()
{
  while (ros::ok())
  {
    if (!has_pose_to_ik_solve_)
    {
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

void Teleoperation::setupInteractiveMarker()
{
  remote_control_->initializeInteractiveMarkers(chooseNewIMarkerPose());

  // Load robot state
  ik_teleop_state_.reset(new moveit::core::RobotState(*manipulation_->getCurrentState()));
  command_state_.reset(new moveit::core::RobotState(*manipulation_->getCurrentState()));

  // Set a callback function
  remote_control_->setInteractiveMarkerCallback(std::bind(
      &Teleoperation::processMarkerPose, this, std::placeholders::_1, std::placeholders::_2));
}

geometry_msgs::Pose Teleoperation::chooseNewIMarkerPose()
{
  // Check that we have a current state
  if (!manipulation_->getCurrentState())
    ROS_ERROR_STREAM_NAMED("teleoperation", "No current state");

  interactive_marker_pose_ = manipulation_->getCurrentState()->getGlobalLinkTransform(
      grasp_datas_[arm_jmg_]->parent_link_);
  visual_tools_->printTransform(interactive_marker_pose_);

  // Move marker to tip of fingers
  interactive_marker_pose_ = interactive_marker_pose_ * ee_offset_.inverse();

  return visual_tools_->convertPose(interactive_marker_pose_);
}

void Teleoperation::processMarkerPose(const geometry_msgs::Pose& pose, int mode)
{
  // NOTE this is in a separate thread, so we should only use visuals_->trajectory_lines_ for
  // debugging!

  if (mode == 2)
  {
    // User has requested to reset location of interactive marker
    remote_control_->updateMarkerPose(chooseNewIMarkerPose());
    return;  // do not perform IK for thie callback
  }

  // Get pose
  interactive_marker_pose_ = visual_tools_->convertPose(pose);

  // Offset ee pose forward, because interactive marker is a special thing in front of hand
  desired_ee_pose_ = interactive_marker_pose_ * ee_offset_;

  // Mark pose as ready for ik solving
  has_pose_to_ik_solve_ = true;

  /*
  // Offset ee pose forward, because we are treating interactive marker as a special thing in front
  // of hand
  Eigen::Affine3d ee_pose = interactive_marker_pose_ * ee_offset_;

  // IK on robot method (faster)
  // Convert pose to frame of robot base
  const Eigen::Affine3d& world_to_base =
  manipulation_->getCurrentState()->getGlobalLinkTransform("base_link");

  Eigen::Affine3d base_to_desired = world_to_base.inverse() * ee_pose;
  manipulation_->getExecutionInterface()->executePose(base_to_desired, config_->right_arm_);
  */
}

}  // end namespace
