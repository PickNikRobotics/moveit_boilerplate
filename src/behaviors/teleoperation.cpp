/* Author: Dave Coleman <dave@dav.ee>
   Desc:   Teleoperation
*/

// Command line arguments
//#include <gflags/gflags.h>

// moveit_manipulation
#include <moveit_manipulation/behaviors/teleoperation.h>

// Parameter loading
#include <ros_param_utilities/ros_param_utilities.h>

namespace moveit_manipulation
{
Teleoperation::Teleoperation()
  : MoveItBoilerplate()
{
  // Load rosparams
  const std::string parent_name = "teleoperation";  // for namespacing logging messages
  std::vector<double> offset_doubles;
  ros_param_utilities::getDoubleParameters(parent_name, nh_, parent_name + "/ee_offset",
                                           offset_doubles);
  ros_param_utilities::convertDoublesToEigen(parent_name, offset_doubles, ee_offset_);
  ros_param_utilities::getDoubleParameter(parent_name, nh_, parent_name + "/ik_consistency_limit",
                                          ik_consistency_limit_);
  ros_param_utilities::getDoubleParameter(parent_name, nh_, parent_name + "/ik_timeout",
                                          ik_timeout_);
  ros_param_utilities::getDoubleParameter(parent_name, nh_, parent_name + "/ik_attempts",
                                          ik_attempts_);

  // Create consistency limits for IK solving. Pre-build this vector for improved speed
  // This specifies the desired distance between the solution and the seed state
  if (ik_consistency_limit_)
    for (std::size_t i = 0; i < config_->right_arm_->getActiveJointModels().size(); ++i)            
      ik_consistency_limits_vector_.push_back(ik_consistency_limit_);

  // Show interactive marker
  setupInteractiveMarker();
}

void Teleoperation::startTeleopStatePublishing()
{
  // In separate thread from imarker & Ik solver, send joint commands
  ROS_INFO_STREAM_NAMED("teleoperation", "Teleoperation state command loop ready");

  moveit_msgs::RobotTrajectory trajectory_msg;
  JointModelGroup* arm_jmg = config_->right_arm_;

  // Assign joint names
  trajectory_msg.joint_trajectory.joint_names = arm_jmg->getActiveJointModelNames();
  trajectory_msg.joint_trajectory.points.resize(1);
  trajectory_msg.joint_trajectory.points[0].positions.resize(
      arm_jmg->getActiveJointModels().size());

  // Assign duration (some small number)
  trajectory_msg.joint_trajectory.points[0].time_from_start = ros::Duration(0.0001);

  while (ros::ok())
  {
    if (!has_new_state_)
      continue;

    has_new_state_ = false;

    // Assign joint values
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      command_state_->copyJointGroupPositions(arm_jmg,
                                              trajectory_msg.joint_trajectory.points[0].positions);
      // User feedback
      manipulation_->getVisualStartState()->publishRobotState(command_state_, rvt::BLUE);
    }

    // Wait for previous trajectory to finish
    manipulation_->getExecutionInterface()->waitForExecution();

    // Execute
    const bool wait_for_execution = false;
    if (!manipulation_->getExecutionInterface()->executeTrajectory(trajectory_msg, arm_jmg,
                                                                   wait_for_execution))
    {
      ROS_ERROR_STREAM_NAMED("teleopertion", "Failed to execute trajectory");
    }
  }
}

void Teleoperation::setupInteractiveMarker()
{
  remote_control_->initializeInteractiveMarkers(getInteractiveMarkerPose());

  // Load robot state
  teleop_state_.reset(new moveit::core::RobotState(*manipulation_->getCurrentState()));
  command_state_.reset(new moveit::core::RobotState(*manipulation_->getCurrentState()));

  // Set a callback function
  remote_control_->setInteractiveMarkerCallback(std::bind(
      &Teleoperation::processMarkerPose, this, std::placeholders::_1, std::placeholders::_2));
}

geometry_msgs::Pose Teleoperation::getInteractiveMarkerPose()
{
  JointModelGroup* arm_jmg = config_->dual_arm_ ? config_->both_arms_ : config_->right_arm_;

  // Check that we have a current state
  if (!manipulation_->getCurrentState())
    ROS_ERROR_STREAM_NAMED("teleoperation", "No current state");

  interactive_marker_pose_ =
      manipulation_->getCurrentState()->getGlobalLinkTransform(grasp_datas_[arm_jmg]->parent_link_);
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
    remote_control_->updateMarkerPose(getInteractiveMarkerPose());
    return;  // do not perform IK for thie callback
  }

  // Get pose and visualize
  interactive_marker_pose_ = visual_tools_->convertPose(pose);

  // Debug
  if (false)
    visual_tools_->publishSphere(interactive_marker_pose_, rvt::RED);

  // Offset ee pose forward, because we are treating interactive marker as a special thing in front
  // of hand
  Eigen::Affine3d ee_pose = interactive_marker_pose_ * ee_offset_;

  bool use_cartesian_commands = false;

  // IK on robot method (faster)
  if (use_cartesian_commands)
  {
    // Convert pose to frame of robot base
    const Eigen::Affine3d& world_to_base =
        manipulation_->getCurrentState()->getGlobalLinkTransform("base_link");

    Eigen::Affine3d base_to_desired = world_to_base.inverse() * ee_pose;
    manipulation_->getExecutionInterface()->executePose(base_to_desired, config_->right_arm_);
  }
  else
  {
    // IK on dev computer method
    teleoperate(ee_pose, config_->right_arm_);
  }
}

bool Teleoperation::teleoperate(const Eigen::Affine3d& ee_pose, JointModelGroup* arm_jmg)
{
  // NOTE this is in a separate thread, so we should only use visual_tools_ for debugging!

  // Solve IK
  static bool collision_checking_verbose = false;
  if (collision_checking_verbose)
    ROS_WARN_STREAM_NAMED("teleoperation", "collision_checking_verbose turned on");                            

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
    const moveit::core::LinkModel* ik_tip_link = grasp_datas_[arm_jmg]->parent_link_;
    if (!teleop_state_->setFromIK(arm_jmg, ee_pose, ik_tip_link->getName(), ik_consistency_limits_vector_,
                                  ik_attempts_, ik_timeout_)) //, constraint_fn))
    {
      static std::size_t warning_counter = 0;
      ROS_WARN_STREAM_NAMED("manipulation", "Unable to find arm solution for desired pose " << warning_counter++);
      return false;
    }
  }  // end scoped pointer of locked planning scene

  // Copy solution to command RobotState
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    *command_state_ = *teleop_state_;  // deep copy
  }

  // Tell other thread new state is ready to command
  has_new_state_ = true;

  return true;
}

}  // end namespace
