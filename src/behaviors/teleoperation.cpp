/* Author: Dave Coleman <dave@dav.ee>
   Desc:   Teleoperation
*/

// Command line arguments
#include <gflags/gflags.h>

// MoveItManipulation
#include <moveit_manipulation/behaviors/teleoperation.h>

namespace moveit_manipulation
{
Teleoperation::Teleoperation()
  : MoveItBoilerplate()
{
  // Show interactive marker
  setupInteractiveMarker();
}

// Mode 1
void Teleoperation::enableTeleoperation()
{
  ROS_INFO_STREAM_NAMED("pick_manager", "Teleoperation enabled");
  teleoperation_enabled_ = true;
  manipulation_->enableTeleoperation();

  // TEST - measure the offset between blue tool frame and ROS tool frame
  if (false)
  {
    moveit::core::RobotStatePtr before_state(
        new moveit::core::RobotState(*manipulation_->getCurrentState()));

    JointModelGroup* arm_jmg = config_->dual_arm_ ? config_->both_arms_ : config_->right_arm_;

    const Eigen::Affine3d world_to_desired = interactive_marker_pose_;
    const Eigen::Affine3d& world_to_base =
        manipulation_->getCurrentState()->getGlobalLinkTransform("base_link");
    Eigen::Affine3d base_to_desired = world_to_base.inverse() * world_to_desired;

    // New Method
    manipulation_->getExecutionInterface()->executePose(base_to_desired, arm_jmg);

    ros::Duration(1.0).sleep();
    moveit::core::RobotStatePtr after_state(
        new moveit::core::RobotState(*manipulation_->getCurrentState()));

    // Now find the difference between EE poses
    visual_tools_->printTransform(
        before_state->getGlobalLinkTransform(grasp_datas_[arm_jmg]->parent_link_));

    visual_tools_->printTransform(
        after_state->getGlobalLinkTransform(grasp_datas_[arm_jmg]->parent_link_));
  }
}

geometry_msgs::Pose Teleoperation::getInteractiveMarkerPose() 
{
  JointModelGroup* arm_jmg = config_->dual_arm_ ? config_->both_arms_ : config_->right_arm_;  

  // Check that we have a current state
  if (!manipulation_->getCurrentState())
  {
    ROS_ERROR_STREAM_NAMED("teleoperation","No current state");
    manipulation_->getCurrentState()->printStateInfo();
  }

  visual_tools_->publishRobotState(manipulation_->getCurrentState(), rvt::PURPLE);

  interactive_marker_pose_ = manipulation_->getCurrentState()->getGlobalLinkTransform(grasp_datas_[arm_jmg]->parent_link_);
  visual_tools_->printTransform(interactive_marker_pose_);
  
  // Move marker to tip of fingers
  interactive_marker_pose_ = interactive_marker_pose_ * config_->teleoperation_offset_.inverse();

  std::cout << "pose\n" << visual_tools_->convertPose(interactive_marker_pose_) << std::endl;

  return visual_tools_->convertPose(interactive_marker_pose_);
}

void Teleoperation::setupInteractiveMarker()
{
  remote_control_->initializeInteractiveMarkers(getInteractiveMarkerPose());

  // Set a callback function
  remote_control_->setInteractiveMarkerCallback(std::bind(
      &Teleoperation::processMarkerPose, this, std::placeholders::_1, std::placeholders::_2));
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

  if (!teleoperation_enabled_)
    return;

  // Choose arm
  JointModelGroup* arm_jmg = config_->dual_arm_ ? config_->both_arms_ : config_->right_arm_;

  // Offset ee pose forward, because we are treating interactive marker as a special thing in front
  // of hand
  Eigen::Affine3d ee_pose = interactive_marker_pose_ * config_->teleoperation_offset_;

  // Convert pose to frame of robot base
  const Eigen::Affine3d& world_to_base =
      manipulation_->getCurrentState()->getGlobalLinkTransform("base_link");

  bool use_cartesian_commands = false;

  // IK on robot method (faster)
  if (use_cartesian_commands)
  {
    Eigen::Affine3d base_to_desired = world_to_base.inverse() * ee_pose;
    manipulation_->getExecutionInterface()->executePose(base_to_desired, arm_jmg);
  }
  else 
    {
      // IK on dev computer method
      bool move = true;
      manipulation_->teleoperation(ee_pose, move, arm_jmg);
    }
}

}  // end namespace
