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
   Desc:   Holds common parameters for manipulation
*/

#include <moveit_boilerplate/manipulation_data.h>
#include <moveit_boilerplate/namespaces.h>

// Parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

#include <rviz_visual_tools/rviz_visual_tools.h>

namespace moveit_boilerplate
{
ManipulationData::ManipulationData() : nh_("~")
{
}

bool ManipulationData::load(robot_model::RobotModelPtr robot_model, bool fake_execution,
                            const std::string& package_path)
{
  fake_execution_ = fake_execution;
  package_path_ = package_path;
  const std::string parent_name = "manipulation_data";  // for namespacing logging messages

  // Load performance variables
  rosparam_shortcuts::getDoubleParam(parent_name, nh_, "main_velocity_scaling_factor", main_velocity_scaling_factor_);
  rosparam_shortcuts::getDoubleParam(parent_name, nh_, "approach_velocity_scaling_factor",
                                     approach_velocity_scaling_factor_);
  rosparam_shortcuts::getDoubleParam(parent_name, nh_, "lift_velocity_scaling_factor", lift_velocity_scaling_factor_);
  rosparam_shortcuts::getDoubleParam(parent_name, nh_, "retreat_velocity_scaling_factor",
                                     retreat_velocity_scaling_factor_);
  rosparam_shortcuts::getDoubleParam(parent_name, nh_, "calibration_velocity_scaling_factor",
                                     calibration_velocity_scaling_factor_);

  if (fake_execution_)
  {
    ROS_WARN_STREAM_NAMED("manipulation_data", "In simulation mode - velocity set to 100%");
    main_velocity_scaling_factor_ = 1.0;
    approach_velocity_scaling_factor_ = 1.0;
    lift_velocity_scaling_factor_ = 1.0;
    retreat_velocity_scaling_factor_ = 1.0;
    calibration_velocity_scaling_factor_ = 1.0;
  }

  // rosparam_shortcuts::getDoubleParam(parent_name, nh_, "wait_before_grasp",
  //                                         wait_before_grasp_);
  // rosparam_shortcuts::getDoubleParam(parent_name, nh_, "wait_after_grasp", wait_after_grasp_);
  // rosparam_shortcuts::getDoubleParam(parent_name, nh_, "place_goal_down_distance_desired",
  //                                          place_goal_down_distance_desired_);
  // rosparam_shortcuts::getDoubleParam(parent_name, nh_, "goal_bin_clearance",
  //                                          goal_bin_clearance_);
  rosparam_shortcuts::getDoubleParam(parent_name, nh_, "jump_threshold", jump_threshold_);

  // Load robot semantics
  rosparam_shortcuts::getStringParam(parent_name, nh_, "start_pose", start_pose_);
  rosparam_shortcuts::getStringParam(parent_name, nh_, "right_hand_name", right_hand_name_);
  rosparam_shortcuts::getStringParam(parent_name, nh_, "left_hand_name", left_hand_name_);
  rosparam_shortcuts::getStringParam(parent_name, nh_, "right_arm_name", right_arm_name_);
  rosparam_shortcuts::getStringParam(parent_name, nh_, "right_arm_only_name", right_arm_only_name_);
  rosparam_shortcuts::getStringParam(parent_name, nh_, "left_arm_name", left_arm_name_);
  rosparam_shortcuts::getStringParam(parent_name, nh_, "both_arms_name", both_arms_name_);

  // Load planning configs
  rosparam_shortcuts::getBoolParam(parent_name, nh_, "moveit_ompl/use_experience_setup", use_experience_setup_);
  rosparam_shortcuts::getStringParam(parent_name, nh_, "moveit_ompl/experience_type", experience_type_);
  rosparam_shortcuts::getDoubleParam(parent_name, nh_, "moveit_ompl/planning_time", planning_time_);

  // Behavior configs
  rosparam_shortcuts::getBoolMap(parent_name, nh_, "behavior", enabled_);

  // Decide on dual arm mode we are in
  rosparam_shortcuts::getBoolParam(parent_name, nh_, "dual_arm", dual_arm_);

  // Generic test variable
  rosparam_shortcuts::getDoubleParam(parent_name, nh_, "test/test_double", test_double_);

  // Get test pose
  std::vector<double> test_pose_doubles;
  rosparam_shortcuts::getDoubleParams(parent_name, nh_, "test/test_pose", test_pose_doubles);
  rosparam_shortcuts::convertDoublesToEigen(parent_name, test_pose_doubles, test_pose_);
  // test_pose_ = rvt::RvizVisualTools::convertFromXYZRPY(test_pose_doubles_); // TODO

  // Get grasp location doubles
  // std::vector<double> grasp_location_transform_doubles;
  // rosparam_shortcuts::getDoubleParams(parent_name, nh_, "grasp_location_transform",
  //                                          grasp_location_transform_doubles);
  // rosparam_shortcuts::convertDoublesToEigen(parent_name, grasp_location_transform_doubles,
  //                                            grasp_location_transform_);

  // Pick Manager settings
  rosparam_shortcuts::getStringParam(parent_name, nh_, "joint_state_topic", joint_state_topic_);

  // Load proper groups
  // TODO - check if joint model group exists
  if (dual_arm_)
  {
    // Load arm groups
    left_arm_ = robot_model->getJointModelGroup(left_arm_name_);
    right_arm_ = robot_model->getJointModelGroup(right_arm_name_);
    both_arms_ = robot_model->getJointModelGroup(both_arms_name_);
  }
  else
  {
    // Load arm groups
    right_arm_ = robot_model->getJointModelGroup(right_arm_name_);
  }

  // Set world frame
  robot_base_frame_ = robot_model->getModelFrame();

  ROS_INFO_STREAM_NAMED("manipulation_data", "ManipulationData Ready.");

  return true;
}

Eigen::Affine3d ManipulationData::getTestPose()
{
  using namespace std;
  ROS_WARN_STREAM_NAMED("manipulation_data", "Using test pose, which should only be used during early development");
  // ROS_WARN_STREAM_NAMED("manipulation_data", "Use instead:");

  // std::cout << "  Eigen::Affine3d transform = rvt::RvizVisualTools::convertFromXYZRPY("
  //           << test_pose_doubles_[0] << "," << test_pose_doubles_[1] << "," <<
  //           test_pose_doubles_[2]
  //           << "," << test_pose_doubles_[3] << "," << test_pose_doubles_[4] << ","
  //           << test_pose_doubles_[5] << "); // from testPose()" << std::endl;

  return test_pose_;
}

bool ManipulationData::isEnabled(const std::string& setting_name)
{
  std::map<std::string, bool>::iterator it = enabled_.find(setting_name);
  if (it != enabled_.end())
  {
    // Element found;
    return it->second;
  }
  ROS_ERROR_STREAM_NAMED("manipulation_data", "isEnabled() key '" << setting_name << "' does not exist in the "
                                                                                     "available configuration");
  return false;
}

}  // end namespace
