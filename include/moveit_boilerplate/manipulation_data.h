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

#ifndef MOVEIT_BOILERPLATE__MANIPULATION_DATA
#define MOVEIT_BOILERPLATE__MANIPULATION_DATA

// ROS
#include <ros/ros.h>

// MoveItManipulation
#include <moveit_boilerplate/namespaces.h>

// MoveIt!
#include <moveit/robot_model/robot_model.h>

namespace moveit_boilerplate
{
class ManipulationData
{
public:
  /**
   * \brief Constructor
   */
  ManipulationData();

  /**
   * \brief Load the configuration from rosparam
   * \param robot_model
   * \param fake_execution - whether to load full speed velocity constraints (simulation goes
   * faster)
   * \return true on success
   */
  bool load(robot_model::RobotModelPtr robot_model, bool fake_execution,
            const std::string& package_path);

  /**
   * \brief A tool to quickly tweak location of pose in planning world. Shows warning since this is
   * only for testing
   */
  Eigen::Affine3d getTestPose();

  /**
   * \brief Check if certain key is enabled
   * \return true if enabled
   */
  bool isEnabled(const std::string& setting_name);

  // Performance variables
  double main_velocity_scaling_factor_;
  double approach_velocity_scaling_factor_;
  double lift_velocity_scaling_factor_;
  double retreat_velocity_scaling_factor_;
  double calibration_velocity_scaling_factor_;

  // Wait variables
  //double wait_before_grasp_;
  //double wait_after_grasp_;

  // Distance variables
  double place_goal_down_distance_desired_;
  double goal_bin_clearance_;
  double jump_threshold_;

  // Robot semantics
  std::string start_pose_;  // where to move robot to initially. should be for both arms if
                            // applicable

  std::string right_hand_name_;
  std::string left_hand_name_;
  std::string right_arm_name_;
  std::string right_arm_only_name_;
  std::string left_arm_name_;
  std::string both_arms_name_;

  // Load planning configs
  bool use_experience_setup_;
  std::string experience_type_;
  double planning_time_;

  // Group for each arm
  JointModelGroup* right_arm_;
  JointModelGroup* left_arm_;
  JointModelGroup* both_arms_;  // TODO remove?

  // Logic on type of robot
  bool dual_arm_;

  // Execution mode
  bool fake_execution_;

  // Behavior settings
  std::map<std::string, bool> enabled_;

  // Base frame / model frame e.g. "base_link"
  std::string robot_base_frame_;

  // Generic variable adjustment
  double test_double_;

  // File path to ROS package on drive
  std::string package_path_;

  // Location to grasp
  //Eigen::Affine3d grasp_location_transform_;

  std::string joint_state_topic_;

private:
  // A shared node handle
  ros::NodeHandle nh_;

  // For tweaking
  Eigen::Affine3d test_pose_;

};  // end class

// Create boost pointers for this class
typedef boost::shared_ptr<ManipulationData> ManipulationDataPtr;
typedef boost::shared_ptr<const ManipulationData> ManipulationDataConstPtr;

}  // end namespace

#endif
