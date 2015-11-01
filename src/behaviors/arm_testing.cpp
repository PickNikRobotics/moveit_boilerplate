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
   Desc:   Tools for testing an arm
*/

// Command line arguments
#include <gflags/gflags.h>

// MoveItManipulation
#include <moveit_manipulation/behaviors/arm_testing.h>

namespace moveit_manipulation
{

ArmTesting::ArmTesting()
  : ArmTesting()
{
  ROS_INFO_STREAM_NAMED("arm_testing", "ArmTesting Ready.");
}

// Mode 8
bool ArmTesting::testEndEffectors()
{
  // Test visualization
  std::size_t i = 0;
  bool open;
  moveit::core::RobotStatePtr current_state = planning_interface_->getCurrentState();
  while (ros::ok())
  {
    std::cout << std::endl << std::endl;
    if (i % 2 == 0)
    {
      std::cout << "Showing closed EE of state " << std::endl;

      open = false;
      // planning_interface_->setStateWithOpenEE(open, current_state);
      // visual_tools_->publishRobotState(current_state);

      // Close all EEs
      planning_interface_->openEEs(open);

      ros::Duration(2.0).sleep();
    }
    else
    {
      std::cout << "Showing open EE of state " << std::endl;

      open = true;
      // planning_interface_->setStateWithOpenEE(open, current_state);
      // visual_tools_->publishRobotState(current_state);

      // Close all EEs
      planning_interface_->openEEs(open);

      ros::Duration(2.0).sleep();
    }
    ++i;
  }

  ROS_INFO_STREAM_NAMED("arm_testing", "Done testing end effectors");
  return true;
}

// Mode 5
bool ArmTesting::testUpAndDown()
{
  double lift_distance_desired = 0.5;

  // Test
  std::size_t i = 0;
  while (ros::ok())
  {
    std::cout << std::endl << std::endl;
    if (i % 2 == 0)
    {
      std::cout << "Moving up --------------------------------------" << std::endl;
      planning_interface_->executeVerticlePathWithIK(config_->right_arm_, lift_distance_desired, true);

      if (config_->dual_arm_)
        planning_interface_->executeVerticlePathWithIK(config_->left_arm_, lift_distance_desired, true);

      ros::Duration(1.0).sleep();
    }
    else
    {
      std::cout << "Moving down ------------------------------------" << std::endl;
      planning_interface_->executeVerticlePathWithIK(config_->right_arm_, lift_distance_desired,false);

      if (config_->dual_arm_)
        planning_interface_->executeVerticlePathWithIK(config_->left_arm_, lift_distance_desired, false);

      ros::Duration(1.0).sleep();
    }
    ++i;
  }

  ROS_INFO_STREAM_NAMED("arm_testing", "Done testing up and down");
  return true;
}

// Mode 10
bool ArmTesting::testInAndOut()
{
  double approach_distance_desired = 1.0;

  // Test
  std::size_t i = 1;
  while (ros::ok())
  {
    visual_tools_->deleteAllMarkers();

    std::cout << std::endl << std::endl;
    if (i % 2 == 0)
    {
      std::cout << "Moving in --------------------------------------" << std::endl;
      if (!planning_interface_->executeRetreatPath(config_->right_arm_, approach_distance_desired, false))
        return false;
      if (config_->dual_arm_)
        if (!planning_interface_->executeRetreatPath(config_->left_arm_, approach_distance_desired,
                                               false))
          return false;
      ros::Duration(1.0).sleep();
    }
    else
    {
      std::cout << "Moving out ------------------------------------" << std::endl;
      if (!planning_interface_->executeRetreatPath(config_->right_arm_, approach_distance_desired, true))
        return false;
      if (config_->dual_arm_)
        if (!planning_interface_->executeRetreatPath(config_->left_arm_, approach_distance_desired, true))
          return false;
      ros::Duration(1.0).sleep();
    }
    ++i;
  }

  ROS_INFO_STREAM_NAMED("arm_testing", "Done testing in and out");
  return true;
}

// Mode 41
bool ArmTesting::getSRDFPose()
{
  ROS_DEBUG_STREAM_NAMED("arm_testing", "Get SRDF pose");

  // Choose which planning group to use
  JointModelGroup* arm_jmg = config_->dual_arm_ ? config_->both_arms_ : config_->right_arm_;
  const std::vector<const moveit::core::JointModel*> joints = arm_jmg->getJointModels();

  while (ros::ok())
  {
    ROS_INFO_STREAM("SDF Code for joint values pose:\n");

    // Get current state after grasping
    moveit::core::RobotStatePtr current_state = planning_interface_->getCurrentState();

    // Check if current state is valid
    // planning_interface_->fixCurrentCollisionAndBounds(arm_jmg);

    // Output XML
    std::cout << "<group_state name=\"\" group=\"" << arm_jmg->getName() << "\">\n";
    for (std::size_t i = 0; i < joints.size(); ++i)
    {
      std::cout << "  <joint name=\"" << joints[i]->getName() << "\" value=\""
                << current_state->getJointPositions(joints[i])[0] << "\" />\n";
    }
    std::cout << "</group_state>\n\n\n\n";

    ros::Duration(4.0).sleep();
  }
  return true;
}

// Mode 42
bool ArmTesting::testInCollision()
{
  while (ros::ok())
  {
    std::cout << std::endl;

    // For debugging in console
    planning_interface_->showJointLimits(config_->right_arm_);

    // planning_interface_->fixCurrentCollisionAndBounds(arm_jmg);
    planning_interface_->checkCollisionAndBounds(planning_interface_->getCurrentState());
    ros::Duration(0.1).sleep();
  }

  ROS_INFO_STREAM_NAMED("arm_testing", "Done checking if in collision");
  return true;
}

// Mode 6
bool ArmTesting::testRandomValidMotions()
{
  // Plan to random
  while (ros::ok())
  {
    static const std::size_t MAX_ATTEMPTS = 200;
    for (std::size_t i = 0; i < MAX_ATTEMPTS; ++i)
    {
      ROS_DEBUG_STREAM_NAMED("arm_testing", "Attempt " << i << " to plan to a random location");

      // Create start
      moveit::core::RobotStatePtr current_state = planning_interface_->getCurrentState();

      // Create goal
      moveit::core::RobotStatePtr goal_state(new moveit::core::RobotState(*current_state));

      // Choose arm
      JointModelGroup* arm_jmg = config_->right_arm_;
      if (config_->dual_arm_)
        if (visual_tools_->iRand(0, 1) == 0)
          arm_jmg = config_->left_arm_;

      goal_state->setToRandomPositions(arm_jmg);

      // Check if random goal state is valid
      bool collision_verbose = false;
      if (planning_interface_->checkCollisionAndBounds(current_state, goal_state, collision_verbose))
      {
        // Plan to this position
        bool verbose = true;
        bool execute_trajectory = true;
        if (planning_interface_->move(current_state, goal_state, arm_jmg,
                                config_->main_velocity_scaling_factor_, verbose,
                                execute_trajectory))
        {
          ROS_INFO_STREAM_NAMED("arm_testing", "Planned to random valid state successfullly");
        }
        else
        {
          ROS_ERROR_STREAM_NAMED("arm_testing", "Failed to plan to random valid state");
          return false;
        }
      }
    }
    ROS_ERROR_STREAM_NAMED("arm_testing", "Unable to find random valid state after "
                                               << MAX_ATTEMPTS << " attempts");

    ros::Duration(1).sleep();
  }  // while

  ROS_INFO_STREAM_NAMED("arm_testing", "Done planning to random valid");
  return true;
}

// Mode 2
bool ArmTesting::testGoHome()
{
  ROS_DEBUG_STREAM_NAMED("arm_testing", "Going home");

  // Choose which planning group to use
  JointModelGroup* arm_jmg = config_->dual_arm_ ? config_->both_arms_ : config_->right_arm_;
  moveToStartPosition(arm_jmg);
  return true;
}

// Mode 17
bool ArmTesting::testJointLimits()
{
  ROS_INFO_STREAM_NAMED("arm_testing", "Testing joint limits");
  ROS_WARN_STREAM_NAMED("arm_testing", "DOES NOT CHECK FOR COLLISION");

  moveit::core::RobotStatePtr current_state = planning_interface_->getCurrentState();

  // Create goal
  moveit::core::RobotStatePtr goal_state(new moveit::core::RobotState(*current_state));

  // Setup data
  std::vector<double> joint_position;
  joint_position.resize(1);
  const std::vector<const moveit::core::JointModel*>& joints =
      config_->right_arm_->getActiveJointModels();

  // Decide if we are testing 1 joint or all
  int test_joint_limit_joint = 0;
  std::size_t first_joint;
  std::size_t last_joint;
  if (test_joint_limit_joint < 0)
  {
    first_joint = 0;
    last_joint = joints.size();
  }
  else
  {
    first_joint = test_joint_limit_joint;
    last_joint = test_joint_limit_joint + 1;
  }

  // Keep testing
  while (true)
  {
    // Loop through each joint, assuming each joint has only 1 variable
    for (std::size_t i = first_joint; i < last_joint; ++i)
    {
      if (!ros::ok())
        return false;

      const moveit::core::VariableBounds& bound = joints[i]->getVariableBounds()[0];
      double reduce_bound = 0.01;

      // Move to min bound
      std::cout << std::endl;
      std::cout << "-------------------------------------------------------" << std::endl;
      joint_position[0] = bound.min_position_ + reduce_bound;
      ROS_INFO_STREAM_NAMED("arm_testing", "Sending joint " << joints[i]->getName()
                                                             << " to min position of "
                                                             << joint_position[0]);
      goal_state->setJointPositions(joints[i], joint_position);

      if (!planning_interface_->executeState(goal_state, config_->right_arm_,
                                       config_->main_velocity_scaling_factor_))
      {
        ROS_ERROR_STREAM_NAMED("arm_testing", "Unable to move to min bound of "
                                                   << joint_position[0] << " on joint "
                                                   << joints[i]->getName());
      }
      ros::Duration(1.0).sleep();

      // Move to max bound
      std::cout << std::endl;
      std::cout << "-------------------------------------------------------" << std::endl;
      joint_position[0] = bound.max_position_ - reduce_bound;
      ROS_INFO_STREAM_NAMED("arm_testing", "Sending joint " << joints[i]->getName()
                                                             << " to max position of "
                                                             << joint_position[0]);
      goal_state->setJointPositions(joints[i], joint_position);

      if (!planning_interface_->executeState(goal_state, config_->right_arm_,
                                       config_->main_velocity_scaling_factor_))
      {
        ROS_ERROR_STREAM_NAMED("arm_testing", "Unable to move to max bound of "
                                                   << joint_position[0] << " on joint "
                                                   << joints[i]->getName());
      }
      ros::Duration(1.0).sleep();
    }
  }

  ROS_INFO_STREAM_NAMED("arm_testing", "Done testing joint limits");
  return true;
}

bool ArmTesting::recordTrajectory()
{
  std::string file_path;
  const std::string file_name = "test_trajectory";
  trajectory_io_->getFilePath(file_path, file_name);

  // Start recording
  trajectory_io_->recordTrajectoryToFile(file_path);

  ROS_INFO_STREAM_NAMED("arm_testing", "Done recording");

  return true;
}

bool ArmTesting::moveToStartPosition(JointModelGroup* arm_jmg, bool check_validity)
{
  return planning_interface_->moveToStartPosition(arm_jmg, check_validity);
}

void ArmTesting::publishCurrentState()
{
  planning_scene_monitor::LockedPlanningSceneRO scene(
      planning_scene_monitor_);  // Lock planning scene
  visual_tools_->publishRobotState(scene->getCurrentState(), rvt::PURPLE);
}

// Mode 9
bool ArmTesting::gotoPose(const std::string& pose_name)
{
  ROS_INFO_STREAM_NAMED("arm_testing", "Going to pose " << pose_name);
  ros::Duration(1).sleep();
  ros::spinOnce();

  JointModelGroup* arm_jmg = config_->dual_arm_ ? config_->both_arms_ : config_->right_arm_;
  bool check_validity = true;

  if (!planning_interface_->moveToSRDFPose(arm_jmg, pose_name, config_->main_velocity_scaling_factor_,
                                     check_validity))
  {
    ROS_ERROR_STREAM_NAMED("arm_testing", "Unable to move to pose");
    return false;
  }
  ROS_INFO_STREAM_NAMED("arm_testing", "Spinning until shutdown requested");
  ros::spin();
  return true;
}

// Mode 11
bool ArmTesting::calibrateInCircle()
{
  // Choose which planning group to use
  JointModelGroup* arm_jmg = config_->right_arm_;
  if (!arm_jmg)
  {
    ROS_ERROR_STREAM_NAMED("arm_testing", "No joint model group for arm");
    return false;
  }

  // Get location of camera
  Eigen::Affine3d camera_pose;
  // TODO  planning_interface_->getPose(camera_pose, config_->right_camera_frame_);

  // Move camera pose forward away from camera
  Eigen::Affine3d translate_forward = Eigen::Affine3d::Identity();
  translate_forward.translation().x() += 1.0;  // TODO config_->camera_x_translation_from_bin_;
  translate_forward.translation().z() -= 0.15;
  camera_pose = translate_forward * camera_pose;

  // Debug
  visual_tools_->publishSphere(camera_pose, rvt::GREEN, rvt::LARGE);
  visual_tools_->publishXArrow(camera_pose, rvt::GREEN);

  // Collection of goal positions
  EigenSTL::vector_Affine3d waypoints;

  // Create circle of poses around center
  double radius = 0.05;
  double increment = 2 * M_PI / 4;
  visual_tools_->enableBatchPublishing(true);
  for (double angle = 0; angle <= 2 * M_PI; angle += increment)
  {
    // Rotate around circle
    Eigen::Affine3d rotation_transform = Eigen::Affine3d::Identity();
    rotation_transform.translation().z() += radius * cos(angle);
    rotation_transform.translation().y() += radius * sin(angle);

    Eigen::Affine3d new_point = rotation_transform * camera_pose;

    // Convert pose that has x arrow pointing to object, to pose that has z arrow pointing towards
    // object and x out in the grasp dir
    new_point = new_point * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY());
    // new_point = new_point * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ());

    // Debug
    // visual_tools_->publishZArrow(new_point, rvt::RED);

    // Translate to custom end effector geometry
    Eigen::Affine3d grasp_pose = new_point * grasp_datas_[arm_jmg]->grasp_pose_to_eef_pose_;
    // visual_tools_->publishZArrow(grasp_pose, rvt::PURPLE);
    visual_tools_->publishAxis(grasp_pose);

    // Add to trajectory
    waypoints.push_back(grasp_pose);
  }
  visual_tools_->triggerBatchPublishAndDisable();

  ROS_WARN_STREAM_NAMED("temp","Currently function for execution has been removed!");
  return false;
  /*
  if (!planning_interface_->moveCartesianWaypointPath(arm_jmg, waypoints))
  {
    ROS_ERROR_STREAM_NAMED("arm_testing", "Error executing path");
    return false;
  }
  */
  return true;
}

}  // end namespace
