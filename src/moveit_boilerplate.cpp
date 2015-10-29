/*********************************************************************
 * Software License Agreement
 *
 *  Copyright (c) 2015, Dave Coleman <dave@dav.ee>
 *  All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *********************************************************************/

/* Author: Dave Coleman <dave@dav.ee>
   Desc:   Main logic of picking
*/

// Command line arguments
#include <gflags/gflags.h>

// MoveItManipulation
#include <moveit_manipulation/moveit_boilerplate.h>

// MoveIt
#include <moveit/macros/console_colors.h>

namespace moveit_manipulation
{
DEFINE_bool(fake_execution, false, "Fake execution of motions");
DEFINE_int32(id, 0, "Identification number for various component modes");

MoveItBoilerplate::MoveItBoilerplate()
  : nh_private_("~")
{
  // Warn of fake modes
  if (FLAGS_fake_execution)
    ROS_WARN_STREAM_NAMED("moveit_boilerplate", "In fake execution mode");

  // Load the loader
  robot_model_loader_.reset(new robot_model_loader::RobotModelLoader(ROBOT_DESCRIPTION));

  // Load the robot model
  robot_model_ = robot_model_loader_->getModel();  // Get a shared pointer to the robot

  // Create the planning scene
  planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));

  // Get package path
  package_path_ = ros::package::getPath(package_name_);
  if (package_path_.empty())
    ROS_FATAL_STREAM_NAMED("product", "Unable to get " << package_name_ << " package path");

  // Load manipulation data for our robot
  config_.reset(new ManipulationData());
  config_->load(robot_model_, FLAGS_fake_execution, package_path_);

  // Create tf transformer
  tf_.reset(new tf::TransformListener(nh_private_));
  // TODO: remove these lines, only an attempt to fix loadPlanningSceneMonitor bug
  ros::spinOnce();

  // Load planning scene monitor
  if (!loadPlanningSceneMonitor())
  {
    ROS_ERROR_STREAM_NAMED("moveit_boilerplate", "Unable to load planning scene monitor");
  }

  // Load the Robot Viz Tools for publishing to Rviz
  loadVisualTools();

  // Load the remote control for dealing with GUIs
  remote_control_.reset(new RemoteControl(nh_private_));

  // Load grasp data specific to our robot
  grasp_datas_[config_->right_arm_].reset(
      new moveit_grasps::GraspData(nh_private_, config_->right_hand_name_, robot_model_));

  if (config_->dual_arm_)
    grasp_datas_[config_->left_arm_].reset(
        new moveit_grasps::GraspData(nh_private_, config_->left_hand_name_, robot_model_));

  // Create manipulation manager
  manipulation_.reset(new Manipulation(planning_scene_monitor_, config_,
                                       grasp_datas_, remote_control_, FLAGS_fake_execution));

  // Load trajectory IO class
  trajectory_io_.reset(new TrajectoryIO(remote_control_, config_, manipulation_, visual_tools_));

  // Show interactive marker
  //setupInteractiveMarker();

  ROS_INFO_STREAM_NAMED("moveit_boilerplate", "MoveItBoilerplate Ready.");
}

bool MoveItBoilerplate::checkSystemReady()
{
  std::cout << std::endl;
  std::cout << std::endl;
  std::cout << "-------------------------------------------------------" << std::endl;
  ROS_INFO_STREAM_NAMED("moveit_boilerplate", "Starting system ready check:");

  // Check joint model groups, assuming we are the jaco arm
  if (config_->right_arm_->getVariableCount() < 6 || config_->right_arm_->getVariableCount() > 7)
  {
    ROS_FATAL_STREAM_NAMED("moveit_boilerplate", "Incorrect number of joints for group "
                                               << config_->right_arm_->getName() << ", joints: "
                                               << config_->right_arm_->getVariableCount());
    return false;
  }
  JointModelGroup* ee_jmg = grasp_datas_[config_->right_arm_]->ee_jmg_;
  if (ee_jmg->getVariableCount() > 6)
  {
    ROS_FATAL_STREAM_NAMED("moveit_boilerplate", "Incorrect number of joints for group "
                                               << ee_jmg->getName()
                                               << ", joints: " << ee_jmg->getVariableCount());
    return false;
  }

  // Check trajectory execution manager
  if (!manipulation_->getExecutionInterface()->checkExecutionManager())
  {
    ROS_FATAL_STREAM_NAMED("moveit_boilerplate", "Trajectory controllers unable to connect");
    return false;
  }

  // Choose which planning group to use
  // JointModelGroup* arm_jmg = config_->dual_arm_ ? config_->both_arms_ : config_->right_arm_;

  // Check robot calibrated
  // TODO

  // Check gantry calibrated
  // TODO

  // Check end effectors calibrated
  // TODO

  ROS_INFO_STREAM_NAMED("moveit_boilerplate", "System ready check COMPLETE");
  std::cout << "-------------------------------------------------------" << std::endl;
  return true;
}

// Mode 8
bool MoveItBoilerplate::testEndEffectors()
{
  // Test visualization
  std::size_t i = 0;
  bool open;
  moveit::core::RobotStatePtr current_state = manipulation_->getCurrentState();
  while (ros::ok())
  {
    std::cout << std::endl << std::endl;
    if (i % 2 == 0)
    {
      std::cout << "Showing closed EE of state " << std::endl;

      open = false;
      // manipulation_->setStateWithOpenEE(open, current_state);
      // visual_tools_->publishRobotState(current_state);

      // Close all EEs
      manipulation_->openEEs(open);

      ros::Duration(2.0).sleep();
    }
    else
    {
      std::cout << "Showing open EE of state " << std::endl;

      open = true;
      // manipulation_->setStateWithOpenEE(open, current_state);
      // visual_tools_->publishRobotState(current_state);

      // Close all EEs
      manipulation_->openEEs(open);

      ros::Duration(2.0).sleep();
    }
    ++i;
  }

  ROS_INFO_STREAM_NAMED("moveit_boilerplate", "Done testing end effectors");
  return true;
}

// Mode 5
bool MoveItBoilerplate::testUpAndDown()
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
      manipulation_->executeVerticlePath(config_->right_arm_, lift_distance_desired,
                                         config_->lift_velocity_scaling_factor_, true);
      if (config_->dual_arm_)
        manipulation_->executeVerticlePath(config_->left_arm_, lift_distance_desired,
                                           config_->lift_velocity_scaling_factor_, true);
      ros::Duration(1.0).sleep();
    }
    else
    {
      std::cout << "Moving down ------------------------------------" << std::endl;
      manipulation_->executeVerticlePath(config_->right_arm_, lift_distance_desired,
                                         config_->lift_velocity_scaling_factor_, false);
      if (config_->dual_arm_)
        manipulation_->executeVerticlePath(config_->left_arm_, lift_distance_desired,
                                           config_->lift_velocity_scaling_factor_, false);
      ros::Duration(1.0).sleep();
    }
    ++i;
  }

  ROS_INFO_STREAM_NAMED("moveit_boilerplate", "Done testing up and down");
  return true;
}

// Mode 10
bool MoveItBoilerplate::testInAndOut()
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
      if (!manipulation_->executeRetreatPath(config_->right_arm_, approach_distance_desired, false))
        return false;
      if (config_->dual_arm_)
        if (!manipulation_->executeRetreatPath(config_->left_arm_, approach_distance_desired,
                                               false))
          return false;
      ros::Duration(1.0).sleep();
    }
    else
    {
      std::cout << "Moving out ------------------------------------" << std::endl;
      if (!manipulation_->executeRetreatPath(config_->right_arm_, approach_distance_desired, true))
        return false;
      if (config_->dual_arm_)
        if (!manipulation_->executeRetreatPath(config_->left_arm_, approach_distance_desired, true))
          return false;
      ros::Duration(1.0).sleep();
    }
    ++i;
  }

  ROS_INFO_STREAM_NAMED("moveit_boilerplate", "Done testing in and out");
  return true;
}

// Mode 41
bool MoveItBoilerplate::getSRDFPose()
{
  ROS_DEBUG_STREAM_NAMED("moveit_boilerplate", "Get SRDF pose");

  // Choose which planning group to use
  JointModelGroup* arm_jmg = config_->dual_arm_ ? config_->both_arms_ : config_->right_arm_;
  const std::vector<const moveit::core::JointModel*> joints = arm_jmg->getJointModels();

  while (ros::ok())
  {
    ROS_INFO_STREAM("SDF Code for joint values pose:\n");

    // Get current state after grasping
    moveit::core::RobotStatePtr current_state = manipulation_->getCurrentState();

    // Check if current state is valid
    // manipulation_->fixCurrentCollisionAndBounds(arm_jmg);

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
bool MoveItBoilerplate::testInCollision()
{
  while (ros::ok())
  {
    std::cout << std::endl;

    // For debugging in console
    manipulation_->showJointLimits(config_->right_arm_);

    // manipulation_->fixCurrentCollisionAndBounds(arm_jmg);
    manipulation_->checkCollisionAndBounds(manipulation_->getCurrentState());
    ros::Duration(0.1).sleep();
  }

  ROS_INFO_STREAM_NAMED("moveit_boilerplate", "Done checking if in collision");
  return true;
}

// Mode 6
bool MoveItBoilerplate::testRandomValidMotions()
{
  // Allow collision between Jacob and bottom for most links
  {
    planning_scene_monitor::LockedPlanningSceneRW scene(
        planning_scene_monitor_);  // Lock planning scene

    scene->getAllowedCollisionMatrixNonConst().setEntry("base_39", "frame", true);
    scene->getAllowedCollisionMatrixNonConst().setEntry("base_39", "gantry", true);
    scene->getAllowedCollisionMatrixNonConst().setEntry("base_39", "gantry_plate", true);
    scene->getAllowedCollisionMatrixNonConst().setEntry("base_39", "jaco2_link_base", true);
    scene->getAllowedCollisionMatrixNonConst().setEntry("base_39", "jaco2_link_1", true);
  }

  // Plan to random
  while (ros::ok())
  {
    static const std::size_t MAX_ATTEMPTS = 200;
    for (std::size_t i = 0; i < MAX_ATTEMPTS; ++i)
    {
      ROS_DEBUG_STREAM_NAMED("moveit_boilerplate", "Attempt " << i << " to plan to a random location");

      // Create start
      moveit::core::RobotStatePtr current_state = manipulation_->getCurrentState();

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
      if (manipulation_->checkCollisionAndBounds(current_state, goal_state, collision_verbose))
      {
        // Plan to this position
        bool verbose = true;
        bool execute_trajectory = true;
        if (manipulation_->move(current_state, goal_state, arm_jmg,
                                config_->main_velocity_scaling_factor_, verbose,
                                execute_trajectory))
        {
          ROS_INFO_STREAM_NAMED("moveit_boilerplate", "Planned to random valid state successfullly");
        }
        else
        {
          ROS_ERROR_STREAM_NAMED("moveit_boilerplate", "Failed to plan to random valid state");
          return false;
        }
      }
    }
    ROS_ERROR_STREAM_NAMED("moveit_boilerplate", "Unable to find random valid state after "
                                               << MAX_ATTEMPTS << " attempts");

    ros::Duration(1).sleep();
  }  // while

  ROS_INFO_STREAM_NAMED("moveit_boilerplate", "Done planning to random valid");
  return true;
}

// Mode 2
bool MoveItBoilerplate::testGoHome()
{
  ROS_DEBUG_STREAM_NAMED("moveit_boilerplate", "Going home");

  // Choose which planning group to use
  JointModelGroup* arm_jmg = config_->dual_arm_ ? config_->both_arms_ : config_->right_arm_;
  moveToStartPosition(arm_jmg);
  return true;
}

// Mode 17
bool MoveItBoilerplate::testJointLimits()
{
  ROS_INFO_STREAM_NAMED("moveit_boilerplate", "Testing joint limits");
  ROS_WARN_STREAM_NAMED("moveit_boilerplate", "DOES NOT CHECK FOR COLLISION");

  moveit::core::RobotStatePtr current_state = manipulation_->getCurrentState();

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
      ROS_INFO_STREAM_NAMED("moveit_boilerplate", "Sending joint " << joints[i]->getName()
                                                             << " to min position of "
                                                             << joint_position[0]);
      goal_state->setJointPositions(joints[i], joint_position);

      if (!manipulation_->executeState(goal_state, config_->right_arm_,
                                       config_->main_velocity_scaling_factor_))
      {
        ROS_ERROR_STREAM_NAMED("moveit_boilerplate", "Unable to move to min bound of "
                                                   << joint_position[0] << " on joint "
                                                   << joints[i]->getName());
      }
      ros::Duration(1.0).sleep();

      // Move to max bound
      std::cout << std::endl;
      std::cout << "-------------------------------------------------------" << std::endl;
      joint_position[0] = bound.max_position_ - reduce_bound;
      ROS_INFO_STREAM_NAMED("moveit_boilerplate", "Sending joint " << joints[i]->getName()
                                                             << " to max position of "
                                                             << joint_position[0]);
      goal_state->setJointPositions(joints[i], joint_position);

      if (!manipulation_->executeState(goal_state, config_->right_arm_,
                                       config_->main_velocity_scaling_factor_))
      {
        ROS_ERROR_STREAM_NAMED("moveit_boilerplate", "Unable to move to max bound of "
                                                   << joint_position[0] << " on joint "
                                                   << joints[i]->getName());
      }
      ros::Duration(1.0).sleep();
    }
  }

  ROS_INFO_STREAM_NAMED("moveit_boilerplate", "Done testing joint limits");
  return true;
}

bool MoveItBoilerplate::recordTrajectory()
{
  std::string file_path;
  const std::string file_name = "test_trajectory";
  trajectory_io_->getFilePath(file_path, file_name);

  // Start recording
  trajectory_io_->recordTrajectoryToFile(file_path);

  ROS_INFO_STREAM_NAMED("moveit_boilerplate", "Done recording");

  return true;
}

// Mode 34
bool MoveItBoilerplate::playbackTrajectory()
{
  // Choose which planning group to use
  JointModelGroup* arm_jmg = config_->arm_only_;
  if (!arm_jmg)
  {
    ROS_ERROR_STREAM_NAMED("moveit_boilerplate", "No joint model group for arm");
    return false;
  }

  // Start playing back file
  std::string file_path;
  const std::string file_name = "calibration_waypoints";
  trajectory_io_->getFilePath(file_path, file_name);

  if (!trajectory_io_->playbackWaypointsFromFile(file_path, arm_jmg,
                                                 config_->calibration_velocity_scaling_factor_))
  {
    ROS_ERROR_STREAM_NAMED("moveit_boilerplate", "Unable to playback CSV from file for pose waypoints");
    return false;
  }

  return true;
}

bool MoveItBoilerplate::moveToStartPosition(JointModelGroup* arm_jmg, bool check_validity)
{
  return manipulation_->moveToStartPosition(arm_jmg, check_validity);
}

bool MoveItBoilerplate::loadPlanningSceneMonitor()
{
  // Allows us to sycronize to Rviz and also publish collision objects to ourselves
  ROS_DEBUG_STREAM_NAMED("moveit_boilerplate", "Loading Planning Scene Monitor");
  static const std::string PLANNING_SCENE_MONITOR_NAME = "AmazonShelfWorld";
  planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(
      planning_scene_, robot_model_loader_, tf_, PLANNING_SCENE_MONITOR_NAME));
  ros::spinOnce();

  if (planning_scene_monitor_->getPlanningScene())
  {
    // Optional monitors to start:
    planning_scene_monitor_->startStateMonitor(config_->joint_state_topic_, "");
    planning_scene_monitor_->startPublishingPlanningScene(
        planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE, "picknik_planning_scene");
    planning_scene_monitor_->getPlanningScene()->setName("picknik_planning_scene");
  }
  else
  {
    ROS_ERROR_STREAM_NAMED("moveit_boilerplate", "Planning scene not configured");
    return false;
  }
  ros::spinOnce();
  ros::Duration(0.5).sleep();  // when at 0.1, i believe sometimes vjoint not properly loaded

  // Wait for complete state to be recieved
  bool wait_for_complete_state = false;
  // Break early
  if (!wait_for_complete_state)
    return true;

  std::vector<std::string> missing_joints;
  std::size_t counter = 0;
  while (!planning_scene_monitor_->getStateMonitor()->haveCompleteState() && ros::ok())
  {
    ROS_INFO_STREAM_THROTTLE_NAMED(1, "", "Waiting for complete state from topic "
                                                          << config_->joint_state_topic_);
    ros::Duration(0.1).sleep();
    ros::spinOnce();

    // Show unpublished joints
    if (counter % 10 == 0)
    {
      planning_scene_monitor_->getStateMonitor()->haveCompleteState(missing_joints);
      for (std::size_t i = 0; i < missing_joints.size(); ++i)
        ROS_WARN_STREAM_NAMED("moveit_boilerplate", "Unpublished joints: " << missing_joints[i]);
    }
    counter++;
  }
  ros::spinOnce();

  return true;
}

void MoveItBoilerplate::loadVisualTools()
{
  visual_tools_.reset(new mvt::MoveItVisualTools(robot_model_->getModelFrame(),
                                                 "/moveit_manipulation/markers", planning_scene_monitor_));

  visual_tools_->loadRobotStatePub("/moveit_manipulation/robot_state");
  visual_tools_->loadTrajectoryPub("/moveit_manipulation/display_trajectory");
  visual_tools_->loadMarkerPub();
  visual_tools_->setAlpha(0.8);
  visual_tools_->deleteAllMarkers();  // clear all old markers
  visual_tools_->setManualSceneUpdating(true);
  visual_tools_->hideRobot();  // show that things have been reset
}

void MoveItBoilerplate::publishCurrentState()
{
  planning_scene_monitor::LockedPlanningSceneRO scene(
      planning_scene_monitor_);  // Lock planning scene
  visual_tools_->publishRobotState(scene->getCurrentState(), rvt::PURPLE);
}

bool MoveItBoilerplate::allowCollisions(JointModelGroup* arm_jmg)
{
  // Allow collisions between frame of robot and floor
  {
    planning_scene_monitor::LockedPlanningSceneRW scene(planning_scene_monitor_);  // Lock planning
    collision_detection::AllowedCollisionMatrix& collision_matrix =
        scene->getAllowedCollisionMatrixNonConst();

    // Get links of end effector
    const std::vector<std::string>& ee_link_names =
        grasp_datas_[arm_jmg]->ee_jmg_->getLinkModelNames();
    for (std::size_t i = 0; i < ee_link_names.size(); ++i)
    {
      for (std::size_t j = i + 1; j < ee_link_names.size(); ++j)
      {
        // std::cout << "disabling collsion between " << ee_link_names[i] << " and " <<
        // ee_link_names[j] << std::endl;
        collision_matrix.setEntry(ee_link_names[i], ee_link_names[j], true);
      }
    }
  }

  return true;
}

// Mode 9
bool MoveItBoilerplate::gotoPose(const std::string& pose_name)
{
  ROS_INFO_STREAM_NAMED("moveit_boilerplate", "Going to pose " << pose_name);
  ros::Duration(1).sleep();
  ros::spinOnce();

  JointModelGroup* arm_jmg = config_->dual_arm_ ? config_->both_arms_ : config_->right_arm_;
  bool check_validity = true;

  if (!manipulation_->moveToSRDFPose(arm_jmg, pose_name, config_->main_velocity_scaling_factor_,
                                     check_validity))
  {
    ROS_ERROR_STREAM_NAMED("moveit_boilerplate", "Unable to move to pose");
    return false;
  }
  ROS_INFO_STREAM_NAMED("moveit_boilerplate", "Spinning until shutdown requested");
  ros::spin();
  return true;
}

// Mode 11
bool MoveItBoilerplate::calibrateInCircle()
{
  // Choose which planning group to use
  JointModelGroup* arm_jmg = config_->arm_only_;
  if (!arm_jmg)
  {
    ROS_ERROR_STREAM_NAMED("moveit_boilerplate", "No joint model group for arm");
    return false;
  }

  // Get location of camera
  Eigen::Affine3d camera_pose;
  // TODO  manipulation_->getPose(camera_pose, config_->right_camera_frame_);

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

  if (!manipulation_->moveCartesianWaypointPath(arm_jmg, waypoints))
  {
    ROS_ERROR_STREAM_NAMED("moveit_boilerplate", "Error executing path");
    return false;
  }

  return true;
}

}  // end namespace
