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
   Desc:   Main logic of APC challenge
*/

#ifndef MOVEIT_MANIPULATION__MOVEIT_BOILERPLATE
#define MOVEIT_MANIPULATION__MOVEIT_BOILERPLATE

// MoveItManipulation
#include <moveit_manipulation/namespaces.h>
#include <moveit_manipulation/manipulation.h>
#include <moveit_manipulation/trajectory_io.h>
#include <moveit_manipulation/manipulation_data.h>
#include <moveit_manipulation/remote_control.h>
#include <moveit_manipulation/remote_control.h>

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>

namespace moveit_manipulation
{
static const std::string ROBOT_DESCRIPTION = "robot_description";
static const std::string JOINT_STATE_TOPIC = "/robot/joint_states";

class MoveItBoilerplate
{
public:
  /**
   * \brief Constructor
   */
  MoveItBoilerplate();

  /**
   * \brief Check if all communication is properly active
   * \return true on success
   */
  bool checkSystemReady();

  /**
   * \brief Test the end effectors
   * \param input - description
   * \return true on success
   */
  bool testEndEffectors();

  /**
   * \brief Testing ideal attached object
   * \return true on success
   */
  bool testIdealAttachedCollisionObject();

  /**
   * \brief Simple script to move hand up and down on z axis from whereever it currently is
   * \return true on success
   */
  bool testUpAndDown();

  /**
   * \brief Move hand in and out of bin from whereever it currently is
   * \return true on success
   */
  bool testInAndOut();

  /**
   * \brief Check if current state is in collision
   * \return true on success
   */
  bool testInCollision();

  /**
   * \brief Plan to random valid motions
   * \return true on success
   */
  bool testRandomValidMotions();

  /**
   * \brief Test moving joints to extreme limits
   * \return true on success
   */
  bool testJointLimits();

  /**
   * \brief Send arm(s) to home position
   * \return true on success
   */
  bool testGoHome();

  /**
   * \brief Get cartesian path for grasping object
   * \return true on success
   */
  bool testApproachLiftRetreat();

  /**
   * \brief Get the XML of a SDF pose of joints
   * \return true on success
   */
  bool getSRDFPose();

  /**
   * \brief Test grasp generator abilities and score results
   * \return true on success
   */
  bool testGraspGenerator();

  /**
   * \brief Record trajectory
   * \return true on success
   */
  bool recordTrajectory();

  /**
   * \return true on success
   */
  bool playbackTrajectory();

  /**
   * \brief
   * \param input - description
   * \return true on success
   */
  bool moveToStartPosition(JointModelGroup* arm_jmg, bool check_validity = true);

  /**
   * \brief Connect to the MoveIt! planning scene messages
   */
  bool loadPlanningSceneMonitor();

  /**
   * \brief Load visual tools
   */
  void loadVisualTools();
  
  /**
   * \brief Publish where the robot currently is
   */
  void publishCurrentState();

  /**
   * \brief Disable collision checking for certain pairs
   */
  bool allowCollisions(JointModelGroup* arm_jmg);

  /**
   * \brief Move to a pose named in the SRDF
   * \param pose_name
   * \return true on success
   */
  bool gotoPose(const std::string& pose_name);

  /**
   * \brief Move arm in circle for calibration
   * \return true on success
   */
  bool calibrateInCircle();

  /**
   * \brief Playback waypoint path specified in a csv
   * \return true on success
   */
  bool playbackWaypointsFromFile();

  /** \brief Allow interactive markers to control robot */
  void enableTeleoperation();

  planning_scene_monitor::PlanningSceneMonitorPtr getPlanningSceneMonitor() const
  {
    return planning_scene_monitor_;
  }

protected:

  // A shared node handle
  ros::NodeHandle nh_private_;
  ros::NodeHandle nh_root_;
  boost::shared_ptr<tf::TransformListener> tf_;

  // For visualizing things in rviz
  mvt::MoveItVisualToolsPtr visual_tools_;

  // Core MoveIt components
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  robot_model::RobotModelPtr robot_model_;
  planning_scene::PlanningScenePtr planning_scene_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  // File path to ROS package on drive
  std::string package_name_ = "moveit_manipulation";
  std::string package_path_;

  // Remote control for dealing with GUIs
  RemoteControlPtr remote_control_;

  // Main worker
  ManipulationPtr manipulation_;

  // Robot-sepcific data
  ManipulationDataPtr config_;

  // Robot-specific data for generating grasps
  moveit_grasps::GraspDatas grasp_datas_;

  // Allow loading and saving trajectories to file
  TrajectoryIOPtr trajectory_io_;

};  // end class

}  // end namespace

#endif
