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
   Desc:   Base class for using MoveIt! in C++
*/

#ifndef MOVEIT_MANIPULATION__MOVEIT_BOILERPLATE
#define MOVEIT_MANIPULATION__MOVEIT_BOILERPLATE

// MoveItManipulation
#include <moveit_manipulation/namespaces.h>
#include <moveit_manipulation/planning_interface.h>
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
//static const std::string JOINT_STATE_TOPIC = "/robot/joint_states";

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
   * \brief Connect to the MoveIt! planning scene messages
   */
  bool loadPlanningSceneMonitor();

  /**
   * \brief Load visual tools
   */
  void loadVisualTools();
  
  /**
   * \brief Disable collision checking for certain pairs
   */
  bool allowCollisions(JointModelGroup* arm_jmg);

  /**
   * \brief Use the planning scene to get the robot's current state
   */
  moveit::core::RobotStatePtr getCurrentState();

  /*planning_scene_monitor::PlanningSceneMonitorPtr getPlanningSceneMonitor() const
  {
    return planning_scene_monitor_;
  }
  */

protected:

  // A shared node handle
  ros::NodeHandle nh_;
  //ros::NodeHandle nh_root_;
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
  PlanningInterfacePtr planning_interface_;

  // Robot-sepcific data
  ManipulationDataPtr config_;

  // Robot-specific data for generating grasps
  moveit_grasps::GraspDatas grasp_datas_;

  // Allow loading and saving trajectories to file
  TrajectoryIOPtr trajectory_io_;

};  // end class

}  // end namespace

#endif
