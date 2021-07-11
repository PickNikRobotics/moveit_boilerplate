/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, PickNik LLC
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

/* Author: Dave Coleman, Henning Kayser
   Desc:   Interface for quickly setting up MoveIt
*/

#ifndef MOVEIT_BASE_MOVEIT_BASE_H
#define MOVEIT_BASE_MOVEIT_BASE_H

// C++
#include <string>

// ROS
#include <ros/ros.h>
#include <tf2_ros/buffer.h>

// MoveIt
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/macros/console_colors.h>
#include <moveit/trajectory_execution_manager/trajectory_execution_manager.h>

// Visual tools
#include <moveit_visual_tools/moveit_visual_tools.h>

// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace moveit_base
{
static const std::string ROBOT_DESCRIPTION = "robot_description";

class MoveItBase
{
public:
  /**
   * \brief Constructor
   */
  MoveItBase();

  /**
   * \brief Desctructor
   */
  ~MoveItBase()
  {
  }

  /**
   * \brief Initialize all of the MoveIt! functionality
   */
  bool init(ros::NodeHandle& nh);

  /**
   * \brief Connect to the MoveIt! planning scene messages
   *        Note: this is called within the init() function
   */
  bool loadPlanningSceneMonitor(const std::string& joint_state_topic);

  /**
   * \brief Load visual tools
   *        Note: this is called within the init() function
   */
  void loadVisualTools(const std::string& rviz_markers_topic, const std::string& rviz_robot_state_topic,
                       const std::string& rviz_trajectory_topic);

  /** \brief Output to console the current state of the robot's joint limits */
  bool showJointLimits(const moveit::core::JointModelGroup* jmg);

  /**
   * \brief Use the planning scene to get the robot's current state
   */
  moveit::core::RobotStatePtr getCurrentState();

  /**
   * \brief Get the published tf pose from two frames
   * \param from_frame e.g. 'world'
   * \param to_frame e.g. 'thing'
   * \param pose - the returned valie
   * \return false on missing transform, may just need to wait a little longer and retry
   */
  bool getTFTransform(const std::string& from_frame, const std::string& to_frame, Eigen::Affine3d& pose);

  /** \brief Getter for visual tools */
  moveit_visual_tools::MoveItVisualToolsPtr getVisualTools()
  {
    return visual_tools_;
  }

  /** \brief Getter for robot model */
  const robot_model::RobotModelPtr getRobotModel() const
  {
    return robot_model_;
  }

  /** \brief Getting for planning scene monitor */
  planning_scene_monitor::PlanningSceneMonitorPtr getPlanningSceneMonitor()
  {
    return planning_scene_monitor_;
  }

  void setStartStateToCurrentState()
  {
    start_state_ = NULL;
  }

  void setStartState(const moveit::core::RobotStatePtr& robot_state)
  {
    start_state_ = robot_state;
  }

  void setDefaultPlannerId(const std::string& planner_id)
  {
    default_planner_id_ = planner_id;
  }

  void setAllowedPlanningTime(double allowed_planning_time)
  {
    allowed_planning_time_ = allowed_planning_time;
  }

  void setVelocityAccelelerationScaling(double max_velocity_scaling_factor,
                                          double max_acceleration_scaling_factor)
  {
    max_velocity_scaling_factor_ = max_velocity_scaling_factor;
    max_acceleration_scaling_factor_ = max_acceleration_scaling_factor;
  }

  bool setDefaultPlanningGroup(const std::string& planning_group)
  {
    if (!robot_model_->getJointModelGroup(planning_group))
    {
      ROS_ERROR_STREAM("Specified default planning group " << planning_group
          << " is not part of the robot model");
      return false;
    }
    default_planning_group_ = planning_group;
    return true;
  }

  bool planTo(const std::string& pose_id, std::string planning_group = "")
  {
    if (planning_group.empty())
    {
      if (default_planning_group_.empty())
      {
        ROS_ERROR("Failed to plan - no planning group specified");
        return false;
      }
      planning_group = default_planning_group_;
    }
    moveit::core::RobotState robot_state(*getCurrentState());
    if (!robot_state.setToDefaultValues(robot_state.getJointModelGroup(planning_group), pose_id))
    {
      ROS_ERROR_STREAM("Failed to set robot state to default target '" << pose_id << "'");
      return false;
    }
    robot_state.update();
    return planTo(robot_state, planning_group);
  }

  bool planTo(std::vector<double> joint_values, std::string planning_group = "")
  {
    if (planning_group.empty())
    {
      if (default_planning_group_.empty())
      {
        ROS_ERROR("Failed to plan - no planning group specified");
        return false;
      }
      planning_group = default_planning_group_;
    }
    moveit::core::RobotState robot_state(*getCurrentState());
    robot_state.setJointGroupPositions(planning_group, joint_values);
    robot_state.update();
    return planTo(robot_state, planning_group);
  }

  bool planTo(const moveit::core::RobotState& robot_state, std::string planning_group);

  bool enableExecutionController(const std::string& group_name);

  bool execute(bool blocking = true);

  robot_trajectory::RobotTrajectoryPtr getLastSolutionTrajectory()
  {
    return last_solution_trajectory_;
  }
protected:
  // A shared node handle
  ros::NodeHandle nh_;

  // Short name of this class
  std::string name_ = "moveit_base";

  std::string default_planning_group_;
  std::string default_planner_id_;
  double allowed_planning_time_ = 5.0;
  double max_velocity_scaling_factor_ = 1.0;
  double max_acceleration_scaling_factor_ = 1.0;

  // Settings
  std::string planning_scene_topic_;

  // Transform
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  // For visualizing things in rviz
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  // Core MoveIt components
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  robot_model::RobotModelPtr robot_model_;
  planning_scene::PlanningScenePtr planning_scene_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  planning_pipeline::PlanningPipelinePtr planning_pipeline_;
  robot_trajectory::RobotTrajectoryPtr last_solution_trajectory_;
  trajectory_execution_manager::TrajectoryExecutionManagerPtr trajectory_execution_manager_;

  // Allocated memory for robot state
  moveit::core::RobotStatePtr current_state_;
  moveit::core::RobotStatePtr start_state_;

};  // end class

}  // namespace moveit_base

#endif  // MOVEIT_BASE_MOVEIT_BASE_H
