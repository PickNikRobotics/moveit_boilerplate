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
   Desc:   Loads from and saves to file trajectories in joint states and EE poses
*/

#ifndef MOVEIT_BOILERPLATE_TRAJECTORY_IO_H
#define MOVEIT_BOILERPLATE_TRAJECTORY_IO_H

// C++
#include <string>
#include <vector>

// PickNik
#include <moveit_boilerplate/namespaces.h>

// MoveIt
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

// Visual tools
#include <moveit_visual_tools/moveit_visual_tools.h>

namespace moveit_boilerplate
{
/** \brief Struct for storing the pose of an end effector with a time */
struct TimePose
{
  TimePose(double time, Eigen::Affine3d pose) : time_(time), pose_(pose)
  {
  }

  double time_;
  Eigen::Affine3d pose_;
};

class TrajectoryIO
{
public:
  /**
   * \brief Constructor
   */
  TrajectoryIO(psm::PlanningSceneMonitorPtr planning_scene_monitor, mvt::MoveItVisualToolsPtr visual_tools);

  // JOINT TRAJECTORY ------------------------------------------------------------------

  /**
   * \brief Read a joint trajectory from CSV
   * \param file_name - location of file
   * \param arm_jmg - the kinematic chain of joints that should be controlled (a planning group)
   * \param header - if true, skips first line of CSV to avoid reading the title of each column
   * \return true on success
   */
  bool loadJointTrajectoryFromFile(const std::string& file_name, JointModelGroup* arm_jmg, bool header = false);

  /**
   * \brief Read a joint trajectory from string
   * \param input - string with comma separated values and line breaks for each waypoint
   * \param arm_jmg - the kinematic chain of joints that should be controlled (a planning group)
   * \return true on success
   */
  bool loadJointTrajectoryFromStream(std::istringstream& input_stream, JointModelGroup* arm_jmg);

  /**
   * \brief Record the entire state of a robot to file
   * \param file_name - location of file
   * \return true on success
   */
  bool saveJointTrajectoryToFile(const std::string& file_name);

  robot_trajectory::RobotTrajectoryPtr getJointTrajectory()
  {
    return joint_trajectory_;
  }

  // CARTESIAN TRAJECTORY ------------------------------------------------------------------

  /**
   * \brief Read a waypoint trajectory from CSV and load into this class
   * \param file_name - location of file
   * \return true on success
   */
  bool loadCartTrajectoryFromFile(const std::string& file_name);

  /** \brief Add current robot pose to trajectory */
  void addCartWaypoint(const Eigen::Affine3d& pose, const double& sec = 2.0);

  /** \brief Delete all recorded waypoints */
  void clearCartWaypoints();

  std::vector<TimePose>& getCartWaypoints()
  {
    return cartesian_trajectory_;
  }

  /**
   * \brief Save a trajectory of poses to a file in CSV format
   * \return true on success
   */
  bool saveCartTrajectoryToFile(const std::string& file_path);

  /**
   * \brief Convert a 6-vector of x,y,z, roll,pitch,yall to an Affine3d with quaternion, from a line
   * of a file
   * \param pose - end effector location to read from file
   * \param sec - seconds to execute this waypoint before executing next
   * \param line - single record from file
   */
  bool streamToAffine3d(Eigen::Affine3d& pose, double& sec, const std::string& line);

  // GENERIC UTILS ------------------------------------------------------------------

  /**
   * \brief Get location to save a CSV file
   * \param file_path - the variable to populate with a path
   * \param file_name - the desired name of the file
   * \return true on success
   */
  bool getFilePath(std::string& file_path, const std::string& file_name);

private:
  /** \brief Use the planning scene to get the robot's current state */
  moveit::core::RobotStatePtr getCurrentState();

  // Short class name
  std::string name_;

  // A shared node handle
  ros::NodeHandle nh_;

  // Core MoveIt components
  psm::PlanningSceneMonitorPtr planning_scene_monitor_;

  // Common classes in moveit_boilerplate
  mvt::MoveItVisualToolsPtr visual_tools_;
  std::string package_path_;

  // Allocated memory for robot state
  moveit::core::RobotStatePtr current_state_;

  // JOINT TRAJECTORY ------------------------------------------------------------------

  // Joint trajectory to load/save to/from file
  robot_trajectory::RobotTrajectoryPtr joint_trajectory_;

  // CARTESIAN TRAJECTORY ------------------------------------------------------------------

  // Waypoint trajectory to load/save to/from file
  std::vector<TimePose> cartesian_trajectory_;
};  // end class

// Create boost pointers for this class
typedef boost::shared_ptr<TrajectoryIO> TrajectoryIOPtr;
typedef boost::shared_ptr<const TrajectoryIO> TrajectoryIOConstPtr;

}  // namespace moveit_boilerplate

#endif  // MOVEIT_BOILERPLATE_TRAJECTORY_IO_H
