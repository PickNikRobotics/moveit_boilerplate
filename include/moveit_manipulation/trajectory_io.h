/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
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
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
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

#ifndef MOVEIT_MANIPULATION__TRAJECTORY_IO
#define MOVEIT_MANIPULATION__TRAJECTORY_IO

// PickNik
#include <moveit_manipulation/planning_interface.h>
#include <moveit_manipulation/namespaces.h>

namespace moveit_manipulation
{
class TrajectoryIO
{
public:
  /**
   * \brief Constructor
   */
  TrajectoryIO(RemoteControlPtr remote_control, ManipulationDataPtr config,
               PlanningInterfacePtr planning_interface, mvt::MoveItVisualToolsPtr visual_tools);

  /**
   * \brief Read a joint trajectory from CSV and execute on robot
   * \param file_name - location of file
   * \param arm_jmg - the kinematic chain of joint that should be controlled (a planning group)
   * \param velocity_scaling_factor - the percent of max speed all joints should be allowed to
   * utilize
   * \return true on success
   */
  bool playbackTrajectoryFromFile(const std::string& file_name, JointModelGroup* arm_jmg,
                                  double velocity_scaling_factor);

  /**
   * \brief Read a trajectory from CSV and execute on robot state by state
   * \param file_name - location of file
   * \param arm_jmg - the kinematic chain of joint that should be controlled (a planning group)
   * \param velocity_scaling_factor - the percent of max speed all joints should be allowed to
   * utilize
   * \return true on success
   */
  bool playbackTrajectoryFromFileInteractive(const std::string& file_name, JointModelGroup* arm_jmg,
                                             double velocity_scaling_factor);

  /**
   * \brief Record the entire state of a robot to file
   * \param file_name - location of file
   * \return true on success
   */
  bool recordTrajectoryToFile(const std::string& file_name);

  /**
   * \brief Read a waypoint trajectory from CSV and load into this class
   * \param file_name - location of file
   * \return true on success
   */
  bool loadWaypointsFromFile(const std::string& file_name);

  /** \brief Add current robot pose to trajectory */
  void addWaypoint(const Eigen::Affine3d& pose);

  /** \brief Delete all recorded waypoints */
  void clearWaypoints();
  
  std::vector<Eigen::Affine3d>& getWaypoints()
  {
    return waypoints_trajectory_;
  }

  /**
   * \brief Save a trajectory of poses to a file in CSV format
   * \return true on success
   */
  bool saveWaypointsToFile(const std::string& file_path);

  /**
   * \brief Convert a 6-vector of x,y,z, roll,pitch,yall to an Affine3d with quaternion, from a line
   * of a file
   */
  bool streamToAffine3d(Eigen::Affine3d& pose, const std::string& line);

  /**
   * \brief Get location to save a CSV file
   * \param file_path - the variable to populate with a path
   * \param file_name - the desired name of the file
   * \return true on success
   */
  bool getFilePath(std::string& file_path, const std::string& file_name) const;

private:
  // A shared node handle
  ros::NodeHandle nh_;

  // Common classes in moveit_manipulation
  RemoteControlPtr remote_control_;
  ManipulationDataPtr config_;
  PlanningInterfacePtr planning_interface_;
  mvt::MoveItVisualToolsPtr visual_tools_;

  // Trajectory to load/save to/from file
  std::vector<Eigen::Affine3d> waypoints_trajectory_;

};  // end class

// Create boost pointers for this class
typedef boost::shared_ptr<TrajectoryIO> TrajectoryIOPtr;
typedef boost::shared_ptr<const TrajectoryIO> TrajectoryIOConstPtr;

}  // end namespace

#endif
