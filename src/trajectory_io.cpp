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
           There are essentially two modes to this class, saving entire joint states
           or saving ee poses.
*/

// MoveItManipuation
#include <moveit_boilerplate/trajectory_io.h>

// basic file operations
#include <iostream>
#include <fstream>

// MoveIt
#include <moveit/robot_state/conversions.h>

// Boost
#include <boost/filesystem.hpp>

namespace moveit_boilerplate
{
TrajectoryIO::TrajectoryIO(psm::PlanningSceneMonitorPtr planning_scene_monitor,
                           mvt::MoveItVisualToolsPtr visual_tools)
  : planning_scene_monitor_(planning_scene_monitor)
  , visual_tools_(visual_tools)
{
  // Create initial robot state
  {
    psm::LockedPlanningSceneRO scene(planning_scene_monitor_);  // Lock planning scene
    current_state_.reset(new moveit::core::RobotState(scene->getCurrentState()));
  }  // end scoped pointer of locked planning scene
}

bool TrajectoryIO::loadJointTrajectoryFromFile(const std::string& file_name,
                                               JointModelGroup* arm_jmg)
{
  std::ifstream input_file;
  input_file.open(file_name.c_str());
  ROS_DEBUG_STREAM_NAMED("trajectory_io", "Loading trajectory from file " << file_name);

  std::string line;
  getCurrentState();

  joint_trajectory_.reset(new robot_trajectory::RobotTrajectory(current_state_->getRobotModel(), arm_jmg));
  double dummy_dt = 1;  // temp value

  // Read each line
  while (std::getline(input_file, line))
  {
    // Convert line to a robot state
    moveit::core::RobotStatePtr new_state(new moveit::core::RobotState(*current_state_));
    moveit::core::streamToRobotState(*new_state, line, ",");
    joint_trajectory_->addSuffixWayPoint(new_state, dummy_dt);
  }

  // Close file
  input_file.close();

  // Error check
  if (joint_trajectory_->getWayPointCount() == 0)
  {
    ROS_ERROR_STREAM_NAMED("trajectory_io", "No states loaded from CSV file " << file_name);
    return false;
  }

  return true;
}

bool TrajectoryIO::loadJointTrajectoryFromStream(std::istringstream& input_stream, JointModelGroup* arm_jmg)
{
  ROS_DEBUG_STREAM_NAMED("trajectory_io", "Loading trajectory from string.");

  std::string line;
  getCurrentState();
  std::cout << "here " << std::endl;
  joint_trajectory_.reset(new robot_trajectory::RobotTrajectory(current_state_->getRobotModel(), arm_jmg));
  double dummy_dt = 1;  // temp value

  // Read each line
  std::cout << "before get line " << std::endl;
  while (std::getline(input_stream, line))
  {
    ROS_WARN_STREAM_NAMED("temp","temp hack");
    line += ",0.0";
    std::cout << "line: " << line << std::endl;

    // Convert line to a robot state
    moveit::core::RobotStatePtr new_state(new moveit::core::RobotState(*current_state_));
    moveit::core::streamToRobotState(*new_state, line, ",");
    joint_trajectory_->addSuffixWayPoint(new_state, dummy_dt);
  }

  // Error check
  if (joint_trajectory_->getWayPointCount() == 0)
  {
    ROS_ERROR_STREAM_NAMED("trajectory_io", "No states loaded from string");
    return false;
  }

  return true;
}

bool TrajectoryIO::saveJointTrajectoryToFile(const std::string& file_path)
{
  bool include_header = false;

  std::ofstream output_file;
  output_file.open(file_path.c_str());
  ROS_DEBUG_STREAM_NAMED("trajectory_io", "Saving joint trajectory to file " << file_path);

  for (std::size_t i = 0; i < joint_trajectory_->getWayPointCount(); ++i)
  {
    ROS_INFO_STREAM_THROTTLE_NAMED(1, "trajectory_io", "Saving waypoint #" << i);

    moveit::core::robotStateToStream(joint_trajectory_->getWayPoint(i), output_file,
                                     include_header);
  }

  output_file.close();
  return true;
}

bool TrajectoryIO::loadCartTrajectoryFromFile(const std::string& file_name)
{
  std::ifstream input_file;
  std::string line;
  input_file.open(file_name.c_str());
  ROS_DEBUG_STREAM_NAMED("trajectory_io", "Loading waypoints from file " << file_name);

  // Read each line
  while (std::getline(input_file, line))
  {
    // Ignore empty lines
    if (line.empty())
      continue;

    // Convert line to a robot state
    Eigen::Affine3d pose;
    double sec;
    streamToAffine3d(pose, sec, line);

    // Debug
    visual_tools_->publishZArrow(pose, rvt::RED);

    cartesian_trajectory_.push_back(TimePose(sec,pose));
  }

  // Close file
  input_file.close();

  // Error check
  if (cartesian_trajectory_.empty())
  {
    ROS_ERROR_STREAM_NAMED("trajectory_io", "No waypoints loaded from CSV file " << file_name);
    return false;
  }

  ROS_INFO_STREAM_NAMED("trajectory_io","Loaded " << cartesian_trajectory_.size() << " waypoints from file");

  return true;
}

void TrajectoryIO::addCartWaypoint(const Eigen::Affine3d& pose, const double &sec)
{
  cartesian_trajectory_.push_back(TimePose(sec,pose));
}

void TrajectoryIO::clearCartWaypoints()
{
  cartesian_trajectory_.clear();
}

bool TrajectoryIO::saveCartTrajectoryToFile(const std::string& file_path)
{
  if (cartesian_trajectory_.empty())
    ROS_WARN_STREAM_NAMED("trajectory_io","Saving empty waypoint trajectory");

  std::ofstream output_file;
  output_file.open(file_path.c_str());
  ROS_DEBUG_STREAM_NAMED("trajectory_io", "Saving waypoints trajectory to file " << file_path);

  std::vector<double> xyzrpy;
  for (std::size_t i = 0; i < cartesian_trajectory_.size(); ++i)
  {
    visual_tools_->convertToXYZRPY(cartesian_trajectory_[i].pose_, xyzrpy);
    output_file << xyzrpy[0] << ", "
                << xyzrpy[1] << ", "
                << xyzrpy[2] << ", "
                << xyzrpy[3] << ", "
                << xyzrpy[4] << ", "
                << xyzrpy[5] << ", "
                << cartesian_trajectory_[i].time_
                << std::endl;
  }

  output_file.close();
  return true;
}

bool TrajectoryIO::getFilePath(std::string& file_path, const std::string& file_name)
{
  namespace fs = boost::filesystem;

  // Get package path
  if (package_path_.empty())
    package_path_ = ros::package::getPath("moveit_boilerplate");
  if (package_path_.empty())
  {
    ROS_ERROR_STREAM_NAMED("product", "Unable to get moveit_boilerplate package path");
    return false;
  }

  // Check that the directory exists, if not, create it
  fs::path path;
  path = fs::path(package_path_ + "/trajectories");

  boost::system::error_code returnedError;
  fs::create_directories(path, returnedError);

  // Error check
  if (returnedError)
  {
    ROS_ERROR_STREAM_NAMED("trajectory_io", "Unable to create directory " << path.string());
    return false;
  }

  // Directories successfully created, append the group name as the file name
  path = path / fs::path(file_name + ".csv");
  file_path = path.string();

  ROS_DEBUG_STREAM_NAMED("manipulation.file_path", "Using full file path" << file_path);
  return true;
}

bool TrajectoryIO::streamToAffine3d(Eigen::Affine3d& pose, double &sec, const std::string& line)
{
  std::stringstream line_stream(line);
  std::string cell;
  std::vector<double> transform6;
  transform6.resize(6);

  // For each item/column
  for (std::size_t i = 0; i < transform6.size(); ++i)
  {
    // Get a variable
    if (!std::getline(line_stream, cell, ','))
    {
      ROS_ERROR_STREAM_NAMED("trajectory_io", "Missing variable " << i << " on cell '" << cell
                                                                  << "' line '" << line << "'");
      return false;
    }

    transform6[i] = atof(cell.c_str()); // TODO improve with boost cast
  }

  // Get time
  if (!std::getline(line_stream, cell, ','))
  {
    ROS_WARN_STREAM_NAMED("trajectory_io","No time available");
    return false;
  }

  sec = atof(cell.c_str()); // TODO improve with boost cast

  // Convert to eigen
  pose = visual_tools_->convertFromXYZRPY(transform6);

  return true;
}

moveit::core::RobotStatePtr TrajectoryIO::getCurrentState()
{

  // Get the real current state
  psm::LockedPlanningSceneRO scene(planning_scene_monitor_);  // Lock planning scene
  (*current_state_) = scene->getCurrentState();
  return current_state_;
}

}  // end namespace
