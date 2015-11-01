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

#include <moveit_manipulation/trajectory_io.h>

// basic file operations
#include <iostream>
#include <fstream>

// MoveIt
#include <moveit/robot_state/conversions.h>

// Boost
#include <boost/filesystem.hpp>

namespace moveit_manipulation
{
TrajectoryIO::TrajectoryIO(RemoteControlPtr remote_control, ManipulationDataPtr config, 
			   PlanningInterfacePtr planning_interface, mvt::MoveItVisualToolsPtr visual_tools)
  : remote_control_(remote_control)
  , config_(config)
  , planning_interface_(planning_interface)
  , visual_tools_(visual_tools)
{
}

bool TrajectoryIO::playbackTrajectoryFromFile(const std::string& file_name,
                                              JointModelGroup* arm_jmg,
                                              double velocity_scaling_factor)
{
  std::ifstream input_file;
  input_file.open(file_name.c_str());
  ROS_DEBUG_STREAM_NAMED("manipultion", "Loading trajectory from file " << file_name);

  std::string line;
  moveit::core::RobotStatePtr current_state = planning_interface_->getCurrentState();

  robot_trajectory::RobotTrajectoryPtr robot_traj(
      new robot_trajectory::RobotTrajectory(current_state->getRobotModel(), arm_jmg));
  double dummy_dt = 1;  // temp value

  // Read each line
  while (std::getline(input_file, line))
  {
    // Convert line to a robot state
    moveit::core::RobotStatePtr new_state(new moveit::core::RobotState(*current_state));
    moveit::core::streamToRobotState(*new_state, line, ",");
    robot_traj->addSuffixWayPoint(new_state, dummy_dt);
  }

  // Close file
  input_file.close();

  // Error check
  if (robot_traj->getWayPointCount() == 0)
  {
    ROS_ERROR_STREAM_NAMED("manipultion", "No states loaded from CSV file " << file_name);
    return false;
  }

  // Unwrap joint values if needed

  // Interpolate between each point
  double discretization = 0.25;
  planning_interface_->interpolate(robot_traj, discretization);

  // Perform iterative parabolic smoothing
  planning_interface_->getIterativeSmoother().computeTimeStamps(*robot_traj, velocity_scaling_factor);

  // Convert trajectory to a message
  moveit_msgs::RobotTrajectory trajectory_msg;
  robot_traj->getRobotTrajectoryMsg(trajectory_msg);

  std::cout << std::endl << std::endl;
  std::cout << "-------------------------------------------------------" << std::endl;
  std::cout << "MOVING ARM TO START OF TRAJECTORY" << std::endl;
  std::cout << "-------------------------------------------------------" << std::endl;

  // Plan to start state of trajectory
  bool verbose = true;
  bool execute_trajectory = true;
  bool check_validity = true;
  ROS_INFO_STREAM_NAMED("trajectory_io", "Moving to start state of trajectory");
  if (!planning_interface_->move(current_state, robot_traj->getFirstWayPointPtr(), arm_jmg,
                           config_->main_velocity_scaling_factor_, verbose, execute_trajectory,
                           check_validity))
  {
    ROS_ERROR_STREAM_NAMED("manipultion", "Unable to plan");
    return false;
  }

  std::cout << std::endl;
  std::cout << "-------------------------------------------------------" << std::endl;
  std::cout << "PLAYING BACK TRAJECTORY" << std::endl;
  std::cout << "-------------------------------------------------------" << std::endl;

  // Execute
  if (!planning_interface_->getExecutionInterface()->executeTrajectory(trajectory_msg, arm_jmg))
  {
    ROS_ERROR_STREAM_NAMED("trajectory_io", "Failed to execute trajectory");
    return false;
  }

  return true;
}

bool TrajectoryIO::playbackWaypointsFromFile(const std::string& file_name, JointModelGroup* arm_jmg,
                                             double velocity_scaling_factor)
{
  std::ifstream input_file;
  std::string line;
  input_file.open(file_name.c_str());
  ROS_DEBUG_STREAM_NAMED("manipultion", "Loading waypoints from file " << file_name);

  // Create desired trajectory
  EigenSTL::vector_Affine3d waypoints;

  // Read each line
  while (std::getline(input_file, line))
  {
    // Convert line to a robot state
    Eigen::Affine3d pose;
    streamToAffine3d(pose, line, " ");

    // Convert pose that has x arrow pointing to object, to pose that has z arrow pointing towards
    // object and x out in the grasp dir
    pose = pose * Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitZ());

    Eigen::Affine3d transform =
        rvt::RvizVisualTools::convertXYZRPY(0.15, -0.01, 0, 0, 0, 0);  // from testPose()
    pose = transform * pose;

    // Debug
    // visual_tools_->publishZArrow(new_point, rvt::RED);

    // Translate to custom end effector geometry
    // Eigen::Affine3d grasp_pose = new_point * grasp_datas_[arm_jmg]->grasp_pose_to_eef_pose_;

    waypoints.push_back(pose);
  }

  // Close file
  input_file.close();

  // Error check
  if (waypoints.empty())
  {
    ROS_ERROR_STREAM_NAMED("trajectory_io", "No waypoints loaded from CSV file " << file_name);
    return false;
  }

  // Visualize
  if (true)
  {
    visual_tools_->enableBatchPublishing(true);
    for (std::size_t i = 0; i < waypoints.size(); ++i)
    {
      // printTransform(waypoints[i]);

      // visual_tools_->publishAxis(waypoints[i]);
      // visual_tools_->publishZArrow(waypoints[i]);
      visual_tools_->publishSphere(waypoints[i]);
    }
    visual_tools_->triggerBatchPublishAndDisable();
  }

  // Plan and move
  // TODO restore this function
  ROS_WARN_STREAM_NAMED("temp","Currently function for execution has been removed!");
  return false;
  // if (!planning_interface_->moveCartesianWaypointPath(arm_jmg, waypoints))
  // {
  //   ROS_ERROR_STREAM_NAMED("trajectory_io", "Error executing path");
  //   return false;
  // }

  return true;
}

bool TrajectoryIO::recordTrajectoryToFile(const std::string& file_path)
{
  bool include_header = false;

  std::ofstream output_file;
  output_file.open(file_path.c_str());
  ROS_DEBUG_STREAM_NAMED("trajectory_io", "Saving bin trajectory to file " << file_path);

  remote_control_->waitForNextStep("record trajectory");

  std::cout << std::endl << std::endl << std::endl;
  std::cout << "-------------------------------------------------------" << std::endl;
  std::cout << "START MOVING ARM " << std::endl;
  std::cout << "Press stop button to end recording " << std::endl;
  std::cout << "-------------------------------------------------------" << std::endl;

  std::size_t counter = 0;
  while (ros::ok() && !remote_control_->getStop())
  {
    ROS_INFO_STREAM_THROTTLE_NAMED(1, "trajectory_io", "Recording waypoint #" << counter++);

    moveit::core::robotStateToStream(*planning_interface_->getCurrentState(), output_file,
                                     include_header);

    ros::Duration(0.25).sleep();
  }

  // Reset the stop button
  remote_control_->setStop(false);

  output_file.close();
  return true;
}

bool TrajectoryIO::getFilePath(std::string& file_path, const std::string& file_name) const
{
  namespace fs = boost::filesystem;

  // Check that the directory exists, if not, create it
  fs::path path;
  path = fs::path(config_->package_path_ + "/trajectories");

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

bool TrajectoryIO::streamToAffine3d(Eigen::Affine3d& pose, const std::string& line,
                                    const std::string& separator)
{
  std::stringstream line_stream(line);
  std::string cell;
  std::vector<double> values;
  values.resize(16);

  // For each item/column
  for (std::size_t i = 0; i < values.size(); ++i)
  {
    // Get a variable
    if (!std::getline(line_stream, cell, ','))  // separator.c_str())) TODO
    {
      ROS_ERROR_STREAM_NAMED("trajectory_io", "Missing variable " << i << " on cell '" << cell
                                                                  << "' line '" << line << "'");
      return false;
    }

    values[i] = atof(cell.c_str());
  }

  Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> matrix(values.data(), 4, 4);

  // if (false)
  // {
  //   std::cout << matrix << std::endl;
  //   std::cout << "-------------------------------------------------------" << std::endl;
  // }

  // Eigen::Affine3d dummy;
  pose.matrix() = matrix;

  return true;
}

}  // end namespace
