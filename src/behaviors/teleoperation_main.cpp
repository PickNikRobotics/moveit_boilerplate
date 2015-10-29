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
   Desc:   Main function that processes arguments
*/

#include <string>
#include <iostream>

// Command line arguments
#include <gflags/gflags.h>
#include <moveit_manipulation/behaviors/teleoperation.h>

// ROS
#include <ros/ros.h>

DEFINE_string(pose, "", "Requested robot pose");
DEFINE_int32(mode, 2, "Mode");

int main(int argc, char** argv)
{
  google::SetVersionString("Best version.");
  google::SetUsageMessage("MoveIt!-based picking framework");
  google::ParseCommandLineFlags(&argc, &argv, true);

  ros::init(argc, argv, "moveit_manipulation");

  std::cout << std::endl << std::endl << std::endl;
  ROS_INFO_STREAM_NAMED("main", "Starting Pick Manager");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(4);
  spinner.start();

  // start timer for run length
  ros::Time begin_time = ros::Time::now();

  // Random
  srand(time(NULL));

  // Main program
  moveit_manipulation::Teleoperation manager;

  std::cout << std::endl;
  std::cout << "-------------------------------------------------------" << std::endl;

  switch (FLAGS_mode)
  {
    case 0:
      ROS_INFO_STREAM_NAMED("main", "Do nothing");
      ros::spin();
      break;
    case 1:
      ROS_INFO_STREAM_NAMED("main", "Interactive marker teleoperation");
      manager.startTeleopStatePublishing();
      ros::spin();
      break;
    default:
      ROS_WARN_STREAM_NAMED("main", "Unkown mode: " << FLAGS_mode);
  }

  // Shutdown
  std::cout << std::endl << std::endl << std::endl;
  std::cout << "-------------------------------------------------------" << std::endl;
  ROS_INFO_STREAM_NAMED("main", "Shutting down.");
  std::cout << std::endl << std::endl << std::endl;

  ros::Time end_time = ros::Time::now();

  ros::Duration duration = (end_time - begin_time);

  ROS_INFO_STREAM_NAMED("main", "Test duration = " << duration << " seconds ("
                                                   << (duration.toSec() / 60.0) << " minutes). "
                                                   << "Max time allowed = " << (15.0 * 60.0)
                                                   << " seconds (15 minutes).");

  ros::shutdown();

  return 0;
}
