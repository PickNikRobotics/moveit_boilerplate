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
#include <moveit_boilerplate/behaviors/teleoperation.h>

// ROS
#include <ros/ros.h>

DEFINE_string(pose, "", "Requested robot pose");
DEFINE_int32(mode, 2, "Mode");

int main(int argc, char** argv)
{
  google::SetVersionString("Best version.");
  google::SetUsageMessage("MoveIt!-based picking framework");
  google::ParseCommandLineFlags(&argc, &argv, true);

  ros::init(argc, argv, "moveit_boilerplate");

  std::cout << std::endl
            << std::endl
            << std::endl;
  ROS_INFO_STREAM_NAMED("main", "Starting Pick Manager");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(4);
  spinner.start();

  // start timer for run length
  ros::Time begin_time = ros::Time::now();

  // Random
  srand(time(NULL));

  // Main program
  moveit_boilerplate::Teleoperation manager;

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
      manager.enableTeleoperation();
      ros::spin();
      break;
    case 2:
      ROS_INFO_STREAM_NAMED("main", "Go to home position");
      manager.testGoHome();
      break;
    case 3:
      ROS_INFO_STREAM_NAMED("main", "Insertion");
      // manager.insertion();
      break;
    case 4:
      ROS_INFO_STREAM_NAMED("main", "Touch control");
      // manager.touchControl();
      break;
    case 5:
      ROS_INFO_STREAM_NAMED("main", "Raise the roof (go up and down)");
      manager.testUpAndDown();
      break;
    case 6:
      ROS_INFO_STREAM_NAMED("main", "Plan to random valid locations");
      manager.testRandomValidMotions();
      break;
    case 7:
      ROS_INFO_STREAM_NAMED("main", "Draw spiral");
      // manager.drawSpiral();
      break;
    case 8:
      ROS_INFO_STREAM_NAMED("main", "Test end effectors mode");
      manager.testEndEffectors();
      break;
    case 9:
      ROS_INFO_STREAM_NAMED("main", "Going to pose " << FLAGS_pose);
      manager.gotoPose(FLAGS_pose);
      break;
    case 10:
      ROS_INFO_STREAM_NAMED("main", "Automated insertion test");
      // manager.automatedInsertionTest();
      break;
    case 11:
      ROS_INFO_STREAM_NAMED("main", "Going in circle for calibration");
      manager.calibrateInCircle();
      break;
    case 17:
      ROS_INFO_STREAM_NAMED("main", "Test joint limits");
      manager.testJointLimits();
      break;
    case 41:
      ROS_INFO_STREAM_NAMED("main", "Get SRDF pose");
      manager.getSRDFPose();
      break;
    case 42:
      ROS_INFO_STREAM_NAMED("main", "Check if current state is in collision");
      manager.testInCollision();
      ros::Duration(5.0).sleep();
      break;

    default:
      ROS_WARN_STREAM_NAMED("main", "Unkown mode: " << FLAGS_mode);
  }

  // Shutdown
  std::cout << std::endl
            << std::endl
            << std::endl;
  std::cout << "-------------------------------------------------------" << std::endl;
  ROS_INFO_STREAM_NAMED("main", "Shutting down.");
  std::cout << std::endl
            << std::endl
            << std::endl;

  ros::Time end_time = ros::Time::now();

  ros::Duration duration = (end_time - begin_time);

  ROS_INFO_STREAM_NAMED("main",
                        "Test duration = " << duration << " seconds (" << (duration.toSec() / 60.0) << " minutes). "
                                           << "Max time allowed = " << (15.0 * 60.0) << " seconds (15 minutes).");

  ros::shutdown();

  return 0;
}
