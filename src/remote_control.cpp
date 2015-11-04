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
   Desc:   Contains all hooks for remote control
*/

#include <moveit_boilerplate/remote_control.h>
#include <dashboard_msgs/DashboardControl.h>
#include <moveit/macros/console_colors.h>

// Command line arguments
#include <gflags/gflags.h>

namespace moveit_boilerplate
{
DEFINE_bool(auto_step, false, "Automatically go through each step");
DEFINE_bool(full_auto, false, "Automatically run ignoring all errors");

/**
 * \brief Constructor
 */
RemoteControl::RemoteControl(ros::NodeHandle nh)
  : nh_(nh)
  , is_waiting_(false)
  , next_step_ready_(false)
  , autonomous_(FLAGS_auto_step)
  , full_autonomous_(FLAGS_full_auto)
  , stop_(false)
{
  // Warnings
  if (autonomous_)
    ROS_WARN_STREAM_NAMED("remote_control", "In autonomous mode - will only stop at breakpoints");
  if (full_autonomous_)
    ROS_WARN_STREAM_NAMED("remote_control", "In FULL autonomous mode - will ignore breakpoints");

  // Subscribe to remote control topic
  std::size_t queue_size = 10;
  remote_control_ = nh_.subscribe("/picknik_main/remote_control", queue_size,
                                  &RemoteControl::remoteCallback, this);
  remote_joy_ = nh_.subscribe("/joy", queue_size, &RemoteControl::joyCallback, this);

  throttle_time_ = ros::Time::now();

  ROS_INFO_STREAM_NAMED("remote_control", "RemoteControl Ready.");
}

void RemoteControl::remoteCallback(const dashboard_msgs::DashboardControl::ConstPtr& msg)
{
  if (msg->next_step)
    setReadyForNextStep();
  else if (msg->auto_step)
    setAutonomous();
  else if (msg->full_auto)
    setFullAutonomous();
  else if (msg->stop)
    setStop();
}

void RemoteControl::joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  // Table of index number of /joy.buttons: ------------------------------------
  // 0 - A
  if (msg->buttons[0])
    setReadyForNextStep();
  // 1 - B
  // 2 - X
  // 3 - Y
  if (msg->buttons[3])
    setStop();
  // 4 - LB
  // 5 - RB
  // 6 - back
  // if (msg->buttons[6])
  // parent_->testGoHome();
  // 7 - start
  // 8 - power
  if (msg->buttons[8])
    setFullAutonomous();
  // 9 - Button stick left - does not exist on Logitech Wireless Gamepad
  // 10 - Button stick right - does not exist on Logitech Wireless Gamepad

  // Table of index number of /joy.axis: ------------------------------------

  // 0 - Left/Right Axis stick left
  // 1 - Up/Down Axis stick left
  // 2 - Left/Right Axis stick right
  // 3 - Up/Down Axis stick right
  // 4 - RT
  // 4 - LT
  // 6 - cross key left/right
  // 7 - cross key up/down
}

bool RemoteControl::setReadyForNextStep()
{
  stop_ = false;

  if (is_waiting_)
  {
    next_step_ready_ = true;
  }
  return true;
}

void RemoteControl::setAutonomous(bool autonomous)
{
  // TODO: disable this feature for final competition
  autonomous_ = autonomous;
  stop_ = false;
}

void RemoteControl::setFullAutonomous(bool autonomous)
{
  // TODO: disable this feature for final competition
  full_autonomous_ = autonomous;
  autonomous_ = autonomous;
  stop_ = false;
}

void RemoteControl::setStop(bool stop)
{
  stop_ = stop;
  if (stop)
  {
    autonomous_ = false;
    full_autonomous_ = false;
  }
}

bool RemoteControl::getStop() { return stop_; }
bool RemoteControl::getAutonomous() { return autonomous_; }
bool RemoteControl::getFullAutonomous() { return full_autonomous_; }
bool RemoteControl::waitForNextStep(const std::string& caption)
{
  // Check if we really need to wait
  if (!(!next_step_ready_ && !autonomous_ && ros::ok()))
    return true;

  // Show message
  std::cout << std::endl
            << std::endl;
  std::cout << MOVEIT_CONSOLE_COLOR_CYAN << "Waiting to " << caption << MOVEIT_CONSOLE_COLOR_RESET
            << std::endl;

  is_waiting_ = true;
  // Wait until next step is ready
  while (!next_step_ready_ && !autonomous_ && ros::ok())
  {
    ros::Duration(0.25).sleep();
    ros::spinOnce();
  }
  if (!ros::ok())
    return false;
  next_step_ready_ = false;
  is_waiting_ = false;
  return true;
}

bool RemoteControl::waitForNextFullStep(const std::string& caption)
{
  // Check if we really need to wait
  if (!(!next_step_ready_ && !full_autonomous_ && ros::ok()))
    return true;

  // Show message
  std::cout << MOVEIT_CONSOLE_COLOR_CYAN << "Waiting to " << caption << MOVEIT_CONSOLE_COLOR_RESET
            << std::endl;

  is_waiting_ = true;
  // Wait until next step is ready
  while (!next_step_ready_ && !full_autonomous_ && ros::ok())
  {
    ros::Duration(0.25).sleep();
    ros::spinOnce();
  }
  if (!ros::ok())
    return false;
  next_step_ready_ = false;
  is_waiting_ = false;
  return true;
}

void RemoteControl::initializeInteractiveMarkers(const geometry_msgs::Pose& pose)
{
  // Server
  imarker_server_.reset(new interactive_markers::InteractiveMarkerServer("basic_controls", "", false));

  // Menu
  menu_handler_.insert("Reset", boost::bind(&RemoteControl::processFeedback, this, _1));
  interactive_markers::MenuHandler::EntryHandle sub_menu_handle = menu_handler_.insert("Playback");
  menu_handler_.insert(sub_menu_handle, "Add Pose",
                       boost::bind(&RemoteControl::processFeedback, this, _1));
  menu_handler_.insert(sub_menu_handle, "Play Trajectory",
                       boost::bind(&RemoteControl::processFeedback, this, _1));
  menu_handler_.insert(sub_menu_handle, "Stop Trajectory",
                       boost::bind(&RemoteControl::processFeedback, this, _1));
  menu_handler_.insert(sub_menu_handle, "Clear Trajectory",
                       boost::bind(&RemoteControl::processFeedback, this, _1));

  // marker
  make6DofMarker(pose);

  imarker_server_->applyChanges();
}

void RemoteControl::make6DofMarker(const geometry_msgs::Pose& pose)
{
  using namespace visualization_msgs;

  int_marker_.header.frame_id = "world";
  int_marker_.pose = pose;
  int_marker_.scale = 0.15;

  int_marker_.name = "6dof_teleoperation";
  int_marker_.description = "MoveIt! Teleoperation";

  // insert a box
  makeBoxControl(int_marker_);
  int_marker_.controls[0].interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;

  InteractiveMarkerControl control;
  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker_.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker_.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker_.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker_.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_y";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker_.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker_.controls.push_back(control);

  imarker_server_->insert(int_marker_);
  imarker_server_->setCallback(int_marker_.name,
                               boost::bind(&RemoteControl::processFeedback, this, _1));
  menu_handler_.apply(*imarker_server_, int_marker_.name);
}

visualization_msgs::InteractiveMarkerControl&
RemoteControl::makeBoxControl(visualization_msgs::InteractiveMarker& msg)
{
  using namespace visualization_msgs;

  InteractiveMarkerControl control;
  control.always_visible = true;

  Marker marker;
  marker.type = Marker::CUBE;
  marker.scale.x = msg.scale * 0.1;
  marker.scale.y = msg.scale * 0.1;
  marker.scale.z = msg.scale * 0.75;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  control.markers.push_back(marker);
  msg.controls.push_back(control);

  return msg.controls.back();
}

void RemoteControl::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  using namespace visualization_msgs;

  // Only allow one feedback to be processed at a time
  {
    boost::unique_lock<boost::mutex> scoped_lock(interactive_mutex_);
    if (!teleoperation_ready_)
    {
      return;
    }
    teleoperation_ready_ = false;
  }

  // Redirect to base class
  imarker_callback_(feedback);

  // Allow next feedback to be processed
  {
    boost::unique_lock<boost::mutex> scoped_lock(interactive_mutex_);
    teleoperation_ready_ = true;
  }
}

void RemoteControl::updateMarkerPose(const geometry_msgs::Pose& pose)
{
  // std::cout << "UPDATE imarker pose \n" << pose << std::endl;
  // std::cout << "name: " << int_marker_.name << std::endl;
  imarker_server_->setPose(int_marker_.name, pose);
  imarker_server_->applyChanges();
}

}  // end namespace
