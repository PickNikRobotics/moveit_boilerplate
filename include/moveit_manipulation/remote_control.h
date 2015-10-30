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
   Desc:   Contains all hooks for remote control
*/

#ifndef MOVEIT_MANIPULATION__REMOTE_CONTROL
#define MOVEIT_MANIPULATION__REMOTE_CONTROL

// moveit_grasps
//#include <moveit_grasps/grasp_planner.h>

// ROS
#include <ros/ros.h>
#include <dashboard_msgs/DashboardControl.h>
#include <sensor_msgs/Joy.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

// Boost
#include <boost/thread/mutex.hpp>

namespace moveit_manipulation
{
typedef std::function<void(const geometry_msgs::Pose&, int)> InteractiveMarkerCallback;

class RemoteControl
{
public:
  /**
   * \brief Constructor
   */
  RemoteControl(ros::NodeHandle nh);

  /**
   * \brief Remote control from Rviz
   */
  void remoteCallback(const dashboard_msgs::DashboardControl::ConstPtr& msg);

  /**
   * \brief Recieves inputs from joystick
   * \param ROS message
   */
  void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);

  /**
   * \brief Step to next step
   * \return true on success
   */
  bool setReadyForNextStep();

  /**
   * \brief Enable autonomous mode
   */
  void setAutonomous(bool autonomous = true);
  void setFullAutonomous(bool autonomous = true);

  /**
   * \brief Get the autonomous mode
   * \return true if is in autonomous mode
   */
  bool getAutonomous();
  bool getFullAutonomous();

  /**
   * \brief Stop something in pipeline
   */
  void setStop(bool stop = true);

  /**
   * \brief See if we are in stop mode
   */
  bool getStop();

  /**
   * \brief Wait until user presses a button
   * \return true on success
   */
  bool waitForNextStep(const std::string& caption = "go to next step");
  bool waitForNextFullStep(const std::string& caption = "go to next full step");

  void initializeInteractiveMarkers(const geometry_msgs::Pose& pose);

  /** \brief Return true if remote control is waiting for user input */
  bool isWaiting() { return is_waiting_; }

  void setInteractiveMarkerCallback(InteractiveMarkerCallback callback)
  {
    imarker_callback_ = callback;
  }

  void updateMarkerPose(const geometry_msgs::Pose& pose);

private:
  void make6DofMarker(unsigned int interaction_mode, const geometry_msgs::Pose& pose);
  
  /** \brief Helper for geometric shape */
  visualization_msgs::InteractiveMarkerControl&
  makeBoxControl(visualization_msgs::InteractiveMarker& msg);

  /** \brief Callback from interactive marker server */
  void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

  // A shared node handle
  ros::NodeHandle nh_;

  // Remote control
  ros::Subscriber remote_control_;
  ros::Subscriber remote_joy_;

  // Remote control
  bool is_waiting_;
  bool next_step_ready_;
  bool autonomous_;
  bool full_autonomous_;
  bool stop_;

  // Interactive markers
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> imarker_server_;
  interactive_markers::MenuHandler menu_handler_;
  visualization_msgs::InteractiveMarker int_marker_;
  bool teleoperation_ready_ = true;
  InteractiveMarkerCallback imarker_callback_;  // hook to parent class

  ros::Time throttle_time_;
  boost::mutex interactive_mutex_;

};  // end class

// Create boost pointers for this class
typedef boost::shared_ptr<RemoteControl> RemoteControlPtr;
typedef boost::shared_ptr<const RemoteControl> RemoteControlConstPtr;

}  // end namespace

#endif
