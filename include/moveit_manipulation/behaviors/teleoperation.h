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
   Desc:   Realtime teleoperation capabilities using MoveIt!

   Developer Notes:

   *Threads*

    - Interactive marker subscriber callback thread from Rviz
        Desc: Writes desired pose to memory
        Speed: ~30.3 hz

    - IK Cartesian planning thread
        Desc:  Takes latest desired pose and computes a cartesian path using iterative IK solving
        Speed: As fast as possible, checking for a new pose 100x the speed of the imarker callback
               or every 10 microseconds (.00001 seconds) (100000 hz)
               KDL IK Solver speed ~3716 hz (0.000269 s)

    - Visualization thread
        Desc:  Publishes latest valid RobotState to Rviz for user feedback

    - Command thread
        Desc:  Sends latest valid RobotState to hardware for execution
        Speed: Command robot speed 200 hz
*/

#ifndef MOVEIT_MANIPULATON__TELEOPERATION_
#define MOVEIT_MANIPULATON__TELEOPERATION_

// Teleoperation
#include <moveit_manipulation/moveit_boilerplate.h>
#include <mutex>
#include <boost/thread/locks.hpp>
#include <thread>

namespace moveit_manipulation
{
class Teleoperation : public moveit_manipulation::MoveItBoilerplate
{
public:
  /** \brief Constructor */
  Teleoperation();

  /** \brief Desctructor */
  virtual ~Teleoperation();

  /** \brief In separate thread from imarker & Ik solver, send joint commands */
  void commandJointsThread();
  void commandJointsThreadHelper();

  /**
   * \brief Compute a cartesian path along waypoints
   * \return true on success
   */
  bool computeCartesianWaypointPath(const moveit::core::RobotStatePtr start_state,
                                    const EigenSTL::vector_Affine3d& waypoints,
                                    std::vector<moveit::core::RobotStatePtr>& cartesian_traj);

  /**
   * \brief Convert and parameterize a trajectory with velocity information
   * \param vel_scaling_factor - the percent of max speed all joints should be allowed to
   * utilize
   * \return true on success
   */
  bool convertRobotStatesToTrajectory(const std::vector<moveit::core::RobotStatePtr>& robot_state_traj,
                                      moveit_msgs::RobotTrajectory& trajectory_msg,
                                      const double& vel_scaling_factor, bool use_interpolation = false);

  /** \brief Quickly response to pose requests. Uses IK on dev computer, not embedded */
  void solveIKThread();
  void solveIKThreadHelper();

  /** \brief Publish the robot state to Rviz in a separate thread */
  void visualizationThread(const ros::TimerEvent& e);

  /** \brief Helper to get the current robot state's ee pose */
  geometry_msgs::Pose chooseNewIMarkerPose();

  /**
   * \brief Callback from interactive markers
   * \param pose - pose recived from marker
   * \param mode - 1 is regular - recieve new pose,
   *               2 is reset imarker
   */
  void processIMarkerPose(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

  /** \brief Create smooth path from start to goal poses */
  //void planCartesianPath();

  /** \brief Helper transform function */
  Eigen::Affine3d offsetEEPose(const geometry_msgs::Pose& pose) const;

private:
  // Desired planning group to work with
  JointModelGroup* arm_jmg_;

  // Flag to determine if new command needs to be sent to servos
  bool has_pose_to_ik_solve_ = false;
  bool has_state_to_command_ = false;
  bool has_state_to_visualize_ = false;

  // Debug values
  bool debug_ik_rate_ = false;
  bool debug_command_rate_ = false;

  // Teleoperation -------------------------------------

  // Hook for RemoteControl class
  InteractiveMarkerCallback callback_;

  // Tool offset
  Eigen::Affine3d ee_offset_;

  // Pose of marker
  Eigen::Affine3d desired_ee_pose_;
  boost::shared_mutex desired_ee_pose_mutex_;

  Eigen::Affine3d start_ee_pose_;
  Eigen::Affine3d goal_ee_pose_;
  moveit::core::RobotStatePtr start_planning_state_;

  // Inverse Kinematics -------------------------------
  std::thread ik_thread_;

  // Maintain robot state at interactive marker (not the current robot state)
  //moveit::core::RobotStatePtr ik_teleop_state_;
  const moveit::core::LinkModel* ik_tip_link_;
  EigenSTL::vector_Affine3d cartesian_desired_waypoints_;
  // Settings
  double ik_consistency_limit_;
  double ik_timeout_;
  double ik_attempts_;
  double ik_cartesian_max_step_;  // Resolution of trajectory, the maximum distance in Cartesian
                                  // space between consecutive points on the resulting path
  double ik_cartesian_jump_threshold_;  // Threshold for preventing consequtive joint values from
                                        // 'jumping' by a large amount in joint space

  std::vector<double> ik_consistency_limits_vector_;

  // Stats
  std::size_t total_ik_attempts_ = 0;
  ros::Duration total_ik_duration_;

  // Joint Command -----------------------------------
  std::thread command_joints_thread_;
  moveit_msgs::RobotTrajectory trajectory_msg_;
  moveit_msgs::RobotTrajectory trajectory_msg_copy_;
  boost::shared_mutex trajectory_msg_mutex_;

  ros::Time trajectory_msg_timestamp_;  // when the last trajectory message was sent
  ros::Duration execution_delay_;       // Tuneable parameter for how long it takes to execute a
                                        // trajectory onto hardware

  // Stats
  std::size_t total_commands_ = 0;
  ros::Duration total_command_duration_;

  // Visualization -------------------------------------
  double visualization_rate_;  // hz
  ros::Timer non_realtime_loop_;

  // State to visualize in Rviz, also the current goal state
  moveit::core::RobotStatePtr visualize_goal_state_;

};  // end class

}  // end namespace

#endif
