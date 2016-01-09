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

/* Author: Dave Coleman
   Desc:   Tools for testing an arm
*/

#ifndef MOVEIT_TELEOP_ARM_TESTING_H
#define MOVEIT_TELEOP_ARM_TESTING_H

// MoveItManipulation
#include <moveit_boilerplate/boilerplate.h>
#include <moveit_boilerplate/trajectory_io.h>

namespace moveit_teleop
{
class ArmTesting : public moveit_boilerplate::Boilerplate
{
public:
  /**
   * \brief Constructor
   */
  ArmTesting();

  /**
   * \brief Test the end effectors
   * \param input - description
   * \return true on success
   */
  bool testEndEffectors();

  /**
   * \brief Testing ideal attached object
   * \return true on success
   */
  bool testIdealAttachedCollisionObject();

  /**
   * \brief Simple script to move hand up and down on z axis from whereever it currently is
   * \return true on success
   */
  bool testUpAndDown();

  /**
   * \brief Move hand in and out of bin from whereever it currently is
   * \return true on success
   */
  bool testInAndOut();

  /**
   * \brief Check if current state is in collision
   * \return true on success
   */
  bool testInCollision();

  /**
   * \brief Plan to random valid motions
   * \return true on success
   */
  bool testRandomValidMotions();

  /**
   * \brief Test moving joints to extreme limits
   * \return true on success
   */
  bool testJointLimits();

  /**
   * \brief Send arm(s) to home position
   * \return true on success
   */
  bool testGoHome();

  /**
   * \brief Get cartesian path for grasping object
   * \return true on success
   */
  bool testApproachLiftRetreat();

  /**
   * \brief Get the XML of a SDF pose of joints
   * \return true on success
   */
  bool getSRDFPose();

  /**
   * \brief Test grasp generator abilities and score results
   * \return true on success
   */
  bool testGraspGenerator();

  /**
   * \brief Record trajectory
   * \return true on success
   */
  bool recordTrajectory();

  /**
   * \brief
   * \param input - description
   * \return true on success
   */
  bool moveToStartPosition(JointModelGroup* arm_jmg, bool check_validity = true);

  /**
   * \brief Publish where the robot currently is
   */
  void publishCurrentState();

  /**
   * \brief Move to a pose named in the SRDF
   * \param pose_name
   * \return true on success
   */
  bool gotoPose(const std::string& pose_name);

  /**
   * \brief Move arm in circle for calibration
   * \return true on success
   */
  bool calibrateInCircle();

  /**
   * \brief Playback waypoint path specified in a csv
   * \return true on success
   */
  bool playbackWaypointsFromFile();

  /** \brief Allow interactive markers to control robot */
  void enableTeleoperation();

private:
  // Allow loading and saving trajectories to file
  moveit_boilerplate::TrajectoryIOPtr trajectory_io_;

};  // end class

}  // end namespace

#endif // MOVEIT_TELEOP_ARM_TESTING_H
