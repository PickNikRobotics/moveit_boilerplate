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
*/

#ifndef MOVEIT_MANIPULATON__TELEOPERATION_
#define MOVEIT_MANIPULATON__TELEOPERATION_

// Teleoperation
#include <moveit_manipulation/moveit_boilerplate.h>

namespace moveit_manipulation
{
class Teleoperation : public moveit_manipulation::MoveItBoilerplate
{
public:
  /**
   * \brief Constructor
   */
  Teleoperation();

  void enableTeleoperation();

  geometry_msgs::Pose getInteractiveMarkerPose();

  void setupInteractiveMarker();

  /**
   * \brief Callback from interactive markers
   * \param pose - pose recived from marker
   * \param mode - 1 is regular - recieve new pose, 
   *               2 is reset imarker
   */
  void processMarkerPose(const geometry_msgs::Pose& pose, int mode);

private:

  // Teleop
  bool teleoperation_enabled_ = false;
  Eigen::Affine3d interactive_marker_pose_;
  InteractiveMarkerCallback callback_;
};  // end class

}  // end namespace

#endif
