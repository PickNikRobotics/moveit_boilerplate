/*********************************************************************
 * Software License Agreement
 *
 *  Copyright (c) 2015, Dave Coleman <dave@dav.ee>
 *  All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *********************************************************************/

/* Author: Dave Coleman <dave@dav.ee>
   Desc:   Main logic of APC challenge
*/

#ifndef MOVEIT_TELEOP__MOVEIT_TELEOP
#define MOVEIT_TELEOP__MOVEIT_TELEOP

// MoveItTeleop
#include <moveit_manipulation/moveit_boilerplate.h>

namespace moveit_manipulation
{
class MoveItTeleop : public moveit_manipulation::MoveItBoilerplate
{
public:
  /**
   * \brief Constructor
   */
  MoveItTeleop();

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
