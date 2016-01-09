/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Ioan Sucan, Dave Coleman */

#include <ros/ros.h>
#include <moveit/planning_scene/planning_scene.h>

namespace moveit_boilerplate
{
const std::string BOUNDS_PARAM_NAME = "start_state_max_bounds_error";
const std::string DT_PARAM_NAME = "start_state_max_dt";

class FixStateBounds
{
public:
  FixStateBounds();

  /**
   * \brief Jiggle a specified state so that it becomes within joint bounds
   * \param robot_state to be modified
   * \param jmg - the part of the robot to fix
   * \return true on success
   */
  bool fixBounds(robot_state::RobotState& robot_state, const moveit::core::JointModelGroup* jmg);

  /**
   * \brief Getter for MaxBoundsError
   */
  double getMaxBoundsError() const
  {
    return bounds_dist_;
  }
  /**
   * \brief Setter for MaxBoundsError
   */
  void setMaxBoundsError(const double& bounds_dist)
  {
    bounds_dist_ = bounds_dist;
  }

private:
  ros::NodeHandle nh_;
  double bounds_dist_;
  double max_dt_offset_;
};
}
