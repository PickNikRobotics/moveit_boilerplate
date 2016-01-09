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
   Desc:   Provide a service that allows nodes to request an entire planning
           scene, useful for when they are launched after the initial
           scene is created
*/

#include <moveit_boilerplate/get_planning_scene_service.h>

namespace moveit_boilerplate
{

  GetPlanningSceneService::GetPlanningSceneService()
    : name_("get_planning_scene_service")
  {}

  void GetPlanningSceneService::initialize(ros::NodeHandle nh, const std::string &planning_scene_topic,
                  psm::PlanningSceneMonitorPtr planning_scene_monitor)
  {
    planning_scene_monitor_ = planning_scene_monitor;

    const std::string GET_PLANNING_SCENE_SERVICE_NAME = "/get_planning_scene";
    get_scene_service_ = nh.advertiseService(GET_PLANNING_SCENE_SERVICE_NAME, &GetPlanningSceneService::getPlanningSceneService, this);
  }

  bool GetPlanningSceneService::getPlanningSceneService(moveit_msgs::GetPlanningScene::Request &req,
                               moveit_msgs::GetPlanningScene::Response &res)
  {
    ROS_DEBUG_STREAM_NAMED(name_, "getPlanningSceneService called");
    if (req.components.components & moveit_msgs::PlanningSceneComponents::TRANSFORMS)
      planning_scene_monitor_->updateFrameTransforms();
    planning_scene_monitor::LockedPlanningSceneRO ps(planning_scene_monitor_);
    ps->getPlanningSceneMsg(res.scene, req.components);
    return true;
  }

}  // namespace moveit_boilerplate
