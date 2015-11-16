/* Author: Dave Coleman <dave@dav.ee>
   Desc:   Generate motions that are more complex than cartesian straight line planning
*/

#include <moveit_boilerplate/planning_interface.h>

// Conversions
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

// ROS parameter loading
#include <ros_param_shortcuts/ros_param_shortcuts.h>

namespace moveit_boilerplate
{
PlanningInterface::PlanningInterface(moveit_boilerplate::ExecutionInterfacePtr execution_interface,
                                     psm::PlanningSceneMonitorPtr planning_scene_monitor,
                                     mvt::MoveItVisualToolsPtr visual_tools, JointModelGroup* arm_jmg)
  : nh_("~")
  , execution_interface_(execution_interface)
  , planning_scene_monitor_(planning_scene_monitor)
  , visual_tools_(visual_tools)
  , arm_jmg_(arm_jmg)
{
  // Load rosparams
  const std::string parent_name = "planning_interface";  // for namespacing logging messages
  ros::NodeHandle rosparam_nh(nh_, parent_name);
  using namespace ros_param_shortcuts;
  // getDoubleParam(parent_name, rosparam_nh, "vel_scaling_factor", vel_scaling_factor_);

  // End effector parent link (arm tip for ik solving)
  ik_tip_link_ = arm_jmg_->getOnlyOneEndEffectorTip();

  // Check for kinematic solver
  if (!arm_jmg_->canSetStateFromIK(ik_tip_link_->getName()))
    ROS_ERROR_STREAM_NAMED("planning_interface", "No IK Solver loaded - make sure "
                                                 "moveit_config/kinamatics.yaml is loaded in this namespace");

  // Create initial robot state
  {
    psm::LockedPlanningSceneRO scene(planning_scene_monitor_);  // Lock planning scene
    current_state_.reset(new moveit::core::RobotState(scene->getCurrentState()));
  }  // end scoped pointer of locked planning scene

  // Set robot model
  robot_model_ = current_state_->getRobotModel();

  ROS_INFO_STREAM_NAMED("temp", "PlanningInterface Ready.");
}

PlanningInterface::~PlanningInterface() {}

bool PlanningInterface::moveToSRDFPoseNoPlan(JointModelGroup* jmg, const std::string& pose_name,
                                             double velocity_scaling_factor)
{
  // Set goal state to initial pose
  moveit::core::RobotStatePtr goal_state(new moveit::core::RobotState(*current_state_));
  if (!goal_state->setToDefaultValues(jmg, pose_name))
  {
    ROS_ERROR_STREAM_NAMED("planning_interface", "Failed to set pose '" << pose_name << "' for planning group '"
                                                                        << jmg->getName() << "'");
    return false;
  }

  if (!executeState(jmg, goal_state, velocity_scaling_factor))
  {
    ROS_ERROR_STREAM_NAMED("temp", "Unable to execute state of SRDF pose");
    return false;
  }

  return true;
}

bool PlanningInterface::executeState(JointModelGroup* jmg, const moveit::core::RobotStatePtr goal_state,
                                     double velocity_scaling_factor)
{
  // Get the start state
  getCurrentState();

  // Visualize start/goal
  // visual_start_state_->publishRobotState(current_state_, rvt::GREEN);
  // visual_goal_state_->publishRobotState(goal_state, rvt::ORANGE);

  // Check if already in new position
  if (statesEqual(*current_state_, *goal_state, jmg))
  {
    ROS_INFO_STREAM_NAMED("planning_interface", "Not executing because current state and goal state are "
                                                "close enough.");
    return true;
  }

  // Create trajectory
  std::vector<moveit::core::RobotStatePtr> robot_state_traj;
  robot_state_traj.push_back(current_state_);

  // Add goal state
  robot_state_traj.push_back(goal_state);

  // Get trajectory message
  moveit_msgs::RobotTrajectory trajectory_msg;

  // Convert trajectory to a message
  bool interpolate = true;
  if (!convertRobotStatesToTraj(robot_state_traj, trajectory_msg, jmg, velocity_scaling_factor, interpolate))
  {
    ROS_ERROR_STREAM_NAMED("planning_interface", "Failed to convert to parameterized trajectory");
    return false;
  }

  // Execute
  const bool wait_for_execution = true;
  if (!execution_interface_->executeTrajectory(trajectory_msg, jmg, wait_for_execution))
  {
    ROS_ERROR_STREAM_NAMED("planning_interface", "Failed to execute trajectory");
    return false;
  }
  return true;
}

bool PlanningInterface::convertRobotStatesToTraj(const std::vector<moveit::core::RobotStatePtr>& robot_state_traj,
                                                 moveit_msgs::RobotTrajectory& trajectory_msg, JointModelGroup* jmg,
                                                 const double& velocity_scaling_factor, bool use_interpolation)
{
  // Copy the vector of RobotStates to a RobotTrajectory
  robot_trajectory::RobotTrajectoryPtr robot_traj(new robot_trajectory::RobotTrajectory(robot_model_, jmg));

  // -----------------------------------------------------------------------------------------------
  // Convert to RobotTrajectory datatype
  for (std::size_t k = 0; k < robot_state_traj.size(); ++k)
  {
    double duration_from_previous = 1;  // this is overwritten and unimportant
    robot_traj->addSuffixWayPoint(robot_state_traj[k], duration_from_previous);
  }

  return convertRobotStatesToTraj(robot_traj, trajectory_msg, jmg, velocity_scaling_factor, use_interpolation);
}

bool PlanningInterface::convertRobotStatesToTraj(robot_trajectory::RobotTrajectoryPtr robot_traj,
                                                 moveit_msgs::RobotTrajectory& trajectory_msg, JointModelGroup* jmg,
                                                 const double& velocity_scaling_factor, bool use_interpolation)
{
  if (robot_traj->getFirstWayPoint().hasVelocities())
    ROS_DEBUG_STREAM_NAMED("planning_interface.convert", "First waypoint has velocity");

  // Interpolate any path with two few points
  if (use_interpolation)
  {
    static const std::size_t MIN_TRAJECTORY_POINTS = 20;
    if (robot_traj->getWayPointCount() < MIN_TRAJECTORY_POINTS)
    {
      ROS_INFO_STREAM_NAMED("planning_interface", "Interpolating trajectory because two few points ("
                                                      << robot_traj->getWayPointCount() << ")");

      // Interpolate between each point
      double discretization = 0.25;
      interpolate(robot_traj, discretization);
    }
  }

  bool debug = false;
  if (debug)
  {
    // Convert trajectory to a message
    robot_traj->getRobotTrajectoryMsg(trajectory_msg);
    std::cout << "Before Iterative smoother: " << trajectory_msg << std::endl;
  }

  // Perform iterative parabolic smoothing
  iterative_smoother_.computeTimeStamps(*robot_traj, velocity_scaling_factor);

  // Convert trajectory to a message
  robot_traj->getRobotTrajectoryMsg(trajectory_msg);

  // std::cout << "After Iterative smoother: " << trajectory_msg << std::endl;

  return true;
}

bool PlanningInterface::interpolate(robot_trajectory::RobotTrajectoryPtr robot_traj, const double& discretization)
{
  double dummy_dt = 1;  // dummy value until parameterization

  robot_trajectory::RobotTrajectoryPtr new_robot_traj(
      new robot_trajectory::RobotTrajectory(robot_model_, robot_traj->getGroup()));
  std::size_t original_num_waypoints = robot_traj->getWayPointCount();

  // Error check
  if (robot_traj->getWayPointCount() < 2)
  {
    ROS_ERROR_STREAM_NAMED("planning_interface", "Unable to interpolate between less than two states");
    return false;
  }

  // Debug
  // for (std::size_t i = 0; i < robot_traj->getWayPointCount(); ++i)
  // {
  //   moveit::core::robotStateToStream(robot_traj->getWayPoint(i), std::cout, false);
  // }
  // std::cout << "-------------------------------------------------------" << std::endl;

  // For each set of points (A,B) in the original trajectory
  for (std::size_t i = 0; i < robot_traj->getWayPointCount() - 1; ++i)
  {
    // Add point A to final trajectory
    new_robot_traj->addSuffixWayPoint(robot_traj->getWayPoint(i), dummy_dt);

    for (double t = discretization; t < 1; t += discretization)
    {
      // Create new state
      moveit::core::RobotStatePtr interpolated_state(new moveit::core::RobotState(robot_traj->getFirstWayPoint()));
      // Fill in new values
      robot_traj->getWayPoint(i).interpolate(robot_traj->getWayPoint(i + 1), t, *interpolated_state);
      // Add to trajectory
      new_robot_traj->addSuffixWayPoint(interpolated_state, dummy_dt);
      // std::cout << "inserting " << t << " at " << new_robot_traj->getWayPointCount() <<
      // std::endl;
    }
  }

  // Add final waypoint
  new_robot_traj->addSuffixWayPoint(robot_traj->getLastWayPoint(), dummy_dt);

  // Debug
  // for (std::size_t i = 0; i < new_robot_traj->getWayPointCount(); ++i)
  // {
  //   moveit::core::robotStateToStream(new_robot_traj->getWayPoint(i), std::cout, false);
  // }

  std::size_t modified_num_waypoints = new_robot_traj->getWayPointCount();
  ROS_DEBUG_STREAM_NAMED("planning_interface.interpolation",
                         "Interpolated trajectory from " << original_num_waypoints << " to " << modified_num_waypoints);

  // Copy back to original datastructure
  *robot_traj = *new_robot_traj;

  return true;
}

moveit::core::RobotStatePtr PlanningInterface::getCurrentState()
{
  // Get the real current state
  psm::LockedPlanningSceneRO scene(planning_scene_monitor_);  // Lock planning scene
  (*current_state_) = scene->getCurrentState();
  return current_state_;
}

bool PlanningInterface::statesEqual(const moveit::core::RobotState& s1, const moveit::core::RobotState& s2,
                                    JointModelGroup* jmg)
{
  static const double STATES_EQUAL_THRESHOLD = 0.001;

  double s1_vars[jmg->getVariableCount()];
  double s2_vars[jmg->getVariableCount()];
  s1.copyJointGroupPositions(jmg, s1_vars);
  s2.copyJointGroupPositions(jmg, s2_vars);

  for (std::size_t i = 0; i < jmg->getVariableCount(); ++i)
  {
    const moveit::core::JointModel* this_joint = jmg->getJointModel(jmg->getVariableNames()[i]);

    std::vector<const moveit::core::JointModel*>::const_iterator joint_it;
    joint_it = std::find(jmg->getActiveJointModels().begin(), jmg->getActiveJointModels().end(), this_joint);

    // Make sure joint is active
    if (joint_it != jmg->getActiveJointModels().end())
    {
      if (fabs(s1_vars[i] - s2_vars[i]) > STATES_EQUAL_THRESHOLD)
      {
        // std::cout << "    statesEqual: Variable " << jmg->getVariableNames()[i] << " beyond
        // threshold, diff: "
        // << fabs(s1_vars[i] - s2_vars[i]) << std::endl;
        return false;
      }
    }
  }

  return true;
}

}  // end namespace

namespace
{
bool isStateValid(const planning_scene::PlanningScene* planning_scene, bool verbose, bool only_check_self_collision,
                  mvt::MoveItVisualToolsPtr visual_tools, moveit::core::RobotState* robot_state, JointModelGroup* group,
                  const double* ik_solution, std::size_t collision_check_skip_every)
{
  // Decide if we can skip this check
  static int check_count = 0;  // TODO is static ok?
  if (++check_count % collision_check_skip_every != 0)
    return true;

  // Apply IK solution to robot state
  robot_state->setJointGroupPositions(group, ik_solution);
  robot_state->update();

  if (!planning_scene)
  {
    ROS_ERROR_STREAM_NAMED("planning_interface", "No planning scene provided");
    return false;
  }
  if (only_check_self_collision)
  {
    // No easy API exists for only checking self-collision, so we do it here.
    // TODO: move this into planning_scene.cpp
    collision_detection::CollisionRequest req;
    req.verbose = false;
    req.group_name = group->getName();
    collision_detection::CollisionResult res;
    planning_scene->checkSelfCollision(req, res, *robot_state);
    if (!res.collision)
      return true;  // not in collision
  }
  else if (!planning_scene->isStateColliding(*robot_state, group->getName()))
    return true;  // not in collision

  // Display more info about the collision
  if (verbose)
  {
    visual_tools->publishRobotState(*robot_state, rvt::RED);
    planning_scene->isStateColliding(*robot_state, group->getName(), true);
    visual_tools->publishContactPoints(*robot_state, planning_scene);
    ros::Duration(0.4).sleep();
  }
  return false;
}

}  // end annonymous namespace
