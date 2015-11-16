/* Author: Dave Coleman <dave@dav.ee>
   Desc:   Generate motions that are more complex than cartesian straight line planning
*/

#ifndef MOVEIT_BOILERPLATE__PLANNING_INTERFACE_H
#define MOVEIT_BOILERPLATE__PLANNING_INTERFACE_H

// moveit_boilerplate
#include <moveit_boilerplate/namespaces.h>
#include <moveit_boilerplate/execution_interface.h>

// ROS
#include <ros/ros.h>

// MoveIt!
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/planning_scene/planning_scene.h>

// Visual tools
#include <moveit_visual_tools/moveit_visual_tools.h>

namespace moveit_boilerplate
{
MOVEIT_CLASS_FORWARD(PlanningInterface);

class PlanningInterface
{
public:
  /** \brief Constructor */
  PlanningInterface(moveit_boilerplate::ExecutionInterfacePtr execution_interface,
                    psm::PlanningSceneMonitorPtr planning_scene_monitor, mvt::MoveItVisualToolsPtr visual_tools,
                    JointModelGroup* arm_jmg);

  /** \brief Destructor */
  virtual ~PlanningInterface();

  /**
   * \brief Move to any pose as defined in the SRDF
   * \param arm_jmg - the kinematic chain of joints that should be controlled (a planning group)
   * \param velocity_scaling_factor - the percent of max speed all joints should be allowed to
   * utilize
   * \return true on success
   */
  bool moveToSRDFPoseNoPlan(JointModelGroup* jmg, const std::string& pose_name, double velocity_scaling_factor);

  /**
   * \brief Send a single state to the controllers for execution
   * \param arm_jmg - the kinematic chain of joints that should be controlled (a planning group)
   * \param goal_state
   * \param velocity_scaling_factor - the percent of max speed all joints should be allowed to utilize
   * \return true on success
   */
  bool executeState(JointModelGroup* jmg, const moveit::core::RobotStatePtr goal_state, double velocity_scaling_factor);

  /**
   * \brief Convert and parameterize a trajectory with velocity information
   * \param robot_state_traj - input
   * \param trajectory_msg - output
   * \param velocity_scaling_factor - the percent of max speed all joints should be allowed to utilize
   * \param arm_jmg - the kinematic chain of joints that should be controlled (a planning group)
   * \param use_interpolation (recommended) whether to add more points to trajectory to make it more smooth
   * \return true on success
   */
  bool convertRobotStatesToTraj(const std::vector<robot_state::RobotStatePtr>& robot_state_traj,
                                moveit_msgs::RobotTrajectory& trajectory_msg, JointModelGroup* arm_jmg,
                                const double& velocity_scaling_factor, bool interpolate = true);
  bool convertRobotStatesToTraj(robot_trajectory::RobotTrajectoryPtr robot_traj,
                                moveit_msgs::RobotTrajectory& trajectory_msg, JointModelGroup* jmg,
                                const double& velocity_scaling_factor, bool use_interpolation);

private:
  /**
   * \brief Helper function for determining if robot is already in desired state
   * \param robotstate to compare to
   * \param robotstate to compare to
   * \param arm_jmg - only compare joints in this joint model group
   * \return true if states are close enough in similarity
   */
  bool statesEqual(const moveit::core::RobotState& s1, const moveit::core::RobotState& s2, JointModelGroup* arm_jmg);

  /**
   * \brief Interpolate
   * \return true on success
   */
  bool interpolate(robot_trajectory::RobotTrajectoryPtr robot_trajectory, const double& discretization);

  /**
   * \brief Use the planning scene to get the robot's current state
   */
  moveit::core::RobotStatePtr getCurrentState();

  // --------------------------------------------------------

  // A shared node handle
  ros::NodeHandle nh_;

  // For executing joint and cartesian trajectories
  moveit_boilerplate::ExecutionInterfacePtr execution_interface_;

  // Core MoveIt components
  psm::PlanningSceneMonitorPtr planning_scene_monitor_;
  robot_model::RobotModelConstPtr robot_model_;

  // Visualization
  mvt::MoveItVisualToolsPtr visual_tools_;
  // VisualizeStateCallback visualize_state_callback_;
  // VisualizeStateCallback visualize_start_state_callback_;

  // Desired planning group to work with
  JointModelGroup* arm_jmg_;

  const moveit::core::LinkModel* ik_tip_link_;

  // Joint Command -----------------------------------
  moveit_msgs::RobotTrajectory trajectory_msg_;

  // Tool for parameterizing trajectories with velocities and accelerations
  trajectory_processing::IterativeParabolicTimeParameterization iterative_smoother_;

  // Allocated memory for robot state
  moveit::core::RobotStatePtr current_state_;
};  // end class

}  // end namespace

namespace
{
/** \brief Collision checking handle for IK solvers */
bool isStateValid(const planning_scene::PlanningScene* planning_scene, bool verbose, bool only_check_self_collision,
                  mvt::MoveItVisualToolsPtr visual_tools_, robot_state::RobotState* state,
                  const robot_state::JointModelGroup* group, const double* ik_solution,
                  std::size_t collision_check_skip_every);
}

#endif  // MOVEIT_BOILERPLATE__PLANNING_INTERFACE_
