#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include "shape_completion/parametrize_cartesian_path.h"

bool parametrize(shape_completion::parametrize_cartesian_path::Request &req,
                 shape_completion::parametrize_cartesian_path::Response &res)
{

  static const std::string PLANNING_GROUP = "arm";

  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  moveit_msgs::RobotTrajectory trajectory_msg = req.trajectory_in;

  robot_trajectory::RobotTrajectory trajectory(move_group.getRobotModel(), PLANNING_GROUP);

  robot_state::RobotState start_state(*move_group.getCurrentState());

  trajectory.setRobotTrajectoryMsg(start_state, trajectory_msg);

  double path_tolerance = 0.002;
  double resample_dt = 0.001;
  double min_angle_change = 0.001;

  trajectory_processing::TimeOptimalTrajectoryGeneration time_param(path_tolerance, resample_dt, min_angle_change);

  time_param.computeTimeStamps(trajectory, req.velocity_scaling);

  moveit_msgs::RobotTrajectory trajectory_out;
  trajectory.getRobotTrajectoryMsg(trajectory_out);

  res.trajectory_out = trajectory_out;

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "parametrize_cartesian_path_server");

  ros::NodeHandle node_handle;
  ros::ServiceServer service = node_handle.advertiseService("parametrize_cartesian_path", parametrize);
  ROS_INFO("Ready to parametrize");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}