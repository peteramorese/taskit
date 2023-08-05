#pragma once

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>

// Msg types
#include "taskit/MovementProperties.h"

namespace taskit {

/// Helper functions for gathering movement properties
/// @param mv_props Movement properties to edit in place
/// @param execution_success Success of the execution
/// @param start_time Time stamp marking the start of the action primitive. The execution time is the difference btw the start time and the time this function is called
/// @param path_length Length of the robot trajectory
/// @param max_velocity Maximum velocity encountered along the robot trajectory
void makeMovementProperties(taskit::MovementProperties& mv_props, bool execution_success, ros::Time start_time, double path_length = 0.0, double max_velocity = 0.0) {
    mv_props.execution_success = execution_success;
    mv_props.execution_time = (ros::Time::now() - start_time).toSec();
    mv_props.path_length = path_length;
    mv_props.max_velocity = max_velocity;
}

/// Helper functions for gathering movement properties
/// @param mv_props Movement properties to edit in place
/// @param execution_success Success of the execution
/// @param start_time Time stamp marking the start of the action primitive. The execution time is the difference btw the start time and the time this function is called
/// @param move_group 
/// @param motion_plan Motion plan for the manipulator
void makeMovementProperties(taskit::MovementProperties& mv_props, bool execution_success, ros::Time start_time, const robot_trajectory::RobotTrajectory& r_traj) {
    mv_props.path_length = robot_trajectory::path_length(r_traj);
    makeMovementProperties(mv_props, execution_success, start_time, robot_trajectory::path_length(r_traj), 0.0);
}

/// Helper functions for gathering movement properties
/// @param mv_props Movement properties to edit in place
/// @param execution_success Success of the execution
/// @param start_time Time stamp marking the start of the action primitive. The execution time is the difference btw the start time and the time this function is called
/// @param move_group 
/// @param motion_plan Motion plan for the manipulator
void makeMovementProperties(taskit::MovementProperties& mv_props, bool execution_success, ros::Time start_time, const moveit::planning_interface::MoveGroupInterface& move_group, const moveit::planning_interface::MoveGroupInterface::Plan& motion_plan) {
    robot_trajectory::RobotTrajectory r_traj(move_group.getRobotModel(), "panda_arm");
    r_traj.setRobotTrajectoryMsg(*move_group.getCurrentState(), motion_plan.trajectory_);
    makeMovementProperties(mv_props, execution_success, start_time, r_traj);
}

/// Appends a movement to an existing movement properties message
/// @param mv_props Movement properties to edit in place
/// @param execution_success Success of the execution
/// @param start_time Time stamp marking the start of the action primitive. The execution time is the difference btw the start time and the time this function is called
/// @param move_group 
/// @param motion_plan Motion plan for the manipulator
void appendMovementProperties(taskit::MovementProperties& mv_props, const taskit::MovementProperties& mv_props_append) {
    mv_props.execution_success = mv_props.execution_success && mv_props_append.execution_success;
    mv_props.execution_time += mv_props_append.execution_time;
    mv_props.path_length += mv_props_append.path_length;
    if (mv_props.max_velocity < mv_props_append.max_velocity) 
        mv_props.max_velocity = mv_props_append.max_velocity;
}

}