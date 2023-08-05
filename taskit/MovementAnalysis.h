#pragma once

#include <cmath>

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
/// @param max_velocity Maximum speed encountered along the robot trajectory
/// @param max_acceleration Maximum acceleration magnitude encountered along the robot trajectory
/// @param max_effort Maximum effort magnitude encountered along the robot trajectory
void makeMovementProperties(taskit::MovementProperties& mv_props, bool execution_success, ros::Time start_time, double path_length = 0.0, double max_velocity = 0.0, double max_acceleration = 0.0, double max_effort = 0.0) {
    mv_props.execution_success = execution_success;
    mv_props.execution_time = (ros::Time::now() - start_time).toSec();
    mv_props.path_length = path_length;
    mv_props.max_velocity = max_velocity;
    mv_props.max_acceleration = max_acceleration;
    mv_props.max_effort = max_effort;
}

/// Helper functions for gathering movement properties
/// @param mv_props Movement properties to edit in place
/// @param execution_success Success of the execution
/// @param start_time Time stamp marking the start of the action primitive. The execution time is the difference btw the start time and the time this function is called
/// @param move_group 
/// @param motion_plan Motion plan for the manipulator
void makeMovementProperties(taskit::MovementProperties& mv_props, bool execution_success, ros::Time start_time, const robot_trajectory::RobotTrajectory& r_traj) {
    // Trajectory path length
    mv_props.path_length = robot_trajectory::path_length(r_traj);

    // Max quantities for any joint along trajectory (velocity, acceleration, effort)
    double max_v = 0.0;
    double max_a = 0.0;
    double max_e = 0.0;
    for (std::size_t i = 0; i < r_traj.getWayPointCount(); ++i) {
        const moveit::core::RobotState& state = r_traj.getWayPoint(i);
        ROS_ASSERT_MSG(state.hasVelocities() && state.hasAccelerations() && state.hasEffort(), "State is missing at least one of: velocities, accelerations, effort");
        const double* velocities = state.getVariableVelocities();
        const double* accelerations = state.getVariableAccelerations();
        const double* efforts = state.getVariableEffort();
        for (std::size_t j = 0; j < state.getVariableCount(); ++j) {
            double velocity = std::abs(velocities[j]);
            double acceleration = std::abs(accelerations[j]);
            double effort = std::abs(efforts[j]);
            if (velocity > max_v) {
                max_v = velocity;
            }
            if (acceleration > max_a) {
                max_a = acceleration;
            }
            if (effort > max_e) {
                max_e = effort;
            }
        }
    }
    makeMovementProperties(mv_props, execution_success, start_time, robot_trajectory::path_length(r_traj), max_v, max_a, max_e);
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
    if (mv_props.max_acceleration < mv_props_append.max_acceleration) 
        mv_props.max_acceleration = mv_props_append.max_acceleration;
    if (mv_props.max_effort < mv_props_append.max_effort) 
        mv_props.max_effort = mv_props_append.max_effort;
}

}