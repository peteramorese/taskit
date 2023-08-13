#pragma once

#include <cmath>

#include <tf2_eigen/tf2_eigen.h>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>

// Msg types
#include "taskit/MovementProperties.h"

// TaskIt
#include "Config.h"

namespace taskit {

/// Helper functions for gathering movement properties
/// @param mv_props Movement properties to edit in place
/// @param execution_success Success of the execution
/// @param start_time Time stamp marking the start of the action primitive. The execution time is the difference btw the start time and the time this function is called
/// @param path_length Length of the robot trajectory
/// @param max_velocity Maximum speed encountered along the robot trajectory
/// @param max_acceleration Maximum acceleration magnitude encountered along the robot trajectory
/// @param max_effort Maximum effort magnitude encountered along the robot trajectory
void makeMovementProperties(
        taskit::MovementProperties& mv_props, 
        bool execution_success, 
        ros::Time start_time, 
        double path_length = 0.0, 
        double max_velocity = 0.0, 
        double max_acceleration = 0.0, 
        double max_effort = 0.0, 
        const std::vector<geometry_msgs::Pose>& eef_trajectory = std::vector<geometry_msgs::Pose>{},
        const std::vector<double>& waypoint_durations = std::vector<double>{}) {
    mv_props.execution_success = execution_success;
    mv_props.execution_time = (ros::Time::now() - start_time).toSec();
    mv_props.path_length = path_length;
    mv_props.max_velocity = max_velocity;
    mv_props.max_acceleration = max_acceleration;
    mv_props.max_effort = max_effort;
    mv_props.eef_trajectory = eef_trajectory;
    mv_props.waypoint_durations = waypoint_durations;
    ROS_ASSERT(eef_trajectory.size() == waypoint_durations.size());
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

    // Waypoint durations
    const std::deque<double>& waypoint_durations_deque = r_traj.getWayPointDurations();
    std::vector<double> waypoint_durations;
    waypoint_durations.reserve(waypoint_durations_deque.size());
    for (auto duration : waypoint_durations_deque) {
        waypoint_durations.push_back(duration);
    }

    // Sequence of eef poses for each waypoint in the trajectory
    std::vector<geometry_msgs::Pose> eef_trajectory;
    eef_trajectory.reserve(r_traj.getWayPointCount());

    for (std::size_t i = 0; i < r_traj.getWayPointCount(); ++i) {
        const moveit::core::RobotState& state = r_traj.getWayPoint(i);

        // Determine max velocity for any joint and state
        if (state.hasVelocities()) {
            const double* velocities = state.getVariableVelocities();
            for (std::size_t j = 0; j < state.getVariableCount(); ++j) {
                double velocity = std::abs(velocities[j]);
                if (velocity > max_v) {
                    max_v = velocity;
                }
            }
        }

        // Determine max velocity for any joint and state
        if (state.hasAccelerations()) {
            const double* accelerations = state.getVariableAccelerations();
            for (std::size_t j = 0; j < state.getVariableCount(); ++j) {
                double acceleration = std::abs(accelerations[j]);
                if (acceleration > max_a) {
                    max_a = acceleration;
                }
            }
        }

        // Determine max effort for any joint and state
        if (state.hasEffort()) {
            const double* efforts = state.getVariableEffort();
            for (std::size_t j = 0; j < state.getVariableCount(); ++j) {
                double effort = std::abs(efforts[j]);
                if (effort > max_e) {
                    max_e = effort;
                }
            }
        }

        // Get the end effector pose for the current state
        const Eigen::Affine3d& transform = state.getFrameTransform(TASKIT_EEF_LINK_ID);
        eef_trajectory.emplace_back(tf2::toMsg(transform));
    }
    makeMovementProperties(mv_props, execution_success, start_time, robot_trajectory::path_length(r_traj), max_v, max_a, max_e, eef_trajectory, waypoint_durations);
}

/// Helper functions for gathering movement properties
/// @param mv_props Movement properties to edit in place
/// @param execution_success Success of the execution
/// @param start_time Time stamp marking the start of the action primitive. The execution time is the difference btw the start time and the time this function is called
/// @param move_group 
/// @param motion_plan Motion plan for the manipulator
void makeMovementProperties(taskit::MovementProperties& mv_props, bool execution_success, ros::Time start_time, const moveit::planning_interface::MoveGroupInterface& move_group, const moveit::planning_interface::MoveGroupInterface::Plan& motion_plan) {
    robot_trajectory::RobotTrajectory r_traj(move_group.getRobotModel(), TASKIT_PLANNING_GROUP_ID);
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