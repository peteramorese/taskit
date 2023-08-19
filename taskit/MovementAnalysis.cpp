#include "MovementAnalysis.h"

namespace TaskIt {

void MovementAnalysis::create(
        taskit::MovementProperties& mv_props, 
        bool execution_success, 
        ros::Time start_time, 
        double path_length, 
        double max_velocity, 
        double max_acceleration, 
        double max_effort, 
        const std::vector<geometry_msgs::Pose>& eef_trajectory,
        const std::vector<double>& waypoint_durations) {
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

void MovementAnalysis::create(taskit::MovementProperties& mv_props, bool execution_success, ros::Time start_time, const robot_trajectory::RobotTrajectory& r_traj) {
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
    create(mv_props, execution_success, start_time, robot_trajectory::path_length(r_traj), max_v, max_a, max_e, eef_trajectory, waypoint_durations);
}

void MovementAnalysis::create(taskit::MovementProperties& mv_props, bool execution_success, ros::Time start_time, const moveit::planning_interface::MoveGroupInterface& move_group, const moveit::planning_interface::MoveGroupInterface::Plan& motion_plan) {
    robot_trajectory::RobotTrajectory r_traj(move_group.getRobotModel(), TASKIT_PLANNING_GROUP_ID);
    r_traj.setRobotTrajectoryMsg(*move_group.getCurrentState(), motion_plan.trajectory_);
    create(mv_props, execution_success, start_time, r_traj);
}

void MovementAnalysis::appendExisting(Argument& mv_arg, const taskit::MovementProperties& mv_props_append) {
    taskit::MovementProperties& mv_props = mv_arg;

    // Both movements must be successful
    mv_props.execution_success = mv_props.execution_success && mv_props_append.execution_success;
    
    // If infemum time is true, use the appended execution time, otherwise sum the previous time with appended time
    mv_props.execution_time = (!mv_arg.infemum_time) * mv_props.execution_time + mv_props_append.execution_time;

    mv_props.path_length += mv_props_append.path_length;
    if (mv_props.max_velocity < mv_props_append.max_velocity) 
        mv_props.max_velocity = mv_props_append.max_velocity;
    if (mv_props.max_acceleration < mv_props_append.max_acceleration) 
        mv_props.max_acceleration = mv_props_append.max_acceleration;
    if (mv_props.max_effort < mv_props_append.max_effort) 
        mv_props.max_effort = mv_props_append.max_effort;
    mv_props.eef_trajectory.insert(mv_props.eef_trajectory.end(), mv_props_append.eef_trajectory.begin(), mv_props_append.eef_trajectory.end());
    mv_props.waypoint_durations.insert(mv_props.waypoint_durations.end(), mv_props_append.waypoint_durations.begin(), mv_props_append.waypoint_durations.end());
}

}
