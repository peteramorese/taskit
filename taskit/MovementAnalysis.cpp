#include "MovementAnalysis.h"

namespace TaskIt {

void MovementAnalysis::create(
        bool execution_success, 
        ros::Time start_time, 
        double path_length, 
        double max_velocity, 
        double max_acceleration, 
        double max_effort, 
        const std::vector<geometry_msgs::Pose>& eef_trajectory,
        const std::vector<double>& waypoint_durations) {
    m_mv_props.execution_success = execution_success;
    m_mv_props.execution_time = (ros::Time::now() - start_time).toSec();
    m_mv_props.path_length = path_length;
    m_mv_props.max_velocity = max_velocity;
    m_mv_props.max_acceleration = max_acceleration;
    m_mv_props.max_effort = max_effort;
    m_mv_props.eef_trajectory = eef_trajectory;
    m_mv_props.waypoint_durations = waypoint_durations;
    ROS_ASSERT(eef_trajectory.size() == waypoint_durations.size());
}

void MovementAnalysis::create(bool execution_success, ros::Time start_time, const robot_trajectory::RobotTrajectory& r_traj) {
    // Trajectory path length
    m_mv_props.path_length = robot_trajectory::path_length(r_traj);

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
    create(execution_success, start_time, robot_trajectory::path_length(r_traj), max_v, max_a, max_e, eef_trajectory, waypoint_durations);
}

void MovementAnalysis::create(bool execution_success, ros::Time start_time, const moveit::planning_interface::MoveGroupInterface& move_group, const moveit::planning_interface::MoveGroupInterface::Plan& motion_plan) {
    robot_trajectory::RobotTrajectory r_traj(move_group.getRobotModel(), TASKIT_PLANNING_GROUP_ID);
    r_traj.setRobotTrajectoryMsg(*move_group.getCurrentState(), motion_plan.trajectory_);
    create(execution_success, start_time, r_traj);
}

void MovementAnalysis::appendExisting(const MovementAnalysis& mv_analysis_append) {
    appendExisting(mv_analysis_append.m_mv_props);
}

void MovementAnalysis::appendExisting(const taskit::MovementProperties& mv_props_append) {
    // Both movements must be successful
    m_mv_props.execution_success = m_mv_props.execution_success && mv_props_append.execution_success;
    
    // If infemum time is true, use the appended execution time, otherwise sum the previous time with appended time
    m_mv_props.execution_time = (!m_infemum_time) * m_mv_props.execution_time + mv_props_append.execution_time;

    m_mv_props.path_length += mv_props_append.path_length;
    if (m_mv_props.max_velocity < mv_props_append.max_velocity) 
        m_mv_props.max_velocity = mv_props_append.max_velocity;
    if (m_mv_props.max_acceleration < mv_props_append.max_acceleration) 
        m_mv_props.max_acceleration = mv_props_append.max_acceleration;
    if (m_mv_props.max_effort < mv_props_append.max_effort) 
        m_mv_props.max_effort = mv_props_append.max_effort;
    m_mv_props.eef_trajectory.insert(m_mv_props.eef_trajectory.end(), mv_props_append.eef_trajectory.begin(), mv_props_append.eef_trajectory.end());
    m_mv_props.waypoint_durations.insert(m_mv_props.waypoint_durations.end(), mv_props_append.waypoint_durations.begin(), mv_props_append.waypoint_durations.end());
}

}
