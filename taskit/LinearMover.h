#pragma once

#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include "Config.h"
#include "MovementAnalysis.h"

namespace TaskIt {
namespace ActionPrimitives {

class LinearMover {
    public:
        virtual bool move(MovementAnalysis& mv_analysis, moveit::planning_interface::MoveGroupInterface& move_group, const geometry_msgs::Point& dst_point, const std::shared_ptr<Visualizer>& vis = nullptr) = 0;
};

class CartesianMover : public LinearMover {
    public:
        CartesianMover()
            : m_eef_step(ManipulatorProperties::getLinearEEFStepSize(TASKIT_PLANNING_GROUP_ID))
            , m_jump_thresh(ManipulatorProperties::getLinearJumpThreshold(TASKIT_PLANNING_GROUP_ID))
            , m_max_acceleration_scale(ManipulatorProperties::getMaxAccelerationScale(TASKIT_PLANNING_GROUP_ID))
            , m_max_velocity_scale(ManipulatorProperties::getMaxVelocityScale(TASKIT_PLANNING_GROUP_ID))
        {}

        virtual bool move(MovementAnalysis& mv_analysis, moveit::planning_interface::MoveGroupInterface& move_group, const geometry_msgs::Point& dst_point, const std::shared_ptr<Visualizer>& vis = nullptr) override {

            ros::Time begin = ros::Time::now();
            const geometry_msgs::Pose& curr_pose = move_group.getCurrentPose().pose;

            uint32_t n_waypoints = ManipulatorProperties::getLinearNumWaypoints(TASKIT_PLANNING_GROUP_ID);
            ROS_ASSERT_MSG(n_waypoints > 2, "Number of linear waypoints must be greater than 2, check the arm config file");
            std::vector<geometry_msgs::Pose> waypts(n_waypoints);

            // Convert to tf2
            tf2::Vector3 dst_position, curr_position;
            tf2::fromMsg(dst_point, dst_position);
            tf2::fromMsg(curr_pose.position, curr_position);

            tf2::Vector3 diff = dst_position - curr_position;
            for (uint32_t i = 0; i < n_waypoints; ++i) {
                waypts[i] = curr_pose;
                double scale = (i > 0) ? static_cast<double>(i) / static_cast<double>(n_waypoints - 1) : ManipulatorProperties::getLinearFirstPointFraction(TASKIT_PLANNING_GROUP_ID);
                tf2::toMsg(curr_position + scale * diff, waypts[i].position);
            }

            moveit_msgs::RobotTrajectory trajectory;
            double fraction = move_group.computeCartesianPath(waypts, m_eef_step, m_jump_thresh, trajectory);

            if (fraction <= 0.0) {
                ROS_ERROR("Safe cartesian path computation failed!");
                if (!ManipulatorProperties::enforceSafeLinearMovement(TASKIT_PLANNING_GROUP_ID)) {
                    ROS_WARN("Attempting cartesian path computation without collision checking. Adjust the arm_config file if this behavior is undesired");
                    fraction = move_group.computeCartesianPath(waypts, m_eef_step, m_jump_thresh, trajectory, false);
                    if (fraction <= 0.0) {
                        ROS_ERROR("Cartesian path computation failed!");
                        mv_analysis.add(false, begin);
                        return false;
                    }
                }
            }

			robot_trajectory::RobotTrajectory robot_trajectory(move_group.getRobotModel(), TASKIT_PLANNING_GROUP_ID);
            robot_trajectory.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory);
            m_iptp.computeTimeStamps(robot_trajectory, m_max_velocity_scale, m_max_acceleration_scale); 

            moveit_msgs::RobotTrajectory robot_trajectory_msg;
            robot_trajectory.getRobotTrajectoryMsg(robot_trajectory_msg);
            move_group.setMaxVelocityScalingFactor(m_max_velocity_scale);

            // Sleep a bit to make sure the arm has no residual velocity
            ros::WallDuration(1.0).sleep();

            bool execution_success = move_group.execute(robot_trajectory_msg) == moveit::core::MoveItErrorCode::SUCCESS;

            // Add the time
            mv_analysis.toggleInfemumTime(false);
            mv_analysis.add(execution_success, begin, robot_trajectory);
            mv_analysis.resetInfemumTime();

            return execution_success;
        }

    private:
        double m_eef_step;
        double m_jump_thresh;
        double m_max_acceleration_scale;
        double m_max_velocity_scale;
        trajectory_processing::IterativeParabolicTimeParameterization m_iptp;
};

class SmoothPlanMover : public LinearMover, protected Mover {
    public:
        SmoothPlanMover() {}

        virtual bool move(MovementAnalysis& mv_analysis, moveit::planning_interface::MoveGroupInterface& move_group, const geometry_msgs::Point& dst_point, const std::shared_ptr<Visualizer>& vis = nullptr) override {
            ros::Time begin = ros::Time::now();

            setScalingFactors(move_group);

            // Set the planner to an optimal planner
            setOptimalPlannerProperties(move_group);

            move_group.setStartStateToCurrentState();
            geometry_msgs::Pose goal_pose = move_group.getCurrentPose().pose;
            goal_pose.position = dst_point;
            move_group.setPoseTarget(goal_pose);

            if (vis)
                visualizePlanGoal(goal_pose, *vis);

            moveit::planning_interface::MoveGroupInterface::Plan plan;
            bool planning_success = planWithRetries(move_group, plan);
            if (!planning_success) {
                mv_analysis.add(false, begin);
                return false;
            }

            if (vis)
                visualizeExecuteGoal(goal_pose, *vis);

            bool execution_success = move_group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;

            // Add the time
            mv_analysis.toggleInfemumTime(false);
            mv_analysis.add(execution_success, begin, move_group, plan);
            mv_analysis.resetInfemumTime();

            // Reset to the default planner
            setPlannerProperties(move_group);

            return execution_success;
        }
    
};

std::shared_ptr<LinearMover> makeLinearMover() {
	std::shared_ptr<LinearMover> linear_mover;
    const std::string& linear_mover_name = ManipulatorProperties::getLinearMover(TASKIT_PLANNING_GROUP_ID);
	if (linear_mover_name == "cartesian") {
		return std::make_shared<CartesianMover>();
	} else if (linear_mover_name == "smoothplan") {
		return std::make_shared<SmoothPlanMover>();
	}
    ROS_ASSERT_MSG(false, "Unrecognized linear mover");
    return std::shared_ptr<LinearMover>();
}

}
}