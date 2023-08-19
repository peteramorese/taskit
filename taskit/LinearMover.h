#pragma once

#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include "Config.h"
#include "MovementAnalysis.h"

namespace TaskIt {
namespace ActionPrimitives {

class LinearMover {
    public:
        virtual bool move(MovementAnalysis::Argument& mv_props, moveit::planning_interface::MoveGroupInterface& move_group, const geometry_msgs::Point& dst_point) = 0;
};

class CartesianMover : public LinearMover {
    public:
        CartesianMover()
            : m_eef_step(ManipulatorProperties::getLinearEEFStepSize(TASKIT_PLANNING_GROUP_ID))
            , m_jump_thresh(ManipulatorProperties::getLinearJumpThreshold(TASKIT_PLANNING_GROUP_ID))
            , m_max_acceleration_scale(ManipulatorProperties::getMaxAccelerationScale(TASKIT_PLANNING_GROUP_ID))
            , m_max_velocity_scale(ManipulatorProperties::getMaxVelocityScale(TASKIT_PLANNING_GROUP_ID))
        {}

        virtual bool move(MovementAnalysis::Argument& mv_props, moveit::planning_interface::MoveGroupInterface& move_group, const geometry_msgs::Point& dst_point) override {

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
                        MovementAnalysis::make(mv_props, false, begin);
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
            MovementAnalysis::make(mv_props, execution_success, begin, robot_trajectory);
            return execution_success;
        }

    private:
        double m_eef_step;
        double m_jump_thresh;
        double m_max_acceleration_scale;
        double m_max_velocity_scale;
        trajectory_processing::IterativeParabolicTimeParameterization m_iptp;
};

class SmoothPlanMover : public LinearMover {
    public:
        SmoothPlanMover() 
            : m_optimal_planner_id(ManipulatorProperties::getOptimalPlannerID(TASKIT_PLANNING_GROUP_ID))
            , m_planning_time(10.0)
        {}

        void setPlanningTime(double planning_time) {m_planning_time = planning_time;}

        virtual bool move(MovementAnalysis::Argument& mv_props, moveit::planning_interface::MoveGroupInterface& move_group, const geometry_msgs::Point& dst_point) override {
            ros::Time begin = ros::Time::now();

            // Set the planner to an optimal planner
            move_group.setPlannerId(m_optimal_planner_id);
            move_group.setPlanningTime(m_planning_time);

            move_group.setStartStateToCurrentState();
            geometry_msgs::Pose goal_pose = move_group.getCurrentPose().pose;
            goal_pose.position = dst_point;
            move_group.setPoseTarget(goal_pose);

            bool planning_success = false;
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            for (uint32_t trial = 0; trial < m_n_retries; ++trial) {
                planning_success = move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;
                if (planning_success)
                    break;
            }
            if (!planning_success) {
                MovementAnalysis::make(mv_props, false, begin);
                return false;
            }

            bool execution_success = move_group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;

            MovementAnalysis::make(mv_props, execution_success, begin, move_group, plan);

            // Reset to the default planner
            move_group.setPlannerId(ManipulatorProperties::getPlannerID(TASKIT_PLANNING_GROUP_ID));

            return execution_success;
        }
    
    private:
        std::string m_optimal_planner_id;
        double m_planning_time;
        uint32_t m_n_retries = 3;
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