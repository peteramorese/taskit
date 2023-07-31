#pragma once

#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include "BaseActionPrimitives.h"

namespace TaskIt {
namespace ActionPrimitives {

class CartesianMover {
    public:
        CartesianMover(double max_velocity_scale)
            : m_max_velocity_scale(max_velocity_scale)
        {
            m_eef_step = ManipulatorProperties::getLinearEEFStepSize("panda_arm");
            m_jump_thresh = ManipulatorProperties::getLinearJumpThreshold("panda_arm");
            m_max_acceleration_scale = ManipulatorProperties::getLinearMaxAccelerationScale("panda_arm");
        }

        bool cartesianMove(moveit::planning_interface::MoveGroupInterface& move_group, const geometry_msgs::Point& dst_point) {
            const geometry_msgs::Pose& curr_pose = move_group.getCurrentPose().pose;

            uint32_t n_waypoints = ManipulatorProperties::getLinearNumWaypoints("panda_arm");
            ROS_ASSERT_MSG(n_waypoints > 1, "Number of linear waypoints must be greater than 1, check the arm config file");
            std::vector<geometry_msgs::Pose> waypts(n_waypoints);

            // Convert to tf2
            tf2::Vector3 dst_position, curr_position;
            tf2::fromMsg(dst_point, dst_position);
            tf2::fromMsg(curr_pose.position, curr_position);

            tf2::Vector3 diff = dst_position - curr_position;
            for (uint32_t i = 0; i < n_waypoints; ++i) {
                waypts[i] = curr_pose;
                tf2::toMsg(curr_position + static_cast<float>(i) / static_cast<float>(n_waypoints - 1) * diff, waypts[i].position);
                //DEBUG_VEC("Waypoint " << i << " position: ", waypts[i].position);
            }
            //waypts[0] = move_group.getCurrentPose().pose;
            //waypts[1] = dst_pose;
            //DEBUG_VEC("dst pose: ", waypts[1].position);

            moveit_msgs::RobotTrajectory trajectory;
            double fraction = move_group.computeCartesianPath(waypts, m_eef_step, m_jump_thresh, trajectory);

            if (fraction <= 0.0) {
                ROS_ERROR("Safe cartesian path computation failed!");
                if (!ManipulatorProperties::enforceSafeLinearMovement("panda_arm")) {
                    ROS_WARN("Attempting cartesian path computation without collision checking. Adjust the arm_config file if this behavior is undesired");
                    fraction = move_group.computeCartesianPath(waypts, m_eef_step, m_jump_thresh, trajectory, false);
                    if (fraction <= 0.0) {
                        ROS_ERROR("Cartesian path computation failed!");
                        return false;
                    }
                }
            }

			robot_trajectory::RobotTrajectory robot_trajectory(move_group.getRobotModel(), "panda_arm");
            robot_trajectory.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory);
            m_iptp.computeTimeStamps(robot_trajectory, m_max_acceleration_scale); 

            moveit_msgs::RobotTrajectory robot_trajectory_msg;
            robot_trajectory.getRobotTrajectoryMsg(robot_trajectory_msg);
            move_group.setMaxVelocityScalingFactor(m_max_velocity_scale);
            return move_group.execute(robot_trajectory_msg) == moveit::core::MoveItErrorCode::SUCCESS;
        }
    private:
        double m_eef_step;
        double m_jump_thresh;
        double m_max_acceleration_scale;
        double m_max_velocity_scale;
        trajectory_processing::IterativeParabolicTimeParameterization m_iptp;

};

class LinearTransit : public Transit, public CartesianMover {
    public:
        LinearTransit(const std::string& topic, double planning_time, uint8_t max_trials, double distance, double max_velocity_scaling_factor = 0.5)
            : Transit(topic, planning_time, max_trials, max_velocity_scaling_factor)
            , CartesianMover(max_velocity_scaling_factor)
            , m_approach_distance(distance)
            , m_retreat_distance(distance)
        {}


        virtual bool operator()(ManipulatorNodeInterface&& interface, msg_t::Request& request, msg_t::Response& response) override {
            ros::Time begin = ros::Time::now();

            // Extract what we need
            auto move_group = interface.move_group.lock();
            auto obj_group = interface.object_group.lock();
            auto predicate_handler = interface.predicate_handler.lock();
            auto vis = interface.visualizer.lock();
            auto state = interface.state.lock();

            // Get the goal pose from the request location
            GoalPoseProperties goal_pose_props = getGoalPose(*predicate_handler, *obj_group, request);
            if (goal_pose_props.moving_to_object) {
                ROS_INFO_STREAM("Goal pose for location '" << request.destination_location <<"' extracted from pose for object: " << goal_pose_props.obj_id);
            } else {
                ROS_INFO_STREAM("Goal pose for location '" << request.destination_location <<"' extracted from predicate handler");
            }

            // Get the grasp pose options
            std::vector<EndEffectorGoalPoseProperties> eef_poses = getGraspGoalPoses(*obj_group, goal_pose_props);
            std::vector<EndEffectorGoalPoseProperties> approach_offset_eef_poses = getGraspGoalPoses(*obj_group, goal_pose_props, m_approach_distance);

            response.plan_success = false;
            response.execution_success = false;

            move_group->setPlanningTime(m_planning_time);

            uint8_t trial = 0;
            while (true) {

                // If the eef is near an object, perform the retreat
                if (state->near_object) {
                    geometry_msgs::Point dst_retreat_point = move_group->getCurrentPose().pose.position;
                    tf2::Vector3 retreat_direction = Quaternions::getEndEffectorHeading(Quaternions::convert(move_group->getCurrentPose().pose.orientation));
                    dst_retreat_point.x -= m_retreat_distance * retreat_direction[0];
                    dst_retreat_point.y -= m_retreat_distance * retreat_direction[1];
                    dst_retreat_point.z -= m_retreat_distance * retreat_direction[2];
                    if (!cartesianMove(*move_group, dst_retreat_point)) {
                        // Failure
                        return true;
                    }
                }

                move_group->setStartStateToCurrentState();
                moveit::planning_interface::MoveGroupInterface::Plan plan;

                for (uint32_t i=0; i<eef_poses.size(); ++i) {

                    const auto& eef_pose = eef_poses[i].pose;

                    const auto& approach_offset_eef_pose = approach_offset_eef_poses[i].pose;

                    move_group->setPoseTarget(approach_offset_eef_pose);
                    ros::WallDuration(1.0).sleep();
                    
                    // Visualize plan goal
                    if (goal_pose_props.moving_to_object) {
                        vis->publishGoalObjectMarker(approach_offset_eef_pose, goal_pose_props.pose, "Planning...");
                    } else {
                        vis->publishGoalMarker(approach_offset_eef_pose, "Planning...");
                    }

                    response.plan_success = move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;
                    ros::WallDuration(1.0).sleep();
                    if (response.plan_success) {
                        move_group->setMaxVelocityScalingFactor(m_max_velocity_scaling_factor);

                        // Visualize actual goal
                        if (goal_pose_props.moving_to_object) {
                            vis->publishGoalObjectMarker(approach_offset_eef_pose, goal_pose_props.pose, "Goal");
                        } else {
                            vis->publishGoalMarker(approach_offset_eef_pose, "Goal");
                        }

                        response.execution_success = move_group->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;

                        // If the execution succeeded, perform the cartesian approach
                        if (response.execution_success && cartesianMove(*move_group, eef_pose.position)) {
                            updateState(*state, request.destination_location, goal_pose_props.moving_to_object, eef_poses[i].rotation_type, eef_poses[i].placing_offset);
                        }
                        response.execution_time = (ros::Time::now() - begin).toSec();
                        return true;
                    }
                }

                ++trial;        
                if (trial >= m_max_trials) {
                    response.execution_time = (ros::Time::now() - begin).toSec();
                    return true;
                }
            }
        }

    protected:
        double m_approach_distance;
        double m_retreat_distance;
};

class LinearTransitUp : public LinearTransit {
    public:
        LinearTransitUp(const std::string& topic, double planning_time, uint8_t max_trials, double distance, double max_velocity_scaling_factor = 0.5)
            : LinearTransit(topic, planning_time, max_trials, distance, max_velocity_scaling_factor) {}

        virtual std::vector<Quaternions::RotationType> getTransitRotationTypes() const override {
            // Use only None and pitch 180 (up or down)
            return {Quaternions::RotationType::None, Quaternions::RotationType::Pitch180};
        }
};

class LinearTransitSide : public LinearTransit {
    public:
        LinearTransitSide(const std::string& topic, double planning_time, uint8_t max_trials, double distance, double max_velocity_scaling_factor = 0.5)
            : LinearTransit(topic, planning_time, max_trials, distance, max_velocity_scaling_factor) {}

        virtual std::vector<Quaternions::RotationType> getTransitRotationTypes() const override {
            // Use only pitch 90 and pitch 270 (side left or side right)
            return {Quaternions::RotationType::Pitch90, Quaternions::RotationType::Pitch270};
        }
};

class LinearTransport : public Transport, public CartesianMover {
    public:
        LinearTransport(const std::string& topic, double planning_time, uint8_t max_trials, double distance, double max_velocity_scaling_factor = 0.5)
            : Transport(topic, planning_time, max_trials, max_velocity_scaling_factor)
            , CartesianMover(max_velocity_scaling_factor)
            , m_approach_offset(0.0, 0.0, distance)
            , m_retreat_offset(0.0, 0.0, distance)
        {}

        inline void setApproachOffset(const tf2::Vector3& approach_offset) {m_approach_offset = approach_offset;}
        inline void setRetreatOffset(const tf2::Vector3& retreat_offset) {m_retreat_offset = retreat_offset;}

        virtual bool operator()(ManipulatorNodeInterface&& interface, msg_t::Request& request, msg_t::Response& response) override {
            ros::Time begin = ros::Time::now();

            // Extract what we need
            auto move_group = interface.move_group.lock();
            auto obj_group = interface.object_group.lock();
            auto predicate_handler = interface.predicate_handler.lock();
            auto vis = interface.visualizer.lock();
            auto state = interface.state.lock();

            response.plan_success = false;
            response.execution_success = false;

            // Make sure at least one object is attached
            if (!interface.planning_scene_interface.lock()->getAttachedObjects().size()) {
                ROS_WARN_STREAM("Did not find attached object (is the manipulator grasping?), not executing");
                return false;
            }

            // Get the goal pose from the request location
            GoalPoseProperties goal_pose_props = getGoalPose(*predicate_handler, *obj_group, request);
            if (goal_pose_props.moving_to_object) {
                ROS_ERROR_STREAM("Destination location '" << request.destination_location << "' is occupied by object: " << goal_pose_props.obj_id << ", not executing");
                return false;
            } else {
                ROS_INFO_STREAM("Goal pose for location '" << request.destination_location <<"' extracted from predicate handler");
            }

            // Set the temporary member before getting grasp goal poses
            t_grasp_rotation_type = state->grasp_rotation_type;

            // Get the grasp pose options
            std::vector<EndEffectorGoalPoseProperties> eef_poses = getGraspGoalPoses(*obj_group, goal_pose_props, state->placing_offset);

            move_group->setPlanningTime(m_planning_time);

            uint8_t trial = 0;
            while (true) {
                move_group->setStartStateToCurrentState();
                moveit::planning_interface::MoveGroupInterface::Plan plan;

                // If the eef is near an object, perform the retreat
                if (state->near_object) {
                    geometry_msgs::Point dst_retreat_point = move_group->getCurrentPose().pose.position;
                    dst_retreat_point.x += m_retreat_offset[0];
                    dst_retreat_point.y += m_retreat_offset[1];
                    dst_retreat_point.z += m_retreat_offset[2];
                    if (!cartesianMove(*move_group, dst_retreat_point)) {
                        // Failure
                        return true;
                    }
                }

                for (const auto& eef_pose_props : eef_poses) {

                    const auto& eef_pose = eef_pose_props.pose;

                    geometry_msgs::Pose approach_offset_eef_pose = eef_pose;

                    approach_offset_eef_pose.position.x += m_approach_offset[0];
                    approach_offset_eef_pose.position.y += m_approach_offset[1];
                    approach_offset_eef_pose.position.z += m_approach_offset[2];

                    move_group->setPoseTarget(approach_offset_eef_pose);
                    ros::WallDuration(1.0).sleep();
                    
                    // Visualize plan goal
                    if (goal_pose_props.moving_to_object) {
                        vis->publishGoalObjectMarker(approach_offset_eef_pose, goal_pose_props.pose, "Planning...");
                    } else {
                        vis->publishGoalMarker(approach_offset_eef_pose, "Planning...");
                    }

                    response.plan_success = move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;
                    ros::WallDuration(1.0).sleep();
                    if (response.plan_success) {
                        move_group->setMaxVelocityScalingFactor(m_max_velocity_scaling_factor);

                        // Visualize actual goal
                        if (goal_pose_props.moving_to_object) {
                            vis->publishGoalObjectMarker(approach_offset_eef_pose, goal_pose_props.pose, "Goal");
                        } else {
                            vis->publishGoalMarker(approach_offset_eef_pose, "Goal");
                        }

                        response.execution_success = move_group->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
                        if (response.execution_success && cartesianMove(*move_group, eef_pose.position)) {
                            // Update destination location, must be near object (holding), keep rotation type, keep placing offset
                            updateState(*state, request.destination_location, true, state->grasp_rotation_type, state->placing_offset);
                        }
                        response.execution_time = (ros::Time::now() - begin).toSec();
                        return true;
                    }
                }

                ++trial;        
                if (trial >= m_max_trials) {
                    response.execution_time = (ros::Time::now() - begin).toSec();
                    return true;
                }
            }
        }
    protected:
        tf2::Vector3 m_approach_offset;
        tf2::Vector3 m_retreat_offset;

};

}
}