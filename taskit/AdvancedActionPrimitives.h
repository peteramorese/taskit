#pragma once

#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include "BaseActionPrimitives.h"
#include "MovementAnalysis.h"
#include "LinearMover.h"

namespace TaskIt {
namespace ActionPrimitives {

class LinearTransit : public Transit {
    public:
        LinearTransit(const std::string& topic, double planning_time, uint8_t max_trials, double distance, const std::shared_ptr<LinearMover>& linear_mover)
            : Transit(topic, planning_time, max_trials)
            , m_approach_distance(distance)
            , m_retreat_distance(distance)
            , m_linear_mover(linear_mover)
        {}


        virtual bool operator()(ManipulatorNodeInterface&& interface, msg_t::Request& request, msg_t::Response& response) override {
            ros::Time begin = ros::Time::now();

            // Extract what we need
            auto move_group = interface.move_group.lock();
            auto obj_group = interface.object_group.lock();
            auto predicate_handler = interface.predicate_handler.lock();
            auto vis = interface.visualizer.lock();
            auto state = interface.state.lock();
            auto pci = interface.planning_scene_interface.lock();

            response.plan_success = false;
            bool execution_success = false;

            // Make sure no objects are attached
            if (pci->getAttachedObjects().size()) {
                ROS_ERROR_STREAM("Cannot 'transit' while gripping an object. Use 'transport' instead. Not executing");
                makeMovementProperties(response.mv_props, execution_success, begin);
                return false;
            }

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
                    if (!m_linear_mover->move(response.mv_props, *move_group, dst_retreat_point)) {
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

                        execution_success = move_group->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;

                        // Make the movement properties for the motion plan
                        taskit::MovementProperties motion_plan_mv_props;
                        makeMovementProperties(motion_plan_mv_props, execution_success, begin, *move_group, plan);

                        // Append the movement properties for the motion plan
                        appendMovementProperties(response.mv_props, motion_plan_mv_props);

                        // If the execution succeeded, perform the cartesian approach
                        taskit::MovementProperties approach_mv_props;
                        if (execution_success && m_linear_mover->move(approach_mv_props, *move_group, eef_pose.position)) {
                            updateState(*state, request.destination_location, goal_pose_props.moving_to_object, eef_poses[i].rotation_type, eef_poses[i].placing_offset);
                        }

                        // Append the movement properties for the cartesian movement
                        appendMovementProperties(response.mv_props, approach_mv_props);
                        return true;
                    }
                }

                ++trial;        
                if (trial >= m_max_trials) {
                    makeMovementProperties(response.mv_props, execution_success, begin);
                    return true;
                }
            }
        }

    protected:
        double m_approach_distance;
        double m_retreat_distance;
        std::shared_ptr<LinearMover> m_linear_mover;
};

class LinearTransitUp : public LinearTransit {
    public:
        LinearTransitUp(const std::string& topic, double planning_time, uint8_t max_trials, double distance, const std::shared_ptr<LinearMover>& linear_mover)
            : LinearTransit(topic, planning_time, max_trials, distance, linear_mover) {}

        virtual std::vector<Quaternions::RotationType> getTransitRotationTypes() const override {
            // Use only None and pitch 180 (up or down)
            return {Quaternions::RotationType::None, Quaternions::RotationType::Pitch180};
        }
};

class LinearTransitSide : public LinearTransit {
    public:
        LinearTransitSide(const std::string& topic, double planning_time, uint8_t max_trials, double distance, const std::shared_ptr<LinearMover>& linear_mover)
            : LinearTransit(topic, planning_time, max_trials, distance, linear_mover) {}

        virtual std::vector<Quaternions::RotationType> getTransitRotationTypes() const override {
            // Use only pitch 90 and pitch 270 (side left or side right)
            return {Quaternions::RotationType::Pitch90, Quaternions::RotationType::Pitch270};
        }
};

class LinearTransport : public Transport {
    public:
        LinearTransport(const std::string& topic, double planning_time, uint8_t max_trials, double distance, const std::shared_ptr<LinearMover>& linear_mover)
            : Transport(topic, planning_time, max_trials)
            , m_approach_offset(0.0, 0.0, distance)
            , m_retreat_offset(0.0, 0.0, distance)
            , m_linear_mover(linear_mover)
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
            auto pci = interface.planning_scene_interface.lock();

            response.plan_success = false;
            bool execution_success = false;

            auto attached_objects = pci->getAttachedObjects();

            // Make sure at least one object is attached
            if (!attached_objects.size()) {
                ROS_WARN_STREAM("Did not find attached object (is the manipulator holding an object?), not executing");
                makeMovementProperties(response.mv_props, execution_success, begin);
                return false;
            }

            // Get the goal pose from the request location. Use the 
            GoalPoseProperties goal_pose_props = getGoalPose(*predicate_handler, *obj_group, request, attached_objects.begin()->first); 
            if (goal_pose_props.moving_to_object) {
                ROS_ERROR_STREAM("Destination location '" << request.destination_location << "' is occupied by object: " << goal_pose_props.obj_id << ", not executing");
                makeMovementProperties(response.mv_props, execution_success, begin);
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
                    if (!m_linear_mover->move(response.mv_props, *move_group, dst_retreat_point)) {
                        // Failure
                        return true;
                    }
                }

                for (const auto& eef_pose_props : eef_poses) {

                    const auto& eef_pose = eef_pose_props.pose;

                    geometry_msgs::Pose approach_offset_eef_pose = eef_pose;

                    approach_offset_eef_pose.position.x += m_approach_offset[0];
                    approach_offset_eef_pose.position.y += m_approach_offset[1];
                    // Apply the vertical placing offset to the goal pose:
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

                        execution_success = move_group->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;

                        // Make the movement properties for the motion plan
                        taskit::MovementProperties motion_plan_mv_props;
                        makeMovementProperties(motion_plan_mv_props, execution_success, begin, *move_group, plan);

                        // Append the movement properties for the motion plan
                        appendMovementProperties(response.mv_props, motion_plan_mv_props);

                        // If the execution succeeded, perform the cartesian approach
                        taskit::MovementProperties approach_mv_props;
                        geometry_msgs::Point vertical_offset_position = eef_pose.position;
                        vertical_offset_position.z += ManipulatorProperties::getVerticalPlacingOffset("panda_arm");
                        if (execution_success && m_linear_mover->move(approach_mv_props, *move_group, vertical_offset_position)) {
                            // Update destination location, must be near object (holding), keep rotation type, keep placing offset
                            updateState(*state, request.destination_location, true, state->grasp_rotation_type, state->placing_offset);
                        }

                        // Append the movement properties for the cartesian movement
                        appendMovementProperties(response.mv_props, approach_mv_props);
                        return true;
                    }
                }

                ++trial;        
                if (trial >= m_max_trials) {
                    makeMovementProperties(response.mv_props, execution_success, begin);
                    return true;
                }
            }
        }
    protected:
        tf2::Vector3 m_approach_offset;
        tf2::Vector3 m_retreat_offset;
        std::shared_ptr<LinearMover> m_linear_mover;

};

}
}