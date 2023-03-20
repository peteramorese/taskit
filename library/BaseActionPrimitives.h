#pragma once

// STL
#include <string>
#include <vector>

// Srv types
#include "manipulation_interface/StowSrv.h"
#include "manipulation_interface/GraspSrv.h"
#include "manipulation_interface/ReleaseSrv.h"
#include "manipulation_interface/TransitSrv.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// MoveIt
#include <moveit/planning_interface/planning_interface.h>

#include "ManipulatorNodeInterface.h"
#include "PredicateHandler.h"
#include "Gripper.h"
#include "Tools.h"

namespace ManipulationInterface {
namespace ActionPrimitives {


template <class SRV_MSG_T>
class ActionPrimitive {
    protected:
        ActionPrimitive(const std::string& topic) : m_topic(topic) {}

    public:
        typedef SRV_MSG_T msg_t;

        const std::string& topic() const {return m_topic;}

        virtual bool operator()(ManipulatorNodeInterface&& interface, msg_t::Request& request, msg_t::Response& response) = 0;

    private:
        const std::string m_topic;
};

class Stow : public ActionPrimitive<manipulation_interface::StowSrv> {
    public:
        Stow(const std::string& topic)
            : ActionPrimitive<manipulation_interface::StowSrv>(topic)
            {}

        virtual bool operator()(ManipulatorNodeInterface&& interface, msg_t::Request& request, msg_t::Response& response) override {
            auto move_group = interface.move_group.lock();
            interface.state.lock()->reset();
            
            move_group->clearPoseTargets();
            move_group->setStartStateToCurrentState();
            move_group->setJointValueTarget(ManipulatorProperties::getStowJointValues("panda_arm"));
            response.execution_success = move_group->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS;
            return true;
        }
};

template <GripperUse GRIPPER_USE_T>
class SimpleGrasp : public ActionPrimitive<manipulation_interface::GraspSrv> {
    public: 
        SimpleGrasp(const std::string& topic, const std::shared_ptr<GripperHandler<GRIPPER_USE_T>>& gripper_handler, const std::string& attachment_link) 
            : ActionPrimitive<manipulation_interface::GraspSrv>(topic)
            , m_gripper_handler(gripper_handler)
            , m_attachment_link(attachment_link)
            {}

        virtual bool operator()(ManipulatorNodeInterface&& interface, msg_t::Request& request, msg_t::Response& response) override {
            auto move_group = interface.move_group.lock();
            auto obj_group = interface.object_group.lock();
            ROS_ASSERT_MSG(obj_group->hasObject(request.obj_id), "Object not found");
            response.success = m_gripper_handler->close(*(obj_group->getObject(request.obj_id).spec));
            move_group->attachObject(request.obj_id, m_attachment_link);

            return true;
        }

    private:
        std::shared_ptr<GripperHandler<GRIPPER_USE_T>> m_gripper_handler;
        std::string m_attachment_link;
};

template <GripperUse GRIPPER_USE_T>
class SimpleRelease : public ActionPrimitive<manipulation_interface::ReleaseSrv> {
    public: 
        SimpleRelease(const std::string& topic, const std::shared_ptr<GripperHandler<GRIPPER_USE_T>>& gripper_handler) 
            : ActionPrimitive<manipulation_interface::ReleaseSrv>(topic)
            , m_gripper_handler(gripper_handler)
            {}

        virtual bool operator()(ManipulatorNodeInterface&& interface, msg_t::Request& request, msg_t::Response& response) override {
            auto move_group = interface.move_group.lock();
            auto obj_group = interface.object_group.lock();
            auto planning_interface = interface.planning_interface.lock();
            ROS_ASSERT_MSG(obj_group->hasObject(request.obj_id), "Object not found");

            auto attached_objects = planning_interface->getAttachedObjects();
            ROS_ASSERT_MSG(attached_objects.find(request.obj_id) != attached_objects.end(), "Release object is not currently attached");

            response.success = m_gripper_handler->open(*(obj_group->getObject(request.obj_id).spec));
            move_group->detachObject(request.obj_id);
            return true;
        }

    private:
        std::shared_ptr<GripperHandler<GRIPPER_USE_T>> m_gripper_handler;
};

class Transit : public ActionPrimitive<manipulation_interface::TransitSrv> {
    public:
        Transit(const std::string& topic, double planning_time, uint8_t max_trials, double max_velocity_scaling_factor = 1.0)
            : ActionPrimitive<manipulation_interface::TransitSrv>(topic)
            , m_planning_time(planning_time)
            , m_max_trials(max_trials)
            , m_max_velocity_scaling_factor(max_velocity_scaling_factor)
        {}

        virtual bool operator()(ManipulatorNodeInterface&& interface, msg_t::Request& request, msg_t::Response& response) override {

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
            std::vector<std::pair<geometry_msgs::Pose, Quaternions::RotationType>> eef_poses = getGraspGoalPoses(*obj_group, goal_pose_props);

            response.plan_success = false;
            response.execution_success = false;

            move_group->setPlanningTime(m_planning_time);

            uint8_t trial = 0;
            while (true) {
                move_group->setStartStateToCurrentState();
                moveit::planning_interface::MoveGroupInterface::Plan plan;

                for (const auto& eef_pose_props : eef_poses) {

                    const auto& eef_pose = eef_pose_props.first;
                    Quaternions::RotationType pose_rot_type = eef_pose_props.second;

                    move_group->setPoseTarget(eef_pose);
                    ros::WallDuration(1.0).sleep();
                    
                    // Visualize plan goal
                    if (goal_pose_props.moving_to_object) {
                        vis->publishGoalObjectMarker(eef_pose, goal_pose_props.pose, "planning...");
                    } else {
                        vis->publishGoalMarker(eef_pose, "planning...");
                    }

                    response.plan_success = move_group->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
                    ros::WallDuration(1.0).sleep();
                    if (response.plan_success) {
                        move_group->setMaxVelocityScalingFactor(m_max_velocity_scaling_factor);

                        // Visualize actual goal
                        if (goal_pose_props.moving_to_object) {
                            vis->publishGoalObjectMarker(eef_pose, goal_pose_props.pose, "goal");
                        } else {
                            vis->publishGoalMarker(eef_pose, "goal");
                        }

                        response.execution_success = move_group->execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
                        if (response.execution_success) {
                            updateState(*state, goal_pose_props.moving_to_object, pose_rot_type);
                        }
                        return true;
                    }
                }

                ++trial;        
                if (trial >= m_max_trials) {
                    return true;
                }
            }
        }

        void updateState(ManipulatorNodeState& state, bool near_object, Quaternions::RotationType final_rotation_type) const {
            state.near_object = near_object;
            state.grasp_rotation_type = final_rotation_type;
        }

        virtual std::vector<Quaternions::RotationType> getTransitRotationTypes() const {
            // Use all rotation types
            return {Quaternions::RotationType::None, Quaternions::RotationType::Pitch90, Quaternions::RotationType::Pitch180, Quaternions::RotationType::Pitch270};
        }

    protected:
        struct GoalPoseProperties {
            GoalPoseProperties(const geometry_msgs::Pose& pose_, bool moving_to_object_, const std::string& obj_id_)
                : pose(pose_)
                , moving_to_object(moving_to_object_)
                , obj_id(obj_id_)
            {}

            const geometry_msgs::Pose& pose;
            bool moving_to_object;
            std::string obj_id;
        };
    protected:

        static float getOffsetDimension(const Object& obj, Quaternions::RotationType rotation_type) {
            switch (rotation_type) {
                case Quaternions::RotationType::None:
                case Quaternions::RotationType::Pitch180: return obj.spec->getHeightOffset();
                case Quaternions::RotationType::Pitch90:
                case Quaternions::RotationType::Pitch270: return obj.spec->getWidthOffset();
                // length offset
            }
            ROS_ASSERT_MSG("Unknown rotation type");
            return 0.0f;
        }

        static GoalPoseProperties getGoalPose(const PredicateHandler& predicate_handler, const ObjectGroup& obj_group, const msg_t::Request& request) {

		    const PredicateHandler::PredicateSet predicate_set = predicate_handler.getPredicates();
            std::pair<bool, std::string> location_predicate = predicate_set.lookupLocationPredicate(request.destination_location);

            if (location_predicate.first) // Object is in location
                return GoalPoseProperties(obj_group.getObject(location_predicate.second).pose, true, location_predicate.second);
            else // No object, just to the location
                return GoalPoseProperties(predicate_handler.getLocationPose(request.destination_location), false, std::string());
        }

        std::vector<std::pair<geometry_msgs::Pose, Quaternions::RotationType>> getGraspGoalPoses(const ObjectGroup& obj_group, const GoalPoseProperties& goal_pose_props, double distance_offset = 0.0) {
            // Grab object along its 'height' axis
            tf2::Vector3 relative_offset = tf2::Vector3(0.0, 0.0, 0.1);

            std::vector<Quaternions::RotationType> rotation_types = getTransitRotationTypes();
            std::vector<std::pair<geometry_msgs::Pose, Quaternions::RotationType>> grasp_poses;
            grasp_poses.reserve(rotation_types.size());
            
            for (auto rot_type : rotation_types) {
                if (goal_pose_props.moving_to_object) {
                    relative_offset[2] = getOffsetDimension(obj_group.getObject(goal_pose_props.obj_id), rot_type) + ManipulatorProperties::getEndEffectorOffset("panda_arm") + distance_offset;
                }
                grasp_poses.emplace_back(Quaternions::getPointAlongPose("panda_arm", relative_offset, goal_pose_props.pose, rot_type), rot_type);
            }
            return grasp_poses;
        }

    
    protected:
        double m_planning_time;
        uint8_t m_max_trials;
        double m_max_velocity_scaling_factor;
};

class TransitUp : public Transit {
    public:
        TransitUp(const std::string& topic, double planning_time, uint8_t max_trials, double max_velocity_scaling_factor = 1.0) 
            : Transit(topic, planning_time, max_trials, max_velocity_scaling_factor) {}

        virtual std::vector<Quaternions::RotationType> getTransitRotationTypes() const override {
            // Use only None and pitch 180 (up or down)
            return {Quaternions::RotationType::None, Quaternions::RotationType::Pitch180};
        }
};

class TransitSide : public Transit {
    public:
        TransitSide(const std::string& topic, double planning_time, uint8_t max_trials, double max_velocity_scaling_factor = 1.0) 
            : Transit(topic, planning_time, max_trials, max_velocity_scaling_factor) {}

        virtual std::vector<Quaternions::RotationType> getTransitRotationTypes() const override {
            // Use only pitch 90 and pitch 270 (side left or side right)
            return {Quaternions::RotationType::Pitch90, Quaternions::RotationType::Pitch270};
        }
};



}
}