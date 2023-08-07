#pragma once

// STL
#include <string>
#include <vector>

// ROS
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// MoveIt
#include <moveit/planning_interface/planning_interface.h>

// TaskIt
#include "ManipulatorNode.h"
#include "ManipulatorNodeInterface.h"
#include "PredicateHandler.h"
#include "Gripper.h"
#include "Tools.h"
#include "MovementAnalysis.h"

// Srv types
#include "taskit/Stow.h"
#include "taskit/Grasp.h"
#include "taskit/Release.h"
#include "taskit/Transit.h"
#include "taskit/UpdateEnv.h"
#include "taskit/GetObjectLocations.h"

namespace TaskIt {
namespace ActionPrimitives {


// Abstract specification for an action primitive
template <class SRV_MSG_T>
class ActionPrimitive {
    protected:
        ActionPrimitive(const std::string& topic) : m_topic(topic) {}

    public:
        typedef SRV_MSG_T msg_t;

        const std::string& topic() const {return m_topic;}

        virtual bool operator()(ManipulatorNodeInterface&& interface, typename msg_t::Request& request, typename msg_t::Response& response) = 0;

    private:
        const std::string m_topic;
};

//// Convenience class for moving the arm with the move group interface
//class Mover {
//    protected:
//        Mover() = default;
//
//        void setScalingFactors() const {
//
//        }
//    protected:
//        std::string m_default_planning_time;
//};

class Stow : public ActionPrimitive<taskit::Stow> {
    public:
        Stow(const std::string& topic)
            : ActionPrimitive<taskit::Stow>(topic)
            {}

        virtual bool operator()(ManipulatorNodeInterface&& interface, typename msg_t::Request& request, typename msg_t::Response& response) override {
            ros::Time begin = ros::Time::now();
            auto move_group = interface.move_group.lock();
            interface.state.lock()->reset();
            
            move_group->clearPoseTargets();
            move_group->setStartStateToCurrentState();
            move_group->setJointValueTarget(ManipulatorProperties::getStowJointValues("panda_arm"));
            
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            response.plan_success = move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;
            bool execution_success = move_group->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;

            makeMovementProperties(response.mv_props, execution_success, begin, *move_group, plan);
            return true;
        }
};

class UpdateEnvironment : public ActionPrimitive<taskit::UpdateEnv> {
    public:
        UpdateEnvironment(const std::string& topic)
            : ActionPrimitive<taskit::UpdateEnv>(topic)
            {}

        virtual bool operator()(ManipulatorNodeInterface&& interface, typename msg_t::Request& request, typename msg_t::Response& response) override {
            auto move_group = interface.move_group.lock();
            auto obj_group = interface.object_group.lock();
            auto pci = interface.planning_scene_interface.lock();
            response.found_all = obj_group->updatePlanningScene(*pci, move_group->getPlanningFrame(), !request.include_static);
            return true;
        }
        
};

class GetObjectLocations : public ActionPrimitive<taskit::GetObjectLocations> {
    public:
        GetObjectLocations(const std::string& topic)
            : ActionPrimitive<taskit::GetObjectLocations>(topic)
            {}

        virtual bool operator()(ManipulatorNodeInterface&& interface, typename msg_t::Request& request, typename msg_t::Response& response) override {
            auto predicate_handler = interface.predicate_handler.lock();
            auto pci = interface.planning_scene_interface.lock();
            
            // Get the current predicates
            typename PredicateHandler::PredicateSet predicate_set = predicate_handler->getPredicates();

            // Get any objects that the eef is holding 
            auto attached_objects = pci->getAttachedObjects();
            response.eef_holding = !attached_objects.empty();

            response.object_locations.resize(request.object_ids.size());
            response.found_all = true;
            std::size_t i = 0;
            for (const auto& obj_id : request.object_ids) {
                if (!attached_objects.empty() && attached_objects.find(obj_id) != attached_objects.end()) {
                    response.object_locations[i++] = "eef";
                    continue;
                }
                std::pair<bool, std::string> pred = predicate_set.lookupObjectPredicate(obj_id);   
                if (!pred.first) {
                    ROS_WARN_STREAM("Could not retrieve predicate for object '" << obj_id << "'");
                    response.found_all = false;
                }
                response.object_locations[i++] = pred.second;
            }
            if (!response.found_all) {
                response.object_locations.clear();
            }
            return true;
        }
};

template <GripperUse GRIPPER_USE_T>
class SimpleGrasp : public ActionPrimitive<taskit::Grasp> {
    public: 
        SimpleGrasp(const std::string& topic, const std::shared_ptr<GripperHandler<GRIPPER_USE_T>>& gripper_handler, const std::string& attachment_link) 
            : ActionPrimitive<taskit::Grasp>(topic)
            , m_gripper_handler(gripper_handler)
            , m_attachment_link(attachment_link)
            {}

        virtual bool operator()(ManipulatorNodeInterface&& interface, typename msg_t::Request& request, typename msg_t::Response& response) override {
            ros::Time begin = ros::Time::now();
            auto move_group = interface.move_group.lock();
            auto obj_group = interface.object_group.lock();
            auto predicate_handler = interface.predicate_handler.lock();
            auto state = interface.state.lock();

            if (request.obj_id == "none") {
                GripperSpecification spec;
                spec.grip_force = 10.0f;
                spec.grip_width_closed = 0.0f;
                bool execution_success = m_gripper_handler->close(spec);
                makeMovementProperties(response.mv_props, execution_success, begin);
                return true;
            }

            std::string obj_id;
            if (request.obj_id.empty()) {
                std::pair<bool, std::string> curr_location_predicate = predicate_handler->getPredicates().lookupLocationPredicate(state->curr_location_name);
                obj_id = curr_location_predicate.second;
                ROS_ASSERT_MSG(curr_location_predicate.first, "Object not found in current location");
                ROS_INFO_STREAM("Found object '" << curr_location_predicate.second << "' to pickup");
            } else {
                ROS_ASSERT_MSG(obj_group->hasObject(request.obj_id), "Object not found");
                obj_id = request.obj_id;
            }
            bool execution_success = m_gripper_handler->close(*(obj_group->getObject(obj_id).spec));
            move_group->attachObject(obj_id, m_attachment_link);

            makeMovementProperties(response.mv_props, execution_success, begin);
            return true;
        }

    private:
        std::shared_ptr<GripperHandler<GRIPPER_USE_T>> m_gripper_handler;
        std::string m_attachment_link;
};

template <GripperUse GRIPPER_USE_T>
class SimpleRelease : public ActionPrimitive<taskit::Release> {
    public: 
        SimpleRelease(const std::string& topic, const std::shared_ptr<GripperHandler<GRIPPER_USE_T>>& gripper_handler) 
            : ActionPrimitive<taskit::Release>(topic)
            , m_gripper_handler(gripper_handler)
            {}

        virtual bool operator()(ManipulatorNodeInterface&& interface, typename msg_t::Request& request, typename msg_t::Response& response) override {
            ros::Time begin = ros::Time::now();
            auto move_group = interface.move_group.lock();
            auto obj_group = interface.object_group.lock();
            auto planning_scene_interface = interface.planning_scene_interface.lock();

            if (request.obj_id == "none") {
                if (!planning_scene_interface->getAttachedObjects().empty()) {
                    move_group->detachObject();
                }
                bool execution_success = m_gripper_handler->open(GripperSpecification{});
                makeMovementProperties(response.mv_props, execution_success, begin);
                return true;
            }

            ROS_ASSERT_MSG(obj_group->hasObject(request.obj_id), "Object not found");

            auto attached_objects = planning_scene_interface->getAttachedObjects();
            std::string obj_id;
            if (!request.obj_id.empty()) {
                auto it = attached_objects.find(request.obj_id);
                ROS_ASSERT_MSG(it != attached_objects.end(), "Release object is not currently attached");
                obj_id = it->first;
            } else {
                ROS_ASSERT_MSG(!attached_objects.empty(), "No objects attached, cannot release");
                obj_id = attached_objects.begin()->first;
            }

            bool execution_success = m_gripper_handler->open(*(obj_group->getObject(obj_id).spec));
            move_group->detachObject(obj_id);

            makeMovementProperties(response.mv_props, execution_success, begin);
            return true;
        }

    private:
        std::shared_ptr<GripperHandler<GRIPPER_USE_T>> m_gripper_handler;
};

class Transit : public ActionPrimitive<taskit::Transit> {
    public:
        Transit(const std::string& topic, double planning_time, uint8_t max_trials)
            : ActionPrimitive<taskit::Transit>(topic)
            , m_planning_time(planning_time)
            , m_max_trials(max_trials)
            , m_max_velocity_scaling_factor(ManipulatorProperties::getMaxAccelerationScale("panda_arm"))
        {}

        virtual bool operator()(ManipulatorNodeInterface&& interface, typename msg_t::Request& request, typename msg_t::Response& response) override {
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

            response.plan_success = false;
            bool execution_success = false;

            move_group->setPlanningTime(m_planning_time);

            uint8_t trial = 0;
            while (true) {
                move_group->setStartStateToCurrentState();
                moveit::planning_interface::MoveGroupInterface::Plan plan;

                for (const auto& eef_pose_props : eef_poses) {

                    const auto& eef_pose = eef_pose_props.pose;
                    Quaternions::RotationType pose_rot_type = eef_pose_props.rotation_type;

                    move_group->setPoseTarget(eef_pose);
                    ros::WallDuration(1.0).sleep();
                    
                    // Visualize plan goal
                    if (goal_pose_props.moving_to_object) {
                        vis->publishGoalObjectMarker(eef_pose, goal_pose_props.pose, "Planning...");
                    } else {
                        vis->publishGoalMarker(eef_pose, "Planning...");
                    }

                    response.plan_success = move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;
                    ros::WallDuration(1.0).sleep();
                    if (response.plan_success) {
                        move_group->setMaxVelocityScalingFactor(m_max_velocity_scaling_factor);

                        // Visualize actual goal
                        if (goal_pose_props.moving_to_object) {
                            vis->publishGoalObjectMarker(eef_pose, goal_pose_props.pose, "Goal");
                        } else {
                            vis->publishGoalMarker(eef_pose, "Goal");
                        }

                        execution_success = move_group->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
                        if (execution_success) {
                            updateState(*state, request.destination_location, goal_pose_props.moving_to_object, pose_rot_type, eef_pose_props.placing_offset);
                        }

                        makeMovementProperties(response.mv_props, execution_success, begin, *move_group, plan);
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

        struct EndEffectorGoalPoseProperties {
            EndEffectorGoalPoseProperties(const geometry_msgs::Pose& pose_, Quaternions::RotationType rotation_type_, float placing_offset_)
                : pose(pose_)
                , rotation_type(rotation_type_)
                , placing_offset(placing_offset_)
            {}

            geometry_msgs::Pose pose;
            Quaternions::RotationType rotation_type;
            float placing_offset;
        };
    protected:

        void updateState(ManipulatorNodeState& state, const std::string& curr_location_name, bool near_object, Quaternions::RotationType final_rotation_type, double placing_offset) const {
            state.curr_location_name = curr_location_name;
            state.near_object = near_object;
            state.grasp_rotation_type = final_rotation_type;
            state.placing_offset = placing_offset;
        }

        virtual std::vector<Quaternions::RotationType> getTransitRotationTypes() const {
            // Use all rotation types
            return {Quaternions::RotationType::None, Quaternions::RotationType::Pitch90, Quaternions::RotationType::Pitch180, Quaternions::RotationType::Pitch270};
        }

        static float getOffsetDimension(const Object& obj, Quaternions::RotationType rotation_type) {
            switch (rotation_type) {
                case Quaternions::RotationType::None:
                case Quaternions::RotationType::Pitch180: return obj.spec->getHeightOffset();
                case Quaternions::RotationType::Pitch90:
                case Quaternions::RotationType::Pitch270: return obj.spec->getLengthOffset();
                // length offset
            }
            ROS_ASSERT_MSG(false, "Unknown rotation type");
            return 0.0f;
        }

        static GoalPoseProperties getGoalPose(const PredicateHandler& predicate_handler, const ObjectGroup& obj_group, const typename msg_t::Request& request) {

		    const PredicateHandler::PredicateSet predicate_set = predicate_handler.getPredicates();
            std::pair<bool, std::string> location_predicate = predicate_set.lookupLocationPredicate(request.destination_location);

            if (location_predicate.first) // Object is in location
                return GoalPoseProperties(obj_group.getObject(location_predicate.second).pose(), true, location_predicate.second);
            else // No object, just to the location
                return GoalPoseProperties(predicate_handler.getLocationPose(request.destination_location), false, std::string());
        }

        std::vector<EndEffectorGoalPoseProperties> getGraspGoalPoses(const ObjectGroup& obj_group, const GoalPoseProperties& goal_pose_props, double distance_offset = 0.0) {

            std::vector<Quaternions::RotationType> rotation_types = getTransitRotationTypes();
            std::vector<EndEffectorGoalPoseProperties> grasp_poses;
            grasp_poses.reserve(rotation_types.size());
            
            for (auto rot_type : rotation_types) {
                // Grab object along its 'height' axis
                tf2::Vector3 relative_offset(0.0, 0.0, 0.0);
                if (goal_pose_props.moving_to_object) {
                    relative_offset[2] = getOffsetDimension(obj_group.getObject(goal_pose_props.obj_id), rot_type);
                }
                double eef_offset = ManipulatorProperties::getEndEffectorOffset("panda_arm");
                relative_offset[2] += distance_offset + eef_offset; // Apply distance and eef offset regardless
                grasp_poses.emplace_back(Quaternions::getPointAlongPose("panda_arm", relative_offset, goal_pose_props.pose, rot_type), rot_type, relative_offset[2] - eef_offset);
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
        TransitUp(const std::string& topic, double planning_time, uint8_t max_trials) 
            : Transit(topic, planning_time, max_trials) {}

        virtual std::vector<Quaternions::RotationType> getTransitRotationTypes() const override {
            // Use only None and pitch 180 (up or down)
            return {Quaternions::RotationType::None, Quaternions::RotationType::Pitch180};
        }
};

class TransitSide : public Transit {
    public:
        TransitSide(const std::string& topic, double planning_time, uint8_t max_trials) 
            : Transit(topic, planning_time, max_trials) {}

        virtual std::vector<Quaternions::RotationType> getTransitRotationTypes() const override {
            // Use only pitch 90 and pitch 270 (side left or side right)
            return {Quaternions::RotationType::Pitch90, Quaternions::RotationType::Pitch270};
        }
};

class Transport : public Transit {
    public:
        Transport(const std::string& topic, double planning_time, uint8_t max_trials)
            : Transit(topic, planning_time, max_trials)
        {}

        virtual bool operator()(ManipulatorNodeInterface&& interface, typename msg_t::Request& request, typename msg_t::Response& response) override {
            ros::Time begin = ros::Time::now();

            // Extract what we need
            auto move_group = interface.move_group.lock();
            auto obj_group = interface.object_group.lock();
            auto predicate_handler = interface.predicate_handler.lock();
            auto vis = interface.visualizer.lock();
            auto state = interface.state.lock();

            response.plan_success = false;
            bool execution_success = false;

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

                for (const auto& eef_pose_props : eef_poses) {

                    const auto& eef_pose = eef_pose_props.pose;

                    move_group->setPoseTarget(eef_pose);
                    ros::WallDuration(1.0).sleep();
                    
                    // Visualize plan goal
                    if (goal_pose_props.moving_to_object) {
                        vis->publishGoalObjectMarker(eef_pose, goal_pose_props.pose, "Planning...");
                    } else {
                        vis->publishGoalMarker(eef_pose, "Planning...");
                    }

                    response.plan_success = move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;
                    ros::WallDuration(1.0).sleep();
                    if (response.plan_success) {
                        move_group->setMaxVelocityScalingFactor(m_max_velocity_scaling_factor);

                        // Visualize actual goal
                        if (goal_pose_props.moving_to_object) {
                            vis->publishGoalObjectMarker(eef_pose, goal_pose_props.pose, "Goal");
                        } else {
                            vis->publishGoalMarker(eef_pose, "Goal");
                        }

                        execution_success = move_group->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
                        if (execution_success) {
                            // Update destination location, must be near object (holding), keep rotation type, keep placing offset
                            updateState(*state, request.destination_location, true, state->grasp_rotation_type, state->placing_offset);
                        }
                        makeMovementProperties(response.mv_props, execution_success, begin, *move_group, plan);
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

        virtual std::vector<Quaternions::RotationType> getTransitRotationTypes() const {
            // Use all rotation types
            switch (t_grasp_rotation_type) {
                case Quaternions::RotationType::None:
                case Quaternions::RotationType::Pitch180: return {Quaternions::RotationType::None, Quaternions::RotationType::Pitch180};
                case Quaternions::RotationType::Pitch90:
                case Quaternions::RotationType::Pitch270: return {Quaternions::RotationType::Pitch90, Quaternions::RotationType::Pitch270};
                // length offset
            }
            ROS_ASSERT_MSG(false, "Unknown rotation type");
            return {};
        }

    protected:
        Quaternions::RotationType t_grasp_rotation_type;
};


}
}