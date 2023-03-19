#pragma once

// STL
#include <string>
#include <vector>

// Srv types
#include "manipulation_interface/GraspSrv.h"
#include "manipulation_interface/ReleaseSrv.h"
#include "manipulation_interface/TransitSrv.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// MoveIt
#include <moveit/planning_interface/planning_interface.h>


#include "ManipulatorNodeInterface.h"
#include "PredicateHandler.h"

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

class SimpleGrasp : public ActionPrimitive<manipulation_interface::GraspSrv> {
    public: 
        SimpleGrasp(const std::string& topic, const std::string& attachment_link) 
            : ActionPrimitive<manipulation_interface::GraspSrv>(topic)
            , m_attachment_link(attachment_link)
            {}


        virtual bool operator()(ManipulatorNodeInterface&& interface, msg_t::Request& request, msg_t::Response& response) override {
            DEBUG("grasping!!");
            return interface.move_group.lock()->attachObject(request.grasp_object_id, m_attachment_link);
        }

    private:
        std::string m_attachment_link;

};

class SimpleRelease : public ActionPrimitive<manipulation_interface::ReleaseSrv> {
    
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

            // Get the goal pose from the request location
            DEBUG("looking up location: " << request.destination_location);
            GoalPoseProperties goal_pose_props = getGoalPose(*predicate_handler, *obj_group, request);
            if (goal_pose_props.moving_to_object) {
                ROS_INFO_STREAM("Goal pose for location '" << request.destination_location <<"' extracted from pose for object: " << goal_pose_props.obj_id);
            } else {
                ROS_INFO_STREAM("Goal pose for location '" << request.destination_location <<"' extracted from predicate handler");
            }

            // Get the grasp pose options
            std::vector<geometry_msgs::Pose> eef_poses = getGraspGoalPoses(*obj_group, goal_pose_props);

            response.plan_success = false;
            response.execution_success = false;

            move_group->setPlanningTime(m_planning_time);

            uint8_t trial = 0;
            while (true) {
                move_group->setStartStateToCurrentState();
                moveit::planning_interface::MoveGroupInterface::Plan plan;

                for (const auto& eef_pose : eef_poses) {
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
                        return true;
                    }
                }

                ++trial;        
                if (trial >= m_max_trials) {
                    return true;
                }
            }
        }

    private:
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
    private:
        GoalPoseProperties getGoalPose(const PredicateHandler& predicate_handler, const ObjectGroup& obj_group, const msg_t::Request& request) const {

            DEBUG("b4 get prds");
		    const PredicateHandler::PredicateSet predicate_set = predicate_handler.getPredicates();
            DEBUG("af get prds");
            std::pair<bool, std::string> location_predicate = predicate_set.lookupLocationPredicate(request.destination_location);
            DEBUG("af lookup");

            if (location_predicate.first) // Object is in location
                return GoalPoseProperties(obj_group.getObject(location_predicate.second).pose, true, location_predicate.second);
            else // No object, just to the location
                return GoalPoseProperties(predicate_handler.getLocationPose(request.destination_location), false, std::string());
        }

        std::vector<geometry_msgs::Pose> getGraspGoalPoses(const ObjectGroup& obj_group, const GoalPoseProperties& goal_pose_props) const {
            // Grab object along its 'height' axis
            tf2::Vector3 relative_offset = tf2::Vector3(0.0, 0.0, 0.1);
            if (goal_pose_props.moving_to_object)
                relative_offset[2] = obj_group.getObject(goal_pose_props.obj_id).spec->getVerticalDimension();

            std::vector<geometry_msgs::Pose> grasp_poses(2);
            grasp_poses[0] = Quaternions::getPointAlongPose("panda_arm", relative_offset, goal_pose_props.pose, Quaternions::RotationType::DownAxis);
            grasp_poses[0] = Quaternions::getPointAlongPose("panda_arm", relative_offset, goal_pose_props.pose, Quaternions::RotationType::UpAxis);
            return grasp_poses;
        }
    
    private:
        double m_planning_time;
        uint8_t m_max_trials;
        double m_max_velocity_scaling_factor;
};

//class LinearApproachTransit : public ActionPrimitive<manipulation_interface::TransitSrv> {
//    public:
//        LinearApproachTransit(const std::string& topic, const std::shared_ptr<PredicateHandler>&, double planning_time, uint8_t max_trials, const tf2::Vector3 approach_direction, float approach_distance, double max_velocity_scaling_factor = 1.0)
//            : ActionPrimitive<manipulation_interface::TransitSrv>(topic)
//            , m_planning_time(planning_time)
//            , m_max_trials(max_trials)
//            , m_approach_direction(approach_direction)
//            , m_approach_distance(approach_distance)
//            , m_max_velocity_scaling_factor(max_velocity_scaling_factor)
//        {}
//
//
//        std::vector<geometry_msgs::Pose> getGoalPoses(bool use_linear_approach) const {
//
//        }
//    
//    private:
//        double m_planning_time;
//        uint8_t m_max_trials;
//        double m_max_velocity_scaling_factor;
//
//        tf2::Vector3 m_approach_direction;
//        float m_approach_distance;
//
//};

}
}