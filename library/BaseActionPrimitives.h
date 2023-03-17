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
        Transit(const std::string& topic, const std::shared_ptr<PredicateHandler>& predicate_handler, double planning_time, uint8_t max_trials, double max_velocity_scaling_factor = 1.0)
            : ActionPrimitive<manipulation_interface::TransitSrv>(topic)
            , m_predicate_handler(predicate_handler)
            , m_planning_time(planning_time)
            , m_max_trials(max_trials)
            , m_max_velocity_scaling_factor(max_velocity_scaling_factor)
        {}

        virtual bool operator()(ManipulatorNodeInterface&& interface, msg_t::Request& request, msg_t::Response& response) override {
            auto& move_group = *interface.move_group.lock();
            move_group.setPlanningTime(m_planning_time);

            std::vector<geometry_msgs::Pose> goal_poses = getGoalPoses(request);

            response.plan_success = false;
            response.execution_success = false;

            uint8_t trial = 0;
            while (true) {
                move_group.setStartStateToCurrentState();
                moveit::planning_interface::MoveGroupInterface::Plan plan;

                for (const auto& goal_pose : goal_poses) {
                    move_group.setPoseTarget(goal_pose);
                    ros::WallDuration(1.0).sleep();
                    response.plan_success = move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
                    ros::WallDuration(1.0).sleep();
                    if (response.plan_success) {
                        move_group.setMaxVelocityScalingFactor(m_max_velocity_scaling_factor);
                        response.execution_success = move_group.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
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

        std::vector<geometry_msgs::Pose> getGoalPoses(const msg_t::Request& request) const {
            geometry_msgs::Pose test_pose;
            test_pose.position.x = 0.4f;
            test_pose.position.y = 0.4f;
            test_pose.position.z = 0.4f;
            test_pose.orientation.x = 0.0f;
            test_pose.orientation.y = 0.0f;
            test_pose.orientation.z = 0.0f;
            test_pose.orientation.w = 1.0f;
            return {Quaternions::getPointAlongPose("panda_arm", tf2::Vector3(0.0, 0.0, 1.0), test_pose, Quaternions::RotationType::DownAxis)};
        }
    
    private:
        std::shared_ptr<PredicateHandler> m_predicate_handler;
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