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

namespace ManipulationInterface {
namespace ActionPrimitives {

template <class SRV_MSG_T>
class ActionPrimitive {
    public:
        typedef SRV_MSG_T msg_t;

        virtual bool operator()(moveit::planning_interface::MoveGroupInterface& move_group, msg_t::Request& request, msg_t::Response& response) = 0;
};

class SimpleGrasp : public ActionPrimitive<manipulation_interface::GraspSrv> {
    public: 
        SimpleGrasp(const std::string& attachment_link) 
            : m_attachment_link(attachment_link)
            {}

        virtual bool operator()(moveit::planning_interface::MoveGroupInterface& move_group, msg_t::Request& request, msg_t::Response& response) override {
            ROS_ERROR("YO HELLO I RAN");
            return move_group.attachObject(request.grasp_object_id, m_attachment_link);
        }

    private:
        std::string m_attachment_link;

};

class SimpleRelease : public ActionPrimitive<manipulation_interface::ReleaseSrv> {
    
};

class Transit : public ActionPrimitive<manipulation_interface::TransitSrv> {
    public:
        Transit(double planning_time, uint8_t max_trials, const tf2::Vector3 approach_direction, float approach_distance, double max_velocity_scaling_factor = 1.0)
            : m_planning_time(planning_time)
            , m_max_trials(max_trials)
            , m_approach_direction(approach_direction)
            , m_approach_distance(approach_distance)
            , m_max_velocity_scaling_factor(max_velocity_scaling_factor)
        {}

        virtual bool operator()(moveit::planning_interface::MoveGroupInterface& move_group, msg_t::Request& request, msg_t::Response& response) override {
            move_group.setPlanningTime(m_planning_time);

            std::vector<geometry_msgs::Pose> goal_poses = getGoalPoses(request.use_linear_approach);

            response.execution_success = false;

            uint8_t trial = 0;
            while (true) {
                move_group.setStartStateToCurrentState();
                moveit::planning_interface::MoveGroupInterface::Plan plan;

                for (const auto& goal_pose : goal_poses) {
                    move_group.setPoseTarget(goal_pose);
                    ros::WallDuration(1.0).sleep();
                    response.plan_success = (move_group.plan(plan)==moveit::planning_interface::MoveItErrorCode::SUCCESS);
                    ros::WallDuration(1.0).sleep();
                    if (response.plan_success) {
                        move_group.setMaxVelocityScalingFactor(m_max_velocity_scaling_factor);
                        move_group.execute(plan);
                        return true;
                    }
                }

                ++trial;        
                if (trial >= m_max_trials) {
                    return false;
                }
            }
        }

        std::vector<geometry_msgs::Pose> getGoalPoses(bool use_linear_approach) const {

        }
    
    private:
        double m_planning_time;
        uint8_t m_max_trials;
        double m_max_velocity_scaling_factor;

        tf2::Vector3 m_approach_direction;
        float m_approach_distance;

};

}
}