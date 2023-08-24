#pragma once

#include <memory>


#include "Config.h"
#include "Object.h"

#ifdef TASKIT_USE_FRANKA_GRIPPER
    // Franka gripper
    #include "franka_gripper/GraspAction.h"
    #include "franka_gripper/GraspActionGoal.h"

    #define TASKIT_FRANKA_GRIPPER_TOPIC "/franka_gripper/grasp"
#endif


namespace TaskIt {

enum class GripperUse {
    Simulation,
    FrankaHand
};

template <GripperUse GRIPPER_USE_T>
struct GripperHandler;

template<>
class GripperHandler<GripperUse::Simulation> {
    public:
        GripperHandler(const std::string& gripper_topic, double timeout, double fake_execution_time = 1.5) {}

        bool close(const GripperSpecification& grip_spec) {
            // Sleep for roughly the amount of time the FrankaHand takes
            ros::WallDuration(m_fake_execution_time).sleep();
            return true;
        }

        bool open(const GripperSpecification& grip_spec) {
            // Sleep for roughly the amount of time the FrankaHand takes
            ros::WallDuration(m_fake_execution_time).sleep();
            return true;
        }
    private:
        double m_fake_execution_time;
};

#ifdef TASKIT_USE_FRANKA_GRIPPER

template<>
class GripperHandler<GripperUse::FrankaHand> {
    public:
        typedef franka_gripper::GraspAction action_t;
        typedef franka_gripper::GraspActionGoal action_goal_t;
    public:
        GripperHandler(const std::string& gripper_topic, double timeout = 5.0, double grip_width_open = 0.1) 
            : m_gripper_action(gripper_topic, true)
            , m_timeout(timeout) 
            , m_grip_width_open(grip_width_open)
        {}
        bool close(const GripperSpecification& grip_spec) {
            action_goal_t grip_goal;
            grip_goal.goal.width = grip_spec.grip_width_closed;
            grip_goal.goal.speed = grip_spec.grip_speed;
            grip_goal.goal.force = grip_spec.grip_force;
            grip_goal.goal.epsilon.inner = grip_spec.grip_epsilon_inner;
            grip_goal.goal.epsilon.outer = grip_spec.grip_epsilon_outter;
            m_gripper_action.sendGoal(grip_goal.goal);
            return m_gripper_action.waitForResult(ros::Duration(m_timeout));
        }
        bool open(const GripperSpecification& grip_spec) {
            action_goal_t grip_goal;
            grip_goal.goal.width = m_grip_width_open;
            grip_goal.goal.speed = grip_spec.grip_speed;
            grip_goal.goal.force = grip_spec.grip_force;
            grip_goal.goal.epsilon.inner = grip_spec.grip_epsilon_inner;
            grip_goal.goal.epsilon.outer = grip_spec.grip_epsilon_outter;
            m_gripper_action.sendGoal(grip_goal.goal);
            return m_gripper_action.waitForResult(ros::Duration(m_timeout));
        }
    public:
        actionlib::SimpleActionClient<franka_gripper::GraspAction> m_gripper_action;
        double m_timeout;
        double m_grip_width_open = 0.1;
};

#endif
}