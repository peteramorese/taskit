#pragma once

#include <memory>

#define MI_USE_FRANKA_GRIPPER

#ifdef MI_USE_FRANKA_GRIPPER
    // Franka
    #include "franka_gripper/GraspAction.h"
    #include "franka_gripper/GraspActionGoal.h"

    #define MI_FRANKA_GRIPPER_TOPIC "/franka_gripper/grasp"
#endif

#include "Object.h"


namespace ManipulationInterface {

enum class GripperUse {
    Simulation,
    FrankaHand
};

template <GripperUse GRIPPER_USE_T>
struct GripperHandler;

template<>
class GripperHandler<GripperUse::Simulation> {
    public:
        GripperHandler(const std::string& gripper_topic, double timeout) {}
        bool close(const ObjectSpecification& grip_spec) {return true;}
        bool open(const ObjectSpecification& grip_spec) {return true;}
};

// Compile with franka gripper
#ifdef MI_USE_FRANKA_GRIPPER

template<>
class GripperHandler<GripperUse::FrankaHand> {
    public:
        typedef franka_gripper::GraspAction action_t;
        typedef franka_gripper::GraspActionGoal action_goal_t;
    public:
        GripperHandler(const std::string& gripper_topic, double grip_width_open = 0.1, double timeout = 5.0) 
            : m_gripper_action(gripper_topic, true)
            , m_timeout(timeout) 
            , m_grip_width_open(grip_width_open)
        {}
        bool close(const ObjectSpecification& grip_spec) {
            action_goal_t grip_goal;
            grip_goal.goal.width = grip_spec.grip_width_closed;
            grip_goal.goal.speed = grip_spec.grip_speed;
            grip_goal.goal.force = grip_spec.grip_force;
            grip_goal.goal.epsilon.inner = grip_spec.grip_epsilon_inner;
            grip_goal.goal.epsilon.outer = grip_spec.grip_epsilon_outter;
            m_gripper_action.sendGoal(grip_goal.goal);
            return m_gripper_action.waitForResult(ros::Duration(m_timeout));
        }
        bool open(const ObjectSpecification& grip_spec) {
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