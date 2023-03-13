#pragma once

// STL
#include <tuple>
#include <memory>
#include <array>

// ROS
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/assert.h>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>

// Manipulation Interface
#include "Quaternions.h"

namespace ManipulationInterface {

    template <class OBJ_GROUP_T, class...ACTION_PRIMITIVES_T>
    class ManipulatorNode {
        public:
            ManipulatorNode(int argc, char** argv, const std::string& planning_group, const std::string& frame_id, const std::shared_ptr<OBJ_GROUP_T>& obj_group, ACTION_PRIMITIVES_T&&...action_primitives);

            inline bool setPlanner(const std::string& planner_id) { m_move_group.setPlannerId(planner_id); }

            bool createWorkspace(const std::string& param_ns = "workspace");
            //void updateWorkspace();
            //TODO: void applyWorkspace(const std::string& domain);

            void setEndEffectorLink(const std::string& ee_link) { m_move_group->setEndEffectorLink(ee_link); }

            template <class ACTION_PRIMITIVE_T, typename...ARGS_T>
            void callAction(ARGS_T&&...args) {
                _callActionPrimitive<0>(args);
            }

            template <class ACTION_PRIMITIVE_T, typename...ARGS_T>
            void callAction(ARGS_T&&...args) const {
                _callActionPrimitiveConst<0>(args);
            }

        private:
            std::string getParamName(const std::string& param_name, std::string namespace = std::string());

            template <uint32_t I, class ACTION_PRIMITIVE_T, typename...ARGS_T>
            void _callActionPrimitive(ARGS_T&&...args) {
                if constexpr (I != (sizeof...(ACTION_PRIMITIVES_T) - 1)) {
                    if constexpr (std::is_same<std::tuple<ACTION_PRIMITIVES_T...>, ACTION_PRIMITIVE_T>::value) {
                        // Calls operator () when the type is found
                        std::get<I>(m_action_primitives)(*m_move_group, std::forward<ARGS_T>(args)...);
                    } else {
                        // Not found, continue searching
                        _callActionPrimitive<I + 1, ACTION_PRIMITIVE_T, ARGS_T...>(args);
                    }
                } else {
                    static_assert(false, "Action primitive type is not found");
                }
            }

            template <uint32_t I, class ACTION_PRIMITIVE_T, typename...ARGS_T>
            void _callActionPrimitiveConst(ARGS_T&&...args) const {
                if constexpr (I != (sizeof...(ACTION_PRIMITIVES_T) - 1)) {
                    if constexpr (std::is_same<std::tuple<ACTION_PRIMITIVES_T...>, ACTION_PRIMITIVE_T>::value) {
                        // Calls operator () when the type is found
                        std::get<I>(m_action_primitives)(*m_move_group, std::forward<ARGS_T>(args)...);
                    } else {
                        // Not found, continue searching
                        _callActionPrimitiveConst<I + 1, ACTION_PRIMITIVE_T, ARGS_T...>(args);
                    }
                } else {
                    static_assert(false, "Action primitive type is not found");
                }
            }

        private:
            std::unique_ptr<ros::NodeHandle> m_node_handle;
            std::unique_ptr<ros::AsyncSpinner> m_spinner;
            std::tuple<ACTION_PRIMITIVES_T...> m_action_primitives;
            std::shared_ptr<OBJ_GROUP_T> m_obj_group;
            std::shared_ptr<moveit::planning_interface::MoveGroupInterface> m_move_group;
            std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> m_planning_interface;
            std::shared_ptr<moveit_visual_tools::MoveItVisualTools> m_visual_tools;
            planning_scene::PlanningScenePtr m_planning_scene;
            moveit_visual_tools::MoveItVisualTools m_visual_tools;
            std::string m_frame_id;
            std::vector<moveit_msgs::CollisionObject> m_collision_objects;

            std::vector<ros::ServiceServer> m_action_services;

            static inline const std::string s_node_name = "manipulator_node";
    };
}

#include "ManipulatorNode_impl.h"