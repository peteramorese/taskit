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
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// Manipulation Interface
#include "Quaternions.h"
#include "Tools.h"

namespace ManipulationInterface {

    template <class OBJ_GROUP_T, class...ACTION_PRIMITIVES_TYPES>
    class ManipulatorNode {
        public:
            ManipulatorNode(const std::string& node_name, const std::string& planning_group, const std::string& frame_id, const std::shared_ptr<OBJ_GROUP_T>& obj_group, ACTION_PRIMITIVES_TYPES&&...action_primitives);
            ~ManipulatorNode() {DEBUG("in dtor");}

            inline void setPlanner(const std::string& planner_id) { m_move_group->setPlannerId(planner_id); }

            const std::shared_ptr<OBJ_GROUP_T> createWorkspace(const std::string& param_ns = "workspace");
            void updateWorkspace();
            //TODO: void applyWorkspace(const std::string& domain);

            void setEndEffectorLink(const std::string& ee_link) { m_move_group->setEndEffectorLink(ee_link); }

            ros::NodeHandle& getNodeHandle() {return *m_node_handle;}
            const ros::NodeHandle& getNodeHandle() const {return *m_node_handle;}

            template <class ACTION_PRIMITIVE_T, typename...ARGS_T>
            void callAction(ARGS_T&&...args) {
                _callActionPrimitive<0, ACTION_PRIMITIVE_T, ARGS_T...>(args...);
            }

            template <class ACTION_PRIMITIVE_T, typename...ARGS_T>
            void callAction(ARGS_T&&...args) const {
                _callActionPrimitiveConst<0, ACTION_PRIMITIVE_T, ARGS_T...>(args...);
            }

        private:
            static std::string getParamName(const std::string& param_name, std::string ns = std::string());

            template <uint32_t I, class ACTION_PRIMITIVE_T, typename...ARGS_T>
            void _callActionPrimitive(ARGS_T&&...args) {
                if constexpr (I < (sizeof...(ACTION_PRIMITIVES_TYPES))) {
                    if constexpr (std::is_same<std::tuple_element_t<I, std::tuple<ACTION_PRIMITIVES_TYPES...>>, ACTION_PRIMITIVE_T>::value) {
                        // Calls operator () when the type is found
                        std::get<I>(m_action_primitives)(*m_move_group, std::forward<ARGS_T>(args)...);
                    } else {
                        // Not found, continue searching
                        _callActionPrimitive<I + 1, ACTION_PRIMITIVE_T, ARGS_T...>(args...);
                    }
                } else {
                    //static_assert(false, "Action primitive type is not found");
                }
            }

            template <uint32_t I, class ACTION_PRIMITIVE_T, typename...ARGS_T>
            void _callActionPrimitiveConst(ARGS_T&&...args) const {
                if constexpr (I < (sizeof...(ACTION_PRIMITIVES_TYPES))) {
                    if constexpr (std::is_same<std::tuple_element_t<I, std::tuple<ACTION_PRIMITIVES_TYPES...>>, ACTION_PRIMITIVE_T>::value) {
                        // Calls operator () when the type is found
                        std::get<I>(m_action_primitives)(*m_move_group, std::forward<ARGS_T>(args)...);
                    } else {
                        // Not found, continue searching
                        _callActionPrimitiveConst<I + 1, ACTION_PRIMITIVE_T, ARGS_T...>(args...);
                    }
                } else {
                    //static_assert(false, "Action primitive type is not found");
                }
            }

        private:
            const std::string m_node_name;

            std::unique_ptr<ros::NodeHandle> m_node_handle;
            std::unique_ptr<ros::AsyncSpinner> m_spinner;
            std::tuple<ACTION_PRIMITIVES_TYPES...> m_action_primitives;
            std::shared_ptr<OBJ_GROUP_T> m_obj_group;
            std::shared_ptr<moveit::planning_interface::MoveGroupInterface> m_move_group;
            std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> m_planning_interface;
            std::shared_ptr<moveit_visual_tools::MoveItVisualTools> m_visual_tools;
            std::shared_ptr<planning_scene::PlanningScene> m_planning_scene;
            std::string m_frame_id;
            std::vector<moveit_msgs::CollisionObject> m_collision_objects;

            // Things that just need to exist
            robot_model::RobotModelPtr m_robot_model;
            robot_state::RobotStatePtr m_robot_state;
            boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> m_planner_plugin_loader;
            planning_interface::PlannerManagerPtr m_planner_instance;
            //boost::shared_ptr<planning_interface::PlannerManager> m_planner_instance;

            std::vector<ros::ServiceServer> m_action_services;

    };
}

#include "ManipulatorNode_impl.h"