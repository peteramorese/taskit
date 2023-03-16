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
#include "ManipulatorNodeInterface.h"
#include "Quaternions.h"
#include "Tools.h"
#include "PoseTracker.h"
#include "Object.h"

namespace ManipulationInterface {

    template <class...ACTION_PRIMITIVES_TYPES>
    class ManipulatorNode {
        public:
            ManipulatorNode(const std::string& node_name, const std::string& planning_group, const std::string& frame_id, ACTION_PRIMITIVES_TYPES&&...action_primitives);
            ~ManipulatorNode() {DEBUG("in dtor");}

            static constexpr uint32_t numActionPrimitives() {return sizeof...(ACTION_PRIMITIVES_TYPES);}

            inline void setPlanner(const std::string& planner_id) { m_move_group->setPlannerId(planner_id); }

            //// Insert an object group that was manually created
            //void insertObjectGroup(const std::shared_ptr<ObjectGroup>& obj_group) {m_obj_group = obj_group;}

            // Automatically create the objects based off the parameter server (workspace_ns: param namespace for static obstacles, objects_ns: namespace for dynamic objects and predicates)
            const std::shared_ptr<ObjectGroup> createScene(const std::string& workspace_ns = "workspace", const std::string& objects_ns = "objects", const std::shared_ptr<PoseTracker>& environment_pose_tracker = nullptr) {
                m_obj_group.reset(new ObjectGroup);
                createObjects(workspace_ns);
                createObjects(objects_ns, environment_pose_tracker);
                return m_obj_group;
            }

            void createObjects(const std::string& objects_ns, const std::shared_ptr<PoseTracker>& pose_tracker = nullptr);
            void updateEnvironment();
            //TODO: void applyWorkspace(const std::string& domain);

            void setEndEffectorLink(const std::string& ee_link) { m_move_group->setEndEffectorLink(ee_link); }

            ros::NodeHandle& getNodeHandle() {return *m_node_handle;}
            const ros::NodeHandle& getNodeHandle() const {return *m_node_handle;}

            // Spawn services for action primitives
            void spawnAllActionServices() {
                spawnActionServices<0, numActionPrimitives()>();
            }

            template <uint32_t INIT_ACTION_INDEX, uint32_t FINAL_ACTION_INDEX = INIT_ACTION_INDEX + 1>
            void spawnActionServices() {
                static_assert(FINAL_ACTION_INDEX <= numActionPrimitives(), "Final action index out of range");
                if constexpr (INIT_ACTION_INDEX < FINAL_ACTION_INDEX) {
                    auto action = std::get<INIT_ACTION_INDEX>(m_action_primitives);
                    DEBUG("spawing service on topic: " << action.topic());
                    m_action_services[INIT_ACTION_INDEX] = m_node_handle->advertiseService(action.topic(), &ManipulatorNode::actionServiceCallback<INIT_ACTION_INDEX>, this);
                    spawnActionServices<INIT_ACTION_INDEX + 1, FINAL_ACTION_INDEX>();
                }
            }

            // Generic call types
            template <class ACTION_PRIMITIVE_T, typename...ARGS_T>
            bool callActionByType(ARGS_T&&...args) {
                return std::get<getIndexByType<0, ACTION_PRIMITIVE_T>()>(m_action_primitives)(getInterface(), std::forward<ARGS_T>(args)...);
            }

            template <class ACTION_PRIMITIVE_T, typename...ARGS_T>
            bool callActionByType(ARGS_T&&...args) const {
                return std::get<getIndexByType<0, ACTION_PRIMITIVE_T>()>(m_action_primitives)(getInterface(), std::forward<ARGS_T>(args)...);
            }

            template <uint32_t I, typename...ARGS_T>
            bool callActionByIndex(ARGS_T&&...args) {
                return std::get<I>(m_action_primitives)(getInterface(), std::forward<ARGS_T>(args)...);
            }

            template <uint32_t I, typename...ARGS_T>
            bool callActionByIndex(ARGS_T&&...args) const {
                return std::get<I>(m_action_primitives)(getInterface(), std::forward<ARGS_T>(args)...);
            }

            // Access important
            ManipulatorNodeInterface getInterface() {return ManipulatorNodeInterface(m_move_group, m_planning_interface, m_obj_group);}
            ConstManipulatorNodeInterface getInterface() const {return ConstManipulatorNodeInterface(m_move_group, m_planning_interface, m_obj_group);}

        private:

            template <uint32_t I>
            inline bool actionServiceCallback(
                std::tuple_element_t<I, std::tuple<ACTION_PRIMITIVES_TYPES...>>::msg_t::Request& request, 
                std::tuple_element_t<I, std::tuple<ACTION_PRIMITIVES_TYPES...>>::msg_t::Response& response) {
                return callActionByIndex<I>(request, response);
            }


            template <uint32_t I, class ACTION_PRIMITIVE_T>
            static constexpr uint32_t getIndexByType() {
                if constexpr (I < (sizeof...(ACTION_PRIMITIVES_TYPES))) {
                    if constexpr (std::is_same<std::tuple_element_t<I, std::tuple<ACTION_PRIMITIVES_TYPES...>>, ACTION_PRIMITIVE_T>::value) {
                        // Calls operator () when the type is found
                        return I;
                    } else {
                        // Not found, continue searching
                        return getIndexByType<I + 1, ACTION_PRIMITIVE_T>();
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
            std::array<ros::ServiceServer, numActionPrimitives()> m_action_services;

            std::shared_ptr<ObjectGroup> m_obj_group;
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

    };
}

#include "ManipulatorNode_impl.h"