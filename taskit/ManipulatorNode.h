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
#include "PredicateHandler.h"
#include "Gripper.h"
#include "Visualizer.h"

namespace TaskIt {
    
struct ManipulatorNodeState {
    public:
        std::string curr_location_name = std::string();
        bool near_object = false;
        Quaternions::RotationType grasp_rotation_type;
        double placing_offset = 0.0;
    public:
        ManipulatorNodeState() {reset();}
        void reset() {
            curr_location_name = std::string();
            near_object = false;
            grasp_rotation_type = Quaternions::RotationType::None;
            placing_offset = 0.0;
        }
};


template <class...ACTION_PRIMITIVES_TYPES>
class ManipulatorNode {
    public:
        ManipulatorNode(const std::shared_ptr<ros::NodeHandle>& node_handle, const std::string& node_name, const std::string& planning_group, const std::string& planning_frame_id, ACTION_PRIMITIVES_TYPES&&...action_primitives);

        ~ManipulatorNode() {
            m_visualizer->removeAllMarkers();
        }

        // Get the number of action primitives
        static constexpr uint32_t numActionPrimitives() {return sizeof...(ACTION_PRIMITIVES_TYPES);}

        // Get the async spinner created on construction
        const std::shared_ptr<ros::AsyncSpinner>& getSpinner() {return m_spinner;}

        // Set the motion planner ID. Default is RRTConnectkConfigDefault
        inline void setPlanner(const std::string& planner_id) { m_move_group->setPlannerId(planner_id); }

        // Set a custom end effector link.
        inline void setEndEffectorLink(const std::string& ee_link) { m_move_group->setEndEffectorLink(ee_link); }

        // Visualize the goal markers. True by default
        inline void toggleAutoVisualize(bool auto_visualize) {m_auto_visualize = auto_visualize;}

        //// Insert an object group that was manually created
        //void insertObjectGroup(const std::shared_ptr<ObjectGroup>& obj_group) {m_obj_group = obj_group;}

        // Automatically create the objects based off the parameter server (workspace_ns: param namespace for static obstacles, objects_ns: namespace for dynamic objects and predicates)
        void createScene(const std::shared_ptr<PoseTracker>& pose_tracker, const std::string& environment_ns = "environment", const std::string& workspace_ns = "workspace", const std::string& objects_ns = "objects") {
            m_obj_group.reset(new ObjectGroup);
            m_obj_group->createObjects(*m_node_handle, workspace_ns);
            m_obj_group->createObjects(*m_node_handle, objects_ns, pose_tracker);

            m_predicate_handler.reset(new PredicateHandler(m_obj_group));
            m_predicate_handler->createEnvironment(*m_node_handle, environment_ns);
            m_predicate_handler->setObjectPosesToLocations(*m_node_handle, objects_ns);
            updatePlanningScene(false);
            if (m_auto_visualize) m_visualizer->publishLocationMarkers(*m_predicate_handler);
        }

        // Update all of the object poses, and update the collision objects in the planning scene
        bool updatePlanningScene(bool ignore_static = true);

        // Reference node handle.
        inline ros::NodeHandle& getNodeHandle() {return *m_node_handle;}
        inline const ros::NodeHandle& getNodeHandle() const {return *m_node_handle;}

        // Spawn services for action primitives
        inline void spawnAllActionServices() {
            spawnActionServices<0, numActionPrimitives()>();
        }

        // Spawn a specific set of action primitives. INIT_ACTION_INDEX is the starting template argument
        // index, and FINAL_ACTION_INDEX is the ending template argument index
        template <uint32_t INIT_ACTION_INDEX, uint32_t FINAL_ACTION_INDEX = INIT_ACTION_INDEX + 1>
        void spawnActionServices() {
            static_assert(FINAL_ACTION_INDEX <= numActionPrimitives(), "Final action index out of range");
            if constexpr (INIT_ACTION_INDEX < FINAL_ACTION_INDEX) {
                auto action = std::get<INIT_ACTION_INDEX>(m_action_primitives);
                ROS_INFO_STREAM_NAMED(m_node_name, "Spawning action service on topic: " << "action_primitive/" + action.topic());
                m_action_services[INIT_ACTION_INDEX] = m_node_handle->advertiseService("action_primitive/" + action.topic(), &ManipulatorNode::actionServiceCallback<INIT_ACTION_INDEX>, this);
                spawnActionServices<INIT_ACTION_INDEX + 1, FINAL_ACTION_INDEX>();
            }
        }

        // Specify a function that is called before an action is executed
        void setBeforeActionCallback(std::function<void()> beforeActionCall) {
            m_beforeActionCall = [this, beforeActionCall]() -> void {
                if (m_auto_visualize) m_visualizer->removeMarkers(Visualizer::MarkerType::Goal);
                beforeActionCall();
            };
        }

        // Specify a function that is called after an action has finished
        void setAfterActionCallback(std::function<void()> afterActionCall) {
            m_afterActionCall = [this, afterActionCall]() -> void {
                afterActionCall();
            };
        }

        // Generic call types
        template <class ACTION_PRIMITIVE_T, typename...ARGS_T>
        bool callActionByType(ARGS_T&&...args) {
            if (m_beforeActionCall) m_beforeActionCall();
            bool ret = std::get<getIndexByType<0, ACTION_PRIMITIVE_T>()>(m_action_primitives)(getInterface(), std::forward<ARGS_T>(args)...);
            if (m_afterActionCall) m_afterActionCall();
            return ret;
        }

        template <class ACTION_PRIMITIVE_T, typename...ARGS_T>
        bool callActionByType(ARGS_T&&...args) const {
            if (m_beforeActionCall) m_beforeActionCall();
            bool ret = std::get<getIndexByType<0, ACTION_PRIMITIVE_T>()>(m_action_primitives)(getInterface(), std::forward<ARGS_T>(args)...);
            if (m_afterActionCall) m_afterActionCall();
            return ret;
        }

        template <uint32_t I, typename...ARGS_T>
        bool callActionByIndex(ARGS_T&&...args) {
            if (m_beforeActionCall) m_beforeActionCall();
            bool ret = std::get<I>(m_action_primitives)(getInterface(), std::forward<ARGS_T>(args)...);
            if (m_afterActionCall) m_afterActionCall();
            return ret;
        }

        template <uint32_t I, typename...ARGS_T>
        bool callActionByIndex(ARGS_T&&...args) const {
            if (m_beforeActionCall) m_beforeActionCall();
            bool ret = std::get<I>(m_action_primitives)(getInterface(), std::forward<ARGS_T>(args)...);
            if (m_afterActionCall) m_afterActionCall();
            return ret;
        }

        // Access important components
        ManipulatorNodeInterface getInterface() {return ManipulatorNodeInterface(m_move_group, m_planning_scene_interface, m_obj_group, m_predicate_handler, m_visualizer, m_state);}
        ConstManipulatorNodeInterface getInterface() const {return ConstManipulatorNodeInterface(m_move_group, m_planning_scene_interface, m_obj_group, m_predicate_handler, m_visualizer, m_state);}

    private:


        template <uint32_t I>
        inline bool actionServiceCallback(
            typename std::tuple_element_t<I, std::tuple<ACTION_PRIMITIVES_TYPES...>>::msg_t::Request& request, 
            typename std::tuple_element_t<I, std::tuple<ACTION_PRIMITIVES_TYPES...>>::msg_t::Response& response) {
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

        std::shared_ptr<ros::NodeHandle> m_node_handle;
        std::shared_ptr<ros::AsyncSpinner> m_spinner;

        std::tuple<ACTION_PRIMITIVES_TYPES...> m_action_primitives;
        std::array<ros::ServiceServer, numActionPrimitives()> m_action_services;

        // Interface classes
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> m_move_group;
        std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> m_planning_scene_interface;
        std::shared_ptr<ObjectGroup> m_obj_group;
        std::shared_ptr<PredicateHandler> m_predicate_handler;
        std::shared_ptr<Visualizer> m_visualizer;
        std::shared_ptr<ManipulatorNodeState> m_state;

        std::shared_ptr<planning_scene::PlanningScene> m_planning_scene;

        // Things that just need to exist
        robot_model::RobotModelPtr m_robot_model;
        robot_state::RobotStatePtr m_robot_state;
        boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> m_planner_plugin_loader;
        planning_interface::PlannerManagerPtr m_planner_instance;

        // Action Callbacks
        std::function<void()> m_beforeActionCall;
        std::function<void()> m_afterActionCall;

        bool m_auto_visualize;

};

}

#include "ManipulatorNode_impl.h"