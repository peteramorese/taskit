#pragma once

#include "ManipulatorNode.h"

#include "visualization_msgs/MarkerArray.h"

#include "Object.h"
#include "Tools.h"
#include "ManipulatorProperties.h"

namespace ManipulationInterface {

template <class...ACTION_PRIMITIVES_TYPES>
ManipulatorNode<ACTION_PRIMITIVES_TYPES...>::ManipulatorNode(const std::string& node_name, const std::string& planning_group, const std::string& frame_id, ACTION_PRIMITIVES_TYPES&&...action_primitives)
    : m_node_name(node_name)
    , m_action_primitives(std::forward<ACTION_PRIMITIVES_TYPES>(action_primitives)...)
    , m_move_group(std::make_shared<moveit::planning_interface::MoveGroupInterface>(planning_group))
    , m_planning_interface(std::make_shared<moveit::planning_interface::PlanningSceneInterface>())
    , m_frame_id(frame_id)
{

    //if constexpr (std::is_default_constructible_v<OBJ_GROUP_T>) ROS_ASSERT_MSG(obj_group, "Must provide an object group when using a non-default-constructable object group type");
    
    // Init ros items
    m_node_handle = std::make_unique<ros::NodeHandle>("~");
    m_spinner = std::make_unique<ros::AsyncSpinner>(2);
    m_spinner->start();

    // Init MoveIt items
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	m_robot_model = robot_model_loader.getModel();
    m_robot_state.reset(new robot_state::RobotState(m_robot_model));
    const robot_state::JointModelGroup* joint_model_group = m_robot_state->getJointModelGroup(planning_group);
    m_planning_scene = std::make_shared<planning_scene::PlanningScene>(m_robot_model);
    m_planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "ready");
    
    // Read in the manipulator properties
    ManipulatorProperties::load(*m_node_handle, planning_group, "arm_config");

    // Load planning plugin
    std::string planner_plugin_name;

    if (!m_node_handle->getParam("planning_plugin", planner_plugin_name))
        ROS_FATAL_STREAM("Could not find planner plugin name");
    try {
        m_planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
    } catch (pluginlib::PluginlibException& ex) {
        ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
    }
    try {
        m_planner_instance.reset(m_planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
        //m_planner_instance = m_planner_plugin_loader->createInstance(planner_plugin_name);
        if (!m_planner_instance->initialize(m_robot_model, m_node_handle->getNamespace()))
            ROS_FATAL_STREAM("Could not initialize planner instance");
        ROS_INFO_STREAM("Using planner '" << m_planner_instance->getDescription() << "'");
    } catch (pluginlib::PluginlibException& ex) {
        const std::vector<std::string>& classes = m_planner_plugin_loader->getDeclaredClasses();
        std::stringstream ss;
        for (std::size_t i = 0; i < classes.size(); ++i)
            ss << classes[i] << " ";
        ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl << "Available plugins: " << ss.str());
    }

    setPlanner("RRTConnectkConfigDefault");

    // Setup the marker publisher
    m_visualizer.reset(new ManipulatorNodeVisualizer(*m_node_handle, m_frame_id, "/rviz_visual_tools"));
}



template <class...ACTION_PRIMITIVES_TYPES>
void ManipulatorNode<ACTION_PRIMITIVES_TYPES...>::updateEnvironment(bool ignore_static) {
    auto attached_objects = m_planning_interface->getAttachedObjects();

    CollisionObjectVector collision_objects;
    collision_objects.reserve(m_obj_group->size());

    for (auto obj : m_obj_group->getObjects()) {
        const auto& id = obj->id;

        // Do not update if the object is attached
        auto it = attached_objects.find(id);
        if (it != attached_objects.end()) continue;

        // Do not update if the object is static
        if (ignore_static && obj->isStatic()) continue;

        obj->updatePose();
        moveit_msgs::CollisionObject col_obj = obj->getCollisionObject(m_frame_id);

        col_obj.operation = col_obj.ADD;

        collision_objects.push_back(std::move(col_obj));
    }

    m_planning_interface->applyCollisionObjects(collision_objects);

}

#define MI_GOAL_MARKER_R  58.0
#define MI_GOAL_MARKER_G  240.0
#define MI_GOAL_MARKER_B  221.0

void ManipulatorNodeVisualizer::publishGoalMarker(const geometry_msgs::Pose& pose, const std::string& msg, float scale) {
    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.resize(2);
    visualization_msgs::Marker& marker = marker_array.markers[0];
    marker.header.frame_id = m_frame_id;
    marker.header.stamp = ros::Time();
    marker.ns = "vis_marker_ns";
    marker.id = MarkerIds::GoalMarker;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = pose;
    marker.scale.x = scale * 0.03;
    marker.scale.y = scale * 0.03;
    marker.scale.z = scale * 0.03;
    marker.color.a = 1.0; 
    marker.color.r = MI_GOAL_MARKER_R / 255.0;
    marker.color.g = MI_GOAL_MARKER_G / 255.0;
    marker.color.b = MI_GOAL_MARKER_B / 255.0;

    visualization_msgs::Marker& text = marker_array.markers[1];

    text.header.frame_id = m_frame_id;
    text.header.stamp = ros::Time();
    text.ns = "vis_marker_ns";
    text.id = MarkerIds::GoalMarkerText;
    text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::Marker::ADD;
    text.pose = pose;
    text.pose.position.z += 0.04;
    text.scale.x = scale * 0.07;
    text.scale.y = scale * 0.07;
    text.scale.z = scale * 0.07;
    text.color.a = 1.0; 
    text.color.r = MI_GOAL_MARKER_R / 255.0;
    text.color.g = MI_GOAL_MARKER_G / 255.0;
    text.color.b = MI_GOAL_MARKER_B / 255.0;
    text.text = msg;

    m_visualization_marker_pub.publish(marker_array);
}

void ManipulatorNodeVisualizer::publishGoalObjectMarker(const geometry_msgs::Pose& obj_pose, const geometry_msgs::Pose& goal_pose, const std::string& msg, float scale, uint32_t num_points) {
    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.resize(3);
    
    // Goal sphere
    visualization_msgs::Marker& marker = marker_array.markers[0];

    marker.header.frame_id = m_frame_id;
    marker.header.stamp = ros::Time();
    marker.ns = "vis_marker_ns";
    marker.id = MarkerIds::GoalMarker;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = goal_pose;
    marker.scale.x = scale * 0.03;
    marker.scale.y = scale * 0.03;
    marker.scale.z = scale * 0.03;
    marker.color.a = 1.0; 
    marker.color.r = MI_GOAL_MARKER_R / 255.0;
    marker.color.g = MI_GOAL_MARKER_G / 255.0;
    marker.color.b = MI_GOAL_MARKER_B / 255.0;

    // Goal text
    visualization_msgs::Marker& text = marker_array.markers[1];

    text.header.frame_id = m_frame_id;
    text.header.stamp = ros::Time();
    text.ns = "vis_marker_ns";
    text.id = MarkerIds::GoalMarkerText;
    text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::Marker::ADD;
    text.pose = goal_pose;
    text.pose.position.z += 0.04;
    text.scale.x = scale * 0.07;
    text.scale.y = scale * 0.07;
    text.scale.z = scale * 0.07;
    text.color.a = 1.0; 
    text.color.r = MI_GOAL_MARKER_R / 255.0;
    text.color.g = MI_GOAL_MARKER_G / 255.0;
    text.color.b = MI_GOAL_MARKER_B / 255.0;
    text.text = msg;

    // Dotted line to object
    visualization_msgs::Marker& points = marker_array.markers[2];

    points.header.frame_id = m_frame_id;
    points.header.stamp = ros::Time();
    points.ns = "vis_marker_ns";
    points.id = MarkerIds::ToObjLine;
    points.type = visualization_msgs::Marker::POINTS;
    points.action = visualization_msgs::Marker::ADD;
    points.pose = goal_pose;
    points.pose.position.z += 0.04;
    points.scale.x = scale * 0.007;
    points.scale.y = scale * 0.007;
    points.scale.z = scale * 0.007;

    points.points.resize(num_points);
    points.colors.resize(num_points);

    tf2::Vector3 to_goal, to_obj_center;
    tf2::fromMsg(goal_pose.position, to_goal);
    tf2::fromMsg(obj_pose.position, to_obj_center);
    tf2::Vector3 along = to_obj_center - to_goal;
    DEBUG("along.x: " << along[0]);
    DEBUG("along.y: " << along[1]);
    DEBUG("along.z: " << along[2]);

    for (uint32_t i=0; i<num_points; ++i) {
        float scale = static_cast<float>(i + 1) / static_cast<float>(num_points);
        tf2::toMsg(scale * along, points.points[i]);
        points.colors[i].a = 1.0;
        points.colors[i].r = MI_GOAL_MARKER_R / 255.0;
        points.colors[i].g = (1.0f - scale) * MI_GOAL_MARKER_G / 255.0;
        points.colors[i].b = MI_GOAL_MARKER_B / 255.0;
    }

    m_visualization_marker_pub.publish(marker_array);
}
}