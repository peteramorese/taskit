#include "Visualizer.h"
#include "Quaternions.h"
#include "Config.h"

#define TASKIT_GOAL_MARKER_R  (58.0 / 255.0)
#define TASKIT_GOAL_MARKER_G  (240.0 / 255.0)
#define TASKIT_GOAL_MARKER_B  (221.0 / 255.0)

#define TASKIT_LOCATION_MARKER_R  (240.0 / 255.0)
#define TASKIT_LOCATION_MARKER_G  (31.0 / 255.0)
#define TASKIT_LOCATION_MARKER_B  (135.0 / 255.0)
#define TASKIT_DETECTION_RADIUS_ALPHA 0.4

namespace TaskIt {

void Visualizer::addAxes(visualization_msgs::MarkerArray& marker_array, const geometry_msgs::Pose& pose, float scale, MarkerType type, bool invert_default_down) {
    std::size_t ind_offset = marker_array.markers.size();
    marker_array.markers.resize(ind_offset + 3);

    tf2::Quaternion goal_orientation = Quaternions::convert(pose.orientation);
    tf2::Vector3 in_btw_gripper_offset(0.0, 0.0, -ManipulatorProperties::getEndEffectorOffset(TASKIT_PLANNING_GROUP_ID));

    // If invert default down, apply a rotation for the inverted default down quaternion
    if (invert_default_down)
        goal_orientation = goal_orientation * Quaternions::getDefaultDown(TASKIT_PLANNING_GROUP_ID, true);

    visualization_msgs::Marker& x_arrow = marker_array.markers[ind_offset + 0];
    x_arrow.header.frame_id = m_frame_id;
    x_arrow.header.stamp = ros::Time();
    x_arrow.ns = "vis_marker_ns";
    x_arrow.id = newId(type);
    x_arrow.type = visualization_msgs::Marker::ARROW;
    x_arrow.action = visualization_msgs::Marker::ADD;

    x_arrow.pose.position = pose.position;
    x_arrow.pose.orientation = Quaternions::convert(goal_orientation);
    Quaternions::translatePose(x_arrow.pose, in_btw_gripper_offset);

    x_arrow.scale.x = scale * 0.10;
    x_arrow.scale.y = scale * 0.01;
    x_arrow.scale.z = scale * 0.01;
    x_arrow.color.a = 1.0; 
    x_arrow.color.r = 1.0;
    x_arrow.color.g = 0.0;
    x_arrow.color.b = 0.0;

    visualization_msgs::Marker& y_arrow = marker_array.markers[ind_offset + 1];
    y_arrow.header.frame_id = m_frame_id;
    y_arrow.header.stamp = ros::Time();
    y_arrow.ns = "vis_marker_ns";
    y_arrow.id = newId(type);
    y_arrow.type = visualization_msgs::Marker::ARROW;
    y_arrow.action = visualization_msgs::Marker::ADD;
    y_arrow.pose.position = pose.position;

    y_arrow.pose.position = pose.position;
    y_arrow.pose.orientation = Quaternions::convert(goal_orientation);
    Quaternions::translatePose(y_arrow.pose, in_btw_gripper_offset);
    y_arrow.pose.orientation = Quaternions::convert(goal_orientation * Quaternions::getRotation(Quaternions::RotationType::Yaw90));

    y_arrow.scale.x = scale * 0.10;
    y_arrow.scale.y = scale * 0.01;
    y_arrow.scale.z = scale * 0.01;
    y_arrow.color.a = 1.0; 
    y_arrow.color.r = 0.0;
    y_arrow.color.g = 1.0;
    y_arrow.color.b = 0.0;

    visualization_msgs::Marker& z_arrow = marker_array.markers[ind_offset + 2];
    z_arrow.header.frame_id = m_frame_id;
    z_arrow.header.stamp = ros::Time();
    z_arrow.ns = "vis_marker_ns";
    z_arrow.id = newId(type);
    z_arrow.type = visualization_msgs::Marker::ARROW;
    z_arrow.action = visualization_msgs::Marker::ADD;
    z_arrow.pose.position = pose.position;

    z_arrow.pose.position = pose.position;
    z_arrow.pose.orientation = Quaternions::convert(goal_orientation);
    Quaternions::translatePose(z_arrow.pose, in_btw_gripper_offset);
    z_arrow.pose.orientation = Quaternions::convert(goal_orientation * Quaternions::getRotation(Quaternions::RotationType::Pitch270));

    z_arrow.scale.x = scale * 0.10;
    z_arrow.scale.y = scale * 0.01;
    z_arrow.scale.z = scale * 0.01;
    z_arrow.color.a = 1.0; 
    z_arrow.color.r = 0.0;
    z_arrow.color.g = 0.0;
    z_arrow.color.b = 1.0;
}

void Visualizer::addText(visualization_msgs::MarkerArray& marker_array, const geometry_msgs::Pose& pose, float scale, MarkerType type, const std::string& msg, const std_msgs::ColorRGBA& color) {
    visualization_msgs::Marker text;

    text.header.frame_id = m_frame_id;
    text.header.stamp = ros::Time();
    text.ns = "vis_marker_ns";
    text.id = newId(type);
    text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::Marker::ADD;
    text.pose = pose;
    text.pose.position.z -= 0.04;
    text.scale.x = scale * 0.07;
    text.scale.y = scale * 0.07;
    text.scale.z = scale * 0.07;
    text.color = color;
    text.text = msg;
    marker_array.markers.push_back(std::move(text));
}

void Visualizer::addLocation(visualization_msgs::MarkerArray& marker_array, const geometry_msgs::Pose& pose, float scale, const std::string& name, float detection_radius, const std_msgs::ColorRGBA& color) {
    std::size_t ind_offset = marker_array.markers.size();
    marker_array.markers.resize(ind_offset + 2);

    // Sphere for center
    visualization_msgs::Marker& center = marker_array.markers[ind_offset + 0];
    center.header.frame_id = m_frame_id;
    center.header.stamp = ros::Time();
    center.ns = "vis_marker_ns";
    center.id = newId(MarkerType::Location);
    center.type = visualization_msgs::Marker::SPHERE;
    center.action = visualization_msgs::Marker::ADD;
    center.pose = pose;
    center.scale.x = scale * 0.03;
    center.scale.y = scale * 0.03;
    center.scale.z = scale * 0.03;
    center.color = color;

    visualization_msgs::Marker& detection_zone = marker_array.markers[ind_offset + 1];
    detection_zone.header.frame_id = m_frame_id;
    detection_zone.header.stamp = ros::Time();
    detection_zone.ns = "vis_marker_ns";
    detection_zone.id = newId(MarkerType::Location);
    detection_zone.type = visualization_msgs::Marker::SPHERE;
    detection_zone.action = visualization_msgs::Marker::ADD;
    detection_zone.pose = pose;
    detection_zone.scale.x = detection_radius;
    detection_zone.scale.y = detection_radius;
    detection_zone.scale.z = detection_radius;
    detection_zone.color = color;
    detection_zone.color.a = TASKIT_DETECTION_RADIUS_ALPHA;

    addText(marker_array, pose, scale, MarkerType::Location, name, color);

}

void Visualizer::publishEEFGoalPose(const geometry_msgs::Pose& pose, const std::string& msg, float scale) {
    visualization_msgs::MarkerArray marker_array;
    addAxes(marker_array, pose, scale, MarkerType::Goal, true);
    
    std_msgs::ColorRGBA color;
    color.r = TASKIT_GOAL_MARKER_R;
    color.g = TASKIT_GOAL_MARKER_G;
    color.b = TASKIT_GOAL_MARKER_B;
    color.a = 1.0;
    addText(marker_array, pose, scale, MarkerType::Goal, msg, color);
    m_visualization_marker_pub.publish(marker_array);
}

void Visualizer::publishLocations(const PredicateHandler& predicate_handler, float scale) {

    const std::map<std::string, PredicateHandler::Location>& locations = predicate_handler.getLocations();

    visualization_msgs::MarkerArray marker_array;

    for (auto locations_it = locations.begin(); locations_it != locations.end(); ++locations_it) {

        const PredicateHandler::Location& location = locations_it->second;
        std_msgs::ColorRGBA color;
        color.r = TASKIT_LOCATION_MARKER_R;
        color.g = TASKIT_LOCATION_MARKER_G;
        color.b = TASKIT_LOCATION_MARKER_B;
        color.a = 1.0;
        addLocation(marker_array, location.defaultPose(), scale, locations_it->first, location.detectionRadius(), color);
    }

    m_visualization_marker_pub.publish(marker_array);
}

void Visualizer::remove(MarkerType marker_type) {
    std::list<uint32_t>& ids_to_erase = m_type_to_ids[marker_type];

    if (ids_to_erase.empty())
        return;

    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.reserve(ids_to_erase.size());
    for (auto it = ids_to_erase.begin(); it != ids_to_erase.end();) {
        visualization_msgs::Marker marker;
        marker.action = visualization_msgs::Marker::DELETE;
        marker.ns = "vis_marker_ns";
        marker.id = *it;
        marker_array.markers.push_back(std::move(marker));
        ids_to_erase.erase(it++);
    }
    m_visualization_marker_pub.publish(marker_array);
}

void Visualizer::removeAll() {
    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.resize(1);
    marker_array.markers[0].ns = "vis_marker_ns";
    marker_array.markers[0].action = visualization_msgs::Marker::DELETEALL;
    m_visualization_marker_pub.publish(marker_array);
}

}