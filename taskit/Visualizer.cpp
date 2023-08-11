#include "Visualizer.h"

#define MI_GOAL_MARKER_R  58.0
#define MI_GOAL_MARKER_G  240.0
#define MI_GOAL_MARKER_B  221.0

#define MI_LOCATION_MARKER_R  240.0
#define MI_LOCATION_MARKER_G  31.0
#define MI_LOCATION_MARKER_B  135.0
#define MI_DETECTION_RADIUS_ALPHA 0.4

namespace TaskIt {

void Visualizer::publishGoalMarker(const geometry_msgs::Pose& pose, const std::string& msg, float scale, bool keep_existing) {
    if (!keep_existing) removeMarkers(MarkerType::Goal);

    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.resize(2);
    visualization_msgs::Marker& marker = marker_array.markers[0];
    marker.header.frame_id = m_frame_id;
    marker.header.stamp = ros::Time();
    marker.ns = "vis_marker_ns";
    marker.id = newId(MarkerType::Goal);
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
    text.id = newId(MarkerType::Goal);
    text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::Marker::ADD;
    text.pose = pose;
    text.pose.position.z -= 0.04;
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

void Visualizer::publishGoalObjectMarker(const geometry_msgs::Pose& obj_pose, const geometry_msgs::Pose& goal_pose, const std::string& msg, float scale, uint32_t num_points, bool keep_existing) {
    if (!keep_existing) removeMarkers(MarkerType::Goal);

    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.resize(3);
    
    publishGoalMarker(goal_pose, msg, scale, keep_existing);

    // Dotted line to object
    visualization_msgs::Marker& points = marker_array.markers[2];

    points.header.frame_id = m_frame_id;
    points.header.stamp = ros::Time();
    points.ns = "vis_marker_ns";
    points.id = newId(MarkerType::Goal);
    points.type = visualization_msgs::Marker::POINTS;
    points.action = visualization_msgs::Marker::ADD;
    points.pose = geometry_msgs::Pose{};
    points.scale.x = scale * 0.007;
    points.scale.y = scale * 0.007;
    points.scale.z = scale * 0.007;

    points.points.resize(num_points);
    points.colors.resize(num_points);

    tf2::Vector3 to_goal, to_obj_center;
    tf2::fromMsg(goal_pose.position, to_goal);
    tf2::fromMsg(obj_pose.position, to_obj_center);
    tf2::Vector3 along = to_obj_center - to_goal;

    for (uint32_t i=0; i<num_points; ++i) {
        float scale = static_cast<float>(i + 1) / static_cast<float>(num_points);
        tf2::toMsg(to_goal + scale * along, points.points[i]);
        points.colors[i].a = 1.0;
        points.colors[i].r = MI_GOAL_MARKER_R / 255.0;
        points.colors[i].g = (1.0f - scale) * MI_GOAL_MARKER_G / 255.0;
        points.colors[i].b = MI_GOAL_MARKER_B / 255.0;
    }

    m_visualization_marker_pub.publish(marker_array);
}

void Visualizer::publishLocationMarkers(const PredicateHandler& predicate_handler, float scale) {

    const std::map<std::string, PredicateHandler::Location>& locations = predicate_handler.getLocations();

    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.resize(3 * locations.size());

    auto markers_it = marker_array.markers.begin();
    for (auto locations_it = locations.begin(); locations_it != locations.end(); ++locations_it) {

        const PredicateHandler::Location& location = locations_it->second;

        // Sphere for center
        visualization_msgs::Marker& center = *markers_it++;
        center.header.frame_id = m_frame_id;
        center.header.stamp = ros::Time();
        center.ns = "vis_marker_ns";
        center.id = newId(MarkerType::Location);
        center.type = visualization_msgs::Marker::SPHERE;
        center.action = visualization_msgs::Marker::ADD;
        center.pose = location.defaultPose();
        center.scale.x = scale * 0.03;
        center.scale.y = scale * 0.03;
        center.scale.z = scale * 0.03;
        center.color.a = 1.0; 
        center.color.r = MI_LOCATION_MARKER_R / 255.0;
        center.color.g = MI_LOCATION_MARKER_G / 255.0;
        center.color.b = MI_LOCATION_MARKER_B / 255.0;

        visualization_msgs::Marker& detection_zone = *markers_it++;
        detection_zone.header.frame_id = m_frame_id;
        detection_zone.header.stamp = ros::Time();
        detection_zone.ns = "vis_marker_ns";
        detection_zone.id = newId(MarkerType::Location);
        detection_zone.type = visualization_msgs::Marker::SPHERE;
        detection_zone.action = visualization_msgs::Marker::ADD;
        detection_zone.pose = location.defaultPose();
        detection_zone.scale.x = location.detectionRadius();
        detection_zone.scale.y = location.detectionRadius();
        detection_zone.scale.z = location.detectionRadius();
        detection_zone.color.a = MI_DETECTION_RADIUS_ALPHA;
        detection_zone.color.r = MI_LOCATION_MARKER_R / 255.0;
        detection_zone.color.g = MI_LOCATION_MARKER_G / 255.0;
        detection_zone.color.b = MI_LOCATION_MARKER_B / 255.0;

        visualization_msgs::Marker& text = *markers_it++;
        text.header.frame_id = m_frame_id;
        text.header.stamp = ros::Time();
        text.ns = "vis_marker_ns";
        text.id = newId(MarkerType::Location);
        text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::Marker::ADD;
        text.pose = location.defaultPose();
        text.pose.position.z += 0.04;
        text.scale.x = scale * 0.07;
        text.scale.y = scale * 0.07;
        text.scale.z = scale * 0.07;
        text.color.a = 1.0; 
        text.color.r = MI_LOCATION_MARKER_R / 255.0;
        text.color.g = MI_LOCATION_MARKER_G / 255.0;
        text.color.b = MI_LOCATION_MARKER_B / 255.0;
        text.text = locations_it->first;
    }

    m_visualization_marker_pub.publish(marker_array);
}

void Visualizer::removeMarkers(MarkerType marker_type) {
    const std::vector<uint32_t>& ids_to_erase = m_type_to_ids[marker_type];
    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.reserve(ids_to_erase.size());
    for (uint32_t id : ids_to_erase) {
        visualization_msgs::Marker marker;
        marker.action = visualization_msgs::Marker::DELETE;
        marker.ns = "vis_marker_ns";
        marker.id = id;
        marker_array.markers.push_back(std::move(marker));
    }
    m_visualization_marker_pub.publish(marker_array);
}

void Visualizer::removeAllMarkers() {
    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.resize(1);
    marker_array.markers[0].ns = "vis_marker_ns";
    marker_array.markers[0].action = visualization_msgs::Marker::DELETEALL;
    m_visualization_marker_pub.publish(marker_array);
}

}