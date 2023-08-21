#pragma once

#include "visualization_msgs/MarkerArray.h"
#include <geometry_msgs/Pose.h>
#include <std_msgs/ColorRGBA.h>

#include "PredicateHandler.h"

namespace TaskIt {

class Visualizer {
    public:
        enum class MarkerType {
            Goal,
            Axes,
            Location,
        };
    public:
        Visualizer(ros::NodeHandle& nh, const std::string& frame_id, const std::string& topic = "/rviz_visual_tools") 
            : m_visualization_marker_pub(nh.advertise<visualization_msgs::MarkerArray>(topic, 0))
            , m_frame_id(frame_id)
        {}

        void addAxes(visualization_msgs::MarkerArray& marker_array, const geometry_msgs::Pose& pose, float scale, MarkerType type, bool invert_default_down = true);
        void addText(visualization_msgs::MarkerArray& marker_array, const geometry_msgs::Pose& pose, float scale, MarkerType type, const std::string& msg, const std_msgs::ColorRGBA& color);
        void addLocation(visualization_msgs::MarkerArray& marker_array, const geometry_msgs::Pose& pose, float scale, const std::string& name, float detection_radius, const std_msgs::ColorRGBA& color);
        void publishEEFGoalPose(const geometry_msgs::Pose& pose, const std::string& msg = "goal", float scale = 1.0f);
        void publishLocations(const PredicateHandler& predicate_handler, float scale = 1.0f);
        void remove(MarkerType marker_type);
        void removeAll();
    private:
        uint32_t newId(MarkerType marker_type) {
            m_type_to_ids[marker_type].push_back(m_last_id);
            return m_last_id++;
        }

    private:
        std::map<MarkerType, std::list<uint32_t>> m_type_to_ids;
        uint32_t m_last_id = 0;
        ros::Publisher m_visualization_marker_pub;
        std::string m_frame_id;
};

}