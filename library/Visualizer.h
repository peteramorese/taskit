#pragma once

#include "visualization_msgs/MarkerArray.h"
#include <geometry_msgs/Pose.h>

#include "PredicateHandler.h"

namespace ManipulationInterface {

class Visualizer {
    public:
        enum class MarkerType {
            Goal,
            Location,
        };
    public:
        Visualizer(ros::NodeHandle& nh, const std::string& frame_id, const std::string& topic = "/rviz_visual_tools") 
            : m_visualization_marker_pub(nh.advertise<visualization_msgs::MarkerArray>(topic, 0))
            , m_frame_id(frame_id)
        {}

        void publishGoalMarker(const geometry_msgs::Pose& pose, const std::string& msg = "goal", float scale = 1.0f, bool keep_existing = false);
        void publishGoalObjectMarker(const geometry_msgs::Pose& obj_pose, const geometry_msgs::Pose& goal_pose, const std::string& msg = "goal", float scale = 1.0f, uint32_t num_points = 20, bool keep_existing = false);
        void publishLocationMarkers(const PredicateHandler& predicate_handler, float scale = 1.0f);
        void removeMarkers(MarkerType marker_type);
        void removeAllMarkers();
    private:
        uint32_t newId(MarkerType marker_type) {
            m_type_to_ids[marker_type].push_back(m_last_id);
            return m_last_id++;
        }

    private:
        std::map<MarkerType, std::vector<uint32_t>> m_type_to_ids;
        uint32_t m_last_id = 0;
        ros::Publisher m_visualization_marker_pub;
        std::string m_frame_id;
};

}