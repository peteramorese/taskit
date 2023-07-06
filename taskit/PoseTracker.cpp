#include "PoseTracker.h"

#include "Object.h"

namespace TaskIt {

VRPNPoseTracker::VRPNPoseTracker(const std::shared_ptr<ros::NodeHandle>& node_handle, double sampling_duration)
    : m_node_handle(node_handle)
    , m_sampling_duration(sampling_duration)
{
    m_vrpn_client_topic = m_node_handle->param<std::string>(getParamName("vrpn_client_topic"), "/vrpn_client_node/");
}

VRPNPoseTracker::VRPNPoseTracker(const std::shared_ptr<ros::NodeHandle>& node_handle, const std::string& vrpn_client_topic, double sampling_duration)
    : m_node_handle(node_handle)
    , m_vrpn_client_topic(vrpn_client_topic)
    , m_sampling_duration(sampling_duration)
{}

void VRPNPoseTracker::update(Object& object) const {

    ROS_INFO_STREAM("Updating pose for object: " << object.id);
    std::string topic = m_vrpn_client_topic + object.id + "/pose";

    tf2::Vector3 position(0.0f, 0.0f, 0.0f);
    tf2::Quaternion orientation(0.0f, 0.0f, 0.0f, 0.0f);

    uint32_t n_msgs = 0;
    auto onMsg = [&](const geometry_msgs::PoseStamped::ConstPtr& msg) -> void {
        tf2::Vector3 msg_position;
        tf2::Quaternion msg_orientation;
        tf2::fromMsg(msg->pose.position, msg_position);
        tf2::fromMsg(msg->pose.orientation, msg_orientation);
        position += msg_position;
        orientation += msg_orientation;
        ++n_msgs;
    };

    // Create subscriber in scope so that the subscriber is destroyed after the duration
    ros::WallDuration duration(m_sampling_duration);
    for (uint16_t i = 0; i < 5; ++i) {
        ros::Subscriber pose_sub = m_node_handle->subscribe<geometry_msgs::PoseStamped>(topic, 10, onMsg);
        duration.sleep();
        if (position.length() == 0.0f)
            ROS_WARN_STREAM("VRPN Data for object '" << object.id.c_str() << "' may not be found on topic, retrying (" << i + 1 << "/5 attempts)...");
        else {
            break;
        }
    }

    position /= static_cast<float>(n_msgs);
    orientation /= static_cast<float>(n_msgs);
    orientation.normalize();

    tf2::toMsg(position, object.pose.position);
    tf2::convert(orientation, object.pose.orientation);
}

}