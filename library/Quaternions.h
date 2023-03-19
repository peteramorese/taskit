#pragma once

#include <string>

#include <ros/ros.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Quaternion.h>

#include "Tools.h"
#include "ManipulatorProperties.h"

namespace ManipulationInterface {
    class Quaternions {
        public:
            enum class Type {
                None,
                UpX,
                UpY,
                SideX,
                SideY
            };

            enum class RotationType {
                None,
                DownAxis,       // zero radians
                UpAxis,         // pi radians
            };

        public:

            static inline Quaternions::Type toType(const std::string& type_str) {
                if (type_str == "up_x" || type_str == "UpX") {
                    return Type::UpX;
                } else if (type_str == "up_y" || type_str == "UpY") {
                    return Type::UpY;
                } else if (type_str == "side_x" || type_str == "SideX") {
                    return Type::SideX;
                } else if (type_str == "side_y" || type_str == "SideY") {
                    return Type::SideY;
                }
                ROS_ERROR_STREAM("Unrecognized quaternion type '" << type_str << "'");
                return Type::None;
            }

            static inline geometry_msgs::Quaternion convert(const tf2::Quaternion q) {
                //geometry_msgs::Quaternion q_converted;
                return tf2::toMsg(q);
            }

            static inline tf2::Quaternion getDefaultUp() {
                return tf2::Quaternion(0.0f, 0.0f, 1.0f, 0.0f);
            }

            static tf2::Quaternion get(Type type) {
                tf2::Quaternion q_default_up = getDefaultUp();
                tf2::Quaternion q_rot;
                switch (type) {
                    case Type::UpX:         { q_rot.setRPY(0.0f, 0.0f, 0.0f); break; }
                    case Type::UpY:         { q_rot.setRPY(0.0f, 0.0f, M_PI/2.0f); break; }
                    case Type::SideX:       { q_rot.setRPY(0.0f, -M_PI/2.0f, 0.0f); break; }
                    case Type::SideY:       { q_rot.setRPY(-M_PI/2.0f, M_PI/2.0f, 0.0f); break; }
                    default:                { q_rot.setRPY(0.0f, 0.0f, 0.0f); }
                }
                return q_rot * q_default_up;
            }

            static inline tf2::Quaternion get(const std::string& type_str) {
                return get(toType(type_str));
            }

            static tf2::Quaternion getRotation(RotationType rotation_type) {
                tf2::Quaternion q_rot;
                switch (rotation_type) {
                    case RotationType::DownAxis:    { q_rot.setRPY(0.0f, 0.0f, 0.0f); break; }
                    case RotationType::UpAxis:      { q_rot.setRPY(0.0f, M_PI, 0.0f); break; }
                    default:                        { q_rot.setRPY(0.0f, 0.0f, 0.0f); }
                }
                return q_rot;
            }

            static tf2::Quaternion getDefaultDown(const std::string& planning_group) {
                tf2::Quaternion to_default_down;
                ROS_ASSERT_MSG(s_default_down.find(planning_group) != s_default_down.end(), "Default down quaternion not found for planning group");
                auto planning_group_properties = ManipulatorProperties::getDefaultDownRPY(planning_group);
                to_default_down.setRPY(
                    planning_group_properties.at("roll"), 
                    planning_group_properties.at("pitch"), 
                    planning_group_properties.at("yaw")
                );
                return to_default_down;
            }

            static geometry_msgs::Pose getPointAlongPose(const std::string& planning_group, const tf2::Vector3& relative_displacement_vector, const geometry_msgs::Pose& pose_to_match, RotationType rotation_type) {
                geometry_msgs::Pose pose;
                tf2::Quaternion default_down = getDefaultDown(planning_group);
                tf2::Quaternion q_to_match;
                tf2::fromMsg(pose_to_match.orientation, q_to_match);
                tf2::Quaternion orientation = getRotation(rotation_type) * q_to_match * default_down;
                orientation.normalize();
                tf2::Vector3 disp_rotated = tf2::quatRotate(orientation, relative_displacement_vector);
                DEBUG("disp_rotated[0]: " << disp_rotated[0]);
                DEBUG("disp_rotated[1]: " << disp_rotated[1]);
                DEBUG("disp_rotated[2]: " << disp_rotated[2]);
                pose.position.x = pose_to_match.position.x - disp_rotated[0]; 
                pose.position.y = pose_to_match.position.y - disp_rotated[1];
                pose.position.z = pose_to_match.position.z - disp_rotated[2];
                pose.orientation = convert(orientation);
                return pose;
            }


    };
}