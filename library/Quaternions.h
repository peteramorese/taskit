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
                Pitch90,
                Pitch180,
                Pitch270,
                Yaw90,
                Yaw180,
                Yaw270,
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

            static inline tf2::Quaternion convert(const geometry_msgs::Quaternion q) {
                tf2::Quaternion q_converted;
                tf2::fromMsg(q, q_converted);
                return q_converted;
            }

            static inline tf2::Quaternion getDefaultUp() {
                return tf2::Quaternion(0.0, 0.0, 1.0, 0.0);
            }

            static tf2::Quaternion get(Type type) {
                tf2::Quaternion q_default_up = getDefaultUp();
                tf2::Quaternion q_rot;
                switch (type) {
                    case Type::UpX:         { q_rot.setRPY(0.0, 0.0, 0.0); break; }
                    case Type::UpY:         { q_rot.setRPY(0.0, 0.0, M_PI/2.0); break; }
                    case Type::SideX:       { q_rot.setRPY(0.0, -M_PI/2.0, 0.0); break; }
                    case Type::SideY:       { q_rot.setRPY(-M_PI/2.0, M_PI/2.0, 0.0); break; }
                    default:                { q_rot.setRPY(0.0, 0.0, 0.0); }
                }
                return q_rot * q_default_up;
            }

            static inline tf2::Quaternion get(const std::string& type_str) {
                return get(toType(type_str));
            }

            static tf2::Quaternion getRotation(RotationType rotation_type) {
                tf2::Quaternion q_rot;
                switch (rotation_type) {
                    case RotationType::Pitch90:     { q_rot.setRPY(0.0, M_PI/2.0, 0.0); break;}
                    case RotationType::Pitch180:    { q_rot.setRPY(0.0, M_PI, 0.0); break;}
                    case RotationType::Pitch270:    { q_rot.setRPY(0.0, 3.0*M_PI/2.0, 0.0); break;}
                    case RotationType::Yaw90:       { q_rot.setRPY(0.0, 0.0, M_PI/2.0); break;}
                    case RotationType::Yaw180:      { q_rot.setRPY(0.0, 0.0, M_PI); break;}
                    case RotationType::Yaw270:      { q_rot.setRPY(0.0, 0.0, 3.0*M_PI/2.0); break;}
                    case RotationType::None:        { q_rot.setRPY(0.0, 0.0, 0.0); break;}
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

            static tf2::Vector3 getEndEffectorHeading(const tf2::Quaternion& eef_orientation) {
                tf2::Vector3 up(0.0, 0.0, 1.0);
                return tf2::quatRotate(eef_orientation, up);
            }

            static geometry_msgs::Pose getPointAlongPose(const std::string& planning_group, const tf2::Vector3& relative_displacement_vector, const geometry_msgs::Pose& pose_to_match, RotationType rotation_type) {
                geometry_msgs::Pose pose;
                tf2::Quaternion default_down = getDefaultDown(planning_group);
                tf2::Quaternion q_to_match;
                tf2::fromMsg(pose_to_match.orientation, q_to_match);
                tf2::Quaternion rotate_by =  q_to_match * getRotation(rotation_type);

                tf2::Vector3 disp_rotated = tf2::quatRotate(rotate_by, relative_displacement_vector);
                pose.position.x = pose_to_match.position.x + disp_rotated[0]; 
                pose.position.y = pose_to_match.position.y + disp_rotated[1];
                pose.position.z = pose_to_match.position.z + disp_rotated[2];

                //tf2::Quaternion orientation = rotate_by * default_down;
                tf2::Quaternion orientation = q_to_match * getRotation(rotation_type) * default_down;
                orientation.normalize();
                pose.orientation = convert(orientation);
                return pose;
            }


    };
}