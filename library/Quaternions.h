#pragma once

#include <string>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Quaternion.h>

namespace ManipulationInterface {
    class Quaternions {
        enum class Type {
            UpX,
            UpY,
            SideX,
            SideY
        };

        static inline geometry_msgs::Quaternion convert(const tf2::Quaternion q) { return tf2::convert(q); }

        static inline tf2::Quaternion getDefaultUp() {
            return tf2::Quaternion(0.0f, 0.0f, 1.0f, 0.0f);
        }

        static tf2::Quaternion get(Type type) {
            tf2::Quaternion q_default_up = getDefaultUp();
            tf2::Quaternion q_rot;
            switch (type) {
                case Type::UpX: { q_rot.setRPY(0.0f, 0.0f, 0.0f); break; }
                case Type::UpY: { q_rot.setRPY(0.0f, 0.0f, M_PI/2.0f); break; }
                case Type::SideX: { q_rot.setRPY(0.0f, -M_PI/2.0f, 0.0f); break; }
                case Type::SideY: { q_rot.setRPY(-M_PI/2.0f, M_PI/2.0f, 0.0f); break; }
                default: { q_rot.setRPY(0.0f, 0.0f, 0.0f); }
            }
            return q_rot * q_default_up;
        }

        static tf2::Quaternion get(const std::string& type_str) {
            Type type;
            if (type_str == "up_x" || type_str == "UpX") {
                type = Type::UpX;
            } else if (type_str == "up_y" || type_str == "UpY") {
                type = Type::UpY;
            } else if (type_str == "side_x" || type_str == "SideX") {
                type = Type::SideX;
            } else if (type_str == "side_y" || type_str == "SideY") {
                type = Type::SideY;
            }
            return get(type);
        }
    }
}