#pragma once

#include <string>

#include <ros/ros.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Quaternion.h>

#include "Tools.h"

namespace TaskIt {

class ManipulatorProperties {
    public:
        static void load(const ros::NodeHandle& nh, const std::string& planning_group, const std::string& arm_config_ns = "arm_config") {
            nh.getParam(getParamName("default_down_quaternion", arm_config_ns), s_values[planning_group].default_down);
            nh.getParam(getParamName("stow_joint_values", arm_config_ns), s_values[planning_group].stow_joint_values);
            nh.getParam(getParamName("eef_offset", arm_config_ns), s_values[planning_group].eef_offset);
            nh.param(getParamName("linear_safe", arm_config_ns), s_values[planning_group].linear_safe, true);
            nh.param(getParamName("linear_eef_step_size", arm_config_ns), s_values[planning_group].linear_eef_step_size, 0.0001);
            nh.param(getParamName("linear_jump_threshold", arm_config_ns), s_values[planning_group].linear_jump_threshold, 0.0001);
            nh.param(getParamName("linear_max_acceleration_scale", arm_config_ns), s_values[planning_group].linear_max_acceleration_scale, 0.05);
            nh.param(getParamName("linear_n_waypoints", arm_config_ns), s_values[planning_group].linear_n_waypoints, 5);
        }

        static const std::map<std::string, double>& getDefaultDownRPY(const std::string& planning_group) {return s_values.at(planning_group).default_down;}
        static const std::vector<double>& getStowJointValues(const std::string& planning_group) {return s_values.at(planning_group).stow_joint_values;}
        static float getEndEffectorOffset(const std::string& planning_group) {return s_values.at(planning_group).eef_offset;}
        static bool enforceSafeLinearMovement(const std::string& planning_group) {return s_values.at(planning_group).linear_safe;}
        static float getLinearEEFStepSize(const std::string& planning_group) {return s_values.at(planning_group).linear_eef_step_size;}
        static float getLinearJumpThreshold(const std::string& planning_group) {return s_values.at(planning_group).linear_jump_threshold;}
        static float getLinearMaxAccelerationScale(const std::string& planning_group) {return s_values.at(planning_group).linear_max_acceleration_scale;}
        static uint32_t getLinearNumWaypoints(const std::string& planning_group) {return s_values.at(planning_group).linear_n_waypoints;}

    private:
        struct ManipulatorPropertiesValues {
            std::map<std::string, double> default_down;
            std::vector<double> stow_joint_values;
            float eef_offset;
            bool linear_safe;
            double linear_eef_step_size;
            double linear_jump_threshold;
            double linear_max_acceleration_scale;
            int linear_n_waypoints;
        };
    private:
        inline static std::map<std::string, ManipulatorPropertiesValues> s_values;
};

}