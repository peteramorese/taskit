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
            nh.getParam(getParamName("linear_mover", arm_config_ns), s_values[planning_group].linear_mover);
            nh.getParam(getParamName("planner_id", arm_config_ns), s_values[planning_group].planner_id);
            nh.param<std::string>(getParamName("optimal_planner_id", arm_config_ns), s_values[planning_group].optimal_planner_id, "RRTstar");
            nh.param(getParamName("linear_safe", arm_config_ns), s_values[planning_group].linear_safe, true);
            nh.param(getParamName("linear_eef_step_size", arm_config_ns), s_values[planning_group].linear_eef_step_size, 0.0001);
            nh.param(getParamName("linear_jump_threshold", arm_config_ns), s_values[planning_group].linear_jump_threshold, 0.0001);
            nh.param(getParamName("linear_first_point_fraction", arm_config_ns), s_values[planning_group].linear_first_point_fraction, 0.05);
            nh.param(getParamName("linear_n_waypoints", arm_config_ns), s_values[planning_group].linear_n_waypoints, 5);
            nh.param(getParamName("max_acceleration_scale", arm_config_ns), s_values[planning_group].max_acceleration_scale, 0.05);
            nh.param(getParamName("max_velocity_scale", arm_config_ns), s_values[planning_group].max_velocity_scale, 0.5);
        }

        static const std::map<std::string, double>& getDefaultDownRPY(const std::string& planning_group) {return s_values.at(planning_group).default_down;}
        static const std::vector<double>& getStowJointValues(const std::string& planning_group) {return s_values.at(planning_group).stow_joint_values;}
        static double getEndEffectorOffset(const std::string& planning_group) {return s_values.at(planning_group).eef_offset;}
        static const std::string& getLinearMover(const std::string& planning_group) {return s_values.at(planning_group).linear_mover;}
        static const std::string& getPlannerID(const std::string& planning_group) {return s_values.at(planning_group).planner_id;}
        static const std::string& getOptimalPlannerID(const std::string& planning_group) {return s_values.at(planning_group).optimal_planner_id;}
        static bool enforceSafeLinearMovement(const std::string& planning_group) {return s_values.at(planning_group).linear_safe;}
        static double getLinearEEFStepSize(const std::string& planning_group) {return s_values.at(planning_group).linear_eef_step_size;}
        static double getLinearJumpThreshold(const std::string& planning_group) {return s_values.at(planning_group).linear_jump_threshold;}
        static double getLinearFirstPointFraction(const std::string& planning_group) {return s_values.at(planning_group).linear_first_point_fraction;}
        static uint32_t getLinearNumWaypoints(const std::string& planning_group) {return s_values.at(planning_group).linear_n_waypoints;}
        static double getMaxAccelerationScale(const std::string& planning_group) {return s_values.at(planning_group).max_acceleration_scale;}
        static double getMaxVelocityScale(const std::string& planning_group) {return s_values.at(planning_group).max_velocity_scale;}

    private:
        struct ManipulatorPropertiesValues {
            std::map<std::string, double> default_down;
            std::vector<double> stow_joint_values;
            double eef_offset;
            std::string linear_mover;
            std::string planner_id;
            std::string optimal_planner_id;
            bool linear_safe;
            double linear_eef_step_size;
            double linear_jump_threshold;
            double linear_first_point_fraction;
            int linear_n_waypoints;
            double max_acceleration_scale;
            double max_velocity_scale;
        };
    private:
        inline static std::map<std::string, ManipulatorPropertiesValues> s_values;
};

}