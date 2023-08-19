#pragma once

#include <cmath>

#include <tf2_eigen/tf2_eigen.h>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

// Msg types
#include "taskit/MovementProperties.h"

// TaskIt
#include "Config.h"

namespace TaskIt {

class MovementAnalysis {
    public:
        // Wrapper around msg reference to dynamically decide if the function should 'create' or 'append'
        struct Argument {
            public:
                /// @brief Construct an argument with movement properties msg
                /// @param mv_props Properties msg
                /// @param auto_append_ Automatically append properties after the first movement
                /// @param infemum_time Using one clock, append using maximum time value (true) instead of cumulative time value (false)
                Argument(taskit::MovementProperties& mv_props, bool auto_append_ = true, bool infemum_time_ = true) 
                    : m_mv_props(mv_props) 
                    , auto_append(auto_append_)
                    , infemum_time(infemum_time_)
                    , append(false)
                {}
                operator taskit::MovementProperties&() {return m_mv_props;}

            public:
                bool append;
                const bool auto_append;
                const bool infemum_time;
            private:
                taskit::MovementProperties& m_mv_props;
        };
    public:
        /// @brief Create in-place properties of the robotic movement
        /// @param mv_props Movement properties to edit in place
        /// @param execution_success Success of the execution
        /// @param start_time Time stamp marking the start of the action primitive. The execution time is the difference btw the start time and the time this function is called
        /// @param path_length Length of the robot trajectory
        /// @param max_velocity Maximum speed encountered along the robot trajectory
        /// @param max_acceleration Maximum acceleration magnitude encountered along the robot trajectory
        /// @param max_effort Maximum effort magnitude encountered along the robot trajectory
        void create(
                taskit::MovementProperties& mv_props, 
                bool execution_success, 
                ros::Time start_time, 
                double path_length = 0.0, 
                double max_velocity = 0.0, 
                double max_acceleration = 0.0, 
                double max_effort = 0.0, 
                const std::vector<geometry_msgs::Pose>& eef_trajectory = std::vector<geometry_msgs::Pose>{},
                const std::vector<double>& waypoint_durations = std::vector<double>{});

        /// @brief Create in-place properties of the robotic movement
        /// @param mv_props Movement properties to edit in place
        /// @param execution_success Success of the execution
        /// @param start_time Time stamp marking the start of the action primitive. 
        /// The execution time is the difference btw the start time and the time this function is called
        /// @param move_group 
        /// @param motion_plan Motion plan for the manipulator
        void create(taskit::MovementProperties& mv_props, 
                bool execution_success, 
                ros::Time start_time, 
                const robot_trajectory::RobotTrajectory& r_traj);

        /// @brief Create in-place properties of the robotic movement
        /// @param mv_props Movement properties to edit in place
        /// @param execution_success Success of the execution
        /// @param start_time Time stamp marking the start of the action primitive. 
        /// The execution time is the difference btw the start time and the time this function is called
        /// @param move_group 
        /// @param motion_plan Motion plan for the manipulator
        void create(taskit::MovementProperties& mv_props, 
                bool execution_success, 
                ros::Time start_time, 
                const moveit::planning_interface::MoveGroupInterface& move_group, 
                const moveit::planning_interface::MoveGroupInterface::Plan& motion_plan);

        /// @brief Append properties of the robotic movement to existing properties of a previous movement
        /// @param mv_props Movement properties to edit in place
        template <typename...ARGS_T>
        void append(Argument& mv_arg, ARGS_T&&...args) {
            taskit::MovementProperties mv_props_append;
            create(mv_props_append, std::forward<ARGS_T>(args)...);
            appendExisting(mv_arg, mv_props_append);
        }


        /// @brief Dynamically decide if properties should be created (previous properties erased) or appended
        /// (appended to previous properties)
        /// @param mv_arg Movement properties argument
        template <typename...ARGS_T>
        void make(Argument& mv_arg, ARGS_T&&... args) {
            if (mv_arg.append) {
                append(mv_arg, std::forward<ARGS_T>(args)...);
            } else {
                create(mv_arg, std::forward<ARGS_T>(args)...);
            }
            // Auto append sets the append flag to true after the first time movement properties were created
            if (mv_arg.auto_append) {
                mv_arg.append = true;
            }
        }

    private:
        void appendExisting(Argument& mv_arg, const taskit::MovementProperties& mv_props_append);

};

}