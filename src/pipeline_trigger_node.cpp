// ROS
#include "ros/ros.h"
#include "manipulation_interface/PreferenceQuery.h"
#include "manipulation_interface/RunQuery.h"


int main(int argc, char** argv) {
	ros::init(argc, argv, "pipeline_trigger_node");
	ros::NodeHandle pipeline_NH;

    // Plan service:
    ROS_INFO("Planning...");
    ros::ServiceClient plan_client = pipeline_NH.serviceClient<manipulation_interface::PreferenceQuery>("/preference_planning_query");
    manipulation_interface::PreferenceQuery preference_query;

    // 'formulas_ordered' is an ordered list of formulas with the first element being the highest priority formula, and so on:
    preference_query.request.formulas_ordered = {"F(obj_1_L1)", "F(obj_2_L1)"}; 
    // 'flexibility' is the flexibility parameter mu in units of action cost (float):
    preference_query.request.flexibility = 0.0f;

    bool server_found = false;
    bool plan_success;
    ros::Rate r(1);
    while (!server_found) {
        server_found = plan_client.call(preference_query);
        r.sleep();
        plan_success = preference_query.response.success;
    }
    ROS_INFO("Done!");

    // Run and execute service:
    ROS_INFO("Executing...");
    ros::ServiceClient run_client = pipeline_NH.serviceClient<manipulation_interface::RunQuery>("/action_run_query");
    manipulation_interface::RunQuery run_query;

    // 'start_time' marks the start time for determining execution duration (TODO)
    run_query.request.start_time = ros::Time::now(); 

    server_found = false;
    bool run_success;
    while (!server_found) {
        server_found = run_client.call(run_query);
        r.sleep();
        run_success = preference_query.response.success;
    }
    ROS_INFO("Done!");

    return 0;
}