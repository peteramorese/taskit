// System
#include<boost/filesystem.hpp>

// ROS
#include "ros/ros.h"
#include "ros/package.h"
#include "manipulation_interface/ActionSingle.h"
#include "manipulation_interface/PreferenceQuery.h"
#include "manipulation_interface/RunQuery.h"

int main(int argc, char** argv) {
	ros::init(argc, argv, "plan_query_kickoff_node");
	ros::NodeHandle kickoff_NH;

	// Kickoff delay for startup:
	float kickoff_delay = 0.0f;
	kickoff_NH.param("/discrete_environment/kickoff_delay", kickoff_delay, 10.0f);
	
	// Open loop or closed loop option:
	bool open_loop;
	kickoff_NH.param("/discrete_environment/open_loop", open_loop, true);

	std::vector<std::string> task_set;
	kickoff_NH.getParam("/discrete_environment/task_set", task_set);

	std::string query_type;
	kickoff_NH.getParam("/discrete_environment/query_type", query_type);
	float flexibility = 0.0f;
	int pareto_point_index = 0;
	if (query_type == "single") {
		kickoff_NH.getParam("/discrete_environment/flexibility", flexibility);
	} else if (query_type == "all" || query_type == "pareto_front") {
		kickoff_NH.getParam("/discrete_environment/pareto_point_index", pareto_point_index);
	}

	manipulation_interface::PreferenceQuery pref_query;
	pref_query.request.formulas_ordered = task_set;
	pref_query.request.flexibility = flexibility;

	ros::ServiceClient plan_client = kickoff_NH.serviceClient<manipulation_interface::PreferenceQuery>("/preference_planning_query");
	ROS_INFO("Kicking off the planning pipeline...");
	ROS_INFO("Planning...");
	ros::Duration(kickoff_delay).sleep();
	
	//if (plan_client.call(pref_query)) {
	//	ROS_INFO("Plan request succeeded!");
	//} else {
	//	ROS_ERROR("Plan request failed!");
	//	return 1;
	//}

	bool server_found = false;
	int retries = 0; int max_retires = 20;
	double sleep_duration = 10; // seconds
	while (!server_found && ros::ok()) {
		server_found = plan_client.call(pref_query);
		ros::Duration(sleep_duration).sleep();
		retries++;
		if (retries >= max_retires) {
			ROS_ERROR("Plan request timeout!");
			return 1;
		} else {
			ROS_INFO("Planning service not found, retrying");
		}
	}

	if (open_loop) {

		ROS_INFO("Executing...");
		ros::ServiceClient run_client = kickoff_NH.serviceClient<manipulation_interface::RunQuery>("/action_run_query");
		manipulation_interface::RunQuery run_query;

		// 'start_time' marks the start time for determining execution duration (TODO)
		run_query.request.start_time = ros::Time::now(); 

		server_found = false;
		retries = 0; 
		max_retires = 10;
		while (!server_found && ros::ok()) {
			server_found = run_client.call(run_query);
			ros::Duration(sleep_duration).sleep();
			retries++;
			if (retries >= max_retires) {
				ROS_ERROR("Could not find RunQuery service server");
				return 1;
			}
		}
		ROS_INFO("Done!");
	} else {
		ros::ServiceClient com_client = kickoff_NH.serviceClient<manipulation_interface::RunQuery>("/com_node/kickoff");
		manipulation_interface::RunQuery run_query;

		run_query.request.start_time = ros::Time::now(); 

		ROS_INFO("Kicking off Com Node for execution...");
		if (com_client.call(run_query)) {
			ROS_INFO("Plan request succeeded!");
		} else {
			ROS_ERROR("Plan request failed!");
			return 1;
		}

	}

	return 0;
}


