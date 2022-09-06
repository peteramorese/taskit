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
	ros::Rate delay(1.0f/kickoff_delay);
	delay.sleep();
	if (plan_client.call(pref_query)) {
		ROS_INFO("Kickoff Succeeded!");
	} else {
		ROS_ERROR("Kickoff Failed!");
		return 1;
	}
	return 0;
}


