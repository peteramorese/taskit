#include <math.h>
#include <unordered_map>
#include "ros/ros.h"
#include "manipulation_interface/PlanningQuery.h"
#include "manipulation_interface/Strategy.h"
#include "manipulation_interface/ActionSingle.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// OW Imports
#include "ow_lander/DigLinear.h"
#include "ow_lander/Grind.h"
#include "ow_lander/DeliverSample.h"
#include "ow_lander/Stow.h"
#include "ow_lander/Unstow.h"



class LocationCoordinates {
	private:
		std::unordered_map<std::string, geometry_msgs::Pose> location_map;
	public:
		void addLocation(const geometry_msgs::Point& p, const geometry_msgs::Quaternion& q, const std::string& lbl) {
			geometry_msgs::Pose ps;
			ps.position = p;
			ps.orientation = q;
			std::cout<<"Including lbl: "<<lbl<<std::endl;
			location_map[lbl] = ps;
		}
		const geometry_msgs::Pose& getLocation(const std::string& lbl) {
			std::cout<<"Retrieving lbl: "<<lbl<<std::endl;
			return location_map.at(lbl);
		}
		std::vector<geometry_msgs::Pose> getPoseArray(const std::vector<std::string>& state) {
			std::vector<geometry_msgs::Pose> ps_arr(state.size());
			for (int i=0; i<state.size(); ++i) {

				std::cout<<"Retrieving lbl: "<<state[i]<<std::endl;
				ps_arr[i] = location_map.at(state[i]);
			}
			return ps_arr;
		}
};

class ExecuteSrv {
	private:
	 	ros::ServiceClient* plan_query_client;
		// OW Clients
		ros::ServiceClient* plan_dig_linear_client;
		ros::ServiceClient* plan_grind_client;
		ros::ServiceClient* plan_deliver_sample_client;
		ros::ServiceClient* plan_stow_client;
		ros::ServiceClient* plan_unstow_client;
		LocationCoordinates* locs;
		bool setup;
	public:
		ExecuteSrv(LocationCoordinates* locs_, ros::ServiceClient* plan_query_client_, ros::ServiceClient* plan_dig_linear_client_, ros::ServiceClient* plan_grind_client_, 
			ros::ServiceClient* plan_deliver_sample_client_, ros::ServiceClient* plan_stow_client_, ros::ServiceClient* plan_unstow_client_) : 
				locs(locs_), setup(true), plan_query_client(plan_query_client_), plan_dig_linear_client(plan_dig_linear_client_), plan_grind_client(plan_grind_client_), plan_deliver_sample_client(plan_deliver_sample_client_),
					plan_stow_client(plan_stow_client_), plan_unstow_client(plan_unstow_client_) {}
		void reset() {
			setup = true;
		}
		bool execute(manipulation_interface::ActionSingle::Request& req, manipulation_interface::ActionSingle::Response& res) {
			// Execute
			manipulation_interface::PlanningQuery query;
			std::vector<std::string> bag_domain_labels(req.obj_group.size());
			for (int i=0; i<req.obj_group.size(); ++i) {
				bag_domain_labels[i] = "domain";	
			}
			if (setup) {
				query.request.setup_environment = true;
				query.request.bag_poses = locs->getPoseArray(req.init_obj_locs);
			} else {
				query.request.setup_environment = false;
			}
			std::cout<<"Received action: "<<req.action<<std::endl;

			

			if (req.action.find("dig") != std::string::npos) {
				// Get EE Location
				geometry_msgs::Pose ee_loc_msg = locs->getLocation(req.to_eeLoc);
				ow_lander::DigLinear dig_query;
				// Hardcoded defaults
				dig_query.request.length = 0.6;
				dig_query.request.use_defaults = false;
				dig_query.request.ground_position = -0.155;
				// Requested values
				dig_query.request.x = ee_loc_msg.position.x;
				dig_query.request.y = ee_loc_msg.position.y;
				dig_query.request.depth = ee_loc_msg.position.z;
				// call service
				plan_dig_linear_client->call(dig_query);
				if(dig_query.response.success) {
					ROS_INFO("DigLinear Successful");
					res.success = true;
					return true;
				} else {
					ROS_WARN("DigLinear Failed");
					//ROS_WARN(dig_query.response.message);
					res.success = false;
					return true;
				}
			} else if (req.action.find("grind") != std::string::npos) {
				// Get EE Location
				geometry_msgs::Pose ee_loc_msg = locs->getLocation(req.to_eeLoc);
				ow_lander::Grind grind_query;
				// Hardcoded defaults
				grind_query.request.length = 0.6;
				grind_query.request.use_defaults = false;
				grind_query.request.ground_position = -0.155;
				grind_query.request.parallel = true;
				// Requested values
				grind_query.request.x = ee_loc_msg.position.x;
				grind_query.request.y = ee_loc_msg.position.y;
				grind_query.request.depth = ee_loc_msg.position.z;
				// call service
				plan_grind_client->call(grind_query);
				if(grind_query.response.success) {
					ROS_INFO("Grind Successful");
					res.success = true;
					return true;
				} else {
					ROS_WARN("Grind Failed");
					//ROS_WARN(grind_query.response.message);
					res.success = false;
					return true;
				}
			} else if (req.action.find("deliver") != std::string::npos) {
				ow_lander::DeliverSample deliever_query;
				// call service
				plan_deliver_sample_client->call(deliever_query);
				if(deliever_query.response.success) {
					ROS_INFO("DeliverSample Successful");
					res.success = true;
					return true;
				} else {
					ROS_WARN("DeliverSample Failed");
					//ROS_WARN(deliever_query.response.message);
					res.success = false;
					return true;
				}
			} else if (req.action.find("unstow") != std::string::npos) {
				ow_lander::Unstow unstow_query;
				// call service
				plan_unstow_client->call(unstow_query);
				if(unstow_query.response.success) {
					ROS_INFO("Unstow Successful");
					res.success = true;
					return true;
				} else {
					ROS_WARN("Unstow Failed");
					//ROS_WARN(unstow_query.response.message);
					res.success = false;
					return true;
				}
			} else if (req.action.find("stow") != std::string::npos) {
				ow_lander::Stow stow_query;
				// call service
				plan_stow_client->call(stow_query);
				if(stow_query.response.success) {
					ROS_INFO("Stow Successful");
					res.success = true;
					return true;
				} else {
					ROS_WARN("Stow Failed");
					//ROS_WARN(stow_query.response.message);
					res.success = false;
					return true;
				}
			} else {
				ROS_WARN("Unrecognized action %s", req.action.c_str());
				return false;
			}
			if (setup) {
				setup = false;
			}
			// if (plan_query_client->call(query)) {
			// 	ROS_INFO("Completed action primitive call");
			// 	res.success = false;
			// 	return true;
			// } else {
			// 	ROS_WARN("Did not find plan query service");
			// 	res.success = true;
			// 	return true;
			// }
			ROS_WARN("Why??");
			res.success = true;
			return true;
		}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "action_primitive_node");
	ros::NodeHandle action_primitive_NH;

	//RetrieveData vicon_data(30, &com_NH);
	LocationCoordinates locs;
	// Quaternion for downwards release:
	tf2::Quaternion q_init, q_down, q_side, q_rot_down, q_rot_side;
	q_init[0] = 0;
	q_init[1] = 0;
	q_init[2] = 1;
	q_init[3] = 0;
	//q_rot_down.setRPY(0, M_PI, -M_PI/4);
	//q_down = q_rot_down * q_init;
	q_down = q_init;
	geometry_msgs::Quaternion q_up_msg, q_side_msg;
	tf2::convert(q_down, q_up_msg);
	q_rot_side.setRPY(-M_PI/2, 0, 0);
	q_side = q_rot_side * q_init;
	tf2::convert(q_side, q_side_msg);
	q_down.normalize();
	q_side.normalize();



    std::vector<std::string> location_names;
    action_primitive_NH.getParam("/ow_environment/location_names", location_names);

    std::vector<std::map<std::string, float>> location_points(location_names.size());
    //std::vector<std::string> location_orientation_types;
    //action_primitive_NH.getParam("/ow_environment/location_orientation_types", location_orientation_types);
    for (int i=0; i<location_names.size(); ++i) {
        action_primitive_NH.getParam("/ow_environment/" + location_names[i] + "_point", location_points[i]);
        std::cout<<"Loaded point for: "<<location_names[i]<<std::endl;
        std::cout<<"  - x: "<<location_points[i].at("x")<<"\n";
        std::cout<<"  - y: "<<location_points[i].at("y")<<"\n";
        std::cout<<"  - z: "<<location_points[i].at("z")<<"\n";
        // Add condition if orientation type = 'manual'
		geometry_msgs::Point p;
		p.x = location_points[i].at("x");
		p.y = location_points[i].at("y");
		p.z = location_points[i].at("z");
		// if (location_orientation_types[i] == "up") {
		locs.addLocation(p, q_up_msg, location_names[i]); 
		// } else if (location_orientation_types[i] == "side") {
		// 	locs.addLocation(p, q_side_msg, location_names[i]); 
		// } else {
		// 	std::string msg = "Did not find orientation preset:" + location_names[i];
		// 	ROS_ERROR_STREAM(msg.c_str());
		// }
    }


	ros::ServiceClient plan_query_client = action_primitive_NH.serviceClient<manipulation_interface::PlanningQuery>("/manipulation_planning_query");
	ros::ServiceClient plan_dig_linear_client = action_primitive_NH.serviceClient<ow_lander::DigLinear>("/arm/dig_linear");
	ros::ServiceClient plan_grind_client = action_primitive_NH.serviceClient<ow_lander::Grind>("/arm/grind");
	ros::ServiceClient plan_deliver_client = action_primitive_NH.serviceClient<ow_lander::DeliverSample>("/arm/deliver_sample");
	ros::ServiceClient plan_stow_client = action_primitive_NH.serviceClient<ow_lander::Stow>("/arm/stow");
	ros::ServiceClient plan_unstow_client = action_primitive_NH.serviceClient<ow_lander::Unstow>("/arm/unstow");


	ExecuteSrv ex(&locs, &plan_query_client, &plan_dig_linear_client, &plan_grind_client, &plan_deliver_client, &plan_stow_client, &plan_unstow_client);
	ros::ServiceServer ex_srv = action_primitive_NH.advertiseService("/action_primitive", &ExecuteSrv::execute, &ex);
	ROS_INFO("Execution service is online!");
	ros::spin();

	

	return 0;
}
