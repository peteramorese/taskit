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
		LocationCoordinates* locs;
		bool setup;
	public:
		ExecuteSrv(LocationCoordinates* locs_, ros::ServiceClient* plan_query_client_) : locs(locs_), setup(true), plan_query_client(plan_query_client_) {}
		void reset() {
			setup = true;
		}
		bool execute(manipulation_interface::ActionSingle::Request& req, manipulation_interface::ActionSingle::Response& res) {
			// Execute
			std::cout<<"IN ACTION PRIMITIVE EXECUTE"<<std::endl;
			manipulation_interface::PlanningQuery query;
			std::vector<std::string> bag_domain_labels(req.obj_group.size());
			for (int i=0; i<req.obj_group.size(); ++i) {
				bag_domain_labels[i] = "domain";	
			}
			if (setup) {
				query.request.setup_environment = true;
				query.request.bag_poses = locs->getPoseArray(req.init_obj_locs);
				setup = false;
			} else {
				query.request.setup_environment = false;
			}
			query.request.bag_labels = req.obj_group;

			std::cout<<"Received action: "<<req.action<<std::endl;
			if (req.action.find("transit_up") != std::string::npos) {
				//// Find the location label that the eef moves to
				//std::string temp_loc_label = state_seq[i+1]->getVar("eeLoc");
				//std::string temp_obj_label;
				//// Find which object has a location label equal to the eef's location
				//state_seq[i+1]->argFindGroup(temp_loc_label, "object locations", temp_obj_label);
				//// The pose array should be the same length as obj_group if it has
				//// observed all objects, therefore the index of the group will suffice
				//// for sending the true pose
				//std::string temp_obj_label = req.to_obj;
				//int pose_ind;
				//for (int ii=0; ii<obj_group.size(); ++ii) {
				//	if (obj_group[ii] == temp_obj_label) {
				//		pose_ind = ii;
				//	}
				//}

				query.request.manipulator_pose = locs->getLocation(req.to_eeLoc);
				query.request.bag_domain_labels = bag_domain_labels;
				query.request.pickup_object = "none";
				query.request.grasp_type = "up";
				query.request.drop_object = "none";
				query.request.planning_domain = "domain";
				query.request.safe_config = false;
			} else if (req.action.find("transit_side") != std::string::npos) {
				//// Find the location label that the eef moves to
				//std::string temp_loc_label = state_seq[i+1]->getVar("eeLoc");
				//std::string temp_obj_label;
				//// Find which object has a location label equal to the eef's location
				//state_seq[i+1]->argFindGroup(temp_loc_label, "object locations", temp_obj_label);
				//std::cout<<"going to obj: "<<temp_obj_label<<std::endl;
				//// The pose array should be the same length as obj_group if it has
				//// observed all objects, therefore the index of the group will suffice
				//// for sending the true pose
				//int pose_ind;
				//for (int ii=0; ii<obj_group.size(); ++ii) {
				//	if (obj_group[ii] == temp_obj_label) {
				//		pose_ind = ii;
				//	}
				//}
				query.request.manipulator_pose = locs->getLocation(req.to_eeLoc);
				query.request.bag_domain_labels = bag_domain_labels;
				query.request.pickup_object = "none";
				query.request.grasp_type = "side";
				query.request.drop_object = "none";
				query.request.planning_domain = "domain";
				query.request.safe_config = false;
			} else if (req.action.find("transport") != std::string::npos) {
				//std::string temp_loc_label = state_seq[i+1]->getVar("eeLoc");
				//std::cout<<"temp loc label: "<<temp_loc_label<<std::endl;
				query.request.manipulator_pose = locs->getLocation(req.to_eeLoc);
				//std::cout<<"\n"<<std::endl;
				//std::cout<<query.request.manipulator_pose.position.x<<std::endl;
				//std::cout<<query.request.manipulator_pose.position.y<<std::endl;
				//std::cout<<query.request.manipulator_pose.position.z<<std::endl;
				//std::cout<<query.request.manipulator_pose.position.z<<std::endl;
				//std::cout<<query.request.manipulator_pose.orientation.x<<std::endl;
				//std::cout<<query.request.manipulator_pose.orientation.y<<std::endl;
				//std::cout<<query.request.manipulator_pose.orientation.z<<std::endl;
				//std::cout<<query.request.manipulator_pose.orientation.w<<std::endl;
				//query.request.manipulator_pose.position = locs.getLocation(temp_loc_label);
				//query.request.manipulator_pose.orientation = temp_orient;
				//temp_orient = query.request.manipulator_pose.orientation;
				//query.request.setup_environment = true;
				query.request.bag_domain_labels = bag_domain_labels;
				query.request.pickup_object = "none";
				query.request.grasp_type = "up";
				query.request.drop_object = "none";
				query.request.planning_domain = "domain";
				query.request.safe_config = false;
			} else if (req.action.find("grasp") != std::string::npos) {
				//query.request.setup_environment = false;
				//std::string temp_obj_label;
				//state_seq[i+1]->argFindGroup("ee", "object locations", temp_obj_label);
				//std::cout<<"pick obj: "<<temp_obj_label<<std::endl;
				query.request.pickup_object = req.to_grasp_obj;
				query.request.drop_object = "none";
				query.request.planning_domain = "domain";
				query.request.safe_config = false;
			} else if (req.action.find("release") != std::string::npos) {
				//query.request.setup_environment = false;
				//query.request.pickup_object = "none";

				//std::string temp_obj_label;
				//state_seq[i]->argFindGroup("ee", "object locations", temp_obj_label);
				//std::cout<<"rel obj: "<<temp_obj_label<<std::endl;
				query.request.pickup_object = "none";
				query.request.drop_object = req.release_obj; 
				query.request.planning_domain = "domain";
				query.request.safe_config = false;
			} else {
				ROS_WARN("Unrecognized action %s", req.action.c_str());
				return false;
			}
			if (plan_query_client->call(query)) {
				ROS_INFO("Completed action primitive call");
				res.success = true;
			} else {
				ROS_WARN("Did not find plan query service");
				res.success = false;
			}
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
    action_primitive_NH.getParam("/discrete_environment/location_names", location_names);

    std::vector<std::map<std::string, float>> location_points(location_names.size());
    std::vector<std::string> location_orientation_types;
    action_primitive_NH.getParam("/discrete_environment/location_orientation_types", location_orientation_types);
    for (int i=0; i<location_names.size(); ++i) {
        action_primitive_NH.getParam("/discrete_environment/" + location_names[i] + "_point", location_points[i]);
        std::cout<<"Loaded point for: "<<location_names[i]<<std::endl;
        std::cout<<"  - x: "<<location_points[i].at("x")<<"\n";
        std::cout<<"  - y: "<<location_points[i].at("y")<<"\n";
        std::cout<<"  - z: "<<location_points[i].at("z")<<"\n";
        // Add condition if orientation type = 'manual'
		geometry_msgs::Point p;
		p.x = location_points[i].at("x");
		p.y = location_points[i].at("y");
		p.z = location_points[i].at("z");
		if (location_orientation_types[i] == "up") {
			locs.addLocation(p, q_up_msg, location_names[i]); 
		} else if (location_orientation_types[i] == "side") {
			locs.addLocation(p, q_side_msg, location_names[i]); 
		} else {
			std::string msg = "Did not find orientation preset:" + location_names[i];
			ROS_ERROR_STREAM(msg.c_str());
		}
    }


	ros::ServiceClient plan_query_client = action_primitive_NH.serviceClient<manipulation_interface::PlanningQuery>("/manipulation_planning_query");
	ExecuteSrv ex(&locs, &plan_query_client);
	ros::ServiceServer ex_srv = action_primitive_NH.advertiseService("/action_primitive", &ExecuteSrv::execute, &ex);
	ROS_INFO("Action primitive execution service is online!");
	ros::spin();

	

	return 0;
}
