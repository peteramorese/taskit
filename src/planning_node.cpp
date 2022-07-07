#include <math.h>
#include <unordered_map>
#include "ros/ros.h"
#include "manipulation_interface/PlanningQuery.h"
#include "manipulation_interface/Strategy.h"
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
			location_map[lbl] = ps;
		}
		const geometry_msgs::Pose& getLocation(const std::string& lbl) {
			return location_map.at(lbl);
		}
		std::vector<geometry_msgs::Pose> getPoseArray(const std::vector<std::string>& state) {
			std::vector<geometry_msgs::Pose> ps_arr(state.size());
			for (int i=0; i<state.size(); ++i) {
				ps_arr[i] = location_map.at(state[i]);
			}
			return ps_arr;
		}
};

class ExecuteSrv {
	private:
		LocationCoordinates* locs;
		std::vector<std::string> obj_group;
	public:
		ExecuteSrv(LocationCoordinates* locs_, const std::vector<std::string>& obj_group_) : locs(locs_), obj_group(obj_group_) {}
		bool execute(manipulation_interface::ActionSingle& req, manipulation_interface::ActionSingle& res) {
			// Execute
			manipulation_interface::PlanningQuery query;
			std::vector<std::string> bag_domain_labels(N_obj);
			for (int i=0; i<obj_group.size(); ++i) {
				bag_domain_labels[i] = "domain";	
			}
			query.request.setup_environment = true;
			if (req.action == "transit_up") {
				// Find the location label that the eef moves to
				std::string temp_loc_label = state_seq[i+1]->getVar("eeLoc");
				std::string temp_obj_label;
				// Find which object has a location label equal to the eef's location
				state_seq[i+1]->argFindGroup(temp_loc_label, "object locations", temp_obj_label);
				// The pose array should be the same length as obj_group if it has
				// observed all objects, therefore the index of the group will suffice
				// for sending the true pose
				int pose_ind;
				for (int ii=0; ii<obj_group.size(); ++ii) {
					if (obj_group[ii] == temp_obj_label) {
						pose_ind = ii;
					}
				}
				query.request.manipulator_pose = data->poses[pose_ind];
				temp_orient = query.request.manipulator_pose.orientation;
				query.request.bag_poses = *data;
				query.request.bag_labels = obj_group;
				query.request.bag_domain_labels = bag_domain_labels;
				query.request.pickup_object = "none";
				query.request.grasp_type = "up";
				query.request.drop_object = "none";
				query.request.planning_domain = "domain";
				query.request.safe_config = false;
			} else if (req.action == "transit_side") {
				// Find the location label that the eef moves to
				std::string temp_loc_label = state_seq[i+1]->getVar("eeLoc");
				std::string temp_obj_label;
				// Find which object has a location label equal to the eef's location
				state_seq[i+1]->argFindGroup(temp_loc_label, "object locations", temp_obj_label);
				std::cout<<"going to obj: "<<temp_obj_label<<std::endl;
				// The pose array should be the same length as obj_group if it has
				// observed all objects, therefore the index of the group will suffice
				// for sending the true pose
				int pose_ind;
				for (int ii=0; ii<obj_group.size(); ++ii) {
					if (obj_group[ii] == temp_obj_label) {
						pose_ind = ii;
					}
				}
				query.request.manipulator_pose = data->poses[pose_ind];
				temp_orient = query.request.manipulator_pose.orientation;
				query.request.bag_poses = *data;
				//query.request.setup_environment = true;
				query.request.bag_labels = obj_group;
				query.request.bag_domain_labels = bag_domain_labels;
				query.request.pickup_object = "none";
				query.request.grasp_type = "side";
				query.request.drop_object = "none";
				query.request.planning_domain = "domain";
				query.request.safe_config = false;
			} else if (req.action == "transport") {
				std::string temp_loc_label = state_seq[i+1]->getVar("eeLoc");
				std::cout<<"temp loc label: "<<temp_loc_label<<std::endl;
				query.request.manipulator_pose = locs.getLocation(temp_loc_label);
				std::cout<<"\n"<<std::endl;
				std::cout<<query.request.manipulator_pose.position.x<<std::endl;
				std::cout<<query.request.manipulator_pose.position.y<<std::endl;
				std::cout<<query.request.manipulator_pose.position.z<<std::endl;
				std::cout<<query.request.manipulator_pose.position.z<<std::endl;
				std::cout<<query.request.manipulator_pose.orientation.x<<std::endl;
				std::cout<<query.request.manipulator_pose.orientation.y<<std::endl;
				std::cout<<query.request.manipulator_pose.orientation.z<<std::endl;
				std::cout<<query.request.manipulator_pose.orientation.w<<std::endl;
				//query.request.manipulator_pose.position = locs.getLocation(temp_loc_label);
				//query.request.manipulator_pose.orientation = temp_orient;
				//temp_orient = query.request.manipulator_pose.orientation;
				query.request.bag_poses = *data;
				//query.request.setup_environment = true;
				query.request.bag_labels = bag_labels;
				query.request.bag_domain_labels = bag_domain_labels;
				query.request.pickup_object = "none";
				query.request.grasp_type = "up";
				query.request.drop_object = "none";
				query.request.planning_domain = "domain";
				query.request.safe_config = false;

			} else if (req.action == "grasp") {
				query.request.setup_environment = false;

				std::string temp_obj_label;
				state_seq[i+1]->argFindGroup("ee", "object locations", temp_obj_label);
				std::cout<<"pick obj: "<<temp_obj_label<<std::endl;
				query.request.pickup_object = temp_obj_label;
				query.request.drop_object = "none";
				query.request.planning_domain = "domain";
				query.request.safe_config = false;
			} else if (req.action == "release") {
				query.request.setup_environment = false;
				query.request.pickup_object = "none";

				std::string temp_obj_label;
				state_seq[i]->argFindGroup("ee", "object locations", temp_obj_label);
				std::cout<<"rel obj: "<<temp_obj_label<<std::endl;
				query.request.drop_object = temp_obj_label; 
				query.request.planning_domain = "domain";
				query.request.safe_config = false;
			} else {
				ROS_WARN("Unrecognized action %s", req.action.c_str());
			}
			if (plan_query_client.call(query)) {
				ROS_INFO("Completed service");
			} else {
				ROS_WARN("Did not find plan query service");
			}
		}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "planning_node");
	ros::NodeHandle com_NH;

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
	geometry_msgs::Quaternion q_down_msg, q_side_msg;
	tf2::convert(q_down, q_down_msg);
	q_rot_side.setRPY(-M_PI/2, 0, 0);
	q_side = q_rot_side * q_init;
	tf2::convert(q_side, q_side_msg);
	q_down.normalize();
	q_side.normalize();

 	std::vector<std::string> loc_labels = {"L0", "L1", "L2", "arch_c", "arch_left", "arch_right"};
	std::vector<std::string> side_loc_labels = {"arch_c"};
	geometry_msgs::Point p;
	p.x = .5;
	p.y = -.25;
	p.z = .08;
	locs.addLocation(p, q_down_msg, loc_labels[0], .15); // L0

	p.x = .5;
	p.y = .25;
	p.z = .08;
	locs.addLocation(p, q_down_msg, loc_labels[1], .15); // L1

	p.x = -.5;
	p.y = .25;
	p.z = .08;
	locs.addLocation(p, q_down_msg, loc_labels[2], .15); // L2

	p.x = 0;
	p.y = .35;
	p.z = .185;
	locs.addLocation(p, q_side_msg, loc_labels[3], .08); // center of the arch

	p.x = .075;
	p.y = .4;
	p.z = .08;
	locs.addLocation(p, q_down_msg, loc_labels[4], .05); // left of the arch

	p.x = -.075;
	p.y = .4;
	p.z = .08;
	locs.addLocation(p, q_down_msg, loc_labels[5], .05); // right of the arch

	locs.addLocation(0, .4, .09, "L2", .15);
	locs.addLocation(-.4, -.4, .09, "L3", .15);
	locs.addLocation(-.4, .4, .09, "L4", .15);

	//ros::ServiceClient strategy_srv_client = com_NH.serviceClient<manipulation_interface::Strategy>("/com_node/strategy");
	manipulation_interface::Strategy strategy_srv;	
	ros::ServiceClient plan_query_client = com_NH.serviceClient<manipulation_interface::PlanningQuery>("/planning_query");

	//geometry_msgs::PoseArray* data = vicon_data.returnConfigArrPtr();
	//std::vector<std::string> bag_labels = {"box0", "box1", "box2"};
	

	

	return 0;
}
