#include <math.h>
#include <memory>
#include "ros/ros.h"
#include "manipulation_interface/PlanningQuery.h"
#include "manipulation_interface/Strategy.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"



class RetrieveData {
	private:
		class CallBackData {
		//	private:
		//		float x_offset = 0.0; //-.021544;
		//		float y_offset = 0.0;//.19416;
		//		float z_offset = 0.0; //.139;
			public:
				void onReceive(const geometry_msgs::PoseStamped::ConstPtr& pose_msg_ptr) {
					pose = (*pose_msg_ptr).pose;
					//data.push_back(*pose_msg_ptr);

					//configptr->pose.position.x = pose_msg_ptr->pose.position.x;
					//configptr->pose.position.y = pose_msg_ptr->pose.position.y;
					//configptr->pose.position.z = pose_msg_ptr->pose.position.z;
					//configptr->pose.orientation.x = pose_msg_ptr->pose.orientation.x;
					//configptr->pose.orientation.y = pose_msg_ptr->pose.orientation.y;
					//configptr->pose.orientation.z = pose_msg_ptr->pose.orientation.z;
					//configptr->pose.orientation.w = pose_msg_ptr->pose.orientation.w;
				}
				geometry_msgs::Pose pose;
		};

		int Navg, Nboxes;
		bool has_data;
		//geometry_msgs::PoseStamped avgConfig;
		ros::NodeHandle* SUB_NH;
		std::vector<CallBackData> data;
		std::vector<ros::Subscriber> subscribers;
		std::vector<geometry_msgs::Pose> pose_avg;
	public:
		RetrieveData(ros::NodeHandle* SUB_NH_, int Navg_, const std::vector<std::string>& pose_topics) : Navg(Navg_), SUB_NH(SUB_NH_), has_data(false) {

			for (auto pose_topic : pose_topics) std::cout<<"Subscribing to topic: " + pose_topic<< "\n";
			
			subscribers.resize(pose_topics.size());
			data.resize(pose_topics.size());
			pose_avg.resize(pose_topics.size());
			for (int i=0; i<subscribers.size(); ++i) {
				subscribers[i] = SUB_NH->subscribe(pose_topics[i], 10, &CallBackData::onReceive, &data[i]);
			}
			
			//sub_box_1 = SUB_NH->subscribe("/vrpn_client_node/greenBox_1/pose", 10, &callbackdata::sub_callback, &data[0]);
		}


		void retrieve() {
			//ros::Subscriber subscriber = SUB.subscribe("/vrpn_client_node/smallBox1/pose", 10, &callbackdata::sub_callback, &data);

			pose_avg.clear();
			pose_avg.resize(subscribers.size());
			for (int i=0; i<subscribers.size(); ++i) {
				geometry_msgs::Pose sample_pose;
				sample_pose.position.x = 0;
				sample_pose.position.y = 0;
				sample_pose.position.z = 0;
				sample_pose.orientation.x = 0;
				sample_pose.orientation.y = 0;
				sample_pose.orientation.z = 0;
				sample_pose.orientation.w = 0;

				ros::Rate r(30);
				ros::spinOnce();
				int Navg_actual = 0;
				while (Navg_actual<Navg) {
					ros::spinOnce();

					if (data[i].pose.position.x == 0.0){
						std::cout<<"Bad data"<< i << " " << Navg_actual << "<" << Navg << std::endl;
						r.sleep();
					} else {
						sample_pose.position.x += data[i].pose.position.x;
						sample_pose.position.y += data[i].pose.position.y;
						sample_pose.position.z += data[i].pose.position.z;
						sample_pose.orientation.x += data[i].pose.orientation.x;
						sample_pose.orientation.y += data[i].pose.orientation.y;
						sample_pose.orientation.z += data[i].pose.orientation.z;
						sample_pose.orientation.w += data[i].pose.orientation.w;
						Navg_actual++;
						r.sleep();
					}
					if (!ros::ok()){
						break;
					}
				}
				pose_avg[i].position.x = sample_pose.position.x/Navg_actual;
				pose_avg[i].position.y = sample_pose.position.y/Navg_actual;
				pose_avg[i].position.z = sample_pose.position.z/Navg_actual;
				pose_avg[i].orientation.x = sample_pose.orientation.x/Navg_actual;
				pose_avg[i].orientation.y = sample_pose.orientation.y/Navg_actual;
				pose_avg[i].orientation.z = sample_pose.orientation.z/Navg_actual;
				pose_avg[i].orientation.w = sample_pose.orientation.w/Navg_actual;
			}
			has_data = true;
		}

		const geometry_msgs::Pose* returnConfigPtr (int ind) const {
			if (!has_data) {
					std::cout<< "Call retrieve() before calling returnConfigPtr()" <<std::endl;
					return nullptr;
			} else {
					return &pose_avg[ind];
			}
		}

		const std::vector<geometry_msgs::Pose>* returnConfigArrPtr() const {
			if (!has_data) {
					std::cout<< "Call retrieve() before calling returnConfig()" <<std::endl;
					return nullptr;
			} else {
					return &pose_avg;
			}
		}
};



class PredicateGenerator {
	private:
		struct coord {
			geometry_msgs::Pose pose;
			std::string label;
		};
		std::vector<coord> locations;
		//std::vector<bool> is_occupied;
		std::vector<double> max_r_arr;
		// Store quaternion for all hard-coded drop off locations, this assumes
		// all bags should be placed in the same orientation
		double qx, qy, qz, qw;
	public:
		void addLocation(geometry_msgs::Point loc_center, geometry_msgs::Quaternion release_orientation, const std::string& coord_label, double max_r) {
			coord temp_coord;
			temp_coord.pose.position = loc_center;
			temp_coord.pose.orientation = release_orientation;
			temp_coord.label = coord_label;
			locations.push_back(temp_coord);
			max_r_arr.push_back(max_r);
		}

		geometry_msgs::Pose getLocation(const std::string& coord_label) {
			geometry_msgs::Point ret_pose;
			bool found = false;
			int ind = -1;
			for (int i=0; i<locations.size(); ++i) {
				if (locations[i].label == coord_label) {
					ind = i;
					found = true;
					break;
				}
			}
			if (!found) {
					ROS_WARN("Location not found");
			}
			return locations[ind].pose;
		}

		void setOrientation(double qx_, double qy_, double qz_, double qw_) {
			qx = qx_;
			qy = qy_;
			qz = qz_;
			qw = qw_;
		}

		double cartDist(double x, double y, double z, const coord& coord) {
			double sum = 0;
			sum += (x - coord.pose.position.x)*(x - coord.pose.position.x);
			sum += (y - coord.pose.position.y)*(y - coord.pose.position.y);
			sum += (z - coord.pose.position.z)*(z - coord.pose.position.z);
			sum = std::sqrt(sum);
			return sum;
		}

		int getNumLocs() {
			return locations.size();
		}

		bool getNearestLocLabel(geometry_msgs::Point loc, std::string& ret_coord_label) {
			double min_dist;
			int locations_ind;

			for (int i=0; i<locations.size(); ++i) {
				double temp_dist;
				temp_dist = cartDist(loc.x, loc.y, loc.z, locations[i]);

				if (i == 0 || temp_dist < min_dist) {
					min_dist = temp_dist;
					locations_ind = i;
				}
			}

			//std::cout << "max r: " <<max_r_arr[locations_ind] << " for location: " << locations_ind + 1 <<std::endl;
			//std::cout << "min dist: " << min_dist << std::endl;
			//std::cout << "x: " << loc.x << ", y: " << loc.y << ", z: " << loc.z << std::endl;

			if (min_dist < max_r_arr[locations_ind]) {
					ret_coord_label = locations[locations_ind].label;
					//is_occupied[locations_ind] = true;
					return true;
			} else {
					ROS_WARN("Did not find a location within the maximum radius");
					std::cout<<"Detection Radius for location "<<locations_ind<<": "<<max_r_arr[locations_ind]<<std::endl;
					std::cout<<"Minimum discovered distance: " << min_dist << std::endl;
					std::cout << "Coordinates: (x: " << loc.x << ", y: " << loc.y << ", z: " << loc.z << ")"<< std::endl;
					return false;
			}
		}

		bool getPredicates(const std::vector<geometry_msgs::Pose>* obj_locs, std::vector<std::string>& ret_state) {
			ret_state.clear();
			ret_state.resize(obj_locs->size());
			std::cout<<"ret_state in get pred: "<<ret_state.size()<<std::endl;
			for (int i=0; i<obj_locs->size(); ++i) {
				std::string temp_label;
				if (getNearestLocLabel((*obj_locs)[i].position, temp_label)){
					ret_state[i] = temp_label;
				} else {
					ROS_WARN("Cannot get predicates");
					return false;
				}
			}
			return true;
		}
};


int main(int argc, char **argv) {
	ros::init(argc, argv, "com_node");
	ros::NodeHandle com_NH;

    std::vector<std::string> obj_group;
    com_NH.getParam("/discrete_environment/obj_group", obj_group);

	std::vector<std::string> obj_topics(obj_group.size());
	for (int i=0; i<obj_group.size(); ++i) {
		obj_topics[i] = "/vrpn_client_node/" + obj_group[i] + "/pose";
	}
	
	//= {
	//	"/vrpn_client_node/greenBox_1/pose",
	//	"/vrpn_client_node/pinkBox_1/pose",
	//	"/vrpn_client_node/blueBox_1/pose",
	//	"/vrpn_client_node/pinkBox_2/pose",
	//};

	RetrieveData vicon_data(&com_NH, 30, obj_topics);
	PredicateGenerator pred_gen; 

	// Quaternion for downwards release:
	tf2::Quaternion q_init, q_up, q_side, q_rot_down, q_rot_side;
	q_init[0] = 0;
	q_init[1] = 0;
	q_init[2] = 1;
	q_init[3] = 0;
	//q_rot_down.setRPY(0, M_PI, -M_PI/4);
	//q_up = q_rot_down * q_init;
	q_up = q_init;
	geometry_msgs::Quaternion q_up_msg, q_side_msg;
	tf2::convert(q_up, q_up_msg);
	q_rot_side.setRPY(-M_PI, -M_PI/2, 0);
	q_side = q_rot_side * q_init;
	tf2::convert(q_side, q_side_msg);
	q_up.normalize();
	q_side.normalize();


    std::vector<std::string> location_names;
    com_NH.getParam("/discrete_environment/location_names", location_names);

    std::vector<std::map<std::string, float>> location_points(location_names.size());
    std::vector<std::string> location_orientation_types;
    com_NH.getParam("/discrete_environment/location_orientation_types", location_orientation_types);
    for (int i=0; i<location_names.size(); ++i) {
        com_NH.getParam("/discrete_environment/" + location_names[i] + "_point", location_points[i]);
        std::cout<<"Loaded point for: "<<location_names[i]<<std::endl;
        std::cout<<"  - x: "<<location_points[i].at("x")<<"\n";
        std::cout<<"  - y: "<<location_points[i].at("y")<<"\n";
        std::cout<<"  - z: "<<location_points[i].at("z")<<"\n";
        std::cout<<"  - Detection radius: "<<location_points[i].at("r")<<"\n";
        // Add condition if orientation type = 'manual'
		geometry_msgs::Point p;
		p.x = location_points[i].at("x");
		p.y = location_points[i].at("y");
		p.z = location_points[i].at("z");
		if (location_orientation_types[i] == "up") {
			pred_gen.addLocation(p, q_up_msg, location_names[i], location_points[i].at("r")); 
		} else if (location_orientation_types[i] == "side") {
			pred_gen.addLocation(p, q_side_msg, location_names[i], location_points[i].at("r")); 
		} else {
			std::string msg = "Did not find orientation preset:" + location_names[i];
			ROS_ERROR_STREAM(msg.c_str());
		}
    }

	ros::ServiceClient strategy_srv_client = com_NH.serviceClient<manipulation_interface::Strategy>("/com_node/strategy");
	manipulation_interface::Strategy strategy_srv;
	ros::ServiceClient plan_query_client = com_NH.serviceClient<manipulation_interface::PlanningQuery>("/planning_query");
	manipulation_interface::PlanningQuery plan_query_srv;

	std::vector<std::string> bag_domain_labels(obj_group.size());
	for (int i=0; i<obj_group.size(); ++i) { // TODO: Abstract domains in file
		bag_domain_labels[i] = "domain";
	}

	const std::vector<geometry_msgs::Pose>* data;
	std::string holding_state = "";
	geometry_msgs::Quaternion temp_orient;
	int failed_retries = 0;
	int max_failed_retries = 5;
	ros::Rate rate_fail(.2);
	while (ros::ok()) {
		vicon_data.retrieve();
		data = vicon_data.returnConfigArrPtr();

		std::vector<std::string> ret_state;
		bool found = pred_gen.getPredicates(data, ret_state);
		if (found) {
			ROS_INFO("Found predicates");
			failed_retries = 0;
		} else {
			if (failed_retries < max_failed_retries) {
				ROS_WARN("Did not find predicates, retrying in 5 seconds...");
				rate_fail.sleep();
				failed_retries++;
				continue;
			} else {
				ROS_ERROR("Did not find predicates (out of retries)");
				break;
			}
		}
		std::cout<<"Printing "<<ret_state.size()<<" predicates:"<<std::endl;
		for (int ii=0; ii<ret_state.size(); ii++) {
			std::cout<<" - "<<ret_state[ii]<<std::endl;
		}
		strategy_srv.request.world_config = ret_state;
		strategy_srv.request.prev_state = holding_state;
		if (strategy_srv_client.call(strategy_srv)) {
			std::string action = strategy_srv.response.action;
			int obj_ind = strategy_srv.response.obj;
			std::string to_loc = strategy_srv.response.to_loc;
			holding_state = strategy_srv.response.curr_state;
			std::cout<<"\nAction received: "<<action<<std::endl;
			std::cout<<"Obj Ind received: "<<obj_ind<<std::endl;
			std::cout<<"To location received: "<<to_loc<<"\n"<<std::endl;

			if (action == "transit") {
				plan_query_srv.request.manipulator_pose = (*data)[obj_ind];
				std::cout<<"size: "<<data->size()<<std::endl;
				for (int ii=0; ii<data->size(); ii++) {
					std::cout<<"data stuff for box"<<ii<<": "<<(*data)[1].position.x<<std::endl;
				}
				plan_query_srv.request.bag_poses = *data;
				plan_query_srv.request.setup_environment = true;
				plan_query_srv.request.bag_labels = obj_group;
				plan_query_srv.request.bag_domain_labels = bag_domain_labels;
				plan_query_srv.request.pickup_object = "none";
				plan_query_srv.request.grasp_type = "up";
				plan_query_srv.request.drop_object = "none";
				plan_query_srv.request.planning_domain = "domain";
				plan_query_srv.request.safe_config = false;
				plan_query_srv.request.go_to_raised = true;
			} else if (action == "transit_side") {
				plan_query_srv.request.manipulator_pose = (*data)[obj_ind];
				plan_query_srv.request.bag_poses = *data;
				plan_query_srv.request.setup_environment = true;
				plan_query_srv.request.bag_labels = obj_group;
				plan_query_srv.request.bag_domain_labels = bag_domain_labels;
				plan_query_srv.request.pickup_object = "none";
				plan_query_srv.request.grasp_type = "side";
				plan_query_srv.request.drop_object = "none";
				plan_query_srv.request.planning_domain = "domain";
				plan_query_srv.request.safe_config = false;
				plan_query_srv.request.go_to_raised = true;
			} else if (action == "transfer") {
				plan_query_srv.request.manipulator_pose = pred_gen.getLocation(to_loc);
				plan_query_srv.request.bag_poses = *data;
				plan_query_srv.request.setup_environment = true;
				plan_query_srv.request.bag_labels = obj_group;
				plan_query_srv.request.bag_domain_labels = bag_domain_labels;
				plan_query_srv.request.pickup_object = "none";
				// Setting the grasp type as "mode" uses the grasp type specified
				// in the last 'trasit' aciton, or the current grasp
				// mode of the system
				plan_query_srv.request.grasp_type = "mode";
				plan_query_srv.request.drop_object = "none";
				plan_query_srv.request.planning_domain = "domain";
				plan_query_srv.request.safe_config = false;
				plan_query_srv.request.go_to_raised = true;
				plan_query_srv.request.to_loc = to_loc;

			} else if (action == "grasp") {
				//plan_query_srv.request.manipulator_pose = (*data)[obj_ind];
				//plan_query_srv.request.bag_poses = data;
				plan_query_srv.request.setup_environment = false;
				//plan_query_srv.request.obj_group = obj_group;
				//plan_query_srv.request.bag_domain_labels = bag_domain_labels;
				std::cout<<"obj id: "<<obj_group[obj_ind]<<std::endl;
				plan_query_srv.request.pickup_object = obj_group[obj_ind];
				plan_query_srv.request.drop_object = "none";
				plan_query_srv.request.planning_domain = "domain";
				plan_query_srv.request.safe_config = false;
			} else if (action == "release") {
				//plan_query_srv.request.manipulator_pose = (*data)[obj_ind];
				// plan_query_srv.request.bag_poses = *data;
				plan_query_srv.request.setup_environment = false;
				//plan_query_srv.request.obj_group = obj_group;
				//plan_query_srv.request.bag_domain_labels = bag_domain_labels;
				plan_query_srv.request.pickup_object = "none";
				std::cout<<"obj id: "<<obj_group[obj_ind]<<std::endl;
				plan_query_srv.request.drop_object = obj_group[obj_ind];
				plan_query_srv.request.planning_domain = "domain";
				plan_query_srv.request.safe_config = false;
				plan_query_srv.request.to_loc = to_loc;
			} else {
				ROS_WARN("Unrecognized action");
			}
			if (plan_query_client.call(plan_query_srv)) {
				ROS_INFO("Completed action service");
			} else {
				ROS_WARN("Did not find plan query service");
			}
		} else {
			ROS_WARN("Did not find strategy service");
		}
	}
	return 0;
}
