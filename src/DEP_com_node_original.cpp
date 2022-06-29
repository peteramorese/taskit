#include <math.h>
#include "ros/ros.h"
#include "vicon_franka_integration/PlanningQuery_srv.h"
#include "vicon_franka_integration/Strategy.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"


class RetrieveData {
        private:
                class callbackdata {
			private:
				float x_offset = 0; //-.021544;
				float y_offset = 0;//.19416;
				float z_offset = 0; //.139;
                        public:
                                void sub_callback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg_ptr) {
                                        //std::cout<< "x: " << pose_msg_ptr->pose.position.x << std::endl;
                                        //std::cout<< "y: " << pose_msg_ptr->pose.position.y << std::endl;
                                        //std::cout<< "z: " << pose_msg_ptr->pose.position.z << std::endl;
                                        configptr->pose.position.x = pose_msg_ptr->pose.position.x + x_offset;
                                        configptr->pose.position.y = pose_msg_ptr->pose.position.y + y_offset;
                                        configptr->pose.position.z = pose_msg_ptr->pose.position.z + z_offset;
                                        configptr->pose.orientation.x = pose_msg_ptr->pose.orientation.x;
                                        configptr->pose.orientation.y = pose_msg_ptr->pose.orientation.y;
                                        configptr->pose.orientation.z = pose_msg_ptr->pose.orientation.z;
                                        configptr->pose.orientation.w = pose_msg_ptr->pose.orientation.w;
                                }
                                geometry_msgs::PoseStamped config;
                                geometry_msgs::PoseStamped* configptr = &config;
                };

                int Navg, Nboxes;
                bool hasdata;
                //geometry_msgs::PoseStamped avgConfig;
                ros::NodeHandle* SUB_NH;
		std::vector<callbackdata> sub_data;
		ros::Subscriber sub_box_1, sub_box_2, sub_box_3, sub_box_4, sub_box_5;
		geometry_msgs::PoseArray sample_pose_avg;
        public:
                RetrieveData(int Navg_, ros::NodeHandle* SUB_NH_) : Navg(Navg_), SUB_NH(SUB_NH_){
                        hasdata = false;
			sub_data.resize(4);
			std::cout<<"sub data size: "<<sub_data.size()<<std::endl;
			sub_box_3 = SUB_NH->subscribe("/vrpn_client_node/greenBox_1/pose", 10, &callbackdata::sub_callback, &sub_data[0]);
			sub_box_1 = SUB_NH->subscribe("/vrpn_client_node/pinkBox_1/pose", 10, &callbackdata::sub_callback, &sub_data[1]);
			sub_box_2 = SUB_NH->subscribe("/vrpn_client_node/blueBox_1/pose", 10, &callbackdata::sub_callback, &sub_data[2]);
			sub_box_4 = SUB_NH->subscribe("/vrpn_client_node/pinkBox_2/pose", 10, &callbackdata::sub_callback, &sub_data[3]);
			//sub_box_5 = SUB_NH->subscribe("/vrpn_client_node/blueBox_2/pose", 10, &callbackdata::sub_callback, &sub_data[4]);
			Nboxes = 4;
		}


		void retrieve() {
			//ros::Subscriber subscriber = SUB.subscribe("/vrpn_client_node/smallBox1/pose", 10, &callbackdata::sub_callback, &sub_data);

			sample_pose_avg.poses.clear();
			sample_pose_avg.poses.resize(Nboxes);
			for (int i=0; i<Nboxes; ++i) {
				geometry_msgs::Pose sample_pose;
				//sample_pose_avg.resize(7);
				sample_pose_avg.poses[i].position.x = 0;
				sample_pose_avg.poses[i].position.y = 0;
				sample_pose_avg.poses[i].position.z = 0;
				sample_pose_avg.poses[i].orientation.x = 0;
				sample_pose_avg.poses[i].orientation.y = 0;
				sample_pose_avg.poses[i].orientation.z = 0;
				sample_pose_avg.poses[i].orientation.w = 0;

				ros::Rate r(30);
				ros::spinOnce();
				int Navg_actual = 0;
				while (Navg_actual<Navg) {
					ros::spinOnce();
					if (sub_data[i].configptr->pose.position.x == 0.0){
						std::cout<<"Bad data"<<std::endl;
						r.sleep();
					} else {
						sample_pose.position.x += sub_data[i].configptr->pose.position.x;
						sample_pose.position.y += sub_data[i].configptr->pose.position.y;
						sample_pose.position.z += sub_data[i].configptr->pose.position.z;
						sample_pose.orientation.x += sub_data[i].configptr->pose.orientation.x;
						sample_pose.orientation.y += sub_data[i].configptr->pose.orientation.y;
						sample_pose.orientation.z += sub_data[i].configptr->pose.orientation.z;
						sample_pose.orientation.w += sub_data[i].configptr->pose.orientation.w;
						Navg_actual++;
						r.sleep();
					}
					if (!ros::ok()){
						break;
					}
				}
				sample_pose_avg.poses[i].position.x = sample_pose.position.x/Navg_actual;
				sample_pose_avg.poses[i].position.y = sample_pose.position.y/Navg_actual;
				sample_pose_avg.poses[i].position.z = sample_pose.position.z/Navg_actual;
				sample_pose_avg.poses[i].orientation.x = sample_pose.orientation.x/Navg_actual;
				sample_pose_avg.poses[i].orientation.y = sample_pose.orientation.y/Navg_actual;
				sample_pose_avg.poses[i].orientation.z = sample_pose.orientation.z/Navg_actual;
				sample_pose_avg.poses[i].orientation.w = sample_pose.orientation.w/Navg_actual;
				/*
				   for (int i=0; i<7; i++) {
				   sample_pose_avg[i] = sample_pose[i]/Navg_actual;
				   }
				   */
			}
			std::cout<<"sample pose size: "<<sample_pose_avg.poses.size()<<std::endl;
                        hasdata = true;
                }

                geometry_msgs::Pose* returnConfigPtr (int ind) {
                        if (!hasdata) {
                                std::cout<< "Call retrieve() before calling returnConfig()" <<std::endl;
                        } else {
                                return &sample_pose_avg.poses[ind];
                        }
		}

		geometry_msgs::PoseArray* returnConfigArrPtr() {
                        if (!hasdata) {
                                std::cout<< "Call retrieve() before calling returnConfig()" <<std::endl;
                        } else {
                                return &sample_pose_avg;
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
                PredicateGenerator() {
			locations.clear();
			max_r_arr.clear();
		}
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

		/*
                void getUnoccupiedLocation(geometry_msgs::Pose& ret_pose, std::string& ret_label) {
                        // This will return the first unoccupied location, then set
                        // that location to 'occupied'
                        for (int i=0; i<locations.size(); ++i) {
                                if (!is_occupied[i]) {
                                        ret_pose.position.x = locations[i].x;
                                        ret_pose.position.y = locations[i].y;
                                        ret_pose.position.z = locations[i].z;
                                        ret_pose.orientation.x = qx;
                                        ret_pose.orientation.y = qy;
                                        ret_pose.orientation.z = qz;
                                        ret_pose.orientation.w = qw;
                                        ret_label = locations[i].label;
                                        is_occupied[i] = true;
                                        break;
                                }
                        }
                }
		*/
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
		/*
                bool getNearestLocLabel(double x, double y, double z, std::string& ret_coord_label) {
                        // This will return the nearest location label, then set that location
                        // to 'occupied', so that the same nearest location label cannot be
                        // returned twice
                        double min_dist;
                        int locations_ind;
                        for (int i=0; i<locations.size(); ++i) {
                                if (!is_occupied[i]) {
                                        double temp_dist;
                                        temp_dist = cartDist(x, y, z, locations[i]);
                                        if (i == 0 || temp_dist < min_dist) {
                                                min_dist = temp_dist;
                                                locations_ind = i;
                                        }
                                }
                        }
                        if (min_dist < max_r) {
                                ret_coord_label = locations[locations_ind].label;
                                is_occupied[locations_ind] = true;
                                return true;
                        } else {
                                return false;
                                ROS_WARN("Did not find a location within the maximum radius");
                        }
                }
		*/
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
			std::cout<<"max r: "<<max_r_arr[locations_ind]<<" for location: "<<locations_ind + 1<<std::endl;
			std::cout<<"min dist: "<<min_dist<<std::endl;
                        std::cout << "x: " << loc.x << ", y: " << loc.y << ", z: " << loc.z << std::endl;
                        if (min_dist < max_r_arr[locations_ind]) {
                                ret_coord_label = locations[locations_ind].label;
                                //is_occupied[locations_ind] = true;
                                return true;
                        } else {
                                ROS_WARN("Did not find a location within the maximum radius");
                                return false;
                        }
		}
		bool getPredicates(geometry_msgs::PoseArray* obj_locs, std::vector<std::string>& ret_state) {
			ret_state.clear();
			ret_state.resize(obj_locs->poses.size());
			std::cout<<"ret_state in get pred: "<<ret_state.size()<<std::endl;
			bool found = true;
			for (int i=0; i<obj_locs->poses.size(); ++i) {
				std::string temp_label;
				if (getNearestLocLabel(obj_locs->poses[i].position, temp_label)){
					ret_state[i] = temp_label;
				} else {
					ROS_WARN("Cannot get predicates");
					found = false;
					break;
				}
			}
			return found;
		}
};


int main(int argc, char **argv) {
	ros::init(argc, argv, "com_node");
	ros::NodeHandle com_NH;

        RetrieveData vicon_data(30, &com_NH);
        PredicateGenerator pred_gen; // set detection radius to 15 cm
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
        q_rot_side.setRPY(-M_PI, -M_PI/2, 0);
        q_side = q_rot_side * q_init;
        tf2::convert(q_side, q_side_msg);
        q_down.normalize();
        q_side.normalize();

	/* 5 Location Arch */
        //std::vector<std::string> loc_labels = {"l1", "l2", "l3", "l4", "l5"};
        //geometry_msgs::Point p;
	//p.x = 0;
        //p.y = .35;
        //p.z = .185;
        //pred_gen.addLocation(p, q_side_msg, loc_labels[0], .08); // center of the arch

        //p.x = .075;
        //p.y = .4;
        //p.z = .08;
        //pred_gen.addLocation(p, q_down_msg, loc_labels[1], .05); // left of the arch

	//p.x = -.075;
        //p.y = .4;
        //p.z = .08;
        //pred_gen.addLocation(p, q_down_msg, loc_labels[2], .05); // right of the arch

	//p.x = .5;
        //p.y = -.25;
        //p.z = .08;
        //pred_gen.addLocation(p, q_down_msg, loc_labels[3], .15); // L4

        //p.x = .5;
        //p.y = .25;
        //p.z = .08;
        //pred_gen.addLocation(p, q_down_msg, loc_labels[4], .15); // L5

	/* Dual Arch */
        // std::vector<std::string> loc_labels = {"l0", "l8", "l9", "l1", "l2", "l3", "l4", "l6", "l7"};
        // geometry_msgs::Point p;
	// // Left Arch (robot region)
	// p.x = 0 + .45;
        // p.y = .35;
        // p.z = .185 + .006;
        // pred_gen.addLocation(p, q_side_msg, loc_labels[0], .1); // center of the arch

        // p.x = .075 + .45;
        // p.y = .35;
        // p.z = .085 + .002;
        // // pred_gen.addLocation(p, q_down_msg, loc_labels[1], .05); // left of the arch
        // pred_gen.addLocation(p, q_down_msg, loc_labels[1], .15); // left of the arch

	// p.x = -.075 + .45;
        // p.y = .35;
        // p.z = .085 + .002;
        // // pred_gen.addLocation(p, q_down_msg, loc_labels[2], .05); // right of the arch
        // pred_gen.addLocation(p, q_down_msg, loc_labels[2], .15); // right of the arch

	// // Right Arch (human region)
	// p.x = 0 - .45;
        // p.y = .35;
        // p.z = .185 + .002;
        // pred_gen.addLocation(p, q_side_msg, loc_labels[3], .1); // center of the arch

        // p.x = .075 - .45;
        // p.y = .35;
        // p.z = .085;
        // // pred_gen.addLocation(p, q_down_msg, loc_labels[4], .05); // left of the arch
        // pred_gen.addLocation(p, q_down_msg, loc_labels[4], .15); // left of the arch

	// p.x = -.075 - .45;
        // p.y = .35;
        // p.z = .085;
        // // pred_gen.addLocation(p, q_down_msg, loc_labels[5], .05); // right of the arch
        // pred_gen.addLocation(p, q_down_msg, loc_labels[5], .15); // right of the arch

	// p.x = -.075 - .45;
        // p.y = 0;
        // p.z = .085;
        // pred_gen.addLocation(p, q_down_msg, loc_labels[6], .15); // l4

	// p.x = .50;
        // p.y = -.35;
        // p.z = .085;
        // pred_gen.addLocation(p, q_down_msg, loc_labels[7], .15); // l6

	// p.x = .60;
        // p.y = -.1;
        // p.z = .085;
        // pred_gen.addLocation(p, q_down_msg, loc_labels[8], .15); // l7

        // -------------------------------------------------------------------- //
        //                      Kandai's Location Mappings                      //
        // -------------------------------------------------------------------- //
        std::vector<std::string> loc_labels = {"l0", "l1", "l2", "l3", "l4", "l5", "l6", "l7", "l8"};
        geometry_msgs::Point p;

	p.x = .50;
        p.y = -.35;
        p.z = .08;
        pred_gen.addLocation(p, q_down_msg, loc_labels[0], .15); // l0

        p.x = .075 + .45;
        p.y = .35;
        p.z = .08 + .002;
        pred_gen.addLocation(p, q_down_msg, loc_labels[1], .15); // l1

	// p.x = -.075 - .45;
        // p.y = .35;
        // p.z = .085;
        p.x = 0;
        p.y = .45;
        p.z = .08;
        pred_gen.addLocation(p, q_down_msg, loc_labels[2], .15); // l2

        // p.x = -.5;
        // p.y = -.35;
        // p.z = .085;
        p.x = 0;
        p.y = -.45;
        p.z = .08;
        pred_gen.addLocation(p, q_down_msg, loc_labels[3], .15); // l3

        p.x = 0;
        p.y = .45;
        p.z = .08;
        pred_gen.addLocation(p, q_down_msg, loc_labels[4], .15); // l4

        p.x = 0.7;
        p.y = -.12;
        p.z = .09;
        pred_gen.addLocation(p, q_down_msg, loc_labels[5], .15); // l5

        p.x = 0.7;
        p.y = .12;
        p.z = .09;
        pred_gen.addLocation(p, q_down_msg, loc_labels[6], .15); // l6

        p.x = 0.55;
        p.y = .12;
        p.z = .09;
        pred_gen.addLocation(p, q_down_msg, loc_labels[7], .15); // l7

        p.x = 0.55;
        p.y = -.12;
        p.z = .09;
        pred_gen.addLocation(p, q_down_msg, loc_labels[8], .15); // l8

        // -------------------------------------------------------------------- //
        //                      Kandai's Location Mappings                      //
        // -------------------------------------------------------------------- //

	/* Diagonal Placement */
        //std::vector<std::string> loc_labels = {"l0", "l1", "l2", "l3", "l4", "l5", "l6", "l7"};
        //geometry_msgs::Point p;
	//p.x = .50;
        //p.y = -.35;
        //p.z = .085;
        //pred_gen.addLocation(p, q_down_msg, loc_labels[0], .15); // l0

        //p.x = .50;
        //p.y = 0.0;
        //p.z = .085;
        //pred_gen.addLocation(p, q_down_msg, loc_labels[1], .15); // l1

	//p.x = .50;
        //p.y = .35;
        //p.z = .085;
        //pred_gen.addLocation(p, q_down_msg, loc_labels[2], .15); // l2

	//p.x = 0.0;//.20;
        //p.y = .35;
        //p.z = .085;
        //pred_gen.addLocation(p, q_down_msg, loc_labels[3], .15); // l3

        //p.x = -.50;
        //p.y = -.35;
        //p.z = .085;
        //pred_gen.addLocation(p, q_down_msg, loc_labels[4], .15); // l4

        //p.x = -.50;
        //p.y = 0.0;
        //p.z = .085;
        //pred_gen.addLocation(p, q_down_msg, loc_labels[5], .15); // l5

        //p.x = -.50;
        //p.y = .35;
        //p.z = .085;
        //pred_gen.addLocation(p, q_down_msg, loc_labels[6], .15); // l6

	//p.x = -.20;
        //p.y = .35;
        //p.z = .08;
        //pred_gen.addLocation(p, q_down_msg, loc_labels[7], .15); // l7

	ros::ServiceClient strategy_srv_client = com_NH.serviceClient<vicon_franka_integration::Strategy>("/com_node/strategy");
	vicon_franka_integration::Strategy strategy_srv;
	ros::ServiceClient plan_query_client = com_NH.serviceClient<vicon_franka_integration::PlanningQuery_srv>("/planning_query");
	vicon_franka_integration::PlanningQuery_srv plan_query_srv;

	geometry_msgs::PoseArray* data = vicon_data.returnConfigArrPtr();
	int j = 0;
	//std::vector<std::string> bag_labels = {"pinkBox_1", "blueBox_1", "greenBox_1", "pinkBox_2", "blueBox_2"};
	std::vector<std::string> bag_labels = {"greenBox_1", "pinkBox_1", "blueBox_1", "pinkBox_2"};
	//std::vector<std::string> bag_domain_labels = {"domain", "domain", "domain", "domain", "domain"};
	std::vector<std::string> bag_domain_labels = {"domain", "domain", "domain", "domain"};
	std::string holding_state = "";
	geometry_msgs::Quaternion temp_orient;
	while (ros::ok()) {
		vicon_data.retrieve();
		std::vector<std::string> ret_state;
		bool found = pred_gen.getPredicates(data, ret_state);
		if (found) {
			ROS_INFO("Found predicates");
		} else {
			ROS_WARN("Did not find predicates, breaking...");
			break;
		}

		std::cout<<"ret_state size: "<<ret_state.size()<<std::endl;
		for (int ii=0; ii<ret_state.size(); ii++) {
			std::cout<<"ret_state: "<<ret_state[ii]<<std::endl;
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
				plan_query_srv.request.manipulator_pose = data->poses[obj_ind];
				std::cout<<"size: "<<data->poses.size()<<std::endl;
				for (int ii=0; ii<data->poses.size(); ii++) {
					std::cout<<"data stuff for box"<<ii<<": "<<data->poses[1].position.x<<std::endl;
				}
				plan_query_srv.request.bag_poses = *data;
				plan_query_srv.request.setup_environment = true;
				plan_query_srv.request.bag_labels = bag_labels;
				plan_query_srv.request.bag_domain_labels = bag_domain_labels;
				plan_query_srv.request.pickup_object = "none";
				plan_query_srv.request.grasp_type = "up";
				plan_query_srv.request.drop_object = "none";
				plan_query_srv.request.planning_domain = "domain";
				plan_query_srv.request.safe_config = false;
				plan_query_srv.request.go_to_raised = true;
			} else if (action == "transit_side") {
				plan_query_srv.request.manipulator_pose = data->poses[obj_ind];
				plan_query_srv.request.bag_poses = *data;
				plan_query_srv.request.setup_environment = true;
				plan_query_srv.request.bag_labels = bag_labels;
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
				plan_query_srv.request.bag_labels = bag_labels;
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

			} else if (action == "grasp") {
				//plan_query_srv.request.manipulator_pose = data->poses[obj_ind];
				//plan_query_srv.request.bag_poses = data;
				plan_query_srv.request.setup_environment = false;
				//plan_query_srv.request.bag_labels = bag_labels;
				//plan_query_srv.request.bag_domain_labels = bag_domain_labels;
				std::cout<<"obj id: "<<bag_labels[obj_ind]<<std::endl;
				plan_query_srv.request.pickup_object = bag_labels[obj_ind];
				plan_query_srv.request.drop_object = "none";
				plan_query_srv.request.planning_domain = "domain";
				plan_query_srv.request.safe_config = false;
			} else if (action == "release") {
				//plan_query_srv.request.manipulator_pose = data->poses[obj_ind];
				//plan_query_srv.request.bag_poses = data;
				plan_query_srv.request.setup_environment = false;
				//plan_query_srv.request.bag_labels = bag_labels;
				//plan_query_srv.request.bag_domain_labels = bag_domain_labels;
				plan_query_srv.request.pickup_object = "none";
				std::cout<<"obj id: "<<bag_labels[obj_ind]<<std::endl;
				plan_query_srv.request.drop_object = bag_labels[obj_ind];
				plan_query_srv.request.planning_domain = "domain";
				plan_query_srv.request.safe_config = false;
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
