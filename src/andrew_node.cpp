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

#include "edge.h"
#include "astar.h"
#include "state.h"
#include "stateSpace.h"
#include "condition.h"
#include "transitionSystem.h"


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
		ros::Subscriber sub_box_1, sub_box_2, sub_box_3;
		geometry_msgs::PoseArray sample_pose_avg;
        public:
                RetrieveData(int Navg_, ros::NodeHandle* SUB_NH_) : Navg(Navg_), SUB_NH(SUB_NH_){
                        hasdata = false;
			sub_data.resize(3);
			std::cout<<"sub data size: "<<sub_data.size()<<std::endl;
			//sub_box_1 = SUB_NH->subscribe("/vrpn_client_node/pinkBox_1/pose", 10, &callbackdata::sub_callback, &sub_data[0]);
			//sub_box_2 = SUB_NH->subscribe("/vrpn_client_node/pinkBox_2/pose", 10, &callbackdata::sub_callback, &sub_data[1]);
			sub_box_1 = SUB_NH->subscribe("/vrpn_client_node/blueBox_1/pose", 10, &callbackdata::sub_callback, &sub_data[0]);
			sub_box_2 = SUB_NH->subscribe("/vrpn_client_node/blueBox_2/pose", 10, &callbackdata::sub_callback, &sub_data[1]);
			sub_box_3 = SUB_NH->subscribe("/vrpn_client_node/greenBox_1/pose", 10, &callbackdata::sub_callback, &sub_data[2]);
			Nboxes = 3;
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
			std::cout<<"max r: "<<max_r_arr[locations_ind]<<" for location: "<<locations_ind<<std::endl;
			std::cout<<"min dist: "<<min_dist<<std::endl;
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
	ros::init(argc, argv, "andrew_node");
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
	q_rot_side.setRPY(-M_PI/2, 0, 0);
	q_side = q_rot_side * q_init;
	tf2::convert(q_side, q_side_msg);
	q_down.normalize();
	q_side.normalize();

        std::vector<std::string> loc_labels = {"L0", "L1", "L2", "temp_L1", "temp_L2", "temp_L3"};
        std::vector<std::string> side_loc_labels;
	geometry_msgs::Point p;
	p.x = .27;
	p.y = -.22;
	p.z = .083;
	pred_gen.addLocation(p, q_down_msg, loc_labels[0], .15); // L0

	p.x = .41;
	p.y = -.22;
	p.z = .083;
	pred_gen.addLocation(p, q_down_msg, loc_labels[1], .15); // L1

	p.x = .56;
	p.y = -.22;
	p.z = .083;
	pred_gen.addLocation(p, q_down_msg, loc_labels[2], .15); // L2

	p.x = 0.0;
	p.y = .4;
	p.z = .083;
	pred_gen.addLocation(p, q_down_msg, loc_labels[3], .15); // L2

	p.x = 0.15;
	p.y = .4;
	p.z = .083;
	pred_gen.addLocation(p, q_down_msg, loc_labels[4], .15); // L2

	p.x = 0.3;
	p.y = .4;
	p.z = .083;
	pred_gen.addLocation(p, q_down_msg, loc_labels[5], .15); // L2

	//ros::ServiceClient strategy_srv_client = com_NH.serviceClient<vicon_franka_integration::Strategy>("/com_node/strategy");
	vicon_franka_integration::Strategy strategy_srv;	
	ros::ServiceClient plan_query_client = com_NH.serviceClient<vicon_franka_integration::PlanningQuery_srv>("/planning_query");
	vicon_franka_integration::PlanningQuery_srv plan_query_srv;	

	geometry_msgs::PoseArray* data = vicon_data.returnConfigArrPtr();
	int j = 0;
	std::vector<std::string> bag_labels = {"blueBox_1", "blueBox_2", "greenBox_1"};
	
	std::string holding_state = "";
	geometry_msgs::Quaternion temp_orient;

	// Observe (observe once at the beginning)
	vicon_data.retrieve();
	std::vector<std::string> ret_state;
	bool found = pred_gen.getPredicates(data, ret_state);
	if (found) {
		ROS_INFO("Found predicates");	
	} else {
		ROS_WARN("Did not find predicates, breaking...");
		return 1;
	}
	std::cout<<"ret_state size: "<<ret_state.size()<<std::endl;
	for (int ii=0; ii<ret_state.size(); ii++) {
		std::cout<<"ret_state: "<<ret_state[ii]<<std::endl;
	}

	// Plan	
        /* CREATE ENVIRONMENT FOR MANIPULATOR */
        BlockingStateSpace SS_MANIPULATOR;

        std::vector<std::string> ee_labels = loc_labels;
        ee_labels.push_back("stow");
        std::vector<std::string> obj_labels = loc_labels;
        obj_labels.push_back("ee");
        std::vector<std::string> grip_labels = {"true","false"};
        std::vector<std::string> grip_type_labels = {"none","up","side"};

        // Create state space:
        SS_MANIPULATOR.setStateDimension(ee_labels, 0); // eef
        SS_MANIPULATOR.setStateDimension(obj_labels, 1); // box1
        SS_MANIPULATOR.setStateDimension(obj_labels, 2); // box2
        SS_MANIPULATOR.setStateDimension(obj_labels, 3); // box3
        SS_MANIPULATOR.setStateDimension(grip_labels, 4); // eef engaged
        SS_MANIPULATOR.setStateDimension(grip_type_labels, 5); // grip type

        // Label state space:
        SS_MANIPULATOR.setStateDimensionLabel(0, "eeLoc");
        SS_MANIPULATOR.setStateDimensionLabel(1, "blueBox_1");
        SS_MANIPULATOR.setStateDimensionLabel(2, "blueBox_2");
        SS_MANIPULATOR.setStateDimensionLabel(3, "greenBox_1");
        SS_MANIPULATOR.setStateDimensionLabel(4, "holding");
        SS_MANIPULATOR.setStateDimensionLabel(5, "grip_type");

        // Create object location group:
        std::vector<std::string> obj_group = {"blueBox_1", "blueBox_2", "greenBox_1"};
        SS_MANIPULATOR.setLabelGroup("object locations", obj_group);

	// Create sideways location domain
	SS_MANIPULATOR.setDomain("side locations", side_loc_labels);

        // Set the initial state:
        std::vector<std::string> set_state = {"stow"};
	for (int i=0; i<ret_state.size(); ++i){
		set_state.push_back(ret_state[i]);
	}
	set_state.push_back("false");
	set_state.push_back("none");
	std::cout<<"printing set state"<<std::endl;
	for (int i=0; i<set_state.size(); ++i){
		std::cout<<set_state[i]<<std::endl;
	}
        BlockingState init_state(&SS_MANIPULATOR);
	std::vector<bool> blocking_dims = {false, true, true, false, true, true};
        init_state.setBlockingDim(blocking_dims);
	init_state.toggleDebug(false);
        init_state.setState(set_state);

        //State test_state(&SS_MANIPULATOR);    
        //test_state.setState(test_set_state);

        /* SET CONDITIONS */
        // Pickup domain conditions:
        std::vector<Condition> conds_m;
        std::vector<Condition*> cond_ptrs_m;
        conds_m.resize(6);
        cond_ptrs_m.resize(6);

        // Grasp regular
        conds_m[0].addCondition(Condition::PRE, Condition::LABEL, "holding", Condition::EQUALS, Condition::VAR, "false");
        //conds_m[0].addCondition(Condition::PRE, Condition::LABEL, "grip_type", Condition::EQUALS, Condition::VAR, "none", Condition::NEGATE, "not_gripping");
        conds_m[0].addCondition(Condition::PRE, Condition::GROUP, "object locations", Condition::ARG_FIND, Condition::LABEL, "eeLoc",Condition::TRUE, "arg");
        conds_m[0].setCondJunctType(Condition::PRE, Condition::CONJUNCTION);

        conds_m[0].addCondition(Condition::POST, Condition::ARG_L, Condition::FILLER, Condition::ARG_EQUALS, Condition::VAR, "ee",Condition::TRUE, "arg");
        conds_m[0].addCondition(Condition::POST, Condition::LABEL, "holding", Condition::EQUALS, Condition::VAR, "true");
        conds_m[0].setCondJunctType(Condition::POST, Condition::CONJUNCTION);
        conds_m[0].setActionLabel("grasp");
	
	
        // Transport Up
        //conds_m[1].addCondition(Condition::PRE, Condition::LABEL, "holding", Condition::EQUALS, Condition::VAR, "true");
        //conds_m[1].addCondition(Condition::PRE, Condition::GROUP, "object locations", Condition::ARG_FIND, Condition::LABEL, "eeLoc", Condition::NEGATE, "arg1");
        //conds_m[1].addCondition(Condition::PRE, Condition::LABEL, "eeLoc", Condition::ARG_FIND, Condition::NONE, Condition::FILLER, Condition::TRUE, "arg2");
        //conds_m[1].setCondJunctType(Condition::PRE, Condition::CONJUNCTION); // Used to store eeLoc pre-state variable
        //conds_m[1].addCondition(Condition::POST, Condition::ARG_V, Condition::FILLER, Condition::ARG_EQUALS, Condition::LABEL, "eeLoc", Condition::NEGATE, "arg2"); // Stored eeLoc pre-state variable is not the same as post-state eeLoc (eeLoc has moved)
        //conds_m[1].addCondition(Condition::POST, Condition::GROUP, "object locations", Condition::ARG_FIND, Condition::LABEL, "eeLoc", Condition::NEGATE,"na");
        //conds_m[1].setCondJunctType(Condition::POST, Condition::CONJUNCTION);
        //conds_m[1].setActionLabel("transport");
        //conds_m[1].print();
	
	// Transport Up
        conds_m[1].addCondition(Condition::PRE, Condition::LABEL, "holding", Condition::EQUALS, Condition::VAR, "true");
        conds_m[1].addCondition(Condition::PRE, Condition::LABEL, "grip_type", Condition::EQUALS, Condition::VAR, "up");
        conds_m[1].addCondition(Condition::PRE, Condition::GROUP, "object locations", Condition::ARG_FIND, Condition::LABEL, "eeLoc", Condition::NEGATE, "arg1");
        conds_m[1].addCondition(Condition::PRE, Condition::LABEL, "eeLoc", Condition::ARG_FIND, Condition::NONE, Condition::FILLER, Condition::TRUE, "arg2");
        conds_m[1].setCondJunctType(Condition::PRE, Condition::CONJUNCTION); // Used to store eeLoc pre-state variable
        conds_m[1].addCondition(Condition::POST, Condition::ARG_V, Condition::FILLER, Condition::ARG_EQUALS, Condition::LABEL, "eeLoc", Condition::NEGATE, "arg2"); // Stored eeLoc pre-state variable is not the same as post-state eeLoc (eeLoc has moved)
        conds_m[1].addCondition(Condition::POST, Condition::LABEL, "eeLoc", Condition::IN_DOMAIN, Condition::DOMAIN, "side locations", Condition::NEGATE, "na"); // Not in side locations
        conds_m[1].addCondition(Condition::POST, Condition::LABEL, "grip_type", Condition::EQUALS, Condition::VAR, "none");
        conds_m[1].addCondition(Condition::POST, Condition::GROUP, "object locations", Condition::ARG_FIND, Condition::LABEL, "eeLoc", Condition::NEGATE,"na");
        conds_m[1].setCondJunctType(Condition::POST, Condition::CONJUNCTION);
        conds_m[1].setActionLabel("transport");
        //conds_m[1].print();

        // Release 
        conds_m[2].addCondition(Condition::PRE, Condition::LABEL, "holding", Condition::EQUALS, Condition::VAR, "true");
        conds_m[2].addCondition(Condition::PRE, Condition::GROUP, "object locations", Condition::ARG_FIND, Condition::LABEL, "eeLoc", Condition::NEGATE, "arg1");
        conds_m[2].addCondition(Condition::PRE, Condition::GROUP, "object locations", Condition::ARG_FIND, Condition::VAR, "ee",Condition::TRUE, "arg2");
        conds_m[2].setCondJunctType(Condition::PRE, Condition::CONJUNCTION);

        conds_m[2].addCondition(Condition::POST, Condition::ARG_L, Condition::FILLER, Condition::ARG_EQUALS, Condition::LABEL, "eeLoc", Condition::TRUE, "arg2");
        conds_m[2].addCondition(Condition::POST, Condition::LABEL, "holding", Condition::EQUALS, Condition::VAR, "false");
        conds_m[2].setCondJunctType(Condition::POST, Condition::CONJUNCTION);
        conds_m[2].setActionLabel("release");
        //conds_m[2].print();



        // Transit Up
        conds_m[3].addCondition(Condition::PRE, Condition::LABEL, "holding", Condition::EQUALS, Condition::VAR, "false");
        conds_m[3].addCondition(Condition::PRE, Condition::LABEL, "grip_type", Condition::EQUALS, Condition::VAR, "none");
        conds_m[3].addCondition(Condition::PRE, Condition::LABEL, "eeLoc", Condition::ARG_FIND, Condition::NONE, Condition::FILLER, Condition::TRUE, "arg");
        conds_m[3].setCondJunctType(Condition::PRE, Condition::CONJUNCTION);

        conds_m[3].addCondition(Condition::POST, Condition::ARG_V, Condition::FILLER, Condition::ARG_EQUALS, Condition::LABEL, "eeLoc", Condition::NEGATE,"arg");
        conds_m[3].addCondition(Condition::POST, Condition::LABEL, "eeLoc", Condition::IN_DOMAIN, Condition::DOMAIN, "side locations", Condition::NEGATE, "arg_2");
        conds_m[3].addCondition(Condition::POST, Condition::LABEL, "grip_type", Condition::EQUALS, Condition::VAR, "up");
        conds_m[3].setCondJunctType(Condition::POST, Condition::CONJUNCTION);
        conds_m[3].setActionLabel("transit_up");
        //conds_m[3].print();



	// Transit Side
        conds_m[4].addCondition(Condition::PRE, Condition::LABEL, "holding", Condition::EQUALS, Condition::VAR, "false");
        conds_m[4].addCondition(Condition::PRE, Condition::LABEL, "grip_type", Condition::EQUALS, Condition::VAR, "none");
        conds_m[4].addCondition(Condition::PRE, Condition::LABEL, "eeLoc", Condition::ARG_FIND, Condition::NONE, Condition::FILLER, Condition::TRUE, "arg");
        conds_m[4].setCondJunctType(Condition::PRE, Condition::CONJUNCTION);

        conds_m[4].addCondition(Condition::POST, Condition::ARG_V, Condition::FILLER, Condition::ARG_EQUALS, Condition::LABEL, "eeLoc", Condition::NEGATE,"arg");
        conds_m[4].addCondition(Condition::POST, Condition::LABEL, "grip_type", Condition::EQUALS, Condition::VAR, "side");
        conds_m[4].setCondJunctType(Condition::POST, Condition::CONJUNCTION);
        conds_m[4].setActionLabel("transit_side");
        //conds_m[3].print();

	// Transport Side
        conds_m[5].addCondition(Condition::PRE, Condition::LABEL, "holding", Condition::EQUALS, Condition::VAR, "true");
        conds_m[5].addCondition(Condition::PRE, Condition::LABEL, "grip_type", Condition::EQUALS, Condition::VAR, "side");
        conds_m[5].addCondition(Condition::PRE, Condition::GROUP, "object locations", Condition::ARG_FIND, Condition::LABEL, "eeLoc", Condition::NEGATE, "arg1");
        conds_m[5].addCondition(Condition::PRE, Condition::LABEL, "eeLoc", Condition::ARG_FIND, Condition::NONE, Condition::FILLER, Condition::TRUE, "arg2");
        conds_m[5].setCondJunctType(Condition::PRE, Condition::CONJUNCTION); // Used to store eeLoc pre-state variable
        conds_m[5].addCondition(Condition::POST, Condition::ARG_V, Condition::FILLER, Condition::ARG_EQUALS, Condition::LABEL, "eeLoc", Condition::NEGATE, "arg2"); // Stored eeLoc pre-state variable is not the same as post-state eeLoc (eeLoc has moved)
        conds_m[5].addCondition(Condition::POST, Condition::LABEL, "eeLoc", Condition::IN_DOMAIN, Condition::DOMAIN, "side locations", Condition::TRUE, "na");
        conds_m[5].addCondition(Condition::POST, Condition::LABEL, "grip_type", Condition::EQUALS, Condition::VAR, "none");
        conds_m[5].addCondition(Condition::POST, Condition::GROUP, "object locations", Condition::ARG_FIND, Condition::LABEL, "eeLoc", Condition::NEGATE,"na");
        conds_m[5].setCondJunctType(Condition::POST, Condition::CONJUNCTION);
        conds_m[5].setActionLabel("transport");
        //conds_m[1].print();

        // Grasp from side
        //conds_m[5].addCondition(Condition::PRE, Condition::LABEL, "holding", Condition::EQUALS, Condition::VAR, "false");
        //conds_m[5].addCondition(Condition::PRE, Condition::LABEL, "grip_type", Condition::EQUALS, Condition::VAR, "none");
        //conds_m[5].addCondition(Condition::PRE, Condition::GROUP, "object locations", Condition::ARG_FIND, Condition::LABEL, "eeLoc",Condition::TRUE, "arg");
        //conds_m[5].setCondJunctType(Condition::PRE, Condition::CONJUNCTION);

        //conds_m[5].addCondition(Condition::POST, Condition::ARG_L, Condition::FILLER, Condition::ARG_EQUALS, Condition::VAR, "ee",Condition::TRUE, "arg");
        //conds_m[5].addCondition(Condition::POST, Condition::LABEL, "holding", Condition::EQUALS, Condition::VAR, "true");
        //conds_m[5].addCondition(Condition::POST, Condition::LABEL, "grip_type", Condition::EQUALS, Condition::VAR, "side");
        //conds_m[5].setCondJunctType(Condition::POST, Condition::CONJUNCTION);
        //conds_m[5].setActionLabel("grasp");
	

        for (int i=0; i<conds_m.size(); ++i){
                cond_ptrs_m[i] = &conds_m[i];
        }


        /* Propositions */
        std::cout<<"Setting Atomic Propositions... "<<std::endl;
        std::vector<SimpleCondition> AP(loc_labels.size() * obj_group.size());
        std::vector<SimpleCondition*> AP_ptrs(loc_labels.size() * obj_group.size());
	std::cout<<"size : "<<loc_labels.size() * obj_group.size()<<std::endl;
        for (int i=0; i<loc_labels.size(); ++i) {
		for (int ii=0; ii<obj_group.size(); ++ii) {
			// wrap indices
			AP[i*obj_group.size() + ii].addCondition(Condition::SIMPLE, Condition::LABEL, obj_group[ii], Condition::EQUALS, Condition::VAR, loc_labels[i]);
			AP[i*obj_group.size() + ii].addCondition(Condition::SIMPLE, Condition::LABEL, "holding", Condition::EQUALS, Condition::VAR, "false");
			AP[i*obj_group.size() + ii].setCondJunctType(Condition::SIMPLE, Condition::CONJUNCTION);
			// propositon format i.e.: box1_L1
			AP[i*obj_group.size() + ii].setLabel(obj_group[ii] + "_" + loc_labels[i]);
			//std::cout<<i*obj_group.size() + ii<<std::endl;
		}
	}
        for (int i=0; i<AP.size(); ++i) {
                AP_ptrs[i] = &AP[i];
        }
	std::cout<<"af aps"<<std::endl;


	/* DFA_m & Graph Instantiations For Manipulator */
        Edge TS(true);
        Edge DFA(true);
        Edge PS(true);

        // Hard-code DFA_m automaton:
        //DFA.connect(1, 1, 1.0, "!box1_L1 | !box2_L2 | !box3_L3");
        //DFA.connect(1, 0, 1.0, "box1_L1 & box2_L2 & box3_L3");
	
        //DFA.connect(1, 1, 1.0, "!box3_arch_left | !box2_arch_right");
        //DFA.connect(1, 0, 1.0, "box3_arch_left & box2_arch_right & !box1_arch_c");
        //DFA.connect(0, 0, 1.0, "!box1_arch_c");
        //DFA.connect(0, 2, 1.0, "box1_arch_c");
        DFA.connect(0, 1, 1.0, "blueBox_1_L1 & !blueBox_2_L0 & !greenBox_1_L2");
        DFA.connect(0, 0, 1.0, "!blueBox_1_L1 & !blueBox_2_L0 & !greenBox_1_L2");
        DFA.connect(1, 1, 1.0, "!blueBox_2_L0 & !greenBox_1_L2");
        DFA.connect(1, 2, 1.0, "blueBox_2_L0 & !greenBox_1_L2");
        DFA.connect(2, 2, 1.0, "!greenBox_1_L2");
        DFA.connect(2, 3, 1.0, "greenBox_1_L2");
	DFA.print();
	

        //DFA_m1.connect(2, 2, 1.0, "!p_aL4 & !p_rL3");
        //DFA_m1.connect(2, 1, 1.0, "p_aL4 & !p_rL3");
        //DFA_m1.connect(2, 3, 1.0, "!p_aL4 & p_rL3");
        //DFA_m1.connect(1, 1, 1.0, "!p_rL3");
        //DFA_m1.connect(1, 0, 1.0, "p_rL3");
        //DFA_m1.connect(3, 3, 1.0, "!p_aL4");
        //DFA_m1.connect(3, 0, 1.0, "p_aL4");
        //DFA_m1.print();
        //std::cout<<"DFA 1 listsize: "<<DFA_m1.returnListCount()<<std::endl;

        // Define the product systems after the automatons have been defined
        ProductSystem<BlockingState> PRODSYS(&TS, &DFA, &PS);


        // Set the pre and post conditions, same for both systems in this case
        PRODSYS.setConditions(cond_ptrs_m);

        // Set the atomic propositions and define the initial and accepting states
        PRODSYS.setPropositions(AP_ptrs);
        PRODSYS.setAutomatonInitStateIndex(0);
        PRODSYS.addAutomatonAcceptingStateIndex(3);

	PRODSYS.setInitState(&init_state);
	PRODSYS.generate();
	//PRODSYS.printTS();
	PRODSYS.compose();
	bool plan_found = PRODSYS.plan();
	
	std::vector<BlockingState*> state_seq;
	std::vector<std::string> act_seq;

	PRODSYS.getPlan(state_seq, act_seq);
	std::cout<<"hello"<<std::endl;
	
	std::vector<std::string> bag_domain_labels(obj_group.size());
	for (int i=0; i<obj_group.size(); ++i) {
		bag_domain_labels[i] = "domain";	
	}
	std::cout<<"hello1"<<std::endl;

	// Execute
	for (int i=0; i<act_seq.size();	++i) {
			//std::string act_seq[i] = strategy_srv.response.act_seq[i];
			//int obj_ind = strategy_srv.response.obj;
			//std::string to_loc = strategy_srv.response.to_loc;
			//holding_state = strategy_srv.response.curr_state;
		std::cout<<"Action received: "<<act_seq[i]<<std::endl;
		vicon_data.retrieve();
		//if (i == 0) {
		//	plan_query_srv.request.setup_environment = true;
		//} else {
		//	plan_query_srv.request.setup_environment = false;
		//}
		plan_query_srv.request.setup_environment = true;
		if (act_seq[i] == "transit_up") {
			ros::WallDuration(3.0).sleep();
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
			plan_query_srv.request.manipulator_pose = data->poses[pose_ind];
			temp_orient = plan_query_srv.request.manipulator_pose.orientation;
			plan_query_srv.request.bag_poses = *data;
			plan_query_srv.request.bag_labels = obj_group;
			plan_query_srv.request.bag_domain_labels = bag_domain_labels;
			plan_query_srv.request.pickup_object = "none";
			plan_query_srv.request.grasp_type = "up";
			plan_query_srv.request.drop_object = "none";
			plan_query_srv.request.planning_domain = "domain";
			plan_query_srv.request.safe_config = false;
		} else if (act_seq[i] == "transit_side") {
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
			plan_query_srv.request.manipulator_pose = data->poses[pose_ind];
			temp_orient = plan_query_srv.request.manipulator_pose.orientation;
			plan_query_srv.request.bag_poses = *data;
			//plan_query_srv.request.setup_environment = true;
			plan_query_srv.request.bag_labels = obj_group;
			plan_query_srv.request.bag_domain_labels = bag_domain_labels;
			plan_query_srv.request.pickup_object = "none";
			plan_query_srv.request.grasp_type = "side";
			plan_query_srv.request.drop_object = "none";
			plan_query_srv.request.planning_domain = "domain";
			plan_query_srv.request.safe_config = false;
		} else if (act_seq[i] == "transport") {
			std::string temp_loc_label = state_seq[i+1]->getVar("eeLoc");
			std::cout<<"temp loc label: "<<temp_loc_label<<std::endl;
			plan_query_srv.request.manipulator_pose = pred_gen.getLocation(temp_loc_label);
			std::cout<<"\n"<<std::endl;
			std::cout<<plan_query_srv.request.manipulator_pose.position.x<<std::endl;
			std::cout<<plan_query_srv.request.manipulator_pose.position.y<<std::endl;
			std::cout<<plan_query_srv.request.manipulator_pose.position.z<<std::endl;
			std::cout<<plan_query_srv.request.manipulator_pose.position.z<<std::endl;
			std::cout<<plan_query_srv.request.manipulator_pose.orientation.x<<std::endl;
			std::cout<<plan_query_srv.request.manipulator_pose.orientation.y<<std::endl;
			std::cout<<plan_query_srv.request.manipulator_pose.orientation.z<<std::endl;
			std::cout<<plan_query_srv.request.manipulator_pose.orientation.w<<std::endl;
			//plan_query_srv.request.manipulator_pose.position = pred_gen.getLocation(temp_loc_label);
			//plan_query_srv.request.manipulator_pose.orientation = temp_orient;
			//temp_orient = plan_query_srv.request.manipulator_pose.orientation;
			plan_query_srv.request.bag_poses = *data;
			//plan_query_srv.request.setup_environment = true;
			plan_query_srv.request.bag_labels = bag_labels;
			plan_query_srv.request.bag_domain_labels = bag_domain_labels;
			plan_query_srv.request.pickup_object = "none";
			plan_query_srv.request.grasp_type = "up";
			plan_query_srv.request.drop_object = "none";
			plan_query_srv.request.planning_domain = "domain";
			plan_query_srv.request.safe_config = false;

		} else if (act_seq[i] == "grasp") {
			plan_query_srv.request.setup_environment = false;

			std::string temp_obj_label;
			state_seq[i+1]->argFindGroup("ee", "object locations", temp_obj_label);
			std::cout<<"pick obj: "<<temp_obj_label<<std::endl;
			plan_query_srv.request.pickup_object = temp_obj_label;
			plan_query_srv.request.drop_object = "none";
			plan_query_srv.request.planning_domain = "domain";
			plan_query_srv.request.safe_config = false;
		} else if (act_seq[i] == "release") {
			plan_query_srv.request.setup_environment = false;
			plan_query_srv.request.pickup_object = "none";

			std::string temp_obj_label;
			state_seq[i]->argFindGroup("ee", "object locations", temp_obj_label);
			std::cout<<"rel obj: "<<temp_obj_label<<std::endl;
			plan_query_srv.request.drop_object = temp_obj_label; 
			plan_query_srv.request.planning_domain = "domain";
			plan_query_srv.request.safe_config = false;
		} else {
			ROS_WARN("Unrecognized action %s", act_seq[i].c_str());
		}
		if (plan_query_client.call(plan_query_srv)) {
			ROS_INFO("Completed service");
		} else {
			ROS_WARN("Did not find plan query service");
		}
	}
	return 0;
}
