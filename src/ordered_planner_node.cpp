// System
#include<boost/filesystem.hpp>

// ROS
#include "ros/ros.h"
#include "ros/package.h"
#include "manipulation_interface/ActionSingle.h"
#include "manipulation_interface/PreferenceQuery.h"
#include "manipulation_interface/RunQuery.h"
#include "manipulation_interface/Strategy.h"

// Task Planner
#include "orderedPlanner.h"



class PlanSrvOP {
	private: 
	  	bool open_loop;
	 	bool success;
		int cl_i; // Keeps track of the current action index for closed loop planning
		std::shared_ptr<const OrderedPlanner::Plan> plan;
		const OrderedPlanner::Result* result_ptr;
	 	TransitionSystem<State>& ts;
		std::vector<DFA_EVAL*> dfa_eval_ptrs;
		const std::vector<std::string>& obj_group;
		ros::NodeHandle* current_NH;
		std::string formulaListToStr(const std::vector<std::string>& list) {
			std::string ret_str = "";
			for (auto& formula : list) {
				ret_str += " \"" + formula + "\"";
			}
			return ret_str;
		}
		

	public:
		PlanSrvOP(TransitionSystem<State>& ts_, const std::vector<std::string>& obj_group_, ros::NodeHandle* current_NH_, bool open_loop_ = true) : 
			ts(ts_), 
			obj_group(obj_group_), 
			current_NH(current_NH_), 
			open_loop(open_loop_), 
			result_ptr(nullptr), 
			success(false), 
			plan(nullptr), 
			cl_i(0) {std::cout<<"CONSTRUCTING..."<<std::endl;}

		bool planCB(manipulation_interface::PreferenceQuery::Request& req, manipulation_interface::PreferenceQuery::Response& res) {
			clearDFAPtrs();
			int N_DFAs = req.formulas_ordered.size();

			// Create DFA files:
			std::vector<std::string> python_envs = {"/anaconda3/envs", "/miniconda3/envs"};
			for (const auto& py_env : python_envs) {
				std::string home_dir = getenv("HOME");
				std::string python_executable = home_dir + py_env +"/tpenv/bin/python";
				std::string formula2dfa_path = ros::package::getPath("manipulation_interface") + "/task_planner/spot_automaton_file_dump";
				std::string command = python_executable + 
					" " + formula2dfa_path + 
					"/formula2dfa.py --dfa-path " + formula2dfa_path + "/dfas" + " " +
					"--formulas " + formulaListToStr(req.formulas_ordered);
				std::cout<<"COMMAND: "<<command<<std::endl;
				int p_ret_val = system(command.c_str());
				std::cout<<"\n COMMAND RET VALUE: "<<p_ret_val<<std::endl;
				if (p_ret_val == 0) {
					// Command succeeded
					break;
				}
			}


			std::string dfa_filename_path_prefix = ros::package::getPath("manipulation_interface") + "/task_planner/spot_automaton_file_dump/dfas/";

			// Read in the DFAs:
			std::vector<DFA> dfa_arr(N_DFAs);
			std::vector<std::string> filenames(N_DFAs);
			for (int i=0; i<N_DFAs; ++i) {
				filenames[i] = dfa_filename_path_prefix +"dfa_" + std::to_string(i) +".txt";
			}
			for (int i=0; i<N_DFAs; ++i) {
				dfa_arr[i].readFileSingle(filenames[i]);
			}
			std::cout<<"\n\nPrinting all DFA's (read into an array)...\n\n"<<std::endl;
			for (int i=0; i<N_DFAs; ++i) {
				dfa_arr[i].print();
				std::cout<<"\n"<<std::endl;
			}

			// Construct the planner:
			for (int i=0; i<N_DFAs; ++i) {
				DFA_EVAL* temp_dfa_eval_ptr = new DFA_EVAL(&dfa_arr[i]);
				dfa_eval_ptrs.push_back(temp_dfa_eval_ptr);
			}

			// Flexibility Function:
			auto setToMuDelay = [](const std::vector<float>& set) {
				std::vector<bool> skip(set.size(), false);
				bool sorted = false;
				int mu = 0;
				// Determine max prio val:
				int max_prio = set.size() - 1;
				std::unordered_map<float, bool> seen;
				float set_sum = 0.0f; // Check if the set is non-zero (all quantities have to be positive)
				for (int i=0; i<set.size(); ++i) {
					if (seen[set[i]]) {
						max_prio--;
						skip[i] = true;
					}
					seen[set[i]] = true;
					set_sum += set[i];
				}
				if (set_sum == 0.0f) return 0.0f;
				//std::cout<<"max prio: "<<max_prio<<std::endl;
				for (int prio = max_prio; prio >= 0; --prio) {
					float max_val = 0.0f;
					int max_ind = -1;
					for (int i=0; i<set.size(); ++i) {
						if (!skip[i] && (set[i] > max_val || max_val == 0.0f)) {
							max_val = set[i];
							max_ind = i;
						} 
					}
					//std::cout<<"Max ind: "<<max_ind<<" curr prio: "<<prio<<std::endl;
					if (max_ind == -1) {
						std::cout<<"ERROR -1!!!"<<std::endl;
						for (auto item : set) std::cout<<"set item: "<<item<<std::endl;
					}
					skip.at(max_ind) = true;
					int delay = prio - max_ind;
					//if (delay > 0) std::cout<<"Adding: "<<delay<<" delay..."<<std::endl;
					if (delay > 0) mu += delay;
				}
				return static_cast<float>(mu); 
			};

			// Run the planner:
			OrderedPlanner planner(ts);
			success = planner.search(dfa_eval_ptrs, setToMuDelay, true);
			res.success = success;
			result_ptr = planner.getResult();
			plan = planner.getResult()->getPlan(); // Single query returns first plan
			std::cout<<"AFTER PLAN PRITING ACT SQ"<<std::endl;
			for (const auto & a : plan->action_sequence) std::cout<<" - "<< a<<std::endl;
			cl_i = 0; // Reset current action index
			res.pathlength = result_ptr->getParetoFront()->front().path_length;
			return true;
		}

		bool runOpenLoop(manipulation_interface::RunQuery::Request& req, manipulation_interface::RunQuery::Response& res) {
			std::cout<<"IN RUN OPEN LOOP"<<std::endl;

			// Open loop uses 'run()' as a service client for Action Primitive Node:

			if (!result_ptr) return false;
			if (!success) {
				ROS_WARN("Planner did not succeed. Skipping run query...");
				return false;
			}

			ros::ServiceClient ex_client = current_NH->serviceClient<manipulation_interface::ActionSingle>("/action_primitive");
			manipulation_interface::ActionSingle action_single;

			action_single.request.obj_group = obj_group;
			std::vector<std::string> init_obj_locs;
			const State* init_state_ptr = ts.getState(ts.getInitStateInd());
			for (auto& obj : obj_group) {
				init_obj_locs.push_back(init_state_ptr->getVar(obj));
			}
			action_single.request.init_obj_locs = init_obj_locs;

			for (int i=0; i<plan->action_sequence.size(); ++i) {
				std::cout<<"b4 in loop"<<std::endl;
				std::cout<<"Sending action:" + plan->action_sequence[i]<<std::endl;
				std::cout<<"af in loop"<<std::endl;
				// Action:
				action_single.request.action = plan->action_sequence[i];

				// Next state eef location:
				action_single.request.to_eeLoc = plan->state_sequence[i+1]->getVar("eeLoc");

				// Next state grasped object:
				std::string temp_obj_label;
				if (plan->state_sequence[i+1]->argFindGroup("ee", "object locations", temp_obj_label)) {
					std::cout<<"GRASP OBJ: "<<temp_obj_label<<std::endl;
					action_single.request.to_grasp_obj = temp_obj_label;
				} else {
					action_single.request.to_grasp_obj = "none";
				}

				// Current state object to release:
				if (plan->state_sequence[i]->argFindGroup("ee", "object locations", temp_obj_label)) {
					std::cout<<"RELEASE OBJ: "<<temp_obj_label<<std::endl;
					action_single.request.release_obj = temp_obj_label;
				} else {
					action_single.request.release_obj = "none";
				}

				// Call the service:
				if (ex_client.call(action_single)) {
					ROS_INFO("Execution client call succeeded!");
					res.success = true;
				} else {
					ROS_ERROR("Execution client call failed!");
					res.success = false;
					break;
				}
			}
			res.end_time = ros::Time::now();
			return true; // Success in the response allows for failed actions, thus failed execution
		}
		
		bool runClosedLoop(manipulation_interface::Strategy::Request& req, manipulation_interface::Strategy::Response& res) {

			// Closed loop uses 'run()' as a service server for Com Node:

			if (!result_ptr) return false;
			if (!success) {
				ROS_WARN("Planner did not succeed. Skipping run query...");
				return false;
			}

			std::cout<<"Sending action:" + plan->action_sequence[cl_i]<<std::endl;
			// Action:
			res.action = plan->action_sequence[cl_i];

			// Next state eef location:
			res.to_eeloc = plan->state_sequence[cl_i+1]->getVar("eeLoc");

			// Next state grasped object:
			std::string temp_obj_label;
			if (plan->state_sequence[cl_i+1]->argFindGroup("ee", "object locations", temp_obj_label)) {
				std::cout<<"GRASP OBJ: "<<temp_obj_label<<std::endl;
				res.to_grasp_obj = temp_obj_label;
			} else if (plan->state_sequence[cl_i+1]->argFindGroup(res.to_eeloc, "object locations", temp_obj_label)) {
				res.to_grasp_obj = temp_obj_label;
			} else {
				res.to_grasp_obj = "none";
			}

			// Current state object to release:
			if (plan->state_sequence[cl_i]->argFindGroup("ee", "object locations", temp_obj_label)) {
				std::cout<<"RELEASE OBJ: "<<temp_obj_label<<std::endl;
				res.release_obj = temp_obj_label;
			} else {
				res.release_obj = "none";
			}

			//}
			cl_i++;
			return true; // Success in the response allows for failed actions, thus failed execution
		}

		void clearDFAPtrs() {
			for (int i=0; i<dfa_eval_ptrs.size(); ++i) {
				delete dfa_eval_ptrs[i];
			}
			dfa_eval_ptrs.clear();
		}

		~PlanSrvOP() {
			clearDFAPtrs();
		}

};

int main(int argc, char** argv) {
	ros::init(argc, argv, "ordered_planner_node");
	ros::NodeHandle planner_NH;

    // Object group:
    std::vector<std::string> obj_group;
    planner_NH.getParam("/discrete_environment/obj_group", obj_group);
    std::cout<<"Found "<<obj_group.size()<<" objects: ";
    for (auto& obj : obj_group) {
        std::cout<<obj<<" ";
    }
	std::cout<<"\n";

    // Run in open loop or closed loop mode:
	bool open_loop;
    planner_NH.param("/ordered_planner_node/open_loop", open_loop, true);
	if (open_loop) {
		ROS_INFO("Execution in open loop");
	} else {
		ROS_INFO("Execution in closed loop");
	}

    // Location names:
    std::vector<std::string> loc_labels;
    planner_NH.getParam("/discrete_environment/location_names", loc_labels);

    // Initial object locations:
    std::vector<std::string> init_obj_locations;
    planner_NH.getParam("/discrete_environment/init_obj_locations", init_obj_locations);

    // Location orientation types:
    std::vector<std::string> location_orientation_types;
    planner_NH.getParam("/discrete_environment/location_orientation_types", location_orientation_types);

	if (init_obj_locations.size() != obj_group.size()) {
		ROS_ERROR("Initial object locations does match number of objects");
		return 1;
	}

	if (loc_labels.size() > location_orientation_types.size()) {
		ROS_ERROR("Number of locations is greater than number of location orientation types");
		return 1;
	}

	std::vector<std::string> side_loc_labels;
	for (int i=0; i<loc_labels.size(); ++i) {
		if (location_orientation_types[i] == "side_x" ||	
			location_orientation_types[i] == "side_y" 
			) {
				side_loc_labels.push_back(loc_labels[i]);
			}
	}
	
	bool use_side_grasps;
	if (side_loc_labels.size() > 0) {
		ROS_INFO("Building model that includes SIDE GRASPS");
		use_side_grasps = true;
	} else {
		ROS_INFO("Building model that does not include SIDE GRASPS");
		use_side_grasps = false;
	}
	//////////////////////////////////////////////////////
	/* Create the Transition System for the Manipualtor */
	//////////////////////////////////////////////////////

	/* CREATE ENVIRONMENT FOR MANIPULATOR */
	StateSpace SS_MANIPULATOR;

    // Properties of the planning environment:
	std::vector<std::string> set_state = {"stow"};
	for (auto& obj_loc : init_obj_locations) {
		set_state.push_back(obj_loc);
	}
	set_state.push_back("false");
	if (use_side_grasps) {
		set_state.push_back("none");
	}


	std::vector<std::string> ee_labels = loc_labels;
	ee_labels.push_back("stow");
	std::vector<std::string> obj_labels = loc_labels;
	obj_labels.push_back("ee");
	std::vector<std::string> grip_labels = {"true","false"};

	// Create state space:
	SS_MANIPULATOR.setStateDimension(ee_labels, 0); // eef
    for (int i=0; i<obj_group.size(); ++i) {
		SS_MANIPULATOR.setStateDimension(obj_labels, i + 1); 
    }
	SS_MANIPULATOR.setStateDimension(grip_labels, obj_group.size() + 1); // eef engaged
	// Side grasp needs to keep track of the grasp:
	if (use_side_grasps) {
		std::vector<std::string> grip_type_labels = {"none","up","side"};
		SS_MANIPULATOR.setStateDimension(grip_type_labels, obj_group.size() + 2); // grip type
	}

	// Label state space:
	SS_MANIPULATOR.setStateDimensionLabel(0, "eeLoc");
    for (int i=0; i<obj_group.size(); ++i) {
        SS_MANIPULATOR.setStateDimensionLabel(i + 1, obj_group[i]);
    }
	SS_MANIPULATOR.setStateDimensionLabel(obj_group.size() + 1, "holding");
	if (use_side_grasps) {
		SS_MANIPULATOR.setStateDimensionLabel(obj_group.size() + 2, "grip_type");

		// Construct domain for side grasps:
		SS_MANIPULATOR.setDomain("side locations", side_loc_labels);
	}

	// Create object location group:
	SS_MANIPULATOR.setLabelGroup("object locations", obj_group);

	// Set the initial state:
	State init_state(&SS_MANIPULATOR);	
	init_state.setState(set_state);

	/* SET CONDITIONS */
	std::vector<Condition> conds_m;
	std::vector<Condition*> cond_ptrs_m;

	if (!use_side_grasps) {
		conds_m.resize(4);
		cond_ptrs_m.resize(4);

		// Grasp 
		conds_m[0].addCondition(Condition::PRE, Condition::LABEL, "holding", Condition::EQUALS, Condition::VAR, "false");
		conds_m[0].addCondition(Condition::PRE, Condition::GROUP, "object locations", Condition::ARG_FIND, Condition::LABEL, "eeLoc",Condition::TRUE, "arg");
		conds_m[0].setCondJunctType(Condition::PRE, Condition::CONJUNCTION);

		conds_m[0].addCondition(Condition::POST, Condition::ARG_L, Condition::FILLER, Condition::ARG_EQUALS, Condition::VAR, "ee",Condition::TRUE, "arg");
		conds_m[0].addCondition(Condition::POST, Condition::LABEL, "holding", Condition::EQUALS, Condition::VAR, "true");
		conds_m[0].setCondJunctType(Condition::POST, Condition::CONJUNCTION);
		conds_m[0].setActionLabel("grasp");
		conds_m[0].setActionCost(1);

		// Transport 
		conds_m[1].addCondition(Condition::PRE, Condition::LABEL, "holding", Condition::EQUALS, Condition::VAR, "true");
		conds_m[1].addCondition(Condition::PRE, Condition::GROUP, "object locations", Condition::ARG_FIND, Condition::LABEL, "eeLoc", Condition::NEGATE, "arg1");
		conds_m[1].addCondition(Condition::PRE, Condition::LABEL, "eeLoc", Condition::ARG_FIND, Condition::NONE, Condition::FILLER, Condition::TRUE, "arg2");
		conds_m[1].setCondJunctType(Condition::PRE, Condition::CONJUNCTION); // Used to store eeLoc pre-state variable
		conds_m[1].addCondition(Condition::POST, Condition::ARG_V, Condition::FILLER, Condition::ARG_EQUALS, Condition::LABEL, "eeLoc", Condition::NEGATE, "arg2"); // Stored eeLoc pre-state variable is not the same as post-state eeLoc (eeLoc has moved)
		conds_m[1].addCondition(Condition::POST, Condition::GROUP, "object locations", Condition::ARG_FIND, Condition::LABEL, "eeLoc", Condition::NEGATE,"na");
		conds_m[1].setCondJunctType(Condition::POST, Condition::CONJUNCTION);
		conds_m[1].setActionLabel("transport");
		conds_m[1].setActionCost(5);

		// Release 
		conds_m[2].addCondition(Condition::PRE, Condition::LABEL, "holding", Condition::EQUALS, Condition::VAR, "true");
		conds_m[2].addCondition(Condition::PRE, Condition::GROUP, "object locations", Condition::ARG_FIND, Condition::LABEL, "eeLoc", Condition::NEGATE, "arg1");
		conds_m[2].addCondition(Condition::PRE, Condition::GROUP, "object locations", Condition::ARG_FIND, Condition::VAR, "ee",Condition::TRUE, "arg2");
		conds_m[2].setCondJunctType(Condition::PRE, Condition::CONJUNCTION);

		conds_m[2].addCondition(Condition::POST, Condition::ARG_L, Condition::FILLER, Condition::ARG_EQUALS, Condition::LABEL, "eeLoc", Condition::TRUE, "arg2");
		conds_m[2].addCondition(Condition::POST, Condition::LABEL, "holding", Condition::EQUALS, Condition::VAR, "false");
		conds_m[2].setCondJunctType(Condition::POST, Condition::CONJUNCTION);
		conds_m[2].setActionLabel("release");
		conds_m[2].setActionCost(1);


		// Transit
		conds_m[3].addCondition(Condition::PRE, Condition::LABEL, "holding", Condition::EQUALS, Condition::VAR, "false");
		conds_m[3].addCondition(Condition::PRE, Condition::LABEL, "eeLoc", Condition::ARG_FIND, Condition::NONE, Condition::FILLER, Condition::TRUE, "arg");
		conds_m[3].setCondJunctType(Condition::PRE, Condition::CONJUNCTION);

		conds_m[3].addCondition(Condition::POST, Condition::ARG_V, Condition::FILLER, Condition::ARG_EQUALS, Condition::LABEL, "eeLoc", Condition::NEGATE,"arg");
		conds_m[3].setCondJunctType(Condition::POST, Condition::CONJUNCTION);
		conds_m[3].setActionLabel("transit_up");
		conds_m[3].setActionCost(3);

	} else {
        conds_m.resize(6);
        cond_ptrs_m.resize(6);

        // Grasp regular
        conds_m[0].addCondition(Condition::PRE, Condition::LABEL, "holding", Condition::EQUALS, Condition::VAR, "false");
        conds_m[0].addCondition(Condition::PRE, Condition::LABEL, "grip_type", Condition::EQUALS, Condition::VAR, "none", Condition::NEGATE, "griptype_not_none");
        conds_m[0].addCondition(Condition::PRE, Condition::GROUP, "object locations", Condition::ARG_FIND, Condition::LABEL, "eeLoc",Condition::TRUE, "arg");
        conds_m[0].setCondJunctType(Condition::PRE, Condition::CONJUNCTION);

        conds_m[0].addCondition(Condition::POST, Condition::ARG_L, Condition::FILLER, Condition::ARG_EQUALS, Condition::VAR, "ee",Condition::TRUE, "arg");
        conds_m[0].addCondition(Condition::POST, Condition::LABEL, "grip_type", Condition::EQUALS, Condition::VAR, "none", Condition::NEGATE, "gt_not_none");
        conds_m[0].addCondition(Condition::POST, Condition::LABEL, "holding", Condition::EQUALS, Condition::VAR, "true");
        conds_m[0].setCondJunctType(Condition::POST, Condition::CONJUNCTION);
        conds_m[0].setActionLabel("grasp");
		conds_m[0].setActionCost(1);
	
		// Transport Up
        conds_m[1].addCondition(Condition::PRE, Condition::LABEL, "holding", Condition::EQUALS, Condition::VAR, "true");
        conds_m[1].addCondition(Condition::PRE, Condition::LABEL, "grip_type", Condition::EQUALS, Condition::VAR, "up");
        conds_m[1].addCondition(Condition::PRE, Condition::GROUP, "object locations", Condition::ARG_FIND, Condition::LABEL, "eeLoc", Condition::NEGATE, "arg1");
        conds_m[1].addCondition(Condition::PRE, Condition::LABEL, "eeLoc", Condition::ARG_FIND, Condition::NONE, Condition::FILLER, Condition::TRUE, "arg2");
        conds_m[1].setCondJunctType(Condition::PRE, Condition::CONJUNCTION); // Used to store eeLoc pre-state variable
        conds_m[1].addCondition(Condition::POST, Condition::ARG_V, Condition::FILLER, Condition::ARG_EQUALS, Condition::LABEL, "eeLoc", Condition::NEGATE, "arg2"); // Stored eeLoc pre-state variable is not the same as post-state eeLoc (eeLoc has moved)
        conds_m[1].addCondition(Condition::POST, Condition::LABEL, "eeLoc", Condition::IN_DOM, Condition::DOM, "side locations", Condition::NEGATE, "na"); // Not in side locations
        conds_m[1].addCondition(Condition::POST, Condition::LABEL, "grip_type", Condition::EQUALS, Condition::VAR, "none");
        conds_m[1].addCondition(Condition::POST, Condition::GROUP, "object locations", Condition::ARG_FIND, Condition::LABEL, "eeLoc", Condition::NEGATE,"na");
        conds_m[1].setCondJunctType(Condition::POST, Condition::CONJUNCTION);
        conds_m[1].setActionLabel("transport");
		conds_m[1].setActionCost(5);

        // Release 
        conds_m[2].addCondition(Condition::PRE, Condition::LABEL, "holding", Condition::EQUALS, Condition::VAR, "true");
        conds_m[2].addCondition(Condition::PRE, Condition::GROUP, "object locations", Condition::ARG_FIND, Condition::LABEL, "eeLoc", Condition::NEGATE, "arg1");
        conds_m[2].addCondition(Condition::PRE, Condition::GROUP, "object locations", Condition::ARG_FIND, Condition::VAR, "ee",Condition::TRUE, "arg2");
        conds_m[2].setCondJunctType(Condition::PRE, Condition::CONJUNCTION);

        conds_m[2].addCondition(Condition::POST, Condition::ARG_L, Condition::FILLER, Condition::ARG_EQUALS, Condition::LABEL, "eeLoc", Condition::TRUE, "arg2");
        conds_m[2].addCondition(Condition::POST, Condition::LABEL, "holding", Condition::EQUALS, Condition::VAR, "false");
        conds_m[2].setCondJunctType(Condition::POST, Condition::CONJUNCTION);
        conds_m[2].setActionLabel("release");
		conds_m[2].setActionCost(1);

        // Transit Up
        conds_m[3].addCondition(Condition::PRE, Condition::LABEL, "holding", Condition::EQUALS, Condition::VAR, "false");
        conds_m[3].addCondition(Condition::PRE, Condition::LABEL, "grip_type", Condition::EQUALS, Condition::VAR, "none");
        conds_m[3].addCondition(Condition::PRE, Condition::LABEL, "eeLoc", Condition::ARG_FIND, Condition::NONE, Condition::FILLER, Condition::TRUE, "arg");
        conds_m[3].setCondJunctType(Condition::PRE, Condition::CONJUNCTION);

        conds_m[3].addCondition(Condition::POST, Condition::ARG_V, Condition::FILLER, Condition::ARG_EQUALS, Condition::LABEL, "eeLoc", Condition::NEGATE,"arg");
        conds_m[3].addCondition(Condition::POST, Condition::LABEL, "grip_type", Condition::EQUALS, Condition::VAR, "up");
        conds_m[3].addCondition(Condition::POST, Condition::LABEL, "eeLoc", Condition::IN_DOM, Condition::DOM, "side locations", Condition::NEGATE, "arg_2");
        conds_m[3].setCondJunctType(Condition::POST, Condition::CONJUNCTION);
        conds_m[3].setActionLabel("transit_up");
		conds_m[3].setActionCost(3);

		// Transit Side
        conds_m[4].addCondition(Condition::PRE, Condition::LABEL, "holding", Condition::EQUALS, Condition::VAR, "false");
        conds_m[4].addCondition(Condition::PRE, Condition::LABEL, "grip_type", Condition::EQUALS, Condition::VAR, "none");
        conds_m[4].addCondition(Condition::PRE, Condition::LABEL, "eeLoc", Condition::ARG_FIND, Condition::NONE, Condition::FILLER, Condition::TRUE, "arg");
        conds_m[4].setCondJunctType(Condition::PRE, Condition::CONJUNCTION);

        conds_m[4].addCondition(Condition::POST, Condition::ARG_V, Condition::FILLER, Condition::ARG_EQUALS, Condition::LABEL, "eeLoc", Condition::NEGATE,"arg");
        conds_m[4].addCondition(Condition::POST, Condition::LABEL, "grip_type", Condition::EQUALS, Condition::VAR, "side");
        //conds_m[4].addCondition(Condition::POST, Condition::LABEL, "eeLoc", Condition::IN_DOM, Condition::DOM, "side locations", Condition::TRUE, "na");
        conds_m[4].setCondJunctType(Condition::POST, Condition::CONJUNCTION);
        conds_m[4].setActionLabel("transit_side");
		conds_m[4].setActionCost(3);

		// Transport Side
        conds_m[5].addCondition(Condition::PRE, Condition::LABEL, "holding", Condition::EQUALS, Condition::VAR, "true");
        conds_m[5].addCondition(Condition::PRE, Condition::LABEL, "grip_type", Condition::EQUALS, Condition::VAR, "side");
        conds_m[5].addCondition(Condition::PRE, Condition::GROUP, "object locations", Condition::ARG_FIND, Condition::LABEL, "eeLoc", Condition::NEGATE, "arg1");
        conds_m[5].addCondition(Condition::PRE, Condition::LABEL, "eeLoc", Condition::ARG_FIND, Condition::NONE, Condition::FILLER, Condition::TRUE, "arg2");
        conds_m[5].setCondJunctType(Condition::PRE, Condition::CONJUNCTION); // Used to store eeLoc pre-state variable
        conds_m[5].addCondition(Condition::POST, Condition::ARG_V, Condition::FILLER, Condition::ARG_EQUALS, Condition::LABEL, "eeLoc", Condition::NEGATE, "arg2"); // Stored eeLoc pre-state variable is not the same as post-state eeLoc (eeLoc has moved)
        conds_m[5].addCondition(Condition::POST, Condition::LABEL, "eeLoc", Condition::IN_DOM, Condition::DOM, "side locations", Condition::TRUE, "na");
        conds_m[5].addCondition(Condition::POST, Condition::LABEL, "grip_type", Condition::EQUALS, Condition::VAR, "none");
        conds_m[5].addCondition(Condition::POST, Condition::GROUP, "object locations", Condition::ARG_FIND, Condition::LABEL, "eeLoc", Condition::NEGATE,"na");
        conds_m[5].setCondJunctType(Condition::POST, Condition::CONJUNCTION);
        conds_m[5].setActionLabel("transport");
		conds_m[5].setActionCost(5);
	}


	for (int i=0; i<conds_m.size(); ++i){
		cond_ptrs_m[i] = &conds_m[i];
	}

	/* Propositions */
	std::cout<<"Setting Atomic Propositions: "<<std::endl;
	std::vector<SimpleCondition> AP_m;
	std::vector<SimpleCondition*> AP_m_ptrs;
	for (auto& loc_label : loc_labels) {
        for (auto& obj : obj_group) {
            SimpleCondition ap;
            ap.addCondition(Condition::SIMPLE, Condition::LABEL, obj, Condition::EQUALS, Condition::VAR, loc_label);
            ap.addCondition(Condition::SIMPLE, Condition::LABEL, "holding", Condition::EQUALS, Condition::VAR, "false");
            ap.setCondJunctType(Condition::SIMPLE, Condition::CONJUNCTION);
            ap.setLabel(obj + "_" + loc_label);
			std::cout<<" - "<<obj + "_" + loc_label<<std::endl;
            AP_m.push_back(ap);
        }
	}
    AP_m_ptrs.resize(AP_m.size());
	for (int i=0; i<AP_m.size(); ++i) {
		AP_m_ptrs[i] = &AP_m[i];
	}

	// Create the transition system:
	TransitionSystem<State> ts(true, false); 

	ts.setInitState(&init_state);
	ts.setConditions(cond_ptrs_m);
	ts.setPropositions(AP_m_ptrs);
	ts.generate();
	std::cout<<"\n\n Printing the Transition System: \n\n"<<std::endl;
	ts.print();
	std::cout<<"\n";

	PlanSrvOP plan_obj(ts, obj_group, &planner_NH, open_loop);
	ros::ServiceServer plan_srv = planner_NH.advertiseService("/preference_planning_query", &PlanSrvOP::planCB, &plan_obj);
	if (open_loop) {
		ros::ServiceServer run_srv_open_loop = planner_NH.advertiseService("/action_run_query", &PlanSrvOP::runOpenLoop, &plan_obj);
		ROS_INFO("Plan and Run services are online (open loop)!");
		ros::spin();
	} else {
		// Make sure plan is preference_planning_query is called before com node is kicked off
		ros::ServiceServer run_srv_closed_loop = planner_NH.advertiseService("/com_node/strategy", &PlanSrvOP::runClosedLoop, &plan_obj);
		ROS_INFO("Plan and Run services are online (closed loop)!");
		ros::spin();
	}

	return 0;
}


