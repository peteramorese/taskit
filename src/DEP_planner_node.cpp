// System
#include<boost/filesystem.hpp>

// ROS
#include "ros/ros.h"
#include "ros/package.h"
#include "manipulation_interface/ActionSingle.h"
#include "manipulation_interface/PreferenceQuery.h"
#include "manipulation_interface/RunQuery.h"

// Task Planner
#include "graph.h"
#include "condition.h"
#include "transitionSystem.h"
#include "stateSpace.h"
#include "state.h"
#include "symbSearch.h"



class PlanSrv {
	private: 
		SymbSearch search_obj;
	 	TS_EVAL<State>* ts_ptr;
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
		PlanSrv(TS_EVAL<State>* ts_ptr_, const std::vector<std::string>& obj_group_, ros::NodeHandle* current_NH_) : ts_ptr(ts_ptr_), obj_group(obj_group_), current_NH(current_NH_) {}
		bool plan(manipulation_interface::PreferenceQuery::Request& req, manipulation_interface::PreferenceQuery::Response& res) {
			clearDFAPtrs();

			int N_DFAs = req.formulas_ordered.size();

			//std::cout<<"ROS PACKAGE PATH: "<<ros::package::getPath("manipulation_interface")<<std::endl;

			// Create DFA files:
			std::string home_dir = getenv("HOME");
			std::string python_executable = home_dir + "/anaconda3/envs/tpenv/bin/python";
			std::string formula2dfa_path = ros::package::getPath("manipulation_interface") + "/task_planner/spot_automaton_file_dump";
			std::string command = python_executable + 
				" " + formula2dfa_path + 
				"/formula2dfa.py --dfa_path " + formula2dfa_path + "/dfas" + " " +
				"--formulas " + formulaListToStr(req.formulas_ordered);
			std::cout<<"COMMAND: "<<command<<std::endl;
			int _ = system(command.c_str());

			std::string dfa_filename_path_prefix = ros::package::getPath("manipulation_interface") + "/task_planner/spot_automaton_file_dump/dfas/";

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



			/////////////////////////////////////////////////
			/*        Construct the Symbolic Search        */
			/////////////////////////////////////////////////

			// First construct the graph evaluation objects:
			//TS_EVAL<State> ts_eval(&ts, 0); 
			//std::vector<DFA_EVAL> dfa_eval_vec;
			for (int i=0; i<N_DFAs; ++i) {
				DFA_EVAL* temp_dfa_eval_ptr = new DFA_EVAL(&dfa_arr[i]);
				dfa_eval_ptrs.push_back(temp_dfa_eval_ptr);
			}

			search_obj.setAutomataPrefs(&dfa_eval_ptrs);
			search_obj.setTransitionSystem(ts_ptr);


			search_obj.setFlexibilityParam(req.flexibility);

			std::pair<bool, float> result = search_obj.search(true); // Use heuristic
			res.success = result.first;
			res.pathlength = result.second;
			return true;
		}

		bool run(manipulation_interface::RunQuery::Request& req, manipulation_interface::RunQuery::Response& res) {
			auto state_sequence = search_obj.getStateSequence();
			auto action_sequence = search_obj.getActionSequence();

			ros::ServiceClient ex_client = current_NH->serviceClient<manipulation_interface::ActionSingle>("/action_primitive");
			manipulation_interface::ActionSingle action_single;

			action_single.request.obj_group = obj_group;
			std::vector<std::string> init_obj_locs;
			const State* init_state_ptr = ts_ptr->getState(ts_ptr->getInitStateInd());
			for (auto& obj : obj_group) {
				init_obj_locs.push_back(init_state_ptr->getVar(obj));
			}
			action_single.request.init_obj_locs = init_obj_locs;

			for (int i=0; i<action_sequence.size(); ++i) {
				std::cout<<"Sending action:" + action_sequence[i]<<std::endl;
				// Action:
				action_single.request.action = action_sequence[i];

				// Next state eef location:
				action_single.request.to_eeLoc = state_sequence[i+1]->getVar("eeLoc");

				// Next state grasped object:
				std::string temp_obj_label;
				if (state_sequence[i+1]->argFindGroup("ee", "object locations", temp_obj_label)) {
					std::cout<<"GRASP OBJ: "<<temp_obj_label<<std::endl;
					action_single.request.to_grasp_obj = temp_obj_label;
				} else {
					action_single.request.to_grasp_obj = "none";
				}

				// Current state object to release:
				if (state_sequence[i]->argFindGroup("ee", "object locations", temp_obj_label)) {
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

		void clearDFAPtrs() {
			for (int i=0; i<dfa_eval_ptrs.size(); ++i) {
				delete dfa_eval_ptrs[i];
			}
			dfa_eval_ptrs.clear();
		}

		~PlanSrv() {
			clearDFAPtrs();
		}

};

int main(int argc, char** argv) {
	ros::init(argc, argv, "planner_node");
	ros::NodeHandle planner_NH;

    // Object group:
    std::vector<std::string> obj_group;
    planner_NH.getParam("/discrete_environment/obj_group", obj_group);
    std::cout<<"Found "<<obj_group.size()<<" objects: ";
    for (auto& obj : obj_group) {
        std::cout<<obj<<" ";
    }
	std::cout<<"\n";

    // Location names:
    std::vector<std::string> loc_labels;
    planner_NH.getParam("/discrete_environment/location_names", loc_labels);

    // Initial object locations:
    std::vector<std::string> init_obj_locations;
    planner_NH.getParam("/discrete_environment/init_obj_locations", init_obj_locations);

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

	// Label state space:
	SS_MANIPULATOR.setStateDimensionLabel(0, "eeLoc");
    for (int i=0; i<obj_group.size(); ++i) {
        SS_MANIPULATOR.setStateDimensionLabel(i + 1, obj_group[i]);
    }
	SS_MANIPULATOR.setStateDimensionLabel(obj_group.size() + 1, "holding");

	// Create object location group:
	SS_MANIPULATOR.setLabelGroup("object locations", obj_group);

	// Set the initial state:
	State init_state(&SS_MANIPULATOR);	
	init_state.setState(set_state);

	/* SET CONDITIONS */
	// Pickup domain conditions:
	std::vector<Condition> conds_m;
	std::vector<Condition*> cond_ptrs_m;
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
	conds_m[0].setActionCost(0);

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
	conds_m[2].setActionCost(0);
	//conds_m[2].print();


	// Transit
	conds_m[3].addCondition(Condition::PRE, Condition::LABEL, "holding", Condition::EQUALS, Condition::VAR, "false");
	conds_m[3].addCondition(Condition::PRE, Condition::LABEL, "eeLoc", Condition::ARG_FIND, Condition::NONE, Condition::FILLER, Condition::TRUE, "arg");
	conds_m[3].setCondJunctType(Condition::PRE, Condition::CONJUNCTION);

	conds_m[3].addCondition(Condition::POST, Condition::ARG_V, Condition::FILLER, Condition::ARG_EQUALS, Condition::LABEL, "eeLoc", Condition::NEGATE,"arg");
	conds_m[3].setCondJunctType(Condition::POST, Condition::CONJUNCTION);
	conds_m[3].setActionLabel("transit_up");
	conds_m[3].setActionCost(0);
	//conds_m[3].print();


	for (int i=0; i<conds_m.size(); ++i){
		cond_ptrs_m[i] = &conds_m[i];
	}


	/* Propositions */
	std::cout<<"Setting Atomic Propositions... "<<std::endl;
	std::vector<SimpleCondition> AP_m;
	std::vector<SimpleCondition*> AP_m_ptrs;
	for (auto& loc_label : loc_labels) {
        for (auto& obj : obj_group) {
            SimpleCondition ap;
            ap.addCondition(Condition::SIMPLE, Condition::LABEL, obj, Condition::EQUALS, Condition::VAR, loc_label);
            ap.addCondition(Condition::SIMPLE, Condition::LABEL, "holding", Condition::EQUALS, Condition::VAR, "false");
            ap.setCondJunctType(Condition::SIMPLE, Condition::CONJUNCTION);
            ap.setLabel(obj + "_" + loc_label);
            AP_m.push_back(ap);
        }
	}
    AP_m_ptrs.resize(AP_m.size());
	for (int i=0; i<AP_m.size(); ++i) {
		AP_m_ptrs[i] = &AP_m[i];
	}


	// Create the transition system:
	TS_EVAL<State> ts_eval(true, false, 0); // by default, the init node for the ts is 0

	ts_eval.setInitState(&init_state);
	ts_eval.setConditions(cond_ptrs_m);
	ts_eval.setPropositions(AP_m_ptrs);
	ts_eval.generate();
	//std::cout<<"\n\n Printing the Transition System: \n\n"<<std::endl;
	//ts_eval.print();


	PlanSrv plan_obj(&ts_eval, obj_group, &planner_NH);
	ros::ServiceServer plan_srv = planner_NH.advertiseService("/preference_planning_query", &PlanSrv::plan, &plan_obj);
	ros::ServiceServer run_srv = planner_NH.advertiseService("/action_run_query", &PlanSrv::run, &plan_obj);
	ROS_INFO("Plan and Run services are online!");
	ros::spin();

	return 0;
}


