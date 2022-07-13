// ROS
#include "ros/ros.h"
#include "manipulation_interface/ActionSingle.h"

// Task Planner
#include "graph.h"
#include "condition.h"
#include "transitionSystem.h"
#include "stateSpace.h"
#include "state.h"
#include "symbSearch.h"

int main() {


	//////////////////////////////////////////////////////
	/* Create the Transition System for the Manipualtor */
	//////////////////////////////////////////////////////

	/* CREATE ENVIRONMENT FOR MANIPULATOR */
	StateSpace SS_MANIPULATOR;

    // Edit properties about the planning environment:
	std::vector<std::string> loc_labels = {"L0", "L1", "L2", "L3", "L4"};	
	std::vector<std::string> obj_group = {"obj_1", "obj_2", "obj_3"};	
	std::vector<std::string> set_state = {"stow", "L0", "L1", "L2", "false"};


	std::vector<std::string> ee_labels = loc_labels;
	ee_labels.push_back("stow");
	std::vector<std::string> obj_labels = loc_labels;
	obj_labels.push_back("ee");
	std::vector<std::string> grip_labels = {"true","false"};

	// Create state space:
	SS_MANIPULATOR.setStateDimension(ee_labels, 0); // eef
	SS_MANIPULATOR.setStateDimension(obj_labels, 1); // rock
	SS_MANIPULATOR.setStateDimension(obj_labels, 2); // alien
	SS_MANIPULATOR.setStateDimension(grip_labels, 3); // eef engaged

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
	conds_m[3].setActionLabel("transit");
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
	TS_EVAL<State> ts_eval(true, true, 0); // by default, the init node for the ts is 0
	ts_eval.setInitState(&init_state);
	ts_eval.setConditions(cond_ptrs_m);
	ts_eval.setPropositions(AP_m_ptrs);
	ts_eval.generate();
	std::cout<<"\n\n Printing the Transition System: \n\n"<<std::endl;
	ts_eval.print();


	/////////////////////////////////////////////////
	/*       Read in the DFA's from the files      */
	/////////////////////////////////////////////////

    int N_DFAs = 2;
    // TODO REMOVE GLOBAL PATH
	std::string dfa_filename_path_prefix = "/home/peter/ros_ws/src/manipulation_interface/task_planner/spot_automaton_file_dump/dfas";

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
	std::vector<DFA_EVAL*> dfa_eval_ptrs;
	for (int i=0; i<N_DFAs; ++i) {
		DFA_EVAL* temp_dfa_eval_ptr = new DFA_EVAL(&dfa_arr[i]);
		dfa_eval_ptrs.push_back(temp_dfa_eval_ptr);
	}

	SymbSearch search_obj;
	search_obj.setAutomataPrefs(&dfa_eval_ptrs);
	search_obj.setTransitionSystem(&ts_eval);

    float mu;
    std::cout<<"\n------------------------------\n";
    std::cout<<"Enter flexibility parameter: ";
    std::cout<<"\n";
    std::cin >> mu;

    bool use_h_flag;
    char use_h;
    std::cout<<"\n------------------------------\n";
    std::cout<<"Use heuristic? [y/n]: ";
    std::cout<<"\n";
    std::cin >> use_h;
    use_h_flag = (use_h == 'y') ? true : false;

	search_obj.setFlexibilityParam(mu);
	//benchmark.pushStartPoint("total_search");
    //if (use_h_flag) {
    //    std::cout<<"Using heuristic."<<std::endl;
    //}
	std::pair<bool, float> result = search_obj.search(use_h_flag);

	////std::cout<<"search time: "<<benchmark.measureMicro("before_search")<<std::endl;
	//if (result.first) {
	//	benchmark.measureMilli("total_search");
	//	benchmark.pushAttributesToFile();
	//	benchmark.finishSessionInFile();
	//	//std::cout<<"Found plan? "<<success<<std::endl;
	//	if (result.second > 0.0) {
	//		std::vector<std::string> xtra_info;
	//		for (int i=0; i<dfa_arr.size(); ++i) {
	//			const std::vector<std::string>* ap_ptr = dfa_arr[i].getAP();
	//			for (int ii=0; ii<ap_ptr->size(); ++ii) {
	//				xtra_info.push_back(ap_ptr->operator[](ii));
	//				xtra_info.back() = xtra_info.back() + "_prio" + std::to_string(i);
	//			}
	//		}
	//		if (write_file_flag) {
	//			search_obj.writePlanToFile(plan_filename_path, xtra_info);
	//		}
	//	}
	//}
	for (int i=0; i<dfa_eval_ptrs.size(); ++i) {
		delete dfa_eval_ptrs[i];
	}

	return 0;



}


