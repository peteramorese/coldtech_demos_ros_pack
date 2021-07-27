#include <cstdio>
#include <math.h>
#include <unordered_map>
//#include "std_msgs/String.h"
//#include "river_ros/PlanExecute_srv.h"
//#include "river_ros/PlanningQuery_srv.h"
//#include "river_ros/PlanningQueryStatus_msg.h"
//#include "river_ros/BagConfigPoseArray_msg.h"
#include "geometry_msgs/Pose.h"
#include "ros/ros.h"
#include "geometry_msgs/Point.h"

#include "edge.h"
#include "astar.h"
#include "state.h"
#include "stateSpace.h"
#include "condition.h"
#include "transitionSystem.h"

#include <pluginlib/class_loader.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/PlanningScene.h>
#include <boost/scoped_ptr.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>




int main(int argc, char **argv){			
	ros::init(argc, argv, "planning_demo_node");
	ros::NodeHandle planning_demo_NH("~");

	ros::AsyncSpinner spinner(2);
	spinner.start();
	/*
	// When client requests to plan and execute, begin once service is recieved
	bool begin;
	StartSrv start_srv_container(begin);
	ros::ServiceServer start_TP_service = TP_NH.advertiseService("status/plan_execute", &StartSrv::startTP_serviceCB, &start_srv_container);

	// Publish the status of the task planner
	ros::Publisher status_TP_pub = TP_NH.advertise<std_msgs::String>("task_planner/status", 10);
	std_msgs::String status_TP_msg;

	// Subscribe to the Bag Config topic to setup environmnet and call manipulator node
	bool run;
	EnvironmentSub env_sub_container(begin, run);
	ros::Subscriber environment_TP_sub = TP_NH.subscribe("bag_config/bag_configs", 1, &EnvironmentSub::envSubCB, &env_sub_container);
	*/

	// Service client to send a planning query to manipulator node
	/*
	ros::ServiceClient plan_query_client = TP_NH.serviceClient<river_ros::PlanningQuery_srv>("manipulator_node/task_planner/planning_query");
	river_ros::PlanningQuery_srv plan_query_srv_msg;
	*/


	// Hard code drop off locations here
	// std::vector<std::vector<double>> drop_off_locs = {{-.4, .4, 0}, {-.4, -.4, 0}, {-.4, .4, .5}, {-.4, -.4, .5}};


	


	std::cout<<"Starting... "<<std::endl;
	
	Edge top_automaton(true);

	// Hard-code DFA_m automaton:
	//top_automaton.connect(2, 0, 1.0, "p_a & p_r");
	/*
	top_automaton.connect(2, 2, 1.0, "!phi_1 & !phi_2");
	top_automaton.connect(2, 1, 1.0, "phi_1 & !phi_2");
	top_automaton.connect(2, 3, 1.0, "!phi_1 & phi_2");
	*/
	//top_automaton.connect(2, 2, 1.0, "!phi_1 & !phi_2");
	top_automaton.connect(2, 1, 1.0, "phi_1");
	top_automaton.connect(2, 3, 1.0, "phi_2");
	top_automaton.connect(1, 1, 1.0, "!phi_2");
	top_automaton.connect(1, 0, 1.0, "phi_2");
	top_automaton.connect(3, 3, 1.0, "!phi_1");
	top_automaton.connect(3, 0, 1.0, "phi_1");
	top_automaton.print();

	Astar schedule;
	schedule.setGraph(&top_automaton);
	schedule.setVInit(2);
	std::vector<int> goal_set;
	goal_set.push_back(0);
	schedule.setVGoalSet(goal_set);
	//std::vector<int> reverse_plan;
	std::vector<int> top_plan;
	float path_length_top;
	bool plan_found = schedule.searchDijkstra(top_plan, path_length_top);
	/*
	top_plan.resize(reverse_plan.size());
	for (int i=0; i<reverse_plan.size(); ++i) {
		top_plan[i] = reverse_plan[reverse_plan.size()-1-i];
	}
	*/
	std::cout<<"\nSchedule Plan:";
	for (int i=0; i<top_plan.size(); ++i) {
		std::cout<<" -> "<<top_plan[i];	
	}
	std::cout<<"\n";

	// Extract the observations from the plan:
	std::vector<std::string> label_plan;
	auto heads_top = top_automaton.getHeads();
	for (int i=0; i<top_plan.size()-1; i++) {
		auto currptr_top = heads_top[top_plan[i]]->adjptr;
		while (currptr_top != nullptr) {
			if (currptr_top->nodeind == top_plan[i+1]) {
				label_plan.push_back(currptr_top->label);
			}
			currptr_top = currptr_top->adjptr;
		}
	}



	/* CREATE ENVIRONMENT FOR MANIPULATOR */
	StateSpace SS_MANIPULATOR;

	std::vector<std::string> loc_labels = {"L0", "L1", "L2", "L3", "L4"};	
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
	SS_MANIPULATOR.setStateDimensionLabel(1, "rock");
	SS_MANIPULATOR.setStateDimensionLabel(2, "alien");
	SS_MANIPULATOR.setStateDimensionLabel(3, "holding");

	// Create object location group:
	std::vector<std::string> obj_group = {"rock", "alien"};
	SS_MANIPULATOR.setLabelGroup("object locations", obj_group);

	// Set the initial state:
	std::vector<std::string> set_state = {"stow", "L0", "L1", "false"};
	//std::vector<std::string> test_set_state = {"L1", "L1", "L2", "false"};
	State init_state(&SS_MANIPULATOR);	
	init_state.setState(set_state);

	//State test_state(&SS_MANIPULATOR);	
	//test_state.setState(test_set_state);

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
	
	// Transport 
	conds_m[1].addCondition(Condition::PRE, Condition::LABEL, "holding", Condition::EQUALS, Condition::VAR, "true");
	conds_m[1].addCondition(Condition::PRE, Condition::GROUP, "object locations", Condition::ARG_FIND, Condition::LABEL, "eeLoc", Condition::NEGATE, "arg1");
	conds_m[1].addCondition(Condition::PRE, Condition::LABEL, "eeLoc", Condition::ARG_FIND, Condition::NONE, Condition::FILLER, Condition::TRUE, "arg2");
	conds_m[1].setCondJunctType(Condition::PRE, Condition::CONJUNCTION); // Used to store eeLoc pre-state variable
	conds_m[1].addCondition(Condition::POST, Condition::ARG_V, Condition::FILLER, Condition::ARG_EQUALS, Condition::LABEL, "eeLoc", Condition::NEGATE, "arg2"); // Stored eeLoc pre-state variable is not the same as post-state eeLoc (eeLoc has moved)
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


	// Transit
	conds_m[3].addCondition(Condition::PRE, Condition::LABEL, "holding", Condition::EQUALS, Condition::VAR, "false");
	conds_m[3].addCondition(Condition::PRE, Condition::LABEL, "eeLoc", Condition::ARG_FIND, Condition::NONE, Condition::FILLER, Condition::TRUE, "arg");
	conds_m[3].setCondJunctType(Condition::PRE, Condition::CONJUNCTION);

	conds_m[3].addCondition(Condition::POST, Condition::ARG_V, Condition::FILLER, Condition::ARG_EQUALS, Condition::LABEL, "eeLoc", Condition::NEGATE,"arg");
	conds_m[3].setCondJunctType(Condition::POST, Condition::CONJUNCTION);
	conds_m[3].setActionLabel("transit");
	//conds_m[3].print();

	
	for (int i=0; i<conds_m.size(); ++i){
		cond_ptrs_m[i] = &conds_m[i];
	}


	/* Propositions */
	std::cout<<"Setting Atomic Propositions... "<<std::endl;
	std::vector<SimpleCondition> AP_m(5*2);
	std::vector<SimpleCondition*> AP_m_ptrs(5*2);
	for (int i=0; i<5; ++i) {
		AP_m[2*i].addCondition(Condition::SIMPLE, Condition::LABEL, "rock", Condition::EQUALS, Condition::VAR, loc_labels[i]);
		AP_m[2*i].addCondition(Condition::SIMPLE, Condition::LABEL, "holding", Condition::EQUALS, Condition::VAR, "false");
		AP_m[2*i].setCondJunctType(Condition::SIMPLE, Condition::CONJUNCTION);
		AP_m[2*i].setLabel("p_r" + loc_labels[i]);

		AP_m[2*i + 1].addCondition(Condition::SIMPLE, Condition::LABEL, "alien", Condition::EQUALS, Condition::VAR, loc_labels[i]);
		AP_m[2*i + 1].addCondition(Condition::SIMPLE, Condition::LABEL, "holding", Condition::EQUALS, Condition::VAR, "false");
		AP_m[2*i + 1].setCondJunctType(Condition::SIMPLE, Condition::CONJUNCTION);
		AP_m[2*i + 1].setLabel("p_a" + loc_labels[i]);
	}
	for (int i=0; i<AP_m.size(); ++i) {
		AP_m_ptrs[i] = &AP_m[i];
	}
	
	// Add propositions for the gripper: 
	/*
	AP_m[5*2].addCondition(Condition::SIMPLE, Condition::LABEL, "holding", Condition::EQUALS, Condition::VAR, "true");
	AP_m[5*2].setCondJunctType(Condition::SIMPLE, Condition::CONJUNCTION);
	AP_m[5*2].setLabel("p_hold");
	*/

	/*
	SimpleCondition p_r;
	p_r.addCondition(Condition::SIMPLE, Condition::LABEL, "rock", Condition::EQUALS, Condition::VAR, "basket");
	p_r.setCondJunctType(Condition::SIMPLE, Condition::CONJUNCTION);
	p_r.setLabel("p_rL3");

	SimpleCondition p_a;
	p_a.addCondition(Condition::SIMPLE, Condition::LABEL, "alien", Condition::EQUALS, Condition::VAR, "container");
	p_a.setCondJunctType(Condition::SIMPLE, Condition::CONJUNCTION);
	p_a.setLabel("p_aL4");
	*/


	
	

	/* CREATE ENVIRONMENT FOR CAMERA */
	/*
	StateSpace SS_CAMERA;

	std::vector<std::string> pan_labels = {"left","center","right"};
	std::vector<std::string> tilt_labels = {"up","center","down"};
	std::vector<std::string> power_labels = {"on","off"};
	
	// Create state space:
	SS_CAMERA.setStateDimension(pan_labels, 0); // pan
	SS_CAMERA.setStateDimension(tilt_labels, 1); // tilt
	SS_CAMERA.setStateDimension(power_labels, 2); // power

	// Label state space:
	SS_CAMERA.setStateDimensionLabel(0, "pan");
	SS_CAMERA.setStateDimensionLabel(1, "tilt");
	SS_CAMERA.setStateDimensionLabel(2, "power");

	// Create object location group:
	std::vector<std::string> point_group = {"pan", "tilt"};
	SS_CAMERA.setLabelGroup("pointing locations", point_group);

	// Set the initial state:
	std::vector<std::string> set_state_c = {"center", "center", "off"};
	State init_state_c(&SS_CAMERA);	
	init_state_c.setState(set_state_c);

	//State test_state(&SS_MANIPULATOR);	
	//test_state.setState(test_set_state);
	*/

	/* SET CONDITIONS */
	/*
	// Pickup domain conditions:
	std::vector<Condition> conds_c;
	std::vector<Condition*> cond_ptrs_c;
	conds_c.resize(6);
	cond_ptrs_c.resize(6);

	// Turn On
	conds_c[0].addCondition(Condition::PRE, Condition::LABEL, "power", Condition::EQUALS, Condition::VAR, "off");
	conds_c[0].setCondJunctType(Condition::PRE, Condition::CONJUNCTION);

	conds_c[0].addCondition(Condition::POST, Condition::LABEL, "power", Condition::EQUALS, Condition::VAR, "on");
	conds_c[0].setCondJunctType(Condition::POST, Condition::CONJUNCTION);
	conds_c[0].setActionLabel("power_on");

	// Turn Off
	conds_c[1].addCondition(Condition::PRE, Condition::LABEL, "power", Condition::EQUALS, Condition::VAR, "on");
	conds_c[1].setCondJunctType(Condition::PRE, Condition::CONJUNCTION);

	conds_c[1].addCondition(Condition::POST, Condition::LABEL, "power", Condition::EQUALS, Condition::VAR, "off");
	conds_c[1].setCondJunctType(Condition::POST, Condition::CONJUNCTION);
	conds_c[1].setActionLabel("power_on");

	// Pan Center 
	conds_c[2].addCondition(Condition::PRE, Condition::LABEL, "pan", Condition::EQUALS, Condition::VAR, "center", Condition::NEGATE, "not_center");
	conds_c[2].setCondJunctType(Condition::PRE, Condition::CONJUNCTION); // Used to store eeLoc pre-state variable
	conds_c[2].addCondition(Condition::POST, Condition::LABEL, "pan", Condition::EQUALS, Condition::VAR, "center", Condition::TRUE, "center");
	conds_c[2].setCondJunctType(Condition::POST, Condition::CONJUNCTION);
	conds_c[2].setActionLabel("pan_center");
	
	// Pan Off Center 
	conds_c[3].addCondition(Condition::PRE, Condition::LABEL, "pan", Condition::EQUALS, Condition::VAR, "center", Condition::TRUE, "center");
	conds_c[3].setCondJunctType(Condition::PRE, Condition::CONJUNCTION); // Used to store eeLoc pre-state variable
	conds_c[3].addCondition(Condition::POST, Condition::LABEL, "pan", Condition::EQUALS, Condition::VAR, "center", Condition::NEGATE, "not_center");
	conds_c[3].setCondJunctType(Condition::POST, Condition::CONJUNCTION);
	conds_c[3].setActionLabel("pan_off_center");

	// Tilt Center 
	conds_c[4].addCondition(Condition::PRE, Condition::LABEL, "tilt", Condition::EQUALS, Condition::VAR, "center", Condition::NEGATE, "not_center");
	conds_c[4].setCondJunctType(Condition::PRE, Condition::CONJUNCTION); // Used to store eeLoc pre-state variable
	conds_c[4].addCondition(Condition::POST, Condition::LABEL, "tilt", Condition::EQUALS, Condition::VAR, "center", Condition::TRUE, "center");
	conds_c[4].setCondJunctType(Condition::POST, Condition::CONJUNCTION);
	conds_c[4].setActionLabel("tilt_center");
	
	// Tilt Off Center 
	conds_c[5].addCondition(Condition::PRE, Condition::LABEL, "tilt", Condition::EQUALS, Condition::VAR, "center", Condition::TRUE, "center");
	conds_c[5].setCondJunctType(Condition::PRE, Condition::CONJUNCTION); // Used to store eeLoc pre-state variable
	conds_c[5].addCondition(Condition::POST, Condition::LABEL, "tilt", Condition::EQUALS, Condition::VAR, "center", Condition::NEGATE, "not_center");
	conds_c[5].setCondJunctType(Condition::POST, Condition::CONJUNCTION);
	conds_c[5].setActionLabel("tilt_off_center");

	
	for (int i=0; i<conds_c.size(); ++i){
		cond_ptrs_c[i] = &conds_c[i];
	}


	SimpleCondition p_p;
	p_p.addCondition(Condition::SIMPLE, Condition::LABEL, "pan", Condition::EQUALS, Condition::VAR, "left");
	p_p.setCondJunctType(Condition::SIMPLE, Condition::CONJUNCTION);
	p_p.setLabel("p_p");

	SimpleCondition p_t;
	p_t.addCondition(Condition::SIMPLE, Condition::LABEL, "tilt", Condition::EQUALS, Condition::VAR, "down");
	p_t.setCondJunctType(Condition::SIMPLE, Condition::CONJUNCTION);
	p_t.setLabel("p_t");
	
	SimpleCondition p_on;
	p_on.addCondition(Condition::SIMPLE, Condition::LABEL, "power", Condition::EQUALS, Condition::VAR, "on");
	p_on.setCondJunctType(Condition::SIMPLE, Condition::CONJUNCTION);
	p_on.setLabel("p_on");
	*/


	/* DFA_m & Graph Instantiations For Manipulator */
	std::cout<<"before the graph instantiations... "<<std::endl;	
	Edge TS_m1(true);
	Edge DFA_m1(true);
	Edge PS_m1(true);
	Edge TS_m2(true);
	Edge DFA_m2(true);
	Edge PS_m2(true);
	
	// Hard-code DFA_m automaton:
	DFA_m1.connect(2, 0, 1.0, "p_aL4 & p_rL3");
	DFA_m1.connect(2, 2, 1.0, "!p_aL4 & !p_rL3");
	DFA_m1.connect(2, 1, 1.0, "p_aL4 & !p_rL3");
	DFA_m1.connect(2, 3, 1.0, "!p_aL4 & p_rL3");
	DFA_m1.connect(1, 1, 1.0, "!p_rL3");
	DFA_m1.connect(1, 0, 1.0, "p_rL3");
	DFA_m1.connect(3, 3, 1.0, "!p_aL4");
	DFA_m1.connect(3, 0, 1.0, "p_aL4");
	//DFA_m1.print();
	//std::cout<<"DFA 1 listsize: "<<DFA_m1.returnListCount()<<std::endl;

	DFA_m2.connect(1, 1, 1.0, "!p_aL2 | !p_rL3");
	DFA_m2.connect(1, 0, 1.0, "p_aL2 & p_rL3");
	//std::cout<<"DFA 2 listsize: "<<DFA_m2.returnListCount()<<std::endl;
	//DFA_m2.print();

	// Define the product systems after the automatons have been defined
	ProductSystem<State> PRODSYS_m1(&TS_m1, &DFA_m1, &PS_m1);
	ProductSystem<State> PRODSYS_m2(&TS_m2, &DFA_m2, &PS_m2);

	
	// Set the pre and post conditions, same for both systems in this case
	PRODSYS_m1.setConditions(cond_ptrs_m);
	PRODSYS_m2.setConditions(cond_ptrs_m);
	
	// Set the atomic propositions and define the initial and accepting states

	PRODSYS_m1.setPropositions(AP_m_ptrs);
	PRODSYS_m1.setAutomatonInitStateIndex(2);
	PRODSYS_m1.addAutomatonAcceptingStateIndex(0);

	PRODSYS_m2.setPropositions(AP_m_ptrs);
	PRODSYS_m2.setAutomatonInitStateIndex(1);
	PRODSYS_m2.addAutomatonAcceptingStateIndex(0);



	/* DFA_m & Graph Instantiations For Camera */
	/*
	Edge TS_c(true);
	Edge DFA_c(true);
	Edge PS_c(true);
	
	//bool didwork = conds_m[3].evaluate(&init_state, &test_state);
	//std::cout<<"is true??: "<<didwork<<std::endl;

	// Hard-code DFA_m automaton:
	DFA_c.connect(1, 0, 1.0, "p_on & p_p & p_t");
	DFA_c.connect(1, 1, 1.0, "!p_on | !p_p | !p_t");
	//DFA_c.print();

	ProductSystem<State> PRODSYS_c(&TS_c, &DFA_c, &PS_c);
	//TransitionSystem<State> test(&TS_c);
	PRODSYS_c.setConditions(cond_ptrs_c);
	//test.setConditions(cond_ptrs_c);
	//test.setInitState(&init_state_c);
	//test.generate();
	//test.print();

	
	PRODSYS_c.addProposition(&p_p);
	PRODSYS_c.addProposition(&p_t);
	PRODSYS_c.addProposition(&p_on);
	PRODSYS_c.setAutomatonInitStateIndex(1);
	PRODSYS_c.addAutomatonAcceptingStateIndex(0);
	//PRODSYS_c.print();

	//std::vector<int> plan_c;
	//float pathlength_c;
	
	
	// FOR THE CAMERA
	// Set initial state, generate TS, compose with DFA, plan
	PRODSYS_c.setInitState(&init_state_c);
	PRODSYS_c.generate();
	PRODSYS_c.compose();
	PRODSYS_c.plan();
	PRODSYS_c.getPlan(state_sequence_c, action_sequence_c);
	*/
	std::vector<State*> state_sequence_m, temp_state_seq;
	std::vector<std::string> action_sequence_m, temp_act_seq;

	State* curr_init_state;	
	curr_init_state = &init_state;
	for (int ii=0; ii<label_plan.size(); ii++) {
		std::cout<<"Planning for: "<<label_plan[ii]<<std::endl;
		temp_state_seq.clear();
		temp_act_seq.clear();
		bool plan_found;
		if (label_plan[ii] == "phi_1") {
			PRODSYS_m1.setInitState(curr_init_state);
			PRODSYS_m1.generate();
			PRODSYS_m1.compose();
			plan_found = PRODSYS_m1.plan();
			if (!plan_found) {
				std::cout<<"Error: Plan for 'phi_1' failed"<<std::endl;	
				break;
			}
			PRODSYS_m1.getPlan(temp_state_seq, temp_act_seq);
			curr_init_state = temp_state_seq.back();

		} else if (label_plan[ii] == "phi_2") {
			PRODSYS_m2.setInitState(curr_init_state);
			PRODSYS_m2.generate();
			//PRODSYS_m2.printTS();
			std::cout<<"b4 compose"<<std::endl;
			PRODSYS_m2.compose();
			PRODSYS_m2.printPS();
			std::cout<<"b4 plan"<<std::endl;
			plan_found = PRODSYS_m2.plan();
			std::cout<<"af plan"<<std::endl;
			if (!plan_found) {
				std::cout<<"Error: Plan for 'phi_2' failed"<<std::endl;	
				break;
			}
			PRODSYS_m2.getPlan(temp_state_seq, temp_act_seq);
			curr_init_state = temp_state_seq.back();
		}
		// If we are appending on to the state sequence, pop the last element so the final
		// state and initial state are not repeated
		if (ii != 0) {
			std::cout<<"b4 pop"<<std::endl;
			state_sequence_m.pop_back();
			std::cout<<"Appending ACTION SEQUENCE of length: "<<temp_act_seq.size()<<std::endl;
			state_sequence_m.insert(state_sequence_m.end(), temp_state_seq.begin(), temp_state_seq.end());
			action_sequence_m.insert(action_sequence_m.end(), temp_act_seq.begin(), temp_act_seq.end());
		} else {
			state_sequence_m = temp_state_seq;
			action_sequence_m = temp_act_seq;
		}
	}
	std::cout<<"made it out of discrete planning phew!"<<std::endl;

	//PRODSYS_m.plan();
	//PRODSYS_m.getPlan(state_sequence_m, action_sequence_m);
	/*
	std::cout<<"PLAN C SIZE: "<<plan_c.size()<<std::endl;

	std::cout<<"\n\n-------------\n\n";

	std::cout<<"\nSchedule Plan Labels:";
	for (int i=0; i<label_plan.size(); ++i) {
		std::cout<<" -> "<<label_plan[i];	
	}
	std::cout<<"\n";

	std::cout<<"\n\n-------------\n\n";
	for (int ii=0; ii<label_plan.size(); ii++) {
		std::cout<<"Plan for: "<<label_plan[ii]<<std::endl;
		if (label_plan[ii] == "phi_1") {
			std::cout<<"\nCamera Plan:";
			for (int i=0; i<plan_c.size(); ++i) {
				std::cout<<" -> "<<plan_c[i];	
			}
			std::cout<<"\n";
		} else if (label_plan[ii] == "phi_2") {
			std::cout<<"\nManipulator Plan:";
			for (int i=0; i<plan_m.size(); ++i) {
				std::cout<<" -> "<<plan_m[i];	
			}
			std::cout<<"\n";
		}
	}
	*/

	/////////////////////////////////////////////////////////////////////////
	std::cout<<"before the manipulator stuff... "<<std::endl;
	static const std::string PLANNING_GROUP = "manipulator";

	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

	robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));
	const robot_state::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);

	planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
	planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "ready");
	moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
	visual_tools.deleteAllMarkers();
	visual_tools.trigger();

	std::cout<<"before the loading of planner... "<<std::endl;

	//LOADING A PLANNER
	boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
	planning_interface::PlannerManagerPtr planner_instance;
	std::string planner_plugin_name;

	//from tutorial

	if (!planning_demo_NH.getParam("planning_plugin", planner_plugin_name))
		ROS_FATAL_STREAM("Could not find planner plugin name");
	try
	{
		planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
					"moveit_core", "planning_interface::PlannerManager"));
	}
	catch (pluginlib::PluginlibException& ex)
	{
		ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
	}
	try
	{
		planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
		if (!planner_instance->initialize(robot_model, planning_demo_NH.getNamespace()))
			ROS_FATAL_STREAM("Could not initialize planner instance");
		ROS_INFO_STREAM("Using planner '" << planner_instance->getDescription() << "'");
	}

	catch (pluginlib::PluginlibException& ex)
	{
		const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
		std::stringstream ss;
		for (std::size_t i = 0; i < classes.size(); ++i)
			ss << classes[i] << " ";
		ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
				<< "Available plugins: " << ss.str());
	}
	/////////////////////////////////////////////////////////////////////////

	// HARD CODE LOCATIONS
	std::cout<<"Setting locations... " <<std::endl;
	std::vector<geometry_msgs::Pose> locations(5);
	locations[0].position.x = .4;
	locations[0].position.y = .4;
	locations[0].position.z = .4;
	locations[0].orientation.x = 0;
	locations[0].orientation.y = 0;
	locations[0].orientation.z = 0;
	locations[0].orientation.w = 1;

	locations[1].position.x = -.4;
	locations[1].position.y = .4;
	locations[1].position.z = .4;
	locations[1].orientation.x = 0;
	locations[1].orientation.y = 0;
	locations[1].orientation.z = 0;
	locations[1].orientation.w = 1;

	locations[2].position.x = 0.0;
	locations[2].position.y = .4;
	locations[2].position.z = .4;
	locations[2].orientation.x = 0;
	locations[2].orientation.y = 0;
	locations[2].orientation.z = 0;
	locations[2].orientation.w = 1;

	// This copies the locations above over the x axis, capped at 5 to match labels
	std::cout<<"in da loop"<<std::endl;
	for (int i=3; i<5; ++i) {
		locations[i].position.x = locations[i-3].position.x;
		locations[i].position.y = -locations[i-3].position.y;
		locations[i].position.z = locations[i-3].position.z;
		locations[i].orientation.w = 0;
		locations[i].orientation.w = 0;
		locations[i].orientation.w = 0;
		locations[i].orientation.w = 1;
		//visual_tools.publishText(locations[i], obj_labels[i], rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
		//visual_tools.trigger();
	}

	// Create the map to the location labels
	std::unordered_map<std::string, int> location_int_map;

	for (int i=0; i<obj_labels.size(); ++i) {
		location_int_map[obj_labels[i]] = i;
	}

	// Create the collision objects (2)
	std::vector<moveit_msgs::CollisionObject> col_objs(2);	
	col_objs[0].header.frame_id = "base_link";
	col_objs[0].id = "rock";
	col_objs[1].header.frame_id = "base_link";
	col_objs[1].id = "alien";
	for (int i=0; i<2; ++i) {
		col_objs[i].primitives.resize(1);
		col_objs[i].primitives[0].type = col_objs[i].primitives[0].BOX;
		col_objs[i].primitives[0].dimensions.resize(3);
		col_objs[i].primitives[0].dimensions[0] = .1;
		col_objs[i].primitives[0].dimensions[1] = .1;
		col_objs[i].primitives[0].dimensions[2] = .1;
		col_objs[i].primitive_poses.resize(1);
		col_objs[i].primitive_poses[0] = locations[i];
		col_objs[i].operation = col_objs[i].ADD;
	}

	planning_scene_interface.applyCollisionObjects(col_objs);
	
	// Execution
	std::cout<<"number of actions to carry out: "<<action_sequence_m.size()<<std::endl;
	move_group.setPlanningTime(5.0);
	for (int i=0; i<action_sequence_m.size(); ++i) {
		std::cout<<"\n\n"<<std::endl;
		std::cout<<"Currently working on action: "<<action_sequence_m[i]<<std::endl;
		std::cout<<"\nCurrent state: "<<std::endl;
		state_sequence_m[i]->print();
		std::cout<<"\nNext state: "<<std::endl;
		state_sequence_m[i+1]->print();

		if (action_sequence_m[i] == "grasp") {
			std::string arg_dim_label;
			bool found = state_sequence_m[i+1]->argFindGroup("ee", "object locations", arg_dim_label);
			if (found) {
				move_group.attachObject(arg_dim_label);
			} else {
				std::cout<<"Error: Object was not found in action 'grasp'"<<std::endl;
			}
		}	
		if (action_sequence_m[i] == "transport") {
			int pose_ind = location_int_map[state_sequence_m[i+1]->getVar("eeLoc")];
			geometry_msgs::Pose temp_pose = locations[pose_ind];
			temp_pose.position.z = temp_pose.position.z + .2;
			tf2::Quaternion temp_q;
			temp_q.setRPY(0, M_PI/2, 0);
			temp_pose.orientation.x = temp_q[0];
			temp_pose.orientation.y = temp_q[1];
			temp_pose.orientation.z = temp_q[2];
			temp_pose.orientation.w = temp_q[3];
			move_group.setStartStateToCurrentState();
			move_group.setPoseTarget(locations[pose_ind]);
			moveit::planning_interface::MoveGroupInterface::Plan plan_;
			bool success;
			success = move_group.plan(plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
			if (success) {
				std::cout<<"Plan succeeded!!! (transport)"<<std::endl;
				move_group.execute(plan_);
			} else {
				std::cout<<"Plan failed :( (transport)"<<std::endl;
			}
		}	
		if (action_sequence_m[i] == "transit") {
			int pose_ind = location_int_map[state_sequence_m[i+1]->getVar("eeLoc")];
			geometry_msgs::Pose temp_pose = locations[pose_ind];
			temp_pose.position.z = temp_pose.position.z + .2;
			tf2::Quaternion temp_q;
			temp_q.setRPY(0, M_PI/2, 0);
			//temp_pose.orientation = tf2::toMsg(temp_q);
			temp_pose.orientation.x = temp_q[0];
			temp_pose.orientation.y = temp_q[1];
			temp_pose.orientation.z = temp_q[2];
			temp_pose.orientation.w = temp_q[3];
			move_group.setStartStateToCurrentState();
			move_group.setPoseTarget(temp_pose);
			moveit::planning_interface::MoveGroupInterface::Plan plan_;
			bool success;
			std::cout<<"before planning... "<<std::endl;
			success = move_group.plan(plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
			std::cout<<"made it out of planning phew!"<<std::endl;
			if (success) {
				std::cout<<"Plan succeeded!!! (transit)"<<std::endl;
				move_group.execute(plan_);
			} else {
				std::cout<<"Plan failed :( (transit)"<<std::endl;
			}
		}
		if (action_sequence_m[i] == "release") {
			std::string arg_dim_label;
			bool found = state_sequence_m[i]->argFindGroup("ee", "object locations", arg_dim_label);
			if (found) {
				move_group.detachObject(arg_dim_label);
			} else {
				std::cout<<"Error: Object was not found in action 'release'"<<std::endl;
			}	
		}	
		//if (action_sequence_m[i] == "transit") {}	
	}



	return 0;
}
