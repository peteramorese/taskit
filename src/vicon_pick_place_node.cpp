#include <pluginlib/class_loader.h>
#include <ros/ros.h>


// MoveIt
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


const float pi=M_PI;

struct objStruc {
	std::string objID;
	std::string supportsurface;
	float posx;
	float posy;
	float posz;
	float approachx;
	float approachy;
	float approachz;
	float retreatx;
	float retreaty;
	float retreatz;
	float roll;
	float pitch;
	float yaw;
	float gripwidth;
	float setback;
} pickstruc, placestruc;

int pow(int num, int exp); 
float powf(float num, float exp);
float dot(float vec1[3], float vec2[3]); 
void cross(float *x, float *y, float *z, float vec1[3], float vec2[3]);
void openGrip(trajectory_msgs::JointTrajectory& handstate);
void closeGrip(trajectory_msgs::JointTrajectory& handstate, float width); 



//moveit::planning_interface::MoveItErrorCode planPick(const std::string& object, const std::vector<moveit_msgs::Grasp>& grasps, moveit::planning_interface::MoveGroupInterface::Plan& plan);
bool pickup(moveit::planning_interface::MoveGroupInterface& move_group, std::vector<moveit_msgs::CollisionObject> colObjIns, objStruc objS); 
bool place(moveit::planning_interface::MoveGroupInterface& group, objStruc objS); 


// Vim trash
// "gg =G" auto indent



int main(int argc, char** argv)
{
	ros::init(argc, argv, "pleasework");
	ros::NodeHandle node_handle("~");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	static const std::string PLANNING_GROUP = "panda_arm";

	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

	moveit::planning_interface::PlanningSceneInterface planning_scene_interface; 
	move_group.setPlanningTime(40);
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

	/* Create a RobotState and JointModelGroup to keep track of the current robot pose and planning group*/
	robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));
	const robot_state::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);

	planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
	planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "ready");





	namespace rvt = rviz_visual_tools;
	moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
	visual_tools.deleteAllMarkers();
	visual_tools.trigger();
	ROS_INFO_NAMED("pleasework", "Planning frame: %s", move_group.getPlanningFrame().c_str());
	ROS_INFO_NAMED("pleasework", "End effector link: %s", move_group.getEndEffectorLink().c_str());
	ROS_INFO_NAMED("pleasework", "Available Planning Groups:");
	std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
			std::ostream_iterator<std::string>(std::cout, ", "));








	/////////////////////////////////////////////////////////////////////////////////////////////////////

	//LOADING A PLANNER
	boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
	planning_interface::PlannerManagerPtr planner_instance;
	std::string planner_plugin_name;

	//from tutorial

	if (!node_handle.getParam("planning_plugin", planner_plugin_name))
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
		if (!planner_instance->initialize(robot_model, node_handle.getNamespace()))
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
	/////////////////////////////////////////////////////////////////////////////////////////////////////



	/*
	int Nobj=6;
	std::vector<moveit_msgs::CollisionObject> colObj;
	colObj.resize(Nobj);

	for (int i=0; i<Nobj; i++) {
		colObj[i].header.frame_id = move_group.getPlanningFrame();
	}




	colObj[0].id = "table1";
	colObj[1].id = "pillar1";

	// Pillar
	colObj[1].primitives.resize(1);
	colObj[1].primitives[0].type = colObj[1].primitives[0].CYLINDER;
	colObj[1].primitives[0].dimensions.resize(2);
	colObj[1].primitives[0].dimensions[0] = 2.5; //height
	colObj[1].primitives[0].dimensions[1] = 0.1; //radius

	// Table
	colObj[0].primitives.resize(1);
	colObj[0].primitives[0].type = colObj[0].primitives[0].BOX;
	colObj[0].primitives[0].dimensions.resize(3);
	colObj[0].primitives[0].dimensions[0] = 2; 
	colObj[0].primitives[0].dimensions[1] = 2;
	colObj[0].primitives[0].dimensions[2] = .1;

	// Things 
	float thingWidth = .04;
	float thingHeight = .2;
	float thingLength = .04;
	int Nthings = Nobj-2; // "2" for table and pillar

	for (int i=2; i<Nthings+2; i++) {
		colObj[i].primitives.resize(1);
		colObj[i].primitives[0].type = colObj[3].primitives[0].BOX;
		colObj[i].primitives[0].dimensions.resize(3);
		colObj[i].primitives[0].dimensions[0] = thingLength; 
		colObj[i].primitives[0].dimensions[1] = thingWidth;
		colObj[i].primitives[0].dimensions[2] = thingHeight;
	}
	// Positions
	float objPosArr [Nobj-Nthings][4] = {{1,0,0,-.06}, {1,0,.5,1.25}};

	for (int i=0; i<(Nobj-Nthings); i++) {
		colObj[i].primitive_poses.resize(1);
		colObj[i].primitive_poses[0].orientation.w = objPosArr[i][0];
		colObj[i].primitive_poses[0].position.x = objPosArr[i][1];
		colObj[i].primitive_poses[0].position.y = objPosArr[i][2];
		colObj[i].primitive_poses[0].position.z = objPosArr[i][3];
		colObj[i].operation = colObj[i].ADD; //table, pillar1
	}


	///////////////////////// Object State & Positions //////////////////////////////////

	int Nsubstate = 9;
	float tol = .0008;
	float pullback = .085 + thingLength/2;
	float ymove = .25;
	// x, y, z, ax, ay, az, rx, ry, rz, r, p, y, objw(1)
	// Vertical index of these arrays defines states
	double state_objPick [Nsubstate][13] {
		{ .8,-.3,thingHeight/2, 1.0, 0, 0, 0, 0, 1.0, -pi/2, -pi/4, -pi/2, 1}, 
			{ .8,  0,thingHeight/2, 1.0, 0, 0, 0, 0, 1.0, -pi/2, -pi/4, -pi/2, 1}, 
			{ .8, .3,thingHeight/2, 1.0, 0, 0, 0, 0, 1.0, -pi/2, -pi/4, -pi/2, 1}, 
			{ .4, .6,thingHeight/2, 0, 1.0, 0, 0, 0, 1.0, -pi/2, -pi/4, 0, 1}, 
			{-.7,-thingHeight/2, thingHeight/2+tol, -1.0, 0, 0, 0, 0, 1.0, 0, 0, pi, 1}, 
			{-.7, thingHeight/2, thingHeight/2+tol, -1.0, 0, 0, 0, 0, 1.0, 0, 0, pi, 1}, 
			{-.7, 0, thingHeight+thingWidth/2+4*tol, -1.0, 0, 0, 0, 0, 1.0, pi/2, 0, pi, 1},
			{-.7, 0, thingHeight+thingWidth+thingHeight/2+6*tol, -1.0, 0, 0, 0, 0, 1.0, 0, 0, pi, 1},
			{  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
	};
	// Thing# ("0" of nothing exists)
	int task_objTracker[Nsubstate] = {1, 2, 3, 4, 0, 0, 0, 0, 0};

	double state_objPlace [Nsubstate][12] {
		{ .8,-.3,thingHeight/2, 0, 0, -1.0, -1.0, 0, 0, -pi/2, -pi/4, -pi/2}, 
			{ .8,  0,thingHeight/2, 0, 0, -1.0, -1.0, 0, 0, -pi/2, -pi/4, -pi/2}, 
			{ .8, .3,thingHeight/2, 0, 0, -1.0, -1.0, 0, 0, -pi/2, -pi/4, -pi/2},
			{ .4, .6,thingHeight/2, 0, 0, -1.0, -1.0, 0, 0, -pi/2, -pi/4, -pi/2},
			{-.6,-thingHeight/2+ymove, thingHeight/2+tol, 0, 0, -1.0, 1, 0, 0, 0, 0, pi}, 
			{-.6, thingHeight/2+ymove, thingHeight/2+tol, 0, 0, -1.0, 1, 0, 0, 0, 0, pi}, 
			{-.6, ymove, thingHeight+thingWidth/2+2*tol, -1.0, 0, 0, 1, 0, 0, pi/2, 0, pi},
			{-.6, ymove, thingHeight+thingWidth+thingHeight/2+3*tol, 0, 0, -1.0, 1, 0, 0, 0, 0, pi/2},
			{  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
	};

	// Positions of things
	for (int i=0; i<Nsubstate; i++) {
		if (task_objTracker[i]!=0) {
			int ind = task_objTracker[i]-1 + Nobj - Nthings;
			colObj[ind].id = "thing" + std::to_string(i+1);
			//ROS_INFO_NAMED("pleasework","%s was added.", colObj[ind].id);
			colObj[ind].primitive_poses.resize(1);
			colObj[ind].primitive_poses[0].orientation.w = state_objPick[i][12];
			colObj[ind].primitive_poses[0].position.x = state_objPick[i][0];
			colObj[ind].primitive_poses[0].position.y = state_objPick[i][1];
			colObj[ind].primitive_poses[0].position.z = state_objPick[i][2];
			colObj[ind].operation = colObj[ind].ADD; 
		}
	}
	// Change the position of the object to the position the eef must arrive at
	for (int i=0; i<Nsubstate; i++) {
		for (int ii=0; ii<3; ii++) {
			if (state_objPick[i][ii+3]==1.0) {
				state_objPick[i][ii]=state_objPick[i][ii] - pullback;
			} else if (state_objPick[i][ii+3]==-1.0) {		
				state_objPick[i][ii]=state_objPick[i][ii] + pullback;
			}
			ROS_INFO_NAMED("pleasework"," %.1f",state_objPick[i][ii]);

		}
	}

	std::vector<std::string> colObjIDs;
	for (int i=0; i<Nobj; i++){
		colObjIDs.push_back(colObj[i].id);
	}

	planning_scene_interface.addCollisionObjects(colObj);


	/////////////////////////////// Graph Math //////////////////////////////////////////

	int initstateNum = -1;
	int goalstateNum = -1;
	std::vector<int> initstate = {1,1,1,1,0,0,0,0,0};
	std::vector<int> goalstate = {0,0,0,0,1,1,1,1,0};	

	// CREATE VECTOR CONTAINING ALL STATES
	std::vector<int> stateMat;
	int Nstate = 0;
	std::vector<int> tempVec;
	for (int i=0; i<Nsubstate; i++) {
		tempVec.push_back(0);
	}
	for (int j=0; j<(pow(2,Nsubstate)-1); j++) {
		int dec = j;
		int a = 0;
		//ROS_INFO_NAMED("pleasework","Im still alive: %d, %d", j, pow(2,Nsubstate)-1);

		for (int i=0; i<(Nsubstate); i++){
			if ((dec-pow(2,Nsubstate-1-i))>=0) {
				dec=dec-pow(2,Nsubstate-1-i);
				tempVec[i]=1;
				a++;

				//ROS_INFO_NAMED("pleasework","Hello here is dec: %d, with j=%d", dec, j);
			} else {
				tempVec[i]=0;
			}
		}
		if (a==Nthings) {
			// Horizontal block hangning in air
			if (!((tempVec[6]==1)&&((tempVec[4]==0)||(tempVec[5]==0)))) {
				if (!((tempVec[7]==1)&&((tempVec[4]==0)||((tempVec[5]==0)||tempVec[6]==0)))) {
					Nstate++; // "1" start indexing
					ROS_INFO_NAMED("pleasework","\n\nState %d (%d) was added and converted to:", Nstate, j); 
					for (int ii=0; ii<(Nsubstate); ii++){
						stateMat.push_back(tempVec[ii]);
						ROS_INFO_NAMED("pleasework","%d", tempVec[ii]);
					}
					if (tempVec==initstate) {
						initstateNum=Nstate;
						ROS_INFO_NAMED("pleasework", "This is the initial state");
					} else if (tempVec==goalstate) {
						goalstateNum=Nstate;
						ROS_INFO_NAMED("pleasework", "This is the goal state");
					}
				}
			}
		}
	}
	// Here is how to index the first element in the state tuple: stateMat[Nsubstate*(ind-1)]
	// States are named numerically by this index

	// Check to make sure that initial and goal states are included
	if ((initstateNum==-1)||(goalstateNum==-1)) {
		ROS_ERROR_NAMED("pleasework","Either the initial state or the goal state was excluded");
		ROS_BREAK();
	}

	// PROPAGATE ALL EDGES
	std::vector<int> edgeMat;
	std::vector<float> edgeLength;
	std::vector<int> tempVecd;
	int Nedge = 0;
	// Edges defined by a two element tuple where elements represent connected states/nodes
	// Edit the indexing to include self-loop edges or directional edges
	for (int i=2; i<=Nstate; i++) {
		for (int ii=1; ii<i; ii++) {
			int diff = 0;
			tempVecd.clear();
			for (int iii=0; iii<Nsubstate; iii++) {
				int a = abs(stateMat[Nsubstate*(ii-1)+iii]-stateMat[Nsubstate*(i-1)+iii]);
				diff=diff+a;
				if (a!=0) {
					tempVecd.push_back(iii);
				}
				//ROS_INFO_NAMED("pleasework","diff=%d",diff);
			}
			if (diff==2) {
				edgeMat.push_back(i);
				edgeMat.push_back(ii);
				float cartWeight; // Weight edges by cartesian distance between state locations
				//cartWeight = sqrt(powf((state_objPick[tempVecd[0]][0]-state_objPick[tempVecd[1]][0]),2)
				//		+powf((state_objPick[tempVecd[0]][1]-state_objPick[tempVecd[1]][1]),2)
				//		+powf((state_objPick[tempVecd[0]][2]-state_objPick[tempVecd[1]][2]),2));
				cartWeight = 1;
				edgeLength.push_back(cartWeight); // Use this to propagate edge lengths/weights
				Nedge++;
				ROS_INFO_NAMED("pleasework","Edge connecting state %d and %d with weight= %.3f",i,ii,cartWeight);
			}
		}
	}

	// FIND THE SHORTEST PATH (Dijkstra's Algorithm)
	float distSP[Nstate];
	bool done = false;
	int currentSP = initstateNum; // "1" start indexing
	ROS_INFO_NAMED("pleasework","currentSP = %d ", currentSP);
	bool visit[Nstate];
	int parent[Nstate];
	float minVal = -1.0;
	int minInd;
	// Assign distance to all nodes, init node is zero all other nodes are infinity (-1.0)
	for (int i=0; i<Nstate; i++) {
		parent[i] = 0;
		visit[i] = false;
		if ((i+1)==initstateNum) {
			distSP[i]=0;	
		} else {
			distSP[i]=-1.0;
		}
	}
	while (!done) {
		for (int ii=0; ii<Nedge; ii++) {
			if (edgeMat[2*ii]==currentSP) {
				if (!visit[edgeMat[2*ii+1]-1]) {
					float a = distSP[edgeMat[2*ii+1]-1]; // Current distance value of neighbor node
					float b = edgeLength[ii] + distSP[currentSP-1]; // Current distance plus length of edge
					if ((a==-1.0)||(a > b)) {
						parent[edgeMat[2*ii+1]-1] = currentSP-1; //currentSP; // Index is the node, value is the parent to that node ("0" start)
						distSP[edgeMat[2*ii+1]-1] = b;
					} 
					//ROS_INFO_NAMED("pleasework"," distance element %d is %.1f", edgeMat[2*ii+1], distSP[edgeMat[2*ii+1]-1]);
				}
			} else if (edgeMat[2*ii+1]==currentSP) {
				if (!visit[edgeMat[2*ii]-1]) {
					float a = distSP[edgeMat[2*ii]-1];
					float b = edgeLength[ii] + distSP[currentSP-1]; 
					if ((a==-1.0)||(a > b)) {
						parent[edgeMat[2*ii]-1] = currentSP-1; //currentSP;//currentSP-1;
						distSP[edgeMat[2*ii]-1] = b;
					}
					//ROS_INFO_NAMED("pleasework"," distance element %d is %.1f", edgeMat[2*ii], distSP[edgeMat[2*ii]-1]);
				}
			}
		}
		visit[currentSP-1] = true;
		minVal = -1.0;
		//ROS_INFO_NAMED("pleasework"," current node: %d ", currentSP);
		for (int iii = 0; iii<Nstate; iii++) {
			if (!visit[iii]) {
				if (minVal==-1.0) {
					minVal = distSP[iii];
					minInd = iii;
				} else if ((distSP[iii] < minVal)&&(distSP[iii] >= 0)) {
					minVal = distSP[iii];
					minInd = iii;
				}
				//ROS_INFO_NAMED("pleasework","Unvisitied, minInd= %d", minInd);
			}
		}
		currentSP = minInd+1;
		if (minInd+1 == goalstateNum) {
			done = true;
			//ROS_INFO_NAMED("pleasework","Minimum Distance = %.1f",minVal);
		}
	}

	// Shortest distance calculated, parent already array assembled, trace through parent array from finish to start to assemble path
	int tempstate = goalstateNum-1;
	std::vector<int> pathSPr;
	std::vector<int> pathSP;
	pathSPr.push_back(goalstateNum-1);
	ROS_INFO_NAMED("pleasework", "goalstateNum = %d",goalstateNum);
	while (tempstate!=(initstateNum-1)) {
		tempstate = parent[tempstate];
		pathSPr.push_back(tempstate);
	} 	
	//pathSPr.pop_back();
	//ROS_INFO_NAMED("pleasework"," {PATH} pathSPr %d %d %d %d", pathSPr[0], pathSPr[1], pathSPr[2], pathSPr[3]);
	for (int i=pathSPr.size()-1; i>=0; i--) {

		pathSP.push_back(pathSPr[i]+1);
		//ROS_INFO_NAMED("pleasework"," {PATH} Element %d = State %d", i+1, pathSPr[i]+1);
		//ROS_INFO_NAMED("pleasework"," {PATH} %d %d %d %d", pathSP[0], pathSP[1], pathSP[2], pathSP[3]);
	} 
	int Ntasks = pathSP.size();	

*/
	////////////////////////////////// Initialize ///////////////////////////////////////

	//visual_tools.prompt("done initializing workspace. press next to init pose");

	// SET INITIAL POSE
	move_group.setEndEffectorLink("panda_link8");
	geometry_msgs::Pose initpose;
	tf2::Quaternion initorient;
	initorient.setRPY(-pi/2, -pi/4, -pi/2);
	initpose.orientation = tf2::toMsg(initorient);

	initpose.position.x = .4;
	initpose.position.y = 0;
	initpose.position.z = .15;
	move_group.setStartStateToCurrentState();
	move_group.setPoseTarget(initpose);
	move_group.setPlanningTime(15.0);
	moveit::planning_interface::MoveGroupInterface::Plan initplan;
	move_group.plan(initplan);
	move_group.execute(initplan);

	visual_tools.prompt("done initializing. press to embark! :)");


	////////////////////////////// Convert Path to Plan /////////////////////////////////
	
/*
	std::vector<int> taskpick;
	std::vector<int> taskplace;
	ROS_INFO_NAMED("pleasework","Number of Tasks %d", Ntasks);
	for (int i=0; i<Ntasks-1; i++) {
		int placestate = pathSP[i+1];
		int pickstate = pathSP[i];
		//ROS_INFO_NAMED("pleasework","placestate %d pickstate %d",placestate,pickstate);
		int diff = 0;
		for (int ii=0; ii<Nsubstate; ii++) {
			diff=stateMat[Nsubstate*(placestate-1)+ii]-stateMat[Nsubstate*(pickstate-1)+ii];
			if (diff==-1) {
				taskpick.push_back(ii);
			} else if (diff==1) {
				taskplace.push_back(ii);
			}
			//ROS_INFO_NAMED("pleasework","diff=%d",diff);
		}
	}



	/////////////////////////////// Plan and Execute ////////////////////////////////////
	pickstruc.supportsurface = "table1";
	placestruc.supportsurface = "table1";
	for (int i=0; i<Ntasks-1; i++) {
		int a = taskpick[i];
		int b = taskplace[i];
		ROS_INFO_NAMED("pleasework","a=%d, b=%d",a,b);
		if (task_objTracker[a]!=0) {
			int currObj = task_objTracker[a]+1;
			pickstruc.objID = colObjIDs[currObj];
			placestruc.objID = colObjIDs[currObj];
			std::map<std::string, geometry_msgs::Pose> cPose;
			cPose = planning_scene_interface.getObjectPoses(colObjIDs);
			ROS_INFO_NAMED("pleasework","Position of thing: %.1f %.1f %.1f", cPose[colObjIDs[currObj]].position.x, cPose[colObjIDs[currObj]].position.y, cPose[colObjIDs[currObj]].position.z);

			task_objTracker[b] = task_objTracker[a];
			task_objTracker[a] = 0;
		} else {
			pickstruc.objID = "No Object";
			placestruc.objID = "No Object";
		}
		pickstruc.posx = state_objPick[a][0];
		pickstruc.posy = state_objPick[a][1];
		pickstruc.posz = state_objPick[a][2];
		pickstruc.approachx = state_objPick[a][3];
		pickstruc.approachy = state_objPick[a][4];
		pickstruc.approachz = state_objPick[a][5];
		pickstruc.retreatx = state_objPick[a][6];
		pickstruc.retreaty = state_objPick[a][7];
		pickstruc.retreatz = state_objPick[a][8];
		pickstruc.roll = state_objPick[a][9];
		pickstruc.pitch = state_objPick[a][10];
		pickstruc.yaw = state_objPick[a][11];
		pickstruc.gripwidth = thingWidth;
		pickstruc.setback = 0; //thingLength/2;

		placestruc.posx = state_objPlace[b][0];
		placestruc.posy = state_objPlace[b][1];
		placestruc.posz = state_objPlace[b][2];
		placestruc.approachx = state_objPlace[b][3];
		placestruc.approachy = state_objPlace[b][4];
		placestruc.approachz = state_objPlace[b][5];
		placestruc.retreatx = state_objPlace[b][6];
		placestruc.retreaty = state_objPlace[b][7];
		placestruc.retreatz = state_objPlace[b][8];
		placestruc.roll = state_objPlace[b][9];
		placestruc.pitch = state_objPlace[b][10];
		placestruc.yaw = state_objPlace[b][11];
		placestruc.gripwidth = thingWidth;
		placestruc.setback = 0; //thingLength/2;

		bool success;
		move_group.setPlanningTime(20.0);
		ros::WallDuration(1.0).sleep();
		success = pickup(move_group, colObj, pickstruc);
		ros::WallDuration(1.0).sleep();
		if (!success) {
			ROS_ERROR_NAMED("pleasework","Planning Failed in pickup");
			ROS_BREAK();
		}
		ROS_INFO_NAMED("pleasework","Done picking up");
		success = place(move_group, placestruc);
		if (!success) {
			move_group.detachObject(placestruc.objID);
			ROS_ERROR_NAMED("pleasework","Planning Failed when placing");
			ROS_BREAK();
		}
	}

	//////////////////////////////////// SHUTDOWN //////////////////////////////////////
	
	move_group.setPoseTarget(initpose);
	move_group.setPlanningTime(10.0);
	move_group.plan(initplan);
	move_group.execute(initplan); //move panda to init pose

	planning_scene_interface.removeCollisionObjects(colObjIDs); 
	ros::shutdown();

	////////////////////////////////////////////////////////////////////////////////////
*/	
	return 0;
}

int pow(int num, int exp) {
	if (exp==0){
		return 1;
	} else {
		int base = num;
		for (int i=1; i<exp; i++){
			base = base*num;
		}
		return base;
	}
}

float powf(float num, float exp) {
	if (exp==0){
		return 1;
	} else {
		float base = num;
		for (int i=1; i<exp; i++){
			base = base*num;
		}
		return base;
	}
}







float dot(float *A, float *B) {
	float vec1[3]={*(A),*(A+1),*(A+2)};
	float vec2[3]={*(B),*(B+1),*(B+2)};
	float dotproduct = vec1[0]*vec2[0]+vec1[1]*vec2[1]+vec1[2]*vec2[2];	


	ROS_INFO_NAMED("pleasework","v1 %f, %f, %f",vec1[0], vec1[1], vec1[2]);
	ROS_INFO_NAMED("pleasework","v2 %f, %f, %f",vec2[0], vec2[1], vec2[2]);
	return dotproduct;
}

void cross(float *x, float *y, float *z, float *A, float *B) {
	float vec1[3]={*(A),*(A+1),*(A+2)};
	float vec2[3]={*(B),*(B+1),*(B+2)};

	float crossproduct[3];
	*x=vec1[1]*vec2[2]-vec1[2]*vec2[1];
	*y=vec1[2]*vec2[0]-vec1[0]*vec2[2];
	*z=vec1[0]*vec2[1]-vec1[1]*vec2[0];
}


void openGrip(trajectory_msgs::JointTrajectory& handstate) {
	handstate.joint_names.resize(2);
	handstate.joint_names[0]="panda_finger_joint1";
	handstate.joint_names[1]="panda_finger_joint2";
	handstate.points.resize(1);
	handstate.points[0].positions.resize(2);
	handstate.points[0].positions[0]=0.04;
	handstate.points[0].positions[1]=0.04;
	handstate.points[0].time_from_start=ros::Duration(.5);
}

void closeGrip(trajectory_msgs::JointTrajectory& handstate, float width) {
	handstate.joint_names.resize(2);
	handstate.joint_names[0]="panda_finger_joint1";
	handstate.joint_names[1]="panda_finger_joint2";
	handstate.points.resize(1);
	handstate.points[0].positions.resize(2);
	handstate.points[0].positions[0]=width/2; //.04 for open, .025 for close
	handstate.points[0].positions[1]=width/2;
	handstate.points[0].time_from_start=ros::Duration(.5);
}

bool pickup(moveit::planning_interface::MoveGroupInterface& move_group, std::vector<moveit_msgs::CollisionObject> colObjIns, objStruc objS) {	
	std::vector<moveit_msgs::Grasp> grasps;
	grasps.resize(1);
	grasps[0].grasp_pose.header.frame_id = "panda_link0";
	tf2::Quaternion orient;
	//orient.setRPY(-pi/2, -pi/4, -pi/2);
	orient.setRPY(objS.roll, objS.pitch, objS.yaw);
	grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orient);
	grasps[0].grasp_pose.pose.position.x = objS.posx-objS.setback;//.09
	//grasps[0].grasp_pose.pose.position.y = colObjIns[3].primitive_poses[0].position.y;
	grasps[0].grasp_pose.pose.position.y = objS.posy;
	grasps[0].grasp_pose.pose.position.z = objS.posz;
	ROS_INFO_NAMED("pleasework","Target Pickup x: %.2f y: %.2f z: %.2f", objS.posx-objS.setback, objS.posy, objS.posz);
	// This defines the pre_grasp_posture
	grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
	grasps[0].pre_grasp_approach.direction.vector.x = objS.approachx; //unit in direction of approach
	grasps[0].pre_grasp_approach.direction.vector.y = objS.approachy;
	grasps[0].pre_grasp_approach.direction.vector.z = objS.approachz;
	grasps[0].pre_grasp_approach.min_distance = .05;//.095; //values copied from tutorial
	grasps[0].pre_grasp_approach.desired_distance = .08; //.115;  

	grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0";
	grasps[0].post_grasp_retreat.direction.vector.x = objS.retreatx;
	grasps[0].post_grasp_retreat.direction.vector.y = objS.retreaty;
	grasps[0].post_grasp_retreat.direction.vector.z = objS.retreatz;
	grasps[0].post_grasp_retreat.min_distance = .05;//.1;
	grasps[0].post_grasp_retreat.desired_distance = .08;//.25;

	openGrip(grasps[0].pre_grasp_posture); //Open grip in the approach
	closeGrip(grasps[0].grasp_posture, objS.gripwidth);

	move_group.setSupportSurfaceName(objS.supportsurface);

	moveit::planning_interface::MoveGroupInterface::Plan testplan;
	
	bool success;
	success = (move_group.pick(objS.objID, grasps)==moveit::planning_interface::MoveItErrorCode::SUCCESS);
	ROS_INFO_NAMED("Pickup plan: %s", success ? "Success" : "Failed");
	return success;
}

bool place(moveit::planning_interface::MoveGroupInterface& move_group, objStruc objS) {
	std::vector<moveit_msgs::PlaceLocation> placeloc;
	placeloc.resize(1);
	placeloc[0].place_pose.header.frame_id = "panda_link0";
	tf2::Quaternion orient;
	//orient.setRPY(0, 0, pi);
	orient.setRPY(objS.roll, objS.pitch, objS.yaw);	
	placeloc[0].place_pose.pose.orientation = tf2::toMsg(orient);
	placeloc[0].place_pose.pose.position.x = objS.posx; //this marks the center of the object
	placeloc[0].place_pose.pose.position.y = objS.posy;
	placeloc[0].place_pose.pose.position.z = objS.posz;
	ROS_INFO_NAMED("pleasework","Target Place x: %.2f y: %.2f z: %.2f", objS.posx, objS.posy, objS.posz);

	placeloc[0].pre_place_approach.direction.header.frame_id = "panda_link0";
	placeloc[0].pre_place_approach.direction.vector.x = objS.approachx; //unit in direction of approach
	placeloc[0].pre_place_approach.direction.vector.y = objS.approachy;
	placeloc[0].pre_place_approach.direction.vector.z = objS.approachz;
	placeloc[0].pre_place_approach.min_distance = .05;//.095; //values copied from tutorial
	placeloc[0].pre_place_approach.desired_distance = .08;//.115;  

	placeloc[0].post_place_retreat.direction.header.frame_id = "panda_link0";
	placeloc[0].post_place_retreat.direction.vector.x = objS.retreatx;
	placeloc[0].post_place_retreat.direction.vector.y = objS.retreaty;
	placeloc[0].post_place_retreat.direction.vector.z = objS.retreatz;
	placeloc[0].post_place_retreat.min_distance = .05;//.1;
	placeloc[0].post_place_retreat.desired_distance = .08;//.25;

	openGrip(placeloc[0].post_place_posture); //Open grip in the approach
	move_group.setSupportSurfaceName(objS.supportsurface);
	
	bool success;
	success = (move_group.place(objS.objID, placeloc)==moveit::planning_interface::MoveItErrorCode::SUCCESS);
	ROS_INFO_NAMED("Pickup plan: %s", success ? "Success" : "Failed");
	return success;
}

