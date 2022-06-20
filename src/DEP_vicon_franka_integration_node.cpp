#include "ros/ros.h"
#include <ros/callback_queue.h>
#include <iostream>
#include "geometry_msgs/PoseStamped.h"
#include <vector>
#include <cmath>

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

#include "franka_gripper/GraspAction.h"
#include "franka_gripper/GraspActionGoal.h"

class RetrieveStatus{
	private:
		class callbackstatus {
			public: 
				callbackstatus() {
				//	status = 0;
				}
				void sub_callback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg_ptr) {
					goal_status = msg_ptr->status_list[0];
					bool hello = goal_status.status == goal_status.SUCCEEDED;
					//std::cout<< "Status SUCCEEDED?: " << hello <<std::endl;
					hello = goal_status.status == goal_status.PREEMPTED;
					if (hello) {
						std::cout<< "Status PREEMPTED!!" << std::endl;
					}
				}
				actionlib_msgs::GoalStatus goal_status;
		};
	public:
		int finished(){
			ros::NodeHandle SUB;
			callbackstatus substatus;
			ros::Subscriber subscriber = SUB.subscribe("/execute_trajectory/status", 1, &callbackstatus::sub_callback, &substatus);
			ros::spinOnce();
			std::cout<<"Status Returned: " <<substatus.goal_status.text<< std::endl;
			
			if (substatus.goal_status.status==substatus.goal_status.SUCCEEDED) {
				return 1;
			} else if(substatus.goal_status.status==substatus.goal_status.PREEMPTED) {
				return 2;
			} else if(substatus.goal_status.status==substatus.goal_status.ACTIVE) {
				return 0;
			} else {
				return 3;
			}
			
		}

};

class RetrieveData{
	private:
		class callbackdata {
			public:
				void sub_callback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg_ptr) {
					//std::cout<< "x: " << pose_msg_ptr->pose.position.x << std::endl;
					//std::cout<< "y: " << pose_msg_ptr->pose.position.y << std::endl;
					//std::cout<< "z: " << pose_msg_ptr->pose.position.z << std::endl;
					configptr->pose.position.x = pose_msg_ptr->pose.position.x;
					configptr->pose.position.y = pose_msg_ptr->pose.position.y;
					configptr->pose.position.z = pose_msg_ptr->pose.position.z;
					configptr->pose.orientation.x = pose_msg_ptr->pose.orientation.x;
					configptr->pose.orientation.y = pose_msg_ptr->pose.orientation.y;
					configptr->pose.orientation.z = pose_msg_ptr->pose.orientation.z;
					configptr->pose.orientation.w = pose_msg_ptr->pose.orientation.w;
				}
				geometry_msgs::PoseStamped config;
				geometry_msgs::PoseStamped* configptr = &config;
		};

		int Navg;
		bool hasdata;	
		geometry_msgs::PoseStamped avgConfig;
		float sample_set_avg[7];
	public:
		RetrieveData(int Navg_) {
			Navg = Navg_;	
			hasdata = false;
		}

		void retrieve() {
			ros::NodeHandle SUB;
			callbackdata subdata;
			ros::Subscriber subscriber = SUB.subscribe("/vrpn_client_node/smallBox1/pose", 10, &callbackdata::sub_callback, &subdata);

			float sample_set[7] = {0, 0, 0, 0, 0, 0, 0};
			ros::Rate r(30);
			ros::spinOnce();
			int Navg_actual = 0;
			while (Navg_actual<Navg) {
				ros::spinOnce();
				if (subdata.configptr->pose.position.x == 0.0){
					std::cout<<"Bad data"<<std::endl;
					r.sleep();
				} else {
					sample_set[0] += subdata.configptr->pose.position.x;                 
					sample_set[1] += subdata.configptr->pose.position.y;
					sample_set[2] += subdata.configptr->pose.position.z;
					sample_set[3] += subdata.configptr->pose.orientation.x;
					sample_set[4] += subdata.configptr->pose.orientation.y;
					sample_set[5] += subdata.configptr->pose.orientation.z;
					sample_set[6] += subdata.configptr->pose.orientation.w;
					Navg_actual++;
					//std::cout<<"hello brother"<< subdata.configptr->pose.position.x <<std::endl;
				}
				if (!ros::ok()){
					break;
				}
			}
			for (int i=0; i<7; i++) {
				sample_set_avg[i] = sample_set[i]/Navg_actual;
			}
			hasdata = true;
		}

		geometry_msgs::PoseStamped* returnConfigPtr () {
			if (!hasdata) {
				std::cout<< "Call retrieve() before calling returnConfig()" <<std::endl;
			} else {
				avgConfig.pose.position.x = sample_set_avg[0];
				avgConfig.pose.position.y = sample_set_avg[1];
				avgConfig.pose.position.z = sample_set_avg[2];
				avgConfig.pose.orientation.x = sample_set_avg[3];
				avgConfig.pose.orientation.y = sample_set_avg[4];
				avgConfig.pose.orientation.z = sample_set_avg[5];
				avgConfig.pose.orientation.w = sample_set_avg[6];
				return &avgConfig;
			}
		} 

};

void updateObject(std::vector<std::string>& IDs, std::vector<moveit_msgs::CollisionObject>& colObjVec, moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, float x, float y, float z, float qx, float qy, float qz, float qw){

	planning_scene_interface.removeCollisionObjects(IDs);
	colObjVec[0].primitive_poses[0].position.x = x;
	colObjVec[0].primitive_poses[0].position.y = y;
	colObjVec[0].primitive_poses[0].position.z = z;
	colObjVec[0].primitive_poses[0].orientation.x = qx;
	colObjVec[0].primitive_poses[0].orientation.y = qy; 
	colObjVec[0].primitive_poses[0].orientation.z = qz; 
	colObjVec[0].primitive_poses[0].orientation.w = qw; 
	//colObjVec[0].operation = colObjVec[0].ADD;
	planning_scene_interface.applyCollisionObjects(colObjVec);
	//planning_scene_interface.addCollisionObjects(colObjVec);

}


/*
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

void pickup(moveit::planning_interface::MoveGroupInterface& move_group, std::vector<moveit_msgs::CollisionObject> colObjVec, geometry_msgs::PoseStamped* obj_pose, std::string obj_id, float width) {
        std::vector<moveit_msgs::Grasp> grasps;
        grasps.resize(1);
        grasps[0].grasp_pose.header.frame_id = "panda_link0";
        tf2::Quaternion orient;
        //orient.setRPY(-pi/2, -pi/4, -pi/2);
        //orient.setRPY(objS.roll, objS.pitch, objS.yaw);
        grasps[0].grasp_pose.pose.orientation = obj_pose->pose.orientation//tf2::toMsg(orient);
        grasps[0].grasp_pose.pose.position.x = -obj_pose->pose.position.y 
        grasps[0].grasp_pose.pose.position.y = obj_pose->pose.position.x
        grasps[0].grasp_pose.pose.position.z = obj_pose->pose.position.z + .09;
        // This defines the pre_grasp_posture
        grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
        grasps[0].pre_grasp_approach.direction.vector.x = 0; 
        grasps[0].pre_grasp_approach.direction.vector.y = 0; 
        grasps[0].pre_grasp_approach.direction.vector.z = -1;
        grasps[0].pre_grasp_approach.min_distance = .05;//.095; //values copied from tutorial
        grasps[0].pre_grasp_approach.desired_distance = .08; //.115;  

        grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0";
        grasps[0].post_grasp_retreat.direction.vector.x = 0;
        grasps[0].post_grasp_retreat.direction.vector.y = 0;
        grasps[0].post_grasp_retreat.direction.vector.z = 1;
        grasps[0].post_grasp_retreat.min_distance = .05;//.1;
        grasps[0].post_grasp_retreat.desired_distance = .08;//.25;

        openGrip(grasps[0].pre_grasp_posture); //Open grip in the approach
        closeGrip(grasps[0].grasp_posture, width);

        //move_group.setSupportSurfaceName(objS.supportsurface);
        move_group.pick(obj_id, grasps);
}
void place(moveit::planning_interface::MoveGroupInterface& move_group, geometry_msgs::Pose& place_pose, std::string obj_id) {
        std::vector<moveit_msgs::PlaceLocation> placeloc;
        placeloc.resize(1);
        placeloc[0].place_pose.header.frame_id = "panda_link0";
        //tf2::Quaternion orient;
        //orient.setRPY(0, 0, pi);
        //orient.setRPY(objS.roll, objS.pitch, objS.yaw);
        placeloc[0].place_pose.pose.orientation = place_pose.orientation//tf2::toMsg(orient);
        placeloc[0].place_pose.pose.position = place_pose.position
        //placeloc[0].place_pose.pose.position.y = objS.posy;
        //placeloc[0].place_pose.pose.position.z = objS.posz;

        placeloc[0].pre_place_approach.direction.header.frame_id = "panda_link0";
        placeloc[0].pre_place_approach.direction.vector.x = 0; 
        placeloc[0].pre_place_approach.direction.vector.y = 0;
        placeloc[0].pre_place_approach.direction.vector.z = -1;
        placeloc[0].pre_place_approach.min_distance = .05;//.095; //values copied from tutorial
        placeloc[0].pre_place_approach.desired_distance = .08;//.115;  

        placeloc[0].post_place_retreat.direction.header.frame_id = "panda_link0";
        placeloc[0].post_place_retreat.direction.vector.x = 0;
        placeloc[0].post_place_retreat.direction.vector.y = 0;
        placeloc[0].post_place_retreat.direction.vector.z = 1;
        placeloc[0].post_place_retreat.min_distance = .05;//.1;
        placeloc[0].post_place_retreat.desired_distance = .08;//.25;

        openGrip(placeloc[0].post_place_posture); //Open grip in the approach

        move_group.place(obj_id, placeloc);
}
*/



int main(int argc, char **argv) {
	ros::init(argc, argv, "vicon_franka_integration_node");
	ros::NodeHandle node_handle("~");
	ros::AsyncSpinner spinner(1);
	spinner.start();


	/////////////////////////////////////////////////////////////////////////
	static const std::string PLANNING_GROUP = "panda_arm";

	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	move_group.setPlanningTime(40);
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

	robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));
	const robot_state::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);

	planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
	planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "ready");
	moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
	visual_tools.deleteAllMarkers();
	visual_tools.trigger();


	/////////////////////////////////////////////////////////////////////////
	//Gripper grip("172.25.20.101");
	//grip_client
	actionlib::SimpleActionClient<franka_gripper::GraspAction> grip_client("/franka_gripper/grasp", true);
	franka_gripper::GraspActionGoal grip_goal;


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
	/////////////////////////////////////////////////////////////////////////


	int Nobj=2;
	float epsilon_position = .03;
	float epsilon_orientation = .1;
	std::vector<moveit_msgs::CollisionObject> colObjVec;
	std::vector<std::string> colObjIDs;
	colObjVec.resize(Nobj);

	for (int i=0; i<Nobj; i++) {
		colObjVec[i].header.frame_id = move_group.getPlanningFrame();
	}
	

	// smallBox1 (specialized tube box)
	float box_h = .154;//.03;
	colObjVec[0].id = "smallBox1";
	colObjIDs.push_back(colObjVec[0].id);
	colObjVec[0].primitives.resize(1);
	colObjVec[0].primitives[0].type = colObjVec[0].primitives[0].BOX;
	colObjVec[0].primitives[0].dimensions.resize(3);
	colObjVec[0].primitives[0].dimensions[0] = .045;//.10;
	colObjVec[0].primitives[0].dimensions[1] = .045;//.07;
	colObjVec[0].primitives[0].dimensions[2] = box_h;

	// smallBox1 (table)
	float x_bound, y_bound;
	x_bound = .1;
	y_bound = .1;
	colObjVec[1].id = "table1";
	colObjIDs.push_back(colObjVec[1].id);
	colObjVec[1].primitives.resize(4);
	colObjVec[1].primitives[0].type = colObjVec[1].primitives[0].BOX;
	colObjVec[1].primitives[0].dimensions.resize(3);
	colObjVec[1].primitives[0].dimensions[0] = 1;
	colObjVec[1].primitives[0].dimensions[1] = 2;
	colObjVec[1].primitives[0].dimensions[2] = .05;
	colObjVec[1].primitive_poses.resize(4);
	colObjVec[1].primitive_poses[0].orientation.w = 1;
	colObjVec[1].primitive_poses[0].position.x = 0.50 + x_bound;
	colObjVec[1].primitive_poses[0].position.y = 0;
	colObjVec[1].primitive_poses[0].position.z = -.05/2;
	colObjVec[1].primitives[1].type = colObjVec[1].primitives[0].BOX;
	colObjVec[1].primitives[1].dimensions.resize(3);
	colObjVec[1].primitives[1].dimensions[0] = 1;
	colObjVec[1].primitives[1].dimensions[1] = 2;
	colObjVec[1].primitives[1].dimensions[2] = .05;
	colObjVec[1].primitive_poses[1].orientation.w = 1;
	colObjVec[1].primitive_poses[1].position.x = -(.50 + x_bound);
	colObjVec[1].primitive_poses[1].position.y = 0;
	colObjVec[1].primitive_poses[1].position.z = -.05/2;
	colObjVec[1].primitives[2].type = colObjVec[1].primitives[0].BOX;
	colObjVec[1].primitives[2].dimensions.resize(3);
	colObjVec[1].primitives[2].dimensions[0] = 2 * x_bound;
	colObjVec[1].primitives[2].dimensions[1] = 1 - y_bound;
	colObjVec[1].primitives[2].dimensions[2] = .05;
	colObjVec[1].primitive_poses[2].orientation.w = 1;
	colObjVec[1].primitive_poses[2].position.x = 0;
	colObjVec[1].primitive_poses[2].position.y = (1 - y_bound)/2 + y_bound;
	colObjVec[1].primitive_poses[2].position.z = -.05/2;
	colObjVec[1].primitives[3].type = colObjVec[1].primitives[0].BOX;
	colObjVec[1].primitives[3].dimensions.resize(3);
	colObjVec[1].primitives[3].dimensions[0] = 2 * x_bound;
	colObjVec[1].primitives[3].dimensions[1] = 1 - y_bound;
	colObjVec[1].primitives[3].dimensions[2] = .05;
	colObjVec[1].primitive_poses[3].orientation.w = 1;
	colObjVec[1].primitive_poses[3].position.x = 0;
	colObjVec[1].primitive_poses[3].position.y =  -((1 - y_bound)/2 + y_bound);
	colObjVec[1].primitive_poses[3].position.z = -.05/2 - .00;

	colObjVec[1].operation = colObjVec[1].ADD;


	RetrieveData configData(10);
	RetrieveStatus executionStatus;
	float readx, ready, readz, readx_mon, ready_mon, readz_mon;
	//while (ros::ok()){
		configData.retrieve();
		auto p = configData.returnConfigPtr(); //geometry_msgs::PoseStamped*


		// Coordinate frame transformation
		/*
		readx = -p->pose.position.y + .776;
		ready = p->pose.position.x + .09;
		readz = p->pose.position.z - .05;
		*/
		readx = p->pose.position.x - .021544;
		ready = p->pose.position.y + .19416;
		readz = p->pose.position.z + .033176 - .060;
		geometry_msgs::PoseStamped ret_pose;
		ret_pose = move_group.getCurrentPose("panda_link8");
		std::cout<<"CURRENT LINK 8 POSE X = "<<ret_pose.pose.position.x <<std::endl;
		std::cout<<"CURRENT LINK 8 POSE Y = "<<ret_pose.pose.position.y <<std::endl;
		std::cout<<"CURRENT LINK 8 POSE Z = "<<ret_pose.pose.position.z <<std::endl;
		tf2::Quaternion q_r, q_rot, q_f;
		{
			tf2::Quaternion q_orig, q_in, q_90;

			tf2::convert(p->pose.orientation, q_in);
			//q_90.setRPY(0, 0, 0);
			/*
			   q_f = q_in * q_orig * q_in.inverse(); 
			   q_r = q_in * q_90;
			   */
			q_r = q_in; //q_90 * q_in;
			q_r.normalize();
		}

		colObjVec[0].primitive_poses.resize(1);
		//updateObject(colObjIDs, colObjVec, planning_scene_interface, readx, ready, readz, p->pose.orientation.x, p->pose.orientation.y, p->pose.orientation.z, p->pose.orientation.w);
		updateObject(colObjIDs, colObjVec, planning_scene_interface, readx, ready, readz, q_r[0], q_r[1], q_r[2], q_r[3]);
		std::cout<< "x: " << readx << std::endl;	
		std::cout<< "y: " << ready << std::endl;	
		std::cout<< "z: " << readz << std::endl;
		move_group.setEndEffectorLink("panda_link8");
		geometry_msgs::Pose pose;

		/*
		initpose.position.x = readx; 
		initpose.position.y = ready; 
		initpose.position.z = readz;
		tf2::Quaternion q_init(-p->pose.orientation.x, -p->pose.orientation.y, -p->pose.orientation.z, -p->pose.orientation.w);
		tf2::Quaternion q_rot, q_new;
		//tf2::convert(p->pose.orientation, q_init);
		q_rot.setEuler(0*M_PI, 1*M_PI, -1*M_PI/4);
		q_new = q_init*q_rot;
		q_new.normalize();
		initpose.orientation = tf2::toMsg(q_new);
		*/
		{
			tf2::Quaternion q_orig, q_in, q_180, q_hand_yaw;
			q_orig[0] = 0;
			q_orig[1] = 0;
			q_orig[2] = box_h/2 + .1;
			q_orig[3] = 0;
			q_hand_yaw.setRPY(0, 0, 0);
			q_orig = q_hand_yaw * q_orig;

			q_in = q_r;
			q_180.setRPY(0, M_PI, -M_PI/4);
			q_f = q_in * q_orig * q_in.inverse();
			q_rot = q_in * q_180;
		}

		pose.position.x = readx + q_f[0];
		pose.position.y = ready + q_f[1];
		pose.position.z = readz + q_f[2];
		q_rot.normalize();
		pose.orientation.x = q_rot[0];
		pose.orientation.y = q_rot[1];
		pose.orientation.z = q_rot[2];
		pose.orientation.w = q_rot[3];
		//tf2::Quaternion initorient;
		//initorient.setEuler(0, M_PI, 3*M_PI/4);
		//initpose.orientation = tf2::toMsg(initorient); 
		move_group.setStartStateToCurrentState();
		move_group.setPoseTarget(pose);
		move_group.setPlanningTime(8.0);
		moveit::planning_interface::MoveGroupInterface::Plan initplan;
		move_group.plan(initplan);
		move_group.execute(initplan);

		std::cout<<"b4 wait for server"<<std::endl;
		grip_client.waitForServer();
		std::cout<<"after wait for server"<<std::endl;
		grip_goal.goal.width = .045;
		grip_goal.goal.speed = .1;
		grip_goal.goal.force = 5;
		grip_goal.goal.epsilon.inner = .03;
		grip_goal.goal.epsilon.outer = .03;
		grip_client.sendGoal(grip_goal.goal);
		grip_client.waitForResult(ros::Duration(5.0));

		pose.position.x = readx + q_f[0];
		pose.position.y = ready + q_f[1];
		pose.position.z = readz + q_f[2] + .3;
		q_rot.normalize();
		pose.orientation.x = q_rot[0];
		pose.orientation.y = q_rot[1];
		pose.orientation.z = q_rot[2];
		pose.orientation.w = q_rot[3];

		move_group.setStartStateToCurrentState();
		move_group.setPoseTarget(pose);
		move_group.setPlanningTime(5.0);
		moveit::planning_interface::MoveGroupInterface::Plan upplan;
		move_group.plan(upplan);
		move_group.execute(upplan);

		grip_goal.goal.width = .10;
		grip_goal.goal.speed = .1;
		grip_goal.goal.force = 5;
		grip_goal.goal.epsilon.inner = .03;
		grip_goal.goal.epsilon.outer = .03;
		grip_client.sendGoal(grip_goal.goal);
		grip_client.waitForResult(ros::Duration(5.0));
		//grip.grasp(.07, .1, 20);
		//while (!(moveit::planning_interface::MoveItErrorCode::SUCCESS == move_group.asyncExecute(initplan))){
		//bool success = move_group.asyncExecute(initplan) == true;
		//move_group.asyncExecute(initplan);
		/*
		int stopped = 0;
		while ((stopped==0)&&ros::ok()) {
			ros::WallDuration(.5).sleep();
			configData.retrieve();
			auto p_monitor = configData.returnConfigPtr();
			readx_mon = -p_monitor->pose.position.y + .794;
			ready_mon = p_monitor->pose.position.x + .076;
			readz_mon = p_monitor->pose.position.z;
			std::cout<< "x_mon: " << readx_mon << std::endl;	
			std::cout<< "y_mon: " << ready_mon << std::endl;	
			std::cout<< "z_mon: " << readz_mon << std::endl;	
			float err_position = std::pow((readx_mon-readx),2) + std::pow((ready_mon-ready),2) + std::pow((readz_mon-readz),2);
			//err_orientation = std::pow((p->pose.orientation.x-p_monitor->pose.orientation.x),2) + pow((p->pose.orientation.y-p_monitor->pose.orientation.y),2) + pow((p->pose.orientation.z-p_monitor->pose.orientation.z),2) + pow((p->pose.orientation.w-p_monitor->pose.orientation.w));
			//if ((err_position>epsilon_position)||(err_orientation>epsilon_orientation)) {
			if (err_position>epsilon_position) {
				std::cout<<"   OBJECT MOVEMENT DETECTED"<<std::endl;	
				std::cout<<"   REPLANNING"<<std::endl;	
				move_group.stop();
			}
			stopped = executionStatus.finished();
		}
		*/
	//}
	return 0;
	}
