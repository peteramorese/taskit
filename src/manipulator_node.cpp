#include "ros/ros.h"
#include <ros/callback_queue.h>
#include <iostream>
#include "geometry_msgs/PoseStamped.h"
#include "manipulation_interface/PlanningQuery.h"
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
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include "franka_gripper/GraspAction.h"
#include "franka_gripper/GraspActionGoal.h"

static const std::string NONE = "none";
static const std::string PLANNING_GROUP = "panda_arm";

class PlanningQuerySrv {
	public:
		// DEFINE CONSTANTS FOR PROBLEM
		const double bag_l = .045;//.045;
		const double bag_w = .07;
		const double bag_h = .153;
		const double eef_offset = .08;
		const double grip_width_closed = .044;
		const double grip_width_open = .1;
		const double grip_speed = .1;
		const double grip_force = 50;
		const double grip_epsilon_inner = .03;
		const double grip_epsilon_outer = .03;
		const double approach_dist = 0.04;//.3;
		const double jump_thresh = 0.0;
		const double eef_step = 0.1;
		const double place_safety_dist = .01; // How far off the ground obj is placed
		const int num_waypts = 3;
		const double max_acceleration_scale = 0.1;
	private:
		const int N_TRIALS;
		moveit::planning_interface::MoveGroupInterface* move_group_ptr;
		moveit::planning_interface::PlanningSceneInterface* planning_scene_interface_ptr;
		trajectory_processing::IterativeParabolicTimeParameterization IPTP;
		actionlib::SimpleActionClient<franka_gripper::GraspAction>* grp_act_ptr;
		std::map<std::string, moveit_msgs::CollisionObject> collision_objects;
		std::map<std::string, std::string> collision_object_domains;
		std::vector<std::string> obs_domain_labels;
		franka_gripper::GraspActionGoal grip_goal;	
		bool use_grasp, safe_place, use_gripper, workspace_set;
		std::string attached_obj;
		std::string current_grasp_mode;
		geometry_msgs::Pose prev_pose;
		geometry_msgs::Point displacement_above;

		geometry_msgs::Point getTranslatedPoint(const geometry_msgs::Pose& eef_pose, bool approach, double multiplier = 1.0, const geometry_msgs::Point* displacement = nullptr) {
			geometry_msgs::Point ret_p = eef_pose.position;
			if (displacement) {
				if (approach) {
					ret_p.x -= multiplier*displacement->x;
					ret_p.y -= multiplier*displacement->y;
					ret_p.z -= multiplier*displacement->z;
				} else {
					ret_p.x += multiplier*displacement->x;
					ret_p.y += multiplier*displacement->y;
					ret_p.z += multiplier*displacement->z;
				}
			} else {
				tf2::Quaternion q_orig, q_in, q_f, q_set;
				tf2::convert(eef_pose.orientation, q_in);

				q_orig[0] = 0;
				q_orig[1] = 0;
				q_orig[2] = -(bag_h/2 + eef_offset + approach_dist);
				q_orig[3] = 0;
				// Rotate the q
				q_f = q_in * q_orig * q_in.inverse(); 
				//q_f = q_f * q_set;
				if (approach) {
					ret_p.x -= multiplier*q_f[0];
					ret_p.y -= multiplier*q_f[1];
					ret_p.z -= multiplier*q_f[2];
				} else {
					ret_p.x += multiplier*q_f[0];
					ret_p.y += multiplier*q_f[1];
					ret_p.z += multiplier*q_f[2];
				}
			}
			return ret_p;
		}
		void getWayPoints(const geometry_msgs::Pose& eef_pose, std::vector<geometry_msgs::Pose>& ret_waypts, bool approach, const geometry_msgs::Point* displacement = nullptr) {
			ret_waypts.resize(num_waypts);
			for (int i=0; i<num_waypts; ++i) {
				double multiplier = static_cast<double>(i)/static_cast<double>(num_waypts-1);
				ret_waypts[i].position = getTranslatedPoint(eef_pose, approach, multiplier, displacement);
				ret_waypts[i].orientation = eef_pose.orientation;
			}
		}
		void runGrasp(bool approach, const geometry_msgs::Point* displacement = nullptr) {
			std::vector<geometry_msgs::Pose> waypts;
			moveit_msgs::RobotTrajectory trajectory;
			moveit_msgs::RobotTrajectory r_trajectory_msg;
			robot_trajectory::RobotTrajectory r_trajectory(move_group_ptr->getRobotModel(), PLANNING_GROUP);
			
			getWayPoints(move_group_ptr->getCurrentPose().pose, waypts, approach, displacement);
			double fraction = move_group_ptr->computeCartesianPath(waypts, eef_step, jump_thresh, trajectory);

			r_trajectory.setRobotTrajectoryMsg(*(move_group_ptr->getCurrentState()), trajectory);
			IPTP.computeTimeStamps(r_trajectory, max_acceleration_scale, max_acceleration_scale); // max_acceleration_scale
			r_trajectory.getRobotTrajectoryMsg(r_trajectory_msg);
			move_group_ptr->setMaxVelocityScalingFactor(1);
			move_group_ptr->execute(r_trajectory_msg);
		}
		void runGrip(bool grab) {
			grip_goal.goal.width = (grab) ? grip_width_closed : grip_width_open;
			grip_goal.goal.speed = grip_speed;
			grip_goal.goal.force = grip_force;
			grip_goal.goal.epsilon.inner = grip_epsilon_inner;
			grip_goal.goal.epsilon.outer = grip_epsilon_outer;
			grp_act_ptr->sendGoal(grip_goal.goal);
			grp_act_ptr->waitForResult(ros::Duration(5.0));
		}
	public:
		PlanningQuerySrv(moveit::planning_interface::MoveGroupInterface* move_group_ptr_, moveit::planning_interface::PlanningSceneInterface* psi_ptr_, actionlib::SimpleActionClient<franka_gripper::GraspAction>* grp_act_ptr_,  int N_TRIALS_, bool use_gripper_, bool safe_place_, bool use_grasp_ = true) : 
			move_group_ptr(move_group_ptr_),
			planning_scene_interface_ptr(psi_ptr_),
			grp_act_ptr(grp_act_ptr_),
			N_TRIALS(N_TRIALS_), 
			use_gripper(use_gripper_),
			//use_grasp(use_grasp_),
			use_grasp(true),
			safe_place(safe_place_),
			workspace_set(false) {
				
			// Setup:
			attached_obj = NONE;
			current_grasp_mode = NONE;
			displacement_above.x = 0.0;	
			displacement_above.y = 0.0;	
			displacement_above.z = approach_dist;	
			if (use_gripper) {
				ROS_INFO_NAMED("manipulator_node","Waiting for franka_gripper action to show...");
				grp_act_ptr->waitForServer();
				ROS_INFO_NAMED("manipulator_node","Found franka_gripper action.");
			}
		}


		void printPose(const geometry_msgs::Pose& p) {
			std::cout<<"Printing pose: \n";
			std::cout<<"	position:\n";
			std::cout<<"		x:"<<p.position.x<<"\n";
			std::cout<<"		y:"<<p.position.y<<"\n";
			std::cout<<"		z:"<<p.position.z<<"\n";
			std::cout<<"	orientation:\n";
			std::cout<<"		wx:"<<p.orientation.x<<"\n";
			std::cout<<"		wx:"<<p.orientation.y<<"\n";
			std::cout<<"		wx:"<<p.orientation.z<<"\n";
			std::cout<<"		w:"<<p.orientation.w<<"\n";
		}

		bool getGraspTypePoses(const std::string& grasp_type, const geometry_msgs::Pose& manipulator_pose, std::vector<geometry_msgs::Point>& positions, std::vector<tf2::Quaternion>& orientations) {
			// Compute the quaternions for each grasp type:
			tf2::Quaternion q_in, q_set;
			// This quaternion sets the panda gripper to face down towards the 
			// object grabbing along its length
			//q_set.setRPY(0, M_PI, -M_PI/4 + M_PI/2);
			//q_set.setRPY(0, 0, -M_PI/2);
			q_set.setRPY(0, 0, -M_PI/4);
			//tf2::convert(move_group_ptr->getCurrentPose().pose.orientation, q_in);
			//q_in.setRPY(0, M_PI, M_PI/4 + M_PI/2);
			tf2::convert(manipulator_pose.orientation, q_in);

			if (grasp_type == "up") {
				q_set.setRPY(0, M_PI, M_PI/4);
				tf2::Quaternion q_orig;
				std::vector<tf2::Quaternion> q_f; 
				std::vector<tf2::Quaternion> q_rot;
				int N_grasps = 2;
				q_f.resize(N_grasps);
				q_rot.resize(N_grasps);
				std::vector<geometry_msgs::Point> pts(N_grasps);

				q_orig[0] = 0;
				q_orig[1] = 0;
				q_orig[2] = (bag_h/2 + eef_offset);
				q_orig[3] = 0;
				// Rotate the q
				q_f[0] = q_in * q_orig * q_in.inverse(); 
				q_rot[0] = q_in * q_set;
				//orientations = q_rot;
				pts[0].x = q_f[0][0];
				pts[0].y = q_f[0][1];
				pts[0].z = q_f[0][2];
				


				q_set.setRPY(0, 0, -M_PI/4);
				q_orig[2] = -q_orig[2];
				// Rotate the q
				q_f[1] = q_in * q_orig * q_in.inverse(); 
				q_rot[1] = q_in * q_set;
				orientations = q_rot;
				pts[1].x = q_f[1][0];
				pts[1].y = q_f[1][1];
				pts[1].z = q_f[1][2];
				positions = pts;
				return true;

			} else if (grasp_type == "side") {
				tf2::Quaternion q_orig;
				std::vector<tf2::Quaternion> q_f; 
				std::vector<tf2::Quaternion> q_rot;
				int N_grasps = 2;
				q_f.resize(N_grasps);
				q_rot.resize(N_grasps);
				std::vector<geometry_msgs::Point> pts(N_grasps);

				//q_orig[0] = 0;
				//q_orig[1] = bag_w/2 + eef_offset;
				//q_orig[2] = 0;
				//q_orig[3] = 0;
				q_orig[0] = 0;
				q_orig[1] = (bag_w/2 + eef_offset);
				q_orig[2] = 0;
				q_orig[3] = 0;
				// 90 degrees
				tf2::Quaternion q_set_2;
				{
					tf2::Quaternion q_set_temp;
					q_set_temp.setRPY(M_PI/2, 0, 0);
					q_set_2 = q_set_temp * q_set;
				}
				q_f[0] = q_in * q_orig * q_in.inverse(); 
				q_rot[0] = q_in * q_set_2;
				pts[0].x = q_f[0][0];
				pts[0].y = q_f[0][1];
				pts[0].z = q_f[0][2];


				// -90 degrees
				{
					tf2::Quaternion q_set_temp;
					q_set_temp.setRPY(-M_PI/2, 0, 0);
					q_set_2 = q_set_temp * q_set;
				}
				q_orig[1] = -q_orig[1]; //flip the axis
				q_f[1] = q_in * q_orig * q_in.inverse(); 
				q_rot[1] = q_in * q_set_2;
				orientations = q_rot;
				pts[1].x = q_f[1][0];
				pts[1].y = q_f[1][1];
				pts[1].z = q_f[1][2];
				positions = pts;
				return true;
			} else {
				return false;
			}

		}
		void setWorkspace(std::vector<moveit_msgs::CollisionObject> col_obj_vec_ws, std::vector<std::string> col_obj_vec_dom_lbls) {
			collision_objects.clear();		
			collision_object_domains.clear();
			for (int i=0; i<col_obj_vec_ws.size(); ++i) {
				collision_objects[col_obj_vec_ws[i].id] = col_obj_vec_ws[i];
				collision_object_domains[col_obj_vec_ws[i].id] = col_obj_vec_dom_lbls[i];
			}
			workspace_set = true;
		}
		void setupEnvironment(std::string planning_domain_lbl) {
			std::cout<<"recieved planning domain label in setupEnvironment: "<<planning_domain_lbl<<std::endl;
			std::cout<<"obs_domain_labels size: "<<obs_domain_labels.size()<<std::endl;
			std::vector<moveit_msgs::CollisionObject> temp_col_vec;
			for (auto kv : collision_object_domains) {
				if (kv.first == "none") {
					ROS_INFO_NAMED("manipulator_node", "Environment Setup: Ignoring object with label: %s", kv.first.c_str());
				} else if (kv.second == planning_domain_lbl) {
					std::cout<<"adding:"<< kv.first<<std::endl;
					collision_objects.at(kv.first).operation = collision_objects.at(kv.first).ADD;
					temp_col_vec.push_back(collision_objects.at(kv.first));
				} else {
					std::cout<<"removing:"<< kv.first<<std::endl;
					collision_objects.at(kv.first).operation = collision_objects.at(kv.first).REMOVE;
					temp_col_vec.push_back(collision_objects.at(kv.first));
				}
			}
			planning_scene_interface_ptr->addCollisionObjects(temp_col_vec);
		}
		void findObjAndUpdate(std::string obj_id, std::string domain_label) {
			collision_object_domains.at(obj_id) = domain_label;
			//bool not_found = true;
			//for (int i=0; i<col_obj_vec.size(); ++i) {
			//	if (col_obj_vec[i].id == obj_id) {
			//		std::cout<<"FIND UPDATE: found object id: "<<obj_id<<std::endl;
			//		std::cout<<"FIND UPDATE: updating to domain label: "<<domain_label_<<std::endl;
			//		obs_domain_labels[i] = domain_label_;	
			//		not_found = false;
			//		break;
			//	}	
			//}
			//if (not_found) {
			//	ROS_ERROR_NAMED("manipulator_node","Object id was not found. Cannot update domain label");
			//}
		}
		bool planQuery_serviceCB(manipulation_interface::PlanningQuery::Request &request, manipulation_interface::PlanningQuery::Response &response) {
			std::cout<<"HELLO PLANNING SERVICE CB"<<std::endl;
			std::cout<<"Recieved planning domain: "<<request.planning_domain<<std::endl;
			std::cout<<"\n";
			ROS_INFO_NAMED("manipulator_node", "Recieved Planning Query");
			if (!workspace_set) {
				ROS_ERROR("Need to set workspace before calling planning query!");
				return false;
			}


			if (request.setup_environment) {
				//col_obj_vec.resize(request.bag_poses.size());
				obs_domain_labels = request.bag_domain_labels;

				for (int i=0; i<request.bag_poses.size(); ++i) {
					auto col_obj_itr = collision_objects.find(request.bag_labels[i]);
					if (col_obj_itr == collision_objects.end()) {
						moveit_msgs::CollisionObject temp_col_obj;
						temp_col_obj.header.frame_id = "panda_link0";
						temp_col_obj.id = request.bag_labels[i];
						temp_col_obj.primitives.resize(1);
						temp_col_obj.primitives[0].type = temp_col_obj.primitives[0].BOX;
						temp_col_obj.primitives[0].dimensions.resize(3);
						temp_col_obj.primitives[0].dimensions[0] = bag_l;
						temp_col_obj.primitives[0].dimensions[1] = bag_w;
						temp_col_obj.primitives[0].dimensions[2] = bag_h;
						temp_col_obj.primitive_poses.resize(1);
						collision_objects[request.bag_labels[i]] = temp_col_obj;
						col_obj_itr = collision_objects.find(request.bag_labels[i]);
					}
					//std::cout<<" i see : "<<request.bag_poses[i].position.x<<std::endl;
					//std::cout<<" i see : "<<request.bag_poses[i].position.y<<std::endl;
					//std::cout<<" i see : "<<request.bag_poses[i].position.z<<std::endl;
					col_obj_itr->second.primitive_poses[0].position.x = request.bag_poses[i].position.x;
					col_obj_itr->second.primitive_poses[0].position.y = request.bag_poses[i].position.y;
					col_obj_itr->second.primitive_poses[0].position.z = request.bag_poses[i].position.z;
					col_obj_itr->second.primitive_poses[0].orientation.x = request.bag_poses[i].orientation.x;
					col_obj_itr->second.primitive_poses[0].orientation.y = request.bag_poses[i].orientation.y;
					col_obj_itr->second.primitive_poses[0].orientation.z = request.bag_poses[i].orientation.z;
					col_obj_itr->second.primitive_poses[0].orientation.w = request.bag_poses[i].orientation.w;
					collision_object_domains[request.bag_labels[i]] = request.bag_domain_labels[i];
					//col_obj_vec.push_back(temp_col_obj);
					//obs_domain_labels.push_back(request.bag_domain_labels[i]);
					//std::cout<<"Adding object: "<<temp_col_obj.id<<" to domain: "<<request.bag_domain_labels[i]<<std::endl;
					//col_obj_vec[i].operation = col_obj_vec[i].ADD;
				}
				setupEnvironment(request.planning_domain);
			}
			if (request.pickup_object != "none") {
				
				if (use_grasp) {
					ROS_INFO_NAMED("manipulator_node","Working on grasp approach...");
					runGrasp(true); // <- uses eef pose displacement
				}

				std::string obj_label = request.pickup_object;
				move_group_ptr->attachObject(obj_label,"panda_link8");
				
				// Close gripper if gripper is being used:
				if (use_gripper) runGrip(true);

				// Change the domain of the attached object to be 'none' 
				// so that it does not get removed
				findObjAndUpdate(obj_label, "none");
				attached_obj = obj_label;
				ROS_INFO_NAMED("manipulator_node","Grabbing object...");

				if (use_grasp) {
					ROS_INFO_NAMED("manipulator_node","Working on grasp retreat...");
					runGrasp(false, &displacement_above); // <- displaces up
				}
				ROS_INFO_NAMED("manipulator_node","Done grabbing object");
				response.success = true;
				response.held_obj = request.pickup_object;
			} else if (request.drop_object != "none") {

				if (use_grasp) {
					ROS_INFO_NAMED("manipulator_node","Working on release approach...");
					runGrasp(true, &displacement_above); // <- displaces down since approach is true
				}

				std::string obj_label = request.drop_object;
				move_group_ptr->detachObject(obj_label);

				collision_objects.at(request.drop_object).primitive_poses[0] = request.manipulator_pose;
				//bool found = false;
				//for (int i = 0; i<request.bag_labels.size(); ++i) {
				//	if (request.bag_labels[i] == request.drop_object) {
				//		found = true;
				//		break;
				//	}
				//}
				//if (!found) {
				//	ROS_ERROR("Did not find object to drop!");
				//	return false;
				//}
				// If we are releasing an object, the new domain becomes
				// whatever domain the end effector is in (the request)
				findObjAndUpdate(obj_label, request.planning_domain);
				attached_obj = "none";
				ROS_INFO_NAMED("manipulator_node","Dropping object...");
				if (use_gripper) runGrip(false);

				if (use_grasp) {
					ROS_INFO_NAMED("manipulator_node","Working on release retreat...");
					runGrasp(false); // <- uses eef pose displacement
				}

				response.success = true;
				ROS_INFO_NAMED("manipulator_node","Done dropping object");
			} else {
				//setupEnvironment(request.planning_domain);
				std::cout<<"MOVING"<<std::endl;
				std::vector<geometry_msgs::Pose> poses;
				moveit::planning_interface::MoveGroupInterface::Plan plan;


				// Convert from PoseStamed to Pose
				if (request.safe_config) {
					std::vector<double> joint_val_target = {0.0, -.785398, 0.0, -2.35619, 1.5708, .785398};
					move_group_ptr->setJointValueTarget(joint_val_target);
				} else {
					std::string grasp_type = request.grasp_type;
					if (grasp_type == "current") {
						std::cout<<"Using current grasp mode."<<std::endl;
						grasp_type = current_grasp_mode;
					} else {
						// If the user does not use the current mode grasp, then
						// keep track of the specified grasp type to set the mode
						current_grasp_mode = grasp_type;
					}

					std::vector<geometry_msgs::Point> positions; 
					std::vector<tf2::Quaternion> orientations;
					if (!getGraspTypePoses(grasp_type, request.manipulator_pose, positions, orientations)) {
						ROS_ERROR_NAMED("manipulator_node","Unrecognized grasp type");
						std::cout<<"Grasp type: "<<grasp_type<<std::endl;
						return false;
					}

					poses.resize(positions.size());
					for (int ii=0; ii<positions.size(); ++ii) {
						const geometry_msgs::Point& disp = positions[ii];
						tf2::Quaternion& q_or = orientations[ii];

						poses[ii].position.x = request.manipulator_pose.position.x + disp.x;
						poses[ii].position.y = request.manipulator_pose.position.y + disp.y;
						poses[ii].position.z = request.manipulator_pose.position.z + disp.z;
						q_or.normalize();
						poses[ii].orientation.x = q_or[0];
						poses[ii].orientation.y = q_or[1];  
						poses[ii].orientation.z = q_or[2];
						poses[ii].orientation.w = q_or[3];
					}
				}
				move_group_ptr->setPlanningTime(5.0);

				bool success = false;
				bool success_ex = false;
				for (int ii=0; ii<N_TRIALS; ii++){
					moveit::planning_interface::MoveGroupInterface::Plan direct_plan;

					move_group_ptr->setStartStateToCurrentState();
					int pose_i = 0;
					for (auto& goal_pose : poses) {
						pose_i++;
						ROS_INFO_STREAM(" --- Working on transit --- "<<pose_i<<" out of "<<poses.size());
						printPose(goal_pose);
						move_group_ptr->setPoseTarget(goal_pose);
						ros::WallDuration(1.0).sleep();
						success = (move_group_ptr->plan(direct_plan)==moveit::planning_interface::MoveItErrorCode::SUCCESS);
						if (success){
							ROS_INFO("Plan test succeed!");
							if (use_grasp) {
								geometry_msgs::Pose displaced_pose = goal_pose;
								const auto att_objs = planning_scene_interface_ptr->getAttachedObjects();
								if (att_objs.size() == 0) {
									// Transit to pose oriented displacement:
									displaced_pose.position = getTranslatedPoint(goal_pose, false);
									response.is_obj_displaced = true;
									response.held_obj = "none";
								} else {
									// Transport to position displaced up:
									displaced_pose.position = getTranslatedPoint(goal_pose, false, 1.0, &displacement_above);
									response.is_obj_displaced = true; // obj is displaced (being held)
									response.displacement = displacement_above;
									response.held_obj = att_objs.begin()->first; // there should only be one attached obj
									if (safe_place) {
										displaced_pose.position.z += place_safety_dist;
									}
								}
								move_group_ptr->setPoseTarget(displaced_pose);
							} else {
								move_group_ptr->setPoseTarget(goal_pose);
							}
							success_ex = (move_group_ptr->plan(direct_plan)==moveit::planning_interface::MoveItErrorCode::SUCCESS);
							ROS_INFO_NAMED("manipulator_node","Completed planning on iteration: %d",ii);
							if (success_ex) {
								move_group_ptr->execute(direct_plan);
								break;
							}
						}
					}
					if (success_ex) {
						break;
					}
					ros::WallDuration(1.0).sleep();
				}
				response.success = success_ex;
			}
			return true;
		}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "manipulator_node");
	ros::NodeHandle M_NH("~");

	ros::AsyncSpinner spinner(2);
	spinner.start();

	bool sim_only, mock_observer;
	M_NH.getParam("sim_only", sim_only);
	M_NH.getParam("mock_observer", mock_observer);



	/////////////////////////////////////////////////////////////////////////
	actionlib::SimpleActionClient<franka_gripper::GraspAction> grip_client("/franka_gripper/grasp", true);
	franka_gripper::GraspActionGoal grip_goal;	

	/////////////////////////////////////////////////////////////////////////
	static const std::string PLANNING_GROUP = "panda_arm";

	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

	robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));
	const robot_state::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);

	planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
	planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "ready");
	moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
	visual_tools.deleteAllMarkers();
	visual_tools.trigger();



	//LOADING A PLANNER
	boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
	planning_interface::PlannerManagerPtr planner_instance;
	std::string planner_plugin_name;

	//from tutorial
	M_NH.getParam("planning_plugin", planner_plugin_name);
	std::cout<<"\n\n\n\n"<<planner_plugin_name<<std::endl;
	if (!M_NH.getParam("planning_plugin", planner_plugin_name))
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
		if (!planner_instance->initialize(robot_model, M_NH.getNamespace()))
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

	move_group.setPlannerId("RRTConnectkConfigDefault");
	ROS_INFO_NAMED("manipulator_node", "Reference frame: %s", move_group.getPlanningFrame().c_str());

	std::vector<moveit_msgs::CollisionObject> colObjVec;
	std::vector<std::string> colObjVec_domain_lbls;
	int ind = 0;

	std::vector<std::string> obstacle_names;
	M_NH.getParam("/workspace/obstacle_names",obstacle_names);

	std::vector<std::string> obstacle_types;
	M_NH.getParam("/workspace/obstacle_types",obstacle_types);

	std::vector<std::string> obstacle_domains;
	M_NH.param("/workspace/obstacle_domains",obstacle_domains, {});

	if (obstacle_names.size() != obstacle_types.size()) {
		ROS_ERROR("Number of obstacle names must match number of obstacle types");
		return 1;
	}

	//QUATERNIONS:

	// Quaternions:
	tf2::Quaternion q_init, q_rot, q_res;
	q_init[0] = 0;
	q_init[1] = 0;
	q_init[2] = 1;
	q_init[3] = 0;
	geometry_msgs::Quaternion q_up_x, q_up_y, q_side_x, q_side_y;
	// Up x
	tf2::convert(q_init, q_up_x);
	
	// Up y
	q_rot.setRPY(0, 0, M_PI/2);
	q_res = q_rot * q_init;
	tf2::convert(q_res, q_up_y);

	// Side x
	q_rot.setRPY(0, -M_PI/2, 0);
	q_res = q_rot * q_init;
	tf2::convert(q_res, q_side_x);

	// Side y
	q_rot.setRPY(-M_PI/2, M_PI/2, 0);
	q_res = q_rot * q_init;
	tf2::convert(q_res, q_side_y);


	std::vector<std::map<std::string, float>> obstacle_points(obstacle_names.size());
	std::vector<std::string> obstacle_orientation_types;
	M_NH.getParam("/workspace/obstacle_orientation_types", obstacle_orientation_types);
	for (int i=0; i<obstacle_names.size(); ++i) {
        M_NH.getParam("/workspace/" + obstacle_names[i], obstacle_points[i]);
        std::cout<<"Loaded obstacle for: "<<obstacle_names[i]<<std::endl;
        std::cout<<"  - x: "<<obstacle_points[i].at("x")<<"\n";
        std::cout<<"  - y: "<<obstacle_points[i].at("y")<<"\n";
        std::cout<<"  - z: "<<obstacle_points[i].at("z")<<"\n";
		float x = obstacle_points[i].at("x");
		float y = obstacle_points[i].at("y");
		float z = obstacle_points[i].at("z");

		moveit_msgs::CollisionObject obs;
        obs.header.frame_id = "panda_link0";
		obs.id = obstacle_names[i];
		bool single_primitive = true;

		if (obstacle_types[i] == "box") {
			obs.primitives.resize(1);
			obs.primitives[0].type = obs.primitives[0].BOX;
			obs.primitives[0].dimensions.resize(3);
			obs.primitives[0].dimensions[0] = obstacle_points[i].at("l");
			obs.primitives[0].dimensions[1] = obstacle_points[i].at("w");
			obs.primitives[0].dimensions[2] = obstacle_points[i].at("h");

		} else if (obstacle_types[i] == "sphere") {
			obs.primitives.resize(1);
			obs.primitives[0].type = obs.primitives[0].SPHERE;
			obs.primitives[0].dimensions.resize(1);
			obs.primitives[0].dimensions[0] = obstacle_points[i].at("r");
		} else if (obstacle_types[i] == "cylinder") {
			obs.primitives.resize(1);
			obs.primitives[0].type = obs.primitives[0].CYLINDER;
			obs.primitives[0].dimensions.resize(2);
			obs.primitives[0].dimensions[0] = obstacle_points[i].at("h");
			obs.primitives[0].dimensions[1] = obstacle_points[i].at("r");
		} else if (obstacle_types[i] == "bin") {
			single_primitive = false;
			// TODO
		} else {
			std::string msg = "Did not find obstacle primitive type:" + obstacle_types[i];
			ROS_ERROR_STREAM(msg.c_str());
			return 1;
		}

		if (single_primitive) {
			obs.primitive_poses.resize(1);
			obs.primitive_poses[0].position.x = x;
			obs.primitive_poses[0].position.y = y;
			obs.primitive_poses[0].position.z = z; 

			// Add condition if orientation type = 'manual'
			if (obstacle_orientation_types[i] == "up_x") {
				obs.primitive_poses[0].orientation.x = q_up_x.x;
				obs.primitive_poses[0].orientation.y = q_up_x.y;
				obs.primitive_poses[0].orientation.z = q_up_x.z;
				obs.primitive_poses[0].orientation.w = q_up_x.w;
			} else if (obstacle_orientation_types[i] == "up_y") {
				obs.primitive_poses[0].orientation.x = q_up_y.x;
				obs.primitive_poses[0].orientation.y = q_up_y.y;
				obs.primitive_poses[0].orientation.z = q_up_y.z;
				obs.primitive_poses[0].orientation.w = q_up_y.w;
			} else if (obstacle_orientation_types[i] == "side_x") {
				obs.primitive_poses[0].orientation.x = q_side_x.x;
				obs.primitive_poses[0].orientation.y = q_side_x.y;
				obs.primitive_poses[0].orientation.z = q_side_x.z;
				obs.primitive_poses[0].orientation.w = q_side_x.w;
			} else if (obstacle_orientation_types[i] == "side_y") {
				obs.primitive_poses[0].orientation.x = q_side_y.x;
				obs.primitive_poses[0].orientation.y = q_side_y.y;
				obs.primitive_poses[0].orientation.z = q_side_y.z;
				obs.primitive_poses[0].orientation.w = q_side_y.w;
			} else {
				std::string msg = "Did not find orientation preset:" + obstacle_orientation_types[i];
				ROS_ERROR_STREAM(msg.c_str());
				return 1;
			}
			obs.operation = obs.ADD;

		}
		colObjVec.push_back(obs);
		if (i >= obstacle_domains.size()) {
			colObjVec_domain_lbls.push_back("domain");
		} else {
			colObjVec_domain_lbls.push_back(obstacle_domains[i]);
		}

	}

	//planning_scene_interface.applyCollisionObjects(colObjVec);
	
	move_group.setEndEffectorLink("panda_link8");

	//PlanningQuerySrv plan_query_srv_container(&move_group, &planning_scene_interface, &grip_client, 5, !sim_only, !sim_only, true);
	PlanningQuerySrv plan_query_srv_container(&move_group, &planning_scene_interface, &grip_client, 5, !sim_only, true, true);
	plan_query_srv_container.setWorkspace(colObjVec, colObjVec_domain_lbls);

	ros::ServiceServer plan_query_service = M_NH.advertiseService("/manipulation_planning_query", &PlanningQuerySrv::planQuery_serviceCB, &plan_query_srv_container);
	ROS_INFO("Manipulation query service online!");


	ros::waitForShutdown();
	return 0;
}
