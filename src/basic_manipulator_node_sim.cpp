#include "ros/ros.h"
#include "TaskIt.h"

using namespace TaskIt;

static const std::string node_name = "basic_manipulator_node_sim";

int main(int argc, char** argv) {
	
    ros::init(argc, argv, node_name);
    std::shared_ptr<ros::NodeHandle> node_handle = std::make_shared<ros::NodeHandle>("~");

	const std::string ee_link = "panda_link8";
	
	// Make the simulation gripper handler
	std::shared_ptr<GripperHandler<GripperUse::Simulation>> gripper_handler = std::make_shared<GripperHandler<GripperUse::Simulation>>(MI_FRANKA_GRIPPER_TOPIC, 5.0);

	// Construct action primitives
	ActionPrimitives::UpdateEnvironment update_environment("update_environment");
	ActionPrimitives::Stow stow("stow");
	ActionPrimitives::SimpleGrasp<GripperUse::Simulation> grasp("grasp", gripper_handler, ee_link);
	ActionPrimitives::SimpleRelease<GripperUse::Simulation> release("release", gripper_handler);
	ActionPrimitives::TransitUp transit("transit", 5.0f, 1);
	ActionPrimitives::Transport transport("transport", 5.0f, 1, 0.05);

	ManipulatorNode<
		ActionPrimitives::UpdateEnvironment,
	 	ActionPrimitives::Stow,
		ActionPrimitives::SimpleGrasp<GripperUse::Simulation>, 
		ActionPrimitives::SimpleRelease<GripperUse::Simulation>, 
		ActionPrimitives::TransitUp,
		ActionPrimitives::Transport
	> manipulator_node(node_handle, node_name, "panda_arm", "panda_link0", 
	 	std::move(update_environment),
		std::move(stow), 
		std::move(grasp), 
		std::move(release), 
		std::move(transit), 
		std::move(transport)
	);

	ros::WallDuration(1.0).sleep();

	// Pose tracker
	std::shared_ptr<PoseTracker> pose_tracker;

	std::string pose_tracker_type;
	if (node_handle->getParam("pose_tracker/type", pose_tracker_type)) {
		if (pose_tracker_type == "simulation") {
			pose_tracker = std::make_shared<SimulationPoseTracker>();
		} else if (pose_tracker_type == "vrpn") {
			double sampling_duration = node_handle->param("pose_tracker/sampling_duration", 0.1);
			pose_tracker = std::make_shared<VRPNPoseTracker>(node_handle, sampling_duration);
		} else {
			ROS_ERROR_STREAM_NAMED(node_name, "Unrecognized pose tracker type '" << pose_tracker_type.c_str() << "'");
		}
	} else {
		ROS_WARN_NAMED(node_name, "Did not find param 'pose_tracker', assuming type 'simulation'");
		pose_tracker = std::make_shared<SimulationPoseTracker>();
	}

	// Wait for rviz topic to come up before creating scene
	ros::WallDuration(5.0).sleep();

	manipulator_node.createScene(pose_tracker);
	manipulator_node.updatePlanningScene();

	manipulator_node.spawnAllActionServices();

	ros::waitForShutdown();
	return 0;
}


