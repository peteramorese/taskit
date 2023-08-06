#include "ros/ros.h"
#include "TaskIt.h"

using namespace TaskIt;

static const std::string node_name = "advanced_manipulator_node_real";

int main(int argc, char** argv) {
	
    ros::init(argc, argv, node_name);
    std::shared_ptr<ros::NodeHandle> node_handle = std::make_shared<ros::NodeHandle>("~");

    // Read in the manipulator properties
    ManipulatorProperties::load(*node_handle, "panda_arm", "arm_config");

	const std::string ee_link = "panda_link8";
	
	// Make the simulation gripper handler
	std::shared_ptr<GripperHandler<GripperUse::FrankaHand>> gripper_handler = std::make_shared<GripperHandler<GripperUse::FrankaHand>>(MI_FRANKA_GRIPPER_TOPIC, 5.0);

	// Get the linear mover
	std::shared_ptr<ActionPrimitives::LinearMover> linear_mover = ActionPrimitives::makeLinearMover();

	// Construct action primitives
	ActionPrimitives::UpdateEnvironment update_environment("update_environment");
	ActionPrimitives::GetObjectLocations get_object_locations("get_object_locations");
	ActionPrimitives::Stow stow("stow");
	ActionPrimitives::SimpleGrasp<GripperUse::FrankaHand> grasp("grasp", gripper_handler, ee_link);
	ActionPrimitives::SimpleRelease<GripperUse::FrankaHand> release("release", gripper_handler);
	ActionPrimitives::Transit transit("transit", 5.0f, 1);
	ActionPrimitives::TransitUp transit_up("transit_up", 5.0f, 1);
	ActionPrimitives::TransitSide transit_side("transit_side", 5.0f, 1);
	ActionPrimitives::LinearTransit linear_transit("linear_transit", 5.0f, 1, 0.05, linear_mover);
	ActionPrimitives::LinearTransitUp linear_transit_up("linear_transit_up", 5.0f, 1, 0.05, linear_mover);
	ActionPrimitives::LinearTransitSide linear_transit_side("linear_transit_side", 5.0f, 1, 0.05, linear_mover);
	ActionPrimitives::Transport transport("transport", 5.0f, 1);
	ActionPrimitives::LinearTransport linear_transport("linear_transport", 5.0f, 1, 0.05, linear_mover);

	ManipulatorNode<
	 	decltype(update_environment),
		decltype(get_object_locations),
	 	decltype(stow),
		decltype(grasp), 
		decltype(release), 
		decltype(transit),
		decltype(transit_up),
		decltype(transit_side),
		decltype(linear_transit),
		decltype(linear_transit_up),
		decltype(linear_transit_side),
		decltype(transport),
		decltype(linear_transport)
	> manipulator_node(node_handle, node_name, "panda_arm", "panda_link0", 
		std::move(update_environment), 
		std::move(get_object_locations), 
		std::move(stow), 
		std::move(grasp), 
		std::move(release), 
		std::move(transit), 
		std::move(transit_up), 
		std::move(transit_side), 
		std::move(linear_transit), 
		std::move(linear_transit_up), 
		std::move(linear_transit_side),
		std::move(transport),
		std::move(linear_transport)
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


