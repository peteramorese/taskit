#!/usr/bin/env python
from taskit.srv import GraspSrv, ReleaseSrv, StowSrv, TransitSrv, UpdateEnvSrv
import rospy

### Service call names when running the real robot
# /manipulator_node/action_primitive/grasp                /manipulator_node/action_primitive/stow
# /manipulator_node/action_primitive/linear_transit       /manipulator_node/action_primitive/transit_side
# /manipulator_node/action_primitive/linear_transit_side  /manipulator_node/action_primitive/transit_up
# /manipulator_node/action_primitive/linear_transport     /manipulator_node/action_primitive/transport
# /manipulator_node/action_primitive/release              /manipulator_node/action_primitive/update_environment



#### Servoce call names when using tje Sim robot. Need to make them consistent.
# /manipulator_node/action_primitive/grasp                /manipulator_node/action_primitive/stow
# /manipulator_node/action_primitive/linear_transit       /manipulator_node/action_primitive/transit
# /manipulator_node/action_primitive/linear_transit_side  /manipulator_node/action_primitive/transit_side
# /manipulator_node/action_primitive/linear_transport     /manipulator_node/action_primitive/transport
# /manipulator_node/action_primitive/release              /manipulator_node/action_primitive/update_environment



def send_commands_to_robot(obj_id: str, start_loc: str, end_loc: str) -> bool:
	""" 
	A helper function that run a set of pick and place for object
	"""

	s1 = send_transit_command_to_robot(start_loc)
	s2 = send_grasp_command_to_robot(obj_id)
	s3 = send_transport_command_to_robot(end_loc)
	s4 = send_release_command_to_robot(obj_id)

	return s1 & s2 & s3 & s4



def send_transport_command_to_robot(loc: str) -> bool:
	#  convenience method that blocks until the service named is available
	rospy.wait_for_service("/manipulator_node/action_primitive/linear_transport")

	# create a handle for calling the service
	transport_handle = rospy.ServiceProxy("/manipulator_node/action_primitive/linear_transport", TransitSrv)
	# Return three parsms - execution success, plan_success, and execution_time 
	t = transport_handle(loc)
	return t.execution_success



def send_transit_command_to_robot(loc: str) -> bool:
	#  convenience method that blocks until the service named is available
	
	if rospy.wait_for_service("/manipulator_node/action_primitive/linear_transit"):

		# create a handle for calling the service
		transit_handle = rospy.ServiceProxy("/manipulator_node/action_primitive/linear_transit", TransitSrv)
	else:
		rospy.wait_for_service("/manipulator_node/action_primitive/transit_up")
		# create a handle for calling the service
		transit_handle = rospy.ServiceProxy("/manipulator_node/action_primitive/transit_up", TransitSrv)

	# Return three parsms - execution success, plan_success, and execution_time 
	t = transit_handle(loc)
	return t.execution_success


def send_grasp_command_to_robot(obj_id: str) -> bool:
	#  convenience method that blocks until the service named is available
	rospy.wait_for_service("/manipulator_node/action_primitive/grasp")

	# create a handle for calling the service
	grasp_handle = rospy.ServiceProxy("/manipulator_node/action_primitive/grasp", GraspSrv)
	# Return two params - success, and execution_time 
	t = grasp_handle(obj_id)
	return t.success


def send_release_command_to_robot(obj_id: str) -> bool:
	#  convenience method that blocks until the service named is available
	rospy.wait_for_service("/manipulator_node/action_primitive/grasp")

	# create a handle for calling the service
	release_handle = rospy.ServiceProxy("/manipulator_node/action_primitive/release", ReleaseSrv)
	# Return two params - success, and execution_time 
	t = release_handle(obj_id)
	return t.success
	


if __name__ == "__main__":

	# to robustify this code, we can regularly call update_environment. Currently, the VICON tracking is very reliable.
	success = send_commands_to_robot(obj_id='A_3',start_loc='Else_1', end_loc='L3')
	if success:
		print("Done with the planning.")
	else:
		print("Failed execution of the plan.")
