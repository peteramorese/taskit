#!/usr/bin/env python

'''
In this strategy the human is adversarial and does not move any block.
The Robot accordingly finishes the task in the Robot region. Note: This is identical to
the Regret_adv_str file.
'''
import sys
from taskit.srv import Grasp, Release, Stow, Transit, UpdateEnv
import rospy


def send_commands_to_robot(obj_id: str, start_loc: str, end_loc: str):
	""" 
	A helper function that run a set of pick and place for object
	"""

	s1 = send_transit_command_to_robot(start_loc)
	if s1:
		s2 = send_grasp_command_to_robot(obj_id)
	if s2:
		s3 = send_transport_command_to_robot(end_loc)
	if s3:
		s4 = send_release_command_to_robot(obj_id)

	if not (s1 & s2 & s3 & s4):
		print("Failed execution of the plan.")
		sys.exit(-1)

	# update the env after dropping
	update_env_status()


def stow_robot() -> bool:
	# helper function that updated the status (X, Y, Z) of the all the objects from vicon
	rospy.wait_for_service("/manipulator_node/action_primitive/stow")

	rospy.ServiceProxy("/manipulator_node/action_primitive/stow", Stow)()
	


def update_env_status() -> bool:
	# helper function that motion plans and stows the robot
	rospy.wait_for_service("/manipulator_node/action_primitive/update_environment")

	update_handle = rospy.ServiceProxy("/manipulator_node/action_primitive/update_environment", UpdateEnv)
	t = update_handle(False)


def send_transport_command_to_robot(loc: str) -> bool:
	#  convenience method that blocks until the service named is available
	rospy.wait_for_service("/manipulator_node/action_primitive/linear_transport")

	# create a handle for calling the service
	transport_handle = rospy.ServiceProxy("/manipulator_node/action_primitive/linear_transport", Transit)
	t = transport_handle(loc)
	return t.plan_success



def send_transit_command_to_robot(loc: str) -> bool:
	#  convenience method that blocks until the service named is available
	
	rospy.wait_for_service("/manipulator_node/action_primitive/linear_transit_up")

	# create a handle for calling the service
	transit_handle = rospy.ServiceProxy("/manipulator_node/action_primitive/linear_transit", Transit) 
	t = transit_handle(loc)
	return t.plan_success


def send_grasp_command_to_robot(obj_id: str) -> bool:
	#  convenience method that blocks until the service named is available
	rospy.wait_for_service("/manipulator_node/action_primitive/grasp")

	# create a handle for calling the service
	grasp_handle = rospy.ServiceProxy("/manipulator_node/action_primitive/grasp", Grasp)
	# t = grasp_handle(obj_id)
	t = grasp_handle('')
	return t.mv_props.execution_success


def send_release_command_to_robot(obj_id: str) -> bool:
	#  convenience method that blocks until the service named is available
	rospy.wait_for_service("/manipulator_node/action_primitive/grasp")

	# create a handle for calling the service
	release_handle = rospy.ServiceProxy("/manipulator_node/action_primitive/release", Release)
	# t = release_handle(obj_id)
	t = release_handle('')
	return t.mv_props.execution_success
	


if __name__ == "__main__":

	# init and update the status of all the boxes
	update_env_status()

	# to robustify this code, we can regularly call update_environment. Currently, the VICON tracking is very reliable.
	send_commands_to_robot(obj_id='',start_loc='HL0', end_loc='L2')
	 
	send_commands_to_robot(obj_id='',start_loc='L0', end_loc='L1')


	# send_transit_command_to_robot('Ready')
	stow_robot()

	print("Done with the planning.")