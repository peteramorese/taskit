#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from visualization_msgs.msg import MarkerArray, Marker

def main():
    """
    Test script
    """
  

if __name__ == "__main__":
    rospy.init_node('Publish_Marker')
    print("Hello World!") 
    # rospy.wait_for_service("/manipulator_node")
    marker_array_pub = rospy.Publisher('/rviz_visual_tools', MarkerArray, queue_size=10)
    r = rospy.Rate(10)

    marker_idx = 88
    marker_array_msg = MarkerArray()
    # for i in range(1):
    marker = Marker()
    marker.header.frame_id = "/map"
    marker.id = marker_idx
    marker.type = 2
    marker.action = 2
    marker.pose = Pose()
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 0.0
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.frame_locked = False
    marker.ns = "vis_marker_ns"
    marker_array_msg.markers.append(marker)
    while not rospy.is_shutdown():
        marker_array_pub.publish(marker_array_msg)
        r.sleep()
