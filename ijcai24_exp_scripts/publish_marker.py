#!/usr/bin/env python

import rospy
from visualization_msgs.msg import MarkerArray, Marker

def main():
    """
    Test script
    """
    rospy.init_node('Publish_Marker')
    print("Hello World!")
    rospy.wait_for_service("/manipulator_node")
    marker_array_pub = rospy.Publisher('/rviz_visual_tools', MarkerArray, queue_size=10)
r = rospy.Rate(10)

	marker_idx = 0
    # marker_array_msg = MarkerArray()
    # for i in range(1):
    marker = Marker()
    marker.header.frame_id = "/map"
    marker.id = self.marker_idx
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
    # marker_array_msg.markers.append(marker)
    marker_array_pub.publish(marker)
    r.sleep()


if __name__ == "__main__":
    main()
    


# class PublishMarket():
# 	def __init__(seld):
# 		self._init_markers()
# 		self.marker_array_pub = rospy.Publisher('/rviz_visual_tools',MarkerArray,queue_size=10)

# 	def _init_markers(self):
# 	    marker_idx = 0
# 	    marker_array_msg = MarkerArray()
# 	    for i in range(1):
# 	        marker = Marker()
# 	        marker.header.frame_id = self.global_frame
# 	        marker.id = self.marker_idx
# 	        marker.type = 2
# 	        marker.action = 2
# 	        marker.pose = Pose()
# 	        marker.color.r = 0.0
# 	        marker.color.g = 0.0
# 	        marker.color.b = 0.0
# 	        marker.color.a = 0.0
# 	        marker.scale.x = 0.1
# 	        marker.scale.y = 0.1
# 	        marker.scale.z = 0.1
# 	        marker.frame_locked = False
# 	        marker.ns = "Goal-%u"%i
# 	        marker_array_msg.markers.append(marker)