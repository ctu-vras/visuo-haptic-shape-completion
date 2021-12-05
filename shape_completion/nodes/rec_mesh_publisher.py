#!/usr/bin/env python
"""
Help function to visualize mesh in RVIZ

@author Lukas Rustler
"""
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


def show_reconstruction():
    topic = 'reconstruction_mesh'
    publisher = rospy.Publisher(topic, Marker, queue_size=20)

    while not rospy.is_shutdown():
        mesh_path = rospy.get_param("/shape_completion/rec_mesh_path", False)
        if not mesh_path:
            rospy.sleep(0.1)
            continue
        position = eval(rospy.get_param("/shape_completion/rec_mesh_center"))
        marker = Marker()
        marker.id = 0
        marker.ns = 'mesh_marker'
        marker.header.frame_id = "/base_link"

        marker.action = marker.ADD
        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1
        marker.color.a = 0.8
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        marker.type = marker.MESH_RESOURCE
        marker.mesh_resource = mesh_path
        marker.pose.position = Point(*position)
        marker.pose.orientation.w = 1
        publisher.publish(marker)
        rospy.sleep(0.1)


if __name__ == "__main__":
    rospy.init_node("reconstruction_shower")
    try:
        show_reconstruction()
    except rospy.ROSInterruptException:
        pass
