#!/usr/bin/env python
"""
Node to show direction arrows

@author Lukas Rustler
"""
import rospy
from geometry_msgs.msg import Quaternion, Point, Pose
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np

def show_arrow(topic='movement_direction'):
    """
    Show arrow in RVIZ from point to impact point
    @return:
    @rtype:
    """

    #topic = 'movement_direction'
    publisher = rospy.Publisher(topic, MarkerArray, queue_size=20)
    while not rospy.is_shutdown():  # give RVIZ time to get the message
        impact_point = rospy.get_param("/shape_completion/impact_point", False)
        if not impact_point:
            rospy.sleep(0.1)
            continue
        impact_point = np.array(eval(impact_point))
        direction = np.array(eval(rospy.get_param("/shape_completion/direction")))
        angle_axis = np.array(eval(rospy.get_param("/shape_completion/angle_axis")))

        # common settings
        marray = MarkerArray()
        marker = Marker()
        marker.id = 0
        marker.header.frame_id = "/base_link"
        marker.action = marker.ADD
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        # Cube corresponding to starting point
        marker.type = marker.CUBE
        marker.pose.position.x = (impact_point+direction)[0]
        marker.pose.position.y = (impact_point+direction)[1]
        marker.pose.position.z = (impact_point+direction)[2]
        marker.pose.orientation = Quaternion(*angle_axis)
        marray.markers.append(marker)

        # Arrow to impact point
        marker.id = 1
        marker.pose = Pose()
        marker.type = marker.ARROW
        marker.points.append(Point(*impact_point+direction))
        marker.points.append(Point(*impact_point))
        marker.pose.orientation.w = 1
        marray.markers.append(marker)

        publisher.publish(marray)
        rospy.sleep(0.1)


if __name__ == "__main__":
    rospy.init_node("arrow_shower")
    try:
        show_arrow()
    except rospy.ROSInterruptException:
        pass
