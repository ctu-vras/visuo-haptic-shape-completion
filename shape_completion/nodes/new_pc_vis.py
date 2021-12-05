#!/usr/bin/env python
"""
Help function to visualize new point cloud

@author Lukas Rustler
"""
import rospy
from sensor_msgs.msg import PointCloud2
import std_msgs
import sensor_msgs.point_cloud2 as pcl2
import numpy as np


def new_pc_pub():
    """
    Visualize new point cloud
    @return:
    @rtype:
    """
    pointcloud_publisher = rospy.Publisher("/new_pc", PointCloud2, queue_size=20)
    while not rospy.is_shutdown():
        path = rospy.get_param("/shape_completion/new_pc_path", False)
        if not path:
            rospy.sleep(0.1)
            continue
        points_ = np.load(path)
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'base_link'
        new_pc = pcl2.create_cloud_xyz32(header, points_[:, :3])
        pointcloud_publisher.publish(new_pc)
        rospy.sleep(0.1)


if __name__ == "__main__":
    rospy.init_node("new_pc_node")
    try:
        new_pc_pub()
    except rospy.ROSInterruptException:
        pass