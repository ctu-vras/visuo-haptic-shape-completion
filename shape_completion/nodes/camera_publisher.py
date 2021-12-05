#!/usr/bin/env python
"""
Node to publish cameras with correct names

@author Lukas Rustler
"""
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
import tf
import tf2_ros
import tf.transformations as ts
from geometry_msgs.msg import TransformStamped, Quaternion, Point
import pcl


def publish_camera():
    """
    Function to prepare handles
    @return:
    @rtype:
    """
    camera = rospy.get_param("/camera_used")

    if camera == "both":
        #merge and publish
        class_handle_2 = CAMERA_1()
        class_handle_2.sub = rospy.Subscriber("/camera_1/depth/color/points", PointCloud2, class_handle_2)

        class_handle = CAMERA_0(True)
        class_handle.camera_1 = class_handle_2
        class_handle.tf_listener = tf.TransformListener()
        class_handle.pub = rospy.Publisher("/camera_point_cloud", PointCloud2, queue_size=10)
        class_handle.sub = rospy.Subscriber("/camera_0/depth/color/points", PointCloud2, class_handle)
    else:
        # republish to new topic
        class_handle = REMAPER(10)
        class_handle.pub = rospy.Publisher("/camera_point_cloud", PointCloud2, queue_size=10)
        class_handle.sub = rospy.Subscriber("/camera_"+str(camera)+"/depth/color/points", PointCloud2, class_handle)
        class_handle_2 = None

    return class_handle, class_handle_2, camera


class REMAPER:
    """
    Just change name of topics
    """
    def __init__(self, num_messages):
        self.sub = None
        self.pub = None
        self.points = np.array([])
        self.received = 0
        self.num_messages = num_messages
        self.shape_to_delete = np.array([])

    def __call__(self, data):
        """
        Redirect message from topic to another one
        @return:
        @rtype:
        """
        self.pub.publish(data)

        return 0


class CAMERA_0:
    """
    Handler for main camera, which can use ICP to fit the two cameras
    """
    def __init__(self, icp):
        self.sub = None
        self.pub = None
        self.camera_1 = None
        self.transform = None
        self.camera_transform = None
        self.icp = icp

    def __call__(self, data):
        if self.camera_1.data is not None:
            points_camera_1 = np.array(list(pc2.read_points(self.camera_1.data)))
            if self.camera_transform is None:
                while True:
                    try:
                        (trans, rot) = self.tf_listener.lookupTransform('/camera_0_depth_optical_frame', '/camera_1_depth_optical_frame', rospy.Time(0))
                        break
                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                        continue

                self.camera_transform = ts.concatenate_matrices(ts.translation_matrix(trans), ts.quaternion_matrix(rot))

            points_homog = np.hstack((points_camera_1[:, :3], np.ones((points_camera_1.shape[0], 1))))
            points_homog = np.matmul(self.camera_transform, points_homog.T)
            points_camera_1[:, :3] = points_homog[:3, :].T
            points_camera_0 = np.array(list(pc2.read_points(data)))
            if self.icp and self.transform is None:
                rospy.logerr("Starting ICP")

                cam0_idx = np.logical_and(np.logical_and(points_camera_0[:, 1] > -0.3, points_camera_0[:, 1] < 0.16),
                                          np.logical_and(points_camera_0[:, 2] > 0.4, points_camera_0[:, 2] < 0.8))
                cam1_idx = np.logical_and(np.logical_and(points_camera_1[:, 1] > -0.3, points_camera_1[:, 1] < 0.16),
                                          np.logical_and(points_camera_1[:, 2] > 0.4, points_camera_1[:, 2] < 0.8))

                pc_cam0 = pcl.PointCloud(points_camera_0[cam0_idx, :3].astype(np.float32))
                pc_cam1 = pcl.PointCloud(points_camera_1[cam1_idx, :3].astype(np.float32))
                pcl.save(pc_cam0, "/home/robot3/test.pcd")
                icp = pc_cam0.make_IterativeClosestPoint()
                _, self.transform, _, f = icp.icp(pc_cam0, pc_cam1, max_iter=500)
                rospy.logerr("Ending ICP")
            points_homog = np.hstack((points_camera_1[:, :3], np.ones((points_camera_1.shape[0], 1))))
            points_homog = np.matmul(self.transform, points_homog.T)
            points_camera_1[:, :3] = points_homog[:3, :].T

            # publish all to new topic
            points = np.vstack((points_camera_0, points_camera_1))

            h = Header()
            h.stamp = rospy.Time.now()
            h.frame_id = "camera_0_depth_optical_frame"
            fields = [PointField('x', 0, PointField.FLOAT32, 1),
                      PointField('y', 4, PointField.FLOAT32, 1),
                      PointField('z', 8, PointField.FLOAT32, 1),
                      PointField('rgb', 16, PointField.FLOAT32, 1),
                      ]
            pc = pc2.create_cloud(h, fields, points)

            self.pub.publish(pc)


class CAMERA_1:
    """
    Second camera subscriber just to save data
    """
    def __init__(self):
        self.sub = None
        self.data = None

    def __call__(self, data):
        self.data = data


if __name__ == "__main__":
    """Logic of the functions"""
    rospy.init_node("camera_publisher", anonymous=True)
    try:
        class_handle, class_handle_2, camera = publish_camera()
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            static_transformStamped = TransformStamped()
            static_transformStamped.header.stamp = rospy.Time.now()
            static_transformStamped.child_frame_id = "virtual_camera"
            static_transformStamped.transform.translation = Point(*[0, 0, 0])
            quat = tf.transformations.quaternion_from_euler(np.deg2rad(-90), 0, np.deg2rad(-90))
            static_transformStamped.transform.rotation = Quaternion(*quat)

            if camera == "both":
                static_transformStamped.header.frame_id = "camera_0_link"
            else:
                static_transformStamped.header.frame_id = "camera_"+str(camera)+"_link"

            broadcaster.sendTransform(static_transformStamped)
            rate.sleep()

        class_handle.sub.unregister()
        if class_handle_2 is not None:
            class_handle_2.sub.unregister()
    except rospy.ROSInterruptException:
        pass

