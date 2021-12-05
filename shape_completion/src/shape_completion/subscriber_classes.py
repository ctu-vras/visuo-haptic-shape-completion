"""
File with various subscriber classes allowing easier manipulation

@author Lukas Rustler
"""
import numpy as np
from std_msgs.msg import String
import sensor_msgs.point_cloud2 as pc2


class CUSUM:
    """
    Cumulative sum detection from torques
    """
    def __init__(self, threshold, drift, joints_idxs):
        self.threshold = threshold
        self.drift = drift
        self.data_last = None
        self.g_m_last = np.zeros((1, 7))
        self.g_p_last = np.zeros((1, 7))
        self.zeros_ = np.zeros((1, 7))
        self.impact_pose = None
        self.sub = None
        self.joint_idxs = joints_idxs
        self.trajectory_event_publisher = None

    def __call__(self, data):
        """
        Computes cumulative sum from joint effort data
        @return:
        @rtype:
        """

        if self.data_last is None:
            self.data_last = data
            return 0

        s = np.array(data.effort)[self.joint_idxs] - np.array(self.data_last.effort)[self.joint_idxs]

        g_p = np.maximum(self.g_p_last+s-self.drift, self.zeros_)
        g_m = np.maximum(self.g_m_last-s-self.drift, self.zeros_)

        # if any joint get over the threshold break the loop
        if np.any(g_p > self.threshold) or np.any(g_m > self.threshold):
            e = String()
            e.data = "stop"
            self.trajectory_event_publisher.publish(e)
            self.sub.unregister()
            self.impact_pose = np.array(data.position)
            return 0

        self.data_last = data
        self.g_p_last = g_p
        self.g_m_last = g_m
        return 0


class THRESHOLD:
    """
    Pure threshold detection from joint torques
    """
    def __init__(self, threshold, joints_idxs):
        self.threshold = threshold
        self.impact_pose = None
        self.sub = None
        self.joint_idxs = joints_idxs

    def __call__(self, data):
        """
        Check for torques threshold from external torques given by robot/mujoco
        @return:
        @rtype:
        """

        if np.any(np.abs(data.effort)[self.joint_idxs] > self.threshold):
            e = String()
            e.data = "stop"
            self.trajectory_event_publisher.publish(e)
            self.sub.unregister()
            self.impact_pose = np.array(data.position)

        return 0


class POINTCLOUD:
    """
    Handle to obtain pointclouds
    """
    def __init__(self, num_messages):
        self.sub = None
        self.points = np.array([])
        self.is_bigendian = None
        self.received = 0
        self.end = False
        self.num_messages = num_messages

    def __call__(self, msg):
        """
        Stack point until required number of messages is acquired
        @return:
        @rtype:
        """
        if self.received >= self.num_messages:
            self.sub.unregister()
            self.end = True
        if self.is_bigendian is None:
            self.is_bigendian = msg.is_bigendian
        if self.points.size:
            self.points = np.vstack((self.points, np.array(list(pc2.read_points(msg)))))
        else:
            self.points = np.array(list(pc2.read_points(msg)))
        self.received += 1

        return 0


