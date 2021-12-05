#!/usr/bin/env python
"""
Node to rotate the camera appropriately

@author Lukas Rustler
"""
import rospy
import tf
import numpy as np


def send_tf():
    rospy.init_node('camera_tf_broadcaster')
    rate = rospy.Rate(10)
    br = tf.TransformBroadcaster()
    while not rospy.is_shutdown():
        p_x = rospy.get_param('/kinova_mujoco/look_at_x')
        p_y = rospy.get_param('/kinova_mujoco/look_at_y')
        p_z = rospy.get_param('/kinova_mujoco/look_at_z')
        phi = rospy.get_param('/kinova_mujoco/look_azimuth')
        theta = rospy.get_param('/kinova_mujoco/look_elevation')
        r = rospy.get_param('/kinova_mujoco/look_dist')

        r = np.sign(-np.cos(phi))*r
        x_ = r*np.cos(np.deg2rad(phi))*np.sin(np.deg2rad(theta))
        y  = r*np.sin(np.deg2rad(phi))*np.sin(np.deg2rad(theta))
        z_ = r*np.cos(np.deg2rad(theta))
        x = z_
        z = x_
        R = np.eye(3)

        t = np.deg2rad(phi)-np.pi/2
        R = np.matmul(R, [[np.cos(t), -np.sin(t), 0], [np.sin(t), np.cos(t), 0], [0, 0, 1]])

        t = -np.pi / 2
        R = np.matmul(R, [[1, 0, 0], [0, np.cos(t), -np.sin(t)], [0, np.sin(t), np.cos(t)]])

        t = np.deg2rad(theta)
        R = np.matmul(R, [[1, 0, 0], [0, np.cos(t), -np.sin(t)], [0, np.sin(t), np.cos(t)]])

        R_ = np.zeros((4, 4))
        R_[0:3, 0:3] = R
        # 0.005, 0.005
        R_[0:3, 3] = [x+p_x-0.01, y+p_y, z+p_z-0.004]
        R_[3, 3] = 1

        br.sendTransform(R_[:3, 3], tf.transformations.quaternion_from_matrix(R_), rospy.Time.now(),'virtual_camera','base_link')
        rate.sleep()


if __name__ == "__main__":
    try:
        send_tf()
    except rospy.ROSInterruptException:
        pass