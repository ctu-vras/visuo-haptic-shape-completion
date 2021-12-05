#!/usr/bin/env python
"""
Publish transformation for the finger made on the end of the gripper

@author Lukas Rustler
"""
import rospy
import tf
import numpy as np
import tf.transformations as ts


def send_tf():
    rospy.init_node('finger_tf_broadcaster')
    rate = rospy.Rate(10)
    br = tf.TransformBroadcaster()
    ls = tf.TransformListener()
    while not rospy.is_shutdown():
        while True:
            try:
                (trans_right, _) = ls.lookupTransform('/base_link', '/right_inner_finger_pad', rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        while True:
            try:
                (trans_left, _) = ls.lookupTransform('/base_link', '/left_inner_finger_pad', rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        while True:
            try:
                (_, rot) = ls.lookupTransform('/base_link', '/tool_frame', rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

        trans = (np.array(trans_left) + np.array(trans_right))/2  # Average of pad position
        direction = np.matmul(ts.quaternion_matrix(rot), [0, 0, 1, 1])[:3]  # Rotate Z-axis vector to point in direction of gripper
        direction /= np.linalg.norm(direction)  # normalize
        trans += (34.93/1000/2)*direction  # Move finger link forward to tip of the gripper -- (34.93/2) is a half of pad length
        br.sendTransform(trans, rot, rospy.Time.now(), 'finger_link', 'base_link')
        rate.sleep()


if __name__ == "__main__":
    try:
        send_tf()
    except rospy.ROSInterruptException:
        pass