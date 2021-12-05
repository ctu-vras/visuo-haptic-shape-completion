#!/usr/bin/env python
"""
Copyright (c) 2020 Jan Behrens
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Jan Behrens
"""
from __future__ import print_function

import rospy
import argparse

class CameraControl(object):
    pubs = {}
    def __init__(self):
        pass

    @staticmethod
    def control(lookat=[0,0,1], ori=[90,10], dist=[2], name='/kinova_mujoco'):
        rospy.set_param(name + '/look_at_x', lookat[0])
        rospy.set_param(name + '/look_at_y', lookat[1])
        rospy.set_param(name + '/look_at_z', lookat[2])
        rospy.set_param(name + '/look_azimuth', ori[0])
        rospy.set_param(name + '/look_elevation', ori[1])
        rospy.set_param(name + '/look_dist', dist[0])
        


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Control the camera pose in Mujoco.')
    parser.add_argument('lookat', metavar='float', type=float, nargs='+',
                        help='look at x, y, z, azimuth, elevation, distance')

    args = parser.parse_args()

    rospy.init_node('camera_controller')
    print(args.lookat)
    rospy.wait_for_service('/kinova_mujoco/reset')
    CameraControl.control(args.lookat[0:3], args.lookat[3:5], args.lookat[5:6])

