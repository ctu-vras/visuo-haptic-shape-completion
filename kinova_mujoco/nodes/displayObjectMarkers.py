#!/usr/bin/env python
"""
Copyright (c) 2020 Jan Behrens
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Jan Behrens
"""

import rospy
from visualization_msgs.msg import MarkerArray, Marker
from mujoco_interface_msgs.srv import GetAllObjectPoses, GetAllObjectPosesRequest, GetAllObjectPosesResponse
from urdf_parser_py.urdf import URDF
from rospkg import RosPack
import os

STATE_SRV_NAME = '/kinova_mujoco/getAllObjectPoses'
STATE_SRV = None
DISPLAY_TOPIC = 'scene_objects'
DISPLAY_PUB = None
URDF_PGK = "kinova_mujoco"
URDF_FILE = "world_model/kinova_fel_shape_completion_NoDae.urdf"
robot = None
MODEL_SCALE = 1.0

def update(event):
    # pull state from mujoco
    req = GetAllObjectPosesRequest()
    res = STATE_SRV.call(req)
    assert isinstance(res, GetAllObjectPosesResponse)
    # update/add meshes to rviz
    ma = MarkerArray()
    for name, pose in zip(res.names, res.poses):
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = rospy.Time.now()
        marker.ns = name
        marker.id = 0
        marker.type = Marker.MESH_RESOURCE
        # if we don't know the mesh name, we just skip it. Thus, anything not in the URDF will not be displayed in RVIZ
        try:
            marker.mesh_resource = robot.link_map[name].visual.geometry.filename
        except KeyError:
            continue
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.scale.x = MODEL_SCALE
        marker.scale.y = MODEL_SCALE
        marker.scale.z = MODEL_SCALE
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        ma.markers.append(marker)

    DISPLAY_PUB.publish(ma)

if __name__=="__main__":
    rospy.init_node('state_display')
    rospack = RosPack()
    # rospack.get_path(URDF_PGK)
    SCENE_PATH = os.path.join(rospack.get_path(URDF_PGK), URDF_FILE)
    robot = URDF.from_xml_file(SCENE_PATH)
    STATE_SRV = rospy.ServiceProxy(STATE_SRV_NAME, GetAllObjectPoses)
    rospy.wait_for_service(STATE_SRV_NAME)
    DISPLAY_PUB = rospy.Publisher(DISPLAY_TOPIC, MarkerArray, queue_size=1)
    rospy.timer.Timer(rospy.Duration.from_sec(0.1), update)

    rospy.spin()



