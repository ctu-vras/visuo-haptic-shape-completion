#!/usr/bin/env python2
"""
Util to read point cloud and save it

@author Lukas Rustler
"""
import pcl
import rospy
from sensor_msgs.msg import PointCloud2
import numpy as np
import os
import tf
import tf.transformations as ts
from shape_completion.subscriber_classes import POINTCLOUD
from shape_completion.srv import smooth_pcl


def save_pcl(out_path, camera="table", C=False):
    """
    Subscribes to topic with segmented pointcloud, transforms it into /base_link and saves it into file
    @param out_path: path where to save the resulting .pcd file
    @type out_path: string
    @param camera: Which camera to use -- table, kinova
    @type camera: string
    @param C: whether to use post processing in C or Python
    @type C: bool
    @return:
    @rtype:
    """
    t = rospy.Time.now().to_time()

    rospy.loginfo("Waiting for segmented point cloud message")
    # Create class handle for subscriber and set right parameters
    class_handle = POINTCLOUD(2)
    if camera == "table":
        class_handle.sub = rospy.Subscriber("/table_camera_segmentation/point_cloud_segmentation/segmented_pc", PointCloud2, class_handle)
    elif camera == "kinova":
        class_handle.sub = rospy.Subscriber("/kinova_camera_segmentation/point_cloud_segmentation/segmented_pc", PointCloud2, class_handle)
    while not class_handle.end:
        rospy.sleep(0.1)

    points = class_handle.points
    print(points.shape)
    if camera == "table":
        # Run voxel grid filer, moving least squares filer and remove outliers
        if C:
            pc = pcl.PointCloud_PointXYZRGBA(points.astype("float32"))
            #pc = pcl.PointCloud(points[:, :3].astype("float32"))
            pcl.save(pc, "/home/robot3/inp.pcd")
            try:
                parametrize_cartesian_path_service = rospy.ServiceProxy('smooth_pcl',
                                                                        smooth_pcl)
                parametrize_cartesian_path_service("/home/robot3/inp.pcd", 0.0005, 0.01)
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)

            pc = pcl.load_XYZRGBA("/home/robot3/smooth.pcd")
            points = pc.to_array()

        else:
            pc = pcl.PointCloud(points[:, :3].astype("float32"))

            vg = pc.make_voxel_grid_filter()
            vg.set_leaf_size(0.001, 0.001, 0.001)
            pc = vg.filter()

            tree = pc.make_kdtree()
            mls = pc.make_moving_least_squares()
            mls.set_Compute_Normals(False)
            mls.set_polynomial_fit(True)
            mls.set_Search_Method(tree)
            mls.set_search_radius(0.015)
            pc = mls.process()

            sor = pc.make_statistical_outlier_filter()
            sor.set_mean_k(100)
            sor.set_std_dev_mul_thresh(2.0)
            pc = sor.filter()

            points = pc.to_array()
    else:
        # Only voxel grid filter
        pc = pcl.PointCloud(points[:, :3].astype("float32"))

        vg = pc.make_voxel_grid_filter()
        vg.set_leaf_size(0.001, 0.001, 0.001)
        pc = vg.filter()
        points = pc.to_array()


    rgba = []
    rospy.loginfo("Message from PCL topic received")

    # For this case we can export also colors correctly
    if camera == "table" and C:
        # Get color R,G,B,A colors values from one UINT32 number
        for idx, p in enumerate(points):
            if not class_handle.is_bigendian:
                b = np.uint32(p[3]) >> np.uint32(0) & np.uint32(255)
                g = np.uint32(p[3]) >> np.uint32(8) & np.uint32(255)
                r = np.uint32(p[3]) >> np.uint32(16) & np.uint32(255)
                a = np.uint32(p[3]) >> np.uint32(24) & np.uint32(255)
                p = r << np.uint32(16) | g << np.uint32(8) | b << np.uint(0)  # a << np.uint32(0) |
            else:  # not tested
                b = np.uint32(p) >> np.uint32(24) & np.uint32(255)
                g = np.uint32(p) >> np.uint32(16) & np.uint32(255)
                r = np.uint32(p) >> np.uint32(8) & np.uint32(255)
                a = np.uint32(p) >> np.uint32(0) & np.uint32(255)
                p = r << np.uint32(16) | g << np.uint32(8) | b << np.uint(0)

            points[idx, 3] = p
            rgba.append([r, g, b, a])
            # print(r, g, b, a)

    # Transform point cloud to base
    tf_listener = tf.TransformListener()
    while True:
        try:
            if camera == "table":
                (trans, rot) = tf_listener.lookupTransform('/base_link', '/virtual_camera', rospy.Time(0))
            else:
                (trans, rot) = tf_listener.lookupTransform('/base_link', '/camera_kinova_color_frame', rospy.Time(0))
            transform = ts.concatenate_matrices(ts.translation_matrix(trans), ts.quaternion_matrix(rot))
            break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
    points_homog = np.hstack((points[:, :3], np.ones((points.shape[0], 1))))
    # There is a fixed rotation between RVIZ and what we want to see
    R = np.array([[0, 0, 1, 0], [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]])
    transform = np.matmul(R.T, transform)
    points_homog = np.matmul(transform, points_homog.T)
    points[:, :3] = points_homog[:3, :].T

    # Save .pcd file and .npz with coordinates and colors
    rospy.loginfo("Saving pointcloud to file")
    if camera == "table" and C:
        pc = pcl.PointCloud_PointXYZRGBA(points.astype("float32"))
    else:
        pc = pcl.PointCloud(points.astype("float32"))

    if not os.path.exists(os.path.dirname(out_path.replace("pcd", "npz"))):
        os.makedirs(os.path.dirname(out_path.replace("pcd", "npz")))
    if not os.path.exists(os.path.dirname(out_path)):
        os.makedirs(os.path.dirname(out_path))
    pcl.save(pc, out_path)

    np.savez(out_path.replace("pcd", "npz"), points=points[:, :3], colors=rgba)
    return points

if __name__ == "__main__":
    rospy.init_node("pcl_saver", anonymous=True)
    save_pcl("/home/robot3/test.pcd", camera="table", C=False)
