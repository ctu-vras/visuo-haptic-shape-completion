#!/usr/bin/env python2
"""
Main utils with function mainly for robot movement

@author Lukas Rustler
"""
import copy
import os
import scipy.linalg
import trimesh
import numpy as np
import glob
from subprocess import Popen, PIPE
import binvox_rw
from sklearn.cluster import MeanShift
from geometry_msgs.msg import PoseStamped
import rospy
from scipy.spatial import cKDTree as kd
import tf.transformations as ts
from geometry_msgs.msg import Quaternion, Point, Pose
import tf
from sensor_msgs.msg import JointState
import pcl
import itertools
from shape_completion.subscriber_classes import CUSUM, THRESHOLD
import ctypes
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents, PlanningScene
from shape_completion.srv import parametrize_cartesian_path
from std_msgs.msg import String


def rotate_and_scale(mesh_path, rep, offline_rec=False, scale_path=None, sim=True):
    """
    Rotates and scale the mesh to fit URDF and GT
    @param mesh_path: path to mesh folder
    @type mesh_path: str
    @param rep: From which repetition are the data, to create proper folder
    @type rep: int
    @param offline_rec: whether this function is called from offline reconstruction
    @type offline_rec: bool
    @param scale_path: path to the file with scale (only needed in some cases)
    @type scale_path: string
    @param sim: Whether the function is called from simulation or not
    @type sim: bool
    @return:
    @rtype:
    """
    files = glob.glob(os.path.join(mesh_path, "*.ply"))

    if not offline_rec:
        # load scale from file (saved while preprocessing)
        scale_path = os.path.join(mesh_path.split("meshes/")[0], "npy", mesh_path.split("meshes/")[1].split("/")[0], mesh_path.split("meshes/")[1].split("/")[0])+"_scale.npy"
        scale = np.load(scale_path)
        center = np.load(os.path.join(mesh_path.replace("meshes", "npy"), mesh_path.split("meshes/")[-1])+"_center.npy")
    else:
        for rr in [1, 2, 3, 4, 5, 6]:
            if mesh_path.split("/")[-2].split("_")[rr].split("-")[0] == "2021":
                obj_name_temp = "_".join(mesh_path.split("/")[-2].split("_")[:rr])
                break
        if scale_path is None:
            scale_path = os.path.join(mesh_path.replace("meshes", "npy"), obj_name_temp+ "_scale.npy")

        center_path = scale_path.replace("_scale", "_center")
        center = np.load(center_path)
        scale = np.load(scale_path)

    # Rotate, scale and translate back from IGR reconstruction
    translation = np.eye(4)
    translation[:3, 3] = center.T
    R = np.array([[0, 0, 1, 0], [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]])

    for file in files:
        mesh = trimesh.load(file)
        mesh.apply_scale(1/scale)
        mesh.apply_transform(translation)
        mesh.apply_transform(R)
        mesh.export(file)

    # find only the mean mesh
    mean_mesh = [f for f in files if "mean" in f][0]

    # load the mean mesh
    mesh = trimesh.load(mean_mesh)

    # transfrom center into base link (NEEDED for simulation right now!!!)
    center = np.matmul(R, np.hstack((center, 1)).T)[:3]

    # save .ply suitable for inserting into URDF
    vertices = mesh.vertices
    vertices -= center

    if not offline_rec:
        if not os.path.exists(os.path.join(mesh_path, 'rep'+str(rep))):
            os.makedirs(os.path.join(mesh_path, 'rep'+str(rep)))
        mesh.export(os.path.join(mesh_path, "rep"+str(rep), "rec.stl"))

    mesh.export(os.path.join(mesh_path, "rec.stl"))
    mesh.export(os.path.join(mesh_path, "rec.ply"))
    bbox = trimesh.bounds.corners(mesh.bounding_box.bounds)
    x_len = np.abs(bbox[0, 0]-bbox[1, 0])
    y_len = np.abs(bbox[0, 1]-bbox[2, 1])
    z_len = np.abs(bbox[0, 2]-bbox[4, 2])
    bbox_center = np.mean(bbox, axis=0)

    # translate into (0,0,0) for comparison with GT
    if sim:
        if not offline_rec:
            vertices += center - eval(rospy.get_param("/object_origin"))
        else:
            object_origin = np.load(os.path.join(mesh_path.replace("meshes", "npy"), "file_origin.npy"))
            vertices += center - object_origin
    mesh.vertices = vertices

    # rotate the mesh (to fit GT) and save
    R = np.array([[0, 0, 1, 0], [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]])
    mesh.apply_transform(R.T)
    mesh.export(os.path.join(mesh_path, "rotated.ply"))

    return center, bbox_center, [x_len, y_len, z_len]


def create_binvox(mesh_path):
    """
    Make voxels from meshes and saves them into .binvox files
    @param mesh_path: Path to 'meshes' folder
    @type mesh_path: str
    """
    files = glob.glob(os.path.join(mesh_path, "*.ply"))
    meshes = [f for f in files if "igr" in f]

    if not os.path.exists(mesh_path.replace("meshes", "binvoxes")):
        os.makedirs(mesh_path.replace("meshes", "binvoxes"))
    else:
        binvoxes = glob.glob(os.path.join(mesh_path.replace("meshes", "binvoxes"), "*.binvox"))
        for binvox in binvoxes:
            cmd = "rm "+binvox
            proc = Popen(cmd, shell=True, stdout=PIPE)
            proc.wait()

    # call 'binvox' to create voxels/binvoxes from the .ply files
    for mesh in meshes:
        cmd = "binvox -pb -e -d 40 " + mesh
        proc = Popen(cmd, shell=True, stdout=PIPE)
        proc.wait()

    # move the .binvox files into the right folder
    for mesh in meshes:
        cmd = "mv "+mesh.replace(".ply", ".binvox")+" "+mesh_path.replace("meshes", "binvoxes")
        proc = Popen(cmd, shell=True, stdout=PIPE)
        proc.wait()


def compare_binvoxes(binvoxes_path):
    """
    Computes variance between each binvox and mean binvox
    @param binvoxes_path: path to 'binvoxes' folder
    @type binvoxes_path: str
    """
    # find all binvoxes
    #binvoxes = sorted([_ for _ in glob.glob(os.path.join(binvoxes_path, "*.binvox")) if "average" not in _], key=lambda x: x.split("_")[-1].split(".")[0])[-4:]
    binvoxes = [_ for _ in glob.glob(os.path.join(binvoxes_path, "*.binvox")) if "average" not in _]
    # stack them into one matrix, except mean
    binvoxes_data = []
    for binvox_path in binvoxes:
        with open(binvox_path, 'r') as binvox_file:
            binvox = binvox_rw.read_as_3d_array(binvox_file)
            if "mean" not in binvox_path:
                binvoxes_data.append(binvox.data)
            else:
                mean_binvox = binvox.data
    binvoxes_data = np.stack(binvoxes_data).astype("float32")

    # compute variances between binvoxes and mean binvox
    temp = np.zeros((40, 40, 40))
    for i in range(len(binvoxes)-1):
        temp += np.power(binvoxes_data[i, :, :, :] - mean_binvox, 2)  # np.abs(1. arg)
    temp /= len(binvoxes)-1

    # save uncertain binvox to -binvox file
    uncertain_binvox = np.logical_and(temp == np.max(temp), mean_binvox) #temp == np.max(temp)  # np.logical_and(temp == np.max(temp), mean_binvox)
    mean_binvox_with_uncertainty = temp[np.nonzero(mean_binvox)]
    model = binvox_rw.Voxels(uncertain_binvox, uncertain_binvox.shape, [0, 0, 0], 0, 'xyz')
    with open(os.path.join(binvoxes_path, "uncertain_binvox.binvox"), "w") as out_file:
        binvox_rw.write(model, out_file)

    # save uncertain and mean binvox to .npy for better handling
    np.save(os.path.join(binvoxes_path, 'mean_binvox.npy'), mean_binvox)
    np.save(os.path.join(binvoxes_path, 'uncertain_binvox.npy'), uncertain_binvox)
    np.save(os.path.join(binvoxes_path, 'mean_binvox_with_uncertainty.npy'), mean_binvox_with_uncertainty)


def is_feasible(pose, collision_avoidance, ik):
    """
    Computes inverse kinematics to check if the position is valid for the robot
    @param pose: At which pose to compute the IK
    @type pose: 1x3 list of floats or Pose
    @param collision_avoidance: Whether to use collision avoidance
    @type collision_avoidance: bool
    @param ik: inverse kinematics solver
    @type ik: InverseKinematics()
    @return: List of found joint angles
    @rtype: 1x7 list of floats
    """
    ps = PoseStamped()
    ps.header.stamp = rospy.Time.now()
    ps.header.frame_id = "base_link"
    if isinstance(pose, list):
        ps.pose.position.x = pose[0]
        ps.pose.position.y = pose[1]
        ps.pose.position.z = pose[2]
        ps.pose.orientation.x = pose[3]
        ps.pose.orientation.y = pose[4]
        ps.pose.orientation.z = pose[5]
        ps.pose.orientation.w = pose[6]
    elif isinstance(pose, Pose):
        ps.pose = pose
    if not rospy.get_param("/printed_finger", False):
        result = ik.getIK("arm", "tool_frame", ps, collision_avoidance, attempts=2)
    else:
        result = ik.getIK("arm", "finger_link", ps, collision_avoidance, attempts=2)
    return result


def choose_voxel(uncertain_binvox_path, ik, distance=20, direction_method="KNN", selected=None):
    """
    Choses which voxel to take, when there is more than one with the same weight
    @param uncertain_binvox_path: path to the folder uncertain voxels
    @type uncertain_binvox_path: str
    @param ik: Instance of inverse kinematics solver
    @type ik: kinova_mujoco.kinematics_interface.InverseKinematics
    @param distance: how far is the starting point from the impact point
    @type distance: float
    @param direction_method: Which method to use to compute direction vector, KNN or voxels
    @type direction_method: string
    @param selected: list with already explored positions
    @type selected: list
    @return: Starting position, impact position, quaternion computed from angle-axis
    @rtype: list of float 1x3, list of Pose 1xn, list of float 1x4
    """
    uncertain_binvox = np.load(os.path.join(uncertain_binvox_path, "uncertain_binvox.npy"))
    mesh_path = uncertain_binvox_path.replace("binvoxes", "meshes")


    # Find all uncertain points and cluster them
    nz = np.transpose(np.nonzero(uncertain_binvox))

    cluster = MeanShift(bandwidth=2).fit(nz)
    counts = np.bincount(cluster.labels_)  # number of points in each cluster

    mesh = trimesh.load(glob.glob(os.path.join(mesh_path, "*mean.ply"))[0])
    tree = kd(mesh.triangles_center)
    for centroid_id in range(len(counts)):
        centroid = cluster.cluster_centers_[centroid_id]

        # Weight more the centroids 'on back' of the object
        counts[centroid_id] += (40-centroid[0]+1)

        # Weight by reversed value of standard variation of neighbor triangles
        centroid_metres, _, _, _ = find_center_of_mesh(mesh_path, uncertain_binvox_path, copy.deepcopy(centroid), False)
        [_, i] = tree.query(centroid_metres, k=10)
        normals = mesh.face_normals[i]

        angle_sum = 0
        for a, b in itertools.combinations(np.arange(0, normals.shape[0]), 2):
            vecA = normals[a]
            vecB = normals[b]
            angle_sum += np.arctan2(np.linalg.norm(np.cross(vecA, vecB)), np.dot(vecA, vecB))
        if angle_sum != 0:
            counts[centroid_id] /= angle_sum

    rospy.loginfo("Preproccessing of voxels completed")
    # While there is any untested centroid
    while sum(1 for _ in counts if _ == -1) != len(counts):
        biggest_cluster = np.argmax(counts)  # take the biggest cluster = with the most points
        centroid = cluster.cluster_centers_[biggest_cluster]
        if len(selected) == 0 or np.all(np.linalg.norm(centroid-selected, axis=1) > 0.05):
            for camera_rotation in range(2):
                if camera_rotation == 0:
                    move_to_pose, impact_pose, direction, angle_axis, impact_point = compute_impact_position(uncertain_binvox_path, mesh_path, centroid, distance, direction_method)
                else:
                    move_to_pose, impact_pose, direction, angle_axis, impact_point = compute_impact_position(
                        uncertain_binvox_path, mesh_path, centroid, distance, direction_method, [direction, angle_axis,
                                                                                                 impact_point])
                result_angles = is_feasible(move_to_pose, True, ik)
                if result_angles:  # Try to compute inverse kinematics for the point
                    rospy.loginfo("Impact pose:\n "+str(impact_pose[-10])+"\nfound!")
                    if selected.shape[0] == 0:
                        selected = np.append(selected, centroid)
                        selected = np.expand_dims(selected, 0)
                    else:
                        selected = np.vstack((selected, centroid))
                    return move_to_pose, impact_pose, direction, selected, result_angles
                elif not result_angles and camera_rotation == 1:
                    counts[biggest_cluster] = -1
                    if selected.shape[0] == 0:
                        selected = np.append(selected, centroid)
                        selected = np.expand_dims(selected, 0)
                    else:
                        selected = np.vstack((selected, centroid))
        else:
            counts[biggest_cluster] = -1

    rospy.logerr("No feasible solution found!")
    return None, None, None, None, None


def find_center_of_mesh(mesh_path, binvoxes_path, centroid, save_resolution = True):
    """
    From mesh and selected centroid, compute the centroid position in world
    @param mesh_path: Path to 'meshes/object' folder
    @type mesh_path: str
    @param binvoxes_path: Path to 'binvoxes/object' folder
    @type binvoxes_path: str
    @param centroid: Centroid of selected cluster
    @type centroid: list, 3x1
    @return: centroid in metres, centroid of metres, mesh, resolution
    @rtype: 1x3 list of float, 1x3 list of float, trimesh(), 1x3 list of floats
    """
    centroid = np.array(centroid)
    # load mean binvox and mean mesh
    mean_binvox = np.load(os.path.join(binvoxes_path, "mean_binvox.npy"))
    mean_mesh = glob.glob(os.path.join(mesh_path, "*mean.ply"))[0]
    mesh = trimesh.load(mean_mesh)

    # compute mesh bbox coordinates
    bbox_coords = trimesh.bounds.corners(mesh.bounding_box.bounds)

    # get max index of occupied voxels in all axes
    nnz = np.nonzero(mean_binvox)
    x_max = np.max(nnz[0])+1
    y_max = np.max(nnz[1])+1
    z_max = np.max(nnz[2])+1

    # get voxel resolution
    x_res = np.abs(bbox_coords[0, 0]-bbox_coords[1, 0])/x_max
    y_res = np.abs(bbox_coords[0, 1]-bbox_coords[2, 1])/y_max
    z_res = np.abs(bbox_coords[0, 2]-bbox_coords[4, 2])/z_max

    resolution = [x_res, y_res, z_res]
    if save_resolution:
        np.save(os.path.join(binvoxes_path, "resolution.npy"), resolution)

    # compute mean (center of mass) of the mesh, with origin in (0,0,0)
    vertices = copy.deepcopy(mesh.vertices)

    # get min bbox and shift object to (0,0,0)
    mins = np.min(vertices, axis=0)
    vertices += (-1)*mins
    mesh_center = np.mean(vertices, axis=0)

    # transform it into voxel coords
    binvox_center = np.array(mesh_center/resolution)


    # Origin of the object is not in 0,0,0 but in centre of mass -> deduct it from centroid
    centroid_metres = (centroid-binvox_center)*resolution
    # add back shift and mean to get into original position
    centroid_metres = centroid_metres+mins+mesh_center

    return centroid_metres, binvox_center, mesh, resolution


def compute_direction_vector(mesh, impact_point,  neighbor_size=5, method="KNN", resolution=None, center=None):
    """
    Computes direction vector to impact
    @param mesh: Mesh object or path to mean binvox
    @type mesh: trimesh() object or str
    @param impact_point: impact point in metres of voxels
    @type impact_point: 1x3 list of floats
    @param neighbor_size: Number of neigbors from KNN to take or distance if voxels are used
    @type neighbor_size: int
    @param method: KNN or voxels, which method to use
    @type method: string
    @param resolution: Voxel to metres resolution
    @type resolution: 1x3 list of floats
    @param center: Center of the binvox
    @type center: 1x3 list of floats
    @return: Direction to impact,
    @rtype: 1x3 list of floats
    """
    # Use KN-Neighbors to find 'k' nearest triangles to the impact point and compute mean normal in the point
    if method == "KNN":
        tree = kd(mesh.triangles_center)
        [_, i] = tree.query(impact_point, k=neighbor_size)
        direction = mesh.face_normals[i]
        direction = np.mean(direction, axis=0)
    elif method == "voxels":
        mean_binvox = np.load(os.path.join(mesh, "mean_binvox.npy"))
        nnz = np.nonzero(mean_binvox)
        impact_point = np.round(impact_point-1)
        indexes = np.transpose([nnz[0], nnz[1], nnz[2]])

        #tree = kd(indexes)
        #neighbors = indexes[tree.query_ball_point(impact_point, neighbor_size)]

        dist = np.sqrt((impact_point[0]-nnz[0])**2 + (impact_point[1]-nnz[1])**2 + (impact_point[2]-nnz[2])**2)
        neighbors = indexes[dist <= neighbor_size, :]
        direction = np.sum(neighbors-center, axis=0)*resolution
        direction = direction/np.linalg.norm(direction)

    return direction


def compute_impact_position(binvoxes_path, mesh_path, centroid, distance=20, direction_method="KNN", rotate_up=None):
    """
    Computes impact point on mesh, direction of movement and starting point of the movement
    @param binvoxes_path: Path to 'binvoxes/object' folder
    @type binvoxes_path: str
    @param mesh_path: Path to 'meshes/object' folder
    @type mesh_path: str
    @param centroid: Position of impact point in voxel coordinates
    @type centroid: list, 3x1
    @param distance: How far is the starting point from impact point
    @type distance: float
    @param direction_method: Which method to use to compute direction vector, KNN or voxels
    @type direction_method: string
    @param rotate_up: list with information for rotating the last joint for 180 degs
    @type rotate_up: list
    @return: Starting position, impact position, quaternion computed from angle-axis
    @rtype: list of float 1x3, list of Pose 1xn, list of float 1x4
    """
    # Distance coefficient for direction vector
    distance_coef = 100. / distance
    if rotate_up is None:
        centroid_metres, binvox_center, mesh, resolution = find_center_of_mesh(mesh_path, binvoxes_path, centroid)

        # compute real impact point
        impact_point = centroid_metres

        if direction_method == "KNN":
            direction = compute_direction_vector(mesh, impact_point, 10, "KNN")
        elif direction_method == "voxels":
            direction = compute_direction_vector(binvoxes_path, centroid, 5, "voxels", resolution, binvox_center)

        # compute angle-axis representation
        vecA = [0, 0, 1]
        vecB = -direction
        rotAngle = np.arctan2(np.linalg.norm(np.cross(vecA, vecB)), np.dot(vecA, vecB))
        rotAxis = np.cross(vecA, vecB)
        rotAxis /= np.linalg.norm(rotAxis)
        angle_axis = ts.quaternion_about_axis(rotAngle, rotAxis)

        camera_up = ts.quaternion_about_axis(np.pi, -direction)
        angle_axis = ts.quaternion_multiply(camera_up, angle_axis)
    else:  # Rotate the last link with 180degs
        impact_point = rotate_up[2]
        direction = rotate_up[0]
        angle_axis = rotate_up[1]
        if rospy.get_param("/printed_finger", False):
            camera_up = ts.quaternion_about_axis(np.pi, -direction)
            angle_axis = ts.quaternion_multiply(camera_up, angle_axis)
        else:
            #camera_up = ts.quaternion_about_axis(np.pi/2, -direction)
            #angle_axis = ts.quaternion_multiply(camera_up, angle_axis)
            angle_axis = angle_axis


    # change frame of the impact point
    if not rospy.get_param("/printed_finger", False):
        impact_point, finger_to_tool = change_point_finger(impact_point, angle_axis, False)
    else:
        finger_to_tool = [0, 0, 0]

    # Pose 10cm from impact pose following normal
    move_to_pose = Pose()
    move_to_pose.position = Point(*impact_point+direction/distance_coef)
    move_to_pose.orientation = Quaternion(*angle_axis)

    # Impact pose
    impact_pose = Pose()
    impact_pose.position = Point(*impact_point)
    impact_pose.orientation = Quaternion(*angle_axis)

    # show arrow in RVIZ
    #show_arrow(impact_point-finger_to_tool, direction/distance_coef, angle_axis)

    rospy.set_param("/shape_completion/impact_point", str(list(impact_point-finger_to_tool)))
    rospy.set_param("/shape_completion/direction", str(list(direction/distance_coef)))
    rospy.set_param("/shape_completion/angle_axis", str(list(angle_axis)))

    impact_waypoints = []
    vec = np.array([move_to_pose.position.x, move_to_pose.position.y, move_to_pose.position.z])
    for step in np.arange(10, 0.5, -0.1):
        tmp = copy.deepcopy(move_to_pose)
        tmp.position = Point(*(vec-direction/(distance_coef*step)))
        impact_waypoints.append(tmp)

    return move_to_pose, impact_waypoints, direction, angle_axis, impact_point


def change_point_finger(point, quat, inverse=False):
    """
    Change impact point into tool_frame. Or into finger_link if inverse = True
    @param point: Impact point position
    @type point: list, 3x1
    @param quat: Quaternion representing impact direction
    @type quat: list, 4x1
    @param inverse: If to go from tool to finger
    @type inverse: bool
    @return: Point in right frame, transformation between the two frames
    @rtype: 1x3 list of float, 1x3 list of floats
    """

    if not inverse:
        (finger_to_tool, rotation) = get_transformation('/tool_frame', '/finger_link')
    else:
        (finger_to_tool, rotation) = get_transformation('/finger_link', '/tool_frame')

    rotation_to_base = ts.quaternion_matrix(quat)

    finger_to_tool = np.matmul(rotation_to_base, np.hstack((finger_to_tool, 1)))[:3]

    point += finger_to_tool
    return point, finger_to_tool


def terminate_thread(thread):
    """Terminates a python thread from another thread.

    :param thread: a threading.Thread instance
    """
    if not thread.isAlive():
        return

    proc = Popen("rosservice call /kinova_mujoco/reset '{}'", stdout=PIPE, shell=True)
    proc.wait()
    rospy.sleep(3)
    exc = ctypes.py_object(SystemExit)
    res = ctypes.pythonapi.PyThreadState_SetAsyncExc(
        ctypes.c_long(thread.ident), exc)
    if res == 0:
        raise ValueError("nonexistent thread id")
    elif res > 1:
        # """if it returns a number greater than one, you're in trouble,
        # and you should call it again with exc=NULL to revert the effect"""
        ctypes.pythonapi.PyThreadState_SetAsyncExc(thread.ident, None)
        raise SystemError("PyThreadState_SetAsyncExc failed")


def make_movement(pose, MoveGroup, cartesian=False, joints=False, collisions=True, wait=False, real_setup=False):
    """
    Runs proper movement based on arguments
    @param pose: Pose where to move
    @type pose: geometric_msgs.msg.Pose or list of Poses
    @param MoveGroup: MoveGroup python interface instance
    @type MoveGroup: kinova_mujoco.python_interface.MoveGroupPythonInterface
    @param cartesian: If to use linear movement
    @type cartesian: int/bool
    @param joints: whether to use joint angles movement
    @type joints: bool
    @param collisions: Whether to check for collisions
    @type collisions: bool
    @param wait: Whether to wait for completion
    @type wait: bool
    @param real_setup: Whether real setup is used
    @type real_setup: bool
    @return:
    @rtype:
    """
    if joints:
        MoveGroup.go_to_joint_position(pose)
    elif cartesian:
        plan, frac = MoveGroup.plan_cartesian_path(pose, collisions)
        if frac < 0.8:
            return False
        if real_setup:
            # Parametrize the path to contain 10ms steps between checkpoints
            # Needed specifically for Kinova
            rospy.wait_for_service("parametrize_cartesian_path")
            try:
                parametrize_cartesian_path_service = rospy.ServiceProxy('parametrize_cartesian_path', parametrize_cartesian_path)
                res = parametrize_cartesian_path_service(plan, 0.05)
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
            plan = res.trajectory_out
            plan.joint_trajectory.points.pop()
            MoveGroup.stop_robot()
        MoveGroup.execute_plan(plan, wait)
        return True
    else:
        MoveGroup.go_to_pose(pose)


def change_collision_matrix(planningScenePublisher, add=True, printed_finger=True):
    """
    Changes collision matrix to allow collision of finger with the objects
    @param planningScenePublisher:
    @type planningScenePublisher:
    @param add: whether to add or remove collision
    @type add: bool
    @return:
    @rtype:
    """
    rospy.wait_for_service('/get_planning_scene', 10.0)
    get_planning_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
    request = PlanningSceneComponents(components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)
    response = get_planning_scene(request)
    acm = response.scene.allowed_collision_matrix

    object_id = acm.entry_names.index('object')
    object_visual_id = acm.entry_names.index('object_visual')

    if printed_finger:
        names = ['finger_link', 'finger_holder']
    else:
        names = ["right_inner_finger_pad", "left_inner_finger_pad"]
    ids = []
    for _ in names:
        ids.append(acm.entry_names.index(_))

    for link_id in ids:
        acm.entry_values[link_id].enabled[object_visual_id] = add
        acm.entry_values[link_id].enabled[object_id] = add

        acm.entry_values[object_visual_id].enabled[link_id] = add
        acm.entry_values[object_id].enabled[link_id] = add

    planning_scene_diff = PlanningScene(
                is_diff=True,
                allowed_collision_matrix=acm)
    planningScenePublisher.publish(planning_scene_diff)
    rospy.sleep(1.0)


def detect_collision(detection_type, threshold, drift=None, joints_idxs=None, topic="/joint_states"):
    """
    Handler for collision detection of multiple types
    @param detection_type: What detection to use: cusum, threshold, openrave_threshold, openrave_cusum
    @type detection_type: str
    @param threshold: threshold for robot stopping, based on detection_type
    @type threshold: float
    @param drift: Drift for cusum method
    @type drift: float
    @param joints_idxs: list with indexes of joint into joint angles message
    @type joints_idxs: list
    @param topic: topic from which to read the joint angles
    @type topic: string
    @return:
    @rtype:
    """

    if detection_type == "cusum":  # Cusum detection from internal torques
        class_handle = CUSUM(threshold, drift, joints_idxs)
    elif detection_type == "threshold":  # Detection from external torques
        class_handle = THRESHOLD(threshold, joints_idxs)

    start_time = rospy.Time.now().to_time()
    class_handle.trajectory_event_publisher = rospy.Publisher('/trajectory_execution_event', String, queue_size=10)
    class_handle.sub = rospy.Subscriber(topic, JointState, class_handle)
    while class_handle.impact_pose is None:  # While contact not occurred
        if rospy.Time.now().to_time() - start_time > 35:
            class_handle.sub.unregister()
            break

    return class_handle.impact_pose


def add_to_PC(pose, pcd_path, r_, rep=0, shape="square"):
    """
    Function to add new information to the point cloud
    @param pose: Pose of the robot
    @type pose: Pose
    @param pcd_path: path to the .pcd file
    @type pcd_path: string
    @param r_: radius of the circle/square to be added
    @type r_: fload
    @param rep: To which rep the touch belongs
    @type rep: int
    @param shape: Which shape to add -- circle, square
    @type shape: string
    @return:
    @rtype:
    """
    # Load point cloud
    pc = pcl.load(pcd_path)
    points_ = pc.to_array()

    # Rotate the points
    R = np.array([[0, 0, 1, 0], [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]])
    points = np.matmul(R, np.hstack((points_[:, :3], np.ones((points_.shape[0], 1)))).T)[:3, :].T
    # Add the color back
    points_[:, :3] = points

    voxel_center = np.array([pose.position.x, pose.position.y, pose.position.z])

    # Direction vector computation
    direction = np.matmul(ts.quaternion_matrix([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]), [0, 0, 1, 1])[:3]  # Rotate Z-axis vector to point in direction of gripper
    direction /= np.linalg.norm(direction)  # normalize

    # perpendicular plane is null of the direction vector
    null_space = scipy.linalg.null_space(direction.reshape(1, 3))
    u = null_space[:, 0]
    v = null_space[:, 1]

    noise_ratio = 0.5
    if shape == "circle":
        # circle = Origin + cos(theta)*u + sin(theta)*v, where theta goes from 0 to 2pi
        num_new_points = int((points.shape[0]*0.075)/25)
        print("New points " + str(num_new_points))
        th = np.linspace(0, 2*np.pi, num_new_points).reshape(num_new_points, 1)
        for r in np.linspace(0, r_, 25):
            u_ = u*r
            v_ = v*r
            if r == 0:
                touch = voxel_center + np.multiply(np.cos(th), u_) + np.multiply(np.sin(th), v_) + np.random.uniform(-noise_ratio/1e3, noise_ratio/1e3, (th.shape[0], voxel_center.shape[0]))
            else:
                touch = np.vstack((touch, voxel_center + np.multiply(np.cos(th), u_) + np.multiply(np.sin(th), v_) + np.random.uniform(-noise_ratio/1e3, noise_ratio/1e3, (th.shape[0], voxel_center.shape[0]))))

    elif shape == "square":
        num_points = 30
        r = r_
        xx, yy = np.meshgrid(np.linspace(-r/2, r/2, num_points), np.linspace(-r/2, r/2, num_points))
        touch = xx.reshape(-1, 1)*u.reshape(1, 3)+yy.reshape(-1, 1)*v.reshape(1, 3)+voxel_center + np.random.uniform(-noise_ratio/1e3, noise_ratio/1e3, (num_points**2, voxel_center.shape[0]))

    points_ = np.vstack((points_, touch))

    # save new normals
    normals = np.load(pcd_path.replace(".pcd", "_normals.npy"))
    normals_ = (-1)*direction + np.random.uniform(-noise_ratio/2, noise_ratio/2, (touch.shape[0], voxel_center.shape[0]))
    normals_ = np.matmul(R.T, np.hstack((normals_[:, :3], np.ones((normals_.shape[0], 1)))).T)[:3, :].T
    normals = np.vstack((normals, normals_))

    if not os.path.exists(os.path.join(os.path.dirname(pcd_path), "rep"+str(rep))):
        os.makedirs(os.path.join(os.path.dirname(pcd_path), "rep"+str(rep)))

    files = glob.glob(os.path.join(os.path.dirname(pcd_path), "*"))
    for file in files:
        if not os.path.isdir(file):
            cmd = "cp "+file+" "+os.path.join(os.path.dirname(pcd_path), "rep"+str(rep), file.split("/")[-1])
            proc = Popen(cmd, stdout=PIPE, shell=True)
            proc.wait()

    np.save(pcd_path.replace(".pcd", "_normals.npy"), normals)
    np.save(pcd_path.replace(".pcd", ".npy"), points_)

    # Show in RVIZ
    rospy.set_param("/shape_completion/new_pc_path", pcd_path.replace(".pcd", ".npy"))

    points = np.matmul(R.T, np.hstack((points_[:, :3], np.ones((points_.shape[0], 1)))).T)[:3, :].T
    points_[:, :3] = points

    pc = pcl.PointCloud(points_.astype("float32"))
    pcl.save(pc, pcd_path)


def get_transformation(what, where):
    """
    Help util to get transformation
    @param what: For what frame to obtain the transformation
    @type what: string
    @param where: In which frame to express
    @type where: string
    @return:
    @rtype:
    """
    tf_listener = tf.TransformListener()
    while True:
        try:
            translation, rotation = tf_listener.lookupTransform(where, what, rospy.Time(0))
            return np.array(translation), np.array(rotation)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

