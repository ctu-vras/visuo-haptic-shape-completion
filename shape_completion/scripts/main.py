#!/usr/bin/env python2
"""
The main logic of the pipeline

@author Lukas Rustler
"""
import datetime
import glob
from shape_completion.save_pcl import save_pcl
from shape_completion.utils import rotate_and_scale, create_binvox, compare_binvoxes, compute_impact_position,\
    choose_voxel, make_movement, change_point_finger, add_to_PC, detect_collision, change_collision_matrix
from kinova_mujoco.python_interface import MoveGroupPythonInterface
from kinova_mujoco import kinematics_interface
from subprocess import call, PIPE, Popen
import os
import rospy
import sys
import json
import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion
import argparse
from moveit_msgs.msg import PlanningScene
from sensor_msgs.msg import JointState
from threading import Timer


def log_subprocess_output(proc):
    """
    Change prints to rospy.loginfo() messages
    @param proc: subsprocess.Popen object
    @type proc: subsprocess.Popen object
    @return:
    @rtype:
    """
    while True:
        line = proc.stdout.readline()
        if not line:
            break
        if line.rstrip():
            rospy.loginfo(line.rstrip())


def prepare_parser():
    arg_parser = argparse.ArgumentParser(
        description="Main script for shape completion experiments"
    )
    arg_parser.add_argument(
        "--number_of_reconstruction",
        "-r",
        dest="recs_num",
        required=False,
        default=5,
        help="How many reconstruction to done, not including the first one"
    )
    arg_parser.add_argument(
        "--number_of_touches",
        "-t",
        dest="reps",
        required=False,
        default=1,
        help="How many touches to do between reconstructions"
    )

    arg_parser.add_argument(
        "--detection_type",
        "-d",
        dest="detection_type",
        required=False,
        default="cusum",
        help="What collision detection algo to use: cusum, threshold"
    )

    arg_parser.add_argument(
        "--net",
        "-n",
        dest="net",
        required=False,
        default="IGR",
        help="Which net to use: IGR"
    )

    args = arg_parser.parse_args()
    return int(args.reps), int(args.recs_num)+1, args.detection_type, args.net


if __name__ == "__main__":
    reps, recs_num, detection_type, net = prepare_parser()

    #kill_nodes = True
    # absolute path to this file
    file_dir = os.path.dirname(os.path.abspath(__file__))

    rospy.init_node("shape_completion_main", anonymous=True)
    t = rospy.Time.now().to_time()
    # Get params
    file_name = rospy.get_param("/object_name", "out")
    file_origin = rospy.get_param("/object_origin", [0.6, 0, 0.2])
    printed_finger = rospy.get_param("/printed_finger", False)
    real_setup = rospy.get_param("/real_setup", False)
    save_cameras = rospy.get_param("/save_cameras", False)
    joint_states_topic = "/joint_states"
    msg = rospy.wait_for_message("/joint_states", JointState)
    names = msg.name

    # Find  at which indexes are the joint of the robot (It can change depending on names of the other joints)
    joints_idxs = []
    for idx, name in enumerate(names):
        if "joint_" in name:  # Change this if your joints are not named joint_X
            joints_idxs.append(idx)
        if "finger_joint" == name:
            finger_idx = idx

    if isinstance(file_origin, str):  # origin should be an array
        file_origin = eval(file_origin)

    cur_datetime = str(datetime.datetime.now()).replace("." , "-").replace(" ", "-").replace(":", "-")

    # run rosbag to save information about joint_states
    bag_path = os.path.join(file_dir, "../data/rosbags", file_name, cur_datetime)
    rospy.loginfo("Starting rosbag recording into: "+str(bag_path))

    if not os.path.exists(os.path.dirname(bag_path)):
        os.makedirs(os.path.dirname(bag_path))
    topics_to_save = ["kinova_mujoco/joint_states", "joint_states", "joint_states_custom", "/trajectory_execution_event"]
    if save_cameras:
        topics_to_save += ["camera_0_pc", "camera_1_pc", "camera_pc"]
    command = "rosbag record -O " + bag_path + " " + " ".join(topics_to_save) + " __name:=my_bag"

    Popen(command, stdout=PIPE, shell=True)

    # Move group python interface init for arm
    rospy.loginfo("Creating MoveGroup python interface instance for arm")
    MoveGroupArm = MoveGroupPythonInterface("arm")

    if printed_finger:
        MoveGroupArm.group.set_end_effector_link("finger_link")

    rospy.loginfo("Planning with: "+MoveGroupArm.group.get_end_effector_link())

    # Move group python interface init for gripper
    rospy.loginfo("Creating MoveGroup python interface instance for gripper")
    MoveGroupGripper = MoveGroupPythonInterface("gripper")

    if not real_setup:
        # Close the gripper
        rospy.loginfo("Restarting the simulation if necessary")
        while True:
            gripper_cur_joints = MoveGroupGripper.group.get_current_joint_values()
            # The simulation sometimes glitches and run robot with bad gripper pose -> restart simulation if it happens
            gripper_joints = [0, 0, 0, 0, 0, 0] if printed_finger else [0.80, -0.79, 0.82, 0.83, 0.80, -0.80]

            if np.linalg.norm(np.array(gripper_cur_joints)-np.array(gripper_joints)) > 0.1:
                proc = Popen("rosservice call /kinova_mujoco/reset '{}'", stdout=PIPE, shell=True)
                proc.wait()
                rospy.sleep(3)
            else:
                break

    # Inverse kinematics interface
    rospy.loginfo("Creating Inverse Kinematics instance")
    inverse_kinematics = kinematics_interface.InverseKinematics()

    # Forward kinematics interface
    rospy.loginfo("Creating Forward Kinematics instance")
    forward_kinematics = kinematics_interface.ForwardKinematics()

    # Planning scene publisher
    rospy.loginfo("Creating planning scene publisher")
    planning_scene = rospy.Publisher('planning_scene', PlanningScene, queue_size=10)

    # .pcd file path
    pcd_path = os.path.join(file_dir, "../data/pcd", file_name, file_name+".pcd")
    # save point cloud to .pcd file to right location
    save_pcl(pcd_path, C=False)

    for rec_id in range(recs_num):

        # .npy path
        npy_path = os.path.join(pcd_path.split("pcd")[0], "npy", file_name)


        # prepare command and call preprocessing
        R = np.array([[0, 0, 1, 0], [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]])
        translation = np.matmul(R.T, np.hstack((file_origin, 1)).T)[:3]
        # Add this line if you use virtual environment and have 'shape_rec_pX' as alias in ~./bashrc
        # /bin/bash -i -c shape_rec_p3 &&
        cmd = "python3 "+os.path.join(file_dir, "../../IGR/code/preprocess/preprocess_one_file.py")+" -f "+pcd_path+\
              " -o "+npy_path+" -na " + ("save" if rec_id == 0 else "load")

        rospy.loginfo("Running PCL preprocessing and saving file to: "+npy_path)
        returncode = call(cmd, shell=True)
        if returncode:  # Kill program after error
            sys.exit(1)

        np.save(os.path.join(npy_path, "file_origin.npy"), file_origin)

        # Create json split file for IGR
        split = {"npy": [file_name]}
        with open(os.path.join(file_dir, "../../IGR/code/splits/shape_completion/test_real.json"), "w") as split_file:
            json.dump(split, split_file)


        # Prepare shape completion command
        rospy.loginfo("Starting shape completion using IGR")
        mesh_path = os.path.join(file_dir, "../data/meshes", file_name)
        # Add this line if you use virtual environment and have 'shape_rec_pX' as alias in ~./bashrc
        # /bin/bash -i -c shape_rec_p3 &&
        cmd = "python3 "+os.path.join(file_dir, "../../IGR/code/shapespace/eval.py")+\
              " --gpu 0 --exps-dir exps --conf shape_completion_setup.conf" \
              " --split shape_completion/test_real.json --exp-name no_ycb -c" \
              " 3500 --repeats 1 -u -r 40 --save_directory "+mesh_path+" --loop_rep "+str(rec_id)

        # Run command and turn prints into rospy.loginfo()
        proc = Popen(cmd, stdout=PIPE, shell=True)
        log_subprocess_output(proc)
        proc.wait()  # wait to properly end the subprocess (otherwise return code is None)
        if proc.returncode:  # Kill program after error
            sys.exit(1)

        # Rotate and scale the object back
        mesh_path = os.path.join(file_dir, "../data/meshes", file_name)
        rospy.loginfo("Rotating and scaling back the mean mesh")
        center, bbox_center, bbox = rotate_and_scale(mesh_path, rec_id, sim=not real_setup)
        bbox[0] += 0.02
        # Show reconstruction in RVIZ
        rospy.loginfo("Showing reconstruction in RVIZ")
        rospy.set_param("/shape_completion/rec_mesh_path", "package://shape_completion/data/meshes/"+file_name+"/rep"+str(rec_id)+"/rec.stl")
        rospy.set_param("/shape_completion/rec_mesh_center", str(list(center)))

        # Create bounding box around the point cloud to not bump into the object
        if real_setup:
            clear_iters = 0
            while clear_iters < 10 and (MoveGroupArm.planning_scene.getKnownCollisionObjects() or MoveGroupArm.planning_scene.getKnownAttachedObjects()):
                MoveGroupArm.planning_scene.clear()
                clear_iters += 1

            MoveGroupArm.planning_scene.addBox("object_bbox", bbox[0], bbox[1], bbox[2], center[0]+bbox_center[0], center[1]+bbox_center[1], center[2]+bbox_center[2], frame_id="base_link")
            MoveGroupArm.planning_scene.waitForSync()
            MoveGroupArm.planning_scene.setColor("object_bbox", 1, 1, 1, 0.1)
            MoveGroupArm.planning_scene.sendColors()
            MoveGroupArm.planning_scene.waitForSync()

        # create binvox files
        rospy.loginfo("Turning meshes into voxels")
        create_binvox(mesh_path)

        # compare the object voxel based
        rospy.loginfo("Comparing the voxels")
        binvoxes_path = mesh_path.replace("meshes", "binvoxes")
        compare_binvoxes(binvoxes_path)
        selected = np.array([])
        if rec_id != recs_num - 1:
            rep = 0
            while rep < reps:
                rospy.loginfo("Choosing the impact voxel")
                start_pose, impact_pose, direction, selected, result_angles = choose_voxel(binvoxes_path, inverse_kinematics, 5, "KNN", selected)

                if start_pose != None:
                    # Move to the start position
                    rospy.loginfo("Moving to start position at: "+str(rospy.Time.now().to_time()-t))
                    if not real_setup:
                        make_movement(result_angles, MoveGroupArm, joints=True)
                    else:
                        make_movement(start_pose, MoveGroupArm)

                    rospy.loginfo("Turning off the collision detection for object and finger")
                    if not real_setup:
                        change_collision_matrix(planning_scene, True, printed_finger=printed_finger)
                    else:
                        clear_iters = 0
                        while clear_iters < 10 and (MoveGroupArm.planning_scene.getKnownCollisionObjects() or MoveGroupArm.planning_scene.getKnownAttachedObjects()):
                            MoveGroupArm.planning_scene.clear()
                            clear_iters += 1

                    rospy.sleep(2)
                    # Move to impact position
                    rospy.loginfo("Moving to impact position at: "+str(rospy.Time.now().to_time()-t))
                    possible_move = make_movement(impact_pose, MoveGroupArm, cartesian=True, real_setup=real_setup)
                    rospy.sleep(1)
                    if possible_move:
                        if detection_type == "cusum":
                            joint_angles = detect_collision("cusum", 1.25, drift=0.25, joints_idxs=joints_idxs, topic=joint_states_topic)
                        elif detection_type == "threshold":
                            joint_angles = detect_collision("threshold", 5, joints_idxs=joints_idxs, topic=joint_states_topic)

                        rospy.loginfo("Stopping the robot at: "+str(rospy.Time.now().to_time()-t))
                        MoveGroupArm.stop_robot()

                    if possible_move and joint_angles is not None:
                        # Compute pose when collision detected, from forward kinematics and find position in finger frame
                        if not printed_finger:
                            pose = forward_kinematics.getFK("tool_frame", ["joint_"+str(i) for i in range(1, 8)], joint_angles[joints_idxs]).pose_stamped[0].pose
                            pose_finger, _ = change_point_finger([pose.position.x, pose.position.y, pose.position.z],
                                                       [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w], True)
                            pose_finger = Pose(Point(*pose_finger), pose.orientation)
                        else:
                            pose_finger = forward_kinematics.getFK("finger_link", ["joint_"+str(i) for i in range(1, 8)], joint_angles[joints_idxs]).pose_stamped[0].pose

                        rospy.loginfo("Collision detected at: \n"+str(pose_finger))

                        # Add new information into pointcloud
                        rospy.loginfo("Adding new points into the point cloud")
                        #7.5/1e3
                        add_to_PC(pose_finger, pcd_path, 6/1e3, rec_id*reps+rep, "circle")
                        rep += 1

                    rospy.loginfo("Moving from collision")
                    if not real_setup:
                        make_movement(impact_pose[0], MoveGroupArm, cartesian=True, collisions=False, wait=True)  # get from collision
                    else:
                        timer_handle = Timer(30.0, MoveGroupArm.stop_robot)
                        make_movement(impact_pose[0], MoveGroupArm, cartesian=True, collisions=False, wait=True, real_setup=real_setup)  # get from collision
                        timer_handle.cancel()
                    rospy.loginfo("Turning on the collision detection for object and finger")
                    if not real_setup:
                        change_collision_matrix(planning_scene, False, printed_finger=printed_finger)
                    else:
                        clear_iters = 0
                        while clear_iters < 10 and (MoveGroupArm.planning_scene.getKnownCollisionObjects() or MoveGroupArm.planning_scene.getKnownAttachedObjects()):
                            MoveGroupArm.planning_scene.clear()
                            clear_iters += 1

                        MoveGroupArm.planning_scene.addBox("object_bbox", bbox[0], bbox[1], bbox[2],
                                                           center[0] + bbox_center[0], center[1] + bbox_center[1],
                                                           center[2] + bbox_center[2], frame_id="base_link")
                        MoveGroupArm.planning_scene.waitForSync()
                        MoveGroupArm.planning_scene.setColor("object_bbox", 1, 1, 1, 0.1)
                        MoveGroupArm.planning_scene.sendColors()
                        MoveGroupArm.planning_scene.waitForSync()
                else:
                    break

        # Move old files into corresponding folder
        rospy.loginfo("Moving files into rep"+str(rec_id)+" folder")
        for path in [mesh_path, binvoxes_path, npy_path]:
            files = glob.glob(os.path.join(path, "*"))
            if not os.path.exists(os.path.join(path, "rep"+str(rec_id))):
                os.makedirs(os.path.join(path, "rep"+str(rec_id)))
            for file in files:
                if not os.path.isdir(file):
                    cmd = "mv "+file+" "+os.path.join(path, "rep"+str(rec_id), file.split("/")[-1])
                    proc = Popen(cmd, stdout=PIPE, shell=True)
                    proc.wait()

    rospy.sleep(1)

    files = glob.glob(os.path.join(os.path.dirname(pcd_path), "*"))
    for file in files:
        if not os.path.isdir(file):
            _, f_name = os.path.split(file)
            if not os.path.exists(os.path.join(os.path.dirname(file), "rep"+str(rec_id*reps))):
                os.makedirs(os.path.join(os.path.dirname(file), "rep"+str(rec_id*reps)))
            proc = Popen("cp "+file+" "+os.path.join(os.path.dirname(file), "rep"+str(rec_id*reps), f_name), shell=True)


    # Close the bag recording
    rospy.loginfo("Killing bag recording")
    proc = Popen("rosnode kill /my_bag", shell=True)
    proc.wait()

    #if kill_nodes:
    #    for name in ["arrow_shower", "new_pc_node", "reconstruction_shower"]:
    #        rospy.loginfo("Killing "+name)
    #        proc = Popen("rosnode kill "+name, shell=True)
    #        proc.wait()
    rospy.loginfo("Total time: "+str(rospy.Time.now().to_time()-t))
