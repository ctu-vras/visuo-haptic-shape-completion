"""
Function to create train samples

@author Lukas Rustler
"""
import open3d
import numpy as np
import os
import multiprocessing
import argparse
import json
import glob
import time
import trimesh
from trimesh.sample import sample_surface
from scipy.io import savemat


def preprocess(path, obj, out_path="../data/npy"):

    if not os.path.exists(path):
        return -1

    if not os.path.exists(os.path.join(out_path, obj)):
        os.makedirs(os.path.join(out_path, obj))

    mesh = trimesh.load(path)

    samples = sample_surface(mesh, 100000)
    normals = mesh.face_normals[samples[1]]
    ma = np.max(samples[0], 0)
    mi = np.min(samples[0], 0)
    diff = np.max(np.abs(ma - mi))
    scale = 2 / diff

    # Rotation the we wanted to train on
    rotations = [[[0], ['z']], [[90], ['z']], [[180], ['z']], [[270], ['z']], [[90], ['y']], [[270], ['y']],
                 [[90], ['x']], [[270], ['x']], [[90, 90], ['y', 'z']], [[90, 270], ['y', 'z']],
                 [[90, 180], ['x', 'y']], [[270, 180], ['x', 'y']], [[90, 180], ['y', 'x']], [[270, 180], ['y', 'x']],
                 [[90, 270], ["x", "y"]], [[270, 270], ["x", "y"]]]

    for angle, axis in rotations:
        pc = open3d.geometry.PointCloud()
        pc.points = open3d.utility.Vector3dVector(samples[0])
        pc.normals = open3d.utility.Vector3dVector(normals)

        pc.translate([0, 0, 0], False)
        pc.scale(scale, [0, 0, 0])
        R = np.eye(3)
        name = ""
        for rot_id in range(len(angle)):
            angle_ = np.deg2rad(angle[rot_id])
            axis_ = axis[rot_id]
            name += str(angle[rot_id])+"_"+str(axis_)+"_"
            if axis_ == "x":
                R_ = [[np.cos(angle_), -np.sin(angle_), 0], [np.sin(angle_), np.cos(angle_), 0], [0, 0, 1]]
            elif axis_ == "y":
                R_ = [[1, 0, 0], [0, np.cos(angle_), -np.sin(angle_)], [0, np.sin(angle_), np.cos(angle_)]]
            elif axis_ == "z":
                R_ = [[np.cos(angle_), 0, np.sin(angle_)], [0, 1, 0], [-np.sin(angle_), 0, np.cos(angle_)]]
            R = np.matmul(R_, R)
        name = name[:-1]

        pc.rotate(R)

        #pc.normals = open3d.utility.Vector3dVector(np.asarray(pc.normals))

        #open3d.visualization.draw_geometries([pc], point_show_normal=True)

        data = np.hstack([np.asarray(pc.points), np.array(pc.normals)])

        np.save(os.path.join(out_path, obj, name+".npy"), data)


def multiproc_helper(core_id, data, indices, args):
    for idx, obj in enumerate(indices):
        if idx % 10 == 0:
            print("Process "+str(core_id)+" completed " + str(idx) + " objects from " + str(len(indices)))
        path = os.path.join(args.path, obj+".ply")
        preprocess(path, obj)


if __name__ == "__main__":
    arg_parser = argparse.ArgumentParser(
        description="Preprocessing of uncompleted shapes from YCB and Grasp dataset for IGR"
    )
    arg_parser.add_argument(
        "--path-to-data",
        "-p",
        dest="path",
        required=True,
        help="The directory which includes grasp_database and ycb directories",
    )

    arg_parser.add_argument(
        "--split",
        "-s",
        dest="split_file",
        required=True,
        help=".json file with train/test split"
    )

    arg_parser.add_argument(
        "--cores",
        "-c",
        dest="cores",
        required=False,
        default=1,
        help="Number of cores used."
    )


    args = arg_parser.parse_args()

    args.cores = int(args.cores)

    splits = json.load(open(args.split_file, "r"))
    #splits = {"data": ["avocado_poisson_000"]}
    #ycb = [folder.split("/")[-1] for folder in glob.glob("/media/rustli/data1/ground_truth_meshes/ycb/*")]
    for key in splits.keys():
        temp = splits[key]
        indices = np.arange(len(temp))
        np.random.shuffle(indices)
        jobs = []
        for core_id in range(args.cores):
            if core_id != args.cores - 1:
                indices_ = np.array(list(temp))[
                    indices[core_id * (len(indices) // args.cores):(core_id + 1) * (len(indices) // args.cores)]]
            else:
                indices_ = np.array(list(temp))[indices[core_id * (len(indices) // args.cores):]]
            jobs.append(multiprocessing.Process(target=multiproc_helper, args=(core_id, temp, indices_, args)))
            jobs[-1].start()
        for job in jobs:
            job.join()
    #if args.train:
    #    f = open('scales.json', "w")
    #    json.dump(scale_dict, f)
    #    f.close()
