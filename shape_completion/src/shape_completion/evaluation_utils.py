#!/usr/bin/env python2
"""
Utils for evaluation of the results

@author Lukas Rustler
"""
import sys
from shape_completion.distance_utils import jaccard_similarity, chamfer_distance, robust_icp
import os
import numpy as np
import trimesh
from subprocess import Popen, PIPE
import binvox_rw
import matplotlib.pyplot as plt
import glob
import pcl
from subprocess import call
import json
from shape_completion.utils import rotate_and_scale, compare_binvoxes, create_binvox


def compute_similarity(reps, mesh_path, gt_path, voxel_res=40, sample_mesh=True, fit=False):
    """
    Function to compute Jaccard simlarity and Chamfer distance. Optionally call ICP to fit the meshes
    @param reps: Number of repetitions in given folder
    @type reps: int
    @param mesh_path: path to folder with meshes
    @type mesh_path: string
    @param gt_path: path to GT mesh
    @type gt_path: string
    @param voxel_res: resolution of the voxel grid
    @type voxel_res: int
    @param sample_mesh: Whether to sample the meshes
    @type sample_mesh: bool
    @param fit: Whether to fit the meshes to gt
    @type fit: bool
    @return:
    @rtype:
    """
    jaccard = np.zeros(reps+1)
    chamfer = np.zeros(reps+1)
    for rep in range(reps+1):
        temp_path = os.path.join(mesh_path, "rep"+str(rep), "rotated.ply")
        if os.path.isfile(temp_path):
            rec_mesh = trimesh.load_mesh(temp_path)
            gt_mesh = trimesh.load_mesh(gt_path)
            if fit:
                out_path = temp_path.split("rotated.ply")[0]
                if not sample_mesh:
                    # Use vertices of the mesh
                    res = trimesh.exchange.ply.export_ply(rec_mesh, encoding='ascii')
                    f = open(temp_path, "w")
                    f.writelines(res)
                    f.close()

                else:
                    # Sample to meshes
                    rec_samples = trimesh.sample.sample_surface(rec_mesh, 10000)[0]
                    pc = pcl.PointCloud(rec_samples.astype("float32"))
                    temp_path = temp_path.replace(".ply", "_sampled.ply")
                    pcl.save(pc, temp_path)

                    gt_samples = trimesh.sample.sample_surface(gt_mesh, 10000)[0]
                    pc = pcl.PointCloud(gt_samples.astype("float32"))
                    gt_path = gt_path.replace(".ply", "_sampled.ply")
                    pcl.save(pc, gt_path)

                # Call FRICP
                method = 3
                robust_icp(temp_path, gt_path, out_path, method)

                if sample_mesh:
                    gt_path = gt_path.replace("_sampled.ply", ".ply")
                    temp_path = temp_path.replace("_sampled.ply", ".ply")

                # Transform the mesh with ICP resutls
                R = np.loadtxt(temp_path.replace("rotated.ply", "m"+str(method)+"trans.txt"))
                rec_mesh.apply_transform(R)
                temp_path = temp_path.replace("rotated", "icp")

                rec_mesh.export(temp_path)

            if not sample_mesh:
                rec_samples = rec_mesh.vertices
                gt_samples = gt_mesh.vertices
            else:
                rec_samples = trimesh.sample.sample_surface(rec_mesh, 100000)[0]
                gt_samples = trimesh.sample.sample_surface(gt_mesh, 100000)[0]

            # Compute Chamfer distance
            chamfer[rep] = chamfer_distance(rec_samples, gt_samples)

            data = []
            compute = True
            for path in [gt_path, temp_path]:

                # Compute binvox
                cmd = "binvox -pb -d " + str(voxel_res) + " " + path
                proc = Popen(cmd, shell=True, stdout=PIPE)
                proc.wait()

                with open(path.replace(".ply", ".binvox"), 'r') as binvox_file:
                    try:
                        binvox = binvox_rw.read_as_3d_array(binvox_file)
                        data.append(binvox.data)
                    except:
                        print(path.replace(".ply", ".binvox"))
                        jaccard[rep] = np.nan
                        compute = False
                        break
                cmd = "rm " + path.replace(".ply", ".binvox")
                proc = Popen(cmd, shell=True, stdout=PIPE)
                proc.wait()
                np.save(path.replace(".ply", ".npy"), binvox.data)

            if compute:
                data = np.stack(data)
                # Compute Jaccard Similarity
                jaccard[rep] = jaccard_similarity(data[0, :, :, :], data[1, :, :, :])
        else:
            chamfer[rep] = np.nan
            jaccard[rep] = np.nan
    return chamfer*1e3, jaccard*100


def graphs_from_logs(path, gt_path, log_path, sample_mesh, fit=False, skip=False):
    """
    Main fuction to create graphs and also to save values in .npz file
    @param path: path to meshes
    @type path: string
    @param gt_path: path to GT meshes
    @type gt_path: string
    @param log_path: path to the .log file
    @type log_path: string
    @param sample_mesh: Whether to sample the meshes for ICP
    @type sample_mesh: bool
    @param fit: Whether to fit with ICP
    @type fit: bool
    @param skip: Whether to skip already processes files
    @type skip: bool
    @return:
    @rtype:
    """
    out_path = log_path.replace("logs", "figures").split(".log")[0]
    out_path_npz = os.path.join(os.path.dirname(log_path.replace(".log", ".npz")), "../offline_nn_vals", log_path.replace(".log", ".npz").split("/")[-1])
    if not os.path.exists(os.path.dirname(out_path_npz)):
        os.makedirs(os.path.dirname(out_path_npz))

    # Load log and read important values
    folders = glob.glob(path+"/*")
    with open(log_path, "r") as log_file:
        log = log_file.read().splitlines()[2:]

    reps = []
    for idx, line in enumerate(log[1:]):
        if int(line.split(".")[0]) == 1:
            reps.append(int(log[idx].split(".")[0]))
    reps.append(int(log[-1].split(".")[0]))

    rec_nums = [int(_.split(" ")[3].split(",")[1]) for _ in log]
    touches = [int(_.split(" ")[3].split(",")[2]) for _ in log]

    stamps = ["-".join(_.split(" ")[-2:]).replace(":", "-").replace(".", "-") for _ in log]

    # If we should skip and the output file already exists load the data
    if skip and os.path.isfile(out_path_npz):
        loaded_npz = np.load(out_path_npz, allow_pickle=True)
        jaccard_total_loaded = loaded_npz["jaccard_total"][()]
        chamfer_total_loaded = loaded_npz["chamfer_total"][()]
        chamfer_all_loaded = loaded_npz["chamfer_all"][()]
        jaccard_all_loaded = loaded_npz["jaccard_all"][()]
    else:
        skip = False

    jaccard_total = {}
    chamfer_total = {}
    chamfer_all = {}
    jaccard_all = {}

    for bin_id, bin_res in enumerate([40]):
        idx = 0
        for rep in reps:
            chamf = []
            jacc = []
            only_graphs = False
            if skip:
                idx_temp = idx
                for rep_id in range(rep):
                    if len(stamps) > idx_temp:
                        tmp_path = [_ for _ in folders if stamps[idx_temp] in _]
                        if len(tmp_path) > 0:
                            tmp_path = tmp_path[0]
                        else:
                            idx_temp += 1
                            continue
                        obj = tmp_path.split("/")[-1].split("_2021")[0]

                        idx_temp += 1
                    else:
                        break
                key = str(rec_nums[idx_temp - 1]) + "," + str(touches[idx_temp - 1]) + "," + str(rep_id + 1)
                # If object found on already processed values append them to the output
                if obj in jaccard_total_loaded.keys() and key in jaccard_total_loaded[obj].keys():
                    jaccard_total[obj] = jaccard_total_loaded[obj]
                    chamfer_total[obj] = chamfer_total_loaded[obj]
                    chamfer_all[obj] = chamfer_all_loaded[obj]
                    jaccard_all[obj] = jaccard_all_loaded[obj]
                    print(obj, key)
                    idx = idx_temp
                    only_graphs = True

            if not only_graphs:
                for rep_id in range(rep):
                        if len(stamps) > idx:
                            tmp_path = [_ for _ in folders if stamps[idx] in _]
                            if len(tmp_path) > 0:
                                tmp_path = tmp_path[0]
                            else:
                                idx += 1
                                continue
                            obj = tmp_path.split("/")[-1].split("_2021")[0]
                            print(obj)
                            # Compute similarities
                            chamfer, jaccard = compute_similarity(rec_nums[idx], tmp_path, os.path.join(gt_path, obj) + ".ply", voxel_res=bin_res, sample_mesh=sample_mesh, fit=fit)
                            chamf.append(chamfer)
                            jacc.append(jaccard)

                            idx += 1
                        else:
                            break
            else:
                chamf = chamfer_all[obj][key]
                jacc = jaccard_all[obj][key]
            # Create object wise graphs
            if not only_graphs and bin_id == 0:
                key = str(rec_nums[idx - 1]) + "," + str(touches[idx - 1]) + "," + str(rep_id + 1)
                if obj in jaccard_total.keys():
                    try:
                        jaccard_total[obj][key] = np.nanmax(jacc)
                        chamfer_total[obj][key] = np.nanmin(chamf)
                    except:
                        jaccard_total[obj][key] = np.nan
                        chamfer_total[obj][key] = np.nan
                    jaccard_all[obj][key] = jacc
                    chamfer_all[obj][key] = chamf
                else:
                    try:
                        jaccard_total[obj] = {key: np.nanmax(jacc)}
                        chamfer_total[obj] = {key: np.nanmin(chamf)}
                    except:
                        jaccard_total[obj] = {key: np.nan}
                        chamfer_total[obj] = {key: np.nan}
                    jaccard_all[obj] = {key: jacc}
                    chamfer_all[obj] = {key: chamf}

                np.savez(out_path_npz, chamfer_total=chamfer_total, jaccard_total=jaccard_total,
                         jaccard_all=jaccard_all, chamfer_all=chamfer_all)
            try:

                x = np.arange(0, chamf[0].shape[0])
                plt.figure()
                plt.plot(np.tile(x, (rep_id+1, 1)).T, np.transpose(jacc), '-o')#, colors[rep_id])
                plt.xlabel("Touches [-]")
                plt.ylabel("Jaccard similarity [%]")
                plt.grid()
                plt.legend(["Repetition " + str(i) for i in range(rep_id+1)])
                #plt.title(obj+" with "+str(touches[idx-1])+" touches and "+str(rec_nums[idx-1])+" reconstructions \n binvox resolution "+str(bin_res))
                tit = ""
                for _ in obj.split("_"):
                    try:
                        int(_)
                        continue
                    except:
                        tit += _[0].upper() + _[1:] + " "
                tit = tit[:-1]
                plt.title(tit)
                for folder in ["png", "eps"]:
                    if not os.path.exists(os.path.join(out_path, obj, folder)):
                        os.makedirs(os.path.join(out_path, obj, folder))
                plt.savefig(os.path.join(out_path, obj, "png", obj+"_"+str(rec_nums[idx-1])+"_"+str(touches[idx-1])+"_"+str(rep_id+1)+"_"+str(bin_res))+"_jaccard.png")
                plt.savefig(os.path.join(out_path, obj, "eps", obj+"_"+str(rec_nums[idx-1])+"_"+str(touches[idx-1])+"_"+str(rep_id+1)+"_"+str(bin_res))+"_jaccard.eps")
                plt.close()
                if bin_id == 0:

                    plt.figure()
                    plt.plot(np.tile(x, (rep_id+1,1)).T, np.transpose(chamf), '-o')#, colors[rep_id])
                    plt.xlabel("Touches [-]")
                    plt.ylabel("Chamfer distance [mm]")
                    plt.grid()
                    plt.legend(["Repetition " + str(i) for i in range(rep_id+1)])
                    #plt.title(obj+" with "+str(touches[idx-1])+" touches and "+str(rec_nums[idx-1])+" reconstructions")
                    plt.title(tit)
                    plt.savefig(os.path.join(out_path, obj, "png", obj+"_"+str(rec_nums[idx-1])+"_"+str(touches[idx-1])+"_"+str(rep_id+1))+"_chamfer.png")
                    plt.savefig(os.path.join(out_path, obj, "eps", obj+"_"+str(rec_nums[idx-1])+"_"+str(touches[idx-1])+"_"+str(rep_id+1))+"_chamfer.eps")
                    plt.close()

            except:
                continue
    # Create dataset wise graphs
    for obj in jaccard_total.keys():
        try:
            keys = np.array([key+" ("+str(int(key.split(",")[0])*int(key.split(",")[1]))+")" for key in jaccard_total[obj].keys()])
            values = np.array(jaccard_total[obj].values())
            idxs = sorted(np.arange(0, len(values)), key=lambda x: int(keys[x].split("(")[1].split(")")[0]))

            values = values[idxs]
            keys = keys[idxs]

            x = np.arange(0, len(keys))
            plt.figure()
            plt.bar(x, values)
            plt.xlabel("Experiment [-] (Total number of touches[-])")
            plt.xticks(x, keys)
            plt.ylabel("Jaccard similarity [%]")
            plt.grid()
            plt.savefig(os.path.join(out_path, obj, "png", "jaccard_total.png"))
            plt.savefig(os.path.join(out_path, obj, "eps", "jaccard_total.eps"))
            plt.close()

            keys = np.array([key+" ("+str(int(key.split(",")[0])*int(key.split(",")[1]))+")" for key in chamfer_total[obj].keys()])
            values = np.array(chamfer_total[obj].values())
            idxs = sorted(np.arange(0, len(values)), key=lambda x: int(keys[x].split("(")[1].split(")")[0]))

            values = values[idxs]
            keys = keys[idxs]

            x = np.arange(0, len(keys))
            plt.figure()
            plt.bar(x, values)
            plt.xlabel("Experiment [-] (Total number of touches[-])")
            plt.xticks(x, keys)
            plt.ylabel("Chamfer distance [mm]")
            plt.grid()
            plt.savefig(os.path.join(out_path, obj, "png", obj + "chamfer_total.png"))
            plt.savefig(os.path.join(out_path, obj, "eps", obj + "chamfer_total.eps"))
            plt.close()
        except:
            continue

    # Save .npz files
    out_path = os.path.join(os.path.dirname(log_path.replace(".log", ".npz")), "../offline_nn_vals", log_path.replace(".log", ".npz").split("/")[-1])
    if not os.path.exists(os.path.dirname(out_path)):
        os.makedirs(os.path.dirname(out_path))
    np.savez(out_path, chamfer_total=chamfer_total, jaccard_total=jaccard_total, jaccard_all=jaccard_all, chamfer_all=chamfer_all)


def offline_nn(log_path, path, settings, baseline=False, fit=True, sim=True, skip=False):
    """
    Function to batch reconstruct meshes offline.
    @param log_path: path to the .log file
    @type log_path: string
    @param path: path to the data folder
    @type path: string
    @param settings: settings of the reconstruction. key: name of the settings, value: ["name of the IGR experiment", "epoch number"]
    @type settings: dictionary
    @param baseline: Whether to reconstruct with baselines
    @type baseline: bool
    @param fit: whether to fit the object to GT
    @type fit: bool
    @param sim: Whether the data are from simulation
    @type sim: bool
    @param skip: Whether to skip already computed data
    @type skip: bool
    @return:
    @rtype:
    """

    # Load log file and read variables
    with open(log_path, "r") as log_file:
        log = log_file.read().splitlines()[2:]

    reps = []
    for idx, line in enumerate(log[1:]):
        if int(line.split(".")[0]) == 1:
            reps.append(int(log[idx].split(".")[0]))
    reps.append(int(log[-1].split(".")[0]))

    stamps = ["-".join(_.split(" ")[-2:]).replace(":", "-").replace(".", "-") for _ in log]
    objs = [_.split(" ")[3].split(",")[0].split("(")[1] for _ in log]
    rec_nums = [int(_.split(" ")[3].split(",")[1]) for _ in log]
    touches = [int(_.split(" ")[3].split(",")[2]) for _ in log]
    file_dir = os.path.dirname(os.path.abspath(__file__))

    for sett in sorted(settings.keys()):
        print(sett)
        for obj_idx, obj in enumerate(objs):
            print(obj, rec_nums[obj_idx], touches[obj_idx])
            pcd_temp = os.path.join(path, "pcd", obj + "_" + stamps[obj_idx])
            npy_temp = pcd_temp.replace("pcd", "npy")
            mesh_temp = pcd_temp.replace("pcd", "meshes")
            last_existed = -1

            # Run IGR for all repetitions in the folder
            for rep in range(0, rec_nums[obj_idx]*touches[obj_idx]+1, touches[obj_idx]):
                if os.path.isfile(os.path.join(mesh_temp + "_" + sett, "rep"+str(rep // touches[obj_idx]), "rotated.ply")):
                    continue
                if os.path.exists(os.path.join(pcd_temp, "rep"+str(rep))):
                    if last_existed != -1:
                        cmd = "cp -r " + os.path.join(pcd_temp, "rep"+str(last_existed)) + " " + os.path.join(pcd_temp, "rep"+str(rep-1))
                        proc = Popen(cmd, shell=True, stdout=PIPE)
                        proc.wait()

                        cmd = "cp -r " + os.path.join(mesh_temp + "_" + sett, "rep"+str(last_existed)) + " " + os.path.join(mesh_temp + "_" + sett, "rep"+str(rep-1))
                        proc = Popen(cmd, shell=True, stdout=PIPE)
                        proc.wait()

                        last_existed = -1
                    #/bin/bash -i -c shape_rec_p3 && 
                    cmd = "python3 " + os.path.join(file_dir, "../../../IGR/code/preprocess/preprocess_one_file.py")\
                          + " -f " + os.path.join(pcd_temp, "rep"+str(rep), obj+".pcd") + \
                          " -o " + os.path.join(npy_temp+"_"+sett, "rep"+str(rep // touches[obj_idx])) + " -na " + ("save" if "cameras" in sett else "load")

                    returncode = call(cmd, shell=True)

                    file_origin = np.load(os.path.join(npy_temp, "rep"+str(rep // touches[obj_idx]), "file_origin.npy"))
                    np.save(os.path.join(npy_temp+"_"+sett, "rep"+str(rep // touches[obj_idx]), "file_origin.npy"), file_origin)

                    split = {"npy": [os.path.join(obj+"_"+stamps[obj_idx]+"_"+sett, "rep"+str(rep // touches[obj_idx]))]}
                    with open(os.path.join(file_dir, "../../../IGR/code/splits/shape_completion/test.json"),
                              "w") as split_file:
                        json.dump(split, split_file)

                    if os.path.exists(os.path.join(mesh_temp + "_" + sett, "rep"+str(rep // touches[obj_idx]))):
                        cmd = "rm -r "+os.path.join(mesh_temp + "_" + sett, "rep"+str(rep // touches[obj_idx]))
                        proc = Popen(cmd, shell=True, stdout=PIPE)
                        proc.wait()
                    #/bin/bash -i -c shape_rec_p3 && 
                    cmd = "python3 " + os.path.join(file_dir, "../../../IGR/code/shapespace/eval.py") + \
                          " --gpu 0 --exps-dir exps --conf shape_completion_setup_offline.conf" \
                          " --split shape_completion/test.json --exp-name " + settings[sett][0] + " -c" \
                          " " + settings[sett][1] + " --repeats 1 -u -r 40 --save_directory "\
                          + os.path.join(mesh_temp + "_" + sett, "rep"+str(rep // touches[obj_idx])) + " --loop_rep " + str(rep // touches[obj_idx])+\
                          " --offline_rec"

                    proc = Popen(cmd, shell=True)
                    proc.wait()  # wait to properly end the subprocess (otherwise return code is None)

                    #try:
                    rotate_and_scale(os.path.join(mesh_temp+"_"+sett, "rep"+str(rep // touches[obj_idx])), -1, offline_rec=True, sim=sim)
                    #except:
                    #    continue
                    create_binvox(os.path.join(mesh_temp+"_"+sett, "rep"+str(rep // touches[obj_idx])))
                    compare_binvoxes(os.path.join(mesh_temp+"_"+sett, "rep"+str(rep // touches[obj_idx])).replace("meshes", "binvoxes"))
                else:
                    last_existed = rep-1 if last_existed == -1 else last_existed

        # Save new .log file with appropriate paths
        with open(log_path, "r") as log_file:
            log = log_file.read().splitlines()
        with open(log_path.replace(".log", "_"+sett+".log"), "w") as log_file:
            for line_idx, line in enumerate(log):
                if line_idx>1:
                    log_file.write(line+"_"+sett+"\n")
                else:
                    log_file.write(line+"\n")

        # Compute values for .npz files and create graphs
        meshes_path = os.path.join(file_dir, "../../data/meshes")
        gt_path = os.path.join(file_dir, "../../../kinova_mujoco/GT_meshes")
        graphs_from_logs(meshes_path, gt_path, log_path.replace(".log", "_"+sett+".log"), sample_mesh=True, fit=fit, skip=skip)

    if baseline:
        # Workaround because open3d doesnt like pcl
        file_dir = os.path.dirname(os.path.abspath(__file__))
        #/bin/bash -i -c shape_rec_p2 && 
        call("python2 "+os.path.join(file_dir, "baselines.py")+" "+log_path, shell=True)


def whole_exp_graph(values_path, sim=True):
    """
    Fnction to evaluate the whole experiment. Graphs over train/holdout datasets with different metrics.
    @param values_path: path to the .npz file
    @type values_path: string
    @param sim: whether the data are from simulation
    @type sim: bool
    @return:
    @rtype:
    """
    data = np.load(values_path, allow_pickle=True)
    # Known traning objects
    train_objs = []

    out_path = os.path.join(os.path.dirname(values_path), "../figures/", values_path.split(".npz")[0].split("/")[-1])
    for exp in ["5,1,3"]:
        print(exp)
        for folder in ["png", "eps"]:
            if not os.path.exists(os.path.join(out_path, folder, exp)):
                os.makedirs(os.path.join(out_path, folder, exp))
        object_sets = {"all": data["jaccard_all"][()].keys(), "train": [obj for obj in data["jaccard_all"][()].keys() if obj in train_objs],
                  "holdout": [obj for obj in data["jaccard_all"][()].keys() if obj not in train_objs]}
        for object_set in object_sets:
            values = {"chamfer": [], "jaccard": [], "uncertainty": [], "labels": [], "labels_concat": []}
            for obj in object_sets[object_set]:

                # Get the true number of repetitions (for more specific experiments only 1 was done)
                exists = False
                for rep_counter in ["3", "2", "1"]:
                    exp_local = exp[:-1]+rep_counter
                    if exp_local in data["jaccard_all"][()][obj].keys():
                        exists = True
                        break
                if not exists:
                    continue

                jac = np.array(data["jaccard_all"][()][obj][exp_local])
                cham = np.array(data["chamfer_all"][()][obj][exp_local])
                if cham.size == 0:
                    print(obj)
                    continue

                obj_type = "train" if obj in train_objs else "holdout"
                # fix data if something is missing
                for i in range(3):
                    if cham.shape[0] <= i:
                        cham = np.vstack((cham, np.nan * np.zeros(cham[i-1, :].shape)))
                        jac = np.vstack((jac, np.nan * np.zeros(jac[i-1, :].shape)))

                    values["chamfer"].append(cham[i, :])
                    values["jaccard"].append(jac[i, :])

                    values["labels"].append(obj+" rep "+str(i)+" ["+obj_type+"]")
                values["labels_concat"].append(obj+" ["+obj_type+"]")
            ylabels = {"chamfer": "Chamfer distance [mm]", "jaccard": "Jaccard similarity [%]"}
            yticks = {"jaccard": np.arange(0, 101, 10), "chamfer": np.arange(0, 26, 5)}

            for metric in ["chamfer", "jaccard"]:#ylabels.keys():
                # find how many repetitions were done
                max_shape = 0
                for val_idx, value in enumerate(values[metric]):
                    if value.shape[0] > max_shape:
                        max_shape = value.shape[0]
                for val_idx, value in enumerate(values[metric]):
                    while values[metric][val_idx].shape[0] != max_shape:
                        values[metric][val_idx] = np.hstack((values[metric][val_idx], np.nan))

                # graph with all experiments
                cm = plt.get_cmap('gist_rainbow')
                fig = plt.figure()
                ax = fig.add_subplot(111)
                ax.set_prop_cycle(color=[cm(1. * i / len(values[metric])) for i in range(len(values[metric]))])
                ax.plot(np.arange(0, max_shape), np.transpose(values[metric]), label='placeholder')

                for val in np.transpose(values[metric]).T:
                    ax.scatter(np.arange(0, max_shape), val, label='placeholder')
                ax.set_xticks(np.arange(0, max_shape))
                ax.set_xlim(ax.get_xlim()[0]-1, ax.get_xlim()[-1]+1)

                plt.grid()
                plt.xlabel("Touches [-]")
                plt.ylabel(ylabels[metric])
                handles, _ = ax.get_legend_handles_labels()
                lgd = ax.legend(handles, values["labels"], loc='upper left', bbox_to_anchor=(1, 1))
                #plt.yticks(yticks[metric])
                plt.savefig(os.path.join(out_path, "eps", exp, metric+"_"+object_set+".eps"), bbox_extra_artists=(lgd,), bbox_inches='tight')
                plt.savefig(os.path.join(out_path, "png", exp, metric+"_"+object_set+".png"), bbox_extra_artists=(lgd,), bbox_inches='tight')
                plt.close()

                if max_shape != 0:
                    plt.figure()
                    plt.plot(np.arange(0, max_shape), np.nanmean(values[metric], 0))
                    plt.scatter(np.arange(0, max_shape), np.nanmean(values[metric], 0))
                    plt.xticks(np.arange(0, max_shape))
                    plt.xlim(plt.xlim()[0]-1, plt.xlim()[-1]+1)

                    plt.grid()
                    plt.xlabel("Touches [-]")
                    plt.ylabel(ylabels[metric])
                    #plt.yticks(yticks[metric])
                    plt.savefig(os.path.join(out_path, "eps", exp, metric+"_"+object_set+"_mean_all.eps"))
                    plt.savefig(os.path.join(out_path, "png", exp, metric+"_"+object_set+"_mean_all.png"))
                    plt.close()

                # Evaluation with different metrics
                for sub_metric in ["mean", "median", "mean_outliers"]:
                    tmp = []
                    for i in range(0, len(values[metric]), 3):
                        if sub_metric == "mean":
                            tmp.append(np.nanmean(np.array(values[metric])[i:i + 3, :], 0))

                        elif sub_metric == "median":
                            tmp.append(np.nanmedian(np.array(values[metric])[i:i + 3, :], 0))
                        elif sub_metric == "mean_outliers":
                            # We try to the repetition which was far from the other two (both better and worse)
                            values_temp = np.array(values["jaccard"])[i:i + 3, :]
                            mean_outliers_indexes = np.array([0, 1, 2])
                            if not np.any(np.all(np.isnan(values_temp), 1)):
                                max_diff = [-1, None]
                                for pairs in [[0, (1, 2)], [1, (0, 2)], [2, (0, 1)]]:
                                    max_diff_temp = 0
                                    bigger_counter = np.array([0, 0])
                                    last_values = [-2, 0] if max_shape < 10 else [-10, 0]
                                    if max_shape < 10 and sim:
                                        last_values = [-3, -1]
                                    for j in range(values_temp.shape[1]+last_values[0], values_temp.shape[1]+last_values[1]):
                                        diffs = np.array([np.abs(values_temp[pairs[0], j]-values_temp[pairs[1][0], j]), np.abs(values_temp[pairs[0], j]-values_temp[pairs[1][1], j])])
                                        max_diff_temp += np.sum(diffs)
                                        bigger_counter = bigger_counter * (diffs > 5).astype(float) + (diffs > 5).astype(float)
                                        if np.all(bigger_counter == 2):
                                            max_diff[0] = max_diff_temp
                                            max_diff[1] = np.array(pairs[1])
                                mean_outliers_indexes = max_diff[1] if max_diff[0] != -1 else mean_outliers_indexes
                            tmp.append(np.nanmean(np.array(values[metric])[mean_outliers_indexes + i, :], 0))
                    fig = plt.figure()
                    ax = fig.add_subplot(111)
                    ax.set_prop_cycle(color=[cm(1. * i / len(tmp)) for i in range(len(tmp))])
                    ax.plot(np.arange(0, max_shape), np.transpose(tmp), label='placeholder')

                    for val in np.transpose(tmp).T:
                        ax.scatter(np.arange(0, max_shape), val, label='placeholder')
                    ax.set_xticks(np.arange(0, max_shape))
                    ax.set_xlim(ax.get_xlim()[0]-1, ax.get_xlim()[-1]+1)

                    plt.grid()
                    plt.xlabel("Touches [-]")
                    plt.ylabel(ylabels[metric])
                    plt.title(sub_metric)
                    handles, _ = ax.get_legend_handles_labels()
                    lgd = ax.legend(handles, values["labels_concat"], loc='upper left', bbox_to_anchor=(1, 1))
                    #plt.yticks(yticks[metric])
                    plt.savefig(os.path.join(out_path, "eps", exp, metric + "_" + object_set + "_" + sub_metric + ".eps"), bbox_extra_artists=(lgd,), bbox_inches='tight')
                    plt.savefig(os.path.join(out_path, "png", exp, metric + "_" + object_set + "_" + sub_metric + ".png"), bbox_extra_artists=(lgd,), bbox_inches='tight')
                    plt.close()

                    if max_shape != 0:
                        plt.figure()
                        plt.plot(np.arange(0, max_shape), np.nanmean(tmp, 0))
                        plt.scatter(np.arange(0, max_shape), np.nanmean(tmp, 0))
                        plt.xticks(np.arange(0, max_shape))
                        plt.xlim(plt.xlim()[0]-1, plt.xlim()[-1]+1)
                        plt.grid()
                        plt.xlabel("Touches [-]")
                        plt.ylabel(ylabels[metric])
                        #plt.yticks(yticks[metric])
                        plt.savefig(os.path.join(out_path, "eps", exp, metric+"_"+object_set + "_" + sub_metric + "_mean_all.eps"))
                        plt.savefig(os.path.join(out_path, "png", exp, metric+"_"+object_set + "_" + sub_metric + "_mean_all.png"))
                        plt.close()


if __name__ == "__main__":
    """Prepared calls for easier use of this files"""
    file_dir = os.path.dirname(os.path.abspath(__file__))
    if len(sys.argv) > 1:
        func = sys.argv[1]
        if len(sys.argv) > 2:
            log = sys.argv[2]

    if func == "graphs_from_logs":
        meshes_path = os.path.join(file_dir, "../../data/meshes")
        gt_path = os.path.join(file_dir, "../../../kinova_mujoco/GT_meshes")
        log_path = os.path.join(file_dir, "../../data", "logs", log)
        graphs_from_logs(meshes_path, gt_path, log_path, sample_mesh=True, fit=False, skip=False)

    if func == "whole_exp_graph":
        path = os.path.join(file_dir, "../../data/offline_nn_vals", log)
        whole_exp_graph(path, sim=True)

    if func == "offline_nn":
        path = os.path.join(file_dir, "../../data")
        gt_path = os.path.join(file_dir, "../../../kinova_mujoco/GT_meshes")
        log_path = os.path.join(file_dir, "../../data", "logs", log)
        offline_nn(log_path, path, {"noycb_3500": ["no_ycb", "3500"]}, baseline=True, fit=False, sim=True, skip=False)

    if func == "run_all":
        for log in ["name.npz"]:
            path = os.path.join(file_dir, "../../data/offline_nn_vals", log)
            print(log)
            whole_exp_graph(path, sim=False if "simulated" not in log else True)
