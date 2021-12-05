#!/usr/bin/env python2
"""
Utils for comparison with baselines

@author Lukas Rustler
"""
import open3d
import numpy as np
import os
import sys
import signal
import trimesh


def signal_handler(signal, frame):
    """
    Handler for killing the program
    @param signal: signal type
    @type signal: signal
    @param frame:
    @type frame:
    @return:
    @rtype:
    """
    sys.exit(0)


def reconstruct_mesh(path, out_path, method, debug=False, use_saved_normals=True):
    """
    Reconstructs mesh from PCD with different baseline methods.
    @param path: Path to pointcloud
    @type path: String
    @param out_path: Path to output mesh
    @type out_path: String
    @param method: baseline method to be used
    @type method: String
    @param debug: Whether to show debug visualiazations
    @type debug: bool
    @param use_saved_normals: Whether to load saved normals from file
    @type use_saved_normals: bool
    @return:
    @rtype:
    """
    # load point cloud
    pcd = open3d.io.read_point_cloud(path)
    if not os.path.exists(os.path.dirname(out_path)):
        os.makedirs(os.path.dirname(out_path))

    # Estimate new normals
    if not use_saved_normals:
        pcd.estimate_normals()
        if sys.version_info[0] > 3:
            pcd.orient_normals_consistent_tangent_plane(10)
    # Use saved normals from touches
    elif use_saved_normals or not pcd.has_normals():
        try:
            normals = np.load(path.replace(".pcd", "_normals.npy"))
            pcd.normals = open3d.utility.Vector3dVector(np.asarray(normals))
        except:  # in case of IO error
            pcd.estimate_normals()
    if debug:
        if sys.version_info[0] < 3:  # python 2.7 cant show normals
            open3d.visualization.draw_geometries([pcd])
        else:
            open3d.visualization.draw_geometries([pcd], point_show_normal=True)

    if method == "BPA":  # Ball pivoting algorithm
        # Compute radius from mean distances between neighbors
        distances = pcd.compute_nearest_neighbor_distance()
        avg_dist = np.mean(distances)
        radius = 3 * avg_dist
        # Down sample to have 7000 point at max to improve speed
        while np.asarray(pcd.points).shape[0] > 7000:
            pcd = pcd.uniform_down_sample(2)
        # Compute mesh
        try:
            mesh = open3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd, open3d.utility.DoubleVector([radius, radius * 2]))
        except:
            return -1
    elif method == "poisson":  # Poisson surface reconstruction
        # Compute mesh
        mesh = open3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=8, width=0, scale=1.1, linear_fit=False)[0]
        # Remove things outside bounds of point cloud
        bbox = pcd.get_axis_aligned_bounding_box()
        mesh = mesh.crop(bbox)
    elif method == "hull":  # Convex hull
        mesh, _ = pcd.compute_convex_hull()
    elif method == "alpha":  # Alpha shapes
        # Compute convex hull
        tetra_mesh, pt_map = open3d.geometry.TetraMesh.create_from_point_cloud(pcd)
        # Compute mesh
        try:
            mesh = open3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, 1.0, tetra_mesh, pt_map)
        except:
            return -1
    elif method == "gpis":
        # Create point cloud with normals to be opened with GPIS in matlab
        pcd_with_normals = pcd.uniform_down_sample(3)
        while np.asarray(pcd_with_normals.points).shape[0] > 350:
            pcd_with_normals = pcd_with_normals.uniform_down_sample(2)
            print(np.asarray(pcd_with_normals.points).shape[0])

        # move to center and scale to normal distribution
        center = pcd_with_normals.get_center()
        pcd_with_normals.translate([0, 0, 0], False)
        ma = np.max(np.asarray(pcd_with_normals.points), 0)
        mi = np.min(np.asarray(pcd_with_normals.points), 0)

        diff = np.max(np.abs(ma - mi))
        scale = 2 / diff

        points = np.asarray(pcd_with_normals.points)
        points = scale*points
        pcd_with_normals.points = open3d.utility.Vector3dVector(points)
        #pcd_with_normals.scale(scale, [0, 0, 0])

        open3d.io.write_point_cloud(path.replace(".pcd", "_with_normals.pcd"), pcd_with_normals)

        if debug:
            if sys.version_info[0] < 3:  # python 2.7 cant show normals
                open3d.visualization.draw_geometries([pcd_with_normals])
            else:
                open3d.visualization.draw_geometries([pcd_with_normals], point_show_normal=True)

        # run matlab engine and get results
        import matlab.engine
        eng = matlab.engine.start_matlab("-noFigureWindows")
        file_dir = os.path.dirname(os.path.abspath(__file__))
        gpis_path = os.path.join(file_dir, "../../../GPIS")
        eng.cd(gpis_path)
        try:
            vertices, faces, _ = eng.gpis_runner(path.replace(".pcd", "_with_normals.pcd"), nargout=3)
        except:
            eng.quit()
            return -1
        eng.quit()

        # Create mesh from vertices and faces
        vert = open3d.open3d.utility.Vector3dVector(np.asarray(vertices))
        fac = open3d.open3d.utility.Vector3iVector(np.asarray(faces)-1)
        mesh = open3d.geometry.TriangleMesh(vert, fac)

        # if not os.path.exists(os.path.dirname(out_path)):
        #     os.makedirs(os.path.dirname(out_path))
        # open3d.io.write_triangle_mesh(out_path.replace("mean", "mean_uncrop"), mesh)

        # bbox = pcd_with_normals.get_axis_aligned_bounding_box()
        # mb = bbox.min_bound
        # bbox.min_bound = [mb[0]-0.3*mb[0], mb[1]-0.3*mb[1], mb[2]-0.3*mb[2]]
        # mb = bbox.max_bound
        # bbox.max_bound = [mb[0]+0.3*mb[0], mb[1]+0.3*mb[1], mb[2]+0.3*mb[2]]
        # mesh = mesh.crop(bbox)

    # Mesh fixing methods
    mesh.orient_triangles()
    mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_vertices()
    mesh.remove_duplicated_triangles()
    mesh.remove_non_manifold_edges()
    mesh.paint_uniform_color(np.array([1,1,1]))

    if debug:
        open3d.visualization.draw_geometries([mesh])

    # Save mesh
    open3d.io.write_triangle_mesh(out_path, mesh)

    if os.path.isfile(out_path):
        mesh = trimesh.load(out_path)
        # Fix inversion problems
        trimesh.repair.fix_inversion(mesh)
        # Scale GPIS back to  be in the same format as other baselines
        if method == "gpis":
            mesh.apply_scale(1/scale)
            translation = np.eye(4)
            translation[:3, 3] = center.T
            mesh.apply_transform(translation)
        mesh.export(out_path)


def reconstruct_from_logs(log_path, fit=True, skip=False, debug=False):
    """
    Main method to reconstruct all meshes in one log file
    @param log_path: path to the .log file
    @type log_path: String
    @param fit: Whether to use fit the mesh on to GT with ICP
    @type fit: bool
    @param skip: Whether to skip meshes when evaluating and creating graphs
    @type skip: bool
    @param debug: Whether to show debug visualizations
    @type debug: bool
    @return:
    @rtype:
    """
    # import here because some clashes between libraries
    from shape_completion.evaluation_utils import graphs_from_logs

    # laod log
    with open(log_path, "r") as log_file:
        log = log_file.read().splitlines()[2:]

    # read important variables
    stamps = ["-".join(_.split(" ")[-2:]).replace(":", "-").replace(".", "-") for _ in log]
    objs = [_.split(" ")[3].split(",")[0].split("(")[1] for _ in log]
    rec_nums = [int(_.split(" ")[3].split(",")[1]) for _ in log]
    touches = [int(_.split(" ")[3].split(",")[2]) for _ in log]

    # all available baselines
    algos = ["BPA", "poisson", "hull", "alpha", "gpis"]
    for algo in algos:
        print(algo)
        for obj_idx, obj in enumerate(objs):
            print(obj)
            # Prepare path
            pcd_temp = os.path.join(log_path.split("/logs")[0], "pcd", obj + "_" + stamps[obj_idx])
            mesh_temp = pcd_temp.replace("pcd", "meshes") + "_" + algo
            for rep in range(0, rec_nums[obj_idx]*touches[obj_idx]+1, touches[obj_idx]):
                print(rep)
                if os.path.exists(os.path.join(pcd_temp, "rep" + str(rep))):
                    pcd_path = os.path.join(pcd_temp, "rep"+str(rep), obj+".pcd")
                    mesh_path = os.path.join(mesh_temp, "rep"+str(rep // touches[obj_idx]), obj+"_mean.ply")
                    if not os.path.isfile(os.path.join(os.path.dirname(mesh_path), "rotated.ply")):
                        # Call given reconstruction
                        reconstruct_mesh(pcd_path, mesh_path, algo, debug=debug)
                        # Create mesh for comparison
                        mesh = open3d.io.read_triangle_mesh(mesh_path)

                        R = np.array([[0, 0, 1, 0], [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]])
                        obj_origin = np.load(os.path.join(pcd_temp.replace("pcd", "npy"), "rep"+str(rep), "file_origin.npy"))
                        obj_origin = np.matmul(R.T, np.hstack((obj_origin, 1)).T)[:3]

                        mesh.translate(-obj_origin, True)
                        open3d.io.write_triangle_mesh(os.path.join(os.path.dirname(mesh_path), "rotated.ply"), mesh)

        # Save new log with new paths
        with open(log_path, "r") as log_file:
            log = log_file.read().splitlines()
        with open(log_path.replace(".log", "_"+algo+".log"), "w") as log_file:
            for line_idx, line in enumerate(log):
                if line_idx>1:
                    log_file.write(line+"_"+algo+"\n")
                else:
                    log_file.write(line+"\n")

        # Call graph and .npz file evaluation
        file_dir = os.path.dirname(os.path.abspath(__file__))
        meshes_path = os.path.join(file_dir, "../../data/meshes")
        gt_path = os.path.join(file_dir, "../../../kinova_mujoco/GT_meshes")
        graphs_from_logs(meshes_path, gt_path, log_path.replace(".log", "_"+algo+".log"), sample_mesh=True, skip=skip, fit=fit)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    file_dir = os.path.dirname(os.path.abspath(__file__))

    if len(sys.argv) > 1:
        func = sys.argv[1]
        if len(sys.argv) > 2:
            log = sys.argv[2]
        else:
            log = None
    if log is None:
        reconstruct_from_logs(sys.argv[1], skip=False, fit=False)

    if func == "reconstruct_from_logs":
        log_path = os.path.join(file_dir, "../../data", "logs", log)
        reconstruct_from_logs(log_path, fit=False, skip=True, debug=False)
