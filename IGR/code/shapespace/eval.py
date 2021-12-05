import argparse
import os
import sys
project_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
sys.path.append(project_dir)
os.chdir(project_dir)
import json
import utils.general as utils
import torch
from pyhocon import ConfigFactory
import utils.plots as plt
from shapespace.latent_optimizer import optimize_latent
import time
from utils.plots import get_grid_uniform
from skimage import measure
import numpy as np
import trimesh
from pymeshfix import _meshfix


def evaluate(network, experiment_directory, conf, checkpoint, split_file, epoch, resolution, uniform_grid, repeats, save_directory=None, loop_rep=0, offline_rec=False):

    if not save_directory:
        my_path = os.path.join(experiment_directory, 'evaluation', str(checkpoint))
        utils.mkdir_ifnotexists(os.path.join(experiment_directory, 'evaluation'))
        utils.mkdir_ifnotexists(my_path)
    else:
        my_path = os.path.join(save_directory)
        utils.mkdir_ifnotexists(os.path.join(experiment_directory))
        utils.mkdir_ifnotexists(my_path)
        epoch = ''
        if not offline_rec:
            latent_path = os.path.join(my_path, "rep"+str(loop_rep-1), "latent.pt") if loop_rep != 0 else None
        else:
            latent_path = os.path.join(my_path.split("rep")[0], "rep"+str(loop_rep-1), "latent.pt") if loop_rep != 0 else None

    with open(split_file, "r") as f:
        split = json.load(f)

    ds = utils.get_class(conf.get_string('train.dataset'))(split=split, dataset_path=conf.get_string('train.dataset_path'), with_normals=True)

    total_files = len(ds)
    print("IGR found {0} files".format(total_files))
    counter = 0
    dataloader = torch.utils.data.DataLoader(ds, batch_size=1, shuffle=True, num_workers=1, drop_last=False, pin_memory=True)

    for (input_pc, normals, index) in dataloader:
        start_ = time.time()
        input_pc = input_pc.cuda().squeeze()
        normals = normals.cuda().squeeze()

        print("Processing file number {0}".format(counter))
        counter = counter + 1
        output = []
        for rep in range(repeats):
            s_rep = time.time()
            network.train()

            if loop_rep == 0:
                number_of_iterations = 300
            else:
                number_of_iterations = 150
            print(number_of_iterations)
            latent, output, rep = optimize_latent(input_pc, normals, conf, number_of_iterations, network, lr=5e-3,
                                                  output=output, input_pc=input_pc, ds=ds, index=index, my_path=my_path,
                                                  epoch=epoch, rep=rep, resolution=resolution,
                                                  latent_path=latent_path)

            all_latent = latent.repeat(input_pc.shape[0], 1)

            points = torch.cat([all_latent, input_pc], dim=-1)

            shapename = str.join('_', ds.get_info(index))

            with torch.no_grad():

                network.eval()

                _, mesh = plt.plot_surface(with_points=True,
                                 points=points,
                                 decoder=network,
                                 latent=latent,
                                 path=my_path,
                                 epoch=epoch,
                                 rep = "mean",
                                 shapename=shapename,
                                 resolution=resolution,
                                 mc_value=0,
                                 is_uniform_grid=uniform_grid,
                                 verbose=False,
                                 save_html=False,
                                 save_ply=True,
                                 overwrite=True,
                                 connected=True)
                output.append(torch.tensor(mesh))
            print("Rep time: "+str(time.time()-s_rep))


        output = torch.stack(output)
        np.save('temp.npy', output.cpu().detach().numpy())
        output = output.mean(0)
        output = output.cpu().detach().numpy()
        filename = '{0}/igr_{1}_{2}'.format(my_path, epoch, shapename)+"_average"
        grid = get_grid_uniform(resolution)
        verts, faces, normals, values = measure.marching_cubes_lewiner(
            volume=output,
            level=0,
            spacing=(grid['xyz'][0][2] - grid['xyz'][0][1],
                     grid['xyz'][0][2] - grid['xyz'][0][1],
                     grid['xyz'][0][2] - grid['xyz'][0][1]))

        verts = verts + np.array([grid['xyz'][0][0], grid['xyz'][1][0], grid['xyz'][2][0]])
        meshexport = trimesh.Trimesh(verts, faces, normals, vertex_colors=values)
        meshexport.export(filename + '.ply', 'ply')
        _ = _meshfix.clean_from_file(filename + '.ply', filename + '.ply')
        torch.save(latent, os.path.join(my_path, 'latent.pt'))
        print("Object time: "+str(time.time()-start_))

if __name__ == '__main__':
    start = time.time()
    arg_parser = argparse.ArgumentParser()

    arg_parser.add_argument(
        "--gpu",
        "-g",
        dest="gpu_num",
        required=False,
        default='ignore'
    )

    arg_parser.add_argument(
        "--exps-dir",
        dest="exps_dir",
        required=False,
        default='exps'
    )

    arg_parser.add_argument(
        "--timestamp",
        "-t",
        dest="timestamp",
        default='latest',
        required=False,
    )

    arg_parser.add_argument(
        "--conf",
        "-f",
        dest="conf",
        default='dfaust_setup.conf',
        required=False,
    )

    arg_parser.add_argument(
        "--split",
        "-s",
        dest="split",
        default='dfaust/test_models.json',
        required=False,
    )

    arg_parser.add_argument(
        "--exp-name",
        "-e",
        dest="exp_name",
        required=True,
        help="experiment name",
    )

    arg_parser.add_argument(
        "--checkpoint",
        "-c",
        dest="epoch",
        default='latest',
        help="checkpoint to test.",
    )

    arg_parser.add_argument(
        "--resolution",
        "-r",
        dest="resolution",
        help='resolution of marching cube grid',
        default=40
    )

    arg_parser.add_argument(
        "--uniform-grid",
        "-u",
        dest="uniform_grid",
        action="store_true",
        help='use uniform grid in marching cube or non uniform',
        default=False
    )

    arg_parser.add_argument(
        "--repeats",
        dest="repeats",
        help="Number of repeats",
        default = 1
    )

    arg_parser.add_argument(
        "--save_directory",
        dest="save_directory",
        help="Directory to save the files",
        required=False,
        default = None
    )

    arg_parser.add_argument(
        "--loop_rep",
        dest="loop_rep",
        help="Rep of the main loop",
        required=True
    )

    arg_parser.add_argument(
        "--offline_rec",
        dest="offline_rec",
        help="if to use offline reconstruction settings",
        required=False,
        action="store_true",
        default=False
    )

    print('evaluating')

    args = arg_parser.parse_args()

    code_path = os.path.abspath(os.path.curdir)
    exps_path = os.path.join(os.path.abspath(os.path.pardir), args.exps_dir)

    if args.gpu_num != 'ignore':
        os.environ["CUDA_VISIBLE_DEVICES"] = '{0}'.format(args.gpu_num)

    conf = ConfigFactory.parse_file(os.path.join(code_path, 'shapespace', args.conf))

    experiment_directory = os.path.join(exps_path, args.exp_name)

    if args.timestamp == 'latest':
        timestamps = os.listdir(experiment_directory)
        timestamp = sorted(timestamps)[-1]
    else:
        timestamp = args.timestamp

    experiment_directory = os.path.join(experiment_directory, timestamp)
    saved_model_state = torch.load(os.path.join(experiment_directory, 'checkpoints', 'ModelParameters', args.epoch + ".pth"))
    saved_model_epoch = saved_model_state["epoch"]
    with_normals = conf.get_float('network.loss.normals_lambda') > 0
    network = utils.get_class(conf.get_string('train.network_class'))(d_in=conf.get_int('train.latent_size')+conf.get_int('train.d_in'), **conf.get_config('network.inputs'))

    network.load_state_dict({k.replace('module.', ''): v for k, v in saved_model_state["model_state_dict"].items()})

    split_file = os.path.join(code_path, 'splits', args.split)
    evaluate(
        network=network.cuda(),
        experiment_directory=experiment_directory,
        conf=conf,
        checkpoint=saved_model_epoch,
        split_file=split_file,
        epoch=saved_model_epoch,
        resolution=int(args.resolution),
        uniform_grid=args.uniform_grid,
        repeats=int(args.repeats),
        save_directory=args.save_directory,
        loop_rep=int(args.loop_rep),
        offline_rec=args.offline_rec
    )
    print("Total time: "+str(time.time()-start))


