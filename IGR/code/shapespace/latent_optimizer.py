import os
import sys
project_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
sys.path.append(project_dir)
os.chdir(project_dir)
import torch
from model.network import gradient
from model.sample import Sampler
import utils.plots as plt

def adjust_learning_rate(initial_lr, optimizer, iter):
    adjust_lr_every = 400
    lr = initial_lr * ((0.1) ** (iter // adjust_lr_every))
    for param_group in optimizer.param_groups:
        param_group["lr"] = lr


def optimize_latent(points, normals, conf, num_of_iterations, network, lr=1.0e-2, output=[], input_pc=[], ds=[], index=0, my_path=0, epoch=0,rep=0, resolution=40, latent_path=None):

    latent_size = conf.get_int('train.latent_size')
    global_sigma = conf.get_float('network.sampler.properties.global_sigma')
    local_sigma = conf.get_float('network.sampler.properties.local_sigma')
    sampler = Sampler.get_sampler(conf.get_string('network.sampler.sampler_type'))(global_sigma, local_sigma)

    latent_lambda = conf.get_float('network.loss.latent_lambda')

    normals_lambda = conf.get_float('network.loss.normals_lambda')

    grad_lambda = conf.get_float('network.loss.lambda')

    #num_of_points, dim = points.shape
    num_of_points = 1000
    if latent_path is None:
        latent = torch.ones(latent_size).normal_(0, 1 / latent_size).cuda()

    else:  # Not the first repetitions
        latent = torch.load(latent_path).detach().squeeze(0)
        latent += torch.normal(0.0, 0.001*torch.ones(latent.shape)).cuda()
    # latent = torch.zeros(latent_size).cuda()
    latent.requires_grad = True

    optimizer = torch.optim.Adam([latent], lr=lr)
    points_ = points
    normals_ = normals
    pos_start_idxs = torch.randint(0, points_.shape[0] - num_of_points, (num_of_iterations, 1))
    rep = 0
    for i in range(num_of_iterations):
        pos_start_ind = pos_start_idxs[i]
        points = points_[pos_start_ind: (pos_start_ind + num_of_points)]
        normals = normals_[pos_start_ind: (pos_start_ind + num_of_points)]

        sample = sampler.get_points(points.unsqueeze(0)).squeeze()

        latent_all = latent.expand(num_of_points, -1)
        surface_pnts = torch.cat([latent_all, points], dim=1)

        sample_latent_all = latent.expand(sample.shape[0], -1)
        nonsurface_pnts = torch.cat([sample_latent_all, sample], dim=1)

        surface_pnts.requires_grad_()
        nonsurface_pnts.requires_grad_()

        surface_pred = network(surface_pnts)
        nonsurface_pred = network(nonsurface_pnts)

        surface_grad = gradient(surface_pnts, surface_pred)
        nonsurface_grad = gradient(nonsurface_pnts, nonsurface_pred)

        surface_loss = torch.abs(surface_pred).mean()
        grad_loss = torch.mean((nonsurface_grad.norm(2, dim=-1) - 1).pow(2))
        normals_loss = ((surface_grad - normals).abs()).norm(2, dim=1).mean()
        latent_loss = latent.abs().mean()
        loss = surface_loss + latent_lambda * latent_loss + normals_lambda * normals_loss + grad_lambda * grad_loss

        adjust_learning_rate(lr, optimizer, i)

        optimizer.zero_grad()

        loss.backward()
        optimizer.step()

        # Each 50 iteration save the mesh
        if 0 < i < num_of_iterations - 1 and i % 50 == 0:
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
                                           rep=rep,
                                           shapename=shapename,
                                           resolution=resolution,
                                           mc_value=0,
                                           is_uniform_grid=True,
                                           verbose=False,
                                           save_html=False,
                                           save_ply=True,
                                           overwrite=True,
                                           connected=True)
                output.append(torch.tensor(mesh))
            rep += 1
        #print('latent loss iter {0}:{1}'.format(i, loss.item()))

    return latent.unsqueeze(0), output, rep
