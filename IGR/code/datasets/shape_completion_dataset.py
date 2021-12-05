import torch
import torch.utils.data as data
import numpy as np
import os
import utils.general as utils


class ShapeCompletionDataset(data.Dataset):

    def __init__(self, dataset_path, split, points_batch=16384, d_in=3, with_gt=False, with_normals=False):

        base_dir = os.path.abspath(dataset_path)
        self.npyfiles_mnfld = get_instance_filenames(base_dir, split)
        self.points_batch = points_batch
        self.with_normals = with_normals
        self.d_in = d_in
        if with_gt:
            self.scans_files = get_instance_filenames(utils.concat_home_dir('datasets/dfaust/scans'), split, '','ply')
            self.scripts_files = get_instance_filenames(utils.concat_home_dir('datasets/dfaust/scripts'), split, '','obj')
            self.shapenames = [x.split('/')[-1].split('.ply')[0] for x in self.scans_files]

    def load_points(self, index):
        return np.load(self.npyfiles_mnfld[index])

    def get_info(self, index):
        shape_name, pose, tag = self.npyfiles_mnfld[index].split('/')[-3:]
        return shape_name, pose, tag[:tag.find('.npy')]

    def __getitem__(self, index):

        point_set_mnlfld = torch.from_numpy(self.load_points(index)).float()

        random_idx = torch.randperm(point_set_mnlfld.shape[0])[:self.points_batch]
        point_set_mnlfld = torch.index_select(point_set_mnlfld, 0, random_idx)

        if self.with_normals:
            normals = point_set_mnlfld[:, -self.d_in:]  # todo adjust to case when we get no sigmas

        else:
            normals = torch.empty(0)

        return point_set_mnlfld[:, :self.d_in], normals, index

    def __len__(self):
        return len(self.npyfiles_mnfld)


def get_instance_filenames(base_dir, split, ext='', format='npy'):
    npyfiles = []
    l = 0
    for dataset in split:
        for shape in split[dataset]:
            instance_filename = os.path.join(base_dir, shape, shape + "{0}.{1}".format(ext, format))
            if not os.path.isfile(instance_filename):
                print(
                    'Requested non-existent file "' + instance_filename + "' {0}".format(l)
                )
                l = l + 1
            npyfiles.append(instance_filename)
    return npyfiles
