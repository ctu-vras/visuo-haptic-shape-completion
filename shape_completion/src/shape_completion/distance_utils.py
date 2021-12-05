#!/usr/bin/env python2
"""
Utils for distance and similarity computation

@author Lukas Rustler
"""
import numpy as np
from scipy.spatial import cKDTree as kd
from subprocess import call, PIPE


def jaccard_similarity(s1, s2):
    """
    Computes Jaccard similarity (intersection over union) of two arrays.
    @param s1: Array for the first object
    @type s1: list / np.array()
    @param s3: Array for the second object
    @type s2: list / np.array()
    @return: Jaccard similarity, in range <0,1>
    @rtype: float
    """
    intersection = np.count_nonzero(np.logical_and(s1, s2))
    union = np.count_nonzero(np.logical_or(s1, s2))

    return float(intersection)/float(union) if union != 0 else 0


def chamfer_distance(s1, s2):
    """
    Computes chamfer distance between two sets of point.
    @param s1: Array for the first object
    @type s1: list / np.array()
    @param s3: Array for the second object
    @type s2: list / np.array()
    @return: Chamfer distance
    @rtype: float
    """
    s1 = np.array(s1)
    s2 = np.array(s2)

    s1_tree = kd(s1)
    s2_tree = kd(s2)

    d_s1, _ = s2_tree.query(s1, p=2)
    d_s2, _ = s1_tree.query(s2, p=2)

    return np.mean(d_s1) + np.mean(d_s2)


def robust_icp(source, target, out, method):
    """
    Help method to call FRICP ICP
    @param source: path to the source point cloud
    @type source: string
    @param target: path to target point cloud
    @type target: string
    @param out: path to output folder
    @type out: string
    @param method: number of method to use
    @type method: int
    @return:
    @rtype:
    """
    call("FRICP "+target+" "+source+" "+out+" "+str(method), shell=True, stdout=PIPE)
