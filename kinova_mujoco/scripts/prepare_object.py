#!/usr/bin/env python2
"""
Main function to prepare the object in simulation

@author Lukas Rustler
"""
from kinova_mujoco.utils import *
import os
import argparse


def parse_arguments():
    arg_parser = argparse.ArgumentParser(
        description="Script for object preparation for the simulation"
    )
    arg_parser.add_argument(
        "--object_name",
        "-n",
        dest="object_name",
        required=True,
        help="Name of the .stl of the object"
    )

    arg_parser.add_argument(
        "--origin",
        "-o",
        dest="origin",
        required=False,
        default=[0.6,0,0.2],
        help="Position of the object in space"
    )

    arg_parser.add_argument(
        "--printed_finger",
        "-f",
        dest="printed_finger",
        required=False,
        default='false',
        help="If to use printed finger"
    )

    arg_parser.add_argument(
        "--convex_decomp",
        "-c",
        dest="convex_decomp",
        required=False,
        default='false',
        help="If to use convex decomposition"
    )

    args = arg_parser.parse_args()

    return args.object_name, eval(args.origin), args.printed_finger, str_to_bool(args.convex_decomp)


if __name__ == "__main__":

    object_name, origin, printed_finger, convex_decomp = parse_arguments()
    create_object_scene(object_name, mujoco=False, convex_decomp=convex_decomp, origin=origin)#np.hstack((origin, [0, 0, 0]))

    #if not convex_decomp:
    object = open(os.path.join(os.path.dirname(__file__),'../urdf/object.urdf'), "r")
    lines = object.read().splitlines()
    lines[-3] = '     <origin rpy="0.0 0.0 0.0" xyz="'+" ".join(map(str, origin))+'"/>'
    #object = open(os.path.join(os.path.dirname(__file__),'../world_model/object.urdf'),"w")
    object.close()
    object = open(os.path.join(os.path.dirname(__file__),'../urdf/object.urdf'), "w")
    for line in lines:
        object.write(line+"\n")
    object.close()
    prepare_urdf(printed_finger, bool_to_str(convex_decomp))