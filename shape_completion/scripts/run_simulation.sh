#!/bin/bash
rosrun kinova_mujoco prepare_object.py -n "$1" -o "$2" -f "$3" -c "$4"
roslaunch shape_completion simulated.launch object_name:="$1" object_origin:="$2" printed_finger:="$3" convex_decomp:="$4"