#!/usr/bin/env bash
kortex_description='rospack find kinova_mujoco'
folder=`$kortex_description`
file="${folder}/urdf/kinova_fel_shape_completion.xacro"
rosrun xacro xacro --inorder $file -o "${folder}/urdf/kinova_fel_shape_completion.urdf" printed_finger:=$1 convex_decomp:=$2 real_setup:=$3
