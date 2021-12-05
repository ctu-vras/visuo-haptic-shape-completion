#!/usr/bin/env python
"""
Utils for object handling in the simulation

@author Jan Behrens, edited by Lukas Rustler
"""

import shutil
import urdf_parser_py.urdf
import rospkg
import trimesh
import subprocess
import os
import os

import numpy as np
import trimesh
import rospkg

from odio_urdf import *


def link(mesh, robot_name, center_gravity, mass, I, material, geom_origin, has_inertia=True, has_visual=True, has_collision=True):
    """
        Most of the links are the same except for the passed in info.
        This function just allows grouping the important numbers better.
    """
    # N = str(N)
    assert isinstance(mesh, str)
    if robot_name is None:
        ret = Link(name = mesh.split('.')[0])
    else:
        ret = Link(name=robot_name)
    if has_inertia:
        ret(Inertial(
            Origin(list(center_gravity)),
            Mass(value=mass * 1000),
            Inertia(I)))

    if has_visual:
        ret(Visual(
            Origin(geom_origin),
            Geometry(Mesh(filename = os.path.join('package://kinova_mujoco/meshes', mesh.replace(os.path.dirname(mesh),'').replace('/','')))),
            Material(material)))
    if has_collision:
        ret(Collision(
            Origin(geom_origin),
            Geometry(Mesh(filename = os.path.join('package://kinova_mujoco/meshes', mesh.replace(os.path.dirname(mesh),'').replace('/','')))),
            Material(material)),
            Contact())

    return ret


def mesh2urdf(mesh_path, obj_name, scale=[0.1, 0.1, 0.1, 1.0], maxhulls=10000, init_orig=[0.2, 0, 1.0, 0, 0, 0], robot_urdf=None, mujoco=True, convex_decomp=False):

    mesh = trimesh.load(mesh_path)  # type: trimesh

    assert isinstance(mesh, trimesh.Trimesh)
    scale_mat = np.diag(scale)
    mesh = mesh.apply_transform(scale_mat)
    rot_mat = [[0, 0, 1, 0], [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]]
    mesh = mesh.apply_transform(rot_mat)

    # save scaled mesh
    mesh_path_visual = os.path.join(os.path.dirname(mesh_path), '../meshes/', obj_name + '_visual.stl')
    obj_path = os.path.join(os.path.dirname(mesh_path), '../meshes/', 'object_visual.stl')
    body_obj_path = os.path.join(os.path.dirname(mesh_path), '../meshes/', 'object.stl')
    mesh.export(file_obj=mesh_path_visual, file_type='stl')
    mesh.export(file_obj=obj_path, file_type='stl')
    mesh.export(file_obj=body_obj_path, file_type='stl')

    # test if mesh has several components
    mesh_components = mesh.split(only_watertight=False)

    # make convex decomposition for simplified collision model
    if not convex_decomp:
        bodies = [mesh_components[0]]
    else:
        bodies = []
        print(len(mesh_components))
        for mesh_component in mesh_components:
            assert isinstance(mesh_component, trimesh.Trimesh)
            # if mesh_component.bounding_box.volume < 1e-6 or mesh_component.scale < 0.0005 or len(mesh_component.vertices) < 10:
            if mesh_component.bounding_box.volume < 1e-5 or mesh_component.scale < 0.0005 or len(
                    mesh_component.vertices) < 15:

                print('skipping mesh component because it is of negligible size')
                continue

            # mesh_component.show()
            decomp = mesh_component.convex_decomposition(maxhulls=maxhulls, pca=0, mode=0, resolution=1000000,
                                                         maxNumVerticesPerCH=1, gamma=0.0005, concavity=0)
            if isinstance(decomp, list):
                bodies += decomp
            elif isinstance(decomp, trimesh.Trimesh):
                bodies.append(decomp)
                # decomp.show()
    print("Completed")
    # show the decomposition
    scene = trimesh.Scene()
    for body in bodies:
        scene.add_geometry(body)
    #scene.show()

    if robot_urdf is None:
        myobj = Robot()
        obj_start_index = len(myobj)
    else:
        myobj = robot_urdf
        obj_start_index = len(myobj)
    # add visual link for RVIZ
    # mesh_path = obj_name + '.stl'
    if convex_decomp:
        link_vis = link(str(mesh_path_visual), obj_name + '_visual', None, None, None, "Grey", [0, 0, 0],
                    has_collision=False,
                    has_inertia=False,
                    has_visual=True)
        myobj(link_vis)

    for idx, body in enumerate(bodies):  # type: trimesh.Trimesh
        name = obj_name + '_' + '{:06d}'.format(idx)
        body_mesh_path = name + '.stl'
        # os.path.join(os.path.dirname(mesh_path), body_mesh_path)
        body.export(file_obj=os.path.join(os.path.dirname(mesh_path), '../meshes/', body_mesh_path), file_type='stl')

        mass = body.mass
        inertia = body.moment_inertia
        # print(body.principal_inertia_components)

        ixx = inertia[0, 0]
        iyy = inertia[1, 1]
        izz = inertia[2, 2]
        ixy = inertia[0, 1]
        ixz = inertia[0, 2]
        iyz = inertia[1, 2]

        if convex_decomp:
            link4 = link(body_mesh_path, None, body.center_mass, mass, [ixx, ixy, ixz, iyy, iyz, izz], "Blue", [0, 0, 0])

            myobj(link4)

    if convex_decomp:
        for idx, links in enumerate(zip(myobj[obj_start_index:-1], myobj[obj_start_index+1:])):
            l1, l2 = links
            joint = Joint(obj_name + "_joint_{:04d}".format(idx), Parent(str(l1.name)), Child(str(l2.name)), type="fixed")
            myobj(joint)

        # joint = Joint("joint_world_" + obj_name, Parent('world'), Child(myobj[obj_start_index].name), type="floating")
        if mujoco:
            joint = Joint("joint_world_" + obj_name, Parent('base_link'), Child(myobj[obj_start_index].name), type="floating")
        else:
            joint = Joint("joint_world_" + obj_name, Parent('base_link'), Child(myobj[obj_start_index].name), type="fixed")
        joint(Origin(init_orig))
        myobj(joint)
    return myobj


def create_object_scene(object, scale=1, mujoco=True, convex_decomp=False, origin=[0,0,0]):
    '''
    Make a URDF for the scene composed of objects. The object meshes are decomposed into convex bodies.
    :param objects: dict
    :param mujoco: boolean, if create mujoco object
    :return: odio urdf description of scene.
    '''
    scene_urdf = mesh2urdf(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../GT_meshes', object) + '.stl',
                           obj_name=object,
                           scale=3 * [scale] + [1.0],
                           maxhulls=1e6,
                           init_orig=origin,
                           robot_urdf=None, mujoco=mujoco, convex_decomp=convex_decomp)
    text_file = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../urdf/shape_completion_scene.urdf"), "w")
    text_file.write(scene_urdf.__str__())
    text_file.close()

    return scene_urdf


def str_to_bool(string):
    if string.lower() in ["true", 1]:
        return True
    else:
        return False


def bool_to_str(bool_):
    return "true" if bool_ else "false"


def prepare_urdf(printed_finger, convex_decomp):
    proc = subprocess.Popen(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../scripts/run_xacro.sh '+printed_finger+' '+convex_decomp+ ' false'), shell=True)
    proc.wait()
    robot = urdf_parser_py.urdf.URDF.from_xml_file(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../urdf/kinova_fel_shape_completion.urdf'))
    assert isinstance(robot, urdf_parser_py.urdf.URDF)

    file_object = open(os.path.join(os.path.dirname(__file__),'../../urdf/kinova_fel_shape_completion.urdf'), 'r+')
    lines = file_object.readlines()
    file_object.close()

    new_lines = []
    for line in lines:
        # take over comment lines unchanged
        if '<!--' in line:
            new_lines.append(line + "\n")
        elif '<mesh filename="package://' in line:
            # write new stl mesh location in robot mujoco package
            link_name = line.split('/')[-2]
            if 'scale' in link_name:
                pass
                # link_name.
            new_line = line.split('//')[0] + '//' + 'kinova_mujoco/meshes/' + link_name.replace('.dae', '.STL') + '/>'
            # line = line.replace('.dae', '.stl')
            new_lines.append(new_line + "\n")
        elif '<material name="">' in line:
            # mujoco wants everything to have a filled material tag
            new_line = line.replace('""', '"DUMMY_MATERIAL"')
            new_lines.append(new_line + "\n")
        elif '<mimic joint=' in line:
            # mujoco does not support mimic joints. we have to use a custom controller to make the mimic functionality.
            pass
        else:
            # take over normal lines
            new_lines.append(line + "\n")

    file_object = open(os.path.join(os.path.dirname(__file__),'../../urdf/kinova_fel_shape_completion_NoDae.urdf'), 'w+')
    file_object.writelines(new_lines)
    file_object.close()