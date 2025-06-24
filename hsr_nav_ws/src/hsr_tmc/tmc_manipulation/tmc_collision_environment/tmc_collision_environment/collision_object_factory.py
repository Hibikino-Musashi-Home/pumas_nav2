#!/usr/bin/env python3
'''
Copyright (c) 2024 TOYOTA MOTOR CORPORATION
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
'''
# -*- coding: utf-8 -*-
import math

from geometry_msgs.msg import (
    Point,
    Pose,
)
from moveit_msgs.msg import CollisionObject
import numpy as np
from shape_msgs.msg import (
    Mesh,
    MeshTriangle,
    SolidPrimitive,
)
from stl import mesh


def _generate_box_shape(dimensions: list[float]) -> SolidPrimitive:
    shape = SolidPrimitive()
    shape.type = SolidPrimitive.BOX
    shape.dimensions = dimensions
    return shape


def _generate_sphere_shape(radius: float) -> SolidPrimitive:
    shape = SolidPrimitive()
    shape.type = SolidPrimitive.SPHERE
    shape.dimensions = [radius]
    return shape


def _to_pose(position: list[float], orientation: list[float] = [0.0, 0.0, 0.0, 1.0]) -> Pose:
    pose = Pose()
    pose.position.x = position[0]
    pose.position.y = position[1]
    pose.position.z = position[2]
    pose.orientation.x = orientation[0]
    pose.orientation.y = orientation[1]
    pose.orientation.z = orientation[2]
    pose.orientation.w = orientation[3]
    return pose


def _generate_object(name: str,
                     primitives: list[SolidPrimitive],
                     primitive_poses: list[Pose],
                     ref_to_object: Pose = Pose(),
                     ref_frame_id: str = 'map') -> CollisionObject:
    collision_object = CollisionObject()
    collision_object.header.frame_id = ref_frame_id
    collision_object.id = name
    collision_object.pose = ref_to_object
    collision_object.operation = CollisionObject.ADD
    collision_object.primitives = primitives
    collision_object.primitive_poses = primitive_poses
    return collision_object


def generate_simple_box(name: str, dimensions: list[float], center_x: float, center_y: float, bottom_z: float,
                        yaw: float = 0.0, ref_frame_id: str = 'map') -> CollisionObject:
    """Assuming that it is placed on a plane, generate a box that makes the bottom height become bottom_z."""
    return _generate_object(name,
                            [_generate_box_shape(dimensions)],
                            [Pose()],
                            _to_pose([center_x, center_y, bottom_z + dimensions[2] / 2.0],
                                     [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]),
                            ref_frame_id)


def generate_table_box(name: str, dimensions: list[float], x: float, y: float,
                       yaw: float = 0.0, ref_frame_id: str = 'map') -> CollisionObject:
    """Assuming a table, generate a box where Dimensions [2] is high"""
    return _generate_object(name,
                            [_generate_box_shape(dimensions)],
                            [Pose()],
                            _to_pose([x, y, dimensions[2] / 2.0],
                                     [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]),
                            ref_frame_id)


def generate_shelf(name: str, external_dimensions: list[float], board_heights: list[float],
                   x: float, y: float, yaw: float = 0.0,
                   board_thickness: float = 0.02, ref_frame_id: str = 'map') -> CollisionObject:
    """Assuming a shelf, a box group is generated so that the position at the front of the opening is x, y, and the direction is YAW."""
    thickness_half = board_thickness / 2.0
    width_half = external_dimensions[1] / 2.0
    height_half = external_dimensions[2] / 2.0
    board_depth = external_dimensions[0] - board_thickness
    board_width = external_dimensions[1] - 2.0 * board_thickness

    frame_shapes = [_generate_box_shape([board_thickness, external_dimensions[1], external_dimensions[2]]),
                    _generate_box_shape([board_depth, board_thickness, external_dimensions[2]]),
                    _generate_box_shape([board_depth, board_thickness, external_dimensions[2]]),
                    _generate_box_shape([board_depth, board_width, board_thickness])]
    frame_poses = [_to_pose([-external_dimensions[0] + thickness_half, 0.0, height_half]),
                   _to_pose([-board_depth / 2.0, width_half - thickness_half, height_half]),
                   _to_pose([-board_depth / 2.0, -(width_half - thickness_half), height_half]),
                   _to_pose([-board_depth / 2.0, 0.0, external_dimensions[2] - thickness_half])]

    board_shapes = [_generate_box_shape([board_depth, board_width, board_thickness])] * len(board_heights)
    board_poses = [_to_pose([-board_depth / 2.0, 0.0, z - thickness_half]) for z in board_heights]

    return _generate_object(name,
                            frame_shapes + board_shapes,
                            frame_poses + board_poses,
                            _to_pose([x, y, 0.0], [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]),
                            ref_frame_id)


def generate_spheres(name: str, positions: list[list[float]],
                     radius: float = 0.02, ref_frame_id: str = 'map') -> CollisionObject:
    """It is assumed to be treated as an obstacle that generates a ball group as an obstacle."""
    return _generate_object(name,
                            [_generate_sphere_shape(radius)] * len(positions),
                            [_to_pose(pos) for pos in positions],
                            Pose(),
                            ref_frame_id)


def generate_object_from_mesh_file(name: str, file_path: str,
                                   x: float, y: float, z: float,
                                   qx: float = 0.0, qy: float = 0.0, qz: float = 0.0, qw: float = 1.0,
                                   ref_frame_id: str = 'map') -> CollisionObject:
    """Read a mesh file to generate obstacles"""
    collision_object = CollisionObject()
    collision_object.header.frame_id = ref_frame_id
    collision_object.id = name
    collision_object.pose = _to_pose([x, y, z], [qx, qy, qz, qw])
    collision_object.operation = CollisionObject.ADD
    collision_object.mesh_poses = [Pose()]

    mesh_data = mesh.Mesh.from_file(file_path)
    all_vertices = np.vstack([mesh_data.v0, mesh_data.v1, mesh_data.v2])
    vertices, indices = np.unique(all_vertices, return_inverse=True, axis=0)
    indices = indices.reshape(3, -1).transpose()

    mesh_msg = Mesh()
    for vertex in vertices:
        point = Point()
        point.x, point.y, point.z = vertex.astype(np.float64)
        mesh_msg.vertices.append(point)
    for index in indices:
        triangle = MeshTriangle()
        triangle.vertex_indices = index.tolist()
        mesh_msg.triangles.append(triangle)
    collision_object.meshes.append(mesh_msg)
    return collision_object
