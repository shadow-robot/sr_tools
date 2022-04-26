#!/usr/bin/env python3

from os import listdir

import numpy as np

import open3d as o3d
from open3d import geometry as gmt

import random


if __name__ == "__main__":

    # List directory with samples
    pcd_files_hh = listdir("/home/user/projects/shadow_robot/base/pc_tests")

    # Loop to read all pcd files
    point_clouds_hh = {}
    for file in pcd_files_hh:
        point_clouds_hh.update({file[0:-4] : o3d.io.read_point_cloud("/home/user/projects/shadow_robot/base/pc_tests/" + file)})

    print("All pcd read")

    o3d.visualization.draw_geometries(list(point_clouds_hh.values()))

    # Create meshes
    meshes = {}
    alpha = 0.02

    i = 0.0
    total_pc = float(len(point_clouds_hh))

    # Compute alpha shape for Human Hand point clouds
    for joint in point_clouds_hh:
        points_array = np.asarray(point_clouds_hh[joint].points)

        try:
            mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(point_clouds_hh[joint], alpha)
            if mesh.is_empty():
                continue
            color = np.array([random.uniform(0, 1) ,random.uniform(0, 1) ,random.uniform(0, 1)])
            mesh.paint_uniform_color(color)
            i = i+1.0
            meshes.update({joint : mesh})
        except:
            print("Error with " + joint + " point cloud")
            continue
        

    o3d.visualization.draw_geometries(list(meshes.values()), mesh_show_back_face=True)

    # TODO: Read PC Artificial Hand and check how many points of the AH lay inside the region
    pcd_files_ah = listdir("/home/user/projects/shadow_robot/base/pc_tests")

    # Loop to read all pcd files
    point_clouds_ah = {}
    for file in pcd_files_ah:
        point_clouds_ah.update({file : o3d.io.read_point_cloud("/home/user/projects/shadow_robot/base/pc_tests/" + file)})

    # Compare and see how many points inside every joint
    for joint in meshes:
        if joint in point_clouds_ah.keys():
            # Both joint exist and are comparable
            # Convert points to array, iterate and get occupancy. Add this number up
            array_points_ah = np.asarray(point_clouds_ah[joint].points)
            
            points_inside = 0
            scene = o3d.t.geometry.RaycastingScene()
            mesh_t = o3d.t.geometry.TriangleMesh.from_legacy(meshes[joint])
            scene.add_triangles(mesh_t)

            for point in array_points_ah:
                query_point = o3d.core.Tensor([point], dtype=o3d.core.Dtype.Float32)
                if scene.compute_occupancy(query_point) > 0:
                    points_inside = points_inside + 1
            
            print("Joint " + joint + " has " + str(points_inside) + " points inside")

            o3d.visualization.draw_geometries([meshes[joint], point_clouds_ah[joint]], mesh_show_back_face=True)
