#!/usr/bin/env python3

from os import listdir

import numpy as np

import open3d as o3d
from open3d import geometry as gmt


if __name__ == "__main__":

    # List directory with samples
    pcd_files = listdir("/home/user/projects/shadow_robot/base/pc_tests")

    # Loop to read all pcd files
    point_clouds = []
    for file in pcd_files:
        point_clouds.append(o3d.io.read_point_cloud("/home/user/projects/shadow_robot/base/pc_tests/" + file))

    o3d.visualization.draw_geometries(point_clouds)

    # Create meshes
    meshes = []
    alpha = 0.03

    i = 0.0
    total_pc = float(len(point_clouds))
    print(total_pc)
    for point_cloud in point_clouds:
        points_array = np.asarray(point_cloud.points)

        try:
            mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(point_cloud, alpha)
            color = np.array([i/total_pc,(total_pc-i)/total_pc,i/total_pc])
            print(color)
            mesh.paint_uniform_color(color)
            i = i+1.0
            meshes.append(mesh)
        except:
            print("Error with one point cloud")
            continue
        

    o3d.visualization.draw_geometries(meshes, mesh_show_back_face=True)