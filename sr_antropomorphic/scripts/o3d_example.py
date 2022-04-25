#!/usr/bin/env python3

import open3d as o3d
from open3d import geometry as gmt

import numpy as np

if __name__ == "__main__":

    pcd = o3d.io.read_point_cloud("/home/user/projects/shadow_robot/base/pc_tests/rh_mftip.pcd")
    pcd_2 = o3d.io.read_point_cloud("/home/user/projects/shadow_robot/base/pc_tests/rh_thtip.pcd")
    print("Displaying input pointcloud ...")
    o3d.visualization.draw_geometries([pcd])
    alpha = 0.03
    print(f"alpha={alpha:.3f}")
    print('Running alpha shapes surface reconstruction ...')
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(
        pcd, alpha)

    mesh.paint_uniform_color(np.array([0.8,0.2,0.8]))

    query_point = o3d.core.Tensor([[10, 10, 10]], dtype=o3d.core.Dtype.Float32)
    
    scene = o3d.t.geometry.RaycastingScene()

    mesh_t = o3d.t.geometry.TriangleMesh.from_legacy(mesh)

    scene.add_triangles(mesh_t)

    occupancy = scene.compute_occupancy(query_point)

    # Create array of points
    points = np.asarray(pcd_2.points)

    for point in points:
        query_point = o3d.core.Tensor([point], dtype=o3d.core.Dtype.Float32)
        occupancy = scene.compute_occupancy(query_point)
        print(occupancy)

    print("Displaying reconstructed mesh ...")
    o3d.visualization.draw_geometries([mesh, pcd_2], mesh_show_back_face=True)
