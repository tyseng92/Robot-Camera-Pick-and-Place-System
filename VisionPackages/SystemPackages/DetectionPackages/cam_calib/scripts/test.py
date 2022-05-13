# examples/Python/Basic/python_binding.py

import open3d as o3d
import numpy as np


def example_import_function():
    pcd = o3d.io.read_point_cloud("plane_calibration.pcd")
    print(pcd)


def example_help_function():
    help(o3d)
    help(o3d.geometry.PointCloud)
    help(o3d.io.read_point_cloud)

def display():
    print("Load a ply point cloud, print it, and render it")
    pcd = o3d.io.read_point_cloud("vege_40_L.pcd")
    print(pcd)
    print(np.asarray(pcd.points))
    o3d.visualization.draw_geometries([pcd])

    print("Downsample the point cloud with a voxel of 0.05")
    downpcd = pcd.voxel_down_sample(voxel_size=0.05)
    o3d.visualization.draw_geometries([downpcd])

    print("Recompute the normal of the downsampled point cloud")
    downpcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
        radius=0.1, max_nn=30))
    o3d.visualization.draw_geometries([downpcd])

    print("Print a normal vector of the 0th point")
    print(downpcd.normals[0])
    print("Print the normal vectors of the first 10 points")
    print(np.asarray(downpcd.normals)[:10, :])
    print("")

if __name__ == "__main__":
    display()
    #example_import_function()
    #example_help_function()
