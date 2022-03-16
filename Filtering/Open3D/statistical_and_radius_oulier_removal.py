

import numpy as np
import open3d as o3d




print("Load a ply point cloud, print it, and render it")
pcd = o3d.io.read_point_cloud("/home/mboubaker_tmp/mymap3.ply")
print(pcd)
print(np.asarray(pcd.points))
o3d.visualization.draw_geometries([pcd])

#print("Downsample the point cloud with a voxel of 0.02")
#voxel_down_pcd = pcd.voxel_down_sample(voxel_size=0.01)
#o3d.visualization.draw_geometries([voxel_down_pcd],
#                                  zoom=0.3412,
#                                  front=[0.4257, -0.2125, -0.8795],
#                                  lookat=[2.6172, 2.0475, 1.532],
#                                  up=[-0.0694, -0.9768, 0.2024])
                                  
def display_inlier_outlier(cloud, ind):
    inlier_cloud = cloud.select_by_index(ind)
    outlier_cloud = cloud.select_by_index(ind, invert=True)
    
    voxel_down_pcd1 = inlier_cloud.voxel_down_sample(voxel_size=0.01)
    
    print("Showing outliers (red) and inliers (gray): ")
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    o3d.visualization.draw_geometries([voxel_down_pcd1],
                                      zoom=0.3412,
                                      front=[0.4257, -0.2125, -0.8795],
                                      lookat=[2.6172, 2.0475, 1.532],
                                      up=[-0.0694, -0.9768, 0.2024])

print("Statistical oulier removal")
cl, ind = pcd.remove_statistical_outlier(nb_neighbors=50,
                                                    std_ratio=0.5)
display_inlier_outlier(voxel_down_pcd, ind)


print("Radius oulier removal")
cl, ind = pcd.remove_radius_outlier(nb_points=30, radius=0.05)
display_inlier_outlier(pcd, ind)



