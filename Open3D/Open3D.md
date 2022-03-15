# Open3D
Open3D is an open-source library that supports rapid development of software that deals with 3D data
## Installation with pip 

    pip install open3d
    # or
    pip3 install open3d
    # or
    pip install --user open3d
    # or
    python3 -m pip install --user open3d

If you have other Python versions or operating systems, please refer to Build from source(http://www.open3d.org/docs/release/compilation.html#compilation) 
and compile Open3D from source.

## Verify installation
    python -c "import open3d as o3d; print(o3d.__version__)"
    

Note: Open3D’s Python tutorial utilizes some external packages: numpy, matplotlib, opencv-python. 


## Visualize point cloud
To read a point cloud and visualize it : 
  
    print("Load a ply point cloud, print it, and render it")
    ply_point_cloud = o3d.data.PLYPointCloud()
    pcd = o3d.io.read_point_cloud(ply_point_cloud.path)
    print(pcd)
    print(np.asarray(pcd.points))
    o3d.visualization.draw_geometries([pcd],
                                     zoom=0.3412,
                                     front=[0.4257, -0.2125, -0.8795],
                                     lookat=[2.6172, 2.0475, 1.532],
                                     up=[-0.0694, -0.9768, 0.2024])


## Voxel downsampling
Voxel downsampling uses a regular voxel grid to create a uniformly downsampled point cloud from an input point cloud. It is often used as a pre-processing step for many point cloud processing tasks. The algorithm operates in two steps: points are bucketed into voxels and after that each occupied voxel generates exactly one point by averaging all points inside.

     print("Downsample the point cloud with a voxel of 0.05")
     downpcd = pcd.voxel_down_sample(voxel_size=0.05)
     o3d.visualization.draw_geometries([downpcd],
                                      zoom=0.3412,
                                      front=[0.4257, -0.2125, -0.8795],
                                      lookat=[2.6172, 2.0475, 1.532],
                                      up=[-0.0694, -0.9768, 0.2024])
                                      


## Point cloud outlier removal
When collecting data from scanning devices, the resulting point cloud tends to contain noise and artifacts that one would like to remove. 
This part addresses the outlier removal features of Open3D.

##Prepare input data

A point cloud is loaded and downsampled using voxel_downsample.

    print("Load a ply point cloud, print it, and render it")
    sample_pcd_data = o3d.data.PCDPointCloud()
    pcd = o3d.io.read_point_cloud(sample_pcd_data.path)
    o3d.visualization.draw_geometries([pcd],
                                      zoom=0.3412,
                                      front=[0.4257, -0.2125, -0.8795],
                                      lookat=[2.6172, 2.0475, 1.532],
                                      up=[-0.0694, -0.9768, 0.2024])

    print("Downsample the point cloud with a voxel of 0.02")
    voxel_down_pcd = pcd.voxel_down_sample(voxel_size=0.02)
    o3d.visualization.draw_geometries([voxel_down_pcd],
                                      zoom=0.3412,
                                      front=[0.4257, -0.2125, -0.8795],
                                      lookat=[2.6172, 2.0475, 1.532],
                                      up=[-0.0694, -0.9768, 0.2024])
                                      
## Statistical outlier removal

statistical_outlier_removal removes points that are further away from their neighbors compared to the average for the point cloud. 
It takes two input parameters:

nb_neighbors, which specifies how many neighbors are taken into account in order to calculate the average distance for a given point.

std_ratio, which allows setting the threshold level based on the standard deviation of the average distances across the point cloud.
The lower this number the more aggressive the filter will be.
    
    print("Statistical oulier removal")
    cl, ind = voxel_down_pcd.remove_statistical_outlier(nb_neighbors=20,
                                                        std_ratio=2.0)
    display_inlier_outlier(voxel_down_pcd, ind) 
    


## Radius outlier removal    
radius_outlier_removal: removes points that have few neighbors in a given sphere around them. Two parameters can be used to tune the filter to your data:
nb_points: which lets you pick the minimum amount of points that the sphere should contain.
radius: which defines the radius of the sphere that will be used for counting the neighbors.

    print("Radius oulier removal")
    cl, ind = voxel_down_pcd.remove_radius_outlier(nb_points=16, radius=0.05)
    display_inlier_outlier(voxel_down_pcd, ind)
    
    
## RASNAC 

RANSAC

We use RANSAC for global registration. In each RANSAC iteration, ransac_n random points are picked from the source point cloud. Their corresponding points in the target point cloud are detected by querying the nearest neighbor in the 33-dimensional FPFH feature space. A pruning step takes fast pruning algorithms to quickly reject false matches early.

Open3D provides the following pruning algorithms:

- CorrespondenceCheckerBasedOnDistance checks if aligned point clouds are close (less than the specified threshold).
- CorrespondenceCheckerBasedOnEdgeLength checks if the lengths of any two arbitrary edges (line formed by two vertices) individually drawn from source and target correspondences are similar. 
- CorrespondenceCheckerBasedOnNormal considers vertex normal affinity of any correspondences. It computes the dot product of two normal vectors. It takes a radian value for the threshold.

Only matches that pass the pruning step are used to compute a transformation, which is validated on the entire point cloud. The core function is registration_ransac_based_on_feature_matching. The most important hyperparameter of this function is RANSACConvergenceCriteria. It defines the maximum number of RANSAC iterations and the confidence probability. The larger these two numbers are, the more accurate the result is, but also the more time the algorithm takes.

For more details : http://www.open3d.org/docs/release/tutorial/pipelines/global_registration.html





