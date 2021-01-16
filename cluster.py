import numpy as np
import pcl


cloud = pcl.load('table_scene_lms400.pcd')

#   // Create the filtering object: downsample the dataset using a leaf size of 1cm
vg = cloud.make_voxel_grid_filter()
vg.set_leaf_size (0.01, 0.01, 0.01)
cloud_filtered = vg.filter ()

#   // Create the segmentation object for the planar model and set all the parameters
seg = cloud.make_segmenter()
seg.set_optimize_coefficients (True)
seg.set_model_type (pcl.SACMODEL_PLANE)
seg.set_method_type (pcl.SAC_RANSAC)
seg.set_MaxIterations (100)
seg.set_distance_threshold (0.02)


i = 0
nr_points = cloud_filtered.size


# Creating the KdTree object for the search method of the extraction
tree = cloud_filtered.make_kdtree()


ec = cloud_filtered.make_EuclideanClusterExtraction()
ec.set_ClusterTolerance (0.02)
ec.set_MinClusterSize (100)
ec.set_MaxClusterSize (250000)
ec.set_SearchMethod (tree)
cluster_indices = ec.Extract()

print('cluster_indices : ' + str(cluster_indices.count) + " count.")
cloud_cluster = pcl.PointCloud()

for j, indices in enumerate(cluster_indices):
    # cloudsize = indices
    print('indices = ' + str(len(indices)))
    # cloudsize = len(indices)
    points = np.zeros((len(indices), 3), dtype=np.float32)
    # points = np.zeros((cloudsize, 3), dtype=np.float32)

    # for indice in range(len(indices)):
    for i, indice in enumerate(indices):
        # print('dataNum = ' + str(i) + ', data point[x y z]: ' + str(cloud_filtered[indice][0]) + ' ' + str(cloud_filtered[indice][1]) + ' ' + str(cloud_filtered[indice][2]))
        # print('PointCloud representing the Cluster: ' + str(cloud_cluster.size) + " data points.")
        points[i][0] = cloud_filtered[indice][0]
        points[i][1] = cloud_filtered[indice][1]
        points[i][2] = cloud_filtered[indice][2]

    cloud_cluster.from_array(points)
    ss = "cloud_cluster_" + str(j) + ".pcd";
    pcl.save(cloud_cluster, ss)