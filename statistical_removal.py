# Removing outliers using a StatisticalOutlierRemoval filter
# http://pointclouds.org/documentation/tutorials/statistical_outlier.php#statistical-outlier-removal

import pcl


def main():
    p = pcl.load('cloud_bin_1.ply')

    fil = p.make_statistical_outlier_filter()
    fil.set_mean_k(50)
    fil.set_std_dev_mul_thresh(1.0)

    pcl.save(fil.filter(), "table_scene_lms400_inliers.ply")

    fil.set_negative(True)
    pcl.save(fil.filter(), "table_scene_lms400_outliers.ply")


if __name__ == "__main__":
    # import cProfile
    # cProfile.run('main()', sort='time')
    main()
