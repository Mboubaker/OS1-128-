# PCL 
The Point Cloud Library (PCL) is a standalone, large scale, open project for 2D/3D image and point cloud processing.

## Install python-pcl via pip
PCL is available in a number of Linux distributions namely Ubuntu, Debian, Fedora, Gentoo and Arch. We recommend to install python-pcl via pip:

      pip install python-pcl
      

You can also install the development version of python-pcl from a cloned Git repository:

    git clone https://github.com/strawlab/python-pcl.git
    cd pcl/Python
    python setup.py install
    
    


## Filtering Tutorials

###Filtering a PointCloud using a PassThrough filter : 
In this part, we will learn how to remove points whose values fall inside/outside a user given interval along a specified dimension.

TestCode:
            https://github.com/strawlab/python-pcl/blob/master/examples/official/Filtering/PassThroughFilter.py
      


### Downsampling a PointCloud using a VoxelGrid filter
In this part, we will learn how to downsample (i.e., reduce the number of points) a Point Cloud.

TestCode :
           https://github.com/strawlab/python-pcl/blob/master/examples/official/Filtering/VoxelGrid_160.py
    
    

### Removing outliers using a StatisticalOutlierRemoval filter
In this part, we will learn how to remove sparse outliers from noisy data, using StatisticalRemoval.

TestCode :
           https://github.com/strawlab/python-pcl/blob/master/examples/official/Filtering/statistical_removal.py
    
    



### Removing outliers using a Conditional or RadiusOutlier removal
In this tutorial, we will learn how to remove outliers from noisy data, using ConditionalRemoval, RadiusOutlierRemoval.

TestCode : 
           https://github.com/strawlab/python-pcl/blob/master/examples/official/Filtering/remove_outliers.py
    




    
    


