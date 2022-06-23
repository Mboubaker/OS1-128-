# Instant Neural Graphics Primitives

- This NeRF implementation expects initial camera parameters to be provided in a transforms.json file in a format compatible with the original NeRF codebase. 
Ther's a script as a convenience, scripts/colmap2nerf.py, that can be used to process a video file or sequence of images,
using the open source COLMAP structure from motion software to extract the necessary camera data. 

- it is important for the dataset to have good coverage, to not contain mislabelled camera data, and to not contain 
blurry frames (motion blur and defocus blur are both problematic).

## Preparing new NeRF datasets

Make sure that you have installed COLMAP and it is available in your PATH. 
If you are using a video file as input, also 

### If you are training from a video file : 

- be sure to install FFmpeg and make sure that it is available in your PATH.
- run the scripts/colmap2nerf.py script from the folder containing the video, with the following recommended parameters:

         data-folder$ python [path-to-instant-ngp]/scripts/colmap2nerf.py --video_in <filename of video> --video_fps 2 --run_colmap --aabb_scale 16
         
 It is recommended to choose a frame rate that leads to around 50-150 images. So for a one minute video, --video_fps 2 is ideal.
 
 
### If you are training from images : 

- Place them in a subfolder called images 
- Then use suitable options such as the ones below:

        data-folder$ python [path-to-instant-ngp]/scripts/colmap2nerf.py --colmap_matcher exhaustive --run_colmap --aabb_scale 16

The NeRF model trains best with between 50-150 images which exhibit minimal scene movement, motion blur or other blurring artefacts      

## Training : 

Assuming success, you can now train your NeRF model as follows, starting in the instant-ngp folder:

        instant-ngp$ ./build/testbed --mode nerf --scene [path to training data folder containing transforms.json]
