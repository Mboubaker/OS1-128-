# NeRF in the Wild :Neural Radiance Fields for Unconstrained Photo Collections

The NeRF implementation expects initial camera parameters in a format compatible with the original NeRF codebase. with the script, scripts/colmap2nerf.py, you can use it to process a video file or sequence of images, using the open source COLMAP structure from motion software to extract the necessary camera data.

## Preparing new NeRF datasets :

### For training from images : 

- Step 1  : launch colmap :  
      
      $colmap automatic_reconstructor --workspace_path /work/datas/dossier_projet/ --image_path /work/datas/dossier_projet/images/

- Step2 : change the files .bin to .txt 
               
       $colmap model_converter  --input_path  /work/scratch/caraffl/datas/dossier_projet/sparse/0/ --output_path  /work/scratch/caraffl/datas/dossier_projet/ --output_type TXT

- Step 3  : you have to create a colmap_text folder next to the images folder to put there the 3 .txt files

- Step 4  : For training from images, place them in a subfolder called images and then use suitable options such as the ones below:
       
       folder of images  $ python /home/ad/caraffl/code/colmap/instant-ngp/scripts/colmap2nerf.py --colmap_matcher exhaustive  --aabb_scale 16 ( decreasing the value 16, 8, 4, 2, 1)

### For training from video file 

- If you are training from a video file, run the scripts/colmap2nerf.py script from the folder containing the video, with the following recommended parameters:

        data-folder$ python [path-to-instant-ngp]/scripts/colmap2nerf.py --video_in <filename of video> --video_fps 2 --run_colmap --aabb_scale 16

- It is recommended to choose a frame rate that leads to around 50-150 images. So for a one minute video, --video_fps 2 is ideal.
  
- The script will run FFmpeg and/or COLMAP as needed, followed by a conversion step to the required transforms.json format, which will be written in the current directory.
  
## Training
  
       python train.py \
          >   --root_dir /home/datas/dataset_name --dataset_name phototourism \
          >   --img_downscale 2  --N_importance 64 --N_samples 64 \
          >   --encode_a --encode_t --beta_min 0.03 --N_vocab 500 \
          >   --num_epochs 40 --batch_size 1024 \
          >   --optimizer adam --lr 5e-4 --lr_scheduler cosine \
          >   --exp_name kitti_scale2_nerfw \
          >   --num_gpus 4
          
*--img_downscale : an integer, e.g. 2 means half the image sizes.
*--encode_a and *--encode_t : options are both required to maximize NeRF-W performance.
*--N_vocab : should be set to an integer larger than the number of images (dependent on different scenes).

## Monitoring 

You can monitor the training process by tensorboard --logdir logs/ and go to localhost:6006 in your browser.

## Testing

Use eval.py to create the whole sequence of moving views. It will create folder results/{dataset_name}/{scene_name} and run inference on all test data, finally create a gif out of them.

     python eval.py   --root_dir /work/datas/kitti/ 
        --dataset_name phototourism --scene_name kitti --split test_train --img_downscale 2 --N_samples 256 --N_importance 256  
        --N_vocab 500 --encode_a --encode_t  --use_disp 
        --ckpt_path /work/datas/nerf_pl-nerfw/ckpts/kitti_2_nerfw/epoch=11.ckpt
        --chunk 16384 --img_wh 640 480 --video_format gif

For example, if you want to use the second GPU
https://github.com/kwea123/nerf_pl/blob/dev/eval.py : just change the .cuda() code to .to('cuda: 1') .
  

## Results : 

### kitti dataset : 

<p align="center">   
  <img src="https://user-images.githubusercontent.com/97898968/191943519-473797e5-3736-4113-8e69-0a8dcffb2329.gif?raw=true" alt="Sublime's custom image"/>
</p>

 <p align="center"> 
 Figure: result 1 of Nerf in the wild 
 </p>


<p align="center">   
  <img src="https://user-images.githubusercontent.com/97898968/191944010-66cff858-cf44-46e2-80a6-89888d32b076.gif?raw=true" alt="Sublime's custom image"/>
</p>

 <p align="center"> 
 Figure: result 2 of Nerf in the wild 
 </p>






## References 
Project site : https://nerf-w.github.io/

An implementation of neural graphics primitives : https://github.com/NVlabs/instant-ngp

Unofficial implementation of NeRF-W : https://github.com/kwea123/nerf_pl/tree/nerfw
