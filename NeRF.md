##  Preparing new NeRF datasets :
### For training from images : 
- step 1  : launch colmap :  
      
      $colmap automatic_reconstructor --workspace_path /work/datas/dossier_projet/ --image_path /work/datas/dossier_projet/images/

- step2 : change the files .bin to .txt 
               
       $/softs/rh7/singularity/3.5.3/bin/singularity exec /work/scratch/caraffl/instant_ngp.simg colmap model_converter  --input_path  /work/scratch/caraffl/datas/dossier_projet/sparse/0/ --output_path  /work/scratch/caraffl/datas/dossier_projet/ --output_type TXT

- step 3  : you have to create a colmap_text folder next to the images folder to put there the 3 .txt files

- step 4  : For training from images, place them in a subfolder called images and then use suitable options such as the ones below:
       
       folder of images  $ python /home/ad/caraffl/code/colmap/instant-ngp/scripts/colmap2nerf.py --colmap_matcher exhaustive  --aabb_scale 16 ( decreasing the value 16, 8, 4, 2, 1)

### For training from video file : 


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



## Testing

     python eval.py   --root_dir /work/scratch/caraffl/datas/kitti/ 
        --dataset_name phototourism --scene_name kitti --split test_train --img_downscale 2 --N_samples 256 --N_importance 256  
        --N_vocab 500 --encode_a --encode_t  --use_disp 
        --ckpt_path /work/scratch/caraffl/datas/nerf_pl-nerfw/ckpts/kitti_2_nerfw/epoch=11.ckpt
        --chunk 16384 --img_wh 640 480 --video_format gif

for example, if you want to use the second GPU
https://github.com/kwea123/nerf_pl/blob/dev/eval.py : just change the .cuda() code to .to('cuda: 1') .
  
