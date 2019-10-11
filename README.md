# G2LTex

This repository contains the implementation of "<i>Texture Mapping for 3D Reconstruction with RGB-D Sensor (CVPR2018)</i>" based on [mvs-texturing](https://github.com/nmoehrle/mvs-texturing). Due to the agreement with other company, some parts can only be released in the form of .so files. More information and the paper can be found on [our group website](http://graphvision.whu.edu.cn/) and [Qingan's homepage](https://yanqingan.github.io/).

# Publication
If you find this code useful for your research, please cite our work:

> Yanping Fu, Qingan Yan, Long Yang, Jie Liao, Chunxia Xiao. <i>Texture Mapping for 3D Reconstruction with RGB-D Sensor</i>. In CVPR. 2018.

<pre><code>@inproceedings{fu2018texture,
  title={Texture Mapping for 3D Reconstruction with RGB-D Sensor},
  author={Fu, Yanping and Yan, Qingan and Yang, Long and Liao, Jie and Xiao, Chunxia},
  booktitle={Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition (CVPR)},
  pages={4645--4653},
  year={2018},
  organization={IEEE}
}</code></pre>

# How to use

## 1. Run
To test our algorithm. run G2LTex in command line:
```
./bin/G2LTex [DIR] [PLY] 
```
Params explanation:
-`PLY`: The reconstructed model for texture mapping.
-`DIR`: The texture image directory, include rgb images, depth images, and camera trajectory.

The parameters of the camera and the system can be set in the config file.
```
Config/config.yml
```

How to install and run this code.
```
git clone https://github.com/fdp0525/G2LTex.git
cd G2LTex/bin
./G2LTex ../Data/bloster/textureimages ../Data/bloster/bloster.ply
```
We need to modify the configuration file ```config.yml``` before running the other datasets. 
```
./G2LTex ../Data/apt0/apt0 ../Data/apt0/apt0.ply
```

## 2. Input Format
- Color frames (color_XX.jpg): RGB, 24-bit, JPG.
- Depth frames (depth_XX.png): depth (mm), 16-bit, PNG (invalid depth is set to 0).
- Camera poses (color_XX.cam): world-to-camera [tx, ty, tz, R00, R01, R02, R10, R11, R12, R20, R21, R22].


## 3. Dependencies
The code has following prerequisites:
- ubuntu 16.04
- gcc (5.4.0)
- OpenCV (2.4.10)
- Eigen (>3.0)
- png12
- jpeg

## 4. Parameters
All the parameters can be set in the file ```Config/config.yml``` as follows:
```
%YAML:1.0
depth_fx: 540.69
depth_fy: 540.69
depth_cx: 479.75
depth_cy: 269.75
depth_width: 960
depth_height: 540

RGB_fx: 1081.37
RGB_fy: 1081.37
RGB_cx: 959.5
RGB_cy: 539.5
RGB_width: 1920
RGB_height: 1080
.
.
.
```
## 5. Results
Some precomputed results can be found in the folder ```results/```.






