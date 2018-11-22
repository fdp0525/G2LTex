# G2LTex

This repesitory contains G2LTex, a implementation of global-to-local texture optimization for 3D Reconstruction with RGB-D Sensor based on [mvs-texturing](https://github.com/nmoehrle/mvs-texturing). More information and the paper can be found [here](http://graphvision.whu.edu.cn/).

# How to use

## 1. run
To test our algorithm. run G2LTex in command lien:
```
./bin/G2LTex [DIR] [PLY] 
```
Params explanation:
-`PLY`: The Reconstrucion model for texture mapping.
-`DIR`: The texture image directory, include rgb images, depth images, and camera trajectory.

The parameters of the camera and the system can be set in the configure file.
```
Config/config.yml
```

How install and run this code.
```
git clone https://github.com/fdp0525/G2LTex.git
cd G2LTex/bin
./G2LTex ../Data/bloster/textureimages ../Data/bloster/bloster.ply
```
## 2. Input Format
- Color frames (color_XX.jpg): RGB, 24-bit, JPG.
- Depth frames (depth-XX.png): depth (mm), 16-bit, PNG (invalid depth is set to 0).
- Camera poses (color_XX.cam): world-to-camera [tx, ty, tz, R00, R01, R02, R10, R11, R12, R20, R21, R22].


## 3. Dependencies
The code and the build system have the following prerequisites:
- ubuntu 16.04
- gcc (5.4.0)
- OpenCV (2.4.10)
- Eigen(>3.0)
- png12
- jpeg

## 4. Parameters
All the parameters can be set in file ```Config/config.yml``` as follows:
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
## 5. results
Some results are shown under the folder ```results/```.

# Publication
If you find this deome valuable for your reseach, please cite our works.

> Y. Fu, Q. Yan, L. Yang, J. Liao and X. Chun. Texture Mapping for 3D Reconstruction with RGB-D Sensor. In CVPR. 2018.



