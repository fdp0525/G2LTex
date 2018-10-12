# G2LTex

This repesitory contains G2LTex, a implementation of global-to-local texture optimization for 3D Reconstruction with RGB-D Sensor based  [mvs-texturing](https://github.com/nmoehrle/mvs-texturing). More information and the paper can be found [here](http://graphvision.whu.edu.cn/).

# Publication
If you find this deome valuable for your reseach, please cite our works.

> Y. Fu, Q. Yan, L. Yang, J. Liao and X. Chun. Texture Mapping for 3D Reconstruction with RGB-D Sensor. In CVPR. 2018.

# How to use
## 1. run
To test our algorithm. run G2LTex in command lien:
```
./bin/G2LTex [PLY] [DIR]
```
Params explanation:
-`PLY`: The Reconstrucion model for texture mapping.
-`DIR`: The texture image directory, include rgb images, depth images, and camera trajectory.

##2. Input Format
The camera trajectory for each frame is format as follow:

##3. Dependencies
The code and the build system have the following prerequisites:

- gcc (>= 5.0.0)
- PCL
- OpenCV





