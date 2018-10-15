/*
 * Copyright (C) 2015, Simon Fuhrmann
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#include <iostream>
#include <iomanip>
#include <sstream>
#include <algorithm>

#include "math/matrix_tools.h"
#include "mve/camera.h"

#include <Eigen/Eigen>

#include "G2LTexConfig.h"

MVE_NAMESPACE_BEGIN

CameraInfo::CameraInfo (void)
{
//    this->flen = 0.0f;
//    this->paspect = 1.0f;
//    std::fill(this->ppoint, this->ppoint + 2, 0.5f);
//    std::fill(this->dist, this->dist + 2, 0.0f);
    //init
    depth_cx = G2LTexConfig::get().DEPTH_CX;
    image_cx = G2LTexConfig::get().IMAGE_CX;
    depth_cy = G2LTexConfig::get().DEPTH_CY;
    image_cy = G2LTexConfig::get().IMAGE_CY;
    depth_fx = G2LTexConfig::get().DEPTH_FX;
    image_fx = G2LTexConfig::get().IMAGE_FX;
    depth_fy = G2LTexConfig::get().DEPTH_FY;;
    image_fy = G2LTexConfig::get().IMAGE_FY;

    std::fill(this->trans, this->trans + 3, 0.0f);
    math::matrix_set_identity(this->rot, 3);
}

/* ---------------------------------------------------------------- */
//相机坐标到世界坐标
void CameraInfo::fill_camera_pos (float* pos) const
{
    //-R^T*t。注意这里需要取逆
    pos[0] = -rot[0] * trans[0] - rot[3] * trans[1] - rot[6] * trans[2];
    pos[1] = -rot[1] * trans[0] - rot[4] * trans[1] - rot[7] * trans[2];
    pos[2] = -rot[2] * trans[0] - rot[5] * trans[1] - rot[8] * trans[2];
}

/* ---------------------------------------------------------------- */

void
CameraInfo::fill_camera_translation (float* trans) const
{
    std::copy(this->trans, this->trans + 3, trans);
}

/* ---------------------------------------------------------------- */
//世界坐标系下也就是相机z轴的朝向
void CameraInfo::fill_viewing_direction (float* viewdir) const
{
    //R^T*[0,0,1],把z轴的朝向变换到世界坐标系下，R取转置后就得到如下的式子
    for (int i = 0; i < 3; ++i)
    {
        viewdir[i] = this->rot[6 + i];
    }
}

/* ---------------------------------------------------------------- */

void CameraInfo::fill_world_to_cam (float* mat) const
{
    Eigen::Matrix4f init;
    init<<rot[0], rot[1], rot[2], trans[0],
             rot[3], rot[4], rot[5], trans[1],
             rot[6], rot[7], rot[8], trans[2],
             0.0, 0.0, 0.0 , 1.0;
    mat[0] = init(0,0); mat[1]  =  init(0,1); mat[2]  =  init(0,2); mat[3]  =  init(0,3);
    mat[4] = init(1,0); mat[5]  =  init(1,1); mat[6]  =  init(1,2); mat[7]  =  init(1,3);
    mat[8] = init(2,0); mat[9]  =  init(2,1); mat[10] =  init(2,2); mat[11] =  init(2,3);
    mat[12] = init(3,0); mat[13] = init(3,1); mat[14] =  init(3,2); mat[15] =  init(3,3);
}

/* ---------------------------------------------------------------- */

void CameraInfo::fill_gl_viewtrans (float* mat) const
{
    mat[0]  =  rot[0]; mat[1]  =  rot[1]; mat[2]  =  rot[2]; mat[3]  =  trans[0];
    mat[4]  = -rot[3]; mat[5]  = -rot[4]; mat[6]  = -rot[5]; mat[7]  = -trans[1];
    mat[8]  = -rot[6]; mat[9]  = -rot[7]; mat[10] = -rot[8]; mat[11] = -trans[2];
    mat[12] =  0.0f;   mat[13] =  0.0f;   mat[14] =  0.0f;   mat[15] = 1.0f;
}

/* ---------------------------------------------------------------- */

void CameraInfo::fill_cam_to_world (float* mat) const
{
    mat[0]  = rot[0]; mat[1] = rot[3]; mat[2]  = rot[6];
    mat[4]  = rot[1]; mat[5] = rot[4]; mat[6]  = rot[7];
    mat[8]  = rot[2]; mat[9] = rot[5]; mat[10] = rot[8];
    mat[3]  = -(rot[0] * trans[0] + rot[3] * trans[1] + rot[6] * trans[2]);
    mat[7]  = -(rot[1] * trans[0] + rot[4] * trans[1] + rot[7] * trans[2]);
    mat[11] = -(rot[2] * trans[0] + rot[5] * trans[1] + rot[8] * trans[2]);
    mat[12] = 0.0f;   mat[13] = 0.0f;   mat[14] = 0.0f;   mat[15] = 1.0f;
}

/* ---------------------------------------------------------------- */

void
CameraInfo::fill_world_to_cam_rot (float* mat) const
{
    std::copy(this->rot, this->rot + 9, mat);
}

/* ---------------------------------------------------------------- */

void
CameraInfo::fill_cam_to_world_rot (float* mat) const
{
    mat[0]  = rot[0]; mat[1] = rot[3]; mat[2] = rot[6];
    mat[3]  = rot[1]; mat[4] = rot[4]; mat[5] = rot[7];
    mat[6]  = rot[2]; mat[7] = rot[5]; mat[8] = rot[8];
}

/* ---------------------------------------------------------------- */

void
CameraInfo::set_transformation (float const* mat)
{
    rot[0] = mat[0]; rot[1] = mat[1]; rot[2] = mat[2];  trans[0] = mat[3];
    rot[3] = mat[4]; rot[4] = mat[5]; rot[5] = mat[6];  trans[1] = mat[7];
    rot[6] = mat[8]; rot[7] = mat[9]; rot[8] = mat[10]; trans[2] = mat[11];
}

/* ---------------------------------------------------------------- */

void   CameraInfo::fill_calibration (float*depthmat, float *imagemat) const
{

    depthmat[0] =   G2LTexConfig::get().DEPTH_FX; depthmat[1] = 0.0f; depthmat[2] = G2LTexConfig::get().DEPTH_CX;
    depthmat[3] =  0.0f; depthmat[4] =   G2LTexConfig::get().DEPTH_FY; depthmat[5] = G2LTexConfig::get().DEPTH_CY;
    depthmat[6] = 0.0f; depthmat[7] = 0.0f; depthmat[8] = 1.0f;

    imagemat[0] =   G2LTexConfig::get().IMAGE_FX; imagemat[1] = 0.0f; imagemat[2] = G2LTexConfig::get().IMAGE_CX;
    imagemat[3] =  0.0f; imagemat[4] =   G2LTexConfig::get().IMAGE_FY; imagemat[5] = G2LTexConfig::get().IMAGE_CY;
    imagemat[6] = 0.0f; imagemat[7] = 0.0f; imagemat[8] = 1.0f;
}

/* ---------------------------------------------------------------- */

void CameraInfo::fill_gl_projection (float* mat, float width, float height, float znear, float zfar) const
{
//    float dim_aspect = width / height;
//    float image_aspect = dim_aspect * this->paspect;
//    float ax, ay;
//    if (image_aspect < 1.0f) /* Portrait. */
//    {
//        ax = this->flen / image_aspect;
//        ay = this->flen;
//    }
//    else /* Landscape */
//    {
//        ax = this->flen;
//        ay = this->flen * image_aspect;
//    }

//    std::fill(mat, mat + 16, 0.0f);

//    mat[4 * 0 + 0] = 2.0f * ax;
//    mat[4 * 0 + 2] = 2.0f * (this->ppoint[0] - 0.5f);
//    mat[4 * 1 + 1] = 2.0f * ay;
//    mat[4 * 1 + 2] = 2.0f * (this->ppoint[1] - 0.5f);
//    mat[4 * 2 + 2] = -(zfar + znear) / (zfar - znear);
//    mat[4 * 2 + 3] = -2.0f * zfar * znear / (zfar - znear);
//    mat[4 * 3 + 2] = -1.0f;
}

/* ---------------------------------------------------------------- */

void CameraInfo::fill_inverse_calibration (float* mat,
    float width, float height) const
{
//    float dim_aspect = width / height;
//    float image_aspect = dim_aspect * this->paspect;
//    float ax, ay;
//    if (image_aspect < 1.0f) /* Portrait. */
//    {
//        ax = this->flen * height / this->paspect;
//        ay = this->flen * height;
//    }
//    else /* Landscape. */
//    {
//        ax = this->flen * width;
//        ay = this->flen * width * this->paspect;
//    }

//    //modify
//    mat[0] = 1.0f / ax; mat[1] = 0.0f; mat[2] = -this->ppoint[0] / ax;
//    mat[3] = 0.0f; mat[4] = 1.0f / ay; mat[5] = -this->ppoint[1] / ay;
//    mat[6] = 0.0f; mat[7] = 0.0f;      mat[8] = 1.0f;
}

/* ---------------------------------------------------------------- */

void CameraInfo::fill_reprojection (CameraInfo const& destination, float src_width, float src_height, float dst_width, float dst_height, float* mat, float* vec) const
{
//    math::Matrix3f dst_K, dst_R, src_Ri, src_Ki;
//    math::Vec3f dst_t, src_t;
//    destination.fill_calibration(dst_K.begin(), dst_width, dst_height);
//    destination.fill_world_to_cam_rot(dst_R.begin());
//    destination.fill_camera_translation(dst_t.begin());
//    this->fill_cam_to_world_rot(src_Ri.begin());
//    this->fill_inverse_calibration(src_Ki.begin(), src_width, src_height);
//    this->fill_camera_translation(src_t.begin());

//    math::Matrix3f ret_mat = dst_K * dst_R * src_Ri * src_Ki;
//    math::Vec3f ret_vec = dst_K * (dst_t - dst_R * src_Ri * src_t);
//    std::copy(ret_mat.begin(), ret_mat.end(), mat);
//    std::copy(ret_vec.begin(), ret_vec.end(), vec);
}

/* ---------------------------------------------------------------- */

std::string
CameraInfo::get_rotation_string (void) const
{
    std::stringstream ss;
    ss << std::setprecision(10);
    for (int i = 0; i < 9; ++i)
        ss << this->rot[i] << (i < 8 ? " " : "");
    return ss.str();
}

std::string
CameraInfo::get_translation_string (void) const
{
    std::stringstream ss;
    ss << std::setprecision(10);
    for (int i = 0; i < 3; ++i)
        ss << this->trans[i] << (i < 2 ? " " : "");
    return ss.str();
}

void
CameraInfo::set_translation_from_string (std::string const& trans_string)
{
    std::stringstream ss(trans_string);
    for (int i = 0; i < 3; ++i)
        ss >> this->trans[i];
}

void
CameraInfo::set_rotation_from_string (std::string const& rot_string)
{
    std::stringstream ss(rot_string);
    for (int i = 0; i < 9; ++i)
        ss >> this->rot[i];
}

/* ---------------------------------------------------------------- */

void
CameraInfo::debug_print (void) const
{
    std::cout << "Extrinsic camera parameters:" << std::endl
        << "  Trans: " << math::Vec3f(this->trans) << std::endl
        << "  Rot: " << math::Vec3f(&this->rot[0]) << std::endl
        << "       " << math::Vec3f(&this->rot[3]) << std::endl
        << "       " << math::Vec3f(&this->rot[6]) << std::endl
        << "Intrinsic camera parameters:" << std::endl
//        << "  Focal Length: " << this->flen << std::endl
//        << "  Principal Point: " << math::Vec2f(this->ppoint) << std::endl
//        << "  Pixel aspect: " << this->paspect << std::endl
//        << "  Radial distortion: " << math::Vec2f(this->dist) << std::endl
        << std::endl;
}

void CameraInfo::setDepthIntrinsic(float fx, float fy, float cx, float cy)
{
    depth_cx = cx;
    depth_cy = cy;
    depth_fx = fx;
    depth_fy = fy;
}

void CameraInfo::setImageIntrinsic(float fx, float fy, float cx, float cy)
{
    image_cx = cx;
    image_cy = cy;
    image_fx = fx;
    image_fy = fy;
}

MVE_NAMESPACE_END
