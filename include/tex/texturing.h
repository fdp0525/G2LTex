/*
 * Copyright (C) 2015, Nils Moehrle
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#ifndef TEX_TEXTURING_HEADER
#define TEX_TEXTURING_HEADER

#include <vector>

#include "mve/mesh.h"
#include "mve/mesh_info.h"

#include "mrf/graph.h"

#include "defines.h"
#include "settings.h"
#include "obj_model.h"
#include "uni_graph.h"
#include "texture_view.h"
#include "texture_patch.h"
#include "texture_atlas.h"
#include "sparse_table.h"

#include "seam_leveling.h"


#include <Eigen/Eigen>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <rayint/math/vector.h>
TEX_NAMESPACE_BEGIN

typedef std::vector<TextureView> TextureViews;
typedef std::vector<TexturePatch::Ptr> TexturePatches;
typedef std::vector<TextureAtlas::Ptr> TextureAtlases;
typedef ObjModel Model;
typedef UniGraph Graph;
typedef SparseTable<std::uint32_t, std::uint16_t, float> DataCosts;
typedef std::vector<std::vector<VertexProjectionInfo> > VertexProjectionInfos;
typedef std::vector<std::vector<FaceProjectionInfo> > FaceProjectionInfos;//每个面相对每个视口的属性信息（第一层面的个数，第二层视口的个数）

/** Struct representing a TexturePatch candidate - final texture patches are obtained by merging candiates. */
struct TexturePatchCandidate
{
    Rect<int> bounding_box;
    TexturePatch::Ptr texture_patch;

public:
//    ~TexturePatchCandidate()
//    {
//        std::cout<<"kkkkkkkkk"<<std::endl;
//    }
};

//对相机位置进行优化
struct myFaceInfo
{
    std::size_t   face_id;//面的索引
    std::size_t   chart_id;//对应的chart的索引
    Eigen::Matrix4f   world_to_cam;//当前面到对应的label视口的投影变换矩阵
    int     lable;//对应的视口
    std::vector<math::Vec3f>   samplePoints;
    float              quality;//面的质量（面积或者梯度和）
    Eigen::Matrix<float, 2, 3>  Affine;
    std::map<int, Eigen::Matrix4f>  vertexMats;//int顶点索引，mat对应的变换矩阵
    bool   isChartBoundaryFace;


    myFaceInfo()
    {
        Affine.setZero();
        Affine(0,0) = 1;
        Affine(1,1) = 1;
        quality = 0.0;
        samplePoints.clear();
        lable = 0;
        face_id = -1;
        chart_id = -1;
        world_to_cam = Eigen::Matrix4f::Identity();
        isChartBoundaryFace = false;
    }

    std::vector<math::Vec3f>  generatesampleFromline(math::Vec3f v1, math::Vec3f v2, int ncut)
    {
        std::vector<math::Vec3f>  s;
        for(int i = 1; i < ncut; i++)
        {
            math::Vec3f  v;
            v = (v1*i+v2*(ncut - i))/ncut;
            s.push_back(v);
        }
        return s;
    }

    void generateSample(math::Vec3f  v1,math::Vec3f v2, math::Vec3f v3)
    {
        if(samplePoints.size() == 0)
        {
            samplePoints.push_back(v1);samplePoints.push_back(v2);samplePoints.push_back(v3);
            math::Vec3f   mid1,mid2, mid3, mid4,mid5, mid6, mid7;
            //v1,v2
            mid4 = (v1 + v2)/2.0f; samplePoints.push_back(mid4);
            mid2 = (v1 + mid4)/2.0f; samplePoints.push_back(mid2);
            mid6 = (mid4 + v2)/2.0f; samplePoints.push_back(mid6);
            mid1 = (v1+mid2)/2.0f; samplePoints.push_back(mid1);
            mid3 = (mid2+mid4)/2.0f;samplePoints.push_back(mid3);
            mid5 = (mid4 + mid6)/2.0f;samplePoints.push_back(mid5);
            mid7 = (v2 + mid6)/2.0f; samplePoints.push_back(mid7);
            //v2,v3
            mid4 = (v2 + v3)/2.0f; samplePoints.push_back(mid4);
            mid2 = (v2 + mid4)/2.0f; samplePoints.push_back(mid2);
            mid6 = (mid4 + v3)/2.0f; samplePoints.push_back(mid6);
            mid1 = (v1+mid2)/2.0f; samplePoints.push_back(mid1);
            mid3 = (mid2+mid4)/2.0f;samplePoints.push_back(mid3);
            mid5 = (mid4 + mid6)/2.0f;samplePoints.push_back(mid5);
            mid7 = (v3 + mid6)/2.0f; samplePoints.push_back(mid7);
            //v1,v3
            math::Vec3f   m1,m2, m3, m4,m5, m6, m7;
            m4 = (v1 + v3)/2.0f; samplePoints.push_back(m4);
            m2 = (v1 + m4)/2.0f; samplePoints.push_back(m2);
            m6 = (m4 + v3)/2.0f; samplePoints.push_back(m6);
            m1 = (v1+m2)/2.0f; samplePoints.push_back(m1);
            m3 = (m2+m4)/2.0f;samplePoints.push_back(m3);
            m5 = (m4 + m6)/2.0f;samplePoints.push_back(m5);
            m7 = (v3 + m6)/2.0f; samplePoints.push_back(m7);

            std::vector<math::Vec3f> s2 = generatesampleFromline(mid2, m2, 2);
            samplePoints.insert(samplePoints.end(), s2.begin(), s2.end());
            std::vector<math::Vec3f> s3 = generatesampleFromline(mid3, m3, 3);
            samplePoints.insert(samplePoints.end(), s3.begin(), s3.end());
            std::vector<math::Vec3f> s4 = generatesampleFromline(mid4, m4, 4);
            samplePoints.insert(samplePoints.end(), s4.begin(), s4.end());
            std::vector<math::Vec3f> s5 = generatesampleFromline(mid5, m5, 5);
            samplePoints.insert(samplePoints.end(), s5.begin(), s5.end());
            std::vector<math::Vec3f> s6 = generatesampleFromline(mid6, m6, 6);
            samplePoints.insert(samplePoints.end(), s6.begin(), s6.end());
            std::vector<math::Vec3f> s7 = generatesampleFromline(mid7, m7, 7);
            samplePoints.insert(samplePoints.end(), s7.begin(), s7.end());
        }
    }

    //还需要增加一项这个面上所有的采样点
};

struct myViewImageInfo
{
    std::size_t   view_id;//面的索引
    cv::Mat      img;
    cv::Mat      depth;
    cv::Mat      gradxImg;
    cv::Mat      gradyImg;
};

/**
  * prepares the mesh for texturing
  *  -removes duplicated faces
  *  -ensures normals (face and vertex)
  */
void
prepare_mesh(mve::MeshInfo * mesh_info, mve::TriangleMesh::Ptr mesh);

/**
  * Generates TextureViews from the in_scene.
  */
void
generate_texture_views(std::string const & in_scene, std::vector<TextureView> * texture_views);

/**
  * Builds up the meshes face adjacency graph using the vertex_infos
  */
void
build_adjacency_graph(mve::TriangleMesh::ConstPtr mesh,
    mve::MeshInfo const & mesh_info, UniGraph * graph);

/**
 * Calculates the data costs for each face and texture view combination,
 * if the face is visible within the texture view.
 */
void calculate_data_costs(mve::TriangleMesh::ConstPtr mesh, TextureViews * texture_views, Settings const & settings, DataCosts * data_costs);

void postprocess_face_infos(Settings const & settings, FaceProjectionInfos * projected_face_infos, DataCosts * data_costs);

/**
 * Runs the view selection procedure and saves the labeling in the graph
 */
void view_selection(DataCosts const & data_costs, UniGraph * graph, Settings const & settings);

/**
  * Generates texture patches using the graph to determine adjacent faces with the same label.
  */
void generate_texture_patches(UniGraph const & graph, mve::TriangleMesh::ConstPtr mesh,
                              mve::MeshInfo const & mesh_info,
                              std::vector<TextureView> * texture_views, Settings const & settings,
                              std::vector<std::vector<VertexProjectionInfo> > * vertex_projection_infos,
                              std::vector<TexturePatch::Ptr> * texture_patches);

/**
  * @brief 利用优化后的相机位置来计算纹理块
  */
void generate_option_texture_patches(UniGraph const & graph, mve::TriangleMesh::ConstPtr mesh,
                              mve::MeshInfo const & mesh_info,
                              std::vector<TextureView> * texture_views, Settings const & settings,
                              std::vector<std::vector<VertexProjectionInfo> > * vertex_projection_infos,
                              std::vector<TexturePatch::Ptr> * texture_patches,
                              std::vector<tex::myFaceInfo>   faceInfoList);

void generate_seam_texture_patches(UniGraph const & graph, mve::TriangleMesh::ConstPtr mesh,
                                   mve::MeshInfo const & mesh_info,
                                   std::vector<TextureView> * texture_views, Settings const & settings,
                                   tex::VertexProjectionInfos  chart_vertex_projection_infos,
                                   tex::VertexProjectionInfos  &vertex_projection_infos,
                                   std::vector<TexturePatch::Ptr> * texture_patches,
                                   std::vector<tex::myFaceInfo>   faceInfoList);

void generate_seam_texture_patches_test(UniGraph const & graph, mve::TriangleMesh::ConstPtr mesh,
                                   mve::MeshInfo const & mesh_info,
                                   std::vector<TextureView> * texture_views, Settings const & settings,
                                   tex::VertexProjectionInfos  chart_vertex_projection_infos,
                                   tex::VertexProjectionInfos  &vertex_projection_infos,
                                   std::vector<TexturePatch::Ptr> * texture_patches,
                                   std::vector<tex::myFaceInfo>   faceInfoList);

TexturePatchCandidate generate_chart_candidate(int label,
                                         TextureView const & texture_view,
                                         std::vector<VertexProjectionInfo> &v_infos,
                                         mve::TriangleMesh::ConstPtr mesh,
                                         Settings const & settings);

//计算每个顶点的投影信息为下一步局部优化做准备
void generate_vertex_info(UniGraph const & graph,
                          mve::TriangleMesh::ConstPtr mesh,
                          mve::MeshInfo const & mesh_info,
                          std::vector<TextureView> * texture_views,
                          Settings const & settings,
                          std::vector<std::vector<VertexProjectionInfo> > * vertex_projection_infos,
                          std::vector<tex::myFaceInfo>   faceInfoList);
/**
  * Runs the seam leveling procedure proposed by Ivanov and Lempitsky
  * [<A HREF="https://www.google.de/url?sa=t&rct=j&q=&esrc=s&source=web&cd=1&cad=rja&sqi=2&ved=0CC8QFjAA&url=http%3A%2F%2Fwww.robots.ox.ac.uk%2F~vilem%2FSeamlessMosaicing.pdf&ei=_ZbvUvSZIaPa4ASi7IGAAg&usg=AFQjCNGd4x5HnMMR68Sn2V5dPgmqJWErCA&sig2=4j47bXgovw-uks9LBGl_sA">Seamless mosaicing of image-based texture maps</A>]
  */
void global_seam_leveling(UniGraph const & graph, mve::TriangleMesh::ConstPtr mesh,
                          mve::MeshInfo const & mesh_info,
                          std::vector<std::vector<VertexProjectionInfo> > const & vertex_projection_infos,
                          std::vector<TexturePatch::Ptr> * texture_patches);

void
local_seam_leveling(UniGraph const & graph, mve::TriangleMesh::ConstPtr mesh,
                    VertexProjectionInfos const & vertex_projection_infos,
                    std::vector<TexturePatch::Ptr> * texture_patches);

void generate_texture_atlases(std::vector<TexturePatch::Ptr> * orig_texture_patches,
                              Settings const & settings, std::vector<TextureAtlas::Ptr> * texture_atlases);

/**
  * Builds up an model for the mesh by constructing materials and
  * texture atlases form the texture_patches
  */
void build_model(mve::TriangleMesh::ConstPtr mesh,
            std::vector<TextureAtlas::Ptr> const & texture_atlases, ObjModel * obj_model);


//add by fuyp 2017-09-01
void campos_patch_opt(UniGraph const & graph, mve::TriangleMesh::ConstPtr mesh, mve::MeshInfo const & mesh_info,
                     std::vector<TextureView> texture_views, std::string const & indir, std::vector<myFaceInfo> &faceInfoList);


//add by fuyp 2017-09-18
void local_vertices_option(UniGraph const & graph, mve::TriangleMesh::ConstPtr mesh, mve::MeshInfo const & mesh_info,
                         std::vector<TextureView> texture_views, std::string const & indir, std::vector<myFaceInfo> &faceInfoList,
                         std::vector<myViewImageInfo>  viewImageList, std::vector<std::vector<std::size_t> >   patch_graph,
                         std::vector<std::vector<std::size_t> >   subgraphs,
                         std::vector<std::vector<VertexProjectionInfo> > &vertex_projection_infos, Settings const & settings);

void merge_vertex_infos(std::vector<VertexProjectionInfo> &vertex_projection_infos);

void generate_chart_vertexInfo(UniGraph const & graph,
                                     mve::TriangleMesh::ConstPtr mesh,
                                     mve::MeshInfo const & mesh_info,
                                     std::vector<std::vector<VertexProjectionInfo> > * vertex_projection_infos,
                                     std::vector<tex::myFaceInfo>   faceInfoList,
                                     std::vector<std::vector<std::size_t> >   subgraphs);

void build_patch_adjacency_graph(UniGraph const & graph, std::vector<std::vector<std::size_t> > subgraphs, std::vector<int>  labelPatchCount,
                                 std::vector<std::vector<std::size_t> >   &patch_graph, std::vector<myFaceInfo>   &faceInfoList);

void combineSmallPath(UniGraph & graph, mve::TriangleMesh::ConstPtr mesh, std::vector<TextureView> texture_views, bool zoreflag = false);

void global_face_option(UniGraph const & graph, mve::TriangleMesh::ConstPtr mesh, mve::MeshInfo const & mesh_info,
                     std::vector<TextureView> texture_views, std::string const & indir, std::vector<myFaceInfo>   &faceInfoList,
                     std::vector<myViewImageInfo>&  viewImageList, std::vector<std::vector<std::size_t> >  &patch_graph,
                     std::vector<std::vector<std::size_t> > &subgraphs, Settings const & settings);

/**
 * @brief G2LTexSelection
 * @param mesh
 * @param mesh_info
 * @param texture_views
 * @param graph
 * @param settings
 */
void G2LTexSelection(mve::TriangleMesh::ConstPtr mesh, mve::MeshInfo const & mesh_info, std::vector<TextureView> * texture_views, UniGraph * graph , Settings const & settings);

void globalColorCorrection(UniGraph const & graph, mve::TriangleMesh::ConstPtr mesh,
                                mve::MeshInfo const & mesh_info,
                                std::vector<std::vector<VertexProjectionInfo> > const & vertex_projection_infos,
                                std::vector<TexturePatch::Ptr> * texture_patches);
/**
 * @brief G2LsaveLabelModel Save texture selection result for model
 * @param name
 * @param settings
 * @param graph
 * @param mesh
 * @param mesh_info
 * @param texture_views
 */
void G2LsaveLabelModel(std::string name, Settings const & settings, UniGraph const & graph, mve::TriangleMesh::ConstPtr mesh, mve::MeshInfo const & mesh_info,
                    std::vector<tex::TextureView> texture_views);

void local_face_seam(UniGraph const & graph, mve::TriangleMesh::ConstPtr mesh,
                     mve::MeshInfo const & mesh_info,
                     std::vector<std::vector<VertexProjectionInfo> > & vertex_projection_infos,
                     std::vector<tex::myFaceInfo>  &faceInfoList,
                     std::vector<tex::myViewImageInfo>  viewImageList);

void generateChartsLocalOption(UniGraph const & graph,
                    mve::TriangleMesh::ConstPtr mesh,
                    mve::MeshInfo const & mesh_info,
                    std::vector<TextureView> * texture_views,
                    Settings const & settings,
                    std::vector<std::vector<VertexProjectionInfo> > * vertex_projection_infos,
                    std::vector<TexturePatch::Ptr> * texture_patches,
                    std::vector<tex::myFaceInfo>   faceInfoList);

void generateLocalOptionTexture(UniGraph const & graph,
                    mve::TriangleMesh::ConstPtr mesh,
                    mve::MeshInfo const & mesh_info,
                    std::vector<TextureView> * texture_views,
                    Settings const & settings,
                    std::vector<std::vector<VertexProjectionInfo> > * vertex_projection_infos,
                    std::vector<TexturePatch::Ptr> * texture_patches,
                    std::vector<tex::myFaceInfo>   faceInfoList);
//add 2017-10-31
void local_face_option(UniGraph const & graph, mve::TriangleMesh::ConstPtr mesh,
                     mve::MeshInfo const & mesh_info,
                     std::vector<std::vector<VertexProjectionInfo> > & vertex_projection_infos,
                     std::vector<tex::myFaceInfo>  &faceInfoList,
                     std::vector<tex::myViewImageInfo>  viewImageList);

//tools
float getInterColorFromRGBImgV2(cv::Mat  img, float u, float v);
float getInterColorFromGrayImgV2(cv::Mat  img, float u, float v);
void generateGradImgV2(cv::Mat&  in, cv::Mat& outx, cv::Mat& outy);
bool checkNAN(Eigen::Matrix4f mat);

TEX_NAMESPACE_END

#endif /* TEX_TEXTURING_HEADER */
