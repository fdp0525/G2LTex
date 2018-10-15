/*
 * Copyright (C) 2015, Simon Fuhrmann
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#include <algorithm>
#include <list>
#include <set>

#include "mve/mesh_info.h"
#include <iostream>

MVE_NAMESPACE_BEGIN

void MeshInfo::initialize (TriangleMesh::ConstPtr mesh)
{
    TriangleMesh::VertexList const& verts = mesh->get_vertices();
    TriangleMesh::FaceList const& faces = mesh->get_faces();
    std::size_t face_amount = faces.size() / 3;//每个面对应三个顶点。除以3得到面的个数

//    std::cout<<"------faceseize :"<<faces.size()<<"   face_amount:"<<face_amount<<std::endl;
    this->vertex_info.clear();
    this->vertex_info.resize(verts.size());

    /* Add faces to their three vertices. */
    for (std::size_t i = 0, i3 = 0; i < face_amount; ++i)
    {
        for (std::size_t j = 0; j < 3; ++j, ++i3)
        {
            this->vertex_info[faces[i3]].faces.push_back(i);//每个顶点对应的面的列表
        }
    }

    /* Classify each vertex and compute adjacenty info. */
    for (std::size_t i = 0; i < this->vertex_info.size(); ++i)//遍历所有的顶点
    {
        this->update_vertex(*mesh, i);//更新所有顶点信息，以及邻接面和邻接点的信息
    }
}

/* ---------------------------------------------------------------- */

namespace
{
    /* Adjacent face representation for the ordering algorithm. */
//点的邻接面
    struct AdjacentFace
    {
        std::size_t face_id;//面的索引
        //另外两个点的索引
        std::size_t first;
        std::size_t second;
    };

    typedef std::list<AdjacentFace> AdjacentFaceList;
}

/* ---------------------------------------------------------------- */
/**
 * @brief MeshInfo::update_vertex  更新顶点信息以及邻接面和邻接点信息
 * @param mesh
 * @param vertex_id
 */
void MeshInfo::update_vertex (TriangleMesh const& mesh, std::size_t vertex_id)
{
    TriangleMesh::FaceList const& faces = mesh.get_faces();//得到所有的面
    VertexInfo& vinfo = this->vertex_info[vertex_id];//当前顶点信息

    /* Build new, temporary adjacent faces representation for ordering. */
    AdjacentFaceList adj_temp;
    for (std::size_t i = 0; i < vinfo.faces.size(); ++i)//点邻接面
    {
        std::size_t face_off = vinfo.faces[i] * 3;//邻接面顶点的索引
        for (std::size_t j = 0; j < 3; ++j)
        {
            if (faces[face_off + j] == vertex_id)//当前顶点
            {
                adj_temp.push_back(AdjacentFace());
                adj_temp.back().face_id = vinfo.faces[i];//面的索引
                adj_temp.back().first = faces[face_off + (j + 1) % 3];//另外的两个点的索引
                adj_temp.back().second = faces[face_off + (j + 2) % 3];//另外两个点的索引
                break;
            }
        }
    }

    /* If there are no adjacent faces, the vertex is unreferenced. */
    if (adj_temp.empty())//如果没有邻接面，那么这个点就是没有引用的点
    {
        vinfo = VertexInfo();
        vinfo.vclass = VERTEX_CLASS_UNREF;
        return;
    }

    /* Sort adjacent faces by chaining them. */
    AdjacentFaceList adj_sorted;
    adj_sorted.push_back(adj_temp.front());
    adj_temp.pop_front();
    while (!adj_temp.empty())//把邻接面按顺序串起来
    {
        std::size_t const front_id = adj_sorted.front().first;
        std::size_t const back_id = adj_sorted.back().second;

        /* Find a faces that fits the back or front of sorted list. */
        bool found_face = false;
        for (AdjacentFaceList::iterator iter = adj_temp.begin();
            iter != adj_temp.end(); ++iter)
        {
            if (front_id == iter->second)
            {
                adj_sorted.push_front(*iter);
                adj_temp.erase(iter);
                found_face = true;
                break;
            }
            if (back_id == iter->first)
            {
                adj_sorted.push_back(*iter);
                adj_temp.erase(iter);
                found_face = true;
                break;
            }
        }

        /* If there is no next face, the vertex is complex. */
        if (!found_face)
            break;
    }

    /* If the vertex is complex, add unsorted adjacency information. */
    if (!adj_temp.empty())//没有都遍历完就跳出了，那么就存在多个扇形三角形与这个点相邻
    {
        /* Transfer remaining adjacent faces. */
        adj_sorted.insert(adj_sorted.end(), adj_temp.begin(), adj_temp.end());
        adj_temp.clear();

        /* Create unique list of all adjacent vertices. */
        std::set<std::size_t> vset;
        for (AdjacentFaceList::iterator iter = adj_sorted.begin();
            iter != adj_sorted.end(); ++iter)
        {
            vset.insert(iter->first);
            vset.insert(iter->second);
        }
        vinfo.verts.insert(vinfo.verts.end(), vset.begin(), vset.end());
        vinfo.vclass = VERTEX_CLASS_COMPLEX;//注意这种类型的顶点不存储邻接面
        return;
    }

    /* If the vertex is not on the mesh boundary, the list is circular. */
    if (adj_sorted.front().first == adj_sorted.back().second)//形成了闭环
        vinfo.vclass = VERTEX_CLASS_SIMPLE;
    else
        vinfo.vclass = VERTEX_CLASS_BORDER;//开环扇形

    /* Insert the face IDs in the adjacent faces list. */
    vinfo.faces.clear();
    for (AdjacentFaceList::iterator iter = adj_sorted.begin();iter != adj_sorted.end(); ++iter)
    {
        vinfo.faces.push_back(iter->face_id);//存储邻接面索引
    }

    /* Insert vertex IDs in adjacent vertex list. */
    for (AdjacentFaceList::const_iterator iter = adj_sorted.begin();iter != adj_sorted.end(); ++iter)
    {
        vinfo.verts.push_back(iter->first);
    }
    if (vinfo.vclass == VERTEX_CLASS_BORDER)//开环存储以下最后一个顶点。
    {
        vinfo.verts.push_back(adj_sorted.back().second);
    }
}

/* ---------------------------------------------------------------- */

bool
MeshInfo::is_mesh_edge (std::size_t v1, std::size_t v2) const
{
    AdjacentVertices const& verts = this->vertex_info[v1].verts;
    return std::find(verts.begin(), verts.end(), v2) != verts.end();
}

/* ---------------------------------------------------------------- */
/**
 * @brief MeshInfo::get_faces_for_edge  找到边v1和v2邻接的两个面
 * @param v1
 * @param v2
 * @param adjacent_faces
 */
void MeshInfo::get_faces_for_edge (std::size_t v1, std::size_t v2, std::vector<std::size_t>* adjacent_faces) const
{
    AdjacentFaces const& faces1 = this->vertex_info[v1].faces;//v1点的所有邻接面
    AdjacentFaces const& faces2 = this->vertex_info[v2].faces;//v2点的所有邻接面（也就是面索引）
    std::set<std::size_t> faces2_set(faces2.begin(), faces2.end());
//    std::cout<<"----->faces:"<<v1<<"+"<<faces1.size()<<"  face:"<<v2<<"+"<<faces2.size()<<std::endl;
    for (std::size_t i = 0; i < faces1.size(); ++i)
    {
//        std::cout<<"faces:"<<v1<<" "<<i<<"---->"<<faces1[i]<<std::endl;
        if (faces2_set.find(faces1[i]) != faces2_set.end())
        {
//            std::cout<<v1<<"--"<<v2<<" face:"<<faces1[i]<<std::endl;
            adjacent_faces->push_back(faces1[i]);
        }
    }
}

MVE_NAMESPACE_END
