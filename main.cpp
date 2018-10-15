
/*
 * Copyright (C) 2015, Nils Moehrle
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#include <iostream>
#include <fstream>
#include <vector>

#include <util/timer.h>
#include <util/system.h>
#include <util/file_system.h>
#include <mve/mesh_io_ply.h>

#include "tex/util.h"
#include "tex/timer.h"
#include "tex/debug.h"
#include "tex/texturing.h"
#include "tex/progress_counter.h"

//#include "arguments.h"
#include "paramArgs.h"
#include "G2LTexConfig.h"

int main(int argc, char **argv)
{

    paramArgs conf;
    try
    {
        conf = parse_args(argc, argv);//解析参数
    }
    catch (std::invalid_argument & ia)
    {
        std::cerr << ia.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }

    mve::TriangleMesh::Ptr mesh;
    try
    {
        mesh = mve::geom::load_ply_mesh(conf.in_mesh);//读取网格文件。
    }
    catch (std::exception& e)
    {
        std::cerr << "\tCould not load mesh: "<< e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }

    mve::MeshInfo mesh_info(mesh);
    tex::prepare_mesh(&mesh_info, mesh);

    std::cout << "Generating texture views: " << std::endl;
    tex::TextureViews     texture_views;
    tex::generate_texture_views(conf.in_scene, &texture_views);
    std::size_t const num_faces = mesh->get_faces().size() / 3;

    tex::Graph graph(num_faces);

    tex::G2LTexSelection(mesh, mesh_info, &texture_views, &graph, conf.settings);
    tex::G2LsaveLabelModel("orgTexSelection", conf.settings, graph, mesh, mesh_info, texture_views);
    tex::combineSmallPath(graph, mesh, texture_views, false);
    tex::combineSmallPath(graph, mesh, texture_views, false);
    tex::combineSmallPath(graph, mesh, texture_views, true);

    //global Option
    std::vector<tex::myFaceInfo>               faceInfoList;
    std::vector<tex::myViewImageInfo>       viewImageList;
    std::vector<std::vector<std::size_t> >   chart_graph;
    std::vector<std::vector<std::size_t> >   subchartgraphs;
    tex::global_face_option(graph, mesh, mesh_info, texture_views, conf.in_scene, faceInfoList, viewImageList, chart_graph, subchartgraphs, conf.settings);

    //local texture option
    tex::VertexProjectionInfos    in_vertex_projection_infos;
    tex::generate_chart_vertexInfo(graph, mesh, mesh_info, &in_vertex_projection_infos, faceInfoList, subchartgraphs);
    tex::local_vertices_option(graph, mesh, mesh_info, texture_views, conf.in_scene, faceInfoList, viewImageList, chart_graph, subchartgraphs, in_vertex_projection_infos, conf.settings);

    tex::TextureAtlases texture_atlases;
    {
        tex::TexturePatches            texture_patches;
        tex::VertexProjectionInfos    vertex_projection_infos;
        tex::generate_seam_texture_patches(graph, mesh, mesh_info, &texture_views, conf.settings, in_vertex_projection_infos, vertex_projection_infos, &texture_patches, faceInfoList);

        //global color correction
        if (G2LTexConfig::get().global_color_correcting)
        {
            std::cout<<"----Global Texture Color Correction------"<<std::endl;
            tex::globalColorCorrection(graph, mesh, mesh_info, vertex_projection_infos, &texture_patches);
        }

        /* Generate texture atlases. */
        tex::generate_texture_atlases(&texture_patches, conf.settings, &texture_atlases);
    }

    /* Create and write out obj model. */
    {
        tex::Model model;
        tex::build_model(mesh, texture_atlases, &model);
        tex::Model::save(model, "G2LTex");
    }
    tex::G2LsaveLabelModel ("result_label", conf.settings, graph, mesh, mesh_info, texture_views);

    return EXIT_SUCCESS;
}
