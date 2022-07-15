

#include <iostream>
#include <fstream>
#include <vector>
#include <tbb/task_scheduler_init.h>

#include <util/timer.h>
#include <util/system.h>
#include <util/file_system.h>
#include <mve/mesh_io_ply.h>
#include <mve/image_tools.h>
#include <mve/image_io.h>
#include <mve/mesh.h>
#include <mve/mesh_io.h>
#include "tex/util.h"
#include "tex/timer.h"
#include "tex/debug.h"
#include "tex/texturing.h"
#include "tex/progress_counter.h"
#include <open3d/Open3D.h>
#include <opencv2/opencv.hpp>
#include "vision_tools_msgs/TextureMapping.h"
#include <ros/ros.h>
#include <ros/package.h>
#include "arguments.h"

void readData(const std::string data_path,
              std::vector<tex::TextureView>& texture_views)
{

  std::string scene_dir = data_path + "/texture/scene/";


  std::string const tmp_dir = data_path+"/tmp_dir/";
  if (open3d::utility::filesystem::DirectoryExists(tmp_dir)) {
    open3d::utility::filesystem::DeleteDirectory(tmp_dir);
    open3d::utility::filesystem::MakeDirectory(tmp_dir);
  } else {
    open3d::utility::filesystem::MakeDirectory(tmp_dir);
  }


  std::cout << "Generating texture views: " << std::endl;
  tex::generateTextureViews(scene_dir, texture_views);

}


typedef SparseTable<std::uint32_t, std::uint16_t, float> DataCos;

int main(int argc, char **argv) {



  std::string data_path = "/home/manhha/aibox_data/scan_data1/";
  std::string ply_file = data_path + "test_object.ply";
  open3d::geometry::TriangleMesh o3d_mesh;


  mve::TriangleMesh::Ptr mesh;
  std::vector<tex::TextureView> texture_views;
  readData(data_path,texture_views);
  std::cout << "Load and prepare mesh: " << std::endl;
  std::string mesh_file = ply_file;
  mesh = mve::geom::load_ply_mesh(mesh_file);
  open3d::io::ReadTriangleMesh(ply_file,o3d_mesh);
  //mesh.get_fa
  auto faces = mesh->get_faces();
  std::size_t face_amount = faces.size() / 3;


  std::size_t const num_faces = mesh->get_faces().size() / 3;
  mve::MeshInfo mesh_info(mesh);
  tex::prepare_mesh(&mesh_info, mesh);

  tex::Graph graph(num_faces);
  tex::build_adjacency_graph(mesh, mesh_info, &graph);

  std::cout << "View selection:" << std::endl;
  util::WallTimer rwtimer;
  Arguments conf;
  conf.in_scene = data_path + "/texture/scene/";
  conf.in_mesh = ply_file;
  conf.out_prefix = "texture";
  conf.data_cost_file = "";
  conf.labeling_file = "";

  conf.write_timings = false;
  conf.write_intermediate_results = true;
  conf.write_view_selection_model = false;

  conf.num_threads = -1;
 for(int i = 0; i < texture_views.size(); i++)
 {
    std::cout<<texture_views[i].get_pos()<<std::endl;
    std::cout<<texture_views[i].get_viewing_direction()<<std::endl;
    ///mve::image::save_file(texture_views[0].validity_mask,"validity_mask_"+std::to_string(i)+".png");

 }
  tex::DataCosts data_costs(num_faces, texture_views.size());
  tex::FaceProjectionInfos face_projection_infos(num_faces);
  tex::calculate_face_projection_infos(mesh,o3d_mesh, &texture_views, conf.settings, &face_projection_infos);

  tex::calculate_data_costs(mesh, &texture_views, conf.settings, &data_costs);


  tex::view_selection(data_costs, &graph, conf.settings);
//  tex::TextureAtlases texture_atlases;
//  tex::TexturePatches texture_patches;
//  tex::VertexProjectionInfos vertex_projection_infos;
//  std::cout << "Generating texture patches:" << std::endl;
//  tex::generate_texture_patches(graph, mesh, mesh_info, &texture_views,
//      conf.settings, &vertex_projection_infos, &texture_patches);
//  std::cout << "Running global seam leveling:" << std::endl;
//  tex::global_seam_leveling(graph, mesh, mesh_info, vertex_projection_infos, &texture_patches);
//  std::cout << "Running local seam leveling:" << std::endl;
//  tex::local_seam_leveling(graph, mesh, vertex_projection_infos, &texture_patches);
//  std::cout << "Generating texture atlases:" << std::endl;
//  tex::generate_texture_atlases(&texture_patches, conf.settings, &texture_atlases);
//  std::cout << "Building objmodel:" << std::endl;
//  tex::Model model;
//  tex::build_model(mesh, texture_atlases, &model);
//  std::cout << "\tSaving model... " << std::flush;
//  tex::Model::save(model, conf.out_prefix);
//  std::cout << "done." << std::endl;

  return EXIT_SUCCESS;

}


