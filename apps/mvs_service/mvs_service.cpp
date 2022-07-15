

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


class MVSTexService
{
public:
  MVSTexService(ros::NodeHandle& node);
  bool doTextureMapping(vision_tools_msgs::TextureMappingRequest& req,
                        vision_tools_msgs::TextureMappingResponse& res);

  virtual ~MVSTexService();

private:
  ros::NodeHandle pnh_;
  ros::ServiceServer tex_map_srv_;
};
MVSTexService::MVSTexService(ros::NodeHandle& node) : pnh_(node)
{
  tex_map_srv_ = pnh_.advertiseService("mvs_service", &MVSTexService::doTextureMapping, this);
}
MVSTexService::~MVSTexService()
{
}

bool MVSTexService::doTextureMapping(vision_tools_msgs::TextureMappingRequest& req,
                      vision_tools_msgs::TextureMappingResponse& res)
{
  mve::TriangleMesh::Ptr mesh;
  std::vector<tex::TextureView> texture_views;
  readData(req.data_path,texture_views);
  std::cout << "Load and prepare mesh: " << std::endl;
  std::string mesh_file = req.data_path + "/" + req.ply_name_file;
  mesh = mve::geom::load_ply_mesh(mesh_file);
  std::size_t const num_faces = mesh->get_faces().size() / 3;
  mve::MeshInfo mesh_info(mesh);
  tex::prepare_mesh(&mesh_info, mesh);
  std::cout << "Building adjacency graph: " << std::endl;
  tex::Graph graph(num_faces);
  tex::build_adjacency_graph(mesh, mesh_info, &graph);
  std::cout << "View selection:" << std::endl;
  util::WallTimer rwtimer;
  Arguments conf;
  conf.in_scene = req.data_path + "/texture/scene/";
  conf.in_mesh = mesh_file;
  std::string obj_file;
  if (req.object_name.empty())
  {
    obj_file = req.ply_name_file.substr(0, req.ply_name_file.length() - 3);
  }
  else{
    obj_file = req.object_name;
  }
  if (req.output_path.empty()) req.output_path = req.data_path;
  // Create a folder inside output path
  std::string object_model_dir = req.output_path + "/" + req.object_name + "/";
  if (!open3d::utility::filesystem::DirectoryExists(object_model_dir))
  {
    open3d::utility::filesystem::MakeDirectoryHierarchy(object_model_dir);
  }

  conf.out_prefix = object_model_dir + req.object_name;
  conf.data_cost_file = "";
  conf.labeling_file = "";
  conf.write_timings = false;
  conf.write_intermediate_results = true;
  conf.write_view_selection_model = false;
  conf.num_threads = -1;
  tex::DataCosts data_costs(num_faces, texture_views.size());

  tex::calculate_data_costs(mesh, &texture_views, conf.settings, &data_costs);
  tex::view_selection(data_costs, &graph, conf.settings);
  tex::TextureAtlases texture_atlases;
  tex::TexturePatches texture_patches;
  tex::VertexProjectionInfos vertex_projection_infos;
  std::cout << "Generating texture patches:" << std::endl;
  tex::generate_texture_patches(graph, mesh, mesh_info, &texture_views,
      conf.settings, &vertex_projection_infos, &texture_patches);
  std::cout << "Running global seam leveling:" << std::endl;
  tex::global_seam_leveling(graph, mesh, mesh_info, vertex_projection_infos, &texture_patches);
  std::cout << "Running local seam leveling:" << std::endl;
  tex::local_seam_leveling(graph, mesh, vertex_projection_infos, &texture_patches);
  std::cout << "Generating texture atlases:" << std::endl;
  tex::generate_texture_atlases(&texture_patches, conf.settings, &texture_atlases);
  std::cout << "Building objmodel:" << std::endl;
  tex::Model model;
  tex::build_model(mesh, texture_atlases, &model);
  std::cout << "\tSaving model... " << std::flush;
  tex::Model::save(model, conf.out_prefix);
  std::cout << "done." << std::endl;
  res.status.value = res.status.SUCCESS;
  res.status.message = "Successful";
  ROS_INFO("[MVSTexService] Texture Mapping is finished !");
  return true;
}


int main(int argc, char **argv) {

   ros::init(argc, argv, "mvs_service_node");
   ros::NodeHandle node;
   std::shared_ptr<MVSTexService> tex_map = std::make_shared<MVSTexService>(node);
   ROS_INFO("MVS service is ready, wait for client");
   ros::spin();
 

}


