// Copyright (c) 2014 Bauhaus-Universitaet Weimar
// This Software is distributed under the Modified BSD License, see license.txt.
//
// Virtual Reality and Visualization Research Group 
// Faculty of Media, Bauhaus-Universitaet Weimar
// http://www.uni-weimar.de/medien/vr


//lamure
#include <lamure/types.h>
#include <lamure/ren/config.h>
#include <lamure/ren/model_database.h>
#include <lamure/ren/cut_database.h>
#include <lamure/ren/dataset.h>
#include <lamure/ren/policy.h>
#include <lamure/ren/controller.h>
#include <lamure/pvs/pvs_database.h>

//schism
#include <scm/core/math.h>
#include <scm/core.h>
#include <scm/core/io/tools.h>
#include <scm/core/pointer_types.h>
#include <scm/gl_core/gl_core_fwd.h>
#include <scm/gl_util/primitives/primitives_fwd.h>
#include <scm/core/platform/platform.h>
#include <scm/core/utilities/platform_warning_disable.h>
#include <scm/gl_core/render_device/opengl/gl_core.h>
#include <scm/gl_util/primitives/quad.h>

#include <GL/freeglut.h>

//boost
#include <boost/assign/list_of.hpp>
#include <boost/regex.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>

#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <vector>
#include <algorithm>

//fwd
void initialize_glut(int argc, char** argv, uint32_t width, uint32_t height);
void glut_display();
void glut_resize(int w, int h);
void glut_mousefunc(int button, int state, int x, int y);
void glut_mousemotion(int x, int y);
void glut_idle();
void glut_keyboard(unsigned char key, int x, int y);
void glut_keyboard_release(unsigned char key, int x, int y);
void glut_specialfunc(int key, int x, int y);
void glut_specialfunc_release(int key, int x, int y);
void glut_close();

uint32_t update_ms_ = 16;
bool rendering_ = false;
int32_t window_width_ = 1280;
int32_t window_height_ = 720;
float frame_div = 1;
uint64_t frame_ = 0;
bool fast_travel_ = false;

float near_plane_ = 0.01f;
float far_plane_ = 60.f;

float trackball_x_ = 0.f;
float trackball_y_ = 0.f;
float dolly_sens_ = 10.f;
float fov_ = 30.f;

std::vector<std::string> input_files_;

std::vector<scm::math::mat4d> model_transformations_;
int32_t num_models_ = 0;
int32_t selected_model_ = -1;

float error_threshold_ = 2.0f;

float height_divided_by_top_minus_bottom_ = 0.f;

lamure::ren::camera::mouse_state mouse_state_;

lamure::ren::camera* camera_ = nullptr;

scm::shared_ptr<scm::gl::render_device> device_;
scm::shared_ptr<scm::gl::render_context> context_;

scm::gl::program_ptr vis_xyz_shader_;
scm::gl::program_ptr vis_xyz_pass1_shader_;
scm::gl::program_ptr vis_xyz_pass2_shader_;
scm::gl::program_ptr vis_xyz_pass3_shader_;

scm::gl::frame_buffer_ptr fbo_;
scm::gl::texture_2d_ptr fbo_color_buffer_;
scm::gl::texture_2d_ptr fbo_depth_buffer_;

scm::gl::frame_buffer_ptr pass1_fbo_;
scm::gl::texture_2d_ptr pass1_depth_buffer_;
scm::gl::frame_buffer_ptr pass2_fbo_;
scm::gl::texture_2d_ptr pass2_color_buffer_;
scm::gl::texture_2d_ptr pass2_depth_buffer_;

scm::gl::texture_2d_ptr gaussian_texture_;

scm::gl::depth_stencil_state_ptr depth_state_disable_;
scm::gl::depth_stencil_state_ptr depth_state_less_;
scm::gl::rasterizer_state_ptr no_backface_culling_rasterizer_state_;
scm::gl::rasterizer_state_ptr change_point_size_in_shader_state_;

scm::gl::blend_state_ptr color_blending_state_;
scm::gl::blend_state_ptr color_no_blending_state_;

scm::gl::sampler_state_ptr filter_linear_;
scm::gl::sampler_state_ptr filter_nearest_;

scm::gl::program_ptr quad_shader_;
scm::shared_ptr<scm::gl::quad_geometry> screen_quad_;

lamure::ren::Data_Provenance data_provenance_;

struct settings {
  int32_t width_;
  int32_t height_;
  int32_t vram_;
  int32_t ram_;
  int32_t upload_;
  int32_t prov_;
  int32_t splatting_;
  int32_t gamma_correction_;
  int32_t show_normals_;
  int32_t show_accuracy_;
  int32_t channel_;
  float point_size_;
  int32_t heatmap_;
  float heatmap_min_;
  float heatmap_max_;
  scm::math::vec3f heatmap_color_min_;
  scm::math::vec3f heatmap_color_max_;
  std::string json_;
  std::string pvs_;
  std::vector<std::string> models_;

};

settings settings_;

void load_settings(std::string const& vis_file_name, settings& settings) {

  std::ifstream vis_file(vis_file_name.c_str());

  if (!vis_file.is_open()) {
    std::cout << "could not open vis file" << std::endl;
    exit(-1);
  }
  else {
    lamure::model_t model_id = 0;

    std::string line;
    while(std::getline(vis_file, line)) {
      if(line.length() >= 2) {
        if (line[0] == '#') {
          continue;
        }
        std::istringstream line_ss(line);
        
        auto colon = line.find_first_of(':');
        if (colon == std::string::npos) {
          //std::cout << "lod: " << line << std::endl;
          settings.models_.push_back(line);

        }
        else {
          std::string key = line.substr(0, colon);
          key.erase(std::remove(key.begin(), key.end(), ' '), key.end());
          std::string value = line.substr(colon+1);
          value.erase(std::remove(value.begin(), value.end(), ' '), value.end());
          if (key == "width") {
            settings.width_ = std::max(atoi(value.c_str()), 64);
          }
          else if (key == "height") {
            settings.height_ = std::max(atoi(value.c_str()), 64);
          }
          else if (key == "vram") {
            settings.vram_ = std::max(atoi(value.c_str()), 8);
          }
          else if (key == "ram") {
            settings.ram_ = std::max(atoi(value.c_str()), 8);
          }
          else if (key == "upload") {
            settings.upload_ = std::max(atoi(value.c_str()), 8);
          }
          else if (key == "splatting") {
            settings.splatting_ = std::max(atoi(value.c_str()), 0);
          }
          else if (key == "gamma_correction") {
            settings.gamma_correction_ = std::max(atoi(value.c_str()), 0);
          }
          else if (key == "provenance") {
            settings.prov_ = std::max(atoi(value.c_str()), 0);
          }
          else if (key == "show_normals") {
            settings.show_normals_ = std::max(atoi(value.c_str()), 0);
          }
          else if (key == "show_accuracy") {
            settings.show_accuracy_ = std::max(atoi(value.c_str()), 0);
          }
          else if (key == "channel") {
            settings.channel_ = std::max(atoi(value.c_str()), 0);
          }
          else if (key == "point_size") {
            settings.point_size_ = std::min(std::max(atof(value.c_str()), 0.0), 10.0);
          }
          else if (key == "heatmap") {
            settings.heatmap_ = std::max(atoi(value.c_str()), 0);
          }
          else if (key == "heatmap_min") {
            settings.heatmap_min_ = std::max(atof(value.c_str()), 0.0);
          }
          else if (key == "heatmap_max") {
            settings.heatmap_max_ = std::max(atof(value.c_str()), 0.0);
          }
          else if (key == "heatmap_min_r") {
            settings.heatmap_color_min_.x = std::min(std::max(atoi(value.c_str()), 0), 255)/255.f;
          }
          else if (key == "heatmap_min_g") {
            settings.heatmap_color_min_.y = std::min(std::max(atoi(value.c_str()), 0), 255)/255.f;
          }
          else if (key == "heatmap_min_b") {
            settings.heatmap_color_min_.z = std::min(std::max(atoi(value.c_str()), 0), 255)/255.f;
          }
          else if (key == "heatmap_max_r") {
            settings.heatmap_color_max_.x = std::min(std::max(atoi(value.c_str()), 0), 255)/255.f;
          }
          else if (key == "heatmap_max_g") {
            settings.heatmap_color_max_.y = std::min(std::max(atoi(value.c_str()), 0), 255)/255.f;
          }
          else if (key == "heatmap_max_b") {
            settings.heatmap_color_max_.z = std::min(std::max(atoi(value.c_str()), 0), 255)/255.f;
          }
          else if (key == "json") {
            settings.json_ = value;
          }

          //std::cout << key << " : " << value << std::endl;
        }

      }
    }
    vis_file.close();
  }

  //assertions
  if (settings.prov_ != 0) {
    if (settings.json_ == "") {
      std::cout << "error: pls provide a provenance json description or set prov to 0" << std::endl;
      exit(-1);
    }
  }
  if (settings.models_.empty()) {
    std::cout << "error: no model filename specified" << std::endl;
    exit(-1);
  }


}

char* get_cmd_option(char** begin, char** end, const std::string& option) {
  char** it = std::find(begin, end, option);
  if (it != end && ++it != end) {
    return *it;
  }
  return 0;
}

bool cmd_option_exists(char** begin, char** end, const std::string& option) {
  return std::find(begin, end, option) != end;
}

static void glut_timer(int e) {
  glutPostRedisplay();
  glutTimerFunc(update_ms_, glut_timer, 1);
}

void draw_all_models(const lamure::context_t context_id, const lamure::view_t view_id, scm::gl::program_ptr shader) {

  lamure::ren::controller* controller = lamure::ren::controller::get_instance();
  lamure::ren::cut_database* cuts = lamure::ren::cut_database::get_instance();
  lamure::ren::model_database* database = lamure::ren::model_database::get_instance();

  if (lamure::ren::policy::get_instance()->size_of_provenance() > 0) {
    context_->bind_vertex_array(
      controller->get_context_memory(context_id, lamure::ren::bvh::primitive_type::POINTCLOUD, device_, data_provenance_));
  }
  else {
   context_->bind_vertex_array(
      controller->get_context_memory(context_id, lamure::ren::bvh::primitive_type::POINTCLOUD, device_)); 
  }
  context_->apply();
  
  for (int32_t model_id = 0; model_id < num_models_; ++model_id) {
    if (selected_model_ != -1) {
      model_id = selected_model_;
    }
    lamure::ren::cut& cut = cuts->get_cut(context_id, view_id, model_id);
    std::vector<lamure::ren::cut::node_slot_aggregate> renderable = cut.complete_set();
    const lamure::ren::bvh* bvh = database->get_model(model_id)->get_bvh();
    if (bvh->get_primitive() != lamure::ren::bvh::primitive_type::POINTCLOUD) {
      continue;
    }
    
    //uniforms per model
    scm::math::mat4d model_matrix = model_transformations_[model_id];
    scm::math::mat4d projection_matrix = scm::math::mat4d(camera_->get_projection_matrix());
    scm::math::mat4d view_matrix = camera_->get_high_precision_view_matrix();
    scm::math::mat4d model_view_matrix = view_matrix * model_matrix;
    scm::math::mat4d model_view_projection_matrix = projection_matrix * model_view_matrix;

    shader->uniform("mvp_matrix", scm::math::mat4f(model_view_projection_matrix));
    shader->uniform("model_view_matrix", scm::math::mat4f(model_view_matrix));
    shader->uniform("inv_mv_matrix", scm::math::mat4f(scm::math::transpose(scm::math::inverse(model_view_matrix))));

    size_t surfels_per_node = database->get_primitives_per_node();
    std::vector<scm::gl::boxf>const & bounding_box_vector = bvh->get_bounding_boxes();
    
    scm::gl::frustum frustum_by_model = camera_->get_frustum_by_model(scm::math::mat4f(model_transformations_[model_id]));
    
    for(auto const& node_slot_aggregate : renderable) {
      uint32_t node_culling_result = camera_->cull_against_frustum(
        frustum_by_model,
        bounding_box_vector[node_slot_aggregate.node_id_]);
        
      if (node_culling_result != 1) {
        
        if (settings_.show_accuracy_) {
          const float accuracy = 1.0 - (bvh->get_depth_of_node(node_slot_aggregate.node_id_) * 1.0)/(bvh->get_depth() - 1);// 0...1
          shader->uniform("accuracy", accuracy);
        }

        context_->draw_arrays(scm::gl::PRIMITIVE_POINT_LIST,
          (node_slot_aggregate.slot_id_) * (GLsizei)surfels_per_node, surfels_per_node);
      
      }
    }
    if (selected_model_ != -1) {
      break;
    }
  }

}

void set_uniforms(scm::gl::program_ptr shader) {

    shader->uniform("win_size", scm::math::vec2f(window_width_, window_height_));

    shader->uniform("height_divided_by_top_minus_bottom", height_divided_by_top_minus_bottom_);
    shader->uniform("near_plane", near_plane_);
    shader->uniform("far_minus_near_plane", far_plane_-near_plane_);
    shader->uniform("point_size_factor", settings_.point_size_);

    shader->uniform("ellipsify", true);
    shader->uniform("clamped_normal_mode", true);
    shader->uniform("max_deform_ratio", 0.35f);

    shader->uniform("show_normals", (bool)settings_.show_normals_);
    shader->uniform("show_accuracy", (bool)settings_.show_accuracy_);

    shader->uniform("channel", settings_.channel_);
    shader->uniform("heatmap", (bool)settings_.heatmap_);

    shader->uniform("heatmap_min", settings_.heatmap_min_);
    shader->uniform("heatmap_max", settings_.heatmap_max_);
    shader->uniform("heatmap_min_color", settings_.heatmap_color_min_);
    shader->uniform("heatmap_max_color", settings_.heatmap_color_max_);

    
}

void glut_display() {
  if (rendering_) {
    return;
  }
  rendering_ = true;

  lamure::ren::model_database* database = lamure::ren::model_database::get_instance();
  lamure::ren::cut_database* cuts = lamure::ren::cut_database::get_instance();
  lamure::ren::controller* controller = lamure::ren::controller::get_instance();
  

  bool signal_shutdown = false;
  if (lamure::ren::policy::get_instance()->size_of_provenance() > 0) {
    controller->reset_system(data_provenance_);
  }
  else {
    controller->reset_system();
  }
  lamure::context_t context_id = controller->deduce_context_id(0);
  
  
  for (lamure::model_t model_id = 0; model_id < num_models_; ++model_id) {
    lamure::model_t m_id = controller->deduce_model_id(std::to_string(model_id));

    cuts->send_transform(context_id, m_id, scm::math::mat4f(model_transformations_[m_id]));
    cuts->send_threshold(context_id, m_id, error_threshold_);
    cuts->send_rendered(context_id, m_id);
    
    database->get_model(m_id)->set_transform(scm::math::mat4f(model_transformations_[m_id]));
  }


  lamure::view_t cam_id = controller->deduce_view_id(context_id, camera_->view_id());
  cuts->send_camera(context_id, cam_id, *camera_);

  std::vector<scm::math::vec3d> corner_values = camera_->get_frustum_corners();
  double top_minus_bottom = scm::math::length((corner_values[2]) - (corner_values[0]));
  height_divided_by_top_minus_bottom_ = lamure::ren::policy::get_instance()->window_height() / top_minus_bottom;

  cuts->send_height_divided_by_top_minus_bottom(context_id, cam_id, height_divided_by_top_minus_bottom_);
 
  if (lamure::ren::policy::get_instance()->size_of_provenance() > 0) {
    controller->dispatch(context_id, device_, data_provenance_);
  }
  else {
    controller->dispatch(context_id, device_); 
  }
  lamure::view_t view_id = controller->deduce_view_id(context_id, camera_->view_id());
 
  if (settings_.splatting_) {
    //2 pass splatting
    //PASS 1

    context_->clear_color_buffer(pass1_fbo_ , 0, scm::math::vec4f( .0f, .0f, .0f, 0.0f));
    context_->clear_depth_stencil_buffer(pass1_fbo_);
    context_->set_frame_buffer(pass1_fbo_);
      
    
    context_->bind_program(vis_xyz_pass1_shader_);
    context_->set_blend_state(color_no_blending_state_);
    context_->set_rasterizer_state(change_point_size_in_shader_state_);
    context_->set_depth_stencil_state(depth_state_less_);
    
    vis_xyz_pass1_shader_->uniform("near_plane", near_plane_);
    vis_xyz_pass1_shader_->uniform("point_size_factor", settings_.point_size_);
    vis_xyz_pass1_shader_->uniform("height_divided_by_top_minus_bottom", height_divided_by_top_minus_bottom_);
    vis_xyz_pass1_shader_->uniform("far_minus_near_plane", far_plane_-near_plane_);

    vis_xyz_pass1_shader_->uniform("ellipsify", true);
    vis_xyz_pass1_shader_->uniform("clamped_normal_mode", true);
    vis_xyz_pass1_shader_->uniform("max_deform_ratio", 0.35f);

    context_->set_viewport(scm::gl::viewport(scm::math::vec2ui(0, 0), scm::math::vec2ui(window_width_, window_height_)));
    context_->apply();

    draw_all_models(context_id, view_id, vis_xyz_pass1_shader_);

    //PASS 2

    context_->clear_color_buffer(pass2_fbo_ , 0, scm::math::vec4f( .0f, .0f, .0f, 0.0f));
    context_->set_frame_buffer(pass2_fbo_);

    context_->set_blend_state(color_blending_state_);
    context_->set_depth_stencil_state(depth_state_disable_);

    context_->bind_program(vis_xyz_pass2_shader_);
    
    vis_xyz_pass2_shader_->uniform("depth_texture_pass1", 0);
    vis_xyz_pass2_shader_->uniform("pointsprite_texture", 1);

    context_->bind_texture(pass1_depth_buffer_, filter_nearest_, 0);
    context_->bind_texture(gaussian_texture_, filter_nearest_, 1);

    set_uniforms(vis_xyz_pass2_shader_);

    context_->set_viewport(scm::gl::viewport(scm::math::vec2ui(0, 0), scm::math::vec2ui(window_width_, window_height_)));
    context_->apply();

    draw_all_models(context_id, view_id, vis_xyz_pass2_shader_);

    //PASS 3

    context_->clear_color_buffer(fbo_, 0, scm::math::vec4f(0.0, 0.0, 0.0, 1.0f));
    context_->clear_depth_stencil_buffer(fbo_);
    context_->set_frame_buffer(fbo_);
    
    context_->set_depth_stencil_state(depth_state_disable_);
    context_->bind_program(vis_xyz_pass3_shader_);

    vis_xyz_pass3_shader_->uniform("background_color", 
      scm::math::vec3f(LAMURE_DEFAULT_COLOR_R, LAMURE_DEFAULT_COLOR_G, LAMURE_DEFAULT_COLOR_B));

    vis_xyz_pass3_shader_->uniform_sampler("in_color_texture", 0);
    context_->bind_texture(pass2_color_buffer_, filter_nearest_, 0);

    context_->set_viewport(scm::gl::viewport(scm::math::vec2ui(0, 0), scm::math::vec2ui(window_width_, window_height_)));
    context_->apply();

    screen_quad_->draw(context_);

  }
  else {
    //single pass

    context_->clear_color_buffer(fbo_, 0,
      scm::math::vec4f(LAMURE_DEFAULT_COLOR_R, LAMURE_DEFAULT_COLOR_G, LAMURE_DEFAULT_COLOR_B, 1.0f));   
    context_->clear_depth_stencil_buffer(fbo_);
    context_->set_frame_buffer(fbo_);

    context_->bind_program(vis_xyz_shader_);
    context_->set_rasterizer_state(change_point_size_in_shader_state_);
    context_->set_blend_state(color_no_blending_state_);
    context_->set_depth_stencil_state(depth_state_less_);
    
    set_uniforms(vis_xyz_shader_);

    context_->set_viewport(scm::gl::viewport(scm::math::vec2ui(0, 0), scm::math::vec2ui(window_width_, window_height_)));
    context_->apply();

    draw_all_models(context_id, view_id, vis_xyz_shader_);

  }


  //PASS 4: fullscreen quad
  
  context_->clear_default_depth_stencil_buffer();
  context_->clear_default_color_buffer();
  context_->set_default_frame_buffer();
  
  context_->bind_program(quad_shader_);
  
  context_->bind_texture(fbo_color_buffer_, filter_linear_, 0);
  quad_shader_->uniform("gamma_correction", (bool)settings_.gamma_correction_);

  context_->set_viewport(scm::gl::viewport(scm::math::vec2ui(0, 0), scm::math::vec2ui(window_width_, window_height_)));
  context_->apply();
  
  screen_quad_->draw(context_);

  rendering_ = false;
  glutSwapBuffers();
  
}


void glut_resize(int32_t w, int32_t h) {
  window_width_ = w;
  window_height_ = h;
  context_->set_viewport(scm::gl::viewport(scm::math::vec2ui(0, 0), scm::math::vec2ui(w, h)));

  fbo_ = device_->create_frame_buffer();
  fbo_color_buffer_ = device_->create_texture_2d(scm::math::vec2ui(window_width_, window_height_), scm::gl::FORMAT_RGBA_32F , 1, 1, 1);
  fbo_depth_buffer_ = device_->create_texture_2d(scm::math::vec2ui(window_width_, window_height_), scm::gl::FORMAT_D24, 1, 1, 1);
  fbo_->attach_color_buffer(0, fbo_color_buffer_);
  fbo_->attach_depth_stencil_buffer(fbo_depth_buffer_);

  pass1_fbo_ = device_->create_frame_buffer();
  pass1_depth_buffer_ = device_->create_texture_2d(scm::math::vec2ui(window_width_, window_height_), scm::gl::FORMAT_D24, 1, 1, 1);
  pass1_fbo_->attach_depth_stencil_buffer(pass1_depth_buffer_);

  pass2_fbo_ = device_->create_frame_buffer();
  pass2_color_buffer_ = device_->create_texture_2d(scm::math::vec2ui(window_width_, window_height_), scm::gl::FORMAT_RGBA_32F, 1, 1, 1);
  pass2_fbo_->attach_color_buffer(0, pass2_color_buffer_);

  
  lamure::ren::policy* policy = lamure::ren::policy::get_instance();
  policy->set_window_width(w);
  policy->set_window_height(h);
  
  camera_->set_projection_matrix(30.0f, float(w)/float(h),  near_plane_, far_plane_);

}

void glut_keyboard(unsigned char key, int32_t x, int32_t y) {
  switch (key) {
    case 27:
      exit(0);
      break;

    case '.':
      glutFullScreenToggle();
      break;


    case 'q':
      settings_.splatting_ = !settings_.splatting_;
      break;

    case 'u':
      if (settings_.point_size_ > 0.1) {
        settings_.point_size_ -= 0.1;
        std::cout << "point_size: " << settings_.point_size_ << std::endl;
      }
      break;

    case 'j':
      if (settings_.point_size_ < 10.0) {
        settings_.point_size_ += 0.1;
        std::cout << "point_size: " << settings_.point_size_ << std::endl;
      }
      break;
      
    
    case 'f':
      std::cout<<"fast travel: ";
      if (fast_travel_) {
        camera_->set_dolly_sens_(0.5f);
        std::cout<<"OFF\n\n";
      }
      else {
        camera_->set_dolly_sens_(20.5f);
        std::cout<<"ON\n\n";
      }
      fast_travel_ = ! fast_travel_;
      break;

    case '0':
      selected_model_ = -1;
      break;
    case '1':
      selected_model_ = std::min(num_models_-1, 0);
      break;
    case '2':
      selected_model_ = std::min(num_models_-1, 1);
      break;
    case '3':
      selected_model_ = std::min(num_models_-1, 2);
      break;
    case '4':
      selected_model_ = std::min(num_models_-1, 3);
      break;
    case '5':
      selected_model_ = std::min(num_models_-1, 4);
      break;
    case '6':
      selected_model_ = std::min(num_models_-1, 5);
      break;
    case '7':
      selected_model_ = std::min(num_models_-1, 6);
      break;
    case '8':
      selected_model_ = std::min(num_models_-1, 7);
      break;
    case '9':
      selected_model_ = std::min(num_models_-1, 8);
      break;
      

    default:
      break;

  }

}


void glut_motion(int32_t x, int32_t y) {

  camera_->update_trackball(x,y, window_width_, window_height_, mouse_state_);

}

void glut_mouse(int32_t button, int32_t state, int32_t x, int32_t y) {

  switch (button) {
    case GLUT_LEFT_BUTTON:
    {
        mouse_state_.lb_down_ = (state == GLUT_DOWN) ? true : false;
    } break;
    case GLUT_MIDDLE_BUTTON:
    {
        mouse_state_.mb_down_ = (state == GLUT_DOWN) ? true : false;
    } break;
    case GLUT_RIGHT_BUTTON:
    {
        mouse_state_.rb_down_ = (state == GLUT_DOWN) ? true : false;
    } break;
  }

  trackball_x_ = 2.f * float(x - (window_width_/2))/float(window_width_) ;
  trackball_y_ = 2.f * float(window_height_ - y - (window_height_/2))/float(window_height_);
  
  camera_->update_trackball_mouse_pos(trackball_x_, trackball_y_);
}

std::string const strip_whitespace(std::string const& in_string) {
  return boost::regex_replace(in_string, boost::regex("^ +| +$|( ) +"), "$1");

}

//checks for prefix AND removes it (+ whitespace) if it is found; 
//returns true, if prefix was found; else false
bool parse_prefix(std::string& in_string, std::string const& prefix) {

 uint32_t num_prefix_characters = prefix.size();

 bool prefix_found 
  = (!(in_string.size() < num_prefix_characters ) 
     && strncmp(in_string.c_str(), prefix.c_str(), num_prefix_characters ) == 0); 

  if( prefix_found ) {
    in_string = in_string.substr(num_prefix_characters);
    in_string = strip_whitespace(in_string);
  }

  return prefix_found;
}

bool read_shader(std::string const& path_string, 
                 std::string& shader_string) {


  if ( !boost::filesystem::exists( path_string ) ) {
    std::cout << "WARNING: File " << path_string << "does not exist." <<  std::endl;
    return false;
  }

  std::ifstream shader_source(path_string, std::ios::in);
  std::string line_buffer;

  std::string include_prefix("INCLUDE");

  std::size_t slash_position = path_string.find_last_of("/\\");
  std::string const base_path =  path_string.substr(0,slash_position+1);

  while( std::getline(shader_source, line_buffer) ) {
    line_buffer = strip_whitespace(line_buffer);
    //std::cout << line_buffer << "\n";

    if( parse_prefix(line_buffer, include_prefix) ) {
      std::string filename_string = line_buffer;
      read_shader(base_path+filename_string, shader_string);
    } else {
      shader_string += line_buffer+"\n";
    }
  }

  return true;
}


int32_t main(int argc, char* argv[]) {

  std::string vis_file = "";
  if (argc == 2) {
    vis_file = std::string(argv[1]);
  }
  else {
    std::cout << "Usage: " << argv[0] << " <vis_file.vis>\n" << 
      "\n";
    return 0;
  }


  settings_ = settings{1920, 1080, 2048, 4096, 32, 0, 1, 1, 0, 0, 0, 1.f, 0, 0.f, 0.05f, 
    scm::math::vec3f(68.f/255.f, 0.f, 84.f/255.f), scm::math::vec3f(251.f/255.f, 231.f/255.f, 35.f/255.f),
    "", "", std::vector<std::string>()};
  load_settings(vis_file, settings_);
 
  lamure::ren::policy* policy = lamure::ren::policy::get_instance();
  policy->set_max_upload_budget_in_mb(settings_.upload_);
  policy->set_render_budget_in_mb(settings_.vram_);
  policy->set_out_of_core_budget_in_mb(settings_.ram_);
  window_width_ = settings_.width_;
  window_height_ = settings_.height_;
  policy->set_window_width(window_width_);
  policy->set_window_height(window_height_);
  policy->set_size_of_provenance(settings_.prov_);

  if (policy->size_of_provenance() > 0) {
    data_provenance_ = lamure::ren::Data_Provenance::parse_json(settings_.json_);
  }

  lamure::ren::model_database* database = lamure::ren::model_database::get_instance();
  
  float scene_diameter = far_plane_;
  num_models_ = 0;
  for (const auto& input_file : settings_.models_) {
    lamure::model_t model_id = database->add_model(input_file, std::to_string(num_models_));
    
    const auto& bb = database->get_model(num_models_)->get_bvh()->get_bounding_boxes()[0];
    scene_diameter = scm::math::max(scm::math::length(bb.max_vertex()-bb.min_vertex()), scene_diameter);
    model_transformations_.push_back(scm::math::mat4d(scm::math::make_translation(database->get_model(num_models_)->get_bvh()->get_translation())));
    
    ++num_models_;
  }
  
  if(settings_.pvs_ != "") {
    std::cout << "loading pvs: " << settings_.pvs_ << std::endl;
    std::string pvs_grid_file_path = settings_.pvs_;
    pvs_grid_file_path.resize(pvs_grid_file_path.length() - 3);
    pvs_grid_file_path = pvs_grid_file_path + "grid";

    lamure::pvs::pvs_database* pvs = lamure::pvs::pvs_database::get_instance();
    pvs->load_pvs_from_file(pvs_grid_file_path, settings_.pvs_, false);
  }
 
  glutInit(&argc, argv);
  glutInitContextVersion(4, 4);
  glutInitContextProfile(GLUT_CORE_PROFILE);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_DEPTH | GLUT_RGBA);
  glutInitWindowSize(window_width_, window_height_);
  glutInitWindowPosition(64, 64);
  glutCreateWindow(argv[0]);
  glutSetWindowTitle("lamure_vis");
  
  glutDisplayFunc(glut_display);
  glutReshapeFunc(glut_resize);
  glutKeyboardFunc(glut_keyboard);
  glutMotionFunc(glut_motion);
  glutMouseFunc(glut_mouse);

  device_.reset(new scm::gl::render_device());


  context_ = device_->main_context();
  
  try
  {
    std::string quad_shader_fs_source;
    std::string quad_shader_vs_source;
    
    std::string vis_xyz_vs_source;
    std::string vis_xyz_fs_source;
    
    std::string vis_xyz_pass1_vs_source;
    std::string vis_xyz_pass1_fs_source;
    std::string vis_xyz_pass2_vs_source;
    std::string vis_xyz_pass2_fs_source;
    std::string vis_xyz_pass3_vs_source;
    std::string vis_xyz_pass3_fs_source;
    if (!read_shader("../share/lamure/shaders/vis/vis_quad.glslv", quad_shader_vs_source)
      || !read_shader("../share/lamure/shaders/vis/vis_quad.glslf", quad_shader_fs_source)
      || !read_shader("../share/lamure/shaders/vis/vis_xyz.glslv", vis_xyz_vs_source)
      || !read_shader("../share/lamure/shaders/vis/vis_xyz.glslf", vis_xyz_fs_source)
      || !read_shader("../share/lamure/shaders/vis/vis_xyz_pass1.glslv", vis_xyz_pass1_vs_source)
      || !read_shader("../share/lamure/shaders/vis/vis_xyz_pass1.glslf", vis_xyz_pass1_fs_source)
      || !read_shader("../share/lamure/shaders/vis/vis_xyz_pass2.glslv", vis_xyz_pass2_vs_source)
      || !read_shader("../share/lamure/shaders/vis/vis_xyz_pass2.glslf", vis_xyz_pass2_fs_source)
      || !read_shader("../share/lamure/shaders/vis/vis_xyz_pass3.glslv", vis_xyz_pass3_vs_source)
      || !read_shader("../share/lamure/shaders/vis/vis_xyz_pass3.glslf", vis_xyz_pass3_fs_source)
      ) {
      std::cout << "error reading shader files" << std::endl;
      return 1;
    }

    quad_shader_ = device_->create_program(
      boost::assign::list_of
        (device_->create_shader(scm::gl::STAGE_VERTEX_SHADER, quad_shader_vs_source))
        (device_->create_shader(scm::gl::STAGE_FRAGMENT_SHADER, quad_shader_fs_source)));
    if (!quad_shader_) {
      std::cout << "error creating shader programs" << std::endl;
      return 1;
    }

    vis_xyz_shader_ = device_->create_program(
      boost::assign::list_of
        (device_->create_shader(scm::gl::STAGE_VERTEX_SHADER, vis_xyz_vs_source))
        (device_->create_shader(scm::gl::STAGE_FRAGMENT_SHADER, vis_xyz_fs_source)));
    if (!vis_xyz_shader_) {
      std::cout << "error creating shader programs" << std::endl;
      return 1;
    }

    vis_xyz_pass1_shader_ = device_->create_program(
      boost::assign::list_of
        (device_->create_shader(scm::gl::STAGE_VERTEX_SHADER, vis_xyz_pass1_vs_source))
        (device_->create_shader(scm::gl::STAGE_FRAGMENT_SHADER, vis_xyz_pass1_fs_source)));
    if (!vis_xyz_pass1_shader_) {
      std::cout << "error creating shader programs" << std::endl;
      return 1;
    }

    vis_xyz_pass2_shader_ = device_->create_program(
      boost::assign::list_of
        (device_->create_shader(scm::gl::STAGE_VERTEX_SHADER, vis_xyz_pass2_vs_source))
        (device_->create_shader(scm::gl::STAGE_FRAGMENT_SHADER, vis_xyz_pass2_fs_source)));
    if (!vis_xyz_pass2_shader_) {
      std::cout << "error creating shader programs" << std::endl;
      return 1;
    }

    vis_xyz_pass3_shader_ = device_->create_program(
      boost::assign::list_of
        (device_->create_shader(scm::gl::STAGE_VERTEX_SHADER, vis_xyz_pass3_vs_source))
        (device_->create_shader(scm::gl::STAGE_FRAGMENT_SHADER, vis_xyz_pass3_fs_source)));
    if (!vis_xyz_pass3_shader_) {
      std::cout << "error creating shader programs" << std::endl;
      return 1;
    }
  }
  catch (std::exception& e)
  {
      std::cout << e.what() << std::endl;
  }


  glutShowWindow();
  
  glutTimerFunc(update_ms_, glut_timer, 1);

  fbo_ = device_->create_frame_buffer();
  fbo_color_buffer_ = device_->create_texture_2d(scm::math::vec2ui(window_width_, window_height_), scm::gl::FORMAT_RGBA_32F , 1, 1, 1);
  fbo_depth_buffer_ = device_->create_texture_2d(scm::math::vec2ui(window_width_, window_height_), scm::gl::FORMAT_D24, 1, 1, 1);
  fbo_->attach_color_buffer(0, fbo_color_buffer_);
  fbo_->attach_depth_stencil_buffer(fbo_depth_buffer_);

  pass1_fbo_ = device_->create_frame_buffer();
  pass1_depth_buffer_ = device_->create_texture_2d(scm::math::vec2ui(window_width_, window_height_), scm::gl::FORMAT_D24, 1, 1, 1);
  pass1_fbo_->attach_depth_stencil_buffer(pass1_depth_buffer_);

  pass2_fbo_ = device_->create_frame_buffer();
  pass2_color_buffer_ = device_->create_texture_2d(scm::math::vec2ui(window_width_, window_height_), scm::gl::FORMAT_RGBA_32F, 1, 1, 1);
  pass2_fbo_->attach_color_buffer(0, pass2_color_buffer_);


  color_blending_state_ = device_->create_blend_state(true, scm::gl::FUNC_ONE, scm::gl::FUNC_ONE, scm::gl::FUNC_ONE, 
    scm::gl::FUNC_ONE, scm::gl::EQ_FUNC_ADD, scm::gl::EQ_FUNC_ADD);
  color_no_blending_state_ = device_->create_blend_state(false);

  depth_state_less_ = device_->create_depth_stencil_state(true, true, scm::gl::COMPARISON_LESS);
  auto no_depth_test_descriptor = depth_state_less_->descriptor();
  no_depth_test_descriptor._depth_test = false;
  depth_state_disable_ = device_->create_depth_stencil_state(no_depth_test_descriptor);

  change_point_size_in_shader_state_ = device_->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_NONE, scm::gl::ORIENT_CCW, false, false, 0.0, false, false, scm::gl::point_raster_state(true));
  no_backface_culling_rasterizer_state_ = device_->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_NONE, scm::gl::ORIENT_CCW, false, false, 0.0, false, false);

  auto root_bb = database->get_model(0)->get_bvh()->get_bounding_boxes()[0];
  scm::math::vec3f center = scm::math::mat4f(model_transformations_[0]) * ((root_bb.min_vertex() + root_bb.max_vertex()) / 2.f);

  filter_linear_ = device_->create_sampler_state(scm::gl::FILTER_ANISOTROPIC, scm::gl::WRAP_CLAMP_TO_EDGE, 16u);  
  filter_nearest_ = device_->create_sampler_state(scm::gl::FILTER_MIN_MAG_LINEAR, scm::gl::WRAP_CLAMP_TO_EDGE);

  float gaussian_buffer[32] = {255, 255, 252, 247, 244, 234, 228, 222, 208, 201,
                              191, 176, 167, 158, 141, 131, 125, 117, 100,  91,
                              87,  71,  65,  58,  48,  42,  39,  32,  28,  25,
                              19, 16};
  scm::gl::texture_region ur(scm::math::vec3ui(0u), scm::math::vec3ui(32, 1, 1));
  gaussian_texture_ = device_->create_texture_2d(scm::math::vec2ui(32,1), scm::gl::FORMAT_R_32F, 1, 1, 1);
  context_->update_sub_texture(gaussian_texture_, ur, 0u, scm::gl::FORMAT_R_32F, gaussian_buffer);


  camera_ = new lamure::ren::camera(0, 
    scm::math::make_look_at_matrix(center+scm::math::vec3f(0.f, 0.1f, -0.01f), center, scm::math::vec3f(0.f, 1.f, 0.f)), 
    scm::math::length(root_bb.max_vertex()-root_bb.min_vertex()), false, false);
  
  
  screen_quad_.reset(new scm::gl::quad_geometry(device_, scm::math::vec2f(-1.0f, -1.0f), scm::math::vec2f(1.0f, 1.0f)));
  
  
  glutMainLoop();

  return 0;


}

