// Copyright (c) 2014-2018 Bauhaus-Universitaet Weimar
// This Software is distributed under the Modified BSD License, see license.txt.
//
// Virtual Reality and Visualization Research Group 
// Faculty of Media, Bauhaus-Universitaet Weimar
// http://www.uni-weimar.de/medien/vr

#include <boost/assign/list_of.hpp>
#include <queue>
#include <unordered_map>

#include "VTRenderer.h"
#include "imgui.h"
#include "imgui_impl_glfw_gl3.h"

#include <lamure/vt/ren/CutDatabase.h>
#include <lamure/vt/ren/CutUpdate.h>

namespace vt
{
VTRenderer::VTRenderer() : _data_resources(), _ctxt_resources(), _view_resources()
{
    this->_cut_update = &CutUpdate::get_instance();
    this->init();
}

void VTRenderer::init()
{
    choreography_scale = 0.3f;
    _scm_core.reset(new scm::core(0, nullptr));
    _device.reset(new scm::gl::render_device());

    std::string fs_vt_color, vx_vt_elevation, fs_vt_color_debug;

    {
        using namespace scm::gl;
        using namespace boost::assign;

#ifndef NDEBUG
        if(!scm::io::read_text_file(std::string(LAMURE_SHADERS_DIR) + "/textured_world_elevation.glslv", vx_vt_elevation) ||
           !scm::io::read_text_file(std::string(LAMURE_SHADERS_DIR) + "/textured_world_hierarchical.glslf", fs_vt_color_debug))
        {
            scm::err() << "error reading shader files" << scm::log::end;
            throw std::runtime_error("Error reading shader files");
        }
#else
        if(!scm::io::read_text_file(std::string(LAMURE_SHADERS_DIR) + "/textured_world_elevation.glslv", vx_vt_elevation) ||
           !scm::io::read_text_file(std::string(LAMURE_SHADERS_DIR) + "/textured_world_hierarchical.glslf", fs_vt_color))
        {
            scm::err() << "error reading shader files" << scm::log::end;
            throw std::runtime_error("Error reading shader files");
        }
#endif

#ifndef NDEBUG
        //_obj_earth.reset(new scm::gl::wavefront_obj_geometry(_device, std::string(LAMURE_PRIMITIVES_DIR) + "/quad.obj"));
        _obj_earth.reset(new scm::gl::wavefront_obj_geometry(_device, "earth.obj"));
        _obj_moon.reset(new scm::gl::wavefront_obj_geometry(_device, "moon.obj"));
#else
        _obj_earth.reset(new scm::gl::wavefront_obj_geometry(_device, "earth.obj"));
        _obj_moon.reset(new scm::gl::wavefront_obj_geometry(_device, "moon.obj"));
#endif
    }

    {
        using namespace scm::gl;
        using namespace boost::assign;

#ifndef NDEBUG
        _shader_vt = _device->create_program(list_of(_device->create_shader(STAGE_VERTEX_SHADER, vx_vt_elevation))(_device->create_shader(STAGE_FRAGMENT_SHADER, fs_vt_color_debug)));
#else
        _shader_vt = _device->create_program(list_of(_device->create_shader(STAGE_VERTEX_SHADER, vx_vt_elevation))(_device->create_shader(STAGE_FRAGMENT_SHADER, fs_vt_color)));
#endif
    }

    if(!_shader_vt)
    {
        scm::err() << "error creating shader program" << scm::log::end;
        throw std::runtime_error("Error creating shader program");
    }

    {
        using namespace scm::gl;

        _dstate_less = _device->create_depth_stencil_state(true, true, COMPARISON_LESS);
        _blend_state = _device->create_blend_state(true, FUNC_ONE, FUNC_ONE, FUNC_ONE, FUNC_ONE, EQ_FUNC_ADD, EQ_FUNC_ADD);

        _filter_nearest = _device->create_sampler_state(FILTER_MIN_MAG_NEAREST, WRAP_CLAMP_TO_EDGE);
        _filter_linear = _device->create_sampler_state(FILTER_MIN_MAG_LINEAR, WRAP_CLAMP_TO_EDGE);

        _ms_no_cull = _device->create_rasterizer_state(FILL_SOLID, CULL_NONE, ORIENT_CCW, true);
        _ms_cull = _device->create_rasterizer_state(FILL_SOLID, CULL_BACK, ORIENT_CCW, true);
    }
}

void VTRenderer::add_data(uint64_t cut_id, uint32_t data_id)
{
    using namespace scm::math;
    using namespace scm::gl;

    data_resource *resource = new data_resource();

    uint16_t depth = (uint16_t)((*CutDatabase::get_instance().get_cut_map())[cut_id]->get_atlas()->getDepth());
    uint16_t level = 0;
    while(level < depth)
    {
        uint32_t size_index_texture = (uint32_t)QuadTree::get_tiles_per_row(level);

        auto index_texture_level_ptr = _device->create_texture_2d(vec2ui(size_index_texture, size_index_texture), FORMAT_RGBA_8UI);

        _device->main_context()->clear_image_data(index_texture_level_ptr, 0, FORMAT_RGBA_8UI, 0);
        resource->_index_hierarchy.emplace_back(index_texture_level_ptr);

        level++;
    }

    _data_resources[data_id] = resource;
}
void VTRenderer::add_view(uint16_t view_id, uint32_t width, uint32_t height, float scale)
{
    view_resource *resource = new view_resource();

    resource->_width = width;
    resource->_height = height;
    resource->_start = std::chrono::high_resolution_clock::now();
    resource->_scale = scale;

    _view_resources[view_id] = resource;
}
void VTRenderer::add_context(uint16_t context_id)
{
    using namespace scm::math;
    using namespace scm::gl;

    ctxt_resource *resource = new ctxt_resource();

    // TODO: create auxiliary contexts
    resource->_render_context = _device->main_context();
    resource->_physical_texture_dimension = vec2ui(VTConfig::get_instance().get_phys_tex_tile_width(), VTConfig::get_instance().get_phys_tex_tile_width());

    vec2ui physical_texture_size = vec2ui(VTConfig::get_instance().get_phys_tex_px_width(), VTConfig::get_instance().get_phys_tex_px_width());
    resource->_physical_texture = _device->create_texture_2d(physical_texture_size, get_tex_format(), 1, VTConfig::get_instance().get_phys_tex_layers() + 1);

    resource->_size_feedback = VTConfig::get_instance().get_phys_tex_tile_width() * VTConfig::get_instance().get_phys_tex_tile_width() * VTConfig::get_instance().get_phys_tex_layers();
    resource->_feedback_lod_storage = _device->create_buffer(BIND_STORAGE_BUFFER, USAGE_STREAM_COPY, resource->_size_feedback * size_of_format(FORMAT_R_32I));
    resource->_feedback_count_storage = _device->create_buffer(BIND_STORAGE_BUFFER, USAGE_STREAM_COPY, resource->_size_feedback * size_of_format(FORMAT_R_32UI));

    resource->_feedback_lod_cpu_buffer = new int32_t[resource->_size_feedback];
    resource->_feedback_count_cpu_buffer = new uint32_t[resource->_size_feedback];

    for(size_t i = 0; i < resource->_size_feedback; ++i)
    {
        resource->_feedback_lod_cpu_buffer[i] = 0;
        resource->_feedback_count_cpu_buffer[i] = 0;
    }

    _ctxt_resources[context_id] = resource;
}
void VTRenderer::update_view(uint16_t view_id, uint32_t width, uint32_t height, float scale, const scm::math::mat4f &view_matrix)
{
    _view_resources[view_id]->_width = width;
    _view_resources[view_id]->_height = height;
    _view_resources[view_id]->_scale = scale;
    _view_resources[view_id]->_view_matrix = view_matrix;
}
void VTRenderer::clear_buffers(uint16_t context_id)
{
    using namespace scm::math;
    using namespace scm::gl;

    _ctxt_resources[context_id]->_render_context->set_default_frame_buffer();

    _ctxt_resources[context_id]->_render_context->clear_default_color_buffer(FRAMEBUFFER_BACK, vec4f(0.f, 0.f, 0.f, 1.0f));
    _ctxt_resources[context_id]->_render_context->clear_default_depth_stencil_buffer();

    _ctxt_resources[context_id]->_render_context->apply();
}
void VTRenderer::render_earth(uint32_t earth_color_id, uint32_t earth_elevation_id, uint16_t view_id, uint16_t context_id, bool exocentric, bool enable_hierarchical, int vis)
{
    using namespace scm::math;
    using namespace scm::gl;

    uint64_t color_cut_id = (((uint64_t)earth_color_id) << 32) | ((uint64_t)view_id << 16) | ((uint64_t)context_id);
    uint64_t elevation_cut_id = (((uint64_t)earth_elevation_id) << 32) | ((uint64_t)view_id << 16) | ((uint64_t)context_id);

    uint32_t color_max_depth = (*CutDatabase::get_instance().get_cut_map())[color_cut_id]->get_atlas()->getDepth() - 1;
    uint32_t elevation_max_depth = (*CutDatabase::get_instance().get_cut_map())[elevation_cut_id]->get_atlas()->getDepth() - 1;

    mat4f projection_matrix = mat4f::identity();
    perspective_matrix(projection_matrix, 32.f, float(_view_resources[view_id]->_width) / float(_view_resources[view_id]->_height), 0.01f, 1000.0f);

    _shader_vt->uniform("projection_matrix", projection_matrix);

    if(!exocentric)
    {
        std::chrono::duration<double> elapsed_seconds = std::chrono::high_resolution_clock::now() - _view_resources[view_id]->_start;
        mat4f model_matrix = mat4f::identity() * make_rotation(90.f, 1.f, 0.f, 0.f) * make_rotation(-90.f, 0.f, 0.f, 1.f) * make_scale(0.17529f, 0.17529f, 0.17529f);
        mat4f choreography_matrix = get_choreograpy((float)elapsed_seconds.count()*choreography_scale);
        mat4f model_view_matrix = _view_resources[view_id]->_view_matrix * choreography_matrix * model_matrix;

        _shader_vt->uniform("model_view_matrix", model_view_matrix);
    }
    else
    {
        mat4f model_matrix = mat4f::identity() * make_scale(_view_resources[view_id]->_scale, _view_resources[view_id]->_scale, _view_resources[view_id]->_scale);
        mat4f model_view_matrix = _view_resources[view_id]->_view_matrix * model_matrix;

        _shader_vt->uniform("model_view_matrix", model_view_matrix);
    }

    _shader_vt->uniform("physical_texture_dim", _ctxt_resources[context_id]->_physical_texture_dimension);
    _shader_vt->uniform("color_max_level", color_max_depth);
    _shader_vt->uniform("elevation_max_level", elevation_max_depth);
    _shader_vt->uniform("tile_size", scm::math::vec2((uint32_t)VTConfig::get_instance().get_size_tile()));
    _shader_vt->uniform("tile_padding", scm::math::vec2((uint32_t)VTConfig::get_instance().get_size_padding()));
    _shader_vt->uniform("enable_displacement", 1);
    
    for(uint32_t i = 0; i < _data_resources[earth_color_id]->_index_hierarchy.size(); ++i)
    {
        std::string texture_string = "hierarchical_idx_textures";
        _shader_vt->uniform(texture_string, i, int((i)));
    }

    _shader_vt->uniform("physical_texture_array", 17);

    for(uint32_t i = 18; i < _data_resources[earth_elevation_id]->_index_hierarchy.size() + 17; ++i)
    {
        std::string texture_string = "hierarchical_idx_textures_elevation";
        _shader_vt->uniform(texture_string, i, int((i)));
    }

    _shader_vt->uniform("enable_hierarchy", enable_hierarchical);
    _shader_vt->uniform("toggle_visualization", (int)vis);

    context_state_objects_guard csg(_ctxt_resources[context_id]->_render_context);
    context_texture_units_guard tug(_ctxt_resources[context_id]->_render_context);
    context_framebuffer_guard fbg(_ctxt_resources[context_id]->_render_context);

    _ctxt_resources[context_id]->_render_context->set_viewport(viewport(vec2ui(0, 0), 1 * vec2ui(_view_resources[view_id]->_width, _view_resources[view_id]->_height)));

    _ctxt_resources[context_id]->_render_context->set_depth_stencil_state(_dstate_less);
    _ctxt_resources[context_id]->_render_context->set_rasterizer_state(_ms_cull);
    _ctxt_resources[context_id]->_render_context->set_blend_state(_blend_state);

    _ctxt_resources[context_id]->_render_context->bind_program(_shader_vt);

    _ctxt_resources[context_id]->_render_context->sync();

    apply_cut_update(context_id);

    for(uint16_t i = 0; i < _data_resources[earth_color_id]->_index_hierarchy.size(); ++i)
    {
        _ctxt_resources[context_id]->_render_context->bind_texture(_data_resources[earth_color_id]->_index_hierarchy.at(i), _filter_nearest, i);
    }

    _ctxt_resources[context_id]->_render_context->bind_texture(_ctxt_resources[context_id]->_physical_texture, _filter_linear, 17);

    for(uint16_t i = 0; i < _data_resources[earth_elevation_id]->_index_hierarchy.size(); ++i)
    {
        _ctxt_resources[context_id]->_render_context->bind_texture(_data_resources[earth_elevation_id]->_index_hierarchy.at(i), _filter_nearest, i + 18);
    }

    _ctxt_resources[context_id]->_render_context->bind_storage_buffer(_ctxt_resources[context_id]->_feedback_lod_storage, 0);
    _ctxt_resources[context_id]->_render_context->bind_storage_buffer(_ctxt_resources[context_id]->_feedback_count_storage, 1);

    _ctxt_resources[context_id]->_render_context->apply();

    _obj_earth->draw(_ctxt_resources[context_id]->_render_context, geometry::MODE_SOLID);

    _ctxt_resources[context_id]->_render_context->sync();
}

void VTRenderer::render_moon(uint32_t moon_data_id, uint16_t view_id, uint16_t context_id, bool exocentric, bool enable_hierarchical, int vis)
{
    using namespace scm::math;
    using namespace scm::gl;

    uint64_t color_cut_id = (((uint64_t)moon_data_id) << 32) | ((uint64_t)view_id << 16) | ((uint64_t)context_id);
    uint32_t max_depth_level = (*CutDatabase::get_instance().get_cut_map())[color_cut_id]->get_atlas()->getDepth() - 1;

    mat4f projection_matrix = mat4f::identity();
    perspective_matrix(projection_matrix, 32.f, float(_view_resources[view_id]->_width) / float(_view_resources[view_id]->_height), 0.01f, 100000.0f);

    _shader_vt->uniform("projection_matrix", projection_matrix);

    if(!exocentric)
    {
        std::chrono::duration<double> elapsed_seconds = std::chrono::high_resolution_clock::now() - _view_resources[view_id]->_start;
        mat4f model_matrix =
            mat4f::identity() * make_translation(0.0f, 1.0f, 0.0f) * make_rotation(90.f, 1.f, 0.f, 0.f) * make_rotation(-90.f, 0.f, 0.f, 1.f) * make_scale(0.04422f, 0.04422f, 0.04422f);
        mat4f choreography_matrix = get_choreograpy((float)elapsed_seconds.count()*choreography_scale);
        mat4f model_view_matrix = _view_resources[view_id]->_view_matrix * choreography_matrix * model_matrix;

        _shader_vt->uniform("model_view_matrix", model_view_matrix);
    }
    else
    {
        mat4f model_matrix = mat4f::identity() * make_scale(_view_resources[view_id]->_scale, _view_resources[view_id]->_scale, _view_resources[view_id]->_scale);
        mat4f model_view_matrix = _view_resources[view_id]->_view_matrix * model_matrix;

        _shader_vt->uniform("model_view_matrix", model_view_matrix);
    }

    _shader_vt->uniform("physical_texture_dim", _ctxt_resources[context_id]->_physical_texture_dimension);
    _shader_vt->uniform("max_level", max_depth_level);
    _shader_vt->uniform("tile_size", scm::math::vec2((uint32_t)VTConfig::get_instance().get_size_tile()));
    _shader_vt->uniform("tile_padding", scm::math::vec2((uint32_t)VTConfig::get_instance().get_size_padding()));
    _shader_vt->uniform("enable_displacement", 0);

    for(uint32_t i = 0; i < _data_resources[moon_data_id]->_index_hierarchy.size(); ++i)
    {
        std::string texture_string = "hierarchical_idx_textures";
        _shader_vt->uniform(texture_string, i, int((i)));
    }

    _shader_vt->uniform("physical_texture_array", 17);

    _shader_vt->uniform("enable_hierarchy", enable_hierarchical);
    _shader_vt->uniform("toggle_visualization", (int)vis);

    context_state_objects_guard csg(_ctxt_resources[context_id]->_render_context);
    context_texture_units_guard tug(_ctxt_resources[context_id]->_render_context);
    context_framebuffer_guard fbg(_ctxt_resources[context_id]->_render_context);

    _ctxt_resources[context_id]->_render_context->set_viewport(viewport(vec2ui(0, 0), 1 * vec2ui(_view_resources[view_id]->_width, _view_resources[view_id]->_height)));

    _ctxt_resources[context_id]->_render_context->set_depth_stencil_state(_dstate_less);
    _ctxt_resources[context_id]->_render_context->set_rasterizer_state(_ms_cull);
    _ctxt_resources[context_id]->_render_context->set_blend_state(_blend_state);

    _ctxt_resources[context_id]->_render_context->bind_program(_shader_vt);

    _ctxt_resources[context_id]->_render_context->sync();

    apply_cut_update(context_id);

    for(uint16_t i = 0; i < _data_resources[moon_data_id]->_index_hierarchy.size(); ++i)
    {
        _ctxt_resources[context_id]->_render_context->bind_texture(_data_resources[moon_data_id]->_index_hierarchy.at(i), _filter_nearest, i);
    }

    _ctxt_resources[context_id]->_render_context->bind_texture(_ctxt_resources[context_id]->_physical_texture, _filter_linear, 17);

    for(uint16_t i = 0; i < _data_resources[moon_data_id]->_index_hierarchy.size(); ++i)
    {
        _ctxt_resources[context_id]->_render_context->bind_texture(_data_resources[moon_data_id]->_index_hierarchy.at(i), _filter_nearest, i + 18);
    }

    _ctxt_resources[context_id]->_render_context->bind_storage_buffer(_ctxt_resources[context_id]->_feedback_lod_storage, 0);
    _ctxt_resources[context_id]->_render_context->bind_storage_buffer(_ctxt_resources[context_id]->_feedback_count_storage, 1);

    _ctxt_resources[context_id]->_render_context->apply();

    _obj_moon->draw(_ctxt_resources[context_id]->_render_context, geometry::MODE_SOLID);

    _ctxt_resources[context_id]->_render_context->sync();
}

void VTRenderer::collect_feedback(uint16_t context_id)
{
    using namespace scm::math;
    using namespace scm::gl;

    int32_t *feedback_lod = (int32_t *)_ctxt_resources[context_id]->_render_context->map_buffer(_ctxt_resources[context_id]->_feedback_lod_storage, ACCESS_READ_ONLY);
    memcpy(_ctxt_resources[context_id]->_feedback_lod_cpu_buffer, feedback_lod, _ctxt_resources[context_id]->_size_feedback * size_of_format(FORMAT_R_32I));
    _ctxt_resources[context_id]->_render_context->sync();

    _ctxt_resources[context_id]->_render_context->unmap_buffer(_ctxt_resources[context_id]->_feedback_lod_storage);
    _ctxt_resources[context_id]->_render_context->clear_buffer_data(_ctxt_resources[context_id]->_feedback_lod_storage, FORMAT_R_32I, nullptr);

    uint32_t *feedback_count = (uint32_t *)_ctxt_resources[context_id]->_render_context->map_buffer(_ctxt_resources[context_id]->_feedback_count_storage, ACCESS_READ_ONLY);
    memcpy(_ctxt_resources[context_id]->_feedback_count_cpu_buffer, feedback_count, _ctxt_resources[context_id]->_size_feedback * size_of_format(FORMAT_R_32UI));
    _ctxt_resources[context_id]->_render_context->sync();

    _cut_update->feedback(_ctxt_resources[context_id]->_feedback_lod_cpu_buffer, _ctxt_resources[context_id]->_feedback_count_cpu_buffer);

    _ctxt_resources[context_id]->_render_context->unmap_buffer(_ctxt_resources[context_id]->_feedback_count_storage);
    _ctxt_resources[context_id]->_render_context->clear_buffer_data(_ctxt_resources[context_id]->_feedback_count_storage, FORMAT_R_32UI, nullptr);
}

void VTRenderer::apply_cut_update(uint16_t context_id)
{
    auto start = std::chrono::high_resolution_clock::now();

    CutDatabase *cut_db = &CutDatabase::get_instance();

    for(cut_map_entry_type cut_entry : (*cut_db->get_cut_map()))
    {
        Cut *cut = cut_db->start_reading_cut(cut_entry.first);

        if(!cut->is_drawn())
        {
            cut_db->stop_reading_cut(cut_entry.first);
            continue;
        }

        std::set<uint16_t> updated_levels;

        for(auto position_slot_updated : cut->get_front()->get_mem_slots_updated())
        {
            const mem_slot_type *mem_slot_updated = cut_db->read_mem_slot_at(position_slot_updated.second);

            if(mem_slot_updated == nullptr || !mem_slot_updated->updated || !mem_slot_updated->locked || mem_slot_updated->pointer == nullptr)
            {
                if(mem_slot_updated == nullptr)
                {
                    std::cerr << "Mem slot at " << position_slot_updated.second << " is null" << std::endl;
                }
                else
                {
                    std::cerr << "Mem slot at " << position_slot_updated.second << std::endl;
                    std::cerr << "Mem slot #" << mem_slot_updated->position << std::endl;
                    std::cerr << "Tile id: " << mem_slot_updated->tile_id << std::endl;
                    std::cerr << "Locked: " << mem_slot_updated->locked << std::endl;
                    std::cerr << "Updated: " << mem_slot_updated->updated << std::endl;
                    std::cerr << "Pointer valid: " << (mem_slot_updated->pointer != nullptr) << std::endl;
                }

                throw std::runtime_error("updated mem slot inconsistency");
            }

            updated_levels.insert(QuadTree::get_depth_of_node(mem_slot_updated->tile_id));
            update_physical_texture_blockwise(context_id, mem_slot_updated->pointer, mem_slot_updated->position);
        }

        for(auto position_slot_cleared : cut->get_front()->get_mem_slots_cleared())
        {
            const mem_slot_type *mem_slot_cleared = cut_db->read_mem_slot_at(position_slot_cleared.second);

            if(mem_slot_cleared == nullptr)
            {
                std::cerr << "Mem slot at " << position_slot_cleared.second << " is null" << std::endl;
            }

            updated_levels.insert(QuadTree::get_depth_of_node(position_slot_cleared.first));
        }

        for(uint16_t updated_level : updated_levels)
        {
            // std::cout << "Updated level:  "+std::to_string(updated_level) << std::endl;
            update_index_texture(Cut::get_dataset_id(cut_entry.first), context_id, updated_level, cut->get_front()->get_index(updated_level));
        }

        cut_db->stop_reading_cut(cut_entry.first);
    }

    _ctxt_resources[context_id]->_render_context->sync();

    auto end = std::chrono::high_resolution_clock::now();
    _apply_time = std::chrono::duration<float, std::milli>(end - start).count();
}

void VTRenderer::update_index_texture(uint32_t data_id, uint16_t context_id, uint16_t level, const uint8_t *buf_cpu)
{
    using namespace scm::math;
    using namespace scm::gl;

    uint32_t size_index_texture = (uint32_t)QuadTree::get_tiles_per_row(level);

    vec3ui origin = vec3ui(0, 0, 0);
    vec3ui dimensions = vec3ui(size_index_texture, size_index_texture, 1);

    _ctxt_resources[context_id]->_render_context->update_sub_texture(_data_resources[data_id]->_index_hierarchy.at(level), texture_region(origin, dimensions), 0, FORMAT_RGBA_8UI, buf_cpu);
}

scm::gl::data_format VTRenderer::get_tex_format()
{
    switch(VTConfig::get_instance().get_format_texture())
    {
    case VTConfig::R8:
        return scm::gl::FORMAT_R_8;
    case VTConfig::RGB8:
        return scm::gl::FORMAT_RGB_8;
    case VTConfig::RGBA8:
    default:
        return scm::gl::FORMAT_RGBA_8;
    }
}

void VTRenderer::update_physical_texture_blockwise(uint16_t context_id, const uint8_t *buf_texel, size_t slot_position)
{
    using namespace scm::math;
    using namespace scm::gl;

    size_t slots_per_texture = VTConfig::get_instance().get_phys_tex_tile_width() * VTConfig::get_instance().get_phys_tex_tile_width();
    size_t layer = slot_position / slots_per_texture;
    size_t rel_slot_position = slot_position - layer * slots_per_texture;
    size_t x_tile = rel_slot_position % VTConfig::get_instance().get_phys_tex_tile_width();
    size_t y_tile = rel_slot_position / VTConfig::get_instance().get_phys_tex_tile_width();

    vec3ui origin = vec3ui((uint32_t)x_tile * VTConfig::get_instance().get_size_tile(), (uint32_t)y_tile * VTConfig::get_instance().get_size_tile(), (uint32_t)layer);
    vec3ui dimensions = vec3ui(VTConfig::get_instance().get_size_tile(), VTConfig::get_instance().get_size_tile(), 1);

    _ctxt_resources[context_id]->_render_context->update_sub_texture(_ctxt_resources[context_id]->_physical_texture, texture_region(origin, dimensions), 0, get_tex_format(), buf_texel);
}

VTRenderer::~VTRenderer()
{
    _shader_vt.reset();

    _filter_nearest.reset();
    _filter_linear.reset();

    _ms_no_cull.reset();

    _device.reset();
    _scm_core.reset();
}
void VTRenderer::extract_debug_cut(uint32_t data_id, uint16_t view_id, uint16_t context_id)
{
    CutDatabase *cut_db = &CutDatabase::get_instance();

    uint64_t cut_id = (((uint64_t)data_id) << 32) | ((uint64_t)view_id << 16) | ((uint64_t)context_id);

    cut_db->start_reading_cut(cut_id);

    if(_cut_debug_outputs[cut_id] == nullptr)
    {
        _cut_debug_outputs[cut_id] = new debug_cut();
    }

    std::stringstream stream_cut;
    stream_cut << "Cut { ";
    for(id_type iter : (*cut_db->get_cut_map())[cut_id]->get_front()->get_cut())
    {
        stream_cut << iter << " ";
    }
    stream_cut << "}" << std::endl;

    _cut_debug_outputs[cut_id]->_string_cut = stream_cut.str();

    std::stringstream stream_index_string;

    uint16_t depth = (uint16_t)(*cut_db->get_cut_map())[cut_id]->get_atlas()->getDepth();

    uint16_t level = 0;

    while(level < depth - 7)
    {
        size_t size_index_texture = QuadTree::get_tiles_per_row(level);

        for(size_t y = 0; y < size_index_texture; ++y)
        {
            for(size_t x = 0; x < size_index_texture; ++x)
            {
                auto ptr = &(*cut_db->get_cut_map())[cut_id]->get_front()->get_index(level)[y * size_index_texture * 4 + x * 4];

                stream_index_string << (int)ptr[0] << "." << (int)ptr[1] << "." << (int)ptr[2] << "." << (int)ptr[3] << " ";
            }

            stream_index_string << std::endl;
        }

        stream_index_string << std::endl;

        level++;
    }

    _cut_debug_outputs[cut_id]->_string_index = stream_index_string.str();

    cut_db->stop_reading_cut(cut_id);
}
void VTRenderer::extract_debug_context(uint16_t context_id)
{
    CutDatabase *cut_db = &CutDatabase::get_instance();

    cut_db->start_reading_cut(0);

    if(_ctxt_debug_outputs[context_id] == nullptr)
    {
        _ctxt_debug_outputs[context_id] = new debug_context();
    }

    _ctxt_debug_outputs[context_id]->_fps.push_back(ImGui::GetIO().Framerate);
    _ctxt_debug_outputs[context_id]->_fps.pop_front();

    size_t free_slots = 0;

    std::stringstream stream_mem_slots;
    for(size_t layer = 0; layer < VTConfig::get_instance().get_phys_tex_layers(); layer++)
    {
        for(size_t y = 0; y < cut_db->get_size_mem_y(); ++y)
        {
            for(size_t x = 0; x < cut_db->get_size_mem_x(); ++x)
            {
                mem_slot_type *mem_slot = cut_db->read_mem_slot_at(x + y * cut_db->get_size_mem_x() + layer * cut_db->get_size_mem_x() * cut_db->get_size_mem_y());

                if(!(*mem_slot).locked)
                {
                    stream_mem_slots << "F ";
                    free_slots++;
                }
                else
                {
                    stream_mem_slots << (*mem_slot).tile_id << " ";
                }
            }
            stream_mem_slots << std::endl;
        }
        stream_mem_slots << std::endl;
    }
    _ctxt_debug_outputs[context_id]->_string_mem_slots = stream_mem_slots.str();
    _ctxt_debug_outputs[context_id]->_mem_slots_busy = ((float)cut_db->get_size_mem_interleaved() - cut_db->get_available_memory()) / (float)cut_db->get_size_mem_interleaved();

    std::stringstream stream_feedback;
    size_t phys_tex_tile_width = VTConfig::get_instance().get_phys_tex_tile_width();

    for(size_t layer = 0; layer < VTConfig::get_instance().get_phys_tex_layers(); layer++)
    {
        for(size_t y = 0; y < phys_tex_tile_width; ++y)
        {
            for(size_t x = 0; x < phys_tex_tile_width; ++x)
            {
                stream_feedback << _ctxt_resources[context_id]->_feedback_lod_cpu_buffer[x + y * phys_tex_tile_width + layer * phys_tex_tile_width * phys_tex_tile_width] << " ";
            }

            stream_feedback << std::endl;
        }
        stream_feedback << std::endl;
    }

    _ctxt_debug_outputs[context_id]->_string_feedback = stream_feedback.str();

    _ctxt_debug_outputs[context_id]->_string_mem_slots = stream_mem_slots.str();

    _ctxt_debug_outputs[context_id]->_times_cut_dispatch.push_back(_cut_update->get_dispatch_time());
    _ctxt_debug_outputs[context_id]->_times_cut_dispatch.pop_front();

    _ctxt_debug_outputs[context_id]->_times_apply.push_back(_apply_time);
    _ctxt_debug_outputs[context_id]->_times_apply.pop_front();

    cut_db->stop_reading_cut(0);
}
void VTRenderer::render_debug_cut(uint32_t data_id, uint16_t view_id, uint16_t context_id)
{
    uint64_t cut_id = (((uint64_t)data_id) << 32) | ((uint64_t)view_id << 16) | ((uint64_t)context_id);

    ImGui::SetNextWindowSize(ImVec2(550, 680), ImGuiCond_FirstUseEver);

    if(!ImGui::Begin(("Dataset: " + std::to_string(data_id) + ", view: " + std::to_string(view_id) + ", context: " + std::to_string(context_id)).c_str()))
    {
        ImGui::End();
        return;
    }

    ImGui::SetNextTreeNodeOpen(true);

    if(ImGui::CollapsingHeader("Cut state"))
    {
        ImGui::TextColored(ImVec4(0.5f, 1.0f, 0.5f, 1.0), _cut_debug_outputs[cut_id]->_string_cut.c_str());
        ImGui::TextColored(ImVec4(0.5f, 0.5f, 1.0f, 1.0), _cut_debug_outputs[cut_id]->_string_index.c_str());
    }

    ImGui::End();
}
void VTRenderer::render_debug_context(uint16_t context_id)
{
    ImGui::SetNextWindowSize(ImVec2(550, 680), ImGuiCond_FirstUseEver);

    if(!ImGui::Begin(("Context")))
    {
        ImGui::End();
        return;
    }

    ImVec2 plot_dims(0, 160);

    ImGui::SetNextTreeNodeOpen(true);

    if(ImGui::CollapsingHeader("Performance metrics"))
    {
        auto max_fps = *std::max_element(_ctxt_debug_outputs[context_id]->_fps.begin(), _ctxt_debug_outputs[context_id]->_fps.end());
        auto min_fps = *std::min_element(_ctxt_debug_outputs[context_id]->_fps.begin(), _ctxt_debug_outputs[context_id]->_fps.end());

        std::stringstream stream_average;
        stream_average << "Application average " << 1000.0f / ImGui::GetIO().Framerate << " ms/frame (" << ImGui::GetIO().Framerate << " FPS)\"";

        ImGui::PlotHistogram("FPS", &_ctxt_debug_outputs[context_id]->_fps[0], debug_context::FPS_S, 0, stream_average.str().c_str(), min_fps, max_fps, plot_dims);

        std::stringstream stream_usage;
        stream_usage << "Physical texture slots usage: " << _ctxt_debug_outputs[context_id]->_mem_slots_busy * 100 << "%";

        ImGui::ProgressBar(_ctxt_debug_outputs[context_id]->_mem_slots_busy, ImVec2(0, 80), stream_usage.str().c_str());
    }

    ImGui::SetNextTreeNodeOpen(true);

    if(ImGui::CollapsingHeader("Cut update metrics"))
    {
        auto max_disp = *std::max_element(_ctxt_debug_outputs[context_id]->_times_cut_dispatch.begin(), _ctxt_debug_outputs[context_id]->_times_cut_dispatch.end());
        auto max_apply = *std::max_element(_ctxt_debug_outputs[context_id]->_times_apply.begin(), _ctxt_debug_outputs[context_id]->_times_apply.end());

        auto min_disp = *std::min_element(_ctxt_debug_outputs[context_id]->_times_cut_dispatch.begin(), _ctxt_debug_outputs[context_id]->_times_cut_dispatch.end());
        auto min_apply = *std::min_element(_ctxt_debug_outputs[context_id]->_times_apply.begin(), _ctxt_debug_outputs[context_id]->_times_apply.end());

        ImGui::Text(_ctxt_debug_outputs[context_id]->_string_mem_slots.c_str());
        ImGui::TextColored(ImVec4(1.0f, 0.5f, 1.0f, 1.0), _ctxt_debug_outputs[context_id]->_string_feedback.c_str());

        std::stringstream stream_dispatch_max;
        stream_dispatch_max << "Max: " << max_disp << " msec";
        ImGui::PlotLines("Dispatch time, msec", &_ctxt_debug_outputs[context_id]->_times_cut_dispatch[0], debug_context::DISP_S, 0, stream_dispatch_max.str().c_str(), min_disp, max_disp, plot_dims);

        std::stringstream stream_apply_max;
        stream_apply_max << "Max: " << max_apply << " msec";
        ImGui::PlotLines("Apply time, msec", &_ctxt_debug_outputs[context_id]->_times_apply[0], debug_context::APPLY_S, 0, stream_apply_max.str().c_str(), min_apply, max_apply, plot_dims);
    }

    ImGui::End();
}
scm::math::mat4f VTRenderer::get_choreograpy(float time)
{
    using namespace scm::math;

    float period = 60.f;

    mat4f matrix = mat4f::identity();

    float stage_time = time - std::floor(time / period) * period;

    if(stage_time < period * 0.1f)
    {
        float stage_point = stage_time / (period * 0.1f);
        matrix *= make_translation(0.f, -4.50f * std::sqrt(stage_point), 0.f);
    }
    else if(stage_time < period * 0.4f)
    {
        float stage_point = (stage_time - period * 0.1f) / (period * 0.3f);
        matrix *= make_translation(0.f, -4.50f, 0.f) * make_rotation(360.f * stage_point, 1.f, 0.f, 0.f);
    }
    else if(stage_time < period * 0.8f)
    {
        float stage_point = (stage_time - period * 0.4f) / (period * 0.4f);

        float z = 0.1f * std::sin(stage_point * 2.f * scm::math::pi_f);
        float y = 0.35f - 0.35f * std::cos(stage_point * 2.f * scm::math::pi_f);

        float angle = std::atan(z / y) * 360.0f / scm::math::pi_f + 180.0f;
        float dist = std::sqrt(z * z + y * y);

        matrix *= make_translation(0.f, -4.50f + dist, 0.f) * make_rotation(-angle, 1.f, 0.f, 0.f);
    }
    else if(stage_time < period)
    {
        float stage_point = (stage_time - period * 0.8f) / (period * 0.2f);
        matrix *= make_translation(0.f, -4.50f * (1.0f - (float)std::pow(stage_point, 2)), 0.f);
    }

    return matrix;
}
}