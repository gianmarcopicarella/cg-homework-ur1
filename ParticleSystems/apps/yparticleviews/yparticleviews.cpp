//
// LICENSE:
//
// Copyright (c) 2020 -- 2020 Fabio Pellacini
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#include <yocto/yocto_commonio.h>
#include <yocto/yocto_image.h>
#include <yocto/yocto_sceneio.h>
#include <yocto/yocto_shape.h>
#include <yocto_gui/yocto_gui.h>
#include <yocto_particle/yocto_particle.h>
using namespace yocto::math;
namespace sio = yocto::sceneio;
namespace cli = yocto::commonio;
namespace gui = yocto::gui;
namespace par = yocto::particle;

#include <atomic>
#include <deque>
#include <future>
using namespace std::string_literals;

#include "ext/filesystem.hpp"
namespace sfs = ghc::filesystem;

#ifdef _WIN32
#undef near
#undef far
#endif

// Application state
struct app_state {
  // loading parameters
  std::string filename  = "scene.json";
  std::string imagename = "out.png";
  std::string outname   = "scene.json";
  std::string name      = "";

  // scene
  sio::model*  ioscene  = new sio::model{};
  sio::camera* iocamera = nullptr;

  // rendering state
  gui::scene*       glscene  = new gui::scene{};
  gui::camera*      glcamera = nullptr;
  gui::scene_params glparams = {};

  // simulation scene
  par::scene*            ptscene  = new par::scene{};
  par::simulation_params ptparams = {};
  int                    ptframe  = 0;

  // shape maps
  std::unordered_map<sio::shape*, par::shape*> ptshapemap = {};
  std::unordered_map<sio::shape*, gui::shape*> glshapemap = {};

  // loading status
  std::atomic<bool> ok           = false;
  std::future<void> loader       = {};
  std::string       status       = "";
  std::string       error        = "";
  std::atomic<int>  current      = 0;
  std::atomic<int>  total        = 0;
  std::string       loader_error = "";

  ~app_state() {
    if (ioscene) delete ioscene;
    if (glscene) delete glscene;
    if (ptscene) delete ptscene;
  }
};

void init_glscene(gui::scene* glscene, sio::model* ioscene,
    gui::camera*& glcamera, sio::camera* iocamera,
    std::unordered_map<sio::shape*, gui::shape*>& glshapemap,
    sio::progress_callback                        progress_cb) {
  // handle progress
  auto progress = vec2i{
      0, (int)ioscene->cameras.size() + (int)ioscene->materials.size() +
             (int)ioscene->textures.size() + (int)ioscene->shapes.size() +
             (int)ioscene->subdivs.size() + (int)ioscene->instances.size() +
             (int)ioscene->objects.size()};

  // create scene
  init_scene(glscene);

  // camera
  auto camera_map     = std::unordered_map<sio::camera*, gui::camera*>{};
  camera_map[nullptr] = nullptr;
  for (auto iocamera : ioscene->cameras) {
    if (progress_cb) progress_cb("convert camera", progress.x++, progress.y);
    auto camera = add_camera(glscene);
    set_frame(camera, iocamera->frame);
    set_lens(camera, iocamera->lens, iocamera->aspect, iocamera->film);
    set_nearfar(camera, 0.001, 10000);
    camera_map[iocamera] = camera;
  }

  // textures
  auto texture_map     = std::unordered_map<sio::texture*, gui::texture*>{};
  texture_map[nullptr] = nullptr;
  for (auto iotexture : ioscene->textures) {
    if (progress_cb) progress_cb("convert texture", progress.x++, progress.y);
    auto gltexture = add_texture(glscene);
    if (!iotexture->colorf.empty()) {
      set_texture(gltexture, iotexture->colorf);
    } else if (!iotexture->colorb.empty()) {
      set_texture(gltexture, iotexture->colorb);
    } else if (!iotexture->scalarf.empty()) {
      set_texture(gltexture, iotexture->scalarf);
    } else if (!iotexture->scalarb.empty()) {
      set_texture(gltexture, iotexture->scalarb);
    }
    texture_map[iotexture] = gltexture;
  }

  // material
  auto material_map     = std::unordered_map<sio::material*, gui::material*>{};
  material_map[nullptr] = nullptr;
  for (auto iomaterial : ioscene->materials) {
    if (progress_cb) progress_cb("convert material", progress.x++, progress.y);
    auto glmaterial = add_material(glscene);
    set_emission(glmaterial, iomaterial->emission,
        texture_map.at(iomaterial->emission_tex));
    set_color(glmaterial, (1 - iomaterial->transmission) * iomaterial->color,
        texture_map.at(iomaterial->color_tex));
    set_specular(glmaterial,
        (1 - iomaterial->transmission) * iomaterial->specular,
        texture_map.at(iomaterial->specular_tex));
    set_metallic(glmaterial,
        (1 - iomaterial->transmission) * iomaterial->metallic,
        texture_map.at(iomaterial->metallic_tex));
    set_roughness(glmaterial, iomaterial->roughness,
        texture_map.at(iomaterial->roughness_tex));
    set_opacity(glmaterial, iomaterial->opacity,
        texture_map.at(iomaterial->opacity_tex));
    set_normalmap(glmaterial, texture_map.at(iomaterial->normal_tex));
    material_map[iomaterial] = glmaterial;
  }

  for (auto iosubdiv : ioscene->subdivs) {
    if (progress_cb) progress_cb("convert subdiv", progress.x++, progress.y);
    tesselate_subdiv(ioscene, iosubdiv);
  }

  // shapes
  auto shape_map     = std::unordered_map<sio::shape*, gui::shape*>{};
  shape_map[nullptr] = nullptr;
  for (auto ioshape : ioscene->shapes) {
    if (progress_cb) progress_cb("convert shape", progress.x++, progress.y);
    auto glshape = add_shape(glscene);
    set_positions(glshape, ioshape->positions);
    set_normals(glshape, ioshape->normals);
    set_texcoords(glshape, ioshape->texcoords);
    set_colors(glshape, ioshape->colors);
    set_points(glshape, ioshape->points);
    set_lines(glshape, ioshape->lines);
    set_triangles(glshape, ioshape->triangles);
    set_quads(glshape, ioshape->quads);
    set_edges(glshape, ioshape->triangles, ioshape->quads);
    shape_map[ioshape]  = glshape;
    glshapemap[ioshape] = glshape;
  }

  // instances
  auto instance_map     = std::unordered_map<sio::instance*, gui::instance*>{};
  instance_map[nullptr] = nullptr;
  for (auto ioinstance : ioscene->instances) {
    if (progress_cb) progress_cb("convert instance", progress.x++, progress.y);
    auto glinstance = add_instance(glscene);
    set_frames(glinstance, ioinstance->frames);
    instance_map[ioinstance] = glinstance;
  }

  // shapes
  for (auto ioobject : ioscene->objects) {
    if (progress_cb) progress_cb("convert object", progress.x++, progress.y);
    auto globject = add_object(glscene);
    set_frame(globject, ioobject->frame);
    set_shape(globject, shape_map.at(ioobject->shape));
    set_material(globject, material_map.at(ioobject->material));
    set_instance(globject, instance_map.at(ioobject->instance));
  }

  // done
  if (progress_cb) progress_cb("convert done", progress.x++, progress.y);

  // get cmmera
  glcamera = camera_map.at(iocamera);
}

void init_ptscene(par::scene* ptscene, sio::model* ioscene,
    std::unordered_map<sio::shape*, par::shape*>& ptshapemap,
    sio::progress_callback                        progress_cb) {
  // handle progress
  auto progress = vec2i{0, (int)ioscene->objects.size()};

  // shapes
  static auto velocity = std::unordered_map<std::string, float>{
      {"floor", 0}, {"particles", 1}, {"cloth", 0}, {"collider", 0}};
  for (auto ioobject : ioscene->objects) {
    if (progress_cb) progress_cb("convert object", progress.x++, progress.y);
    auto ioshape    = ioobject->shape;
    auto iomaterial = ioobject->material;
    if (iomaterial->name == "particles") {
      auto ptshape = add_particles(
          ptscene, ioshape->points, ioshape->positions, ioshape->radius, 1, 1);
      ptshapemap[ioshape] = ptshape;
    } else if (ioobject->material->name == "cloth") {
      auto nverts  = (int)ioshape->positions.size();
      auto ptshape = add_cloth(ptscene, ioshape->quads, ioshape->positions,
          ioshape->normals, ioshape->radius, 0.5, 1/8000.0,
          {nverts - 1, nverts - (int)sqrt(nverts)});
      ptshapemap[ioshape] = ptshape;
    } else if (ioobject->material->name == "collider") {
      add_collider(ptscene, ioshape->triangles, ioshape->quads,
          ioshape->positions, ioshape->normals, ioshape->radius);
    } else if (ioobject->material->name == "floor") {
      add_collider(ptscene, ioshape->triangles, ioshape->quads,
          ioshape->positions, ioshape->normals, ioshape->radius);
    } else {
      cli::print_fatal("unknown material " + ioobject->material->name);
    }
  }

  // done
  if (progress_cb) progress_cb("convert done", progress.x++, progress.y);
}

void update_glscene(
    const std::unordered_map<sio::shape*, gui::shape*>& glshapemap) {
  for (auto [ioshape, glshape] : glshapemap) {
    set_positions(glshape, ioshape->positions);
    set_normals(glshape, ioshape->normals);
  }
}

void update_ioscene(
    const std::unordered_map<sio::shape*, par::shape*>& ptshapemap) {
  for (auto [ioshape, ptshape] : ptshapemap) {
    get_positions(ptshape, ioshape->positions);
    get_normals(ptshape, ioshape->normals);
  }
}

void flatten_scene(sio::model* ioscene) {
  for (auto ioobject : ioscene->objects) {
    for (auto& position : ioobject->shape->positions)
      position = transform_point(ioobject->frame, position);
    for (auto& normal : ioobject->shape->normals)
      normal = transform_normal(ioobject->frame, normal);
    ioobject->frame = identity3x4f;
  }
}

int main(int argc, const char* argv[]) {
  // initialize app
  auto app_guard   = std::make_unique<app_state>();
  auto app         = app_guard.get();
  auto camera_name = ""s;

  // parse command line
  auto cli = cli::make_cli("ysceneviews", "views scene inteactively");
  add_option(cli, "--frames,-f", app->ptparams.frames, "Frames");
  add_option(
      cli, "--solver,-s", app->ptparams.solver, "Solver", par::solver_names);
  add_option(cli, "--gravity", app->ptparams.gravity, "Gravity");
  add_option(cli, "--camera", camera_name, "Camera name.");
  add_option(cli, "scene", app->filename, "Scene filename", true);
  parse_cli(cli, argc, argv);

  // loading scene
  auto ioerror = ""s;
  if (!load_scene(app->filename, app->ioscene, ioerror, cli::print_progress))
    cli::print_fatal(ioerror);
  flatten_scene(app->ioscene);

  // get camera
  app->iocamera = get_camera(app->ioscene, camera_name);

  // initialize particles
  init_ptscene(
      app->ptscene, app->ioscene, app->ptshapemap, cli::print_progress);

  // callbacks
  auto callbacks    = gui::ui_callbacks{};
  callbacks.init_cb = [app](gui::window* win, const gui::input& input) {
    init_glscene(app->glscene, app->ioscene, app->glcamera, app->iocamera,
        app->glshapemap,
        [app](const std::string& message, int current, int total) {
          app->status  = "init scene";
          app->current = current;
          app->total   = total;
        });
  };
  callbacks.clear_cb = [app](gui::window* win, const gui::input& input) {
    clear_scene(app->glscene);
  };
  callbacks.draw_cb = [app](gui::window* win, const gui::input& input) {
    draw_scene(
        app->glscene, app->glcamera, input.framebuffer_viewport, app->glparams);
  };
  callbacks.widgets_cb = [app](gui::window* win, const gui::input& input) {
    draw_progressbar(win, app->status.c_str(), app->current, app->total);
    if (draw_combobox(win, "camera", app->iocamera, app->ioscene->cameras)) {
      for (auto idx = 0; idx < app->ioscene->cameras.size(); idx++) {
        if (app->ioscene->cameras[idx] == app->iocamera)
          app->glcamera = app->glscene->cameras[idx];
      }
    }
    auto& params = app->glparams;
    draw_checkbox(win, "wireframe", params.wireframe);
  };
  callbacks.update_cb = [app](gui::window* win, const gui::input& input) {
    if (app->ptframe > app->ptparams.frames) app->ptframe = 0;
    if (app->ptframe == 0) init_simulation(app->ptscene, app->ptparams);
    simulate_frame(app->ptscene, app->ptparams);
    app->ptframe++;
    update_ioscene(app->ptshapemap);
    update_glscene(app->glshapemap);
    app->current = app->ptframe;
    app->total   = app->ptparams.frames;
  };
  callbacks.uiupdate_cb = [app](gui::window* win, const gui::input& input) {
    // handle mouse and keyboard for navigation
    if ((input.mouse_left || input.mouse_right) && !input.modifier_alt &&
        !input.widgets_active) {
      auto dolly  = 0.0f;
      auto pan    = zero2f;
      auto rotate = zero2f;
      if (input.mouse_left && !input.modifier_shift)
        rotate = (input.mouse_pos - input.mouse_last) / 100.0f;
      if (input.mouse_right)
        dolly = (input.mouse_pos.x - input.mouse_last.x) / 100.0f;
      if (input.mouse_left && input.modifier_shift)
        pan = (input.mouse_pos - input.mouse_last) / 100.0f;
      update_turntable(
          app->iocamera->frame, app->iocamera->focus, rotate, dolly, pan);
      set_frame(app->glcamera, app->iocamera->frame);
    }
  };

  // run ui
  run_ui({1280 + 320, 720}, "ysceneviews", callbacks);

  // done
  return 0;
}
