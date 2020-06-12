//
// LICENSE:
//
// Copyright (c) 2016 -- 2020 Fabio Pellacini
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
#include <yocto/yocto_math.h>
#include <yocto/yocto_sceneio.h>
#include <yocto/yocto_shape.h>
using namespace yocto::math;
namespace sio = yocto::sceneio;
namespace shp = yocto::shape;
namespace cli = yocto::commonio;

#include <memory>
using std::string;
using namespace std::string_literals;

#include "ext/filesystem.hpp"
namespace sfs = ghc::filesystem;

#include "ext/perlin-noise/noise1234.h"

float noise(const vec3f& p) { return noise3(p.x, p.y, p.z); }
vec2f noise2(const vec3f& p) {
  return {noise(p + vec3f{0, 0, 0}), noise(p + vec3f{3, 7, 11})};
}
vec3f noise3(const vec3f& p) {
  return {noise(p + vec3f{0, 0, 0}), noise(p + vec3f{3, 7, 11}),
      noise(p + vec3f{13, 17, 19})};
}
float fbm(const vec3f& p, int octaves) {
    float res = 0.f;
    for(int i = 0; i <= octaves; i++) {
        res += yocto::math::pow(2.f, -i) * noise(yocto::math::pow(2.f, i) * p);
    }
    return res;
}
float turbulence(const vec3f& p, int octaves) {
    float res = 0.f;
    for(int i = 0; i <= octaves; i++) {
        res += yocto::math::pow(2.f, -i) * yocto::math::abs(noise(yocto::math::pow(2.f, i) * p));
    }
    return res;
}
float ridge(const vec3f& p, int octaves) {
    float res = 0.f;
    for(int i = 0; i <= octaves; i++) {
        res += yocto::math::pow(2.f, -i) * yocto::math::pow((1.f - yocto::math::abs(noise(yocto::math::pow(2.f, i) * p))), 2.f) / 2.f;
    }
    return res;
}

sio::object* get_object(sio::model* scene, const std::string& name) {
  for (auto object : scene->objects)
    if (object->name == name) return object;
  cli::print_fatal("unknown object " + name);
  return nullptr;
}

void add_polyline(sio::shape* shape, const std::vector<vec3f>& positions,
    const std::vector<vec3f>& colors, float thickness = 0.0001f) {
  auto offset = (int)shape->positions.size();
  shape->positions.insert(
      shape->positions.end(), positions.begin(), positions.end());
  shape->colors.insert(shape->colors.end(), colors.begin(), colors.end());
  shape->radius.insert(shape->radius.end(), positions.size(), thickness);
  for (auto idx = 0; idx < positions.size() - 1; idx++) {
    shape->lines.push_back({offset + idx, offset + idx + 1});
  }
}

void sample_shape(std::vector<vec3f>& positions, std::vector<vec3f>& normals,
    std::vector<vec2f>& texcoords, sio::shape* shape, int num) {
  auto triangles  = shape->triangles;
  auto qtriangles = shp::quads_to_triangles(shape->quads);
  triangles.insert(triangles.end(), qtriangles.begin(), qtriangles.end());
  auto cdf = shp::sample_triangles_cdf(triangles, shape->positions);
  auto rng = make_rng(19873991);
  for (auto idx = 0; idx < num; idx++) {
    auto [elem, uv] = shp::sample_triangles(cdf, rand1f(rng), rand2f(rng));
    auto q          = triangles[elem];
    positions.push_back(interpolate_triangle(shape->positions[q.x],
        shape->positions[q.y], shape->positions[q.z], uv));
    normals.push_back(normalize(interpolate_triangle(
        shape->normals[q.x], shape->normals[q.y], shape->normals[q.z], uv)));
    if (!texcoords.empty()) {
      texcoords.push_back(interpolate_triangle(shape->texcoords[q.x],
          shape->texcoords[q.y], shape->texcoords[q.z], uv));
    } else {
      texcoords.push_back(uv);
    }
  }
}

struct terrain_params {
  float size    = 0.1f;
  vec3f center  = zero3f;
  float height  = 0.1f;
  float scale   = 10;
  int   octaves = 8;
  vec3f bottom  = srgb_to_rgb(vec3f{154, 205, 50} / 255);
  vec3f middle  = srgb_to_rgb(vec3f{205, 133, 63} / 255);
  vec3f top     = srgb_to_rgb(vec3f{240, 255, 255} / 255);
};

void make_terrain(
    sio::model* scene, sio::object* object, const terrain_params& params) {
    // iterate through shape positions
    for(int i = 0; i < object->shape->positions.size(); i++) {
      // calculate new vertex position
      vec3f position = object->shape->positions[i];
      position += object->shape->normals[i] * ridge(position * params.scale, params.octaves) * params.height * (1.f - length(position - params.center) / params.size);
      // calculate height percentage and apply color to each vertex
      float perc = position.y / params.height;
      if(perc <= 0.3f) object->shape->colors.push_back(params.bottom);
      else if(perc > 0.6f) object->shape->colors.push_back(params.top);
      else object->shape->colors.push_back(params.middle);
      object->shape->positions[i] = position;
  }
  // update shape normals
  yocto::shape::update_normals(object->shape->normals, object->shape->quads, object->shape->positions);
}

struct displacement_params {
  float height = 0.02f;
  float scale  = 50;
  int   octaves = 8;
  vec3f bottom = srgb_to_rgb(vec3f{64, 224, 208} / 255);
  vec3f top    = srgb_to_rgb(vec3f{244, 164, 96} / 255);
};

void make_displacement(
    sio::model* scene, sio::object* object, const displacement_params& params) {
    // iterate through shape positions
    for(int i = 0; i < object->shape->positions.size(); i++) {
        // calculate new vertex position
        vec3f position = object->shape->positions[i];
        vec3f last_p = vec3f(position);
        position += object->shape->normals[i] * (turbulence(position * params.scale, params.octaves) * params.height);
        // apply color to each vertex
        object->shape->colors.push_back(interpolate_line(params.bottom, params.top, yocto::math::distance(position, last_p) / params.height));
        object->shape->positions[i] = position;
    }
    // update shape normals
    yocto::shape::update_normals(object->shape->normals, object->shape->quads, object->shape->positions);
}

struct hair_params {
  int   num      = 100000;
  int   steps    = 1;
  float lenght   = 0.02f;
  float scale    = 250;
  float strength = 0.01f;
  float gravity  = 0.0f;
  vec3f bottom   = srgb_to_rgb(vec3f{25, 25, 25} / 255);
  vec3f top      = srgb_to_rgb(vec3f{244, 164, 96} / 255);
};

void make_hair(sio::model* scene, sio::object* object, sio::object* hair,
    const hair_params& params) {
    // add a shape to the scene
    hair->shape = add_shape(scene);
    int size = object->shape->positions.size();
    sample_shape(object->shape->positions, object->shape->normals, object->shape->texcoords, object->shape, params.num);
    // calculate length / steps and save it
    float t = params.lenght / params.steps;
    // iterate through shape positions
    for(int i = size; i < object->shape->positions.size(); i++) {
        std::vector<vec3f> positions, colors;
        positions.push_back(object->shape->positions[i]);
        colors.push_back(params.bottom);
        vec3f normal = object->shape->normals[i];
        // generate polyline vertices and colours
        for(int k = 0; k < params.steps; k++) {
            vec3f next = positions[k] + t * normal + noise3(positions[k] * params.scale) * params.strength;
            // apply gravity
            next.y -= params.gravity;
            // calculate next normal
            normal = yocto::math::normalize(next - positions[k]);
            positions.push_back(next);
            // calculate and push back the new color
            colors.push_back(yocto::math::interpolate_line(params.bottom, params.top,
                                                           yocto::math::distance(next, positions[0]) / params.lenght));
        }
        // overwrite the last color with params.top
        colors[params.steps] = params.top;
        // generate new polyline attaching it to hair->shape
        add_polyline(hair->shape, positions, colors);
    }
    // compute tangents
    std::vector<vec3f> tang = yocto::shape::compute_tangents(hair->shape->lines, hair->shape->positions);
    for(int i = 0; i < tang.size(); i++) hair->shape->tangents.push_back(vec4f(tang[i], 0.f));
}

struct grass_params {
  int num = 10000;
};

void make_grass(sio::model* scene, sio::object* object,
    const std::vector<sio::object*>& grasses, const grass_params& params) {
    auto rng = make_rng(198767);
    for (auto grass : grasses) grass->instance = add_instance(scene);
    // generate a random set of points
    sample_shape(object->shape->positions, object->shape->normals, object->shape->texcoords, object->shape, params.num);
    // iterate through shape positions
    for(int i = 0; i < object->shape->positions.size(); i++) {
        vec3f position = object->shape->positions[i];
        sio::object * new_el = yocto::sceneio::add_object(scene);
        // take a random grass element
        auto grass = grasses[rand1i(rng, grasses.size())];

        // assign a shape and material to the object
        new_el->shape = grass->shape;
        new_el->material = grass->material;

        // calculate new object frame from its normal
        new_el->frame.y = object->shape->normals[i];
        new_el->frame.x = yocto::math::normalize(vec3f(1,0,0) - yocto::math::dot(vec3f(1,0,0), new_el->frame.y) * new_el->frame.y);
        new_el->frame.z = yocto::math::cross(new_el->frame.x, new_el->frame.y);
        new_el->frame.o = position;

        float rand = 0.9f + rand1f(rng) * 0.1f;
        // apply a random scale
        new_el->frame *= scaling_frame(vec3f(rand,rand,rand));

        rand = rand1f(rng) * 2 * pif;
        // apply a random rotation to the Y axis
        new_el->frame *= rotation_frame(new_el->frame.y, rand);

        rand = 0.1f + rand1f(rng) * 0.1f;
        // apply a random rotation to the Z axis
        new_el->frame *= rotation_frame(new_el->frame.z, rand);
    }
}

void make_dir(const std::string& dirname) {
  if (sfs::exists(dirname)) return;
  try {
    sfs::create_directories(dirname);
  } catch (...) {
    cli::print_fatal("cannot create directory " + dirname);
  }
}

int main(int argc, const char* argv[]) {
  // command line parameters
  auto terrain      = ""s;
  auto tparams      = terrain_params{};
  auto displacement = ""s;
  auto dparams      = displacement_params{};
  auto hair         = ""s;
  auto hairbase     = ""s;
  auto hparams      = hair_params{};
  auto grass        = ""s;
  auto grassbase    = ""s;
  auto gparams      = grass_params{};
  auto output       = "out.json"s;
  auto filename     = "scene.json"s;

  // parse command line
  auto cli = cli::make_cli("yscenegen", "Make procedural scenes");
  add_option(cli, "--terrain", terrain, "terrain object");
  add_option(cli, "--displacement", displacement, "displacement object");
  add_option(cli, "--hair", hair, "hair object");
  add_option(cli, "--hairbase", hairbase, "hairbase object");
  add_option(cli, "--grass", grass, "grass object");
  add_option(cli, "--grassbase", grassbase, "grassbase object");
  add_option(cli, "--hairnum", hparams.num, "hair number");
  add_option(cli, "--hairlen", hparams.lenght, "hair length");
  add_option(cli, "--hairstr", hparams.strength, "hair strength");
  add_option(cli, "--hairgrav", hparams.gravity, "hair gravity");
  add_option(cli, "--hairstep", hparams.steps, "hair steps");
  add_option(cli, "--output,-o", output, "output scene");
  add_option(cli, "scene", filename, "input scene", true);
  parse_cli(cli, argc, argv);

  // load scene
  auto scene_guard = std::make_unique<sio::model>();
  auto scene       = scene_guard.get();
  auto ioerror     = ""s;
  if (!load_scene(filename, scene, ioerror, cli::print_progress))
    cli::print_fatal(ioerror);

  // create procedural geometry
  if (terrain != "") {
    make_terrain(scene, get_object(scene, terrain), tparams);
  }
  if (displacement != "") {
    make_displacement(scene, get_object(scene, displacement), dparams);
  }
  if (hair != "") {
    make_hair(
        scene, get_object(scene, hairbase), get_object(scene, hair), hparams);
  }
  if (grass != "") {
    auto grasses = std::vector<sio::object*>{};
    for (auto object : scene->objects)
      if (object->name.find(grass) != scene->name.npos)
        grasses.push_back(object);
    make_grass(scene, get_object(scene, grassbase), grasses, gparams);
  }

  // make a directory if needed
  make_dir(sfs::path(output).parent_path());
  if (!scene->shapes.empty())
    make_dir(sfs::path(output).parent_path() / "shapes");
  if (!scene->subdivs.empty())
    make_dir(sfs::path(output).parent_path() / "subdivs");
  if (!scene->textures.empty())
    make_dir(sfs::path(output).parent_path() / "textures");
  if (!scene->instances.empty())
    make_dir(sfs::path(output).parent_path() / "instances");

  // save scene
  if (!save_scene(output, scene, ioerror, cli::print_progress))
    cli::print_fatal(ioerror);

  // done
  return 0;
}
