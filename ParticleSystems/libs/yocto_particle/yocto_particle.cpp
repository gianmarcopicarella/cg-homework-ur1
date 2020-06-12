//
// Implementation for Yocto/Particle.
//

//
// LICENSE:
//
// Copyright (c) 2020 -- 2020 Fabio Pellacini
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
//

#include "yocto_particle.h"

#include <yocto/yocto_shape.h>

#include <unordered_set>

// -----------------------------------------------------------------------------
// ALIASES
// -----------------------------------------------------------------------------
namespace yocto::particle {

// import math symbols for use
using math::abs;
using math::acos;
using math::atan;
using math::atan2;
using math::clamp;
using math::cos;
using math::exp;
using math::exp2;
using math::flt_max;
using math::fmod;
using math::log;
using math::log2;
using math::make_rng;
using math::max;
using math::min;
using math::perspective_mat;
using math::pow;
using math::ray3f;
using math::sin;
using math::sqrt;
using math::tan;

}  // namespace yocto::particle

// -----------------------------------------------------------------------------
// SIMULATION DATA AND API
// -----------------------------------------------------------------------------
namespace yocto::particle {

// cleanup
scene::~scene() {
  for (auto shape : shapes) delete shape;
  for (auto collider : colliders) delete collider;
}

// Scene creation
par::shape* add_shape(par::scene* scene) {
  return scene->shapes.emplace_back(new par::shape{});
}
par::collider* add_collider(par::scene* scene) {
  return scene->colliders.emplace_back(new par::collider{});
}
par::shape* add_particles(par::scene* scene, const std::vector<int>& points,
    const std::vector<vec3f>& positions, const std::vector<float>& radius,
    float mass, float random_velocity) {
  auto shape               = add_shape(scene);
  shape->points            = points;
  shape->initial_positions = positions;
  shape->initial_normals.assign(shape->positions.size(), {0, 0, 1});
  shape->initial_radius = radius;
  shape->initial_invmass.assign(
      positions.size(), 1 / (mass * positions.size()));
  shape->initial_velocities.assign(positions.size(), {0, 0, 0});
  shape->emit_rngscale = random_velocity;
  return shape;
}
par::shape* add_cloth(par::scene* scene, const std::vector<vec4i>& quads,
    const std::vector<vec3f>& positions, const std::vector<vec3f>& normals,
    const std::vector<float>& radius, float mass, float coeff,
    const std::vector<int>& pinned) {
  auto shape               = add_shape(scene);
  shape->quads             = quads;
  shape->initial_positions = positions;
  shape->initial_normals   = normals;
  shape->initial_radius    = radius;
  shape->initial_invmass.assign(
      positions.size(), 1 / (mass * positions.size()));
  shape->initial_velocities.assign(positions.size(), {0, 0, 0});
  shape->initial_pinned = pinned;
  shape->spring_coeff   = coeff;
  return shape;
}
par::collider* add_collider(par::scene* scene,
    const std::vector<vec3i>& triangles, const std::vector<vec4i>& quads,
    const std::vector<vec3f>& positions, const std::vector<vec3f>& normals,
    const std::vector<float>& radius) {
  auto collider       = add_collider(scene);
  collider->quads     = quads;
  collider->triangles = triangles;
  collider->positions = positions;
  collider->normals   = normals;
  collider->radius    = radius;
  return collider;
}

// Set shapes
void set_velocities(
    par::shape* shape, const vec3f& velocity, float random_scale) {
  shape->emit_velocity = velocity;
  shape->emit_rngscale = random_scale;
}

// Get shape properties
void get_positions(par::shape* shape, std::vector<vec3f>& positions) {
  positions = shape->positions;
}
void get_normals(par::shape* shape, std::vector<vec3f>& normals) {
  normals = shape->normals;
}

}  // namespace yocto::particle

// -----------------------------------------------------------------------------
// SIMULATION DATA AND API
// -----------------------------------------------------------------------------
#include <iostream>
namespace yocto::particle {

// Init simulation
void init_simulation(par::scene* scene, const simulation_params& params) {
  auto sid = 0;
  for (auto shape : scene->shapes) {
    shape->emit_rng   = make_rng(params.seed, (sid++) * 2 + 1);
    // initialize state
    shape->invmass = shape->initial_invmass;
    shape->normals = shape->initial_normals;
    shape->positions = shape->initial_positions;
    shape->radius = shape->initial_radius;
    shape->velocities = shape->initial_velocities;

    // initialize forces
    shape->forces.clear();
    for(int i = 0; i < shape->positions.size(); i++) shape->forces.push_back(zero3f);

    // initialize pinned particles
    for(auto index : shape->initial_pinned) {
      shape->invmass[index] = 0;
    }

    // initialize velocities
    for(auto& velocity : shape->velocities) {
      velocity += math::sample_sphere(math::rand2f(shape->emit_rng)) * shape->emit_rngscale * math::rand1f(shape->emit_rng);
    }

    // clear springs array
    shape->springs.clear();

    // initialize springs
    if(shape->spring_coeff > 0) {
      // add springs for each edge
      for(auto& edge : yocto::shape::get_edges(shape->quads)) {
        shape->springs.push_back({ edge.x, edge.y, math::distance(shape->positions[edge.x], shape->positions[edge.y]), shape->spring_coeff });
      }
      // add 2 diagonal springs foreach square
      for(auto& quad : shape->quads) {
        shape->springs.push_back({ quad.x, quad.z, math::distance(shape->positions[quad.x], shape->positions[quad.z]), shape->spring_coeff });
        shape->springs.push_back({ quad.y, quad.w, math::distance(shape->positions[quad.y], shape->positions[quad.w]), shape->spring_coeff });
      }
    }
  }
  for (auto collider : scene->colliders) {
    // intiialize bvh
    collider->bvh = {};
    // add quads or triangles to bvh
    if(collider->quads.size() > 0)
      yocto::shape::make_quads_bvh(collider->bvh, collider->quads, collider->positions, collider->radius);
    else
      yocto::shape::make_triangles_bvh(collider->bvh, collider->triangles, collider->positions, collider->radius);
  }
}

// check if a point is inside a collider
bool collide_collider(par::collider* collider, const vec3f& position,
    vec3f& hit_position, vec3f& hit_normal) {
  auto ray = ray3f{position, vec3f{0,1,0}};
  yocto::shape::bvh_intersection isec;

  // call the appropriate intersect method
  if(collider->quads.size() > 0)
    isec = intersect_quads_bvh(collider->bvh, collider->quads, collider->positions, ray);
  else
    isec = intersect_triangles_bvh(collider->bvh, collider->triangles, collider->positions, ray);

  if (!isec.hit) return false;

  // calculate hit position and normal
  if(collider->quads.size() > 0) {
    auto q = collider->quads[isec.element];
    hit_position = math::interpolate_quad(collider->positions[q.x], collider->positions[q.y], collider->positions[q.z], collider->positions[q.w], isec.uv);
    hit_normal = math::normalize(math::interpolate_quad(collider->normals[q.x], collider->normals[q.y], collider->normals[q.z], collider->normals[q.w], isec.uv));
  }
  else {
    auto q = collider->triangles[isec.element];
    hit_position = math::interpolate_triangle(collider->positions[q.x], collider->positions[q.y], collider->positions[q.z], isec.uv);
    hit_normal = math::normalize(math::interpolate_triangle(collider->normals[q.x], collider->normals[q.y], collider->normals[q.z], isec.uv));
  }
  // return the angle between hit hormal and ray direction
  return dot(hit_normal, ray.d) > 0;
}

// simulate mass-spring
void simulate_massspring(par::scene* scene, const simulation_params& params) {

  // SAVE OLD POSITIONS
  for (auto& shape : scene->shapes) {
    shape->old_positions = shape->positions;
  }

  // COMPUTE DYNAMICS
  for (auto& shape : scene->shapes) {
    for (int i = 0; i < params.mssteps; i++) {
      auto ddt = params.deltat / params.mssteps;

      // Compute forces
      for (int k = 0; k < shape->positions.size(); k++) {
        if (!shape->invmass[k]) continue;
        shape->forces[k] = vec3f{0, -params.gravity, 0} / shape->invmass[k];
      }

      for (auto& spring : shape->springs) {
        auto& particle0 = shape->positions[spring.vert0];
        auto& particle1 = shape->positions[spring.vert1];
        auto invmass = shape->invmass[spring.vert0] + shape->invmass[spring.vert1];

        if (!invmass) continue;

        auto delta_pos  = particle1 - particle0;
        auto delta_vel  = shape->velocities[spring.vert1] - shape->velocities[spring.vert0];

        auto spring_dir = normalize(delta_pos);
        auto spring_len = length(delta_pos);

        auto force = spring_dir * (spring_len / spring.rest - 1.f) / (spring.coeff * invmass);
        force += dot(delta_vel / spring.rest, spring_dir) * spring_dir / (spring.coeff * 1000 * invmass);

        shape->forces[spring.vert0] += force;
        shape->forces[spring.vert1] -= force;
      }

      for (int k = 0; k < shape->positions.size(); k++) {
        if (!shape->invmass[k]) continue;
        shape->velocities[k] += ddt * shape->forces[k] * shape->invmass[k];
        shape->positions[k] += ddt * shape->velocities[k];
      }
    }
  }

  // Collision detection
  for (auto& shape : scene->shapes) {
    for (int k = 0; k < shape->positions.size(); k++) {
      if (!shape->invmass[k]) continue;
      for (auto& collider : scene->colliders) {
        auto hitpos = zero3f, hit_normal = zero3f;
        if (collide_collider(collider, shape->positions[k], hitpos, hit_normal)) {
          shape->positions[k] = hitpos + hit_normal * 0.005f;
          auto projection = math::dot(shape->velocities[k], hit_normal);
          shape->velocities[k] = (shape->velocities[k] - projection * hit_normal) * (1.f - params.bounce.x) - projection * hit_normal * (1.f - params.bounce.y);
        }
      }
    }
  }

  // ADJUST VELOCITY
  for (auto& shape : scene->shapes) {
    for (int k = 0; k < shape->positions.size(); k++) {
      if (!shape->invmass[k]) continue;
      shape->velocities[k] *= (1.f - params.dumping * params.deltat);
      if (math::length(shape->velocities[k]) < params.minvelocity)
        shape->velocities[k] = {0, 0, 0};
    }
  }

  // RECOMPUTE NORMALS
  for (auto& shape : scene->shapes) {
    if(shape->quads.size() > 0)
      shape->normals = yocto::shape::compute_normals(shape->quads, shape->positions);
    else
      shape->normals = yocto::shape::compute_normals(shape->triangles, shape->positions);
  }
}

// simulate pbd
void simulate_pbd(par::scene* scene, const simulation_params& params) {

  // SAVE OLD POSITOINS
  for (auto& shape : scene->shapes) {
    shape->old_positions = shape->positions;
  }

  // PREDICT POSITIONS
  for (auto& shape : scene->shapes) {
    for(int k = 0; k < shape->positions.size(); k++) {
      if(!shape->invmass[k]) continue;
      shape->velocities[k] += vec3f{0, -params.gravity, 0} * params.deltat;
      shape->positions[k] += shape->velocities[k] * params.deltat;
    }
  }

  // DETECT COLLISIONS
  for (auto& shape : scene->shapes) {
    // clear collisions array
    shape->collisions.clear();
    for(int k = 0; k < shape->positions.size(); k++) {
      if(!shape->invmass[k]) continue;
      for(auto& collider : scene->colliders) {
        auto hitpos = zero3f, hit_normal = zero3f;
        if(!collide_collider(collider, shape->positions[k], hitpos, hit_normal)) continue;
        // if there is a collision, push it back
        shape->collisions.push_back({ k, hitpos, hit_normal });
      }
    }
  }

  // SOLVE CONSTRAINTS
  for (auto& shape : scene->shapes) {
    for (int i = 0; i < params.mssteps; i++) {
      for(auto& spring : shape->springs) {
        auto& particle0 = shape->positions[spring.vert0];
        auto& particle1 = shape->positions[spring.vert1];

        auto invmass = shape->invmass[spring.vert0] + shape->invmass[spring.vert1];

        if (!invmass) continue;

        auto dir = particle1 - particle0;
        // calculate len and normalize dir
        auto len = math::length(dir);
        dir /= len;

        // calculate lambda
        auto lambda = (1.f - spring.coeff) * (len - spring.rest) / invmass;

        shape->positions[spring.vert0] += shape->invmass[spring.vert0] * lambda * dir;
        shape->positions[spring.vert1] -= shape->invmass[spring.vert1] * lambda * dir;
      }

      // handle collisions
      for(auto& collision : shape->collisions) {
        auto& particle = shape->positions[collision.vert];
        if (!shape->invmass[collision.vert]) continue;
        auto projection = dot(particle - collision.position, collision.normal);
        if (projection >= 0) continue;
        particle += -projection * collision.normal;
      }
    }
  }

  // COMPUTE VELOCITIES
  for (auto& shape : scene->shapes) {
    for (int k = 0; k < shape->positions.size(); k++) {
      if (!shape->invmass[k]) continue;
      shape->velocities[k] = (shape->positions[k] - shape->old_positions[k]) / params.deltat;
    }
  }

  // ADJUST VELOCITY
  for (auto& shape : scene->shapes) {
    for (int k = 0; k < shape->positions.size(); k++) {
      if (!shape->invmass[k]) continue;
      shape->velocities[k] *= (1.f - params.dumping * params.deltat);
      if (math::length(shape->velocities[k]) < params.minvelocity)
        shape->velocities[k] = {0, 0, 0};
    }
  }

  // RECOMPUTE NORMALS
  for (auto& shape : scene->shapes) {
    if(shape->quads.size() > 0)
      shape->normals = yocto::shape::compute_normals(shape->quads, shape->positions);
    else
      shape->normals = yocto::shape::compute_normals(shape->triangles, shape->positions);
  }
}

// Simulate one step
void simulate_frame(par::scene* scene, const simulation_params& params) {
  switch (params.solver) {
    case solver_type::mass_spring: return simulate_massspring(scene, params);
    case solver_type::position_based: return simulate_pbd(scene, params);
    default: throw std::invalid_argument("unknown solver");
  }
}

// Simulate the whole sequence
void simulate_frames(par::scene* scene, const simulation_params& params,
    progress_callback progress_cb) {
  // handle progress
  auto progress = vec2i{0, 1 + (int)params.frames};

  if (progress_cb) progress_cb("init simulation", progress.x++, progress.y);
  init_simulation(scene, params);

  for (auto idx = 0; idx < params.frames; idx++) {
    if (progress_cb) progress_cb("simulate frames", progress.x++, progress.y);
    simulate_frame(scene, params);
  }

  if (progress_cb) progress_cb("simulate frames", progress.x++, progress.y);
}

}  // namespace yocto::particle
