//
// Yocto/Grade: Tiny library for color grading.
//

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

#ifndef _YOCTO_GRADE_
#define _YOCTO_GRADE_

#define GAUSSIAN(x, o) ((1.f / sqrt(2*pif*o*o)) * exp(-((x*x)/(2*o*o))))

#include <yocto/yocto_image.h>
#include <yocto/yocto_math.h>
#include <thread>
#include <atomic>
#include <future>

// -----------------------------------------------------------------------------
// COLOR GRADING FUNCTIONS
// -----------------------------------------------------------------------------
namespace yocto::grade {

// Using directives
using namespace yocto::math;
namespace img = yocto::image;

// Color grading parameters
struct grade_params {
  float exposure   = 0.0f;
  bool  filmic     = false;
  bool  srgb       = true;
  vec3f tint       = vec3f{1, 1, 1};
  float saturation = 0.5f;
  float contrast   = 0.5f;
  float vignette   = 0.0f;
  float grain      = 0.0f;
  int   mosaic     = 0;
  int   grid       = 0;
  bool custom_filter_switch = false;
  int scale_factor = 4; /* 1 - 4 */
  int bilateral_kernel_size = 4; /* 1 - 5 */
  float bilateral_threshold = 0.04f; /* 0.01 - 0.2 */
  int bilateral_loops = 5; /* 1 - 5 */
  int median_kernel_size = 4; /* 1 - 4 */
  float sobel_threshold = 0.3f; /* 0.0 - 1.0 */
};

// Custom filter utils
template <typename Func>
void parallel_for(const vec2i& size, Func&& func);
// draw edges to the input image
void sobel_edge_detection(img::image<vec4f> & in, img::image<vec4f> & out, float threshold);
// Applies a bilateral filter to every image pixel
void bilateral_filter_mt(img::image<vec4f> & in, img::image<vec4f> & out, int kernel_size, float threshold, int loops, vec2i num_threads);
// Applies a median filter for every image pixel
void median_byte_image_mt(img::image<vec4b> & in, img::image<vec4b> & out, int kernel_size, int num_threads);
// Quantize byte images channels by a factor f
void quantize_byte_image_mt(img::image<vec4b> & in, img::image<vec4b> & out, int f, vec2i num_threads);
// calculate median value for every RGB channel
vec3b median(vec3i * arr, int n);

// Grading functions
img::image<vec4f> grade_image(
    const img::image<vec4f>& img, const grade_params& params);

};  // namespace yocto::grade

#endif
