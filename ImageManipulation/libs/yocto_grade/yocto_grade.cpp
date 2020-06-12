//
// Implementation for Yocto/Grade.
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

#include "yocto_grade.h"

// -----------------------------------------------------------------------------
// COLOR GRADING FUNCTIONS
// -----------------------------------------------------------------------------
namespace yocto::grade {

template <typename Func>
inline void parallel_for(const vec2i& size, Func&& func){
    auto             futures  = std::vector<std::future<void>>{};
    auto             nthreads = std::thread::hardware_concurrency();
    std::atomic<int> next_idx(0);
    for (auto thread_id = 0; thread_id < nthreads; thread_id++) {
        futures.emplace_back(
                std::async(std::launch::async, [&func, &next_idx, size]() {
                    while (true) {
                        auto j = next_idx.fetch_add(1);
                        if (j >= size.y) break;
                        for (auto i = 0; i < size.x; i++) func({i, j});
                    }
                }));
    }
    for (auto& f : futures) f.get();
}
// calculate integral
inline vec3b median(vec3i * arr, int n) {
    float N = n / 2.f;
    vec3i count = vec3i(0);
    vec3b res = vec3b(0);
    for(int i = 0; i < 256; i++) {
        if(count.x < N) {
            res.x = i;
            count.x += arr[i].x;
        }
        if(count.y < N){
            res.y = i;
            count.y += arr[i].y;
        }
        if(count.z < N) {
            res.z = i;
            count.z += arr[i].z;
        }
    }
    return res;
}
inline void quantize_byte_image_mt(img::image<vec4b> & in, img::image<vec4b> & out, int f, vec2i num_threads){
    parallel_for(num_threads, [&](const vec2i& ij) {
        out[ij].x = floor(in[ij].x / f) * f;
        out[ij].y = floor(in[ij].y / f) * f;
        out[ij].z = floor(in[ij].z / f) * f;
    });
}
inline void median_byte_image_mt(img::image<vec4b> & in, img::image<vec4b> & out, int kernel_size, int num_threads) {
    vec2i img_size = in.size();
    std::vector<vec2i> offset, temp;
    auto buffer = img::image<vec4b>{img_size};
    for(int i = 0; i < 4*kernel_size*kernel_size; i++) {
        vec2i p = vec2i(i % (2*kernel_size) - kernel_size , i / (2*kernel_size) - kernel_size);
        offset.push_back(p);
        if(p.x <= -kernel_size) temp.push_back(p);
    }
    int chunk = img_size.y / num_threads + 1;
    parallel_for(vec2i(1, num_threads), [&](const vec2i& ij){
        vec3i hist[256];
        int n;
        for(int y = chunk * ij.y; y < min(chunk * ij.y + chunk, img_size.y); y++) {
            memset(hist, 0, 256 * sizeof(vec3i));
            n = 0;
            for (int x = 0; x < img_size.x; x++) {
                // update histogram
                if(x == 0) {
                    for(int i = 0; i < offset.size(); i++) {
                        vec2i p = vec2i(x, y) + offset[i];
                        if(in.contains(p)) {
                            vec4b indices = in[p.x + p.y * img_size.x];
                            hist[indices.x].x++;
                            hist[indices.y].y++;
                            hist[indices.z].z++;
                            n++;
                        }
                    }
                }
                else {
                    for(int i = 0; i < temp.size(); i++){
                        vec2i p = vec2i(x, y) + temp[i];
                        if(in.contains(p)) {
                            vec4b indices = in[p.x + p.y * img_size.x];
                            hist[indices.x].x--;
                            hist[indices.y].y--;
                            hist[indices.z].z--;
                            n--;
                        }
                        p.x = x + abs(temp[i].x);
                        if(in.contains(p)) {
                            vec4b indices = in[p.x + p.y * img_size.x];
                            hist[indices.x].x++;
                            hist[indices.y].y++;
                            hist[indices.z].z++;
                            n++;
                        }
                    }
                }
                vec3b res = median(hist, n);
                buffer[y * img_size.x + x] = vec4b(res.x, res.y, res.z, in[y * img_size.x + x].w);
            }
        }
    });
    for(int i = 0; i < img_size.x*img_size.y; i++) out[i] = buffer[i];
}
inline void bilateral_filter_mt(img::image<vec4f> & in, img::image<vec4f> & out, int kernel_size, float threshold, int loops, vec2i num_threads) {
    vec2i img_size = in.size();
    std::vector<vec2f> offset;

    for(int y = -kernel_size; y < kernel_size+1; y++) {
        for(int x = -kernel_size; x < kernel_size+1; x++) {
            offset.push_back(vec2f(x, y));
        }
    }

    int offset_size = offset.size();
    auto temp = img::image<vec4f>{img_size};

    for(int k = 0; k < loops; k++){
        parallel_for(num_threads, [&](const vec2i& ij){
            int index = ij.x + ij.y * img_size.x;
            float weight = 0;
            vec2f p = vec2f(ij);
            vec3f mean = vec3f(0,0,0);
            vec3f col_p = xyz(in[index]);
            for(int i = 0; i < offset_size; i++){
                vec2f q = p + offset[i];
                if(in.contains(vec2i(q.x, q.y))) {
                    vec3f col_q = xyz(in[q.x + q.y * img_size.x]);
                    double w = GAUSSIAN(length(p - q), kernel_size) * GAUSSIAN(length(col_p - col_q), threshold);
                    mean += col_q * w;
                    weight += w;
                }
            }
            temp[index] = vec4f(mean * (1.f / weight), in[index].w);
        });
        for(int i = 0; i < img_size.x*img_size.y; i++) out[i] = temp[i];
    }
}
inline void sobel_edge_detection(img::image<vec4f> & in, img::image<vec4f> & out, float threshold){
    static int sobel_dx[9] { -1, 0, 1, -2, 0, 2, -1, 0, 1 };
    static int sobel_dy[9] { 1, 2, 1, 0, 0, 0, -1, -2, -1 };
    vec2i img_size = in.size();

    // convert the image to grayscale
    auto ldr_grayscale = img::image<float>{img_size};
    parallel_for(img_size, [&](const vec2i& ij) {
        ldr_grayscale[ij] = in[ij].x * 0.299f + in[ij].y * 0.587f + in[ij].z * 0.114;
    });

    // apply sobel edge algorithm
    parallel_for(img_size-4, [&](const vec2i& ij) {
        float gx = 0, gy = 0;
        for(int i = 0; i < 9; i++){
            vec2i p = ij + 2 + vec2i(i % 3 - 1,i / 3 - 1);
            if (p.x > 0 && p.x < img_size.x - 1 && p.y > 0 && p.y < img_size.y - 1) {
                gx += ldr_grayscale[p] * sobel_dx[i];
                gy += ldr_grayscale[p] * sobel_dy[i];
            }
        }
        if((abs(gx)+abs(gy)) > threshold) out[ij+2] = vec4f(img::zero3f, in[ij+2].w);
    });
}

img::image<vec4f> grade_image(
    const img::image<vec4f>& img, const grade_params& params) {
    static auto rng = make_rng(1998);

    // image size and output image
    vec2i img_size = vec2i(img.size());
    auto ldr = img::image<vec4f>{img_size};

    // apply tone mapping
    parallel_for(img_size, [&](const vec2i& ij) {
        vec3f p = xyz(img[ij]);
        p *= pow(2, params.exposure);

        if(params.filmic){
            p *= 0.6;
            vec3f pw = pow(p, 2);
            p = (pw * 2.51 + p * 0.03) / (pw * 2.43 + p * 0.59 + 0.14);
        }

        if(params.srgb) p = rgb_to_srgb(p);
        ldr[ij] = vec4f(p, img[ij].w);
    });

    // adapt tint for vec4f operations
    vec4f tint = vec4f(params.tint, 1);
    vec2f img_size_half = vec2f(img.size()) / 2.f;

    parallel_for(img_size, [&](const vec2i& ij) {
        // temporary save w
        float w = ldr[ij].w;

        // calculate tint
        ldr[ij] = clamp(ldr[ij], 0.f, 1.f) * tint;

        // calculate saturation
        float g = (ldr[ij].x + ldr[ij].y + ldr[ij].z) / 3.f;
        ldr[ij] = g + (ldr[ij] - g) * (params.saturation * 2);

        // calculate contrast
        ldr[ij] = gain(ldr[ij], 1 - params.contrast);

        // if vignette differ from 0 then calculate it
        if(params.vignette != 0.f){
            float vr = 1.f - params.vignette;
            float r = length(img_size_half - vec2f(ij)) / length(img_size_half);
            ldr[ij] *= (1.f - smoothstep(vr, 2 * vr, r));
        }

        // calculate film grain
        ldr[ij] += (rand1f(rng) - 0.5f) * params.grain;
        ldr[ij].w = w;
    });

    // if mosaic differ from 0 then calculate it
    if(params.mosaic != 0) {
        parallel_for(img_size, [&](const vec2i& ij){
            int index = ij.x + ij.y * img_size.x;
            int source_index = ij.x - ij.x % params.mosaic + (ij.y - ij.y % params.mosaic) * img_size.x ;
            if(index != source_index) ldr[index] = ldr[source_index];
        });
    }

    // if grid differ from 0 then calculate it
    if(params.grid != 0) {
        parallel_for(img_size, [&](const vec2i& ij){
            if(ij.x % params.grid == 0 || ij.y % params.grid == 0) ldr[ij] *= 0.5f;
        });
    }


    if(!params.custom_filter_switch) return ldr;

    // Watercolor filter - turn an image into a painting
    // This filter is composed by two sections, the first one turns colors so that mimic the ones from a handmade painting while the second part is aimed to find edges
    // The result is the union of these 2 images

    // Downscale the LDR image by a factor f
    vec2i img_size_d = img_size / params.scale_factor;
    auto ldr_downscale = img::resize_image(ldr, img_size_d);

    // Apply a bilateral filter to smooth the colors
    bilateral_filter_mt(ldr_downscale, ldr_downscale, params.bilateral_kernel_size, params.bilateral_threshold, params.bilateral_loops, img_size_d);

    // Upscale back the image
    ldr = img::resize_image(ldr_downscale, img_size);

    // the upscaling filter may have generated negative color values so to be sure that every color channel is equal to or greater than 0 i apply max function to every pixel
    for(int i = 0; i < img_size.y*img_size.x; i++) ldr[i] = clamp(ldr[i], 0, 1);

    // convert the image from float channels to byte
    auto ldr_byte = img::float_to_byte(ldr);

    // to smooth the image and remove any artifacts produced by the upscaling procedure i apply a median filter
    median_byte_image_mt(ldr_byte, ldr_byte, params.median_kernel_size, 15);

    // apply a color quantization factor c to every channel
    quantize_byte_image_mt(ldr_byte, ldr_byte, 10.f, img_size);
    // convert the image from byte channels to float
    ldr = img::byte_to_float(ldr_byte);

    // apply Sobel operator to approximate edges
    sobel_edge_detection(ldr, ldr, params.sobel_threshold);

    return ldr;
}

}  // namespace yocto::grade