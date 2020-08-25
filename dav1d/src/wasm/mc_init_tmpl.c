/*
 * Copyright © 2018, VideoLAN and dav1d authors
 * Copyright © 2018, Two Orioles, LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "src/cpu.h"
#include "src/mc.h"

#if BITDEPTH == 8

decl_mc_fn(dav1d_put_8tap_regular_wasm);
decl_mc_fn(dav1d_put_8tap_regular_smooth_wasm);
decl_mc_fn(dav1d_put_8tap_regular_sharp_wasm);
decl_mc_fn(dav1d_put_8tap_smooth_wasm);
decl_mc_fn(dav1d_put_8tap_smooth_regular_wasm);
decl_mc_fn(dav1d_put_8tap_smooth_sharp_wasm);
decl_mc_fn(dav1d_put_8tap_sharp_wasm);
decl_mc_fn(dav1d_put_8tap_sharp_regular_wasm);
decl_mc_fn(dav1d_put_8tap_sharp_smooth_wasm);
decl_mc_fn(dav1d_put_bilin_wasm);

decl_mct_fn(dav1d_prep_8tap_regular_wasm);
decl_mct_fn(dav1d_prep_8tap_regular_smooth_wasm);
decl_mct_fn(dav1d_prep_8tap_regular_sharp_wasm);
decl_mct_fn(dav1d_prep_8tap_smooth_wasm);
decl_mct_fn(dav1d_prep_8tap_smooth_regular_wasm);
decl_mct_fn(dav1d_prep_8tap_smooth_sharp_wasm);
decl_mct_fn(dav1d_prep_8tap_sharp_wasm);
decl_mct_fn(dav1d_prep_8tap_sharp_regular_wasm);
decl_mct_fn(dav1d_prep_8tap_sharp_smooth_wasm);
decl_mct_fn(dav1d_prep_bilin_wasm);

//decl_avg_fn(dav1d_avg_wasm);
//decl_w_avg_fn(dav1d_w_avg_wasm);
//decl_mask_fn(dav1d_mask_wasm);
//decl_w_mask_fn(dav1d_w_mask_420_wasm);
//decl_w_mask_fn(dav1d_w_mask_422_wasm);
//decl_w_mask_fn(dav1d_w_mask_444_wasm);
//decl_blend_fn(dav1d_blend_wasm);
//decl_blend_dir_fn(dav1d_blend_v_wasm);
//decl_blend_dir_fn(dav1d_blend_h_wasm);

//decl_warp8x8_fn(dav1d_warp_affine_8x8_wasm);
//decl_warp8x8t_fn(dav1d_warp_affine_8x8t_wasm);

//decl_emu_edge_fn(dav1d_emu_edge_wasm);

#endif

void bitfn(dav1d_mc_dsp_init_wasm)(Dav1dMCDSPContext *const c) {
#define init_mc_fn(type, name, suffix) \
    c->mc[type] = dav1d_put_##name##_##suffix
#define init_mct_fn(type, name, suffix) \
    c->mct[type] = dav1d_prep_##name##_##suffix
    const unsigned flags = dav1d_get_cpu_flags();

    if(!(flags & DAV1D_WASM_CPU_FLAG_SIMD_128))
        return;

#if BITDEPTH == 8
    init_mc_fn (FILTER_2D_8TAP_REGULAR,        8tap_regular,        wasm);
    init_mc_fn (FILTER_2D_8TAP_REGULAR_SMOOTH, 8tap_regular_smooth, wasm);
    init_mc_fn (FILTER_2D_8TAP_REGULAR_SHARP,  8tap_regular_sharp,  wasm);
    init_mc_fn (FILTER_2D_8TAP_SMOOTH_REGULAR, 8tap_smooth_regular, wasm);
    init_mc_fn (FILTER_2D_8TAP_SMOOTH,         8tap_smooth,         wasm);
    init_mc_fn (FILTER_2D_8TAP_SMOOTH_SHARP,   8tap_smooth_sharp,   wasm);
    init_mc_fn (FILTER_2D_8TAP_SHARP_REGULAR,  8tap_sharp_regular,  wasm);
    init_mc_fn (FILTER_2D_8TAP_SHARP_SMOOTH,   8tap_sharp_smooth,   wasm);
    init_mc_fn (FILTER_2D_8TAP_SHARP,          8tap_sharp,          wasm);
    //init_mc_fn (FILTER_2D_BILINEAR,            bilin,               wasm);

    init_mct_fn(FILTER_2D_8TAP_REGULAR,        8tap_regular,        wasm);
    init_mct_fn(FILTER_2D_8TAP_REGULAR_SMOOTH, 8tap_regular_smooth, wasm);
    init_mct_fn(FILTER_2D_8TAP_REGULAR_SHARP,  8tap_regular_sharp,  wasm);
    init_mct_fn(FILTER_2D_8TAP_SMOOTH_REGULAR, 8tap_smooth_regular, wasm);
    init_mct_fn(FILTER_2D_8TAP_SMOOTH,         8tap_smooth,         wasm);
    init_mct_fn(FILTER_2D_8TAP_SMOOTH_SHARP,   8tap_smooth_sharp,   wasm);
    init_mct_fn(FILTER_2D_8TAP_SHARP_REGULAR,  8tap_sharp_regular,  wasm);
    init_mct_fn(FILTER_2D_8TAP_SHARP_SMOOTH,   8tap_sharp_smooth,   wasm);
    init_mct_fn(FILTER_2D_8TAP_SHARP,          8tap_sharp,          wasm);
    //init_mct_fn(FILTER_2D_BILINEAR,            bilin,               wasm);

    //c->avg = dav1d_avg_wasm;
    //c->w_avg = dav1d_w_avg_wasm;
    //c->mask = dav1d_mask_wasm;
    //c->w_mask[0] = dav1d_w_mask_444_wasm;
    //c->w_mask[1] = dav1d_w_mask_422_wasm;
    //c->w_mask[2] = dav1d_w_mask_420_wasm;
    //c->blend = dav1d_blend_wasm;
    //c->blend_v = dav1d_blend_v_wasm;
    //c->blend_h = dav1d_blend_h_wasm;

    //c->warp8x8  = dav1d_warp_affine_8x8_wasm;
    //c->warp8x8t = dav1d_warp_affine_8x8t_wasm;

    //c->emu_edge = dav1d_emu_edge_wasm;
#endif
}
