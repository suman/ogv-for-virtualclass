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

#define BITDEPTH 8

#include "config.h"

#include <assert.h>
#include <stdlib.h>
#include <string.h>

#include "common/attributes.h"
#include "common/intops.h"

#include "src/mc.h"
#include "src/tables.h"

#include "wasm_simd.h"


#if BITDEPTH == 8
#define get_intermediate_bits(bitdepth_max) 4
// Output in interval [-5132, 9212], fits in int16_t as is
#define PREP_BIAS 0
#else
// 4 for 10 bits/component, 2 for 12 bits/component
#define get_intermediate_bits(bitdepth_max) (14 - bitdepth_from_max(bitdepth_max))
// Output in interval [-20588, 36956] (10-bit), [-20602, 36983] (12-bit)
// Subtract a bias to ensure the output fits in int16_t
#define PREP_BIAS 8192
#endif

static NOINLINE void
put_wasm(pixel *dst, const ptrdiff_t dst_stride,
         const pixel *src, const ptrdiff_t src_stride, const int w, int h)
{
    if (w > 16) {
        do {
            for (int x = 0; x < w; x += 16) {
                *(uint8x16 *)(dst + x) = *(uint8x16 *)(src + x);
            }

            dst += dst_stride;
            src += src_stride;
        } while (--h);
    } else if (w == 16) {
        do {
            *(uint8x16 *)dst = *(uint8x16 *)src;

            dst += dst_stride;
            src += src_stride;
        } while (--h);
    } else if (w == 8) {
        do {
            *(uint64_t *)dst = *(uint64_t *)src;

            dst += dst_stride;
            src += src_stride;
        } while (--h);
    } else if (w == 4) {
        do {
            *(uint32_t *)dst = *(uint32_t *)src;

            dst += dst_stride;
            src += src_stride;
        } while (--h);
    } else if (w == 2) {
        do {
            *(uint16_t *)dst = *(uint16_t *)src;

            dst += dst_stride;
            src += src_stride;
        } while (--h);
    } else {
        abort();
    }
}

static NOINLINE void
prep_wasm(int16_t *tmp, const pixel *src, const ptrdiff_t src_stride,
          const int w, int h HIGHBD_DECL_SUFFIX)
{
    const int intermediate_bits = get_intermediate_bits(bitdepth_max);
    //
    // Vector route is slow because of the shift hacks for now.
    // Put this back once it's working.
    //
    // Note for hbd the intermediate values won't fit in 16 bits.
    //
    if (w >= 8) {
        do {
            for (int x = 0; x < w; x += 8) {
                // Read 16 8-bit pixels, of which we'll expand 8 of them
                const uint16x8 px_raw = *(uint16x8 *)(src + x);
                const int16x8 px_v = expand_pixels(px_raw);
                // This is a constant 4 for 8bpc
                const int16x8 shifted_v = wasm_i16x8_shl(px_v, intermediate_bits);
                // Note: for hbd, also subtract PREP_BIAS! It's zero for 8bpc.
                // Write 8 shifted 16-bit pixels into the tmp array.
                *(int16x8 *)(tmp + x) = shifted_v;
            }
            tmp += w;
            src += src_stride;
        } while (--h);
    } else {
        do {
            for (int x = 0; x < w; x++)
                tmp[x] = (src[x] << intermediate_bits) - PREP_BIAS;
            tmp += w;
            src += src_stride;
        } while (--h);
    }
}

#define FILTER_8TAP(src, x, F, stride) \
    (F[0] * src[x + -3 * stride] + \
     F[1] * src[x + -2 * stride] + \
     F[2] * src[x + -1 * stride] + \
     F[3] * src[x + +0 * stride] + \
     F[4] * src[x + +1 * stride] + \
     F[5] * src[x + +2 * stride] + \
     F[6] * src[x + +3 * stride] + \
     F[7] * src[x + +4 * stride])

#define DAV1D_FILTER_8TAP_RND(src, x, F, stride, sh) \
    ((FILTER_8TAP(src, x, F, stride) + ((1 << (sh)) >> 1)) >> (sh))

#define DAV1D_FILTER_8TAP_CLIP(src, x, F, stride, sh) \
    iclip_pixel(DAV1D_FILTER_8TAP_RND(src, x, F, stride, sh))


// Vectorized

#define clip_pixel_vec(v) clip_vec((v), zero_v, maxpixel_v)

#define clip_pixel_vec32(v) clip_vec32((v), zero32_v, maxpixel32_v)

#define FILTER_8TAP_VEC(src, x, F, stride) \
    (F[0] * expand_pixels(read_u8x16(src + x + -3 * stride)) + \
     F[1] * expand_pixels(read_u8x16(src + x + -2 * stride)) + \
     F[2] * expand_pixels(read_u8x16(src + x + -1 * stride)) + \
     F[3] * expand_pixels(read_u8x16(src + x + +0 * stride)) + \
     F[4] * expand_pixels(read_u8x16(src + x + +1 * stride)) + \
     F[5] * expand_pixels(read_u8x16(src + x + +2 * stride)) + \
     F[6] * expand_pixels(read_u8x16(src + x + +3 * stride)) + \
     F[7] * expand_pixels(read_u8x16(src + x + +4 * stride)))

#define DAV1D_FILTER_8TAP_RND_VEC(src, x, F, stride, sh, rnd_v) \
    wasm_i16x8_shr((FILTER_8TAP_VEC(src, x, F, stride) + rnd_v), (sh))

// Version that takes 4 pixels each from 2 row
#define FILTER_8TAP_VEC_4X2(src, x, F, stride, src_stride) \
    (F[0] * expand_pixels(read_u8x16_4x2(src + x + -3 * stride, src_stride)) + \
     F[1] * expand_pixels(read_u8x16_4x2(src + x + -2 * stride, src_stride)) + \
     F[2] * expand_pixels(read_u8x16_4x2(src + x + -1 * stride, src_stride)) + \
     F[3] * expand_pixels(read_u8x16_4x2(src + x + +0 * stride, src_stride)) + \
     F[4] * expand_pixels(read_u8x16_4x2(src + x + +1 * stride, src_stride)) + \
     F[5] * expand_pixels(read_u8x16_4x2(src + x + +2 * stride, src_stride)) + \
     F[6] * expand_pixels(read_u8x16_4x2(src + x + +3 * stride, src_stride)) + \
     F[7] * expand_pixels(read_u8x16_4x2(src + x + +4 * stride, src_stride)))

#define DAV1D_FILTER_8TAP_RND_VEC_4X2(src, x, F, stride, src_stride, sh, rnd_v) \
    wasm_i16x8_shr((FILTER_8TAP_VEC_4X2(src, x, F, stride, src_stride) + rnd_v), (sh))

// Version that takes 2 pixels each from 4 rows
#define FILTER_8TAP_VEC_2X4(src, x, F, stride, src_stride) \
    (F[0] * expand_pixels(read_u8x16_2x4(src + x + -3 * stride, src_stride)) + \
     F[1] * expand_pixels(read_u8x16_2x4(src + x + -2 * stride, src_stride)) + \
     F[2] * expand_pixels(read_u8x16_2x4(src + x + -1 * stride, src_stride)) + \
     F[3] * expand_pixels(read_u8x16_2x4(src + x + +0 * stride, src_stride)) + \
     F[4] * expand_pixels(read_u8x16_2x4(src + x + +1 * stride, src_stride)) + \
     F[5] * expand_pixels(read_u8x16_2x4(src + x + +2 * stride, src_stride)) + \
     F[6] * expand_pixels(read_u8x16_2x4(src + x + +3 * stride, src_stride)) + \
     F[7] * expand_pixels(read_u8x16_2x4(src + x + +4 * stride, src_stride)))

#define DAV1D_FILTER_8TAP_RND_VEC_2X4(src, x, F, stride, src_stride, sh, rnd_v) \
    wasm_i16x8_shr((FILTER_8TAP_VEC_2X4(src, x, F, stride, src_stride) + rnd_v), (sh))

// Version that takes 2 pixels each from 3 rows (for 2x2 blocks in put_8tap with 3 rows left at end)
#define FILTER_8TAP_VEC_2X3(src, x, F, stride, src_stride) \
    (F[0] * expand_pixels(read_u8x16_2x3(src + x + -3 * stride, src_stride)) + \
     F[1] * expand_pixels(read_u8x16_2x3(src + x + -2 * stride, src_stride)) + \
     F[2] * expand_pixels(read_u8x16_2x3(src + x + -1 * stride, src_stride)) + \
     F[3] * expand_pixels(read_u8x16_2x3(src + x + +0 * stride, src_stride)) + \
     F[4] * expand_pixels(read_u8x16_2x3(src + x + +1 * stride, src_stride)) + \
     F[5] * expand_pixels(read_u8x16_2x3(src + x + +2 * stride, src_stride)) + \
     F[6] * expand_pixels(read_u8x16_2x3(src + x + +3 * stride, src_stride)) + \
     F[7] * expand_pixels(read_u8x16_2x3(src + x + +4 * stride, src_stride)))

#define DAV1D_FILTER_8TAP_RND_VEC_2X3(src, x, F, stride, src_stride, sh, rnd_v) \
    wasm_i16x8_shr((FILTER_8TAP_VEC_2X3(src, x, F, stride, src_stride) + rnd_v), (sh))


// For the mid-level filter where we have an int16-sized buffer
#define FILTER_8TAP_VEC_16(src, x, F, stride) \
    (F[0] * expand_pixels32_s(read_i16x8(src + x + -3 * stride)) + \
     F[1] * expand_pixels32_s(read_i16x8(src + x + -2 * stride)) + \
     F[2] * expand_pixels32_s(read_i16x8(src + x + -1 * stride)) + \
     F[3] * expand_pixels32_s(read_i16x8(src + x + +0 * stride)) + \
     F[4] * expand_pixels32_s(read_i16x8(src + x + +1 * stride)) + \
     F[5] * expand_pixels32_s(read_i16x8(src + x + +2 * stride)) + \
     F[6] * expand_pixels32_s(read_i16x8(src + x + +3 * stride)) + \
     F[7] * expand_pixels32_s(read_i16x8(src + x + +4 * stride)))

#define DAV1D_FILTER_8TAP_RND_VEC_16(src, x, F, stride, sh, rnd_v) \
    wasm_i32x4_shr((FILTER_8TAP_VEC_16(src, x, F, stride) + rnd_v), (sh))

#define FILTER_8TAP_VEC_16_2X2(src, x, F, stride) \
    (F[0] * expand_pixels32_s(read_i16x8_2x2(src + x + -3 * stride, stride)) + \
     F[1] * expand_pixels32_s(read_i16x8_2x2(src + x + -2 * stride, stride)) + \
     F[2] * expand_pixels32_s(read_i16x8_2x2(src + x + -1 * stride, stride)) + \
     F[3] * expand_pixels32_s(read_i16x8_2x2(src + x + +0 * stride, stride)) + \
     F[4] * expand_pixels32_s(read_i16x8_2x2(src + x + +1 * stride, stride)) + \
     F[5] * expand_pixels32_s(read_i16x8_2x2(src + x + +2 * stride, stride)) + \
     F[6] * expand_pixels32_s(read_i16x8_2x2(src + x + +3 * stride, stride)) + \
     F[7] * expand_pixels32_s(read_i16x8_2x2(src + x + +4 * stride, stride)))

#define DAV1D_FILTER_8TAP_RND_VEC_16_2X2(src, x, F, stride, sh, rnd_v) \
    wasm_i32x4_shr((FILTER_8TAP_VEC_16_2X2(src, x, F, stride) + rnd_v), (sh))


#define GET_H_FILTER(mx) \
    const int8_t *const fh = !(mx) ? NULL : w > 4 ? \
        dav1d_mc_subpel_filters[filter_type & 3][(mx) - 1] : \
        dav1d_mc_subpel_filters[3 + (filter_type & 1)][(mx) - 1]

#define GET_V_FILTER(my) \
    const int8_t *const fv = !(my) ? NULL : h > 4 ? \
        dav1d_mc_subpel_filters[filter_type >> 2][(my) - 1] : \
        dav1d_mc_subpel_filters[3 + ((filter_type >> 2) & 1)][(my) - 1]

#define GET_FILTERS() \
    GET_H_FILTER(mx); \
    GET_V_FILTER(my)

#define GET_FILTER_FH_V() \
    int16x8 fh_v[8]; \
    for (int k = 0; k < 8; k++) { \
        fh_v[k] = wasm_i16x8_splat(fh[k]); \
    }

#define GET_FILTER_FV_V() \
    int16x8 fv_v[8]; \
    for (int k = 0; k < 8; k++) { \
        fv_v[k] = wasm_i16x8_splat(fv[k]); \
    }

#define GET_FILTER_FV_V32() \
    int32x4 fv_v[8]; \
    for (int k = 0; k < 8; k++) { \
        fv_v[k] = wasm_i32x4_splat(fv[k]); \
    }

static NOINLINE void
put_8tap_wasm(pixel *dst, ptrdiff_t dst_stride,
              const pixel *src, ptrdiff_t src_stride,
              const int w, int h, const int mx, const int my,
              const int filter_type HIGHBD_DECL_SUFFIX)
{
    const int intermediate_bits = get_intermediate_bits(bitdepth_max);
    const int intermediate_rnd = (1 << intermediate_bits) >> 1;
    const int16x8 intermediate_rnd_v = wasm_i16x8_splat(intermediate_rnd);

    GET_FILTERS();
    dst_stride = PXSTRIDE(dst_stride);
    src_stride = PXSTRIDE(src_stride);

    // w can be as small as 2
    if (fh) {
        if (fv) {
            int tmp_h = h + 7;
            int16_t mid[128 * 135], *mid_ptr = mid;

            int sh = 6 - intermediate_bits;
            int16x8 rnd_v = wasm_i16x8_splat((1 << (sh)) >> 1);

            src -= src_stride * 3;
            if (w > 8) {
                GET_FILTER_FH_V();
                do {
                    for (int x = 0; x < w; x += 8) {
                        // Read and process 8 pixels
                        const int16x8 mid_v = DAV1D_FILTER_8TAP_RND_VEC(src, x, fh_v, 1, sh, rnd_v);
                        write_i16x8(mid_ptr + x, mid_v);
                    }

                    mid_ptr += 128;
                    src += src_stride;
                } while (--tmp_h);
            } else if (w == 8) {
                GET_FILTER_FH_V();
                do {
                    // Read and process 8 pixels
                    const int16x8 mid_v = DAV1D_FILTER_8TAP_RND_VEC(src, 0, fh_v, 1, sh, rnd_v);
                    write_i16x8(mid_ptr, mid_v);

                    mid_ptr += 128;
                    src += src_stride;
                } while (--tmp_h);
            } else if (w == 4) {
                GET_FILTER_FH_V();
                do {
                    // Read and process 8 pixels, 4 each from 2 rows
                    const int16x8 mid_v = DAV1D_FILTER_8TAP_RND_VEC_4X2(src, 0, fh_v, 1, src_stride, sh, rnd_v);
                    write_i16x8_4x2(mid_ptr, 128, mid_v);

                    mid_ptr += 128 * 2;
                    src += src_stride * 2;
                    --tmp_h;
                } while (--tmp_h > 1);

                // odd line
                // Read and process 4 pixels plus some stray leftovers
                const int16x8 mid_v = DAV1D_FILTER_8TAP_RND_VEC(src, 0, fh_v, 1, sh, rnd_v);
                write_i16x8_4x1(mid_ptr, mid_v);

                mid_ptr += 128;
                src += src_stride;
#if 0
            } else if ( w == 2 ) {
                // This is a wash for now, try again after shift and splat get fixed.
                //
                // Run 8 pixels at a time, 2 each from 4 lines
                GET_FILTER_FH_V();
                do {
                    const int16x8 mid_v = DAV1D_FILTER_8TAP_RND_VEC_2X4(src, 0, fh_v, 1, src_stride, sh, rnd_v);
                    write_i16x8_2x4(mid_ptr, 128, mid_v);

                    mid_ptr += 128 * 4;
                    src += src_stride * 4;
                    tmp_h -= 3;
                } while (--tmp_h > 3);

                // 3 or 1 odd line(s)
                if (tmp_h >= 3) {
                    const int16x8 mid_v = DAV1D_FILTER_8TAP_RND_VEC_2X3(src, 0, fh_v, 1, src_stride, sh, rnd_v);
                    write_i16x8_2x3(mid_ptr, 128, mid_v);
                } else {
                    // Single line. Don't bother vectorizing.
                    for (int x = 0; x < w; x++)
                        mid_ptr[x] = DAV1D_FILTER_8TAP_RND(src, x, fh, 1,
                                                           6 - intermediate_bits);
                }
#endif
            } else {
                do {
                    for (int x = 0; x < w; x++)
                        mid_ptr[x] = DAV1D_FILTER_8TAP_RND(src, x, fh, 1,
                                                           6 - intermediate_bits);
                    mid_ptr += 128;
                    src += src_stride;
                } while (--tmp_h);
            }

            sh = 6 + intermediate_bits;
            rnd_v = wasm_i16x8_splat((1 << (sh)) >> 1);
            const int32x4 rnd32_v = wasm_i32x4_splat((1 << (sh)) >> 1);

            mid_ptr = mid + 128 * 3;
            if (w > 4) {
                GET_FILTER_FV_V32();
                do {
                    // This run can only do 4 pixels at a time because intermediates
                    // don't fit in 16 bits, and we only have 128-bits total.
                    for (int x = 0; x < w; x += 4) {
                        // Read and process 4 pixels
                        const int32x4 px_v = DAV1D_FILTER_8TAP_RND_VEC_16(mid_ptr, x, fv_v, 128, sh, rnd32_v);
                        write_u8x16_4x1(dst + x, merge_pixels32(px_v));
                    }

                    mid_ptr += 128;
                    dst += dst_stride;
                } while (--h);
            } else if (w == 4) {
                GET_FILTER_FV_V32();
                do {
                    // Read and process 4 pixels
                    const int32x4 px_v = DAV1D_FILTER_8TAP_RND_VEC_16(mid_ptr, 0, fv_v, 128, sh, rnd32_v);
                    write_u8x16_4x1(dst, merge_pixels32(px_v));

                    mid_ptr += 128;
                    dst += dst_stride;
                } while (--h);
            } else /* w == 2 */ {
                GET_FILTER_FV_V32();
                do {
                    // Read and process 4 pixels, 2 from each of 2 rows
                    const int32x4 px_v = DAV1D_FILTER_8TAP_RND_VEC_16_2X2(mid_ptr, 0, fv_v, 128, sh, rnd32_v);
                    write_u8x16_2x2(dst, dst_stride, merge_pixels32(px_v));
                    mid_ptr += 256;
                    dst += dst_stride * 2;
                    --h;
                } while (--h);
            }
        } else {
            int sh = 6 - intermediate_bits;
            int16x8 rnd_v = wasm_i16x8_splat((1 << (sh)) >> 1);

            if (w > 8) {
                GET_FILTER_FH_V();
                do {
                    for (int x = 0; x < w; x += 8) {
                        // Read and process 8 pixels
                        int16x8 px_v = DAV1D_FILTER_8TAP_RND_VEC(src, x, fh_v, 1, sh, rnd_v);
                        px_v = (px_v + intermediate_rnd_v) >> intermediate_bits;

                        // Write 8 pixels
                        write_u8x16_8x1(dst + x, merge_pixels(px_v));
                    }

                    dst += dst_stride;
                    src += src_stride;
                } while (--h);
            } else if (w == 8) {
                GET_FILTER_FH_V();
                do {
                    // Read and process 8 pixels
                    int16x8 px_v = DAV1D_FILTER_8TAP_RND_VEC(src, 0, fh_v, 1, sh, rnd_v);
                    px_v = (px_v + intermediate_rnd_v) >> intermediate_bits;

                    // Write 8 pixels
                    write_u8x16_8x1(dst, merge_pixels(px_v));

                    dst += dst_stride;
                    src += src_stride;
                } while (--h);
            } else if (w == 4) {
                GET_FILTER_FH_V();
                do {
                    // Read and process 8 pixels, 4 each from 2 rows
                    int16x8 px_v = DAV1D_FILTER_8TAP_RND_VEC_4X2(src, 0, fh_v, 1, src_stride, sh, rnd_v);
                    px_v = (px_v + intermediate_rnd_v) >> intermediate_bits;
                    write_u8x16_4x2(dst, dst_stride, merge_pixels(px_v));

                    dst += dst_stride * 2;
                    src += src_stride * 2;
                    --h;
                } while (--h);
#if 0
            } else if (w == 2) {
                // This is a wash; try again when shift and splat are fixed.
                //
                // Read and process 8 pixels, 2 from each of 4 rows
                GET_FILTER_FH_V();
                const int16x8 zero_v = wasm_i16x8_splat(0);
                const int16x8 maxpixel_v = wasm_i16x8_splat(255);
                while (h >= 4) {
                    int16x8 px_v = DAV1D_FILTER_8TAP_RND_VEC_2X4(src, 0, fh_v, 1, src_stride, sh, rnd_v);
                    px_v = (px_v + intermediate_rnd_v) >> intermediate_bits;
                    write_u8x16_2x4(dst, dst_stride, merge_pixels(px_v));

                    dst += dst_stride * 4;
                    src += src_stride * 4;
                    h -= 4;
                }
                while (h) {
                    for (int x = 0; x < w; x++) {
                        const int px = DAV1D_FILTER_8TAP_RND(src, x, fh, 1, sh);
                        dst[x] = iclip_pixel((px + intermediate_rnd) >> intermediate_bits);
                    }

                    dst += dst_stride;
                    src += src_stride;
                    --h;
                }
#endif
            } else {
                do {
                    for (int x = 0; x < w; x++) {
                        const int px = DAV1D_FILTER_8TAP_RND(src, x, fh, 1,
                                                            6 - intermediate_bits);
                        dst[x] = iclip_pixel((px + intermediate_rnd) >> intermediate_bits);
                    }

                    dst += dst_stride;
                    src += src_stride;
                } while (--h);
            }
        }
    } else if (fv) {
        int sh = 6;
        int16x8 rnd_v = wasm_i16x8_splat((1 << (sh)) >> 1);
        if (w > 8) {
            GET_FILTER_FV_V();
            do {
                // Read and process 8 pixels at a time
                for (int x = 0; x < w; x += 8) {
                    const int16x8 px_v = DAV1D_FILTER_8TAP_RND_VEC(src, x, fv_v, src_stride, sh, rnd_v);
                    write_u8x16_8x1(dst + x, merge_pixels(px_v));
                }

                dst += dst_stride;
                src += src_stride;
            } while (--h);
        } else if (w == 8) {
            GET_FILTER_FV_V();
            do {
                // Read and process 8 pixels at a time
                const int16x8 px_v = DAV1D_FILTER_8TAP_RND_VEC(src, 0, fv_v, src_stride, sh, rnd_v);
                write_u8x16_8x1(dst, merge_pixels(px_v));

                dst += dst_stride;
                src += src_stride;
            } while (--h);
        } else if (w == 4) {
            GET_FILTER_FV_V();
            do {
                // Read and process 8 pixels at a time, 4 from each of 2 rows
                const int16x8 px_v = DAV1D_FILTER_8TAP_RND_VEC_4X2(src, 0, fv_v, src_stride, src_stride, sh, rnd_v);
                write_u8x16_4x2(dst, dst_stride, merge_pixels(px_v));

                dst += dst_stride * 2;
                src += src_stride * 2;
                --h;
            } while (--h);
        } else /* w == 2 */ {
            do {
                for (int x = 0; x < w; x++)
                    dst[x] = DAV1D_FILTER_8TAP_CLIP(src, x, fv, src_stride, sh);

                dst += dst_stride;
                src += src_stride;
            } while (--h);
        }
    } else
        put_wasm(dst, dst_stride, src, src_stride, w, h);
}

static NOINLINE void
prep_8tap_wasm(int16_t *tmp, const pixel *src, ptrdiff_t src_stride,
               const int w, int h, const int mx, const int my,
               const int filter_type HIGHBD_DECL_SUFFIX)
{
    const int intermediate_bits = get_intermediate_bits(bitdepth_max);
    GET_FILTERS();
    src_stride = PXSTRIDE(src_stride);

    if (fh) {
        if (fv) {
            int tmp_h = h + 7;
            int16_t mid[128 * 135], *mid_ptr = mid;

            int sh = 6 - intermediate_bits;
            int16x8 rnd_v = wasm_i16x8_splat((1 << (sh)) >> 1);

            src -= src_stride * 3;

            if (w > 8) {
                GET_FILTER_FH_V();
                do {
                    // Process 8 pixels at a time
                    for (int x = 0; x < w; x += 8) {
                        const int16x8 clipped_v = DAV1D_FILTER_8TAP_RND_VEC(src, x, fh_v, 1, sh, rnd_v);
                        write_i16x8(mid_ptr + x, clipped_v);
                    }

                    mid_ptr += 128;
                    src += src_stride;
                } while (--tmp_h);
            } else if (w == 8) {
                GET_FILTER_FH_V();
                do {
                    // Process 8 pixels at a time
                    const int16x8 clipped_v = DAV1D_FILTER_8TAP_RND_VEC(src, 0, fh_v, 1, sh, rnd_v);
                    write_i16x8(mid_ptr, clipped_v);

                    mid_ptr += 128;
                    src += src_stride;
                } while (--tmp_h);
            } else /* w == 4 */ {
                GET_FILTER_FH_V();
                do {
                    // Process 8 pixels at a time, 4 each from 2 lines
                    const int16x8 clipped_v = DAV1D_FILTER_8TAP_RND_VEC_4X2(src, 0, fh_v, 1, src_stride, sh, rnd_v);
                    write_i16x8_4x2(mid_ptr, 128, clipped_v);

                    mid_ptr += 128 * 2;
                    src += src_stride * 2;
                    --tmp_h;
                } while (--tmp_h > 1);

                // odd line
                // Read and process 4 pixels plus some stray leftovers
                const int16x8 mid_v = DAV1D_FILTER_8TAP_RND_VEC(src, 0, fh_v, 1, sh, rnd_v);
                write_i16x8_4x1(mid_ptr, mid_v);
            }

            sh = 6;
            rnd_v = wasm_i16x8_splat((1 << (sh)) >> 1);
            const int32x4 rnd32_v = wasm_i32x4_splat((1 << (sh)) >> 1);

            mid_ptr = mid + 128 * 3;
            do {
                GET_FILTER_FV_V32();
                for (int x = 0; x < w; x += 4) {
                    // Process 4 pixels from one row
                    int32x4 t = DAV1D_FILTER_8TAP_RND_VEC_16(mid_ptr, x, fv_v, 128, sh, rnd32_v) /*-
                                wasm_i16x8_splat(PREP_BIAS) */;
                    write_i16x8_4x1(tmp + x, halfmerge_pixels32(t));
                }

                mid_ptr += 128;
                tmp += w;
            } while (--h);
        } else {
            int sh = 6 - intermediate_bits;
            int16x8 rnd_v = wasm_i16x8_splat((1 << (sh)) >> 1);

            if (w > 8) {
                GET_FILTER_FH_V();
                do {
                    for (int x = 0; x < w; x += 8) {
                        // Process 8 pixels from one row
                        const int16x8 tmp_v = DAV1D_FILTER_8TAP_RND_VEC(src, x, fh_v, 1, sh, rnd_v) /* -
                                              wasm_i16x8_splat(PREP_BIAS) */;
                        write_i16x8(tmp + x, tmp_v);
                    }

                    tmp += w;
                    src += src_stride;
                } while (--h);
            } else if (w == 8) {
                GET_FILTER_FH_V();
                do {
                    // Process 8 pixels from one row
                    const int16x8 tmp_v = DAV1D_FILTER_8TAP_RND_VEC(src, 0, fh_v, 1, sh, rnd_v) /* -
                                            wasm_i16x8_splat(PREP_BIAS) */;
                    write_i16x8(tmp, tmp_v);

                    tmp += w;
                    src += src_stride;
                } while (--h);
            } else /* w == 4 */ {
                GET_FILTER_FH_V();
                do {
                    // Process 8 pixels from two rows
                    // Assumes even height.
                    const int16x8 tmp_v = DAV1D_FILTER_8TAP_RND_VEC_4X2(src, 0, fh_v, 1, src_stride, sh, rnd_v) /* -
                                            wasm_i16x8_splat(PREP_BIAS) */;
                    write_i16x8(tmp, tmp_v);

                    tmp += w * 2;
                    src += src_stride * 2;
                    --h;
                } while (--h);
            }
        }
    } else if (fv) {
        int sh = 6 - intermediate_bits;
        int16x8 rnd_v = wasm_i16x8_splat((1 << (sh)) >> 1);
        if (w > 8) {
            GET_FILTER_FV_V();
            do {
                for (int x = 0; x < w; x += 8) {
                    const int16x8 tmp_v = DAV1D_FILTER_8TAP_RND_VEC(src, x, fv_v, src_stride, sh, rnd_v) /* -
                                          wasm_i16x8_splat(PREP_BIAS) */;
                    write_i16x8(tmp + x, tmp_v);
                }

                tmp += w;
                src += src_stride;
            } while (--h);
        } else if (w == 8) {
            GET_FILTER_FV_V();
            do {
                const int16x8 tmp_v = DAV1D_FILTER_8TAP_RND_VEC(src, 0, fv_v, src_stride, sh, rnd_v) /* -
                                        wasm_i16x8_splat(PREP_BIAS) */;
                write_i16x8(tmp, tmp_v);

                tmp += w;
                src += src_stride;
            } while (--h);
        } else /* w == 4 */ {
            GET_FILTER_FV_V();
            do {
                // Process 8 pixels from 2 rows, which are adjacent in the tmp array
                const int16x8 tmp_v = DAV1D_FILTER_8TAP_RND_VEC_4X2(src, 0, fv_v, src_stride, src_stride, sh, rnd_v) /* -
                                      wasm_i16x8_splat(PREP_BIAS) */;
                write_i16x8(tmp, tmp_v);

                tmp += w * 2;
                src += src_stride * 2;
                --h;
            } while (--h);
        }
    } else
        prep_wasm(tmp, src, src_stride, w, h HIGHBD_TAIL_SUFFIX);
}

#define filter_fns(type, type_h, type_v) \
void dav1d_put_8tap_##type##_wasm(pixel *const dst, \
                                   const ptrdiff_t dst_stride, \
                                   const pixel *const src, \
                                   const ptrdiff_t src_stride, \
                                   const int w, const int h, \
                                   const int mx, const int my \
                                   HIGHBD_DECL_SUFFIX); \
void dav1d_put_8tap_##type##_wasm(pixel *const dst, \
                                   const ptrdiff_t dst_stride, \
                                   const pixel *const src, \
                                   const ptrdiff_t src_stride, \
                                   const int w, const int h, \
                                   const int mx, const int my \
                                   HIGHBD_DECL_SUFFIX) \
{ \
    put_8tap_wasm(dst, dst_stride, src, src_stride, w, h, mx, my, \
                  type_h | (type_v << 2) HIGHBD_TAIL_SUFFIX); \
} \
void dav1d_prep_8tap_##type##_wasm(int16_t *const tmp, \
                                    const pixel *const src, \
                                    const ptrdiff_t src_stride, \
                                    const int w, const int h, \
                                    const int mx, const int my \
                                    HIGHBD_DECL_SUFFIX); \
void dav1d_prep_8tap_##type##_wasm(int16_t *const tmp, \
                                    const pixel *const src, \
                                    const ptrdiff_t src_stride, \
                                    const int w, const int h, \
                                    const int mx, const int my \
                                    HIGHBD_DECL_SUFFIX) \
{ \
    prep_8tap_wasm(tmp, src, src_stride, w, h, mx, my, \
                   type_h | (type_v << 2) HIGHBD_TAIL_SUFFIX); \
}

filter_fns(regular,        DAV1D_FILTER_8TAP_REGULAR, DAV1D_FILTER_8TAP_REGULAR)
filter_fns(regular_sharp,  DAV1D_FILTER_8TAP_REGULAR, DAV1D_FILTER_8TAP_SHARP)
filter_fns(regular_smooth, DAV1D_FILTER_8TAP_REGULAR, DAV1D_FILTER_8TAP_SMOOTH)
filter_fns(smooth,         DAV1D_FILTER_8TAP_SMOOTH,  DAV1D_FILTER_8TAP_SMOOTH)
filter_fns(smooth_regular, DAV1D_FILTER_8TAP_SMOOTH,  DAV1D_FILTER_8TAP_REGULAR)
filter_fns(smooth_sharp,   DAV1D_FILTER_8TAP_SMOOTH,  DAV1D_FILTER_8TAP_SHARP)
filter_fns(sharp,          DAV1D_FILTER_8TAP_SHARP,   DAV1D_FILTER_8TAP_SHARP)
filter_fns(sharp_regular,  DAV1D_FILTER_8TAP_SHARP,   DAV1D_FILTER_8TAP_REGULAR)
filter_fns(sharp_smooth,   DAV1D_FILTER_8TAP_SHARP,   DAV1D_FILTER_8TAP_SMOOTH)

