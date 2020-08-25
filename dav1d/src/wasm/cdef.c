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

#include "src/cpu.h"
#include "src/cdef.h"
#include "src/cdef.h"

#include "common/intops.h"

#include <assert.h>
#include <stdlib.h>

#include "wasm_simd.h"

static inline int16x8 constrain_vec(const int16x8 diff_v, const int16x8 threshold_v,
                                        const int shift)
{
    // this condition was moved up
    //if (!threshold) return wasm_i16x8_splat(0);

    const int16x8 abs_v = wasm_i16x8_abs(diff_v);
    const int16x8 min_v = wasm_i16x8_min(
        abs_v,
        wasm_i16x8_max(
            wasm_i16x8_splat(0),
            wasm_i16x8_sub(
                threshold_v,
                wasm_i16x8_shr(abs_v, shift)
            )
        )
    );

    return wasm_v128_bitselect(
        wasm_i16x8_neg(min_v),
        min_v,
        wasm_i16x8_shr(diff_v, 15) // sign bit
    );
}


#define FILL16 INT16_MAX
#define FILL32 0x7fff7fff
#define FILL64 0x7fff7fff7fff7fffLL

static int16x8 fill128 = {
    INT16_MAX, INT16_MAX, INT16_MAX, INT16_MAX,
    INT16_MAX, INT16_MAX, INT16_MAX, INT16_MAX,
};

static inline void fill(int16_t *tmp, const ptrdiff_t stride,
                        const int w, const int h)
{
    /* Use a value that's a large positive number when interpreted as unsigned,
     * and a large negative number when interpreted as signed. */
    for (int y = 0; y < h; y++) {
        if (w >= 8) {
            for (int x = 0; x < w; x += 8) {
                *(v128_t *)&tmp[x] = fill128;
            }
        } else if (w >= 4) {
            for (int x = 0; x < w; x += 4) {
                *(int64_t *)&tmp[x] = FILL64;
            }
        } else if (w >= 2) {
            for (int x = 0; x < w; x += 2) {
                *(int32_t *)&tmp[x] = FILL32;
            }
        } else {
            for (int x = 0; x < w; x++) {
                tmp[x] = INT16_MIN;
            }
        }
        tmp += stride;
    }
}

static inline void padding_copy(int16_t *tmp, const pixel *src, const int w)
{
    for (int x = 0; x < w; x++)
        tmp[x] = src[x];
}

static inline void padding_copy_2(int16_t *tmp, const pixel *src)
{
    tmp[0] = src[0];
    tmp[1] = src[1];
}

static inline void padding_copy_4(int16_t *tmp, const pixel *src)
{
    const v128_t px_v = expand_pixels(*(v128_t *)src);
    const int64_t px_64 = wasm_i64x2_extract_lane(px_v, 0);
    *(int64_t *)tmp = px_64;
}

static inline void padding_copy_8(int16_t *tmp, const pixel *src)
{
    const v128_t px_v = expand_pixels(*(v128_t *)src);
    *(v128_t *)tmp = px_v;
}

static inline void padding_copy_w(int16_t *tmp, const pixel *src, const int w, const enum CdefEdgeFlags edges)
{
    if (edges & CDEF_HAVE_LEFT) {
        padding_copy_2(tmp - 2, src - 2);
    }
    if (w == 8) {
        padding_copy_8(tmp, src);
    } else if (w == 4) {
        padding_copy_4(tmp, src);
    } else {
        assert(w == 8 || w == 4);
    }
    if (edges & CDEF_HAVE_RIGHT) {
        padding_copy_2(tmp + w, src + w);
    }
}

static void padding(int16_t *tmp, const ptrdiff_t tmp_stride,
                    const pixel *src, const ptrdiff_t src_stride,
                    const pixel (*left)[2], const pixel *top,
                    const int w, const int h,
                    const enum CdefEdgeFlags edges)
{
    // fill extended input buffer
    int x_start = -2, x_end = w + 2, y_start = -2, y_end = h + 2;
    if (!(edges & CDEF_HAVE_TOP)) {
        fill(tmp - 2 - 2 * tmp_stride, tmp_stride, w + 4, 2);
        y_start = 0;
    }
    if (!(edges & CDEF_HAVE_BOTTOM)) {
        fill(tmp + h * tmp_stride - 2, tmp_stride, w + 4, 2);
        y_end -= 2;
    }
    if (!(edges & CDEF_HAVE_LEFT)) {
        fill(tmp + y_start * tmp_stride - 2, tmp_stride, 2, y_end - y_start);
        x_start = 0;
    }
    if (!(edges & CDEF_HAVE_RIGHT)) {
        fill(tmp + y_start * tmp_stride + w, tmp_stride, 2, y_end - y_start);
        x_end -= 2;
    }

    for (int y = y_start; y < 0; y++) {
        padding_copy_w(&tmp[y * tmp_stride], top, w, edges);
        top += PXSTRIDE(src_stride);
    }
    if (edges & CDEF_HAVE_LEFT) {
        for (int y = 0; y < h; y++)
            padding_copy_2(&tmp[x_start + y * tmp_stride], left[y]);
    }
    for (int y = 0; y < h; y++) {
        // Skip the left edge, which is taken from 'left' above.
        padding_copy_w(tmp, src, w, edges & ~CDEF_HAVE_LEFT);
        src += PXSTRIDE(src_stride);
        tmp += tmp_stride;
    }
    for (int y = h; y < y_end; y++) {
        // Except at the final rows where we do copy them?
        padding_copy_w(tmp, src, w, edges);
        src += PXSTRIDE(src_stride);
        tmp += tmp_stride;
    }
}

#undef FILL16
#undef FILL32

static const int8_t cdef_directions[8 /* dir */][2 /* pass */] = {
    { -1 * 16 + 1, -2 * 16 + 2 },
    {  0 * 16 + 1, -1 * 16 + 2 },
    {  0 * 16 + 1,  0 * 16 + 2 },
    {  0 * 16 + 1,  1 * 16 + 2 },
    {  1 * 16 + 1,  2 * 16 + 2 },
    {  1 * 16 + 0,  2 * 16 + 1 },
    {  1 * 16 + 0,  2 * 16 + 0 },
    {  1 * 16 + 0,  2 * 16 - 1 }
};

static inline void
__attribute((always_inline))
cdef_filter_block_wasm_8(uint8_t *dst, const ptrdiff_t dst_stride,
                    const uint8_t (*left)[2], /*const*/ uint8_t *const top[2],
                    const int w, const int h, const int pri_strength,
                    const int sec_strength, const int dir,
                    const int damping, const enum CdefEdgeFlags edges
                    HIGHBD_DECL_SUFFIX)
{
    const ptrdiff_t tmp_stride = 16;
    assert((w == 4 || w == 8) && (h == 4 || h == 8));
    __attribute__((aligned(16))) int16_t tmp_buf[200];  // 16*12 is the maximum value of tmp_stride * (h + 4)
    int16_t *tmp = tmp_buf + 2 * tmp_stride + 2 + 6; // add 2 for edge pixels, 6 to stay aligned
    const int pri_tap = 4 - (pri_strength & 1);
    const int pri_shift = imax(0, damping - ulog2(pri_strength));
    const int sec_shift = imax(0, damping - ulog2(sec_strength));
    const int16x8 int16max_v = wasm_i16x8_splat(INT16_MAX);

    padding(tmp, tmp_stride, dst, dst_stride, left, top, w, h, edges);

    // run actual filter
    for (int y = 0; y < h; y++) {
        int16x8 sum_v = wasm_i16x8_splat(0);

        // Read and process one 8-pixel row at a time in 128 bits

        // Expand 8 8-bit pixels to 16 bits
        const int16x8 px_v = expand_pixels(read_u8x16(dst));

        int16x8 max_v = px_v;
        int16x8 min_v = px_v;
        int pri_tap_k = pri_tap;
        for (int k = 0; k < 2; k++) {
            const int off1 = cdef_directions[dir][k];
            const int16x8 p0_v = *(const int16x8 *)(tmp + off1);
            const int16x8 p1_v = *(const int16x8 *)(tmp - off1);
            if (pri_strength) {
                const int16x8 pri_strength_v = wasm_i16x8_splat(pri_strength);
                const int16x8 tap1_v = wasm_i16x8_add(
                    constrain_vec(
                        wasm_i16x8_sub(p0_v, px_v),
                        pri_strength_v,
                        pri_shift
                    ),
                    constrain_vec(
                        wasm_i16x8_sub(p1_v, px_v),
                        pri_strength_v,
                        pri_shift
                    )
                );
                sum_v = wasm_i16x8_add(
                    sum_v,
                    wasm_i16x8_mul(wasm_i16x8_splat(pri_tap_k), tap1_v)
                );
            }
            // if pri_tap_k == 4 then it becomes 2 else it remains 3
            pri_tap_k -= (pri_tap_k << 1) - 6;

            max_v = wasm_v128_bitselect(wasm_i16x8_max(p0_v, max_v), max_v, wasm_i16x8_ne(p0_v, int16max_v));
            max_v = wasm_v128_bitselect(wasm_i16x8_max(p1_v, max_v), max_v, wasm_i16x8_ne(p1_v, int16max_v));
            min_v = wasm_i16x8_min(p0_v, min_v);
            min_v = wasm_i16x8_min(p1_v, min_v);
            const int off2 = cdef_directions[(dir + 2) & 7][k];
            const int16x8 s0_v = *(const int16x8 *)(tmp + off2);
            const int16x8 s1_v = *(const int16x8 *)(tmp - off2);
            const int off3 = cdef_directions[(dir + 6) & 7][k];
            const int16x8 s2_v = *(const int16x8 *)(tmp + off3);
            const int16x8 s3_v = *(const int16x8 *)(tmp - off3);
            max_v = wasm_v128_bitselect(wasm_i16x8_max(s0_v, max_v), max_v, wasm_i16x8_ne(s0_v, int16max_v));
            max_v = wasm_v128_bitselect(wasm_i16x8_max(s1_v, max_v), max_v, wasm_i16x8_ne(s1_v, int16max_v));
            max_v = wasm_v128_bitselect(wasm_i16x8_max(s2_v, max_v), max_v, wasm_i16x8_ne(s2_v, int16max_v));
            max_v = wasm_v128_bitselect(wasm_i16x8_max(s3_v, max_v), max_v, wasm_i16x8_ne(s3_v, int16max_v));
            min_v = wasm_i16x8_min(s0_v, min_v);
            min_v = wasm_i16x8_min(s1_v, min_v);
            min_v = wasm_i16x8_min(s2_v, min_v);
            min_v = wasm_i16x8_min(s3_v, min_v);
            if (sec_strength) {
                const int16x8 sec_strength_v = wasm_i16x8_splat(sec_strength);
                const int16x8 tap2_v = wasm_i16x8_add(
                    wasm_i16x8_add(
                        constrain_vec(wasm_i16x8_sub(s0_v, px_v), sec_strength_v, sec_shift),
                        constrain_vec(wasm_i16x8_sub(s1_v, px_v), sec_strength_v, sec_shift)
                    ),
                    wasm_i16x8_add(
                        constrain_vec(wasm_i16x8_sub(s2_v, px_v), sec_strength_v, sec_shift),
                        constrain_vec(wasm_i16x8_sub(s3_v, px_v), sec_strength_v, sec_shift)
                    )
                );
                sum_v = wasm_i16x8_add(sum_v, tap2_v);
                if (k == 0) {
                    sum_v = wasm_i16x8_add(sum_v, tap2_v);
                }
            }
        }

        const int16x8 dst_v = clip_vec(
            wasm_i16x8_add(
                px_v,
                wasm_i16x8_shr(
                    wasm_i16x8_add(
                        wasm_i16x8_splat(8),
                        wasm_i16x8_sub(
                            sum_v,
                            // take the sign bit
                            wasm_u16x8_shr(sum_v, 15)
                        )
                    ),
                    4
                )
            ),
            min_v,
            max_v
        );
        write_u8x16_8x1(dst, merge_pixels(dst_v));

        dst += dst_stride;
        tmp += tmp_stride;
    }
}

static inline void
__attribute((always_inline))
cdef_filter_block_wasm_4(uint8_t *dst, const ptrdiff_t dst_stride,
                    const uint8_t (*left)[2], /*const*/ uint8_t *const top[2],
                    const int w, const int h, const int pri_strength,
                    const int sec_strength, const int dir,
                    const int damping, const enum CdefEdgeFlags edges
                    HIGHBD_DECL_SUFFIX)
{
    const ptrdiff_t tmp_stride = 16;
    assert((w == 4 || w == 8) && (h == 4 || h == 8));
    __attribute__((aligned(16))) int16_t tmp_buf[200];  // 16*12 is the maximum value of tmp_stride * (h + 4)
    int16_t *tmp = tmp_buf + 2 * tmp_stride + 2 + 6;
    const int bitdepth_min_8 = 8 - 8;
    const int pri_tap = 4 - ((pri_strength >> bitdepth_min_8) & 1);
    const int pri_shift = imax(0, damping - ulog2(pri_strength));
    const int sec_shift = imax(0, damping - ulog2(sec_strength));
    const int16x8 int16max_v = wasm_i16x8_splat(INT16_MAX);

    padding(tmp, tmp_stride, dst, dst_stride, left, top, w, h, edges);

    // run actual filter
    for (int y = 0; y < h; y += 2) {
        int16x8 sum_v = wasm_i16x8_splat(0);

        // Read and process two 4-pixel rows at a time in 128 bits
        const int16x8 px_v = expand_pixels(read_u8x16_4x2(dst, dst_stride));

        int16x8 max_v = px_v;
        int16x8 min_v = px_v;
        int pri_tap_k = pri_tap;
        for (int k = 0; k < 2; k++) {
            const int off1 = cdef_directions[dir][k];
            const int16x8 p0_v = read_u16x8_4x2(tmp + off1, tmp_stride);
            const int16x8 p1_v = read_u16x8_4x2(tmp - off1, tmp_stride);
            if (pri_strength) {
                const int16x8 pri_strength_v = wasm_i16x8_splat(pri_strength);
                const int16x8 tap1_v = wasm_i16x8_add(
                    constrain_vec(
                        wasm_i16x8_sub(p0_v, px_v),
                        pri_strength_v,
                        pri_shift
                    ),
                    constrain_vec(
                        wasm_i16x8_sub(p1_v, px_v),
                        pri_strength_v,
                        pri_shift
                    )
                );
                sum_v = wasm_i16x8_add(
                    sum_v,
                    wasm_i16x8_mul(wasm_i16x8_splat(pri_tap_k), tap1_v)
                );
            }
            // if pri_tap_k == 4 then it becomes 2 else it remains 3
            pri_tap_k -= (pri_tap_k << 1) - 6;
            max_v = wasm_v128_bitselect(wasm_i16x8_max(p0_v, max_v), max_v, wasm_i16x8_ne(p0_v, int16max_v));
            max_v = wasm_v128_bitselect(wasm_i16x8_max(p1_v, max_v), max_v, wasm_i16x8_ne(p1_v, int16max_v));
            min_v = wasm_i16x8_min(p0_v, min_v);
            min_v = wasm_i16x8_min(p1_v, min_v);
            const int off2 = cdef_directions[(dir + 2) & 7][k];
            const int16x8 s0_v = read_u16x8_4x2(tmp + off2, tmp_stride);
            const int16x8 s1_v = read_u16x8_4x2(tmp - off2, tmp_stride);
            const int off3 = cdef_directions[(dir + 6) & 7][k];
            const int16x8 s2_v = read_u16x8_4x2(tmp + off3, tmp_stride);
            const int16x8 s3_v = read_u16x8_4x2(tmp - off3, tmp_stride);
            max_v = wasm_v128_bitselect(wasm_i16x8_max(s0_v, max_v), max_v, wasm_i16x8_ne(s0_v, int16max_v));
            max_v = wasm_v128_bitselect(wasm_i16x8_max(s1_v, max_v), max_v, wasm_i16x8_ne(s1_v, int16max_v));
            max_v = wasm_v128_bitselect(wasm_i16x8_max(s2_v, max_v), max_v, wasm_i16x8_ne(s2_v, int16max_v));
            max_v = wasm_v128_bitselect(wasm_i16x8_max(s3_v, max_v), max_v, wasm_i16x8_ne(s3_v, int16max_v));
            min_v = wasm_i16x8_min(s0_v, min_v);
            min_v = wasm_i16x8_min(s1_v, min_v);
            min_v = wasm_i16x8_min(s2_v, min_v);
            min_v = wasm_i16x8_min(s3_v, min_v);
            if (sec_strength) {
                const int16x8 sec_strength_v = wasm_i16x8_splat(sec_strength);
                const int16x8 tap2_v = wasm_i16x8_add(
                    wasm_i16x8_add(
                        constrain_vec(wasm_i16x8_sub(s0_v, px_v), sec_strength_v, sec_shift),
                        constrain_vec(wasm_i16x8_sub(s1_v, px_v), sec_strength_v, sec_shift)
                    ),
                    wasm_i16x8_add(
                        constrain_vec(wasm_i16x8_sub(s2_v, px_v), sec_strength_v, sec_shift),
                        constrain_vec(wasm_i16x8_sub(s3_v, px_v), sec_strength_v, sec_shift)
                    )
                );
                sum_v = wasm_i16x8_add(sum_v, tap2_v);
                if (k == 0) {
                    sum_v = wasm_i16x8_add(sum_v, tap2_v);
                }
            }
        }

        const int16x8 dst_v = clip_vec(
            wasm_i16x8_add(
                px_v,
                wasm_i16x8_shr(
                    wasm_i16x8_add(
                        wasm_i16x8_splat(8),
                        wasm_i16x8_sub(
                            sum_v,
                            // take the sign bit
                            wasm_u16x8_shr(sum_v, 15)
                        )
                    ),
                    4
                )
            ),
            min_v,
            max_v
        );
        write_u8x16_4x2(dst, dst_stride, merge_pixels(dst_v));

        dst += dst_stride * 2;
        tmp += tmp_stride * 2;
    }
}

decl_cdef_fn(dav1d_cdef_filter_8x8_wasm);
decl_cdef_fn(dav1d_cdef_filter_4x8_wasm);
decl_cdef_fn(dav1d_cdef_filter_4x4_wasm);

decl_cdef_fn(dav1d_cdef_filter_8x8_wasm)
{
    cdef_filter_block_wasm_8(dst, stride, left, top, 8, 8, pri_strength, sec_strength,
                        dir, damping, edges HIGHBD_TAIL_SUFFIX);
}

decl_cdef_fn(dav1d_cdef_filter_4x8_wasm)
{
    cdef_filter_block_wasm_4(dst, stride, left, top, 4, 8, pri_strength, sec_strength,
                        dir, damping, edges HIGHBD_TAIL_SUFFIX);
}

decl_cdef_fn(dav1d_cdef_filter_4x4_wasm)
{
    cdef_filter_block_wasm_4(dst, stride, left, top, 4, 4, pri_strength, sec_strength,
                        dir, damping, edges HIGHBD_TAIL_SUFFIX);
}
