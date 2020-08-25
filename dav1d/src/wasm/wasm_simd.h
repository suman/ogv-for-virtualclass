#include <wasm_simd128.h>
#include <stdlib.h>

typedef int8_t int8x16 __attribute((vector_size(16)));
typedef int16_t int16x8 __attribute((vector_size(16)));
typedef int32_t int32x4 __attribute((vector_size(16)));
typedef int64_t int64x2 __attribute((vector_size(16)));

typedef uint8_t uint8x16 __attribute((vector_size(16)));
typedef uint16_t uint16x8 __attribute((vector_size(16)));
typedef uint64_t uint64x2 __attribute((vector_size(16)));
typedef uint32_t uint32x4 __attribute((vector_size(16)));


static inline int16x8 expand_pixels(const uint8x16 pixels) {
    return (int16x8)wasm_i16x8_widen_low_u8x16(pixels);
}

static inline uint8x16 merge_pixels(const int16x8 work) {
    return (uint8x16)wasm_u8x16_narrow_i16x8(work, work);
}


static inline uint32x4 expand_pixels32(const uint16x8 pixels) {
    return (uint32x4)wasm_i32x4_widen_low_u16x8(pixels);
}

static inline int32x4 expand_pixels32_s(const int16x8 pixels) {
    return (int32x4)wasm_i32x4_widen_low_i16x8(pixels);
}

static inline int16x8 halfmerge_pixels32(const int32x4 work) {
    return (int16x8)wasm_i16x8_narrow_i32x4(work, work);
}

static inline uint8x16 merge_pixels32(const int32x4 work) {
    int16x8 squished = wasm_i16x8_narrow_i32x4(work, work);
    return (uint8x16)wasm_u8x16_narrow_i16x8(squished, squished);
}

static inline int16x8 clip_vec(const int16x8 val, const int16x8 min, const int16x8 max) {
   return wasm_i16x8_max(wasm_i16x8_min(val, max), min);
}

static inline int32x4 clip_vec32(const int32x4 val, const int32x4 min, const int32x4 max) {
    return wasm_i32x4_max(wasm_i32x4_min(val, max), min);
}




static inline uint8x16 read_u8x16(const uint8_t *ptr) {
    return *(uint8x16 *)ptr;
}

static inline uint8x16 read_i8x16(const int8_t *ptr) {
    return *(int8x16 *)ptr;
}

static inline uint8x16 read_u8x16_4x2(const uint8_t *ptr, ptrdiff_t stride) {
    int32x4 dwords = *(int32x4 *)ptr;
    dwords[1] = *(int32_t *)(ptr + stride);
    return dwords;
}

static inline uint8x16 read_u8x16_2x3(const uint8_t *ptr, ptrdiff_t stride) {
    int16x8 words = *(int16x8 *)ptr;
    words[1] = *(int32_t *)(ptr + stride);
    words[2] = *(int32_t *)(ptr + stride * 2);
    return words;
}

static inline uint8x16 read_u8x16_2x4(const uint8_t *ptr, ptrdiff_t stride) {
    int16x8 words = *(int16x8 *)ptr;
    words[1] = *(int32_t *)(ptr + stride);
    words[2] = *(int32_t *)(ptr + stride * 2);
    words[3] = *(int32_t *)(ptr + stride * 3);
    return words;
}

static inline uint16x8 read_u16x8(const uint16_t *ptr) {
    return *(uint16x8 *)ptr;
}

static inline uint16x8 read_u16x8_4x2(const uint16_t *ptr, const ptrdiff_t stride) {
    /*
    // Slower
    uint64x2 qwords = *(const uint64x2 *)ptr;
    qwords[1] = *(const uint64_t *)(ptr + stride);
    return qwords;
    */

    /*
    // Slower
    uint64_t row1 = *(const uint64_t *)ptr;
    uint64_t row2 = *(const uint64_t *)(ptr + stride);
    uint64x2 qwords = {row1, row2};
    return (uint16x8)qwords;
    */

    /*
    // Slower but not as much
    // Note using 64-bit is tempting but is slower
    // because lanewise ops for 64-bit ints are awful?
    const uint32x4 qwords = {
        *(const uint32_t *)ptr,
        *(const uint32_t *)(ptr + 2),
        *(const uint32_t *)(ptr + stride),
        *(const uint32_t *)(ptr + stride + 2)
    };
    return (uint16x8)qwords;
    */

    // Faster but feels verbose
    const int16x8 row1_v = *(const int16x8 *)(ptr);
    const int16x8 row2_v = *(const int16x8 *)(ptr + stride);
    return (int16x8)__builtin_shufflevector(row1_v, row2_v,
        0,  1,  2,  3,
        8,  9, 10, 11
    );
}

static inline int16x8 read_i16x8(const int16_t *ptr) {
    return *(int16x8 *)ptr;
}

static inline int16x8 read_i16x8_2x2(const int16_t *ptr, const ptrdiff_t stride) {
    int32x4 dwords = *(int32x4 *)ptr;
    dwords[1] = *(int32_t *)(ptr + stride);
    return dwords;
}


static inline uint32x4 read_u32x4(const uint32_t *ptr) {
    return *(uint32x4 *)ptr;
}

static inline int32x4 read_i32x4(const int32_t *ptr) {
    return *(int32x4 *)ptr;
}

static inline uint64x2 read_u64x2(const uint64_t *ptr) {
    return *(uint64x2 *)ptr;
}

static inline int64x2 read_i64x2(const int64_t *ptr) {
    return *(int64x2 *)ptr;
}


static inline void write_u8x16(uint8_t *ptr, const uint8x16 data) {
    *(uint8x16 *)ptr = data;
}

static inline void write_u8x16_8x1(uint8_t *ptr, const uint8x16 data) {
    *(uint64_t *)ptr = ((uint64x2)data)[0];
}

static inline void write_u8x16_4x1(uint8_t *ptr, const uint8x16 data) {
    *(uint32_t *)ptr = ((uint32x4)data)[0];
}

static inline void write_u8x16_4x2(uint8_t *ptr, const ptrdiff_t stride, const uint8x16 data) {
    write_u8x16_4x1(ptr, data);
    *(uint32_t *)(ptr + stride * 1) = ((uint32x4)data)[1];
}

static inline void write_u8x16_2x2(uint8_t *ptr, const ptrdiff_t stride, const uint8x16 data) {
    *(uint16_t *)(ptr + stride * 0) = ((uint16x8)data)[0];
    *(uint16_t *)(ptr + stride * 1) = ((uint16x8)data)[1];
}

static inline void write_u8x16_2x3(uint8_t *ptr, const ptrdiff_t stride, const uint8x16 data) {
    write_u8x16_2x2(ptr, stride, data);
    *(uint16_t *)(ptr + stride * 2) = ((uint16x8)data)[2];
}

static inline void write_u8x16_2x4(uint8_t *ptr, const ptrdiff_t stride, const uint8x16 data) {
    write_u8x16_2x3(ptr, stride, data);
    *(uint16_t *)(ptr + stride * 3) = ((uint16x8)data)[3];
}


static inline void write_u16x8(uint16_t *ptr, const uint16x8 data) {
    *(uint16x8 *)ptr = data;
}

static inline void write_u32x4(uint32_t *ptr, const uint32x4 data) {
    *(uint32x4 *)ptr = data;
}



static inline void write_i8x16(int8_t *ptr, const int8x16 data) {
    *(int8x16 *)ptr = data;
}


static inline void write_i16x8(int16_t *ptr, const int16x8 data) {
    *(int16x8 *)ptr = data;
}

static inline void write_i16x8_4x1(int16_t *ptr, const int16x8 data) {
    *(uint64_t *)ptr = ((uint64x2)data)[0];
}

static inline void write_i16x8_4x2(int16_t *ptr, const ptrdiff_t stride, const int16x8 data) {
    *(uint64_t *)(ptr +      0) = ((uint64x2)data)[0];
    *(uint64_t *)(ptr + stride) = ((uint64x2)data)[1];
}

static inline void write_i16x8_2x3(int16_t *ptr, const ptrdiff_t stride, const int16x8 data) {
    *(uint32_t *)(ptr + stride * 0) = ((uint32x4)data)[0];
    *(uint32_t *)(ptr + stride * 1) = ((uint32x4)data)[1];
    *(uint32_t *)(ptr + stride * 2) = ((uint32x4)data)[2];
}

static inline void write_i16x8_2x4(int16_t *ptr, const ptrdiff_t stride, const int16x8 data) {
    *(uint32_t *)(ptr + stride * 0) = ((uint32x4)data)[0];
    *(uint32_t *)(ptr + stride * 1) = ((uint32x4)data)[1];
    *(uint32_t *)(ptr + stride * 2) = ((uint32x4)data)[2];
    *(uint32_t *)(ptr + stride * 3) = ((uint32x4)data)[3];
}


static inline void write_i32x4(int32_t *ptr, const int32x4 data) {
    *(int32x4 *)ptr = data;
}
