/*
 * HEVC video Decoder
 *
 * Copyright (C) 2012 Guillaume Martres
 *
 * This file is part of Libav.
 *
 * Libav is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * Libav is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with Libav; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include "libavutil/avassert.h"
#include "libavutil/pixdesc.h"
#include "bit_depth_template.c"
#include "hevcdata.h"
#include "hevcdsp.h"

#define SET(dst, x) (dst) = (x)
#define SCALE(dst, x) (dst) = av_clip_int16_c(((x) + add) >> shift)
#define ADD_AND_SCALE(dst, x) (dst) = av_clip_uintp2((dst) + av_clip_int16_c(((x) + add) >> shift), bit_depth)

static void FUNC(transquant_bypass)(uint8_t *_dst, int16_t *coeffs, ptrdiff_t _stride, int log2_size, int bit_depth)
{
    int x, y;
    pixel *dst = (pixel*)_dst;
    ptrdiff_t stride = _stride / sizeof(pixel);
    int size = 1 << log2_size;

    for (y = 0; y < size; y++) {
        for (x = 0; x < size; x++) {
            dst[x] += *coeffs;
            coeffs++;
        }
        dst += stride;
    }
}

static void FUNC(transform_skip)(uint8_t *_dst, int16_t *coeffs, ptrdiff_t _stride, int log2_size, int bit_depth)
{
    int x, y;
    pixel *dst = (pixel*)_dst;
    ptrdiff_t stride = _stride / sizeof(pixel);
    int size = 1 << log2_size;
#if REFERENCE_ENCODER_QUIRKS
    int shift = 15 - bit_depth - log2_size;
    if (shift > 0) {
        int offset = 1 << (shift - 1);
        for (y = 0; y < size; y++) {
            for (x = 0; x < size; x++)
                dst[x] += (coeffs[y * size + x] + offset) >> shift;
            dst += stride;
        }
    } else {
        for (y = 0; y < size; y++) {
            for (x = 0; x < size; x++)
                dst[x] += coeffs[y * size + x] << (-shift);
            dst += stride;
        }
    }
#else
    for (y = 0; y < size; y++) {
        for (x = 0; x < size; x++)
            dst[x] += coeffs[y * size + x] << 7;
        dst += stride;
    }
#endif
}

static void FUNC(transform_4x4_luma_add)(uint8_t *_dst, int16_t *coeffs, ptrdiff_t _stride, int bit_depth)
{
#define TR_4x4_LUMA(dst, src, step, assign)                                     \
    do {                                                                        \
        int c0 = src[0*step] + src[2*step];                                     \
        int c1 = src[2*step] + src[3*step];                                     \
        int c2 = src[0*step] - src[3*step];                                     \
        int c3 = 74 * src[1*step];                                              \
                                                                                \
        assign(dst[2*step], 74 * (src[0*step] - src[2*step] + src[3*step]));    \
        assign(dst[0*step], 29 * c0 + 55 * c1 + c3);                            \
        assign(dst[1*step], 55 * c2 - 29 * c1 + c3);                            \
        assign(dst[3*step], 55 * c0 + 29 * c2 - c3);                            \
    } while (0)

    int i;
    pixel *dst = (pixel*)_dst;
    ptrdiff_t stride = _stride / sizeof(pixel);
    int shift = 7;
    int add = 1 << (shift - 1);
    int16_t *src = coeffs;

    for (i = 0; i < 4; i++) {
        TR_4x4_LUMA(src, src, 4, SCALE);
        src++;
    }

    shift = 20 - bit_depth;
    add = 1 << (shift - 1);
    for (i = 0; i < 4; i++) {
        TR_4x4_LUMA(dst, coeffs, 1, ADD_AND_SCALE);
        coeffs += 4;
        dst += stride;
    }

#undef TR_4x4_LUMA
}

#define TR_4(dst, src, dstep, sstep, assign)                                    \
    do {                                                                        \
        const int e0 = transform[8*0][0] * src[0*sstep] +                       \
                       transform[8*2][0] * src[2*sstep];                        \
        const int e1 = transform[8*0][1] * src[0*sstep] +                       \
                       transform[8*2][1] * src[2*sstep];                        \
        const int o0 = transform[8*1][0] * src[1*sstep] +                       \
                       transform[8*3][0] * src[3*sstep];                        \
        const int o1 = transform[8*1][1] * src[1*sstep] +                       \
                       transform[8*3][1] * src[3*sstep];                        \
                                                                                \
        assign(dst[0*dstep], e0 + o0);                                          \
        assign(dst[1*dstep], e1 + o1);                                          \
        assign(dst[2*dstep], e1 - o1);                                          \
        assign(dst[3*dstep], e0 - o0);                                          \
    } while (0)
#define TR_4_1(dst, src) TR_4(dst, src, 4, 4, SCALE)
#define TR_4_2(dst, src) TR_4(dst, src, 1, 1, ADD_AND_SCALE)

static void FUNC(transform_4x4_add)(uint8_t *_dst, int16_t *coeffs, ptrdiff_t _stride, int bit_depth)
{
    int i;
    pixel *dst = (pixel*)_dst;
    ptrdiff_t stride = _stride / sizeof(pixel);
    int shift = 7;
    int add = 1 << (shift - 1);
    int16_t *src = coeffs;

    for (i = 0; i < 4; i++) {
        TR_4_1(src, src);
        src++;
    }

    shift = 20 - bit_depth;
    add = 1 << (shift - 1);
    for (i = 0; i < 4; i++) {
        TR_4_2(dst, coeffs);
        coeffs += 4;
        dst += stride;
    }
}

#define TR_8(dst, src, dstep, sstep, assign)                \
    do {                                                    \
        int i, j;                                           \
        int e_8[4];                                         \
        int o_8[4] = { 0 };                                 \
        for (i = 0; i < 4; i++)                             \
            for (j = 1; j < 8; j += 2)                      \
                o_8[i] += transform[4*j][i] * src[j*sstep]; \
        TR_4(e_8, src, 1, 2*sstep, SET);                    \
                                                            \
        for (i = 0; i < 4; i++) {                           \
            assign(dst[i*dstep], e_8[i] + o_8[i]);          \
            assign(dst[(7-i)*dstep], e_8[i] - o_8[i]);      \
        }                                                   \
    } while (0)

#define TR_16(dst, src, dstep, sstep, assign)                   \
    do {                                                        \
        int i, j;                                               \
        int e_16[8];                                            \
        int o_16[8] = { 0 };                                    \
        for (i = 0; i < 8; i++)                                 \
            for (j = 1; j < 16; j += 2)                         \
                o_16[i] += transform[2*j][i] * src[j*sstep];    \
        TR_8(e_16, src, 1, 2*sstep, SET);                       \
                                                                \
        for (i = 0; i < 8; i++) {                               \
            assign(dst[i*dstep], e_16[i] + o_16[i]);            \
            assign(dst[(15-i)*dstep], e_16[i] - o_16[i]);       \
        }                                                       \
    } while (0)

#define TR_32(dst, src, dstep, sstep, assign)               \
    do {                                                    \
        int i, j;                                           \
        int e_32[16];                                       \
        int o_32[16] = { 0 };                               \
        for (i = 0; i < 16; i++)                            \
            for (j = 1; j < 32; j += 2)                     \
                o_32[i] += transform[j][i] * src[j*sstep];  \
        TR_16(e_32, src, 1, 2*sstep, SET);                  \
                                                            \
        for (i = 0; i < 16; i++) {                          \
            assign(dst[i*dstep], e_32[i] + o_32[i]);        \
            assign(dst[(31-i)*dstep], e_32[i] - o_32[i]);   \
        }                                                   \
    } while (0)

#define TR_8_1(dst, src) TR_8(dst, src, 8, 8, SCALE)
#define TR_16_1(dst, src) TR_16(dst, src, 16, 16, SCALE)
#define TR_32_1(dst, src) TR_32(dst, src, 32, 32, SCALE)

#define TR_8_2(dst, src) TR_8(dst, src, 1, 1, ADD_AND_SCALE)
#define TR_16_2(dst, src) TR_16(dst, src, 1, 1, ADD_AND_SCALE)
#define TR_32_2(dst, src) TR_32(dst, src, 1, 1, ADD_AND_SCALE)

static void FUNC(transform_8x8_add)(uint8_t *_dst, int16_t *coeffs, ptrdiff_t _stride, int bit_depth)
{
    int i;
    pixel *dst = (pixel*)_dst;
    ptrdiff_t stride = _stride / sizeof(pixel);
    int shift = 7;
    int add = 1 << (shift - 1);
    int16_t *src = coeffs;

    for (i = 0; i < 8; i++) {
        TR_8_1(src, src);
        src++;
    }

    shift = 20 - bit_depth;
    add = 1 << (shift - 1);
    for (i = 0; i < 8; i++) {
        TR_8_2(dst, coeffs);
        coeffs += 8;
        dst += stride;
    }
}

static void FUNC(transform_16x16_add)(uint8_t *_dst, int16_t *coeffs, ptrdiff_t _stride, int bit_depth)
{
    int i;
    pixel *dst = (pixel*)_dst;
    ptrdiff_t stride = _stride / sizeof(pixel);
    int shift = 7;
    int add = 1 << (shift - 1);
    int16_t *src = coeffs;

    for (i = 0; i < 16; i++) {
        TR_16_1(src, src);
        src++;
    }

    shift = 20 - bit_depth;
    add = 1 << (shift - 1);
    for (i = 0; i < 16; i++) {
        TR_16_2(dst, coeffs);
        coeffs += 16;
        dst += stride;
    }
}

static void FUNC(transform_32x32_add)(uint8_t *_dst, int16_t *coeffs, ptrdiff_t _stride, int bit_depth)
{
    int i;
    pixel *dst = (pixel*)_dst;
    ptrdiff_t stride = _stride / sizeof(pixel);
    int shift = 7;
    int add = 1 << (shift - 1);
    int16_t *src = coeffs;

    for (i = 0; i < 32; i++) {
        TR_32_1(src, src);
        src++;
    }

    shift = 20 - bit_depth;
    add = 1 << (shift - 1);
    for (i = 0; i < 32; i++) {
        TR_32_2(dst, coeffs);
        coeffs += 32;
        dst += stride;
    }
}

static void FUNC(sao_band_filter)(uint8_t * _dst, uint8_t *_src, int _stride, int *sao_offset_val,
                                  int sao_left_class, int width, int height, int bit_depth)
{
    pixel *dst = (pixel*)_dst;
    pixel *src = (pixel*)_src;
    int stride = _stride/sizeof(pixel);
    int band_table[32] = { 0 };
    int shift = bit_depth - 5;

    for (int k = 0; k < 4; k++)
        band_table[(k + sao_left_class) & 31] = k + 1;
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++)
            dst[x] = av_clip_pixel(src[x] + sao_offset_val[band_table[src[x] >> shift]]);
        dst += stride;
        src += stride;
    }
}

static void FUNC(sao_edge_filter)(uint8_t *_dst, uint8_t *_src, int _stride, int *sao_offset_val,
                                  int sao_eo_class, int at_top_border, int at_bottom_border,
                                  int at_left_border, int at_right_border,
                                  int width, int height, int bit_depth)
{
    int x, y;
    pixel *dst = (pixel*)_dst;
    pixel *src = (pixel*)_src;
    int stride = _stride/sizeof(pixel);

    const int8_t pos[4][2][2] = {
        { { -1,  0 }, {  1, 0 } }, // horizontal
        { {  0, -1 }, {  0, 1 } }, // vertical
        { { -1, -1 }, {  1, 1 } }, // 45 degree
        { {  1, -1 }, { -1, 1 } }, // 135 degree
    };

    int init_x = 0, init_y = 0;
    int border_edge_idx = 0;

#define DST(x, y) dst[(x) + stride * (y)]
#define SRC(x, y) src[(x) + stride * (y)]

#define FILTER(x, y, edge_idx)                                      \
    DST(x, y) = av_clip_pixel(SRC(x, y) + sao_offset_val[edge_idx])

#define EDGE_IDX(a) ((a) < 3) ? (((a) + 1) % 3) : (a)
#define DIFF(x, y, k) SIGN(SRC(x, y) - SRC((x) + pos[sao_eo_class][(k)][0],     \
                                           (y) + pos[sao_eo_class][(k)][1]))
#define SIGN(a) ((a) > 0 ? 1 : ((a) == 0 ? 0 : -1))

    if (sao_eo_class != SAO_EO_VERT) {
        if (at_left_border) {
            for (y = 0; y < height; y++) {
                FILTER(0, y, border_edge_idx);
            }
            init_x = 1;
        }
        if (at_right_border) {
            for (x = 0; x < height; x++)
                FILTER(width - 1, x, border_edge_idx);
            width--;
        }
    }
    if (sao_eo_class != SAO_EO_HORIZ) {
        if (at_top_border) {
            for (x = init_x; x < width; x++)
                FILTER(x, 0, border_edge_idx);
            init_y = 1;
        }
        if (at_bottom_border) {
            for (x = init_x; x < width; x++)
                FILTER(x, height - 1, border_edge_idx);
            height--;
        }
    }

    for (y = init_y; y < height; y++) {
        for (x = init_x; x < width; x++) {
            int edge_idx = EDGE_IDX(2 + DIFF(x, y, 0) + DIFF(x, y, 1));
            FILTER(x, y, edge_idx);
        }
    }
#undef DST
#undef SRC
#undef FILTER
#undef EDGE_IDX
#undef DIFF
#undef SIGN
}

#undef SET
#undef SCALE
#undef ADD_AND_SCALE
#undef TR_4
#undef TR_4_1
#undef TR_4_2
#undef TR_8
#undef TR_8_1
#undef TR_8_2
#undef TR_16
#undef TR_16_1
#undef TR_16_2
#undef TR_32
#undef TR_32_1
#undef TR_32_2
