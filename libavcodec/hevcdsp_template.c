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

static void FUNC(dequant)(int16_t *coeffs, int log2_size, int qp)
{
    int x, y;
    int size = 1 << log2_size;

    const uint8_t level_scale[] = { 40, 45, 51, 57, 64, 72 };

    //TODO: scaling_list_enabled_flag support

    int m = 16;
    int shift = BIT_DEPTH + log2_size - 5;
    int scale = level_scale[qp % 6] << (qp / 6);
    for (y = 0; y < size; y++)
        for (x = 0; x < size; x++)
            coeffs[size*y+x] = av_clip_int16_c(((coeffs[size*y+x] * m * scale) +
                                                (1 << (shift - 1))) >> shift);
}

static void FUNC(transquant_bypass)(uint8_t *_dst, int16_t *coeffs, ptrdiff_t _stride, int log2_size)
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

static void FUNC(transform_skip)(uint8_t *_dst, int16_t *coeffs, ptrdiff_t _stride)
{
    int x, y;
    pixel *dst = (pixel*)_dst;
    ptrdiff_t stride = _stride / sizeof(pixel);
    int size = 4;
    int shift = 13 - BIT_DEPTH;
    int offset = 1 << (shift - 1);
    for (y = 0; y < size; y++) {
        for (x = 0; x < size; x++)
#if BIT_DEPTH <= 13
            dst[x] = av_clip_pixel(dst[x] + ((coeffs[y * size + x] + offset) >> shift));
#else
            dst[x] = av_clip_pixel(dst[x] + (coeffs[y * size + x] << (-shift)));
#endif
        dst += stride;
    }
}

#define SET(dst, x) (dst) = (x)
#define SCALE(dst, x) (dst) = av_clip_int16_c(((x) + add) >> shift)
#define ADD_AND_SCALE(dst, x) (dst) = av_clip_pixel((dst) + av_clip_int16_c(((x) + add) >> shift))

static void FUNC(transform_4x4_luma_add)(uint8_t *_dst, int16_t *coeffs, ptrdiff_t _stride)
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

    shift = 20 - BIT_DEPTH;
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

static void FUNC(transform_4x4_add)(uint8_t *_dst, int16_t *coeffs, ptrdiff_t _stride)
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

    shift = 20 - BIT_DEPTH;
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

static void FUNC(transform_8x8_add)(uint8_t *_dst, int16_t *coeffs, ptrdiff_t _stride)
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

    shift = 20 - BIT_DEPTH;
    add = 1 << (shift - 1);
    for (i = 0; i < 8; i++) {
        TR_8_2(dst, coeffs);
        coeffs += 8;
        dst += stride;
    }
}

static void FUNC(transform_16x16_add)(uint8_t *_dst, int16_t *coeffs, ptrdiff_t _stride)
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

    shift = 20 - BIT_DEPTH;
    add = 1 << (shift - 1);
    for (i = 0; i < 16; i++) {
        TR_16_2(dst, coeffs);
        coeffs += 16;
        dst += stride;
    }
}

static void FUNC(transform_32x32_add)(uint8_t *_dst, int16_t *coeffs, ptrdiff_t _stride)
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

    shift = 20 - BIT_DEPTH;
    add = 1 << (shift - 1);
    for (i = 0; i < 32; i++) {
        TR_32_2(dst, coeffs);
        coeffs += 32;
        dst += stride;
    }
}

static void FUNC(sao_band_filter)(uint8_t * _dst, uint8_t *_src, int _stride, int *sao_offset_val,
                                  int sao_left_class, int width, int height)
{
    pixel *dst = (pixel*)_dst;
    pixel *src = (pixel*)_src;
    int stride = _stride/sizeof(pixel);
    int band_table[32] = { 0 };
    int k, y, x;
    int shift = BIT_DEPTH - 5;

    for (k = 0; k < 4; k++)
        band_table[(k + sao_left_class) & 31] = k + 1;
    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++)
            dst[x] = av_clip_pixel(src[x] + sao_offset_val[band_table[src[x] >> shift]]);
        dst += stride;
        src += stride;
    }
}

static void FUNC(sao_edge_filter)(uint8_t *_dst, uint8_t *_src, int _stride, int *sao_offset_val,
                                  int sao_eo_class, int at_top_border, int at_bottom_border,
                                  int at_left_border, int at_right_border,
                                  int width, int height)
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

static void FUNC(put_hevc_qpel_pixels)(uint8_t * _dst, ptrdiff_t _dststride,
                                       uint8_t *_src, ptrdiff_t _srcstride,
                                       int width, int height)
{
    int x, y;
    pixel *dst = (pixel*)_dst;
    pixel *src = (pixel*)_src;
    int dststride = _dststride/sizeof(pixel);
    int srcstride = _srcstride/sizeof(pixel);

    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++)
            dst[x] = src[x] << (14 - BIT_DEPTH);
        src += srcstride;
        dst += dststride;
    }
}

#define QPEL_FILTER_1(src, stride)                                             \
    (-src[x-3*stride] + 4*src[x-2*stride] - 10*src[x-stride] + 58*src[x] +     \
     17*src[x+stride] - 5*src[x+2*stride] + 1*src[x+3*stride])
#define QPEL_FILTER_2(src, stride)                                              \
    (-src[x-3*stride] + 4*src[x-2*stride] - 11*src[x-stride] + 40*src[x] +      \
     40*src[x+stride] - 11*src[x+2*stride] + 4*src[x+3*stride] + src[x+4*stride])
#define QPEL_FILTER_3(src, stride)                                             \
    (src[x-2*stride] - 5*src[x-stride] + 17*src[x] + 58*src[x+stride]          \
     - 10*src[x+2*stride] + 4*src[x+3*stride] - src[x+4*stride])

#define PUT_HEVC_QPEL_H(H)                                                      \
static void FUNC(put_hevc_qpel_h ## H)(uint8_t * _dst, ptrdiff_t _dststride,    \
                                          uint8_t *_src, ptrdiff_t _srcstride,  \
                                          int width, int height)                \
{                                                                               \
    int x, y;                                                                   \
    pixel *dst = (pixel*)_dst;                                                  \
    pixel *src = (pixel*)_src;                                                  \
    int dststride = _dststride/sizeof(pixel);                                   \
    int srcstride = _srcstride/sizeof(pixel);                                   \
                                                                                \
    for (y = 0; y < height; y++) {                                              \
        for (x = 0; x < width; x++)                                             \
            dst[x] = QPEL_FILTER_ ## H (src, 1) >> (BIT_DEPTH - 8);             \
        src += srcstride;                                                       \
        dst += dststride;                                                       \
    }                                                                           \
}

#define PUT_HEVC_QPEL_V(V)                                                      \
static void FUNC(put_hevc_qpel_v ## V)(uint8_t * _dst, ptrdiff_t _dststride,    \
                                          uint8_t *_src, ptrdiff_t _srcstride,  \
                                          int width, int height)                \
{                                                                               \
    int x, y;                                                                   \
    pixel *dst = (pixel*)_dst;                                                  \
    pixel *src = (pixel*)_src;                                                  \
    int dststride = _dststride/sizeof(pixel);                                   \
    int srcstride = _srcstride/sizeof(pixel);                                   \
                                                                                \
    for (y = 0; y < height; y++)  {                                             \
        for (x = 0; x < width; x++)                                             \
            dst[x] = QPEL_FILTER_ ## V (src, srcstride) >> (BIT_DEPTH - 8);     \
        src += srcstride;                                                       \
        dst += dststride;                                                       \
    }                                                                           \
}

#define PUT_HEVC_QPEL_HV(H, V)                                                            \
static void FUNC(put_hevc_qpel_h ## H ## v ## V )(uint8_t * _dst, ptrdiff_t _dststride,   \
                                                  uint8_t *_src, ptrdiff_t _srcstride,    \
                                                  int width, int height)                  \
{                                                                                         \
    int x, y;                                                                             \
    pixel *dst = (pixel*)_dst;                                                            \
    pixel *src = (pixel*)_src;                                                            \
    int dststride = _dststride/sizeof(pixel);                                             \
    int srcstride = _srcstride/sizeof(pixel);                                             \
                                                                                          \
    int tmpstride = MAX_PB_SIZE;                                                          \
    pixel tmp_array[(MAX_PB_SIZE+7)*MAX_PB_SIZE];                                         \
    pixel *tmp = tmp_array;                                                               \
                                                                                          \
    src -= qpel_extra_before[V-1] * srcstride;                                            \
                                                                                          \
    for (y = 0; y < height + qpel_extra[V-1]; y++) {                                      \
        for (x = 0; x < width; x++)                                                       \
            tmp[x] = QPEL_FILTER_ ## H (src, 1) >> (BIT_DEPTH - 8);                       \
              src += srcstride;                                                           \
              dst += dststride;                                                           \
    }                                                                                     \
                                                                                          \
    tmp = tmp_array + qpel_extra_before[V-1] * tmpstride;                                 \
                                                                                          \
    for (y = 0; y < height; y++) {                                                        \
        for (x = 0; x < width; x++)                                                       \
            dst[x] = QPEL_FILTER_ ## V (tmp, tmpstride) >> 6;                             \
              src += srcstride;                                                           \
              dst += dststride;                                                           \
    }                                                                                     \
}

PUT_HEVC_QPEL_H(1)
PUT_HEVC_QPEL_H(2)
PUT_HEVC_QPEL_H(3)
PUT_HEVC_QPEL_V(1)
PUT_HEVC_QPEL_V(2)
PUT_HEVC_QPEL_V(3)
PUT_HEVC_QPEL_HV(1, 1)
PUT_HEVC_QPEL_HV(1, 2)
PUT_HEVC_QPEL_HV(1, 3)
PUT_HEVC_QPEL_HV(2, 1)
PUT_HEVC_QPEL_HV(2, 2)
PUT_HEVC_QPEL_HV(2, 3)
PUT_HEVC_QPEL_HV(3, 1)
PUT_HEVC_QPEL_HV(3, 2)
PUT_HEVC_QPEL_HV(3, 3)

static void FUNC(put_hevc_epel_pixels)(uint8_t * _dst, ptrdiff_t _dststride,
                                       uint8_t *_src, ptrdiff_t _srcstride,
                                       int width, int height, int mx, int my)
{
    int x, y;
    pixel *dst = (pixel*)_dst;
    pixel *src = (pixel*)_src;
    int dststride = _dststride/sizeof(pixel);
    int srcstride = _srcstride/sizeof(pixel);

    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++)
            dst[x] = src[x] << (14 - BIT_DEPTH);
        src += srcstride;
        dst += dststride;
    }
}

#define EPEL_FILTER(src, stride, F) \
    (F[0]*src[x-stride] + F[1]*src[x] + F[2]*src[x+stride] + F[3]*src[x+2*stride])

static void FUNC(put_hevc_epel_h)(uint8_t * _dst, ptrdiff_t _dststride,
                                  uint8_t *_src, ptrdiff_t _srcstride,
                                  int width, int height, int mx, int my)
{
    int x, y;
    pixel *dst = (pixel*)_dst;
    pixel *src = (pixel*)_src;
    int dststride = _dststride/sizeof(pixel);
    int srcstride = _srcstride/sizeof(pixel);
    const int8_t *filter = epel_filters[mx-1];

    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++)
            dst[x] = EPEL_FILTER(src, 1, filter) >> (BIT_DEPTH - 8);
        src += srcstride;
        dst += dststride;
    }
}

static void FUNC(put_hevc_epel_v)(uint8_t * _dst, ptrdiff_t _dststride,
                                  uint8_t *_src, ptrdiff_t _srcstride,
                                  int width, int height, int mx, int my)
{
    int x, y;
    pixel *dst = (pixel*)_dst;
    pixel *src = (pixel*)_src;
    int dststride = _dststride/sizeof(pixel);
    int srcstride = _srcstride/sizeof(pixel);

    const int8_t *filter = epel_filters[my-1];

    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++)
            dst[x] = EPEL_FILTER(src, srcstride, filter) >> (BIT_DEPTH - 8);
        src += srcstride;
        dst += dststride;
    }
}

static void FUNC(put_hevc_epel_hv)(uint8_t * _dst, ptrdiff_t _dststride,
                                   uint8_t *_src, ptrdiff_t _srcstride,
                                   int width, int height, int mx, int my)
{
    int x, y;
    pixel *dst = (pixel*)_dst;
    pixel *src = (pixel*)_src;
    int dststride = _dststride/sizeof(pixel);
    int srcstride = _srcstride/sizeof(pixel);

    const int8_t *filter_h = epel_filters[mx-1];
    const int8_t *filter_v = epel_filters[my-1];

    int tmpstride = MAX_PB_SIZE;
    pixel tmp_array[(MAX_PB_SIZE+3)*MAX_PB_SIZE];
    pixel *tmp = tmp_array;

    src -= epel_extra_before * srcstride;

    for (y = 0; y < height + epel_extra; y++) {
        for (x = 0; x < width; x++)
            tmp[x] = EPEL_FILTER(src, 1, filter_h) >> (BIT_DEPTH - 8);
        src += srcstride;
        dst += dststride;
    }

    tmp += epel_extra_before * tmpstride;

    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++)
            dst[x] = EPEL_FILTER(tmp, tmpstride, filter_v) >> 6;
        src += srcstride;
        dst += dststride;
    }
}
