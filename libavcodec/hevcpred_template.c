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

#include "libavutil/pixdesc.h"
#include "bit_depth_template.c"
#include "hevcpred.h"

#define POS(x, y) src[(x) + stride * (y)]

static void FUNCC(intra_pred)(HEVCContext *s, int x0, int y0, int log2_size, int c_idx)
{
#define MIN_TB_ADDR_ZS(x, y)                                            \
    s->pps->min_tb_addr_zs[(y) * s->sps->pic_width_in_min_tbs + (x)]

#define EXTEND_LEFT(ptr, length)                \
    for (i = 0; i < (length); i++)              \
        (ptr)[-(i+1)] = (ptr)[0];
#define EXTEND_RIGHT(ptr, length)               \
    for (i = 0; i < (length); i++)              \
        (ptr)[i+1] = (ptr)[0];
#define EXTEND_UP(ptr, length) EXTEND_LEFT(ptr, length)
#define EXTEND_DOWN(ptr, length) EXTEND_RIGHT(ptr, length)

    int i;
    int hshift = s->sps->hshift[c_idx];
    int vshift = s->sps->vshift[c_idx];
    int size = (1 << log2_size);
    int size_in_luma = size << hshift;
    int size_in_tbs = size_in_luma >> s->sps->log2_min_transform_block_size;
    int x = x0 >> hshift;
    int y = y0 >> vshift;
    int x_tb = x0 >> s->sps->log2_min_transform_block_size;
    int y_tb = y0 >> s->sps->log2_min_transform_block_size;
    int cur_tb_addr = MIN_TB_ADDR_ZS(x_tb, y_tb);

    ptrdiff_t stride = s->frame.linesize[c_idx] / sizeof(pixel);
    pixel *src = (pixel*)s->frame.data[c_idx] + x + y * stride;

    enum IntraPredMode mode = c_idx ? s->pu.intra_pred_mode_c :
                              s->tu.cur_intra_pred_mode;

    pixel left_array[2*MAX_TB_SIZE+1], filtered_left_array[2*MAX_TB_SIZE+1];
    pixel top_array[2*MAX_TB_SIZE+1], filtered_top_array[2*MAX_TB_SIZE+1];
    pixel *left = left_array + 1;
    pixel *top = top_array + 1;
    pixel *filtered_left = filtered_left_array + 1;
    pixel *filtered_top = filtered_top_array + 1;


    int bottom_left_available = x_tb > 0 && (y_tb + size_in_tbs) < s->sps->pic_height_in_min_tbs &&
                                cur_tb_addr > MIN_TB_ADDR_ZS(x_tb - 1, y_tb + size_in_tbs);
    int left_available = x0 > 0;
    int top_left_available = x0 > 0 && y0 > 0;
    int top_available = y0 > 0;
    int top_right_available = y_tb > 0 && (x_tb + size_in_tbs) < s->sps->pic_width_in_min_tbs &&
                              cur_tb_addr > MIN_TB_ADDR_ZS(x_tb + size_in_tbs, y_tb - 1);

    int bottom_left_size = (FFMIN(y0 + 2*size_in_luma, s->sps->pic_height_in_luma_samples) -
                            (y0 + size_in_luma)) >> vshift;
    int top_right_size = (FFMIN(x0 + 2*size_in_luma, s->sps->pic_width_in_luma_samples) -
                          (x0 + size_in_luma)) >> hshift;

    // Fill left and top with the available samples
    if (bottom_left_available) {
        for (i = 0; i < bottom_left_size; i++)
            left[size + i] = POS(-1, size + i);
    }
    if (left_available) {
        for (i = 0; i < size; i++)
            left[i] = POS(-1, i);
    }
    if (top_left_available)
        left[-1] = POS(-1, -1);
    if (top_available && top_right_available && top_right_size == size) {
        top = &POS(0, -1);
    } else {
        if (top_available)
            memcpy(&top[0], &POS(0, -1), size * sizeof(pixel));
        if (top_right_available)
            memcpy(&top[size], &POS(size, -1), top_right_size * sizeof(pixel));
    }

    // Infer the unavailable samples
    if (!bottom_left_available) {
        if (left_available) {
            EXTEND_DOWN(&left[size-1], size);
            bottom_left_available = 1;
        } else if (top_left_available) {
            EXTEND_DOWN(&left[-1], 2*size);
            left_available = 1;
            bottom_left_available = 1;
        } else if (top_available) {
            left[-1] = top[0];
            EXTEND_DOWN(&left[-1], 2*size);
            top_left_available = 1;
            left_available = 1;
            bottom_left_available = 1;
        } else if (top_right_available) {
            EXTEND_LEFT(&top[size], size);
            left[-1] = top[0];
            EXTEND_DOWN(&left[-1], 2*size);
            top_available = 1;
            top_left_available = 1;
            left_available = 1;
            bottom_left_available = 1;
        } else { // No samples available
            top[0] = left[-1] = (1 << (s->sps->bit_depth[c_idx] - 1));
            EXTEND_RIGHT(&top[0], 2*size-1);
            EXTEND_DOWN(&left[-1], 2*size);
        }
    }
    if (!left_available) {
        EXTEND_UP(&left[size], size);
        left_available = 1;
    }
    if (!top_left_available) {
        left[-1] = left[0];
        top_left_available = 1;
    }
    if (!top_available) {
        top[0] = left[-1];
        EXTEND_RIGHT(&top[0], size-1);
        top_available = 1;
    }
    if (!top_right_available)
        EXTEND_RIGHT(&top[size-1], size);

    if (bottom_left_size < size)
        EXTEND_DOWN(&left[size + bottom_left_size - 1], size - bottom_left_size);
    if (top_right_size < size)
        EXTEND_RIGHT(&top[size + top_right_size - 1], size - top_right_size);

    top[-1] = left[-1];

#undef EXTEND_LEFT
#undef EXTEND_RIGHT
#undef EXTEND_UP
#undef EXTEND_DOWN
#undef MIN_TB_ADDR_ZS

    if (c_idx == 0 && mode != INTRA_DC && size != 4) {
        int intra_hor_ver_dist_thresh[] = { 7, 1, 0 };
        int min_dist_vert_hor = FFMIN(FFABS((int)mode-26), FFABS((int)mode-10));
        if (min_dist_vert_hor > intra_hor_ver_dist_thresh[log2_size-3]) {
            filtered_left[2*size-1] = left[2*size-1];
            filtered_top[2*size-1]  = top[2*size-1];
            for (i = 2*size-2; i >= 0; i--) {
                filtered_left[i] = (left[i+1] + 2*left[i] + left[i-1] + 2) >> 2;
            }
            filtered_top[-1] = filtered_left[-1] = (left[0] + 2*left[-1] + top[0] + 2) >> 2;
            for (i = 2*size-2; i >= 0; i--) {
                filtered_top[i] = (top[i+1] + 2*top[i] + top[i-1] + 2) >> 2;
            }
            left = filtered_left;
            top = filtered_top;
        }
    }

    switch(mode) {
    case INTRA_PLANAR:
        s->hpc[c_idx]->pred_planar((uint8_t*)src, (uint8_t*)top, (uint8_t*)left, stride, log2_size);
        break;
    case INTRA_DC:
        s->hpc[c_idx]->pred_dc((uint8_t*)src, (uint8_t*)top, (uint8_t*)left, stride, log2_size, c_idx);
        break;
    default:
        s->hpc[c_idx]->pred_angular((uint8_t*)src, (uint8_t*)top, (uint8_t*)left, stride, log2_size, c_idx,
                            mode, s->sps->bit_depth[c_idx]);
        break;
    }
}

static void FUNCC(pred_planar)(uint8_t *_src, const uint8_t *_top, const uint8_t *_left,
                               ptrdiff_t stride, int log2_size)
{
    int x, y;
    int size = (1 << log2_size);
    pixel *src = (pixel*)_src;
    const pixel *top = (const pixel*)_top;
    const pixel *left = (const pixel*)_left;
    for (y = 0; y < size; y++)
        for (x = 0; x < size; x++)
            POS(x, y) = ((size - 1 - x) * left[y]  + (x + 1) * top[size] +
                         (size - 1 - y) * top[x] + (y + 1) * left[size] + size) >>
                        (log2_size + 1);
}

static void FUNCC(pred_dc)(uint8_t *_src, const uint8_t *_top, const uint8_t *_left,
                           ptrdiff_t stride, int log2_size, int c_idx)
{
    int i, j, x, y;
    int size = (1 << log2_size);
    pixel *src = (pixel*)_src;
    const pixel *top = (const pixel*)_top;
    const pixel *left = (const pixel*)_left;
    int dc = size;
    pixel4 a;
    for (i = 0; i < size; i++)
        dc += left[i] + top[i];

    dc >>= log2_size + 1;

    a = PIXEL_SPLAT_X4(dc);

    for (i = 0; i < size; i++)
        for (j = 0; j < size / sizeof(pixel4); j++)
            AV_WN4PA(&POS(j * sizeof(pixel4), i), a);

    if (c_idx == 0) {
        POS(0, 0) = (left[0] + 2 * dc  + top[0] + 2) >> 2;
        for (x = 1; x < size; x++)
            POS(x, 0) = (top[x] + 3 * dc + 2) >> 2;
        for (y = 1; y < size; y++)
            POS(0, y) = (left[y] + 3 * dc + 2) >> 2;
    }
}

static void FUNCC(pred_angular)(uint8_t *_src, const uint8_t *_top, const uint8_t *_left,
                                ptrdiff_t stride, int log2_size, int c_idx, int mode, int bit_depth)
{
#define CLIP_1(x) av_clip_c((x), 0, (1 << bit_depth) - 1)

    int x, y;
    int size = 1 << log2_size;
    pixel *src = (pixel*)_src;
    const pixel *top = (const pixel*)_top;
    const pixel *left = (const pixel*)_left;

    const int intra_pred_angle[] = {
        32, 26, 21, 17, 13, 9, 5, 2, 0, -2, -5, -9, -13, -17, -21, -26, -32,
        -26, -21, -17, -13, -9, -5, -2, 0, 2, 5, 9, 13, 17, 21, 26, 32
    };
    const int inv_angle[] = {
        -4096, -1638, -910, -630, -482, -390, -315, -256, -315, -390, -482,
        -630, -910, -1638, -4096
    };

    int angle = intra_pred_angle[mode-2];
    pixel ref_array[3*MAX_TB_SIZE+1];
    const pixel *ref;
    int last = (size * angle) >> 5;

    if (mode >= 18) {
        ref = top - 1;
        if (angle < 0 && last < -1) {
            for (x = 0; x <= size; x++)
                (ref_array + size)[x] = top[x - 1];
            for (x = last; x <= -1; x++)
                (ref_array + size)[x] = left[-1 + ((x * inv_angle[mode-11] + 128) >> 8)];
            ref = ref_array + size;
        }

        for (y = 0; y < size; y++) {
            int idx = ((y + 1) * angle) >> 5;
            int fact = ((y + 1) * angle) & 31;
            for (x = 0; x < size; x++) {
                POS(x, y) = ((32 - fact) * ref[x + idx + 1] + fact * ref[x + idx + 2] + 16) >> 5;
            }
        }
        if (mode == 26 && c_idx == 0) {
            for (y = 0; y < size; y++)
                POS(0, y) = CLIP_1(top[0] + ((left[y] - left[-1]) >> 1));
        }
    } else {
        ref = left - 1;
        if (angle < 0 && last < -1) {
            for (x = 0; x <= size; x++)
                (ref_array + size)[x] = left[x - 1];
            for (x = last; x <= -1; x++)
                (ref_array + size)[x] = top[-1 + ((x * inv_angle[mode-11] + 128) >> 8)];
            ref = ref_array + size;
        }

        for (x = 0; x < size; x++) {
            int idx = ((x + 1) * angle) >> 5;
            int fact = ((x + 1) * angle) & 31;
            for (y = 0; y < size; y++) {
                POS(x, y) = ((32 - fact) * ref[y + idx + 1] + fact * ref[y + idx + 2] + 16) >> 5;
            }
        }
        if (mode == 10 && c_idx == 0) {
            for (x = 0; x < size; x++)
                POS(x, 0) = CLIP_1(left[0] + ((top[x] - top[-1]) >> 1));
        }
    }
#undef CLIP_1
}

#undef POS
