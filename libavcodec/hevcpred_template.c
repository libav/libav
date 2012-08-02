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

#define POS(x,y) src[(x) + stride * (y)]

static void FUNCC(intra_pred)(struct HEVCContext *s, int x0, int y0, int log2_size, int c_idx)
{
#define LEFT_CB_AVAILABLE(i)                                            \
    (x0 > 0 && (y0 + (i)) >= 0 && (y0 + (i)) < s->sps->pic_height_in_luma_samples && \
     (x0 != s->cu.x || s->cu.left_cb_available[(y0+(i))>>s->sps->log2_min_coding_block_size]))

#define TOP_CB_AVAILABLE(i)\
    (y0 > 0 && (x0 + (i)) >= 0 && (x0 + (i)) < s->sps->pic_width_in_luma_samples && \
     (y0 != s->cu.y || s->cu.top_cb_available[(x0+(i))>>s->sps->log2_min_coding_block_size]))

#define EXTEND_LEFT(ptr, length)\
    for (int i = 0; i < (length); i++)\
        (ptr)[-(i+1)] = (ptr)[0];
#define EXTEND_RIGHT(ptr, length)\
    for (int i = 0; i < (length); i++)\
        (ptr)[i+1] = (ptr)[0];
#define EXTEND_UP(ptr, length) EXTEND_LEFT(ptr, length)
#define EXTEND_DOWN(ptr, length) EXTEND_RIGHT(ptr, length)

    int size = (1 << log2_size);
    int hshift = c_idx ? av_pix_fmt_descriptors[s->avctx->pix_fmt].log2_chroma_w : 0;
    int vshift = c_idx ? av_pix_fmt_descriptors[s->avctx->pix_fmt].log2_chroma_h : 0;
    int x = x0 >> hshift;
    int y = y0 >> vshift;

    int stride = s->frame.linesize[c_idx]/sizeof(pixel);
    pixel *src = (pixel*)&s->frame.data[c_idx][x + y * stride];

    enum IntraPredMode mode = c_idx ? s->pu.intra_pred_mode_c :
                              s->tu.cur_intra_pred_mode;

    pixel left_array[2*MAX_TB_SIZE+1], filtered_left_array[2*MAX_TB_SIZE+1];
    pixel top_array[2*MAX_TB_SIZE+1], filtered_top_array[2*MAX_TB_SIZE+1];
    pixel *left = left_array + 1;
    pixel *top = top_array + 1;
    pixel *filtered_left = filtered_left_array + 1;
    pixel *filtered_top = filtered_top_array + 1;

    const uint32_t mask = (1 << (s->sps->log2_min_coding_block_size +
                                 s->sps->log2_diff_max_min_coding_block_size)) - 1;

    // Each bit determine the position of the block in its parent quadtree,
    // read the bits from right to left to go up in the tree.
    uint32_t right_mask = (x0 & mask) >> (log2_size + hshift);
    uint32_t top_mask   = (~y0 & mask) >> (log2_size + vshift);

    int bottom_left_available, left_available, top_left_available;
    int top_available, top_right_available;

    int has_bottom_left = 0;
    int has_top_right = 1;
    int left_block = (right_mask & 1) == 0;

    if (left_block) {
        // Bottom left samples are available only if the current block is on
        // the left side of a top left block
        uint32_t first_right = right_mask & (-right_mask);
        uint32_t first_top   = top_mask & (-top_mask);
        has_bottom_left = first_top != 0 && (first_right == 0 || first_right > first_top);
    } else {
        // Top right samples are available only if the current block is _not_ on
        // the right side of a bottom right block
        uint32_t left_mask   = (~right_mask) & mask;
        uint32_t bottom_mask = (~top_mask) & mask;
        uint32_t first_left   = left_mask & (-left_mask);
        uint32_t first_bottom = bottom_mask & (-bottom_mask);
        has_top_right = !(first_bottom != 0 && (first_left == 0 || (first_left > first_bottom)));
    }

    bottom_left_available = has_bottom_left &&
                            LEFT_CB_AVAILABLE(size << vshift);
    left_available = LEFT_CB_AVAILABLE(0);
    top_left_available = LEFT_CB_AVAILABLE(-1);
    top_available = TOP_CB_AVAILABLE(0);
    top_right_available = has_top_right &&
                           TOP_CB_AVAILABLE(size << hshift);

    // Fill left and top with the available samples
    if (bottom_left_available) {
        for (int i = 0; i < size; i++)
            left[size + i] = POS(-1, size + i);
    }
    if (left_available) {
        for (int i = 0; i < size; i++)
            left[i] = POS(-1, i);
    }
    if (top_left_available)
        left[-1] = POS(-1, -1);
    if (top_available && top_right_available) {
        top = &POS(0,-1);
    } else if (top_available) {
        memcpy(&top[0], &POS(0, -1), size * sizeof(pixel));
    } else if (top_right_available) {
        memcpy(&top[size], &POS(size, -1), size * sizeof(pixel));
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
            top[0] = left[-1] = (1 << (s->sps->bit_depth_luma - 1));
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
    if (!top_right_available) {
        EXTEND_RIGHT(&top[size-1], size);
    }

    top[-1] = left[-1];

#undef EXTEND_LEFT
#undef EXTEND_RIGHT
#undef EXTEND_UP
#undef EXTEND_DOWN
#undef LEFT_CB_AVAILABLE
#undef TOP_CB_AVAILABLE

    if (c_idx == 0 && mode != INTRA_DC && size != 4) {
        int intra_hor_ver_dist_thresh[] = { 7, 1, 0 };
        int min_dist_vert_hor = FFMIN(FFABS((int)mode-26),FFABS((int)mode-10));
        if (min_dist_vert_hor > intra_hor_ver_dist_thresh[log2_size-3]) {
            filtered_left[2*size-1] = left[2*size-1];
            filtered_top[2*size-1]  = top[2*size-1];
            for (int y = 2*size-2; y >= 0; y--) {
                filtered_left[y] = (left[y+1] + 2*left[y] + left[y-1] + 2) >> 2;
            }
            filtered_top[-1] = filtered_left[-1] = (left[0] + 2*left[-1] + top[0] + 2) >> 2;
            for (int x = 2*size-2; x >= 0; x--) {
                filtered_top[x] = (top[x+1] + 2*top[x] + top[x-1] + 2) >> 2;
            }
            left = filtered_left;
            top = filtered_top;
        }
    }

    switch(mode) {
    case INTRA_PLANAR:
        s->hpc.pred_planar((uint8_t*)src, (uint8_t*)top, (uint8_t*)left, stride, log2_size);
        break;
    case INTRA_DC:
        s->hpc.pred_dc((uint8_t*)src, (uint8_t*)top, (uint8_t*)left, stride, log2_size, c_idx);
        break;
    default:
        s->hpc.pred_angular((uint8_t*)src, (uint8_t*)top, (uint8_t*)left, stride, log2_size, c_idx,
                                    mode, s->sps->bit_depth_luma);
        break;
    }
}

static void FUNCC(pred_planar)(uint8_t *_src, const uint8_t *_top, const uint8_t *_left,
                               int stride, int log2_size)
{
    int size = (1 << log2_size);
    pixel *src = (pixel*)_src;
    const pixel *top = (const pixel*)_top;
    const pixel *left = (const pixel*)_left;
    for (int y = 0; y < size; y++)
        for (int x = 0; x < size; x++)
            POS(x,y) = ((size - 1 - x) * left[y]  + (x + 1) * top[size] +
                        (size - 1 - y) * top[x] + (y + 1) * left[size] + size) >>
                       (log2_size + 1);
}

static void FUNCC(pred_dc)(uint8_t *_src, const uint8_t *_top, const uint8_t *_left,
                           int stride, int log2_size, int c_idx)
{
    int size = (1 << log2_size);
    pixel *src = (pixel*)_src;
    const pixel *top = (const pixel*)_top;
    const pixel *left = (const pixel*)_left;
    int dc = size;
    pixel4 a;
    for (int i = 0; i < size; i++)
        dc += left[i] + top[i];

    dc >>= log2_size + 1;

    a = PIXEL_SPLAT_X4(dc);

    for (int i = 0; i < size; i++)
        for (int j = 0; j < size/sizeof(pixel4); j++)
            AV_WN4PA(&POS(j * sizeof(pixel4), i), a);

    if (c_idx == 0) {
        POS(0, 0) = (left[0] + 2 * dc  + top[0] + 2) >> 2;
        for (int x = 1; x < size; x++)
            POS(x, 0) = (top[x] + 3 * dc + 2) >> 2;
        for (int y = 1; y < size; y++)
            POS(0, y) = (left[y] + 3 * dc + 2) >> 2;
    }
}

static void FUNCC(pred_angular)(uint8_t *_src, const uint8_t *_top, const uint8_t *_left,
                                int stride, int log2_size, int c_idx, int mode, int bit_depth)
{
#define CLIP_1(x) av_clip_c((x), 0, (1 << bit_depth) - 1)

    int size = 1 << log2_size;
    pixel *src = (pixel*)_src;
    const pixel *top = (const pixel*)_top;
    const pixel *left = (const pixel*)_left;

    const int intra_pred_angle[] = {
        32, 26, 21, 17, 13, 9, 5, 2, 0, -2, -5, -9, -13,-17, -21, -26, -32,
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
            for (int x = 0; x <= size; x++)
                (ref_array + size)[x] = top[x - 1];
            for (int x = last; x <= -1; x++)
                (ref_array + size)[x] = left[-1 + ((x * inv_angle[mode-11] + 128) >> 8)];
            ref = ref_array + size;
        }

        for (int y = 0; y < size; y++) {
            int idx = ((y + 1) * angle) >> 5;
            int fact = ((y + 1) * angle) & 31;
            for (int x = 0; x < size; x++) {
                POS(x, y) = ((32 - fact) * ref[x + idx + 1] + fact * ref[x + idx + 2] + 16) >> 5;
            }
        }
        if (mode == 26 && c_idx == 0) {
            for (int y = 0; y < size; y++)
                POS(0, y) = CLIP_1(top[0] + ((left[y] - left[-1]) >> 1));
        }
    } else {
        ref = left - 1;
        if (angle < 0 && last < -1) {
            for (int x = 0; x <= size; x++)
                (ref_array + size)[x] = left[x - 1];
            for (int x = last; x <= -1; x++)
                (ref_array + size)[x] = top[-1 + ((x * inv_angle[mode-11] + 128) >> 8)];
            ref = ref_array + size;
        }

        for (int x = 0; x < size; x++) {
            int idx = ((x + 1) * angle) >> 5;
            int fact = ((x + 1) * angle) & 31;
            for (int y = 0; y < size; y++) {
                POS(x, y) = ((32 - fact) * ref[y + idx + 1] + fact * ref[y + idx + 2] + 16) >> 5;
            }
        }
        if (mode == 10 && c_idx == 0) {
            for (int x = 0; x < size; x++)
                POS(x, 0) = CLIP_1(left[0] + ((top[x] - top[-1]) >> 1));
        }
    }
#undef CLIP_1
}

#undef POS
