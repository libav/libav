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

#ifndef AVCODEC_HEVCPRED_H
#define AVCODEC_HEVCPRED_H

typedef struct HEVCContext HEVCContext;

typedef struct HEVCPredContext {
    void (*intra_pred)(HEVCContext *s, int x0, int y0, int log2_size, int c_idx);

    void(*pred_planar)(uint8_t *src, const uint8_t *top, const uint8_t *left, ptrdiff_t stride,
                       int log2_size);
    void(*pred_dc)(uint8_t *src, const uint8_t *top, const uint8_t *left, ptrdiff_t stride,
                   int log2_size, int c_idx);
    void(*pred_angular)(uint8_t *src, const uint8_t *top, const uint8_t *left, ptrdiff_t stride,
                        int log2_size, int c_idx, int mode, int bit_depth);
} HEVCPredContext;

void ff_hevc_pred_init(HEVCPredContext *hpc, int bit_depth);

#endif /* AVCODEC_HEVCPRED_H */
