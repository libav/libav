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

#include "hevc.h"
#include "hevcdsp.h"

#define BIT_DEPTH 8
#include "hevcdsp_template.c"
#undef BIT_DEPTH

#define BIT_DEPTH 9
#include "hevcdsp_template.c"
#undef BIT_DEPTH

void ff_hevc_dsp_init(HEVCDSPContext *hevcdsp, int bit_depth)
{
#undef FUNC
#define FUNC(a, depth) a ## _ ## depth

#define HEVC_DSP(depth)                                                     \
    hevcdsp->transquant_bypass = FUNC(transquant_bypass, depth);            \
    hevcdsp->transform_skip = FUNC(transform_skip, depth);                  \
    hevcdsp->transform_4x4_luma_add = FUNC(transform_4x4_luma_add, depth);  \
    hevcdsp->transform_add[0] = FUNC(transform_4x4_add, depth);             \
    hevcdsp->transform_add[1] = FUNC(transform_8x8_add, depth);             \
    hevcdsp->transform_add[2] = FUNC(transform_16x16_add, depth);           \
    hevcdsp->transform_add[3] = FUNC(transform_32x32_add, depth);           \

    if (bit_depth > 8) {
        HEVC_DSP(9);
    } else {
        HEVC_DSP(8);
    }
}
