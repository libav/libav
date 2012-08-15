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
#include "hevcpred.h"

#define BIT_DEPTH 8
#include "hevcpred_template.c"
#undef BIT_DEPTH

#define BIT_DEPTH 9
#include "hevcpred_template.c"
#undef BIT_DEPTH

void ff_hevc_pred_init(HEVCPredContext *hpc, int bit_depth)
{
#undef FUNCC
#define FUNCC(a, depth) a ## _ ## depth ## _c

#define HEVC_PRED(depth)                            \
    hpc->intra_pred   = FUNCC(intra_pred, depth);   \
    hpc->pred_planar  = FUNCC(pred_planar, depth);  \
    hpc->pred_dc      = FUNCC(pred_dc, depth);      \
    hpc->pred_angular = FUNCC(pred_angular, depth);

    if (bit_depth > 8) {
        HEVC_PRED(9);
    } else {
        HEVC_PRED(8);
    }
}
