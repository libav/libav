/*
 * HEVC CABAC decoding
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

#include "libavutil/attributes.h"
#include "libavutil/common.h"
#include "cabac.h"
#include "hevc.h"

/**
 * Binarization methods described in 9.2.2
 */
enum Binarization {
    FL_BIN = 0,
#if REFERENCE_ENCODER_QUIRKS
    U_BIN,
#endif
    EG_BIN,
    TU_BIN,
    CU_QP_DELTA_BIN,
    PART_MODE_BIN,
    INTRA_CHROMA_PRED_MODE_BIN,
    COEFF_ABS_LEVEL_REMAINING_BIN,
    BINARIZATION_COUNT
};

static int fl_binarization(HEVCContext *s, int cMax);
#if REFERENCE_ENCODER_QUIRKS
static int u_binarization(HEVCContext *s, int unusued);
#endif
static int eg_binarization(HEVCContext *s, int k);

typedef int (*binarization_func)(HEVCContext *s, int arg);

static const binarization_func binarization_funcs[BINARIZATION_COUNT] =
{
    &fl_binarization,
#if REFERENCE_ENCODER_QUIRKS
    &u_binarization,
#endif
    &eg_binarization,
    0
};

/**
 * Binarization type from Table 9-63, indexed by SyntaxElement.
 */
static const uint8_t binarization[][2] =
{
    { FL_BIN, 1 },
    { FL_BIN, 1 },
#if REFERENCE_ENCODER_QUIRKS
    { U_BIN, 0 },
#else
    { FL_BIN, 3 },
#endif
    { FL_BIN, 5 },
    { EG_BIN, 0 },
    { FL_BIN, 1 },
    { FL_BIN, 1 },
    { FL_BIN, 1 }
};

/**
 * maxBinIdxCtx from Table 9-63, indexed by SyntaxElement.
 */
static const uint8_t max_bin_idx_ctxs[][3] =
{
    { 0, 0, 0 },
    { 0, 0, 0 },
    { 1, 1, 1 },
    { -1 },
    { 2, 2, 2 },
    { 0, 0, 0 },
    { 0, 0, 0 },
    { 0, 0, 0 }
};

/**
 * CtxIdxOffset from Table 9-63, indexed by SyntaxElement.
 */
static const int8_t ctx_idx_offsets[][3] =
{
    { 0, 3, 6 },
    { 0, 1, 2 },
    { 0, 2, 4 },
    { -1 },
    { 0, 3, 6 },
    { 0, 1, 2 },
    { -1 },
    { 0, 3, 6 }
};

/**
 * ctxIdxInc from Table 9-68, indexed by SyntaxElement.
 */
static const int8_t ctx_idx_incs[][5] =
{
    { -1 },
    { 0 },
    { 0, 1, 1 },
    { -1 },
    { 0, 1, 2, 2, 2 },
    { 0 },
    { -1 },
    { -1 }
};

/**
 * Offset to ctxIdx 0 in init_values and states, indexed by SyntaxElement.
 */
static const uint8_t elem_offset[] =
{
    0,
    9,
    12,
    -1,
    18,
    27,
    -1,
    30,
    39
};

//FIXME: calculate the real value
#define CTX_IDX_COUNT 256

/**
 * initValue from Tables 9-36 to 9-59, indexed by ctx_idx for each SyntaxElement
 * value.
 *
 * NOTE: these values do not match the spec because the spec
 * is wrong, see http://hevc.kw.bbc.co.uk/trac/ticket/473 .
 */
static const uint8_t init_values[CTX_IDX_COUNT] =
{
    153, 153, 153, 153, 153, 153, 153, 153, 153, //sao_merge_left_flag
    175, 153, 153, //sao_merge_up_flag
    160, 140, 185, 140, 200, 140, //sao_type_idx
    247, 154, 244, 215, 154, 169, 215, 154, 169, //sao_offset
    153, 153, 153, //alf_cu_flag
    139, 141, 157, 107, 139, 126, 107, 139, 126 //split_coding_unit_flag
};

/**
 * mps and pstate, indexed by ctx_idx
 */
static uint8_t states[CTX_IDX_COUNT][2];

/**
 * 9.2.3.1
 */
static int derive_ctx_idx(HEVCContext *s, int bin_idx)
{
    HEVCCabacContext *cc = &s->cc;

    int ctx_idx_inc = cc->ctx_idx_inc[FFMIN(bin_idx, cc->max_bin_idx_ctx)];

    if (ctx_idx_inc == -1) {
        switch (cc->elem) {
        default:
            av_log(s->avctx, AV_LOG_ERROR,
                   "TODO: ctxIdxInc for elem %d, assuming 0.\n", cc->elem);
            ctx_idx_inc = 0;
        }
    }

    return cc->ctx_idx_offset + ctx_idx_inc;
}

/**
 * 9.2.3.2
 */
static int decode_bin(HEVCContext *s, int bin_idx)
{
    HEVCCabacContext *cc = &s->cc;
    GetBitContext *gb = &s->gb;

    int ctx_idx = derive_ctx_idx(s, bin_idx);
    uint8_t *state = cc->state[ctx_idx];

    int mps = state[0];
    int pstate = state[1];

    int lpsrange = lps_range[pstate][(cc->range >> 6) & 3];
    int bin_val = 0;

    av_log(s->avctx, AV_LOG_DEBUG,
           "ctx_idx: %d, pstate: %d, mps: %d\n", ctx_idx, pstate, mps);

    cc->range -= lpsrange;
    if (cc->offset >= cc->range) {
        bin_val = 1 - mps;
        cc->offset -= cc->range;
        cc->range = lpsrange;
    } else {
        bin_val = mps;
    }

    if (bin_val == mps) {
        state[1] = mps_state[pstate];
    } else {
        if (pstate == 0)
            state[0] = !mps;
        state[1] = lps_state[pstate];
    }

    while (cc->range < 256) {
        cc->range <<= 1;
        cc->offset <<= 1;
        cc->offset |= get_bits1(gb);
    }

    av_log(s->avctx, AV_LOG_DEBUG, "cc->range: %d, cc->offset: %d, bin_val: %d\n",
           cc->range, cc->offset, bin_val);
    return bin_val;
}

#ifdef REFERENCE_ENCODER_QUIRKS
/**
 * 9.2.2.1
 */
static int u_binarization(HEVCContext *s, int unused)
{
    int i = 0;

    while (decode_bin(s, i++) == 1);

    return i - 1;
}
#endif

/**
 * 9.2.2.4
 */
static int eg_binarization(HEVCContext *s, int k)
{
    int suffix = 0;
    int log = 0;
    int i = 0;

    while (decode_bin(s, i++) == 1);

    log = i - 1;
    for (int j = 0; j < log; j++)
        suffix = (suffix << 1) | decode_bin(s, i++);

    return (1 << (k + log)) - (1 << k) + suffix;
}

/**
 * 9.2.2.5
 */
static int fl_binarization(HEVCContext *s, int cMax)
{
    int value = 0;
    int length = av_ceil_log2_c(cMax + 1);

    for (int i = 0; i < length; i++)
        value = (value << 1) | decode_bin(s, i);

    return value;
}

/**
 * 9.2.1
 */
void ff_hevc_cabac_init(HEVCContext *s)
{
    HEVCCabacContext *cc = &s->cc;
    GetBitContext *gb = &s->gb;

    cc->range = 510;
    cc->offset = get_bits(gb, 9);
    av_log(s->avctx, AV_LOG_DEBUG, "cc->offset: %d\n", cc->offset);

    for (int i = 0; i < CTX_IDX_COUNT; i++) {
        int init_value = init_values[i];
        int m = (init_value >> 4)*5 - 45;
        int n = ((init_value & 15) << 3) - 16;
        int pre_ctx_state = av_clip_c((m * av_clip_c(s->sh.slice_qp, 0, 51) >> 4) + n,
                                    1, 126);
        states[i][0] = (pre_ctx_state <= 63) ? 0 : 1; //mps
        states[i][1] = states[i][0] ? (pre_ctx_state - 64) : (63 - pre_ctx_state); //stateIdx
    }
}

int ff_hevc_cabac_decode(HEVCContext *s, enum SyntaxElement elem)
{
    HEVCCabacContext *cc = &s->cc;

#if REFERENCE_ENCODER_QUIRKS
    int initialisation_type = s->sh.slice_type;
#else
    int initialisation_type = (s->sh.slice_type + 1) % 3;
#endif

    if (s->sh.cabac_init_flag && s->sh.slice_type != I_SLICE)
        initialisation_type = 2;
    cc->elem = elem;
    cc->state = states + elem_offset[elem];

    // 9.2.2
    //TODO: bypassFlag
    cc->max_bin_idx_ctx = max_bin_idx_ctxs[elem][initialisation_type];
    cc->ctx_idx_offset = ctx_idx_offsets[elem][initialisation_type];
    cc->ctx_idx_inc = ctx_idx_incs[elem];

    return binarization_funcs[binarization[elem][0]](s, binarization[elem][1]);
}

