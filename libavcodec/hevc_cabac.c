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
    U_BIN,
    EG_BIN,
    TU_BIN,
    CU_QP_DELTA_BIN,
    PART_MODE_BIN,
    INTRA_CHROMA_PRED_MODE_BIN,
    COEFF_ABS_LEVEL_REMAINING_BIN,
    BINARIZATION_COUNT
};

static int fl_binarization(HEVCContext *s, int cMax);
static int u_binarization(HEVCContext *s, int unused);
static int eg_binarization(HEVCContext *s, int k);
static int tu_binarization(HEVCContext *s, int cMax);
static int cu_qp_delta_binarization(HEVCContext *s, int unused);
static int part_mode_binarization(HEVCContext *s, int unused);
static int intra_chroma_pred_mode_binarization(HEVCContext *s, int unused);
static int coeff_abs_level_remaining_binarization(HEVCContext *s, int unused);

typedef int (*binarization_func)(HEVCContext *s, int arg);

static const binarization_func binarization_funcs[BINARIZATION_COUNT] =
{
    &fl_binarization,
    &u_binarization,
    &eg_binarization,
    &tu_binarization,
    &cu_qp_delta_binarization,
    &part_mode_binarization,
    &intra_chroma_pred_mode_binarization,
    &coeff_abs_level_remaining_binarization
};

/**
 * Binarization type from Table 9-63, indexed by SyntaxElement.
 */
static const int8_t binarization[][2] =
{
    { FL_BIN, 1 }, //sao_merge_left_flag
    { FL_BIN, 1 }, //sao_merge_up_flag
    { U_BIN }, //sao_type_idx
    { FL_BIN, 5 }, //sao_band_position
    { FL_BIN, 1 }, //sao_offset_sign
    { TU_BIN, -1 }, //sao_offset
    { FL_BIN, 1 }, //alf_cu_flag
    { FL_BIN, 1 }, //end_of_slice_flag
    { FL_BIN, 1 }, //split_coding_unit_flag
    { FL_BIN, 1 }, //cu_transquant_bypass_flag
    { FL_BIN, 1 }, //skip_flag
    { CU_QP_DELTA_BIN }, //cu_qp_delta
    { FL_BIN, 1 }, //pred_mode
    { PART_MODE_BIN }, //part_mode
    { FL_BIN, 1 }, //pcm_flag
    { FL_BIN, 1 }, //prev_intra_luma_pred_flag
    { TU_BIN, 1 }, //mpm_idx
    { FL_BIN, 31 }, //rem_intra_luma_pred_mode
    { INTRA_CHROMA_PRED_MODE_BIN }, //intra_chroma_pred_mode
    { FL_BIN, 1 }, //merge_flag
    { TU_BIN, -1 }, //merge_idx
    { FL_BIN, -1 }, //inter_pred_idc
    { TU_BIN, -1 }, //ref_idx_l0
    { TU_BIN, -1 }, //ref_idx_l1
    { FL_BIN, 1 }, //abs_mvd_greater0_flag
    { FL_BIN, 1 }, //abs_mvd_greater1_flag]
    { EG_BIN, 1 }, //abs_mvd_minus2
    { FL_BIN, 1 }, //mvd_sign_flag
    { FL_BIN, 1 }, //mvp_l0_flag
    { FL_BIN, 1 }, //mvp_l1_flag
    { FL_BIN, 1 }, //no_residual_data_flag
    { FL_BIN, 1 }, //split_transform_flag
    { FL_BIN, 1 }, //cbf_luma
    { FL_BIN, 1 }, //cbf_cb, cbf_cr
    { FL_BIN, 1 }, //transform_skip_flag[][][0]
    { FL_BIN, 1 }, //transform_skip_flag[][][1|2]
    { TU_BIN, -1 }, //last_significant_coeff_x_prefix
    { TU_BIN, -1 }, //last_significant_coeff_y_prefix
    { FL_BIN, -1 }, //last_significant_coeff_x_suffix
    { FL_BIN, -1 }, //last_significant_coeff_y_suffix
    { FL_BIN, 1 }, //significant_coeff_group_flag
    { FL_BIN, 1 }, //significant_coeff_flag
    { FL_BIN, 1 }, //coeff_abs_level_greater1_flag
    { FL_BIN, 1 }, //coeff_abs_level_greater2_flag
    { COEFF_ABS_LEVEL_REMAINING_BIN }, //coeff_abs_level_remaining
    { FL_BIN, 1 } //coeff_sign_flag

};

/**
 * maxBinIdxCtx from Table 9-63, indexed by SyntaxElement.
 */
static const uint8_t max_bin_idx_ctxs[][3] =
{
    { 0, 0, 0 }, //sao_merge_left_flag
    { 0, 0, 0 }, //sao_merge_up_flag
    { 1, 1, 1 }, //sao_type_idx
    { -1, -1, -1 }, //sao_band_position
    { -1, -1, -1 }, //sao_offset_sign
    { 1, 1, 1 }, //sao_offset
    { 0, 0, 0 }, //alf_cu_flag
    { 0, 0, 0 }, //end_of_slice_flag
    { 0, 0, 0 }, //split_coding_unit_flag
    { 0, 0, 0 }, //cu_transquant_bypass_flag
    { 0, 0, 0 }, //skip_flag
    { 2, 2, 2 }, //cu_qp_delta
    { -1, 0, 0 }, //pred_mode
    { 0, 3, 3 }, //part_mode
    { 0, 0, 0 }, //pcm_flag
    { 0, 0, 0 }, //prev_intra_luma_pred_flag
    { -1, -1, -1 }, // mpm_idx
    { -1, -1, -1 }, //rem_intra_luma_pred_mode
    { 1, 1, 1 }, //intra_chroma_pred_mode
    { -1, 0, 0 }, //merge_flag
    { -1, 0, 0 }, //merge_idx
    { -1, 0, 0 }, //inter_pred_idc
    { -1, 2, 2 }, //ref_idx_l0
    { -1, 2, 2 }, //ref_idx_l1
    { -1, 0, 0 }, //abs_mvd_greater0_flag
    { -1, 0, 0 }, //abs_mvd_greater1_flag
    { -1, -1, -1 }, //abs_mvd_minus2
    { -1, -1, -1 }, //mvd_sign_flag
    { -1, 0, 0 }, //mvp_l0_flag
    { -1, 0, 0 }, //mvp_l1_flag
    { -1, 0, 0 }, //no_residual_data_flag
    { 0, 0, 0 }, //split_transform_flag
    { 0, 0, 0 }, //cbf_luma
    { 0, 0, 0 }, //cbf_cb, cbf_cr
    { 0, 0, 0 }, //transform_skip_flag[][][0]
    { 0, 0, 0 }, //transform_skip_flag[][][1|2]
    { 8, 8, 8 }, //last_significant_coeff_x_prefix
    { 8, 8, 8 }, //last_significant_coeff_y_prefix
    { -1, -1, -1 }, //last_significant_coeff_x_suffix
    { -1, -1, -1 }, //last_significant_coeff_y_suffix
    { 0, 0, 0 }, //significant_coeff_group_flag
    { 0, 0, 0 }, //significant_coeff_flag
    { 0, 0, 0 }, //coeff_abs_level_greater1_flag
    { 0, 0, 0 }, //coeff_abs_level_greater2_flag
    { -1, -1, -1 }, //coeff_abs_level_remaining
    { -1, -1, -1 } //coeff_sign_flag
};

/**
 * CtxIdxOffset from Table 9-63, indexed by SyntaxElement.
 */
static const int8_t ctx_idx_offsets[][3] =
{
    { 0, 3, 6 }, //sao_merge_left_flag
    { 0, 1, 2 }, //sao_merge_up_flag
    { 0, 2, 4 }, //sao_type_idx
    { -1, -1, -1 }, //sao_band_position
    { -1, -1, -1 }, //sao_offset_sign
    { 0, 2, 4 }, //sao_offset
    { 0, 1, 2 }, //alf_cu_flag
    { 0, 0, 0 }, //end_of_slice_flag
    { 0, 3, 6 }, //split_coding_unit_flag
    { 0, 1, 2 }, //cu_transquant_bypass_flag
    { -1, 0, 3 }, //skip_flag
    { 0, 3, 6 }, //cu_qp_delta
    { -1, 0, 1 }, //pred_mode
    { 2, 3, 6 }, //part_mode
    { 0, 0, 0 }, //pcm_flag
    { 0, 1, 2 }, //prev_intra_luma_pred_flag
    { -1, -1, -1 }, //mpm_idx
    { -1, -1, -1 }, //rem_intra_luma_pred_mode
    { 0, 2, 4 }, //intra_chroma_pred_mode
    { -1, 0, 1 }, //merge_flag
    { -1, 0, 1 }, //merge_idx
    { -1, 0, 0 }, //inter_pred_idc
    { -1, 0, 3 }, //ref_idx_l0
    { -1, 0, 3 }, //ref_idx_l1
    { -1, 0, 1 }, //abs_mvd_greater0_flag
    { -1, 2, 3 }, //abs_mvd_greater1_flag
    { -1, -1, -1 }, //abs_mvd_minus2
    { -1, -1, -1 }, //mvd_sign_flag
    { -1, 0, 1 }, //mvp_l0_flag
    { -1, 0, 1 }, //mvp_l1_flag
    { -1, 0, 1 }, //no_residual_data_flag
    { 0, 4, 8 }, //split_transform_flag
    { 0, 2, 4 }, //cbf_luma
    { 0, 3, 6 }, //cbf_cb, cbf_cr
    { 0, 1, 2 }, //transform_skip_flag[][][0]
    { 3, 4, 5 }, //transform_skip_flag[][][1|2]
    { 0, 18, 36 }, //last_significant_coeff_x_prefix
    { 0, 18, 36 }, //last_significant_coeff_y_prefix
    { -1, -1, -1 }, //last_significant_coeff_x_suffix
    { -1, -1, -1 }, //last_significant_coeff_y_suffix
    { 0, 4, 8 }, //significant_coeff_group_flag
    { 0, 48, 96 }, //significant_coeff_flag
    { 0, 24, 48 }, //coeff_abs_level_greater1_flag
    { 0, 6, 12 }, //coeff_abs_level_greater2_flag
    { -1, -1, -1 }, //coeff_abs_level_remaining
    { -1, -1, -1 } //coeff_sign_flag
};

/**
 * ctxIdxInc from Table 9-71, indexed by SyntaxElement.
 */
static const int8_t ctx_idx_incs[][5] =
{
    { -1 }, //sao_merge_left_flag
    { 0 }, //sao_merge_up_flag
    { 0, 1, 1, 1, 1 }, //sao_type_idx
    { }, //sao_band_position
    { }, //sao_offset_sign
    { 0, 1, 1, 1, 1 }, //sao_offset
    { 0 }, //alf_cu_flag
    { }, //end_of_slice_flag
    { -1 }, //split_coding_unit_flag
    { 0 }, //cu_transquant_bypass_flag
    { -1 }, //skip_flag
    { 0 }, //cu_qp_delta
    { 0 }, //pred_mode
    { 0, 1, 2 }, //part_mode
    { }, //pcm_flag
    { 0 }, //prev_intra_luma_pred_flag
    { }, //mpm_idx
    { }, //rem_intra_luma_pred_mode
    { 0, 1 }, //intra_chroma_pred_mode
    { 0 }, //merge_flag
    { 0 }, //merge_idx
    { }, //inter_pred_idc
    { 0, 1, 2, 2, 2 }, //ref_idx_l0
    { 0, 1, 2, 2, 2 }, //ref_idx_l1
    { 0 }, //abs_mvd_greater0_flag
    { 0 }, //abs_mvd_greater1_flag
    { }, //abs_mvd_minus2
    { }, //mvd_sign_flag
    { 0 }, //mvp_l0_flag
    { 0 }, //mvp_l1_flag
    { 0 }, //no_residual_data_flag
    { -1 }, //split_transform_flag
    { -1 }, //cbf_luma
    { -1 }, //cbf_cb, cbf_cr
    { 0 }, //transform_skip_flag[][][0]
    { 0 }, //transform_skip_flag[][][1|2]
    { -1, -1, -1, -1, -1 }, //last_significant_coeff_x_prefix
    { -1, -1, -1, -1, -1 }, //last_significant_coeff_y_prefix
    { }, //last_significant_coeff_x_suffix
    { }, //last_significant_coeff_y_suffix
    { -1 }, //significant_coeff_group_flag
    { -1 }, //significant_coeff_flag
    { -1 }, //coeff_abs_level_greater1_flag
    { -1 }, //coeff_abs_level_greater2_flag
    { }, //coeff_abs_level_remaining
    { }  //coeff_sign_flag
};

/**
 * Offset to ctxIdx 0 in init_values and states, indexed by SyntaxElement.
 */
static const int elem_offset[] =
{
    0, //sao_merge_left_flag
    9, //sao_merge_up_flag
    12, //sao_type_idx
    -1, //sao_band_position
    -1, //sao_offset_sign
    18, //sao_offset
    24, //alf_cu_flag
    -1, //end_of_slice_flag
    27, //split_coding_unit_flag
    36, //cu_transquant_bypass_flag
    39, //skip_flag
    45, //cu_qp_delta
    54, //pred_mode
    54, //part_mode
    -1, //pcm_flag
    63, //prev_intra_luma_pred_mode
    -1, //mpm_idx
    -1, //rem_intra_luma_pred_mode
    66, //intra_chroma_pred_mode
    72, //merge_flag
    74, //merge_idx
    76, //inter_pred_idc
    80, //ref_idx_l0
    80, //ref_idx_l1
    86, //abs_mvd_greater0_flag
    86, //abs_mvd_greater1_flag
    -1, //abs_mvd_minus2
    -1, //mvd_sign_flag
    90, //mvp_l0_flag
    90, //mvp_l1_flag
    92, //no_residual_data_flag
    94, //split_transform_flag
    106, //cbf_luma
    112, //cbf_cb, cbf_cr
    121, //transform_skip_flag[][][0]
    121, //transform_skip_flag[][][1|2]
    127, //last_significant_coeff_x_prefix
    127, //last_significant_coeff_y_prefix
    -1, //last_significant_coeff_x_suffix
    -1, //last_significant_coeff_y_suffix
    181, //significant_coeff_group_flag
    193, //significant_coeff_flag
    337, //coeff_abs_level_greater1_flag
    409, //coeff_abs_level_greater2_flag
    -1, //coeff_abs_level_remaining
    -1, //coeff_sign_flag
};

#define CTX_IDX_COUNT 432

/**
 * initValue from Tables 9-38 to 9-65, indexed by ctx_idx for each SyntaxElement
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
    143, 140, 185, 140, 200, 140, //sao_offset
    153, 153, 153, //alf_cu_flag
    139, 141, 157, 107, 139, 126, 107, 139, 126, //split_coding_unit_flag
    109, 102, 102, //cu_transquant_bypass_flag
    197, 185, 201, 197, 185, 201, //skip_flag
    154, 154, 154, 154, 154, 154, 154, 154, 154, //cu_qp_delta
    149, 134, 184, 154, 139, 154, 154, 139, 154, //pred_mode, part_mode
    184, 154, 183, //prev_intra_luma_pred_mode
    63, 139, 152, 139, 152, 139, //intra_chroma_pred_mode
    110, 154, //merge_flag
    122, 137, //merge_idx
    95, 79, 63, 31, //inter_pred_idc
    153, 153, 139, 153, 153, 168, //ref_idx_l0, ref_idx_l1
    140, 198, 169, 198, //abs_mvd_greater0_flag, abs_mvd_greater1_flag
    168, 168, //mvp_l0_flag, mvp_l1_flag
    79, 79, //no_residual_data_flag
    154, 224, 167, 122, 154, 124, 138, 94, 154, 153, 138, 138, //split_transform_flag
    111, 141, 153, 111, 153, 111, //cbf_luma
    94, 138, 182, 149, 107, 167, 149, 92, 167, //cbf_cb, cbf_cr
    139, 139, 139, 139, 139, 139, //transform_skip_flag

    //last_significant_coeff_x_prefix, last_significant_coeff_y_prefix
    110, 110, 124, 110, 140, 111, 125, 111, 127, 111, 111, 156, 127, 127,
    111, 108, 123,  63, 125, 110,  94, 110, 125, 110, 125, 111, 111, 110,
    139, 111, 111, 111, 125, 108, 123, 108, 125, 110, 124, 110, 125, 110,
    125, 111, 111, 110, 139, 111, 111, 111, 125, 108, 123, 93,

    91, 171, 134, 141, 121, 140, 61, 154, 121, 140, 61, 154, //significant_coeff_group_flag

    //significant_coeff_flag
    141, 111, 125, 110, 110, 94, 124, 108, 124, 125, 139, 124, 63, 139, 168,
    138, 107, 123, 92, 111, 141, 107, 125, 141, 179, 153, 125, 140, 139, 182,
    123, 47, 153, 182, 137, 149, 192, 152, 224, 136, 31, 136, 74, 140, 141, 136,
    139, 111, 170, 154, 139, 153, 139, 123, 123, 63, 153, 168, 153, 152, 92,
    152, 152, 137, 122, 92, 61, 155, 185, 166, 183, 140, 136, 153, 154, 155,
    153, 123, 63, 61, 167, 153, 167, 136, 149, 107, 136, 121, 122, 91, 149, 170,
    185, 151, 183, 140, 170, 154, 139, 153, 139, 123, 123, 63, 124, 139, 153,
    152, 92, 152, 152, 137, 137, 92, 61, 170, 185, 166, 183, 140, 136, 153, 154,
    155, 153, 138, 107, 61, 167, 153, 167, 136, 121, 122, 136, 121, 122, 91,
    149, 170, 170, 151, 183, 140,

    //coeff_abs_level_greater1_flag
    140, 92, 137, 138, 140, 152, 138, 139, 153, 74, 149, 92, 139, 107, 122, 152,
    140, 179, 166, 182, 140, 227, 122, 197, 154, 196, 196, 167, 154, 152, 167,
    182, 182, 134, 149, 136, 153, 121, 136, 137, 169, 194, 166, 167, 154, 167,
    137, 182, 154, 196, 167, 167, 154, 152, 167, 182, 182, 134, 149, 136, 153,
    121, 136, 122, 169, 208, 166, 167, 154, 152, 167, 182,

    //coeff_abs_level_greater2_flag
    138, 153, 136, 167, 152, 152, 107, 167, 91, 122, 107, 167, 107, 167, 91,
    107, 107, 167
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

static void renormalization(HEVCContext *s)
{
    HEVCCabacContext *cc = &s->cc;
    GetBitContext *gb = &s->gb;

    while (cc->range < 256) {
        cc->range <<= 1;
        cc->offset <<= 1;
        cc->offset |= get_bits1(gb);
    }
}

/**
 * 9.2.3.2
 */
static int decode_bin(HEVCContext *s, int bin_idx)
{
    HEVCCabacContext *cc = &s->cc;
    GetBitContext *gb = &s->gb;

    int ctx_idx, mps, pstate, lpsrange, bin_val;
    uint8_t *state;

    av_log(s->avctx, AV_LOG_DEBUG, "cc->elem: %d, ", cc->elem);

    if (cc->bypass_flag) {
        cc->offset <<= 1;
        cc->offset |= get_bits1(gb);
        if (cc->offset >= cc->range) {
            cc->offset -= cc->range;
            bin_val = 1;
        } else {
            bin_val = 0;
        }
        av_log(s->avctx, AV_LOG_DEBUG, "bypass bin_val: %d\n", bin_val);
        return bin_val;
    }

    if (cc->elem == END_OF_SLICE_FLAG || cc->elem == PCM_FLAG) {
        cc->range -= 2;
        if (cc->offset >= cc->range) {
            bin_val = 1;
        } else {
            bin_val = 0;
            renormalization(s);
        }
        av_log(s->avctx, AV_LOG_DEBUG, "cc->range: %d, cc->offset: %d, bin_val: %d\n",
           cc->range, cc->offset, bin_val);
        return bin_val;
    }

    ctx_idx = derive_ctx_idx(s, bin_idx);
    state = cc->state[ctx_idx];

    mps = state[0];
    pstate = state[1];

    lpsrange = lps_range[pstate][(cc->range >> 6) & 3];
    bin_val = 0;

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

    renormalization(s);

    av_log(s->avctx, AV_LOG_DEBUG, "cc->range: %d, cc->offset: %d, bin_val: %d\n",
           cc->range, cc->offset, bin_val);
    return bin_val;
}

/**
 * 9.2.2.1
 */
static int u_binarization(HEVCContext *s, int unused)
{
    int i = 0;

    while (decode_bin(s, i++) == 1);

    return i - 1;
}

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
 * 9.2.2.2
 */
static int tu_binarization(HEVCContext *s, int cMax)
{
    int i = 0;

    for (i = 0; i < cMax && decode_bin(s, i); i++);

    return i;
}

/**
 * 9.2.2.6
 */
static int cu_qp_delta_binarization(HEVCContext *s, int unused)
{
    av_log(s->avctx, AV_LOG_ERROR, "TODO: cu_qp_delta_binarization\n");
    return 0;
}

/**
 * 9.2.2.7
 */
static int part_mode_binarization(HEVCContext *s, int unused)
{
    av_log(s->avctx, AV_LOG_ERROR, "TODO: part_mode_binarization\n");
    return 0;
}

/**
 * 9.2.2.9
 */
static int intra_chroma_pred_mode_binarization(HEVCContext *s, int unused)
{
    int cond = s->sps->chroma_pred_from_luma_enabled_flag;
    int ret = 0;
    int i = 0;

    if (decode_bin(s, i++) == 0) // value 0
        return cond ? 5 : 4;
    if (cond && decode_bin(s, i++) == 0) // value 10
        return 4;
    // values 11XX if cond and 1XX otherwise
#ifdef REFERENCE_ENCODER_QUIRKS
    s->cc.bypass_flag = 1;
#endif
    ret = decode_bin(s, i++) << 1;
    ret |= decode_bin(s, i++);
    return ret;
}

/**
 * 9.2.2.8
 */
static int coeff_abs_level_remaining_binarization(HEVCContext *s, int unused)
{
    av_log(s->avctx, AV_LOG_ERROR,
           "TODO: coeff_abs_level_remaining_binarization\n");
    return 0;
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

//TODO: replace this function by one function for each syntax element
int ff_hevc_cabac_decode(HEVCContext *s, enum SyntaxElement elem)
{
    HEVCCabacContext *cc = &s->cc;

    int initialisation_type = 2 - s->sh.slice_type;

    if (s->sh.cabac_init_flag && s->sh.slice_type != I_SLICE)
        initialisation_type = 2;
    cc->elem = elem;
    cc->state = states + elem_offset[elem];

    // 9.2.2
    cc->max_bin_idx_ctx = max_bin_idx_ctxs[elem][initialisation_type];
    cc->ctx_idx_offset = ctx_idx_offsets[elem][initialisation_type];
    cc->ctx_idx_inc = ctx_idx_incs[elem];
    cc->bypass_flag = (cc->ctx_idx_offset == -1);

    if (binarization[elem][1] == -1) {
        av_log(s->avctx, AV_LOG_ERROR, "TODO: binarization argument for elem %d\n",
               cc->elem);
    }

    return binarization_funcs[binarization[elem][0]](s, binarization[elem][1]);
}

int ff_hevc_split_transform_flag_decode(HEVCContext *s, int trafo_depth)
{
    HEVCCabacContext *cc = &s->cc;
    int initialisation_type = 2 - s->sh.slice_type;
    const int8_t ctx_idx_inc[5] = { s->ct.depth + trafo_depth };

    if (s->sh.cabac_init_flag && s->sh.slice_type != I_SLICE)
        initialisation_type = 2;
    cc->elem = SPLIT_TRANSFORM_FLAG;
    cc->state = states + elem_offset[cc->elem];

    cc->max_bin_idx_ctx = 0;
    cc->ctx_idx_offset = 4 * initialisation_type;
    cc->ctx_idx_inc = ctx_idx_inc;

    return fl_binarization(s, 1);
}

int ff_hevc_cbf_cb_cr_decode(HEVCContext *s, int trafo_depth)
{
    HEVCCabacContext *cc = &s->cc;
    int initialisation_type = 2 - s->sh.slice_type;
    const int8_t ctx_idx_inc[5] = { trafo_depth };

    if (s->sh.cabac_init_flag && s->sh.slice_type != I_SLICE)
        initialisation_type = 2;
    cc->elem = CBF_CB_CR;
    cc->state = states + elem_offset[cc->elem];

    cc->max_bin_idx_ctx = 0;
    cc->ctx_idx_offset = 3 * initialisation_type;
    cc->ctx_idx_inc = ctx_idx_inc;

    return fl_binarization(s, 1);
}

int ff_hevc_cbf_luma_decode(HEVCContext *s, int trafo_depth, int log2_trafo_size,
                            int log2_max_trafo_size)
{
    HEVCCabacContext *cc = &s->cc;
    int initialisation_type = 2 - s->sh.slice_type;
    const int8_t ctx_idx_inc[5] = {
        (trafo_depth == 0) || (log2_trafo_size == log2_max_trafo_size)
    };

    if (s->sh.cabac_init_flag && s->sh.slice_type != I_SLICE)
        initialisation_type = 2;
    cc->elem = CBF_LUMA;
    cc->state = states + elem_offset[cc->elem];

    // 9.2.2
    cc->max_bin_idx_ctx = 0;
    cc->ctx_idx_offset = 2 * initialisation_type;
    cc->ctx_idx_inc = ctx_idx_inc;

    return fl_binarization(s, 1);
}
