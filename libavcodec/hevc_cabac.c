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
 * Offset to ctxIdx 0 in init_values and states, indexed by SyntaxElement.
 */
static const int elem_offset[] =
{
    0, //sao_merge_left_flag and sao_merge_up_flag
    3, //sao_type_idx
    -1, //sao_eo_class
    -1, //sao_band_position
    -1, //sao_offset_abs
    -1, //sao_offset_sign
    6, //alf_cu_flag
    -1, //end_of_slice_flag
    9, //split_coding_unit_flag
    18, //cu_transquant_bypass_flag
    21, //skip_flag
    27, //cu_qp_delta
    36, //pred_mode
    36, //part_mode
    -1, //pcm_flag
    45, //prev_intra_luma_pred_mode
    -1, //mpm_idx
    -1, //rem_intra_luma_pred_mode
    48, //intra_chroma_pred_mode
    54, //merge_flag
    56, //merge_idx
    58, //inter_pred_idc
    62, //ref_idx_l0
    62, //ref_idx_l1
    68, //abs_mvd_greater0_flag
    68, //abs_mvd_greater1_flag
    -1, //abs_mvd_minus2
    -1, //mvd_sign_flag
    72, //mvp_l0_flag
    72, //mvp_l1_flag
    74, //no_residual_data_flag
    76, //split_transform_flag
    85, //cbf_luma
    91, //cbf_cb, cbf_cr
    100, //transform_skip_flag[][][0]
    100, //transform_skip_flag[][][1|2]
    106, //last_significant_coeff_x_prefix
    160, //last_significant_coeff_y_prefix
    -1, //last_significant_coeff_x_suffix
    -1, //last_significant_coeff_y_suffix
    214, //significant_coeff_group_flag
    226, //significant_coeff_flag
    352, //coeff_abs_level_greater1_flag
    424, //coeff_abs_level_greater2_flag
    -1, //coeff_abs_level_remaining
    -1, //coeff_sign_flag
};

/**
 * initValue from Tables 9-38 to 9-65, indexed by ctx_idx for each SyntaxElement
 * value.
 */
static const uint8_t init_values[] =
{
    153, 153, 153, //sao_merge_left_flag and sao_merge_up_flag
    160, 185, 200, //sao_type_idx
    153, 153, 153, //alf_cu_flag
    139, 141, 157, 107, 139, 126, 107, 139, 126, //split_coding_unit_flag
    154, 154, 154, //cu_transquant_bypass_flag
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
    224, 167, 122, 124, 138, 94, 153, 138, 138, //split_transform_flag
    111, 141, 153, 111, 153, 111, //cbf_luma
    94, 138, 182, 149, 107, 167, 149, 92, 167, //cbf_cb, cbf_cr
    139, 139, 139, 139, 139, 139, //transform_skip_flag

    //last_significant_coeff_x_prefix
    110, 110, 124, 125, 140, 153, 125, 127, 140, 109, 111, 143, 127, 111,
    79,  108, 123,  63, 125, 110,  94, 110,  95,  79, 125, 111, 110,  78,
    110, 111, 111,  95,  94, 108, 123, 108, 125, 110, 124, 110,  95,  94,
    125, 111, 111,  79, 125, 126, 111, 111,  79, 108, 123,  93,

    //last_significant_coeff_y_prefix
    110, 110, 124, 125, 140, 153, 125, 127, 140, 109, 111, 143, 127, 111,
    79,  108, 123,  63, 125, 110,  94, 110,  95,  79, 125, 111, 110,  78,
    110, 111, 111,  95,  94, 108, 123, 108, 125, 110, 124, 110,  95,  94,
    125, 111, 111,  79, 125, 126, 111, 111,  79, 108, 123,  93,

    91, 171, 134, 141, 121, 140, 61, 154, 121, 140, 61, 154, //significant_coeff_group_flag

    //significant_coeff_flag
    111, 111, 125, 110, 110, 94, 124, 108, 124, 107, 125, 141, 179, 153, 125,
    107, 125, 141, 179, 153, 125, 107, 125, 141, 179, 153, 125, 140, 139, 182,
    182, 152, 136, 152, 136, 153, 136, 139, 111, 136, 139, 111, 155, 154, 139,
    153, 139, 123, 123, 63, 153, 166, 183, 140, 136, 153, 154, 166, 183, 140,
    136, 153, 154, 166, 183, 140, 136, 153, 154, 170, 153, 123, 123, 107, 121,
    107, 121, 167, 151, 183, 140, 151, 183, 140, 170, 154, 139, 153, 139, 123,
    123, 63, 124, 166, 183, 140, 136, 153, 154, 166, 183, 140, 136, 153, 154,
    166, 183, 140, 136, 153, 154, 170, 153, 138, 138, 122, 121, 122, 121, 167,
    151, 183, 140, 151, 183, 140,

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
static uint8_t states[sizeof(init_values)][2];

static int derive_ctx_idx(HEVCContext *s, int bin_idx)
{
    HEVCCabacContext *cc = &s->cc;

    int ctx_idx_inc = cc->ctx_idx_inc[FFMIN(bin_idx, cc->max_bin_idx_ctx)];

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

static int cnt = 0;

static int decode_bin(HEVCContext *s, int bin_idx)
{
    HEVCCabacContext *cc = &s->cc;
    GetBitContext *gb = &s->gb;

    int ctx_idx, mps, pstate, lpsrange, bin_val;
    uint8_t *state;

    av_log(s->avctx, AV_LOG_DEBUG, "cc->elem: %d, ", cc->elem);

    // Bypass decoding
    if (cc->ctx_idx_offset == -1) {
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

    ctx_idx = derive_ctx_idx(s, bin_idx);
    state = cc->state[ctx_idx];

    mps = state[0];
    pstate = state[1];

    lpsrange = lps_range[pstate][(cc->range >> 6) & 3];
    bin_val = 0;

    av_log(s->avctx, AV_LOG_DEBUG,
           "ctx_idx: %d, %d#pstate: %d, mps: %d\n", ctx_idx, cnt++, pstate, mps);

    av_log(s->avctx, AV_LOG_DEBUG,
           "old cc->range: %d, cc->offset: %d, lpsrange: %d\n", cc->range, cc->offset, lpsrange);

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

static int bypass_decode_bin(HEVCContext *s)
{
    HEVCCabacContext *cc = &s->cc;
    int bin_val;

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

static int u_binarization(HEVCContext *s)
{
    int i = 0;

    while (decode_bin(s, i++) == 1);

    return i - 1;
}

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

static int fl_binarization(HEVCContext *s, int c_max)
{
    int value = 0;
    int length = av_ceil_log2_c(c_max + 1);

    for (int i = 0; i < length; i++)
        value = (value << 1) | decode_bin(s, i);

    return value;
}

static int tu_binarization(HEVCContext *s, int c_max)
{
    int i = 0;

    for (i = 0; i < c_max && decode_bin(s, i); i++);

    return i;
}

void ff_hevc_cabac_init(HEVCContext *s)
{
    HEVCCabacContext *cc = &s->cc;
    GetBitContext *gb = &s->gb;

    cc->range = 510;
    cc->offset = get_bits(gb, 9);
    av_log(s->avctx, AV_LOG_DEBUG, "cc->offset: %d\n", cc->offset);

    cc->init_type = 2 - s->sh.slice_type;
    if (s->sh.cabac_init_flag && s->sh.slice_type != I_SLICE)
        cc->init_type ^= 3;

    for (int i = 0; i < sizeof(init_values); i++) {
        int init_value = init_values[i];
        int m = (init_value >> 4)*5 - 45;
        int n = ((init_value & 15) << 3) - 16;
        int pre_ctx_state = av_clip_c(((m * av_clip_c(s->sh.slice_qp, 0, 51)) >> 4) + n,
                                    1, 126);
        states[i][0] = (pre_ctx_state <= 63) ? 0 : 1; //mps
        states[i][1] = states[i][0] ? (pre_ctx_state - 64) : (63 - pre_ctx_state); //stateIdx
    }
}

int ff_hevc_sao_merge_left_up_flag_decode(HEVCContext *s)
{
    HEVCCabacContext *cc = &s->cc;
    const int8_t ctx_idx_inc[1] = { 0 };

    cc->elem = SAO_MERGE_LEFT_UP_FLAG;
    cc->state = states + elem_offset[cc->elem];

    cc->max_bin_idx_ctx = 0;
    cc->ctx_idx_offset = cc->init_type;
    cc->ctx_idx_inc = ctx_idx_inc;

    return fl_binarization(s, 1);
}

int ff_hevc_sao_type_idx_decode(HEVCContext *s)
{
    HEVCCabacContext *cc = &s->cc;
    const int8_t ctx_idx_inc[1] = { 0 };
    int i = 0;

    cc->elem = SAO_TYPE_IDX;
    cc->state = states + elem_offset[cc->elem];

    cc->max_bin_idx_ctx = 0;
    cc->ctx_idx_offset = cc->init_type;
    cc->ctx_idx_inc = ctx_idx_inc;

    if (decode_bin(s, i++) == 0)
        return 0;

    cc->ctx_idx_offset = -1;
    if (decode_bin(s, i++) == 0)
        return SAO_BAND;
    return SAO_EDGE;
}

int ff_hevc_sao_band_position_decode(HEVCContext *s)
{
    HEVCCabacContext *cc = &s->cc;

    cc->elem = SAO_BAND_POSITION;

    cc->ctx_idx_offset = -1;

    return fl_binarization(s, 31);
}

int ff_hevc_sao_offset_abs_decode(HEVCContext *s, int bit_depth)
{
    HEVCCabacContext *cc = &s->cc;

    cc->elem = SAO_OFFSET_ABS;

    cc->ctx_idx_offset = -1;

    return tu_binarization(s, (1 << (FFMIN(bit_depth, 10) - 5)) - 1);
}

int ff_hevc_sao_offset_sign_decode(HEVCContext *s)
{
    HEVCCabacContext *cc = &s->cc;

    cc->elem = SAO_OFFSET_SIGN;

    cc->ctx_idx_offset = -1;

    return fl_binarization(s, 1);
}

int ff_hevc_sao_eo_class_decode(HEVCContext *s)
{
    HEVCCabacContext *cc = &s->cc;

    cc->elem = SAO_EO_CLASS;

    cc->ctx_idx_offset = -1;

    return fl_binarization(s, 2);
}

int ff_hevc_end_of_slice_flag_decode(HEVCContext *s)
{
    return bypass_decode_bin(s);
}

int ff_hevc_cu_transquant_bypass_flag_decode(HEVCContext *s)
{
    HEVCCabacContext *cc = &s->cc;
    const int8_t ctx_idx_inc[1] = { 0 };

    cc->elem = CU_TRANSQUANT_BYPASS_FLAG;
    cc->state = states + elem_offset[cc->elem];

    cc->max_bin_idx_ctx = 0;
    cc->ctx_idx_offset = cc->init_type;
    cc->ctx_idx_inc = ctx_idx_inc;

    return fl_binarization(s, 1);
}

int ff_hevc_split_coding_unit_flag_decode(HEVCContext *s, int ct_depth, int x0, int y0)
{
    HEVCCabacContext *cc = &s->cc;
    int8_t ctx_idx_inc[1] = { 0 };

    int depth_left = 0;
    int depth_top = 0;

    if (x0 > 0)
        depth_left = s->cu.left_ct_depth[y0 >> s->sps->log2_min_coding_block_size];
    if (y0 > 0)
        depth_top = s->cu.top_ct_depth[x0 >> s->sps->log2_min_coding_block_size];

    cc->elem = SPLIT_CODING_UNIT_FLAG;
    cc->state = states + elem_offset[cc->elem];

    av_log(s->avctx, AV_LOG_DEBUG, "depth cur: %d, left: %d, top: %d\n",
           ct_depth, depth_left, depth_top);

    ctx_idx_inc[0] += (depth_left > ct_depth);
    ctx_idx_inc[0] += (depth_top > ct_depth);

    cc->max_bin_idx_ctx = 0;
    cc->ctx_idx_offset = 3 * cc->init_type;
    cc->ctx_idx_inc = ctx_idx_inc;

    return fl_binarization(s, 1);
}

int ff_hevc_part_mode_decode(HEVCContext *s, int log2_cb_size)
{
    HEVCCabacContext *cc = &s->cc;
    const int8_t ctx_idx_inc[3] = { 0, 1, 2 };
    int i = 0;

    cc->elem = PART_MODE;
    cc->state = states + elem_offset[cc->elem];

    cc->max_bin_idx_ctx = cc->init_type ? 3 : 0;
    cc->ctx_idx_offset = 2 + 2 * cc->init_type;
    cc->ctx_idx_inc = ctx_idx_inc;

    if (decode_bin(s, i++) == 1) // 1
        return PART_2Nx2N;
    if (log2_cb_size == s->sps->log2_min_coding_block_size) {
        if (s->cu.pred_mode == MODE_INTRA) // 0
            return PART_NxN;
        if (decode_bin(s, i++) == 1) // 01
            return PART_2NxN;
        if (log2_cb_size == 3) // 00
            return PART_Nx2N;
        if (decode_bin(s, i++) == 1) // 001
            return PART_Nx2N;
        return PART_NxN; // 000
    }
    if (decode_bin(s, i++) == 1) { // 01X, 01XX
        if (decode_bin(s, i++) == 1) // 011
            return PART_2NxN;
        if (decode_bin(s, i++) == 1) // 0101
            return PART_2NxnD;
        return PART_2NxnU; // 0100
    }
    if (decode_bin(s, i++) == 1) // 001
        return PART_Nx2N;

    // Last bin is bypass-coded
    cc->ctx_idx_offset = -1;
    if (decode_bin(s, i++) == 1) // 0001
        return PART_nRx2N;
    return  PART_nLx2N; // 0000
}

int ff_hevc_pcm_flag_decode(HEVCContext *s)
{
    return bypass_decode_bin(s);
}

int ff_hevc_prev_intra_luma_pred_flag_decode(HEVCContext *s)
{
    HEVCCabacContext *cc = &s->cc;
    const int8_t ctx_idx_inc[1] = { 0 };

    cc->elem = PREV_INTRA_LUMA_PRED_FLAG;
    cc->state = states + elem_offset[cc->elem];

    cc->max_bin_idx_ctx = 0;
    cc->ctx_idx_offset = cc->init_type;
    cc->ctx_idx_inc = ctx_idx_inc;

    return fl_binarization(s, 1);
}

int ff_hevc_mpm_idx_decode(HEVCContext *s)
{
    HEVCCabacContext *cc = &s->cc;

    cc->elem = MPM_IDX;

    cc->ctx_idx_offset = -1;

    return tu_binarization(s, 2);
}

int ff_hevc_rem_intra_luma_pred_mode_decode(HEVCContext *s)
{
    HEVCCabacContext *cc = &s->cc;

    cc->elem = REM_INTRA_LUMA_PRED_MODE;

    cc->ctx_idx_offset = -1;

    return fl_binarization(s, 31);
}

int ff_hevc_intra_chroma_pred_mode_decode(HEVCContext *s)
{
    HEVCCabacContext *cc = &s->cc;
    const int8_t ctx_idx_inc[2] = { 0, 1 };

    int ret = 0;
    int i = 0;

    cc->elem = INTRA_CHROMA_PRED_MODE;
    cc->state = states + elem_offset[cc->elem];

    cc->max_bin_idx_ctx = 1;
    cc->ctx_idx_offset = 2 * cc->init_type;
    cc->ctx_idx_inc = ctx_idx_inc;

    if (decode_bin(s, i++) == 0)
        return 4;

    // the last two bits are bypass-coded
    s->cc.ctx_idx_offset = -1;
    ret = decode_bin(s, i++) << 1;
    ret |= decode_bin(s, i++);
    return ret;
}

int ff_hevc_split_transform_flag_decode(HEVCContext *s, int log2_trafo_size)
{
    HEVCCabacContext *cc = &s->cc;
    const int8_t ctx_idx_inc[1] = { 5 - log2_trafo_size };

    cc->elem = SPLIT_TRANSFORM_FLAG;
    cc->state = states + elem_offset[cc->elem];

    cc->max_bin_idx_ctx = 0;
    cc->ctx_idx_offset = 3 * cc->init_type;
    cc->ctx_idx_inc = ctx_idx_inc;

    return fl_binarization(s, 1);
}

int ff_hevc_cbf_cb_cr_decode(HEVCContext *s, int trafo_depth)
{
    HEVCCabacContext *cc = &s->cc;
    const int8_t ctx_idx_inc[1] = { trafo_depth };

    cc->elem = CBF_CB_CR;
    cc->state = states + elem_offset[cc->elem];

    cc->max_bin_idx_ctx = 0;
    cc->ctx_idx_offset = 4 * cc->init_type;
    cc->ctx_idx_inc = ctx_idx_inc;

    return fl_binarization(s, 1);
}

int ff_hevc_cbf_luma_decode(HEVCContext *s, int trafo_depth)
{
    HEVCCabacContext *cc = &s->cc;
    const int8_t ctx_idx_inc[1] = { trafo_depth == 0 };

    cc->elem = CBF_LUMA;
    cc->state = states + elem_offset[cc->elem];

    cc->max_bin_idx_ctx = 0;
    cc->ctx_idx_offset = 2 * cc->init_type;
    cc->ctx_idx_inc = ctx_idx_inc;

    return fl_binarization(s, 1);
}

int ff_hevc_transform_skip_flag_decode(HEVCContext *s, int c_idx)
{
    HEVCCabacContext *cc = &s->cc;
    const int8_t ctx_idx_inc[1] = { 0 };

    cc->elem = c_idx ? TRANSFORM_SKIP_FLAG_1_2 : TRANSFORM_SKIP_FLAG_0;
    cc->state = states + elem_offset[cc->elem];

    cc->max_bin_idx_ctx = 0;
    cc->ctx_idx_offset = cc->init_type + ((c_idx != 0) ? 3 : 0);
    cc->ctx_idx_inc = ctx_idx_inc;

    return fl_binarization(s, 1);
}

int ff_hevc_last_significant_coeff_prefix_decode(HEVCContext *s, int c_idx,
                                                 int log2_size, int is_x)
{
    HEVCCabacContext *cc = &s->cc;
    int8_t ctx_idx_inc[9];
    int ctx_offset, ctx_shift;

    cc->elem = is_x ? LAST_SIGNIFICANT_COEFF_X_PREFIX
               : LAST_SIGNIFICANT_COEFF_Y_PREFIX;
    cc->state = states + elem_offset[cc->elem];

    if (c_idx == 0) {
        ctx_offset = 3 * (log2_size - 2)  + ((log2_size - 1) >> 2);
        ctx_shift = (log2_size + 1) >> 2;
    } else {
        ctx_offset = 15;
        ctx_shift = log2_size - 2;
    }
    for (int i = 0; i < 9; i++)
        ctx_idx_inc[i] = (i >> ctx_shift) + ctx_offset;

    cc->max_bin_idx_ctx = 8;
    cc->ctx_idx_offset = 18 * cc->init_type;
    cc->ctx_idx_inc = ctx_idx_inc;

    return tu_binarization(s, (log2_size << 1 ) - 1);
}

int ff_hevc_last_significant_coeff_suffix_decode(HEVCContext *s,
                                                 int last_significant_coeff_prefix,
                                                 int is_x)
{
    HEVCCabacContext *cc = &s->cc;
    int length = (last_significant_coeff_prefix >> 1) - 1;
#ifdef REFERENCE_ENCODER_QUIRKS
    int value = 0;
#endif

    cc->elem = is_x ? LAST_SIGNIFICANT_COEFF_X_SUFFIX
               : LAST_SIGNIFICANT_COEFF_Y_SUFFIX;

    cc->ctx_idx_offset = -1;

#ifdef REFERENCE_ENCODER_QUIRKS
    for (int i = 0; i < length; i++)
        value = (value << 1) | decode_bin(s, i);
    return value;
#else
    return fl_binarization(s, length);
#endif
}

int ff_hevc_significant_coeff_group_flag_decode(HEVCContext *s, int c_idx, int x_cg,
                                                int y_cg, int log2_trafo_width,
                                                int log2_trafo_height, int scan_idx)
{
    HEVCCabacContext *cc = &s->cc;
    int8_t ctx_idx_inc[1];
    int ctx_cg = 0;

    cc->elem = SIGNIFICANT_COEFF_GROUP_FLAG;
    cc->state = states + elem_offset[cc->elem];

    if (x_cg < (1 << (log2_trafo_width - 2)) - 1)
        ctx_cg += s->rc.significant_coeff_group_flag[x_cg + 1][y_cg];
    if (y_cg < (1 << (log2_trafo_height - 2)) - 1)
        ctx_cg += s->rc.significant_coeff_group_flag[x_cg][y_cg + 1];

    ctx_idx_inc[0] = FFMIN(ctx_cg, 1) + (c_idx ? 2 : 0);

    cc->max_bin_idx_ctx = 0;
    cc->ctx_idx_offset = 4 * cc->init_type;
    cc->ctx_idx_inc = ctx_idx_inc;

    return fl_binarization(s, 1);
}

int ff_hevc_significant_coeff_flag_decode(HEVCContext *s, int c_idx, int x_c, int y_c,
                                          int log2_trafo_width, int log2_trafo_height,
                                          int scan_idx)
{
    HEVCCabacContext *cc = &s->cc;
    int8_t ctx_idx_inc[1];
    static const uint8_t ctx_idx_map[] = {
        0, 1, 4, 5, 2, 3, 4, 5, 6, 6, 8, 8, 7, 7, 8, 8
    };
    int sig_ctx;
    int x_cg = x_c >> 2;
    int y_cg = y_c >> 2;

    cc->elem = SIGNIFICANT_COEFF_FLAG;
    cc->state = states + elem_offset[cc->elem];

    if (x_c + y_c == 0) {
        sig_ctx = 0;
    } else if (log2_trafo_width == 2 && log2_trafo_height == 2) {
        sig_ctx = ctx_idx_map[(y_c << 2) + x_c];
    } else {
        int prev_sig = 0;
        int x_off = x_c - (x_cg << 2);
        int y_off = y_c - (y_cg << 2);

        if (x_c < (1 << log2_trafo_width) - 1)
            prev_sig += s->rc.significant_coeff_group_flag[x_cg + 1][y_cg];
        if (y_c < (1 << log2_trafo_height) - 1)
            prev_sig += (s->rc.significant_coeff_group_flag[x_cg][y_cg + 1] << 1);
        av_log(s->avctx, AV_LOG_DEBUG, "prev_sig: %d\n", prev_sig);

        switch (prev_sig) {
        case 0:
            sig_ctx = ((x_off + y_off) == 0) ? 2 : ((x_off + y_off) <= 2) ? 1 : 0;
            break;
        case 1:
            sig_ctx = 2 - FFMIN(y_off, 2);
            break;
        case 2:
            sig_ctx = 2 - FFMIN(x_off, 2);
            break;
        default:
            sig_ctx = 2;
        }

        if (c_idx == 0 && (x_cg > 0 || y_cg > 0)) {
            sig_ctx += 3;
        }
        if (log2_trafo_width == 3 && log2_trafo_height == 3) {
            sig_ctx += (scan_idx == SCAN_DIAG) ? 9 : 15;
        } else {
            sig_ctx += c_idx ? 12 : 21;
        }
    }

    if (c_idx == 0) {
        ctx_idx_inc[0] = sig_ctx;
    } else {
        ctx_idx_inc[0] = sig_ctx + 27;
    }

    cc->max_bin_idx_ctx = 0;
    cc->ctx_idx_offset = 48 * cc->init_type;
    cc->ctx_idx_inc = ctx_idx_inc;

    return fl_binarization(s, 1);
}

static int ctx_set = 0;

int ff_hevc_coeff_abs_level_greater1_flag_decode(HEVCContext *s, int c_idx,
                                                 int i, int n,
                                                 int first_elem,
                                                 int first_subset)
{
    HEVCCabacContext *cc = &s->cc;
    int8_t ctx_idx_inc[1];

    static int greater1_ctx = 0;
    static int last_coeff_abs_level_greater1_flag = 0;

    cc->elem = COEFF_ABS_LEVEL_GREATER1_FLAG;
    cc->state = states + elem_offset[cc->elem];

    if (first_elem) {
        ctx_set = (i > 0 && c_idx == 0) ? 2 : 0;

        if (!first_subset && greater1_ctx == 0)
            ctx_set++;
        greater1_ctx = 1;
    }

    ctx_idx_inc[0] = (ctx_set * 4) + greater1_ctx;
    if (c_idx > 0)
        ctx_idx_inc[0] += 16;

    cc->max_bin_idx_ctx = 0;
    cc->ctx_idx_offset = 24 * cc->init_type;
    cc->ctx_idx_inc = ctx_idx_inc;

    last_coeff_abs_level_greater1_flag = fl_binarization(s, 1);

    if (last_coeff_abs_level_greater1_flag) {
        greater1_ctx = 0;
    } else if (greater1_ctx > 0 && greater1_ctx < 3) {
        greater1_ctx++;
    }

    return last_coeff_abs_level_greater1_flag;
}

int ff_hevc_coeff_abs_level_greater2_flag_decode(HEVCContext *s, int c_idx,
                                                 int i, int n)
{
    HEVCCabacContext *cc = &s->cc;
    int8_t ctx_idx_inc[1] = { 0 };

    cc->elem = COEFF_ABS_LEVEL_GREATER2_FLAG;
    cc->state = states + elem_offset[cc->elem];

    ctx_idx_inc[0] = ctx_set;
    if (c_idx > 0) {
        ctx_idx_inc[0] += 4;
    }

    cc->max_bin_idx_ctx = 0;
    cc->ctx_idx_offset = 6 * cc->init_type;
    cc->ctx_idx_inc = ctx_idx_inc;

    return fl_binarization(s, 1);
}

int ff_hevc_coeff_abs_level_remaining(HEVCContext *s, int first_elem, int base_level)
{
    HEVCCabacContext *cc = &s->cc;
    static int c_rice_param, last_coeff_abs_level_remaining;
    int prefix = 0;
    int suffix = 0;

    cc->elem = COEFF_ABS_LEVEL_REMAINING;

    cc->ctx_idx_offset = -1;

    if (first_elem) {
        c_rice_param = 0;
        last_coeff_abs_level_remaining = 0;
        av_log(s->avctx, AV_LOG_DEBUG,
           "c_rice_param reset to 0\n");
    }

    prefix = u_binarization(s);
    if (prefix < 3) {
        for (int i = 0; i < c_rice_param; i++)
            suffix = (suffix << 1) | decode_bin(s, i);
        last_coeff_abs_level_remaining = (prefix << c_rice_param) + suffix;
    } else {
        for (int i = 0; i < prefix - 3 + c_rice_param; i++)
            suffix = (suffix << 1) | decode_bin(s, i);
        last_coeff_abs_level_remaining = (((1 << (prefix - 3)) + 3 - 1)
                                          << c_rice_param) + suffix;
    }

    av_log(s->avctx, AV_LOG_DEBUG,
           "coeff_abs_level_remaining c_rice_param: %d\n", c_rice_param);
    av_log(s->avctx, AV_LOG_DEBUG,
           "coeff_abs_level_remaining base_level: %d, prefix: %d, suffix: %d\n",
           base_level, prefix, suffix);
    av_log(s->avctx, AV_LOG_DEBUG,
           "coeff_abs_level_remaining: %d\n",
           last_coeff_abs_level_remaining);

    av_log(s->avctx, AV_LOG_DEBUG, "last_coeff_(%d) > %d\n", base_level + last_coeff_abs_level_remaining, 3*(1<<(c_rice_param)));

    c_rice_param = FFMIN(c_rice_param +
                         ((base_level + last_coeff_abs_level_remaining) >
                          (3 * (1 << c_rice_param))), 4);
    av_log(s->avctx, AV_LOG_DEBUG,
           "new c_rice_param: %d\n", c_rice_param);

    return last_coeff_abs_level_remaining;
}

int ff_hevc_coeff_sign_flag(HEVCContext *s)
{
    HEVCCabacContext *cc = &s->cc;

    cc->elem = COEFF_SIGN_FLAG;
    cc->state = states + elem_offset[cc->elem];

    cc->ctx_idx_offset = -1;

    return fl_binarization(s, 1);
}
