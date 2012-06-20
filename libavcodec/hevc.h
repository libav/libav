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

#ifndef AVCODEC_HEVC_H
#define AVCODEC_HEVC_H

#include "avcodec.h"
#include "get_bits.h"

/**
 * Enable to diverge from the spec when the reference encoder
 * does so.
 */
#define REFERENCE_ENCODER_QUIRKS 1

/**
 * Table 7-3: NAL unit type codes
 */
typedef enum {
    NAL_SLICE = 1,
    NAL_TLA_SLICE = 3,
    NAL_CRA_SLICE = 4,
    NAL_IDR_SLICE = 5,
    NAL_SEI = 6,
    NAL_SPS = 7,
    NAL_PPS = 8,
    NAL_AUD = 9,
    NAL_FILLER_DATA = 12,
    NAL_APS = 14,
    NAL_UNSPECIFIED = 24
} NALUnitType;

typedef struct {
    uint8_t inter_ref_pic_set_prediction_flag;
    int num_negative_pics;
    int num_positive_pics;
} ShortTermRPS;

/**
 * 7.4.2.1
 */
#define MAX_TEMPORAL_LAYERS 8
#define MAX_SPS_COUNT 32
#define MAX_PPS_COUNT 256
#define MAX_SHORT_TERM_RPS_COUNT 64

//TODO: check if this is really the maximum
#define MAX_TRANSFORM_DEPTH 3

/**
 * Not yet specified!
 */
#define MAX_APS_COUNT 256

typedef struct {
    uint8_t profile_idc;
    uint8_t level_idc;

    int chroma_format_idc;
    uint8_t separate_colour_plane_flag;

    uint8_t max_temporal_layers; ///< max_temporal_layers_minus1 + 1

    int pic_width_in_luma_samples;
    int pic_height_in_luma_samples;

    uint8_t pic_cropping_flag;
    struct {
        int left_offset;
        int right_offset;
        int top_offset;
        int bottom_offset;
    } pic_crop;

    int bit_depth_luma; ///< bit_depth_luma_minus8 + 8
    int bit_depth_chroma; ///< bit_depth_chroma_minus8 + 8

    int pcm_enabled_flag;
    struct {
        uint8_t bit_depth_luma; ///< pcm_bit_depth_luma_minus1 + 1
        uint8_t bit_depth_chroma; ///< pcm_bit_depth_chroma_minus1 + 1

        int log2_min_pcm_coding_block_size; ///< log2_min_pcm_coding_block_size_minus3 + 3
        int log2_diff_max_min_pcm_coding_block_size;

        uint8_t loop_filter_disable_flag;
    } pcm;

    uint8_t qpprime_y_zero_transquant_bypass_flag;
    int log2_max_poc_lsb; ///< log2_max_pic_order_cnt_lsb_minus4 + 4

    struct {
        int max_dec_pic_buffering;
        int num_reorder_pics;
        int max_latency_increase;
    } temporal_layer[MAX_TEMPORAL_LAYERS];

    uint8_t restricted_ref_pic_lists_flag;
    uint8_t lists_modification_present_flag;

    int log2_min_coding_block_size; ///< log2_min_coding_block_size_minus3 + 3
    int log2_diff_max_min_coding_block_size;
    int log2_min_transform_block_size; ///< log2_min_transform_block_size_minus2 + 2
    int log2_diff_max_min_transform_block_size;


    int max_transform_hierarchy_depth_inter;
    int max_transform_hierarchy_depth_intra;

    int scaling_list_enable_flag;

    uint8_t chroma_pred_from_luma_enabled_flag;
    uint8_t transform_skip_enabled_flag;
    uint8_t deblocking_filter_in_aps_enabled_flag;

    uint8_t seq_loop_filter_across_slices_enabled_flag;
    uint8_t asymmetric_motion_partitions_enabled_flag;
    uint8_t nsrqt_enabled_flag;
    uint8_t sample_adaptive_offset_enabled_flag;

    uint8_t adaptive_loop_filter_enabled_flag;
    uint8_t alf_coef_in_slice_flag;

    uint8_t temporal_id_nesting_flag;

    uint8_t inter_4x4_enabled_flag;

    int num_short_term_ref_pic_sets;
    ShortTermRPS *short_term_rps_list[MAX_SHORT_TERM_RPS_COUNT];

    uint8_t long_term_ref_pics_present_flag;

#if REFERENCE_ENCODER_QUIRKS
    uint8_t amvp_mode_flag[4];
#endif

    uint8_t tiles_or_entropy_coding_sync_idc;
    int num_tile_columns;
    int num_tile_rows;
    uint8_t uniform_spacing_flag;
    uint8_t loop_filter_across_tiles_enabled_flag;


    // Inferred parameters
    int Log2CtbSize;
    int PicWidthInCtbs;
    int PicHeightInCtbs;
    int pic_width_in_min_cbs;
    int pic_height_in_min_cbs;

    int *column_width; ///< ColumnWidth
    int *row_height; ///< RowHeight
    int *col_bd; ///< ColBd
    int *row_bd; ///< RowBd

    int *ctb_addr_rs_to_ts; ///< CtbAddrRSToTS
    int *ctb_addr_ts_to_rs; ///< CtbAddrTSToRS
    int *tile_id; ///< TileId
    int *min_cb_addr_zs; ///< MinCbAddrZS
} SPS;

typedef struct {
    int sps_id; ///< seq_parameter_set_id

    uint8_t sign_data_hiding_flag;

    uint8_t cabac_init_present_flag;

    int num_ref_idx_l0_default_active; ///< num_ref_idx_l0_default_active_minus1 + 1
    int num_ref_idx_l1_default_active; ///< num_ref_idx_l1_default_active_minus1 + 1
    int pic_init_qp_minus26;

    uint8_t constrained_intra_pred_flag;
    uint8_t enable_temporal_mvp_flag;
    uint8_t slice_granularity;
    int diff_cu_qp_delta_depth;
    int cb_qp_offset;
    int cr_qp_offset;
    uint8_t weighted_pred_flag;
    uint8_t weighted_bipred_idc;
    uint8_t output_flag_present_flag;

    struct {
        uint8_t tile_info_present_flag;

        uint8_t tile_control_present_flag;
        int num_tile_columns; ///< num_tile_columns_minus1 + 1
        int num_tile_rows; ///< num_tile_rows_minus1 + 1
        uint8_t uniform_spacing_flag;

        int column_width[42];
        int row_height[42];

        uint8_t loop_filter_across_tiles_enabled_flag;
    } tiles;

    int num_substreams;

    uint8_t deblocking_filter_control_present_flag;

    int log2_parallel_merge_level; ///< log2_parallel_merge_level_minus2 + 2

    uint8_t pps_extension_flag;
    uint8_t pps_extension_data_flag;

    // Inferred parameters
    int SliceGranularity;
} PPS;

typedef struct {
    uint8_t aps_scaling_list_data_present_flag;
    uint8_t aps_deblocking_filter_flag;
    uint8_t alf_aps_filter_flag[3];
} APS;

typedef enum {
    B_SLICE = 0,
    P_SLICE = 1,
    I_SLICE = 2
} SliceType;

typedef struct {
    uint8_t first_slice_in_pic_flag;
    int slice_address;

    SliceType slice_type;

    uint8_t entropy_slice_flag;
    int pps_id; ///< pic_parameter_set_id
    uint8_t pic_output_flag;
    uint8_t colour_plane_id;
    int idr_pic_id;
    uint8_t no_output_of_prior_pics_flag;

    uint8_t slice_sample_adaptive_offset_flag;
    uint8_t sao_cb_enable_flag;
    uint8_t sao_cr_enable_flag;

    int aps_id;

    uint8_t cabac_init_flag;
    int slice_qp_delta;
    uint8_t disable_deblocking_filter_flag;
    int max_num_merge_cand; ///< 5 - 5_minus_max_num_merge_cand
    uint8_t slice_adaptive_loop_filter_flag;

    uint8_t slice_loop_filter_across_slices_enabled_flag;

#if REFERENCE_ENCODER_QUIRKS
    uint8_t tile_marker_flag;
#endif

    // Inferred parameters
    uint8_t slice_qp; ///< SliceQP
    int slice_ctb_addr_rs; ///< SliceCtbAddrRS
    int slice_cb_addr_zs; ///< SliceCbAddrZS
} SliceHeader;

enum SyntaxElement {
    SAO_MERGE_LEFT_FLAG = 0,
    SAO_MERGE_UP_FLAG,
    SAO_TYPE_IDX,
    SAO_BAND_POSITION,
    SAO_OFFSET_SIGN,
    SAO_OFFSET,
    ALF_CU_FLAG,
    END_OF_SLICE_FLAG,
    SPLIT_CODING_UNIT_FLAG,
    CU_TRANSQUANT_BYPASS_FLAG,
    SKIP_FLAG,
    CU_QP_DELTA,
    PRED_MODE_FLAG,
    PART_MODE,
    PCM_FLAG,
    PREV_INTRA_LUMA_PRED_FLAG,
    MPM_IDX,
    REM_INTRA_LUMA_PRED_MODE,
    INTRA_CHROMA_PRED_MODE,
    MERGE_FLAG,
    MERGE_IDX,
    INTER_PRED_IDC,
    REF_IDX_L0,
    REF_IDX_L1,
    ABS_MVD_GREATER0_FLAG,
    ABS_MVD_GREATER1_FLAG,
    ABS_MVD_MINUS2,
    MVD_SIGN_FLAG,
    MVP_L0_FLAG,
    MVP_L1_FLAG,
    NO_RESIDUAL_DATA_FLAG,
    SPLIT_TRANSFORM_FLAG,
    CBF_LUMA,
    CBF_CB_CR,
    TRANSFORM_SKIP_FLAG_0,
    TRANSFORM_SKIP_FLAG_1_2,
    LAST_SIGNIFICANT_COEFF_X_PREFIX,
    LAST_SIGNIFICANT_COEFF_Y_PREFIX,
    LAST_SIGNIFICANT_COEFF_X_SUFFIX,
    LAST_SIGNIFICANT_COEFF_Y_SUFFIX,
    SIGNIFICANT_COEFF_GROUP_FLAG,
    SIGNIFICANT_COEFF_FLAG,
    COEFF_ABS_LEVEL_GREATER1_FLAG,
    COEFF_ABS_LEVEL_GREATER2_FLAG,
    COEFF_ABS_LEVEL_REMAINING,
    COEFF_SIGN_FLAG
};

typedef struct HEVCCabacContext {
    uint16_t range; ///< codIRange
    uint16_t offset; ///< codIOffset

    enum SyntaxElement elem;

    uint8_t (*state)[2];

    int max_bin_idx_ctx; ///< maxBinIdxCtx
    const int8_t *ctx_idx_inc; ///< ctxIdxInc
    int ctx_idx_offset; ///< ctxIdxOffset

    uint8_t bypass_flag; ///< bypassFlag
} HEVCCabacContext;

enum SAOType {
    SAO_NOT_APPLIED = 0,
    SAO_0_EDGE = 1,
    SAO_90_EDGE = 2,
    SAO_135_EDGE = 3,
    SAO_45_EDGE = 4,
    SAO_BAND = 5
};

enum PartMode {
    PART_2Nx2N = 0,
    PART_2NxN = 1,
    PART_Nx2N = 2,
    PART_NxN = 3,
    PART_2NxnU = 4,
    PART_2NxnD = 5,
    PART_nLx2N = 6,
    PART_nRx2N = 7
};

enum PredMode {
    MODE_INTER = 0,
    MODE_INTRA,
    MODE_SKIP
};

typedef struct CodingUnit {
    uint8_t *skip_flag;
    enum PredMode pred_mode; ///< PredMode
    enum PartMode part_mode; ///< PartMode
    uint8_t no_residual_data_flag;

    // Inferred parameters
    uint8_t intra_split_flag; ///< IntraSplitFlag
    uint8_t max_trafo_depth; ///< MaxTrafoDepth
} CodingUnit;

typedef struct PredictionUnit {
    uint8_t pcm_flag;
    uint8_t merge_flag;

    uint8_t *prev_intra_luma_pred_flag;
    int *mpm_idx;
    uint8_t *rem_intra_luma_pred_mode;
    int *intra_chroma_pred_mode;
} PredictionUnit;

typedef struct TransformTree {
    uint8_t *(split_transform_flag[MAX_TRANSFORM_DEPTH]);
    uint8_t *(cbf_cb[MAX_TRANSFORM_DEPTH]);
    uint8_t *(cbf_cr[MAX_TRANSFORM_DEPTH]);
    uint8_t cbf_luma;

    // Inferred parameters
    uint8_t inter_split_flag;
    uint8_t inter_tb_split_direction_l;
    uint8_t inter_tb_split_direction_c;
} TransformTree;

typedef struct TransformUnit {
    int cu_qp_delta;

    // Inferred parameters;
    uint8_t is_cu_qp_delta_coded;
    int intra_pred_mode;
    int intra_pred_mode_c;
} TransformUnit;

typedef struct {
    AVCodecContext *avctx;
    AVFrame frame;

    GetBitContext gb;
    HEVCCabacContext cc;

    int nal_ref_flag;
    NALUnitType nal_unit_type;
    int temporal_id;

    SPS *sps_list[MAX_SPS_COUNT];
    PPS *pps_list[MAX_PPS_COUNT];
    APS *aps_list[MAX_APS_COUNT];

    SPS *sps;
    PPS *pps;
    APS *aps;

    SliceHeader sh;

    int ctb_addr_in_slice; ///< CtbAddrInSlice
    int num_pcm_block; ///< NumPCMBlock

    int ctb_addr_rs; ///< CtbAddrRS
    int ctb_addr_ts; ///< CtbAddrTS

    uint8_t sao_merge_left_flag;
    uint8_t sao_merge_up_flag;

    uint8_t *split_coding_unit_flag;

    enum SAOType *(sao_type_idx[3]);
    int *(sao_band_position[3]);

    CodingUnit cu;
    PredictionUnit pu;
    TransformTree tt;
    TransformUnit tu;
} HEVCContext;

int ff_hevc_decode_short_term_rps(HEVCContext *s, int idx,
                                  ShortTermRPS **prps);
int ff_hevc_decode_nal_sps(HEVCContext *s);
int ff_hevc_decode_nal_pps(HEVCContext *s);
int ff_hevc_decode_nal_aps(HEVCContext *s);
int ff_hevc_decode_nal_sei(HEVCContext *s);

void ff_hevc_cabac_init(HEVCContext *s);
int ff_hevc_cabac_decode(HEVCContext *s, enum SyntaxElement elem);

#endif // AVCODEC_HEVC_H
