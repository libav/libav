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
#include "hevcpred.h"
#include "hevcdsp.h"

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
    NAL_TFD_SLICE = 2,
    NAL_TLA_SLICE = 3,
    NAL_CRA_SLICE = 4,
    NAL_CRA_SLICE_NO_TFD = 5,
    NAL_BLA_SLICE = 6,
    NAL_BLA_SLICE_NO_TFD = 7,
    NAL_IDR_SLICE = 8,
    NAL_VPS = 25,
    NAL_SPS = 26,
    NAL_PPS = 27,
    NAL_APS = 28,
    NAL_AUD = 29,
    NAL_FILLER_DATA = 30,
    NAL_SEI = 31,
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
#define MAX_VPS_COUNT 16
#define MAX_SPS_COUNT 32
#define MAX_PPS_COUNT 256
#define MAX_SHORT_TERM_RPS_COUNT 64

//TODO: check if this is really the maximum
#define MAX_TRANSFORM_DEPTH 3

#define MAX_TB_SIZE 32

struct VPS {
    int vps_max_temporal_layers; ///< vps_max_temporal_layers_minus1 + 1
    int vps_max_layers; ///< vps_max_layers_minus1 + 1

    uint8_t vps_temporal_id_nesting_flag;
    int vps_max_dec_pic_buffering[MAX_TEMPORAL_LAYERS];
    int vps_num_reorder_pics[MAX_TEMPORAL_LAYERS];
    int vps_max_latency_increase[MAX_TEMPORAL_LAYERS];
};

typedef struct {
    uint8_t profile_space;
    uint8_t profile_idc;
    uint8_t level_idc;

    int vps_id;
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

    uint8_t deblocking_filter_in_aps_enabled_flag;

    uint8_t asymmetric_motion_partitions_enabled_flag;
    uint8_t sample_adaptive_offset_enabled_flag;

    uint8_t temporal_id_nesting_flag;

    int num_short_term_ref_pic_sets;
    ShortTermRPS *short_term_rps_list[MAX_SHORT_TERM_RPS_COUNT];

    uint8_t long_term_ref_pics_present_flag;
    uint8_t sps_temporal_mvp_enabled_flag;

#if REFERENCE_ENCODER_QUIRKS
    uint8_t amvp_mode_flag[4];
#endif

    // Inferred parameters
    int Log2CtbSize;
    int PicWidthInCtbs;
    int PicHeightInCtbs;
    int pic_width_in_min_cbs;
    int pic_height_in_min_cbs;

    int log2_min_pu_size;
} SPS;

typedef struct {
    int sps_id; ///< seq_parameter_set_id

    uint8_t sign_data_hiding_flag;

    uint8_t cabac_init_present_flag;

    int num_ref_idx_l0_default_active; ///< num_ref_idx_l0_default_active_minus1 + 1
    int num_ref_idx_l1_default_active; ///< num_ref_idx_l1_default_active_minus1 + 1
    int pic_init_qp_minus26;

    uint8_t constrained_intra_pred_flag;
    uint8_t transform_skip_enabled_flag;

    uint8_t cu_qp_delta_enabled_flag;
    int diff_cu_qp_delta_depth;

    int cb_qp_offset;
    int cr_qp_offset;
    uint8_t weighted_pred_flag;
    uint8_t weighted_bipred_flag;
    uint8_t output_flag_present_flag;
    uint8_t dependant_slices_enabled_flag;
    uint8_t transquant_bypass_enable_flag;

    uint8_t tiles_or_entropy_coding_sync_idc;
    int num_tile_columns; ///< num_tile_columns_minus1 + 1
    int num_tile_rows; ///< num_tile_rows_minus1 + 1
    uint8_t uniform_spacing_flag;
    uint8_t loop_filter_across_tiles_enabled_flag;

    uint8_t cabac_independant_flag;

    uint8_t seq_loop_filter_across_slices_enabled_flag;

    uint8_t deblocking_filter_control_present_flag;
    uint8_t deblocking_filter_override_enabled_flag;
    uint8_t pps_disable_deblocking_filter_flag;
    int beta_offset; ///< beta_offset_div2 * 2
    int tc_offset; ///< tc_offset_div2 * 2

    int pps_scaling_list_data_present_flag;

    int log2_parallel_merge_level; ///< log2_parallel_merge_level_minus2 + 2
    uint8_t slice_header_extension_present_flag;

    uint8_t pps_extension_flag;
    uint8_t pps_extension_data_flag;

    // Inferred parameters
    int *column_width; ///< ColumnWidth
    int *row_height; ///< RowHeight
    int *col_bd; ///< ColBd
    int *row_bd; ///< RowBd

    int *ctb_addr_rs_to_ts; ///< CtbAddrRSToTS
    int *ctb_addr_ts_to_rs; ///< CtbAddrTSToRS
    int *tile_id; ///< TileId
    int *min_cb_addr_zs; ///< MinCbAddrZS
} PPS;

typedef enum {
    B_SLICE = 0,
    P_SLICE = 1,
    I_SLICE = 2
} SliceType;

typedef struct {
    uint8_t first_slice_in_pic_flag;
    int slice_address;

    SliceType slice_type;

    uint8_t dependent_slice_flag;
    int pps_id; ///< pic_parameter_set_id
    uint8_t pic_output_flag;
    uint8_t colour_plane_id;
    uint8_t no_output_of_prior_pics_flag;

    uint8_t slice_sample_adaptive_offset_flag[3];

    uint8_t cabac_init_flag;
    int slice_qp_delta;

    uint8_t disable_deblocking_filter_flag; ///< slice_header_disable_deblocking_filter_flag
    int beta_offset; ///< beta_offset_div2 * 2
    int tc_offset; ///< tc_offset_div2 * 2

    int max_num_merge_cand; ///< 5 - 5_minus_max_num_merge_cand

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
    SAO_OFFSET_ABS,
    SAO_OFFSET_SIGN,
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
    int init_type; ///< initType

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

typedef struct CodingTree {
    int depth; ///< ctDepth
} CodingTree;

typedef struct CodingUnit {
    uint8_t cu_transquant_bypass_flag;
    uint8_t *skip_flag;
    enum PredMode pred_mode; ///< PredMode
    enum PartMode part_mode; ///< PartMode
    uint8_t no_residual_data_flag;

    // Inferred parameters
    uint8_t intra_split_flag; ///< IntraSplitFlag
    uint8_t max_trafo_depth; ///< MaxTrafoDepth

    int x;
    int y;
    uint8_t *top_cb_available;
    uint8_t *left_cb_available;
} CodingUnit;

enum IntraPredMode {
    INTRA_PLANAR = 0,
    INTRA_DC,
    INTRA_ANGULAR_2,
    INTRA_ANGULAR_3,
    INTRA_ANGULAR_4,
    INTRA_ANGULAR_5,
    INTRA_ANGULAR_6,
    INTRA_ANGULAR_7,
    INTRA_ANGULAR_8,
    INTRA_ANGULAR_9,
    INTRA_ANGULAR_10,
    INTRA_ANGULAR_11,
    INTRA_ANGULAR_12,
    INTRA_ANGULAR_13,
    INTRA_ANGULAR_14,
    INTRA_ANGULAR_15,
    INTRA_ANGULAR_16,
    INTRA_ANGULAR_17,
    INTRA_ANGULAR_18,
    INTRA_ANGULAR_19,
    INTRA_ANGULAR_20,
    INTRA_ANGULAR_21,
    INTRA_ANGULAR_22,
    INTRA_ANGULAR_23,
    INTRA_ANGULAR_24,
    INTRA_ANGULAR_25,
    INTRA_ANGULAR_26,
    INTRA_ANGULAR_27,
    INTRA_ANGULAR_28,
    INTRA_ANGULAR_29,
    INTRA_ANGULAR_30,
    INTRA_ANGULAR_31,
    INTRA_ANGULAR_32,
    INTRA_ANGULAR_33,
    INTRA_ANGULAR_34
};

struct PUContent {
    enum IntraPredMode intra_pred_mode;
    int ct_depth;
};

typedef struct PredictionUnit {
    uint8_t pcm_flag;
    uint8_t merge_flag;

    int mpm_idx;
    int rem_intra_luma_pred_mode;

    enum IntraPredMode intra_pred_mode[4];
    enum IntraPredMode intra_pred_mode_c;

    struct PUContent *pu_vert;
    struct PUContent *pu_horiz;
} PredictionUnit;

typedef struct TransformTree {
    uint8_t *(split_transform_flag[MAX_TRANSFORM_DEPTH]);
    uint8_t *(cbf_cb[MAX_TRANSFORM_DEPTH]);
    uint8_t *(cbf_cr[MAX_TRANSFORM_DEPTH]);
    uint8_t cbf_luma;

    // Inferred parameters
    uint8_t inter_split_flag;
} TransformTree;

typedef struct TransformUnit {
    int cu_qp_delta;

    // Inferred parameters;
    uint8_t is_cu_qp_delta_coded;
    int cur_intra_pred_mode;
} TransformUnit;

typedef struct ResidualCoding {
    //FIXME: check size
    uint8_t significant_coeff_group_flag[64][64];
} ResidualCoding;

struct SAOParams {
    enum SAOType type_idx[3]; ///< sao_type_idx
    int band_position[3]; ///< sao_band_position
    int offset_abs[3][4]; ///< sao_offset_abs
    int offset_sign[3][4]; ///< sao_offset_sign

    // Inferred parameters
    int offset_val[3][5]; ///<SaoOffsetVal
};

typedef struct HEVCContext {
    AVCodecContext *avctx;
    AVFrame frame;

    struct HEVCPredContext hpc;
    struct HEVCDSPContext hevcdsp;

    GetBitContext gb;
    HEVCCabacContext cc;

    int nal_ref_flag;
    NALUnitType nal_unit_type;
    int temporal_id;

    struct VPS *vps_list[MAX_VPS_COUNT];
    SPS *sps_list[MAX_SPS_COUNT];
    PPS *pps_list[MAX_PPS_COUNT];

    struct VPS *vps;
    SPS *sps;
    PPS *pps;

    SliceHeader sh;
    struct SAOParams *sao;

    int ctb_addr_in_slice; ///< CtbAddrInSlice
    int num_pcm_block; ///< NumPCMBlock

    int ctb_addr_rs; ///< CtbAddrRS
    int ctb_addr_ts; ///< CtbAddrTS

    uint8_t *split_coding_unit_flag;

    CodingTree ct;
    CodingUnit cu;
    PredictionUnit pu;
    TransformTree tt;
    TransformUnit tu;
    ResidualCoding rc;
} HEVCContext;

enum ScanType {
    SCAN_DIAG = 0,
    SCAN_HORIZ,
    SCAN_VERT
};

int ff_hevc_decode_short_term_rps(HEVCContext *s, int idx,
                                  ShortTermRPS **prps);
int ff_hevc_decode_nal_vps(HEVCContext *s);
int ff_hevc_decode_nal_sps(HEVCContext *s);
int ff_hevc_decode_nal_pps(HEVCContext *s);
int ff_hevc_decode_nal_sei(HEVCContext *s);

void ff_hevc_cabac_init(HEVCContext *s);
int ff_hevc_cabac_decode(HEVCContext *s, enum SyntaxElement elem);
int ff_hevc_sao_merge_left_flag_decode(HEVCContext *s);
int ff_hevc_sao_offset_abs_decode(HEVCContext *s, int bit_depth);
int ff_hevc_sao_offset_sign_decode(HEVCContext *s);
int ff_hevc_cu_transquant_bypass_flag_decode(HEVCContext *s);
int ff_hevc_split_coding_unit_flag_decode(HEVCContext *s, int ct_depth, int x0, int y0);
int ff_hevc_part_mode_decode(HEVCContext *s, int log2_cb_size);
int ff_hevc_split_transform_flag_decode(HEVCContext *s, int log2_trafo_size);
int ff_hevc_cbf_cb_cr_decode(HEVCContext *s, int trafo_depth);
int ff_hevc_cbf_luma_decode(HEVCContext *s, int trafo_depth, int log2_trafo_size,
                            int log2_max_trafo_size);
int ff_hevc_transform_skip_flag_decode(HEVCContext *s, int c_idx);
int ff_hevc_last_significant_coeff_prefix_decode(HEVCContext *s, int c_idx,
                                                 int log2_size, int is_x);
int ff_hevc_last_significant_coeff_suffix_decode(HEVCContext *s,
                                                 int last_significant_coeff_prefix,
                                                 int is_x);
int ff_hevc_significant_coeff_group_flag_decode(HEVCContext *s, int c_idx, int x_cg,
                                                int y_cg, int log2_trafo_width,
                                                int log2_trafo_height, int scan_idx);
int ff_hevc_significant_coeff_flag_decode(HEVCContext *s, int c_idx, int x_c, int y_c,
                                          int log2_trafo_width, int log2_trafo_height,
                                          int scan_idx);
int ff_hevc_coeff_abs_level_greater1_flag_decode(HEVCContext *s, int c_idx,
                                                 int i, int n,
                                                 int first_greater1_coeff_idx,
                                                 int first_subset);
int ff_hevc_coeff_abs_level_greater2_flag_decode(HEVCContext *s, int c_idx,
                                                 int i, int n);
int ff_hevc_coeff_abs_level_remaining(HEVCContext *s, int n, int base_level);
int ff_hevc_coeff_sign_flag(HEVCContext *s);

#endif // AVCODEC_HEVC_H
