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
 * NOTE: This decoder is based on the JCTVC-H1003 draft of the HEVC specification.
 */

/**
 * Enable to diverge from the spec when the reference encoder
 * does so.
 */
#define SUPPORT_ENCODER 1

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
    uint8_t deblocking_filter_in_aps_enabled_flag;

    uint8_t seq_loop_filter_across_slices_enabled_flag;
    uint8_t asymmetric_motion_partitions_enabled_flag;
    uint8_t non_square_quadtree_enabled_flag;
    uint8_t sample_adaptive_offset_enabled_flag;

    uint8_t adaptive_loop_filter_enabled_flag;
    uint8_t alf_coef_in_slice_flag;

    uint8_t temporal_id_nesting_flag;

    uint8_t inter_4x4_enabled_flag;

    int num_short_term_ref_pic_sets;
    ShortTermRPS *short_term_rps_list[MAX_SHORT_TERM_RPS_COUNT];

    uint8_t long_term_ref_pics_present_flag;

#if SUPPORT_ENCODER
    uint8_t amvp_mode_flag[4];
#endif

    uint8_t tiles_or_entropy_coding_sync_idc;
    int num_tile_columns;
    int num_tile_rows;
    uint8_t uniform_spacing_flag;
    int *column_width;
    int *row_height;
    uint8_t loop_filter_across_tiles_enabled_flag;


    /**
     * Inferred parameters
     */

    int Log2CtbSize;
    int PicWidthInCtbs;
    int PicHeightInCtbs;
} SPS;

typedef struct {
    /**
     * Coded parameters
     */

    int sps_id; ///< seq_parameter_set_id

    uint8_t sign_data_hiding_flag;
    uint8_t sign_hiding_threshold;

    uint8_t cabac_init_present_flag;

#if SUPPORT_ENCODER
    uint8_t entropy_coding_mode_flag;
#endif

    int num_ref_idx_l0_default_active; ///< num_ref_idx_l0_default_active_minus1 + 1
    int num_ref_idx_l1_default_active; ///< num_ref_idx_l1_default_active_minus1 + 1
    int pic_init_qp_minus26;

    uint8_t constrained_intra_pred_flag;
    uint8_t enable_temporal_mvp_flag;
    uint8_t slice_granularity;
    int max_cu_qp_delta_depth;
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

    /**
     * Inferred parameters
     */

    int SliceGranularity;
} PPS;

typedef struct {
    uint8_t aps_scaling_list_data_present_flag;
    uint8_t aps_deblocking_filter_flag;
    uint8_t aps_sao_interleaving_flag;
    uint8_t aps_adaptive_loop_filter_flag;
} APS;

typedef enum {
#if SUPPORT_ENCODER
    I_SLICE = 0,
    P_SLICE = 1,
    B_SLICE = 2
#else
    P_SLICE = 0,
    B_SLICE = 1,
    I_SLICE = 2
#endif
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

    uint8_t slice_sao_interleaving_flag;
    uint8_t slice_sample_adaptive_offset_flag;
    uint8_t sao_cb_enable_flag;
    uint8_t sao_cr_enable_flag;

    int aps_id;

    uint8_t cabac_init_flag;
    int slice_qp_delta;
    uint8_t disable_deblocking_filter_flag;
    int max_num_merge_cand; ///< 5 - 5_minus_max_num_merge_cand
    uint8_t slice_adaptive_loop_filter_flag;
} SliceHeader;

typedef struct {
    AVCodecContext *avctx;
    int nal_ref_flag;
    NALUnitType nal_unit_type;
    int temporal_id;

    SliceHeader cur_slice_header;

    SPS *sps_list[MAX_SPS_COUNT];
    PPS *pps_list[MAX_PPS_COUNT];
    APS *aps_list[MAX_APS_COUNT];

    SPS *sps;
    PPS *pps;
    APS *aps;
} HEVCContext;

int ff_hevc_decode_short_term_rps(HEVCContext *s, GetBitContext *gb, int idx,
                                  ShortTermRPS **prps);
int ff_hevc_decode_nal_sps(HEVCContext *s, GetBitContext *gb);
int ff_hevc_decode_nal_pps(HEVCContext *s, GetBitContext *gb);
int ff_hevc_decode_nal_aps(HEVCContext *s, GetBitContext *gb);
int ff_hevc_decode_nal_sei(HEVCContext *s, GetBitContext *gb);

#endif // AVCODEC_HEVC_H
