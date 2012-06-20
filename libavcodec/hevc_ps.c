/*
 * HEVC Parameter Set Decoding
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

#include "golomb.h"
#include "hevc.h"

/**
 * Section 7.3.3.1
 */
int ff_hevc_decode_short_term_rps(HEVCContext *s, int idx, ShortTermRPS **prps)
{
    GetBitContext *gb = &s->gb;

    ShortTermRPS *rps = NULL;
    *prps = av_malloc(sizeof(ShortTermRPS));
    if (*prps == NULL)
        return -1;

    rps = *prps;

    rps->inter_ref_pic_set_prediction_flag = get_bits1(gb);
    if (rps->inter_ref_pic_set_prediction_flag) {
        av_log(s->avctx, AV_LOG_ERROR,
               "TODO: inter_ref_pic_set_prediction_flag\n");
    } else {
        rps->num_negative_pics = get_ue_golomb(gb);
        rps->num_positive_pics = get_ue_golomb(gb);
        if (rps->num_negative_pics || rps->num_positive_pics) {
            av_log(s->avctx, AV_LOG_ERROR,
                   "TODO: num_negative_pics || num_positive_pics\n");
        }
    }

    return 0;
}

int ff_hevc_decode_nal_sps(HEVCContext *s)
{
    GetBitContext *gb = &s->gb;

    int sps_id = 0;
#if REFERENCE_ENCODER_QUIRKS
    int max_cu_depth = 0;
#endif
    SPS *sps = av_mallocz(sizeof(SPS));
    if (sps == NULL)
        goto err;

    av_log(s->avctx, AV_LOG_DEBUG, "Decoding SPS\n");

    memset(sps->short_term_rps_list, 0, sizeof(sps->short_term_rps_list));

    // Default values
    sps->num_tile_columns     = 1;
    sps->num_tile_rows        = 1;
    sps->uniform_spacing_flag = 1;

    // Coded parameters

    sps->profile_idc = get_bits(gb, 8);
    skip_bits(gb, 8); // reserved_zero_8bits
    sps->level_idc = get_bits(gb, 8);
    sps_id         = get_ue_golomb(gb);
    if (sps_id >= MAX_SPS_COUNT) {
        av_log(s->avctx, AV_LOG_ERROR, "SPS id out of range: %d\n", sps_id);
        goto err;
    }

    sps->chroma_format_idc = get_ue_golomb(gb);
    if (sps->chroma_format_idc == 3)
        sps->separate_colour_plane_flag = get_bits1(gb);

    sps->max_temporal_layers = get_bits(gb, 3) + 1;

    sps->pic_width_in_luma_samples  = get_ue_golomb(gb);
    sps->pic_height_in_luma_samples = get_ue_golomb(gb);

    sps->pic_cropping_flag = get_bits1(gb);
    if (sps->pic_cropping_flag) {
        sps->pic_crop.left_offset   = get_ue_golomb(gb);
        sps->pic_crop.right_offset  = get_ue_golomb(gb);
        sps->pic_crop.top_offset    = get_ue_golomb(gb);
        sps->pic_crop.bottom_offset = get_ue_golomb(gb);
    }

    sps->bit_depth_luma = get_ue_golomb(gb) + 8;
    sps->bit_depth_chroma = get_ue_golomb(gb) + 8;

    sps->pcm_enabled_flag = get_bits1(gb);
    if (sps->pcm_enabled_flag) {
        sps->pcm.bit_depth_luma = get_bits(gb, 4) + 1;
        sps->pcm.bit_depth_luma = get_bits(gb, 4) + 1;
    }

    sps->qpprime_y_zero_transquant_bypass_flag = get_bits1(gb);

    sps->log2_max_poc_lsb = get_ue_golomb(gb) + 4;

    for (int i = 0; i < sps->max_temporal_layers; i++) {
        sps->temporal_layer[i].max_dec_pic_buffering = get_ue_golomb(gb);
        sps->temporal_layer[i].num_reorder_pics      = get_ue_golomb(gb);
        sps->temporal_layer[i].max_latency_increase  = get_ue_golomb(gb);
    }

    sps->restricted_ref_pic_lists_flag = get_bits1(gb);
    if (sps->restricted_ref_pic_lists_flag) {
        sps->lists_modification_present_flag = get_bits1(gb);
    }

    sps->log2_min_coding_block_size             = get_ue_golomb(gb) + 3;
    sps->log2_diff_max_min_coding_block_size    = get_ue_golomb(gb);
    sps->log2_min_transform_block_size          = get_ue_golomb(gb) + 2;
    sps->log2_diff_max_min_transform_block_size = get_ue_golomb(gb);

#if REFERENCE_ENCODER_QUIRKS
    if (sps->log2_min_coding_block_size == 3) {
        sps->inter_4x4_enabled_flag = get_bits1(gb);
    }
#endif

    if (sps->pcm_enabled_flag) {
        sps->pcm.log2_min_pcm_coding_block_size          = get_ue_golomb(gb) + 3;
        sps->pcm.log2_diff_max_min_pcm_coding_block_size = get_ue_golomb(gb);
    }

    sps->max_transform_hierarchy_depth_inter = get_ue_golomb(gb);
    sps->max_transform_hierarchy_depth_intra = get_ue_golomb(gb);

    sps->scaling_list_enable_flag = get_bits1(gb);

    sps->chroma_pred_from_luma_enabled_flag         = get_bits1(gb);
    sps->transform_skip_enabled_flag                = get_bits1(gb);
    sps->deblocking_filter_in_aps_enabled_flag      = get_bits1(gb);
    sps->seq_loop_filter_across_slices_enabled_flag = get_bits1(gb);
    sps->asymmetric_motion_partitions_enabled_flag  = get_bits1(gb);
    sps->nsrqt_enabled_flag                         = get_bits1(gb);
    sps->sample_adaptive_offset_enabled_flag        = get_bits1(gb);

    sps->adaptive_loop_filter_enabled_flag = get_bits1(gb);
    if (sps->adaptive_loop_filter_enabled_flag)
        sps->alf_coef_in_slice_flag = get_bits1(gb);

    if (sps->pcm_enabled_flag)
        sps->pcm.loop_filter_disable_flag = get_bits1(gb);

    sps->temporal_id_nesting_flag = get_bits1(gb);

#if !REFERENCE_ENCODER_QUIRKS
    if (sps->log2_min_coding_block_size == 3)
        sps->inter_4x4_enabled_flag = get_bits1(gb);
#endif

    sps->num_short_term_ref_pic_sets = get_ue_golomb(gb);
    for (int i = 0; i < sps->num_short_term_ref_pic_sets; i++) {
        if (ff_hevc_decode_short_term_rps(s, i, &sps->short_term_rps_list[i]) < 0)
            goto err;
    }

    sps->long_term_ref_pics_present_flag = get_bits1(gb);

#if REFERENCE_ENCODER_QUIRKS
    max_cu_depth = sps->log2_diff_max_min_coding_block_size
                   + ((sps->log2_min_coding_block_size >
                       sps->log2_min_transform_block_size)
                      ? (sps->log2_min_coding_block_size
                         - sps->log2_min_transform_block_size)
                      : 0);
    for (int i = 0; i < max_cu_depth; i++)
        sps->amvp_mode_flag[i] = get_bits1(gb);
#endif

    sps->tiles_or_entropy_coding_sync_idc = get_bits(gb, 2);
    if (sps->tiles_or_entropy_coding_sync_idc == 1) {
        sps->num_tile_columns = get_ue_golomb(gb) + 1;
        sps->num_tile_rows    = get_ue_golomb(gb) + 1;

        sps->uniform_spacing_flag = get_bits1(gb);
        if (!sps->uniform_spacing_flag) {
            av_log(s->avctx, AV_LOG_ERROR, "TODO: !uniform_spacing_flag\n");
            for (int i = 0; i < sps->num_tile_columns - 1; i++) {
                sps->column_width[i] = get_ue_golomb(gb);
            }
            for (int i = 0; i < sps->num_tile_rows - 1; i++) {
                sps->row_height[i] = get_ue_golomb(gb);
            }
        }
    }

    // Inferred parameters

    sps->Log2CtbSize = sps->log2_min_coding_block_size
        + sps->log2_diff_max_min_coding_block_size;
    sps->PicWidthInCtbs = ROUNDED_DIV(sps->pic_width_in_luma_samples,
                                  (1 << sps->Log2CtbSize));
    sps->PicHeightInCtbs = ROUNDED_DIV(sps->pic_height_in_luma_samples,
                                  (1 << sps->Log2CtbSize));
    sps->pic_width_in_min_cbs = sps->pic_width_in_luma_samples / (1 << sps->log2_min_coding_block_size);
    sps->pic_height_in_min_cbs = sps->pic_height_in_luma_samples / (1 << sps->log2_min_coding_block_size);


    sps->column_width = av_malloc(sps->num_tile_columns * sizeof(int));
    sps->row_height   = av_malloc(sps->num_tile_rows * sizeof(int));
    sps->col_bd       = av_malloc((sps->num_tile_columns + 1) * sizeof(int));
    sps->row_bd       = av_malloc((sps->num_tile_rows + 1) * sizeof(int));
    if (sps->column_width == NULL || sps->row_height == NULL ||
        sps->col_bd == NULL || sps->row_bd == NULL)
        goto err;

    if (sps->uniform_spacing_flag) {
        for (int i = 0; i < sps->num_tile_columns; i++) {
            sps->column_width[i] =
                ((i + 1) * sps->PicWidthInCtbs) / (sps->num_tile_columns) -
                (i * sps->PicWidthInCtbs) / (sps->num_tile_columns);
        }

        for (int i = 0; i < sps->num_tile_rows; i++) {
            sps->row_height[i] =
                ((i + 1) * sps->PicHeightInCtbs) / (sps->num_tile_rows) -
                (i * sps->PicHeightInCtbs) / (sps->num_tile_rows);
        }
    }

    sps->col_bd[0] = 0;
    for (int i = 0; i < sps->num_tile_columns; i++)
        sps->col_bd[i+1] = sps->col_bd[i] + sps->column_width[i];

    sps->row_bd[0] = 0;
    for (int i = 0; i < sps->num_tile_rows; i++)
        sps->row_bd[i+1] = sps->row_bd[i] + sps->row_height[i];

    /**
     * 6.5
     */
    sps->ctb_addr_rs_to_ts = av_malloc(sps->PicWidthInCtbs *
                                       sps->PicHeightInCtbs * sizeof(int));
    sps->ctb_addr_ts_to_rs = av_malloc(sps->PicWidthInCtbs *
                                       sps->PicHeightInCtbs * sizeof(int));
    sps->tile_id = av_malloc(sps->PicWidthInCtbs *
                             sps->PicHeightInCtbs * sizeof(int));
    sps->min_cb_addr_zs = av_malloc(sps->pic_width_in_min_cbs *
                                    sps->pic_height_in_min_cbs * sizeof(int));
    if (sps->ctb_addr_rs_to_ts == NULL || sps->ctb_addr_ts_to_rs == NULL ||
        sps->tile_id == NULL || sps->min_cb_addr_zs == NULL)
        goto err;

    for (int ctb_addr_rs = 0;
         ctb_addr_rs < sps->PicWidthInCtbs * sps->PicHeightInCtbs;
         ctb_addr_rs++) {
        int tb_x = ctb_addr_rs % sps->PicWidthInCtbs;
        int tb_y = ctb_addr_rs / sps->PicWidthInCtbs;
        int tile_x = 0;
        int tile_y = 0;
        int val = 0;

        for (int i = 0; i < sps->num_tile_columns; i++) {
            if ( tb_x < sps->col_bd[i + 1] ) {
                tile_x = i;
                break;
            }
        }

        for (int i = 0; i < sps->num_tile_rows; i++) {
            if( tb_y < sps->row_bd[i + 1] ) {
                tile_y = i;
                break;
            }
        }

        val = ctb_addr_rs - tb_x - (tb_y - sps->row_bd[tile_y])*sps->PicWidthInCtbs;
        for (int i = 0; i < tile_x; i++ )
            val += sps->row_height[tile_y] * sps->column_width[i];
        val += (tb_y - sps->row_bd[tile_y]) * sps->column_width[tile_y] +
               tb_x - sps->col_bd[tile_x];

        sps->ctb_addr_rs_to_ts[ctb_addr_rs] = val;
        sps->ctb_addr_ts_to_rs[val] = ctb_addr_rs;
    }

    for (int j = 0, tile_id = 0; j < sps->num_tile_rows; j++)
        for (int i = 0; i < sps->num_tile_columns; i++, tile_id++)
            for (int y = sps->row_bd[j]; y < sps->row_bd[j+1]; y++)
                for (int x = sps->col_bd[j]; x < sps->col_bd[j+1]; x++)
                    sps->tile_id[sps->ctb_addr_rs_to_ts[y*sps->PicWidthInCtbs + x]] =
                        tile_id;

    // Different from the spec, see http://hevc.kw.bbc.co.uk/trac/ticket/527
    for (int y = 0; y < sps->pic_height_in_min_cbs; y++) {
        for (int x = 0; x < sps->pic_width_in_min_cbs; x++) {
            int tb_x = x >> sps->log2_diff_max_min_coding_block_size;
            int tb_y = y >> sps->log2_diff_max_min_coding_block_size;
            int ctb_addr_rs = sps->PicWidthInCtbs * tb_y + tb_x;
            int val = sps->ctb_addr_rs_to_ts[ctb_addr_rs] << sps->log2_diff_max_min_coding_block_size;
            for (int i = 0; i < sps->log2_diff_max_min_coding_block_size; i++) {
                int m = 1 << i;
                val += (m & x ? m*m : 0) + (m & y ? 2*m*m : 0);
            }
            sps->min_cb_addr_zs[sps->pic_height_in_min_cbs * x + y] = val;
        }
    }

    av_free(s->sps_list[sps_id]);
    s->sps_list[sps_id] = sps;
    return 0;

err:
    for (int i = 0; i < MAX_SHORT_TERM_RPS_COUNT; i++)
        av_free(sps->short_term_rps_list[i]);

    av_free(sps->column_width);
    av_free(sps->row_height);
    av_free(sps->col_bd);
    av_free(sps->row_bd);
    av_free(sps->ctb_addr_rs_to_ts);
    av_free(sps->ctb_addr_ts_to_rs);
    av_free(sps->tile_id);
    av_free(sps->min_cb_addr_zs);
    av_free(sps);
    return -1;
}

int ff_hevc_decode_nal_pps(HEVCContext *s)
{
    GetBitContext *gb = &s->gb;

    SPS *sps = 0;
    int pps_id = 0;

    PPS *pps = av_mallocz(sizeof(PPS));
    if (pps == NULL)
        goto err;

    av_log(s->avctx, AV_LOG_DEBUG, "Decoding PPS\n");

    // Default values
    pps->num_substreams = 1;
    pps->tiles.loop_filter_across_tiles_enabled_flag = 1;

    // Coded parameters
    pps_id = get_ue_golomb(gb);
    if (pps_id >= MAX_PPS_COUNT) {
        av_log(s->avctx, AV_LOG_ERROR, "PPS id out of range: %d\n", pps_id);
        goto err;
    }
    pps->sps_id = get_ue_golomb(gb);
    sps = s->sps_list[pps->sps_id];

    pps->sign_data_hiding_flag = get_bits1(gb);

    pps->cabac_init_present_flag = get_bits1(gb);

#if REFERENCE_ENCODER_QUIRKS
    pps->num_ref_idx_l0_default_active = get_bits(gb, 3);
    pps->num_ref_idx_l1_default_active = get_bits(gb, 3);
#else
    pps->num_ref_idx_l0_default_active = get_ue_golomb(gb) + 1;
    pps->num_ref_idx_l1_default_active = get_ue_golomb(gb) + 1;
#endif

    pps->pic_init_qp_minus26 = get_se_golomb(gb);

    pps->constrained_intra_pred_flag = get_bits1(gb);
    pps->enable_temporal_mvp_flag    = get_bits1(gb);
    pps->slice_granularity           = get_bits(gb, 2);

    pps->diff_cu_qp_delta_depth = get_ue_golomb(gb);

    pps->cb_qp_offset = get_se_golomb(gb);
    pps->cr_qp_offset = get_se_golomb(gb);

    pps->weighted_pred_flag       = get_bits1(gb);
    pps->weighted_bipred_idc      = get_bits(gb, 2);
    pps->output_flag_present_flag = get_bits1(gb);

    if (sps->tiles_or_entropy_coding_sync_idc == 1) {
        pps->tiles.tile_info_present_flag    = get_bits1(gb);
        pps->tiles.tile_control_present_flag = get_bits1(gb);
        if (pps->tiles.tile_info_present_flag) {
            pps->tiles.num_tile_columns     = get_ue_golomb(gb) + 1;
            pps->tiles.num_tile_rows        = get_ue_golomb(gb) + 1;
            pps->tiles.uniform_spacing_flag = get_bits1(gb);
            if (!pps->tiles.uniform_spacing_flag) {
                for (int i = 0; i < pps->tiles.num_tile_columns - 1; i++) {
                    pps->tiles.column_width[i] = get_ue_golomb(gb);
                }
                for (int i = 0; i < pps->tiles.num_tile_rows - 1; i++) {
                    pps->tiles.row_height[i] = get_ue_golomb(gb);
                }
            }
        }
        if (pps->tiles.tile_control_present_flag) {
            pps->tiles.loop_filter_across_tiles_enabled_flag = get_bits1(gb);
        }
    } else if (sps->tiles_or_entropy_coding_sync_idc == 2) {
        pps->num_substreams = get_ue_golomb(gb) + 1;
    }

    pps->deblocking_filter_control_present_flag = get_bits1(gb);

    // if (slice_type != I_SLICE)
    pps->log2_parallel_merge_level = get_ue_golomb(gb) + 2;

    // Inferred parameters
    pps->SliceGranularity = pps->slice_granularity << 1;

    av_free(s->pps_list[pps_id]);
    s->pps_list[pps_id] = pps;
    return 0;

err:
    av_free(pps);
    return -1;
}

int ff_hevc_decode_nal_aps(HEVCContext *s)
{
    GetBitContext *gb = &s->gb;

    int aps_id = 0;

    APS *aps = av_mallocz(sizeof(APS));
    if (aps == NULL)
        goto err;

    av_log(s->avctx, AV_LOG_DEBUG, "Decoding APS\n");

    aps_id = get_ue_golomb(gb);
    if (aps_id >= MAX_PPS_COUNT) {
        av_log(s->avctx, AV_LOG_ERROR, "APS id out of range: %d\n", aps_id);
        goto err;
    }

    aps->aps_scaling_list_data_present_flag = get_bits1(gb);
    if (aps->aps_scaling_list_data_present_flag) {
        av_log(s->avctx, AV_LOG_ERROR, "aps_scaling_list_data_present_flag: Not supported\n");
        goto err;
    }
    aps->aps_deblocking_filter_flag = get_bits1(gb);
    if (aps->aps_deblocking_filter_flag) {
        av_log(s->avctx, AV_LOG_ERROR, "aps_deblocking_filter_flag: Not supported\n");
        goto err;
    }

    for (int i = 0; i < 3; i++) {
        aps->alf_aps_filter_flag[i] = get_bits1(gb);
        if (aps->alf_aps_filter_flag[i]) {
            av_log(s->avctx, AV_LOG_ERROR, "alf_aps_filter_flag: Not supported\n");
        }
    }

    av_free(s->aps_list[aps_id]);
    s->aps_list[aps_id] = aps;
    return 0;

err:
    av_free(aps);
    return -1;
}

static int decode_nal_sei_message(HEVCContext *s)
{
    GetBitContext *gb = &s->gb;

    int payload_type = 0;
    int payload_size = 0;
    int byte = 0xFF;
    av_log(s->avctx, AV_LOG_DEBUG, "Decoding SEI\n");

    while (byte == 0xFF)
        payload_type += (byte = get_bits(gb, 8));

    byte = 0xFF;
    while (byte == 0xFF)
        payload_size += (byte = get_bits(gb, 8));

    skip_bits(gb, 8*payload_size);
    return 0;
}

static int more_rbsp_data(GetBitContext *gb)
{
    return get_bits_left(gb) > 0 && show_bits(gb, 8) != 0x80;
}

int ff_hevc_decode_nal_sei(HEVCContext *s)
{
    do {
        decode_nal_sei_message(s);
    } while (more_rbsp_data(&s->gb));
    return 0;
}
