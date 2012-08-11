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

int ff_hevc_decode_nal_vps(HEVCContext *s)
{
    GetBitContext *gb = &s->gb;
    int vps_id = 0;
    struct VPS *vps = av_mallocz(sizeof(struct VPS));

    av_log(s->avctx, AV_LOG_DEBUG, "Decoding VPS\n");

    if (vps == NULL)
        return -1;
    vps->vps_max_temporal_layers = get_bits(gb, 3) + 1;
    vps->vps_max_layers = get_bits(gb, 5) + 1;
    vps_id = get_ue_golomb(gb);
    if (vps_id >= MAX_VPS_COUNT) {
        av_log(s->avctx, AV_LOG_ERROR, "VPS id out of range: %d\n", vps_id);
        av_free(vps);
        return -1;
    }

    for (int i = 0; i < vps->vps_max_temporal_layers; i++) {
        vps->vps_max_dec_pic_buffering[i] = get_ue_golomb(gb);
        vps->vps_num_reorder_pics[i] = get_ue_golomb(gb);
        vps->vps_max_latency_increase[i] = get_ue_golomb(gb);
    }

    av_free(s->vps_list[vps_id]);
    s->vps_list[vps_id] = vps;
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

    // Coded parameters

    sps->profile_space = get_bits(gb, 3);
    sps->profile_idc = get_bits(gb, 5);
    skip_bits(gb, 16); // constraint_flags
    sps->level_idc = get_bits(gb, 8);
    skip_bits(gb, 32); // profile_compability_flag[i]

    sps_id         = get_ue_golomb(gb);
    if (sps_id >= MAX_SPS_COUNT) {
        av_log(s->avctx, AV_LOG_ERROR, "SPS id out of range: %d\n", sps_id);
        goto err;
    }

    sps->vps_id = get_ue_golomb(gb);
    if (sps->vps_id >= MAX_VPS_COUNT) {
        av_log(s->avctx, AV_LOG_ERROR, "VPS id out of range: %d\n", sps->vps_id);
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

    sps->bit_depth[0] = get_ue_golomb(gb) + 8;
    sps->bit_depth[2] =
        sps->bit_depth[1] = get_ue_golomb(gb) + 8;

    sps->pcm_enabled_flag = get_bits1(gb);
    if (sps->pcm_enabled_flag) {
        sps->pcm.bit_depth_luma = get_bits(gb, 4) + 1;
        sps->pcm.bit_depth_luma = get_bits(gb, 4) + 1;
    }

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

    if (sps->pcm_enabled_flag) {
        sps->pcm.log2_min_pcm_coding_block_size          = get_ue_golomb(gb) + 3;
        sps->pcm.log2_diff_max_min_pcm_coding_block_size = get_ue_golomb(gb);
    }

    sps->max_transform_hierarchy_depth_inter = get_ue_golomb(gb);
    sps->max_transform_hierarchy_depth_intra = get_ue_golomb(gb);

    sps->scaling_list_enable_flag = get_bits1(gb);
    if (sps->scaling_list_enable_flag) {
        av_log(s->avctx, AV_LOG_ERROR, "TODO: scaling_list_enable_flag\n");
        goto err;
    }

    sps->asymmetric_motion_partitions_enabled_flag  = get_bits1(gb);
    sps->sample_adaptive_offset_enabled_flag        = get_bits1(gb);

    if (sps->pcm_enabled_flag)
        sps->pcm.loop_filter_disable_flag = get_bits1(gb);

    sps->temporal_id_nesting_flag = get_bits1(gb);

    sps->num_short_term_ref_pic_sets = get_ue_golomb(gb);
    for (int i = 0; i < sps->num_short_term_ref_pic_sets; i++) {
        if (ff_hevc_decode_short_term_rps(s, i, &sps->short_term_rps_list[i]) < 0)
            goto err;
    }

    sps->long_term_ref_pics_present_flag = get_bits1(gb);
    sps->sps_temporal_mvp_enabled_flag   = get_bits1(gb);

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

    // Inferred parameters

    sps->log2_ctb_size = sps->log2_min_coding_block_size
        + sps->log2_diff_max_min_coding_block_size;
    sps->pic_width_in_ctbs = ROUNDED_DIV(sps->pic_width_in_luma_samples,
                                  (1 << sps->log2_ctb_size));
    sps->pic_height_in_ctbs = ROUNDED_DIV(sps->pic_height_in_luma_samples,
                                  (1 << sps->log2_ctb_size));
    sps->pic_width_in_min_cbs = sps->pic_width_in_luma_samples / (1 << sps->log2_min_coding_block_size);
    sps->pic_height_in_min_cbs = sps->pic_height_in_luma_samples / (1 << sps->log2_min_coding_block_size);
    sps->log2_min_pu_size = sps->log2_min_coding_block_size - 1;

    av_free(s->sps_list[sps_id]);
    s->sps_list[sps_id] = sps;
    return 0;

err:
    for (int i = 0; i < MAX_SHORT_TERM_RPS_COUNT; i++)
        av_free(sps->short_term_rps_list[i]);

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
    pps->cabac_independant_flag = 0;
    pps->loop_filter_across_tiles_enabled_flag = 1;
    pps->num_tile_columns     = 1;
    pps->num_tile_rows        = 1;
    pps->uniform_spacing_flag = 1;


    // Coded parameters
    pps_id = get_ue_golomb(gb);
    if (pps_id >= MAX_PPS_COUNT) {
        av_log(s->avctx, AV_LOG_ERROR, "PPS id out of range: %d\n", pps_id);
        goto err;
    }
    pps->sps_id = get_ue_golomb(gb);
    if (pps->sps_id >= MAX_SPS_COUNT) {
        av_log(s->avctx, AV_LOG_ERROR, "SPS id out of range: %d\n", pps->sps_id);
        goto err;
    }
    sps = s->sps_list[pps->sps_id];

    pps->sign_data_hiding_flag = get_bits1(gb);

    pps->cabac_init_present_flag = get_bits1(gb);

    pps->num_ref_idx_l0_default_active = get_ue_golomb(gb) + 1;
    pps->num_ref_idx_l1_default_active = get_ue_golomb(gb) + 1;

    pps->pic_init_qp_minus26 = get_se_golomb(gb);

    pps->constrained_intra_pred_flag = get_bits1(gb);
    pps->transform_skip_enabled_flag = get_bits1(gb);

    pps->cu_qp_delta_enabled_flag = get_bits1(gb);
    if (pps->cu_qp_delta_enabled_flag)
        pps->diff_cu_qp_delta_depth = get_ue_golomb(gb);

    pps->cb_qp_offset = get_se_golomb(gb);
    pps->cr_qp_offset = get_se_golomb(gb);
    pps->pic_slice_level_chroma_qp_offsets_present_flag = get_bits1(gb);

    pps->weighted_pred_flag            = get_bits1(gb);
    pps->weighted_bipred_flag          = get_bits1(gb);
    pps->output_flag_present_flag      = get_bits1(gb);
#if REFERENCE_ENCODER_QUIRKS
    pps->dependant_slices_enabled_flag = get_bits1(gb);
    pps->transquant_bypass_enable_flag = get_bits1(gb);
#else
    pps->transquant_bypass_enable_flag = get_bits1(gb);
    pps->dependant_slices_enabled_flag = get_bits1(gb);
#endif

    pps->tiles_or_entropy_coding_sync_idc = get_bits(gb, 2);
    if (pps->tiles_or_entropy_coding_sync_idc == 1) {
        pps->num_tile_columns     = get_ue_golomb(gb) + 1;
        pps->num_tile_rows        = get_ue_golomb(gb) + 1;

        pps->column_width = av_malloc(pps->num_tile_columns * sizeof(int));
        pps->row_height   = av_malloc(pps->num_tile_rows * sizeof(int));
        if (pps->column_width == NULL || pps->row_height == NULL)
            goto err;

        pps->uniform_spacing_flag = get_bits1(gb);
        if (!pps->uniform_spacing_flag) {
            for (int i = 0; i < pps->num_tile_columns - 1; i++) {
                pps->column_width[i] = get_ue_golomb(gb);
            }
            for (int i = 0; i < pps->num_tile_rows - 1; i++) {
                pps->row_height[i] = get_ue_golomb(gb);
            }
        }
        pps->loop_filter_across_tiles_enabled_flag = get_bits1(gb);
    }
    else if (pps->tiles_or_entropy_coding_sync_idc == 2) {
        pps->cabac_independant_flag = get_bits1(gb);
    }

    pps->seq_loop_filter_across_slices_enabled_flag = get_bits1(gb);

    pps->deblocking_filter_control_present_flag = get_bits1(gb);
    if (pps->deblocking_filter_control_present_flag) {
        pps->deblocking_filter_override_enabled_flag = get_bits1(gb);
        pps->pps_disable_deblocking_filter_flag = get_bits1(gb);
        if (!pps->pps_disable_deblocking_filter_flag) {
            pps->beta_offset = get_se_golomb(gb) * 2;
            pps->tc_offset = get_se_golomb(gb) * 2;
        }
    }

    pps->pps_scaling_list_data_present_flag = get_bits1(gb);
    if (pps->pps_scaling_list_data_present_flag) {
        av_log(s->avctx, AV_LOG_ERROR, "TODO: scaling_list_data_present_flag\n");
        goto err;
    }

    pps->log2_parallel_merge_level = get_ue_golomb(gb) + 2;
    pps->slice_header_extension_present_flag = get_bits1(gb);

    // Inferred parameters
    pps->col_bd = av_malloc((pps->num_tile_columns + 1) * sizeof(int));
    pps->row_bd = av_malloc((pps->num_tile_rows + 1) * sizeof(int));
    if (pps->col_bd == NULL || pps->row_bd == NULL)
        goto err;

    if (pps->uniform_spacing_flag) {
        pps->column_width = av_malloc(pps->num_tile_columns * sizeof(int));
        pps->row_height   = av_malloc(pps->num_tile_rows * sizeof(int));
        if (pps->column_width == NULL || pps->row_height == NULL)
            goto err;

        for (int i = 0; i < pps->num_tile_columns; i++) {
            pps->column_width[i] =
                ((i + 1) * sps->pic_width_in_ctbs) / (pps->num_tile_columns) -
                (i * sps->pic_width_in_ctbs) / (pps->num_tile_columns);
        }

        for (int i = 0; i < pps->num_tile_rows; i++) {
            pps->row_height[i] =
                ((i + 1) * sps->pic_height_in_ctbs) / (pps->num_tile_rows) -
                (i * sps->pic_height_in_ctbs) / (pps->num_tile_rows);
        }
    }

    pps->col_bd[0] = 0;
    for (int i = 0; i < pps->num_tile_columns; i++)
        pps->col_bd[i+1] = pps->col_bd[i] + pps->column_width[i];

    pps->row_bd[0] = 0;
    for (int i = 0; i < pps->num_tile_rows; i++)
        pps->row_bd[i+1] = pps->row_bd[i] + pps->row_height[i];

    /**
     * 6.5
     */
    pps->ctb_addr_rs_to_ts = av_malloc(sps->pic_width_in_ctbs *
                                       sps->pic_height_in_ctbs * sizeof(int));
    pps->ctb_addr_ts_to_rs = av_malloc(sps->pic_width_in_ctbs *
                                       sps->pic_height_in_ctbs * sizeof(int));
    pps->tile_id = av_malloc(sps->pic_width_in_ctbs *
                             sps->pic_height_in_ctbs * sizeof(int));
    pps->min_cb_addr_zs = av_malloc(sps->pic_width_in_min_cbs *
                                    sps->pic_height_in_min_cbs * sizeof(int));
    if (pps->ctb_addr_rs_to_ts == NULL || pps->ctb_addr_ts_to_rs == NULL ||
        pps->tile_id == NULL || pps->min_cb_addr_zs == NULL)
        goto err;

    for (int ctb_addr_rs = 0;
         ctb_addr_rs < sps->pic_width_in_ctbs * sps->pic_height_in_ctbs;
         ctb_addr_rs++) {
        int tb_x = ctb_addr_rs % sps->pic_width_in_ctbs;
        int tb_y = ctb_addr_rs / sps->pic_width_in_ctbs;
        int tile_x = 0;
        int tile_y = 0;
        int val = 0;

        for (int i = 0; i < pps->num_tile_columns; i++) {
            if ( tb_x < pps->col_bd[i + 1] ) {
                tile_x = i;
                break;
            }
        }

        for (int i = 0; i < pps->num_tile_rows; i++) {
            if( tb_y < pps->row_bd[i + 1] ) {
                tile_y = i;
                break;
            }
        }

        for (int i = 0; i < tile_x; i++ )
            val += pps->row_height[tile_y] * pps->column_width[i];
        for (int i = 0; i < tile_y; i++ )
            val += sps->pic_width_in_ctbs * pps->row_height[i];

        val += (tb_y - pps->row_bd[tile_y]) * pps->column_width[tile_x] +
               tb_x - pps->col_bd[tile_x];

        pps->ctb_addr_rs_to_ts[ctb_addr_rs] = val;
        pps->ctb_addr_ts_to_rs[val] = ctb_addr_rs;
    }

    for (int j = 0, tile_id = 0; j < pps->num_tile_rows; j++)
        for (int i = 0; i < pps->num_tile_columns; i++, tile_id++)
            for (int y = pps->row_bd[j]; y < pps->row_bd[j+1]; y++)
                for (int x = pps->col_bd[j]; x < pps->col_bd[j+1]; x++)
                    pps->tile_id[pps->ctb_addr_rs_to_ts[y * sps->pic_width_in_ctbs + x]] =
                        tile_id;

    for (int y = 0; y < sps->pic_height_in_min_cbs; y++) {
        for (int x = 0; x < sps->pic_width_in_min_cbs; x++) {
            int tb_x = x >> sps->log2_diff_max_min_coding_block_size;
            int tb_y = y >> sps->log2_diff_max_min_coding_block_size;
            int ctb_addr_rs = sps->pic_width_in_ctbs * tb_y + tb_x;
            int val = pps->ctb_addr_rs_to_ts[ctb_addr_rs] <<
                      (sps->log2_diff_max_min_coding_block_size * 2);
            for (int i = 0; i < sps->log2_diff_max_min_coding_block_size; i++) {
                int m = 1 << i;
                val += (m & x ? m*m : 0) + (m & y ? 2*m*m : 0);
            }
            pps->min_cb_addr_zs[y * sps->pic_width_in_min_cbs + x] = val;
        }
    }

    av_free(s->pps_list[pps_id]);
    s->pps_list[pps_id] = pps;
    return 0;

err:
    av_free(pps->column_width);
    av_free(pps->row_height);
    av_free(pps->col_bd);
    av_free(pps->row_bd);
    av_free(pps->ctb_addr_rs_to_ts);
    av_free(pps->ctb_addr_ts_to_rs);
    av_free(pps->tile_id);
    av_free(pps->min_cb_addr_zs);

    av_free(pps);
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
