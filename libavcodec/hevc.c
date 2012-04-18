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

#include "libavutil/attributes.h"
#include "golomb.h"
#include "hevc.h"

/**
 * Section 5.7
 */
#define INVERSE_RASTER_SCAN(a,b,c,d,e) ((e) ? ((a)%((d)/(b)))*(b) : ((a)/((d)/(b)))*(c))

/**
 * 7.3.3
 */
static int decode_nal_slice_header(HEVCContext *s)
{
    GetBitContext *gb = &s->gb;
    SliceHeader *sh = &s->sh;
    int slice_address_length = 0;

    av_log(s->avctx, AV_LOG_INFO, "Decoding slice\n");


    // Coded parameters

    sh->first_slice_in_pic_flag = get_bits1(gb);
    if (!sh->first_slice_in_pic_flag) {
        slice_address_length = av_ceil_log2_c(s->sps->PicWidthInCtbs *
                                              s->sps->PicHeightInCtbs) +
                               s->pps->SliceGranularity;

        sh->slice_address = get_bits(gb, slice_address_length);
    }

    sh->slice_type = get_ue_golomb(gb);

    sh->entropy_slice_flag = get_bits1(gb);
    if (!sh->entropy_slice_flag) {
        sh->pps_id = get_ue_golomb(gb);
        if (sh->pps_id >= MAX_PPS_COUNT || s->pps_list[sh->pps_id] == NULL) {
            av_log(s->avctx, AV_LOG_ERROR, "PPS id out of range: %d\n", sh->pps_id);
            return -1;
        }
        s->pps = s->pps_list[sh->pps_id];
        s->sps = s->sps_list[s->pps->sps_id];

        if (s->pps->output_flag_present_flag)
            sh->pic_output_flag = get_bits1(gb);

        if (s->sps->separate_colour_plane_flag == 1)
            sh->colour_plane_id = get_bits(gb, 2);

        if (s->nal_unit_type == NAL_IDR_SLICE) {
            sh->idr_pic_id                   = get_ue_golomb(gb);
            sh->no_output_of_prior_pics_flag = get_bits1(gb);
        } else {
            av_log(s->avctx, AV_LOG_ERROR, "TODO: nal_unit_type != NAL_IDR_SLICE\n");
            return -1;
        }

        if (s->sps->sample_adaptive_offset_enabled_flag) {
            sh->slice_sao_interleaving_flag       = get_bits1(gb);
            sh->slice_sample_adaptive_offset_flag = get_bits1(gb);
            if (sh->slice_sao_interleaving_flag
                && sh->slice_sample_adaptive_offset_flag) {
                sh->sao_cb_enable_flag = get_bits1(gb);
                sh->sao_cr_enable_flag = get_bits1(gb);
            }
        }

        if (s->sps->scaling_list_enable_flag ||
            s->sps->deblocking_filter_in_aps_enabled_flag ||
            (s->sps->sample_adaptive_offset_enabled_flag &&
             !sh->slice_sao_interleaving_flag) ||
#ifdef REFERENCE_ENCODER_QUIRKS
            s->sps->sample_adaptive_offset_enabled_flag ||
#endif
            s->sps->adaptive_loop_filter_enabled_flag) {
            sh->aps_id = get_ue_golomb(gb);
            if (sh->aps_id >= MAX_APS_COUNT || s->aps_list[sh->aps_id] == NULL) {
                av_log(s->avctx, AV_LOG_ERROR, "APS id out of range: %d\n", sh->aps_id);
                return -1;
            }
            s->aps = s->aps_list[sh->aps_id];
        }

        if (sh->slice_type != I_SLICE) {
            av_log(s->avctx, AV_LOG_ERROR, "TODO: slice_type != I_SLICE\n");
            return -1;
        }
    }

    if (s->pps == NULL) {
        av_log(s->avctx, AV_LOG_ERROR, "No PPS active while decoding slice\n");
        return -1;
    }

    if (s->pps->cabac_init_present_flag && sh->slice_type != I_SLICE)
        sh->cabac_init_flag = get_bits1(gb);

    if (!sh->entropy_slice_flag) {
        sh->slice_qp_delta = get_se_golomb(gb);
        if (s->pps->deblocking_filter_control_present_flag) {
            av_log(s->avctx, AV_LOG_ERROR,
                   "TODO: deblocking_filter_control_present_flag\n");
            return -1;
        }
    }

#if !REFERENCE_ENCODER_QUIRKS
    if (sh->slice_type != I_SLICE)
#endif
        sh->max_num_merge_cand = 5 - get_ue_golomb(gb);

    if (s->sps->adaptive_loop_filter_enabled_flag) {
        sh->slice_adaptive_loop_filter_flag = get_bits1(gb);
        av_log(s->avctx, AV_LOG_ERROR, "TODO: slice_adaptive_loop_filter_flag\n");
        return -1;
    }

#if !REFERENCE_ENCODER_QUIRKS
    if (s->sps->seq_loop_filter_across_slices_enabled_flag
        && (sh->slice_adaptive_loop_filter_flag ||
            sh->slice_sample_adaptive_offset_flag ||
            !sh->disable_deblocking_filter_flag)) {
        av_log(s->avctx, AV_LOG_ERROR,
               "TODO: slice_loop_filter_across_slices_enabled_flag\n");
        return -1;
    } else
#else
        sh->slice_loop_filter_across_slices_enabled_flag =
            s->sps->seq_loop_filter_across_slices_enabled_flag;
#endif

#if REFERENCE_ENCODER_QUIRKS
    if (!sh->entropy_slice_flag)
        sh->tile_marker_flag = get_bits1(gb);

    align_get_bits(gb);
#endif

    // Inferred parameters
    sh->slice_qp = 26 + s->pps->pic_init_qp_minus26 + sh->slice_qp_delta;
    sh->slice_ctb_addr_rs = sh->slice_address >> s->pps->SliceGranularity;
    sh->slice_cb_addr_zs = sh->slice_address <<
                        ((s->sps->log2_diff_max_min_coding_block_size -
                          s->pps->SliceGranularity) << 1);

    return 0;
}

/**
 * 7.3.4.2
 */
static int sao_offset_cabac(HEVCContext *s, int rx, int ry, int c_idx)
{
    int offset = ff_hevc_cabac_decode(s, SAO_TYPE_IDX);
    av_log(s->avctx, AV_LOG_DEBUG, "sao_offset: %d\n", offset);
    //TODO
    return 0;
}

/**
 * 7.3.4.1
 */
static int sao_unit_cabac(HEVCContext *s, int rx, int ry, int c_idx)
{
    if (rx > 0 && s->ctb_addr_in_slice != 0)
        ff_hevc_cabac_decode(s, SAO_MERGE_LEFT_FLAG);
    if (!s->sao_merge_left_flag) {
        if (ry > 0 && (s->addr_up > 0 ||
                       s->sh.slice_loop_filter_across_slices_enabled_flag))
            ff_hevc_cabac_decode(s, SAO_MERGE_UP_FLAG);
        if (!s->sao_merge_up_flag)
            sao_offset_cabac(s, rx, rx, c_idx);
    }
    return 0;
}

/**
 * 6.5.1
 */
static int ctb_addr_rs_to_ts(HEVCContext *s, int ctb_addr_rs)
{
    int ret = 0;
    int tb_x = ctb_addr_rs % s->sps->PicWidthInCtbs;
    int tb_y = ctb_addr_rs / s->sps->PicWidthInCtbs;
    int tile_x = 0;
    int tile_y = 0;

    for (int j = 0; j < s->sps->num_tile_columns; j++) {
        if ( tb_x < s->sps->col_bd[j + 1] ) {
            tile_x = j;
            break;
        }
    }

    for (int j = 0; j < s->sps->num_tile_rows; j++) {
        if( tb_y < s->sps->row_bd[j + 1] ) {
            tile_y = j;
            break;
        }
    }

    ret = ctb_addr_rs - tb_x;
    for (int j = 0; j < tile_x; j++ )
        ret += s->sps->row_height[tile_y] * s->sps->column_width[j];
    ret += (tb_y - s->sps->row_bd[tile_y]) * s->sps->column_width[tile_y] +
           tb_x - s->sps->col_bd[tile_x];

    return ret;
}

static int min_cb_addr_zs(HEVCContext *s, int x, int y)
{
    int tb_x = x >> s->sps->log2_diff_max_min_coding_block_size;
    int tb_y = y >> s->sps->log2_diff_max_min_coding_block_size;
    int ctb_addr_rs = s->sps->PicWidthInCtbs * tb_y + tb_x;
    int ret = ctb_addr_rs_to_ts(s, ctb_addr_rs) << s->sps->log2_diff_max_min_coding_block_size;
    int p = 0;
    for (int i = 0; i < s->sps->log2_diff_max_min_coding_block_size; i++) {
        int m = 1 << i;
        p += (m & x ? m*m : 0) + (m & y ? 2*m*m : 0);
        ret += p;
    }
    return ret;
}

/**
 * 7.3.5
 */
static void coding_tree(HEVCContext *s, int x0, int y0, int log2_cb_size, int cb_depth)
{
    if ((x0 + (1 << log2_cb_size) <= s->sps->pic_width_in_luma_samples) &&
        (y0 + (1 << log2_cb_size) <= s->sps->pic_height_in_luma_samples) &&
        min_cb_addr_zs(s, x0 >> s->sps->log2_min_coding_block_size,
                       y0 >> s->sps->log2_min_coding_block_size) >= s->sh.slice_cb_addr_zs &&
        log2_cb_size > s->sps->log2_min_coding_block_size && s->num_pcm_block == 0)
        {
            int split_coding_unit_flag = ff_hevc_cabac_decode(s, SPLIT_CODING_UNIT_FLAG);
            av_log(s->avctx, AV_LOG_DEBUG, "split_coding_unit_flag: %d\n",
                   split_coding_unit_flag);
        }
}

/**
 * 7.3.4
 */
static int decode_nal_slice_data(HEVCContext *s)
{
    int ctb_size = 1 << s->sps->Log2CtbSize;
    int ctb_addr_rs = s->sh.slice_ctb_addr_rs;
    int ctb_addr_ts = ctb_addr_rs_to_ts(s, ctb_addr_rs);
    int x_ctb, y_ctb;

    do {
        x_ctb = INVERSE_RASTER_SCAN(ctb_addr_rs, ctb_size, ctb_size, s->sps->pic_width_in_luma_samples, 0);
        y_ctb = INVERSE_RASTER_SCAN(ctb_addr_rs, ctb_size, ctb_size, s->sps->pic_width_in_luma_samples, 1);
        s->num_pcm_block = 0;
        s->ctb_addr_in_slice = ctb_addr_rs - (s->sh.slice_address >> s->pps->SliceGranularity);
        s->addr_up = ctb_addr_rs - s->sps->PicWidthInCtbs;
        if (s->sh.slice_sao_interleaving_flag) {
            if (s->sh.slice_sample_adaptive_offset_flag)
                sao_unit_cabac(s, x_ctb, y_ctb, 0);
            if (s->sh.sao_cb_enable_flag)
                sao_unit_cabac(s, x_ctb, y_ctb, 1);
            if (s->sh.sao_cr_enable_flag)
                sao_unit_cabac(s, x_ctb, y_ctb, 2);
        }
        coding_tree(s, x_ctb, y_ctb, s->sps->Log2CtbSize, 0);
        ctb_addr_ts++;
        //TODO
    } while (0);

    return 0;
}

static int decode_nal_slice(HEVCContext *s)
{
    int ret = 0;
    if ((ret = decode_nal_slice_header(s)) < 0)
        return ret;

    ff_hevc_cabac_init(s);

    return decode_nal_slice_data(s);
}

/**
 * 7.3.1: NAL unit syntax
 * @return AVERROR_INVALIDDATA if the packet is not a valid NAL unit,
 * 0 if the unit should be skipped, 1 otherwise
 */
static int decode_nal_unit(HEVCContext *s)
{
    GetBitContext *gb = &s->gb;

    if (get_bits1(gb) != 0) {
        return AVERROR_INVALIDDATA;
    }
    s->nal_ref_flag = get_bits1(gb);
    s->nal_unit_type = get_bits(gb, 6);

    s->temporal_id = get_bits(gb, 3);

    av_log(s->avctx, AV_LOG_DEBUG,
           "nal_ref_flag: %d, nal_unit_type: %d, temporal_id: %d\n",
           s->nal_ref_flag, s->nal_unit_type, s->temporal_id);

    return (get_bits(gb, 5) == 1);
}

/**
 * Note: avpkt->data must contain exactly one NAL unit
 */
static int hevc_decode_frame(AVCodecContext *avctx, void *data, int *data_size,
                             AVPacket *avpkt)
{
    HEVCContext *s = avctx->priv_data;
    GetBitContext *gb = &s->gb;

    *data_size = 0;

    init_get_bits(gb, avpkt->data, avpkt->size*8);

    av_log(s->avctx, AV_LOG_DEBUG, "=================\n");

    if (decode_nal_unit(s) <= 0) {
        av_log(s->avctx, AV_LOG_INFO, "Skipping NAL unit\n");
        return avpkt->size;
    }

    switch (s->nal_unit_type) {
    case NAL_SPS:
        ff_hevc_decode_nal_sps(s);
        break;
    case NAL_PPS:
        ff_hevc_decode_nal_pps(s);
        break;
    case NAL_APS:
        ff_hevc_decode_nal_aps(s);
        break;
    case NAL_SEI:
        ff_hevc_decode_nal_sei(s);
        break;
    case NAL_IDR_SLICE:
    case NAL_SLICE:
        decode_nal_slice(s);
        break;
    default:
        av_log(s->avctx, AV_LOG_INFO, "Skipping NAL unit\n");
        return avpkt->size;
    }

    av_log(s->avctx, AV_LOG_DEBUG, "%d bits left in unit\n", get_bits_left(gb));
    return avpkt->size;
}

static av_cold int hevc_decode_init(AVCodecContext *avctx)
{
    HEVCContext *s = avctx->priv_data;

    s->avctx = avctx;
    memset(s->sps_list, 0, sizeof(s->sps_list));
    memset(s->pps_list, 0, sizeof(s->pps_list));
    memset(s->aps_list, 0, sizeof(s->aps_list));
    return 0;
}

static av_cold int hevc_decode_free(AVCodecContext *avctx)
{
    HEVCContext *s = avctx->priv_data;

    for (int i = 0; i < MAX_SPS_COUNT; i++) {
        if (s->sps_list[i]) {
            for (int j = 0; j < MAX_SHORT_TERM_RPS_COUNT; j++)
                av_freep(&s->sps_list[i]->short_term_rps_list[j]);
            av_freep(&s->sps_list[i]->column_width);
            av_freep(&s->sps_list[i]->row_height);
            av_freep(&s->sps_list[i]->col_bd);
            av_freep(&s->sps_list[i]->row_bd);
        }
        av_freep(&s->sps_list[i]);
    }

    for (int i = 0; i < MAX_PPS_COUNT; i++)
        av_freep(&s->pps_list[i]);

    for (int i = 0; i < MAX_APS_COUNT; i++)
        av_freep(&s->aps_list[i]);
    return 0;
}

static void hevc_decode_flush(AVCodecContext *avctx)
{
}

AVCodec ff_hevc_decoder = {
    .name           = "hevc",
    .type           = AVMEDIA_TYPE_VIDEO,
    .id             = AV_CODEC_ID_HEVC,
    .priv_data_size = sizeof(HEVCContext),
    .init           = hevc_decode_init,
    .close          = hevc_decode_free,
    .decode         = hevc_decode_frame,
    .capabilities   = 0,
    .flush          = hevc_decode_flush,
    .long_name      = NULL_IF_CONFIG_SMALL("HEVC"),
};
