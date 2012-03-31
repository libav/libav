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
#include "golomb.h"
#include "libavutil/attributes.h"

/**
 * Section 5.7
 */
#define INVERSE_RASTER_SCAN(a,b,c,d,e) ((e) ? ((a)%((d)/(b)))*(b) : ((a)/((d)/(b)))*(c))

static int decode_nal_slice(HEVCContext *s, GetBitContext *gb)
{
    SliceHeader *sh = av_mallocz(sizeof(SliceHeader));
    int slice_address_length = 0;

    av_log(s->avctx, AV_LOG_INFO, "Decoding slice\n");

    sh->first_slice_in_pic_flag = get_bits1(gb);
    if (!sh->first_slice_in_pic_flag) {
        slice_address_length = av_ceil_log2_c(s->sps->PicWidthInCtbs
                                              * s->sps->PicHeightInCtbs)
                               + s->pps->SliceGranularity;

        sh->slice_address = get_bits(gb, slice_address_length);
    }

    sh->slice_type = get_ue_golomb(gb);

    sh->entropy_slice_flag = get_bits1(gb);
    if (!sh->entropy_slice_flag) {
        sh->pps_id = get_ue_golomb(gb);
        if (sh->pps_id >= MAX_PPS_COUNT || s->pps_list[sh->pps_id] == NULL) {
            av_log(s->avctx, AV_LOG_ERROR, "PPS id out of range: %d\n", sh->pps_id);
            goto err;
        }
        s->pps = s->pps_list[sh->pps_id];
        s->sps = s->sps_list[s->pps->sps_id];

        if (s->pps->output_flag_present_flag) {
            sh->pic_output_flag = get_bits1(gb);
        }

        if (s->sps->separate_colour_plane_flag == 1) {
            sh->colour_plane_id = get_bits(gb, 2);
        }

        if (s->nal_unit_type == NAL_IDR_SLICE) {
            sh->idr_pic_id = get_ue_golomb(gb);
            sh->no_output_of_prior_pics_flag = get_bits1(gb);
        } else {
            av_log(s->avctx, AV_LOG_ERROR, "TODO: nal_unit_type != NAL_IDR_SLICE\n");
            goto err;
        }

        if (s->sps->sample_adaptive_offset_enabled_flag) {
            sh->slice_sao_interleaving_flag = get_bits1(gb);
            sh->slice_sample_adaptive_offset_flag = get_bits1(gb);
            if (sh->slice_sao_interleaving_flag
                && sh->slice_sample_adaptive_offset_flag) {
                sh->sao_cb_enable_flag = get_bits1(gb);
                sh->sao_cr_enable_flag = get_bits1(gb);
            }
        }

        if (s->sps->scaling_list_enable_flag
            || s->sps->deblocking_filter_in_aps_enabled_flag
            || (s->sps->sample_adaptive_offset_enabled_flag
                && !sh->slice_sao_interleaving_flag)
            || s->sps->adaptive_loop_filter_enabled_flag) {
            sh->aps_id = get_ue_golomb(gb);
            if (sh->aps_id >= MAX_APS_COUNT || s->aps_list[sh->aps_id] == NULL) {
                av_log(s->avctx, AV_LOG_ERROR, "APS id out of range: %d\n", sh->aps_id);
                goto err;
            }
            s->aps = s->aps_list[sh->aps_id];
        }

        if (sh->slice_type != I_SLICE) {
            av_log(s->avctx, AV_LOG_ERROR, "TODO: slice_type != I_SLICE\n");
            goto err;
        }
    }

    if (s->pps == NULL) {
        av_log(s->avctx, AV_LOG_ERROR, "No PPS active while decoding slice\n");
        goto err;
    }

    if (s->pps->cabac_init_present_flag && sh->slice_type != I_SLICE) {
        sh->cabac_init_flag = get_bits1(gb);
    }

    if (!sh->entropy_slice_flag) {
        sh->slice_qp_delta = get_se_golomb(gb);
        if (s->pps->deblocking_filter_control_present_flag) {
            av_log(s->avctx, AV_LOG_ERROR,
                   "TODO: deblocking_filter_control_present_flag\n");
            goto err;
        }
    }

#if !SUPPORT_ENCODER
    if (sh->slice_type != I_SLICE) {
#endif
        sh->max_num_merge_cand = get_ue_golomb(gb);
#if !SUPPORT_ENCODER
    }
#endif

    if (s->sps->adaptive_loop_filter_enabled_flag) {
        sh->slice_adaptive_loop_filter_flag = get_bits1(gb);
        av_log(s->avctx, AV_LOG_ERROR, "TODO: slice_adaptive_loop_filter_flag\n");
        goto err;
    }

    /*
    if (s->sps->seq_loop_filter_across_slices_enabled_flag
        && (sh->slice_adaptive_loop_filter_flag
            || sh->slice_sample_adaptive_offset_flag
            || !sh->disable_deblocking_filter_flag)) {
        av_log(s->avctx, AV_LOG_ERROR,
               "TODO: slice_loop_filter_across_slices_enabled_flag\n");
        goto err;
    }
    */
    return 0;

err:
    av_free(sh);
    return -1;
}

/**
 * 7.3.1: NAL unit syntax
 * @return AVERROR_INVALIDDATA if the packet is not a valid NAL unit,
 * 0 if the unit should be skipped, 1 otherwise
 */
static int decode_nal_unit(HEVCContext *s, GetBitContext *gb)
{
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
    GetBitContext gb;
    *data_size = 0;

    init_get_bits(&gb, avpkt->data, avpkt->size*8);

    av_log(s->avctx, AV_LOG_DEBUG, "=================\n");

    if (decode_nal_unit(s, &gb) <= 0) {
        av_log(s->avctx, AV_LOG_INFO, "Skipping NAL unit\n");
        goto end;
    }

    switch (s->nal_unit_type) {
    case NAL_SPS:
        ff_hevc_decode_nal_sps(s, &gb);
        break;
    case NAL_PPS:
        ff_hevc_decode_nal_pps(s, &gb);
        break;
    case NAL_APS:
        ff_hevc_decode_nal_aps(s, &gb);
        break;
    case NAL_SEI:
        ff_hevc_decode_nal_sei(s, &gb);
        break;
    case NAL_IDR_SLICE:
    case NAL_SLICE:
        decode_nal_slice(s, &gb);
        break;
    default:
        av_log(s->avctx, AV_LOG_INFO, "Skipping NAL unit\n");
        goto end;
    }

    av_log(s->avctx, AV_LOG_DEBUG, "%d bits left in unit\n", get_bits_left(&gb));
end:
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
            for (int j = 0; j < MAX_SHORT_TERM_RPS_COUNT; j++) {
                av_freep(&s->sps_list[i]->short_term_rps_list[j]);
            }
        }
        av_freep(&s->sps_list[i]);
    }

    for (int i = 0; i < MAX_PPS_COUNT; i++) {
        av_freep(&s->pps_list[i]);
    }

    for (int i = 0; i < MAX_APS_COUNT; i++) {
        av_freep(&s->aps_list[i]);
    }
    return 0;
}

static void hevc_decode_flush(AVCodecContext *avctx)
{
}

AVCodec ff_hevc_decoder = {
    .name                  = "hevc",
    .type                  = AVMEDIA_TYPE_VIDEO,
    .id                    = AV_CODEC_ID_HEVC,
    .priv_data_size        = sizeof(HEVCContext),
    .init                  = hevc_decode_init,
    .close                 = hevc_decode_free,
    .decode                = hevc_decode_frame,
    .capabilities          = 0,
    .flush                 = hevc_decode_flush,
    .long_name             = NULL_IF_CONFIG_SMALL("HEVC"),
};
