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
#include "libavutil/common.h"
#include "libavutil/pixdesc.h"
#include "libavutil/internal.h"
#include "cabac_functions.h"
#include "golomb.h"
#include "hevcdata.h"
#include "hevc.h"
#include "libavutil/opt.h"
#include "libavutil/md5.h"
#include "libavutil/atomic.h"

/**
 * NOTE: Each function hls_foo correspond to the function foo in the
 * specification (HLS stands for High Level Syntax).
 */

/**
 * Section 5.7
 */
//#define POC_DISPLAY_MD5
#define WPP1
static void pic_arrays_free(HEVCContext *s)
{
    int i;
    HEVCSharedContext *sc = s->HEVCsc;
    av_freep(&sc->sao);
    av_freep(&sc->deblock);

    av_freep(&sc->split_cu_flag);
    av_freep(&sc->skip_flag);

    av_freep(&sc->left_ct_depth);
    av_freep(&sc->top_ct_depth);

    av_freep(&sc->left_ipm);
    av_freep(&sc->top_ipm);
    av_freep(&sc->horizontal_bs);
    av_freep(&sc->vertical_bs);

    av_freep(&sc->cbf_luma);
    av_freep(&sc->is_pcm);

    av_freep(&sc->qp_y_tab);

    av_freep(&sc->sh.entry_point_offset);
    av_freep(&sc->sh.size);
    av_freep(&sc->sh.offset);

    for (i = 0; i < FF_ARRAY_ELEMS(sc->DPB); i++) {
        av_freep(&sc->DPB[i].tab_mvf);
    }
}

static int pic_arrays_init(HEVCContext *s)
{
    int i;
    HEVCSharedContext *sc = s->HEVCsc;
    int pic_size = sc->sps->pic_width_in_luma_samples * sc->sps->pic_height_in_luma_samples;
    int pic_size_in_ctb = pic_size>>(sc->sps->log2_min_coding_block_size<<1);
    int ctb_count = sc->sps->pic_width_in_ctbs * sc->sps->pic_height_in_ctbs;
    int pic_width_in_min_pu  = sc->sps->pic_width_in_min_cbs * 4;
    int pic_height_in_min_pu = sc->sps->pic_height_in_min_cbs * 4;
    sc->bs_width = sc->sps->pic_width_in_luma_samples >> 3;
    sc->bs_height = sc->sps->pic_height_in_luma_samples >> 3;
    sc->sao = av_mallocz(ctb_count * sizeof(*sc->sao));
    sc->deblock = av_mallocz(ctb_count * sizeof(DBParams));
    sc->split_cu_flag = av_malloc(pic_size);
    if (!sc->sao || !sc->deblock || !sc->split_cu_flag)
        goto fail;

    sc->skip_flag     = av_malloc(pic_size_in_ctb);
    sc->left_ct_depth = av_malloc(sc->sps->pic_height_in_min_cbs);
    sc->top_ct_depth  = av_malloc(sc->sps->pic_width_in_min_cbs);
    if (!sc->skip_flag || !sc->left_ct_depth || !sc->top_ct_depth)
        goto fail;

    sc->left_ipm = av_malloc(pic_height_in_min_pu);
    sc->top_ipm  = av_malloc(pic_width_in_min_pu);
    if (!sc->left_ipm || !sc->top_ipm)
        goto fail;

    sc->cbf_luma = av_malloc(pic_width_in_min_pu * pic_height_in_min_pu);
    sc->is_pcm   = av_malloc(pic_width_in_min_pu * pic_height_in_min_pu);
    if (!sc->cbf_luma ||!sc->is_pcm)
        goto fail;

    sc->qp_y_tab = av_malloc(pic_size_in_ctb*sizeof(int8_t));
    if (!sc->qp_y_tab)
        goto fail;

    for (i = 0; i < FF_ARRAY_ELEMS(sc->DPB); i++) {
        sc->DPB[i].tab_mvf = av_malloc(pic_width_in_min_pu  *
                                       pic_height_in_min_pu *
                                       sizeof(*sc->DPB[i].tab_mvf));
        if (!sc->DPB[i].tab_mvf)
            goto fail;
    }

    sc->horizontal_bs = av_mallocz(2 * sc->bs_width * sc->bs_height);
    sc->vertical_bs   = av_mallocz(2 * sc->bs_width * sc->bs_height);
    if (!sc->horizontal_bs || !sc->vertical_bs)
        goto fail;
    return 0;
fail:
    pic_arrays_free(s);
    return AVERROR(ENOMEM);
}

static void pred_weight_table(HEVCSharedContext *sc, GetBitContext *gb)
{
    int i = 0;
    int j = 0;
    int delta_chroma_log2_weight_denom;
    uint8_t luma_weight_l0_flag[16];
    uint8_t chroma_weight_l0_flag[16];
    uint8_t luma_weight_l1_flag[16];
    uint8_t chroma_weight_l1_flag[16];

    sc->sh.luma_log2_weight_denom = get_ue_golomb(gb);
    if (sc->sps->chroma_format_idc != 0) {
        delta_chroma_log2_weight_denom = get_se_golomb(gb);
        sc->sh.chroma_log2_weight_denom = av_clip_c(sc->sh.luma_log2_weight_denom + delta_chroma_log2_weight_denom, 0, 7);
    }
    for (i = 0; i < sc->sh.num_ref_idx_l0_active; i++) {
        luma_weight_l0_flag[i] = get_bits1(gb);
        if (!luma_weight_l0_flag[i]) {
            sc->sh.luma_weight_l0[i] = 1 << sc->sh.luma_log2_weight_denom;
            sc->sh.luma_offset_l0[i] = 0;
        }
    }
    if (sc->sps->chroma_format_idc != 0) { //fix me ! invert "if" and "for"
        for (i = 0; i < sc->sh.num_ref_idx_l0_active; i++) {
            chroma_weight_l0_flag[i] = get_bits1(gb);
        }
    } else {
        for (i = 0; i < sc->sh.num_ref_idx_l0_active; i++) {
            chroma_weight_l0_flag[i] = 0;
        }
    }
    for (i = 0; i < sc->sh.num_ref_idx_l0_active; i++) {
        if (luma_weight_l0_flag[i]) {
            int delta_luma_weight_l0 = get_se_golomb(gb);
            sc->sh.luma_weight_l0[i]  = (1 << sc->sh.luma_log2_weight_denom) + delta_luma_weight_l0;
            sc->sh.luma_offset_l0[i]  = get_se_golomb(gb);
        }
        if (chroma_weight_l0_flag[i]) {
            for (j = 0; j < 2; j++) {
                int delta_chroma_weight_l0   = get_se_golomb(gb);
                int delta_chroma_offset_l0   = get_se_golomb(gb);
                sc->sh.chroma_weight_l0[i][j] = (1 << sc->sh.chroma_log2_weight_denom) + delta_chroma_weight_l0;
                sc->sh.chroma_offset_l0[i][j] = av_clip_c((delta_chroma_offset_l0 - ((128 * sc->sh.chroma_weight_l0[i][j])
                                                                                     >>  sc->sh.chroma_log2_weight_denom) + 128), -128, 127);
            }
        } else {
            sc->sh.chroma_weight_l0[i][0] = 1 << sc->sh.chroma_log2_weight_denom;
            sc->sh.chroma_offset_l0[i][0] = 0;
            sc->sh.chroma_weight_l0[i][1] = 1 << sc->sh.chroma_log2_weight_denom;
            sc->sh.chroma_offset_l0[i][1] = 0;
        }
    }
    if (sc->sh.slice_type == B_SLICE) {
        for (i = 0; i < sc->sh.num_ref_idx_l1_active; i++) {
            luma_weight_l1_flag[i] = get_bits1(gb);
            if (!luma_weight_l1_flag[i]) {
                sc->sh.luma_weight_l1[i] = 1 << sc->sh.luma_log2_weight_denom;
                sc->sh.luma_offset_l1[i] = 0;
            }
        }
        if (sc->sps->chroma_format_idc != 0) {
            for (i = 0; i < sc->sh.num_ref_idx_l1_active; i++) {
                chroma_weight_l1_flag[i] = get_bits1(gb);
            }
        } else {
            for (i = 0; i < sc->sh.num_ref_idx_l1_active; i++) {
                chroma_weight_l1_flag[i] = 0;
            }
        }
        for (i = 0; i < sc->sh.num_ref_idx_l1_active; i++) {
            if (luma_weight_l1_flag[i]) {
                int delta_luma_weight_l1 = get_se_golomb(gb);
                sc->sh.luma_weight_l1[i]  = (1 << sc->sh.luma_log2_weight_denom) + delta_luma_weight_l1;
                sc->sh.luma_offset_l1[i]  = get_se_golomb(gb);
            }
            if (chroma_weight_l1_flag[i]) {
                for (j = 0; j < 2; j++) {
                    int delta_chroma_weight_l1   = get_se_golomb(gb);
                    int delta_chroma_offset_l1   = get_se_golomb(gb);
                    sc->sh.chroma_weight_l1[i][j] = (1 << sc->sh.chroma_log2_weight_denom) + delta_chroma_weight_l1;
                    sc->sh.chroma_offset_l1[i][j] = av_clip_c((delta_chroma_offset_l1 - ((128 * sc->sh.chroma_weight_l1[i][j])
                                                                                         >> sc->sh.chroma_log2_weight_denom) + 128), -128, 127);
                }
            } else {
                sc->sh.chroma_weight_l1[i][0] = 1 << sc->sh.chroma_log2_weight_denom;
                sc->sh.chroma_offset_l1[i][0] = 0;
                sc->sh.chroma_weight_l1[i][1] = 1 << sc->sh.chroma_log2_weight_denom;
                sc->sh.chroma_offset_l1[i][1] = 0;
            }
        }
    }
}

static int hls_slice_header(HEVCContext *s)
{
    int i, ret, j;
    GetBitContext *gb = s->HEVClc->gb;
    HEVCSharedContext *sc = s->HEVCsc;
    SliceHeader *sh = &sc->sh;
    int slice_address_length = 0;

    // initial values
    sh->beta_offset = 0;
    sh->tc_offset = 0;

    // Coded parameters

    sh->first_slice_in_pic_flag = get_bits1(gb);
    if (sc->nal_unit_type == NAL_IDR_W_RADL && sh->first_slice_in_pic_flag) {
        ff_hevc_clear_refs(s);
    }
    if ((sc->nal_unit_type == NAL_IDR_W_RADL || sc->nal_unit_type == NAL_IDR_N_LP) &&
        sh->first_slice_in_pic_flag) {
        sc->seq_decode = (sc->seq_decode + 1) & 0xff;
    }
    if (sc->nal_unit_type >= 16 && sc->nal_unit_type <= 23)
        sh->no_output_of_prior_pics_flag = get_bits1(gb);

    sh->pps_id = get_ue_golomb(gb);
    if (sh->pps_id >= MAX_PPS_COUNT || !sc->pps_list[sh->pps_id]) {
        av_log(s->avctx, AV_LOG_ERROR, "PPS id out of range: %d\n", sh->pps_id);
        return AVERROR_INVALIDDATA;
    }
    sc->pps = sc->pps_list[sh->pps_id];
    if (sc->sps != sc->sps_list[sc->pps->sps_id]) {

        sc->sps = sc->sps_list[sc->pps->sps_id];
        sc->vps = sc->vps_list[sc->sps->vps_id];

        //TODO: Handle switching between different SPS better
        pic_arrays_free(s);
        ret = pic_arrays_init(s);
        if (ret < 0)
            return AVERROR(ENOMEM);

        s->avctx->width = sc->sps->pic_width_in_luma_samples;
        s->avctx->height = sc->sps->pic_height_in_luma_samples;
        if (sc->sps->chroma_format_idc == 0 || sc->sps->separate_colour_plane_flag) {
            av_log(s->avctx, AV_LOG_ERROR,
                   "TODO: sc->sps->chroma_format_idc == 0 || "
                   "sc->sps->separate_colour_plane_flag\n");
            return AVERROR_PATCHWELCOME;
        }

        if (sc->sps->chroma_format_idc == 1) {
            switch (sc->sps->bit_depth) {
            case 8:
                s->avctx->pix_fmt = PIX_FMT_YUV420P;
                break;
            case 9:
                s->avctx->pix_fmt = PIX_FMT_YUV420P9;
                break;
            case 10:
                s->avctx->pix_fmt = PIX_FMT_YUV420P10;
                break;
            }
        } else {
            av_log(s->avctx, AV_LOG_ERROR, "non-4:2:0 support is currently unspecified.\n");
            return AVERROR_PATCHWELCOME;
        }
        sc->sps->hshift[0] = sc->sps->vshift[0] = 0;
        sc->sps->hshift[2] =
        sc->sps->hshift[1] = av_pix_fmt_descriptors[s->avctx->pix_fmt].log2_chroma_w;
        sc->sps->vshift[2] =
        sc->sps->vshift[1] = av_pix_fmt_descriptors[s->avctx->pix_fmt].log2_chroma_h;

        sc->sps->pixel_shift = sc->sps->bit_depth > 8;

        ff_hevc_pred_init(&sc->hpc, sc->sps->bit_depth);
        ff_hevc_dsp_init(&sc->hevcdsp, sc->sps->bit_depth);

        ff_videodsp_init(&sc->vdsp, sc->sps->bit_depth);
    }
    sh->dependent_slice_segment_flag = 0;
    if (!sh->first_slice_in_pic_flag) {
        if (sc->pps->dependent_slice_segments_enabled_flag)
            sh->dependent_slice_segment_flag = get_bits1(gb);

        slice_address_length = av_ceil_log2_c(sc->sps->pic_width_in_ctbs *
                                              sc->sps->pic_height_in_ctbs);
        sh->slice_address = get_bits(gb, slice_address_length);
    } else {
        sh->slice_address = 0;
    }

    if (!sh->dependent_slice_segment_flag) {
        for(i = 0; i < sc->pps->num_extra_slice_header_bits; i++)
            skip_bits(gb, 1); // slice_reserved_undetermined_flag[]
        sh->slice_type = get_ue_golomb(gb);
        if (sc->pps->output_flag_present_flag)
            sh->pic_output_flag = get_bits1(gb);

        if (sc->sps->separate_colour_plane_flag == 1)
            sh->colour_plane_id = get_bits(gb, 2);

        if (sc->nal_unit_type != NAL_IDR_W_RADL && sc->nal_unit_type != NAL_IDR_N_LP) {
            int short_term_ref_pic_set_sps_flag;
            sh->pic_order_cnt_lsb = get_bits(gb, sc->sps->log2_max_poc_lsb);
            ff_hevc_compute_poc(s, sh->pic_order_cnt_lsb);
            short_term_ref_pic_set_sps_flag = get_bits1(gb);
            if (!short_term_ref_pic_set_sps_flag) {
                ff_hevc_decode_short_term_rps(s->HEVClc, sc->sps->num_short_term_ref_pic_sets, sc->sps);
                sh->short_term_rps = &sc->sps->short_term_rps_list[sc->sps->num_short_term_ref_pic_sets];
            } else {
                int numbits = 0;
                int short_term_ref_pic_set_idx;
                while ((1 << numbits) < sc->sps->num_short_term_ref_pic_sets)
                    numbits++;
                if (numbits > 0)
                    short_term_ref_pic_set_idx = get_bits(gb, numbits);
                else
                    short_term_ref_pic_set_idx = 0;
                sh->short_term_rps = &sc->sps->short_term_rps_list[short_term_ref_pic_set_idx];
            }
            sh->long_term_rps.num_long_term_sps = 0;
            sh->long_term_rps.num_long_term_pics = 0;
            if (sc->sps->long_term_ref_pics_present_flag) {
                if( sc->sps->num_long_term_ref_pics_sps > 0 )
                    sh->long_term_rps.num_long_term_sps = get_ue_golomb(gb);
                sh->long_term_rps.num_long_term_pics = get_ue_golomb(gb);
                for( i = 0; i < sh->long_term_rps.num_long_term_sps + sh->long_term_rps.num_long_term_pics; i++ ) {
                    if( i < sh->long_term_rps.num_long_term_sps ) {
                        uint8_t lt_idx_sps = 0;
                        if( sc->sps->num_long_term_ref_pics_sps > 1 )
                            lt_idx_sps = get_bits(gb, av_ceil_log2_c(sc->sps->num_long_term_ref_pics_sps));
                        sh->long_term_rps.PocLsbLt[ i ] = sc->sps->lt_ref_pic_poc_lsb_sps[ lt_idx_sps ];
                        sh->long_term_rps.UsedByCurrPicLt[ i ] = sc->sps->used_by_curr_pic_lt_sps_flag[ lt_idx_sps ];
                    } else {
                        sh->long_term_rps.PocLsbLt[ i ] = get_bits(gb, sc->sps->log2_max_poc_lsb);
                        sh->long_term_rps.UsedByCurrPicLt[ i ] = get_bits1(gb);
                    }
                    sh->long_term_rps.delta_poc_msb_present_flag[ i ] = get_bits1(gb);
                    if( sh->long_term_rps.delta_poc_msb_present_flag[ i ] == 1)
                        if( i == 0 || i == sh->long_term_rps.num_long_term_sps )
                            sh->long_term_rps.DeltaPocMsbCycleLt[ i ] = get_ue_golomb(gb);
                        else
                            sh->long_term_rps.DeltaPocMsbCycleLt[ i ] = get_ue_golomb(gb) + sh->long_term_rps.DeltaPocMsbCycleLt[ i - 1 ];
                }
            }
            if (sc->sps->sps_temporal_mvp_enabled_flag)
                sh->slice_temporal_mvp_enabled_flag = get_bits1(gb);
            else
                sh->slice_temporal_mvp_enabled_flag = 0;
        } else {
            sc->sh.short_term_rps = NULL;
            sc->poc = 0;
        }
        if (sc->temporal_id == 0)
            sc->pocTid0 = sc->poc;
//        av_log(s->avctx, AV_LOG_INFO, "Decode  : POC %d NAL %d\n", s->poc, s->nal_unit_type);
        if (!sc->pps) {
            av_log(s->avctx, AV_LOG_ERROR, "No PPS active while decoding slice\n");
            return AVERROR_INVALIDDATA;
        }

        if (sc->sps->sample_adaptive_offset_enabled_flag) {
            sh->slice_sample_adaptive_offset_flag[0] = get_bits1(gb);
            sh->slice_sample_adaptive_offset_flag[2] =
            sh->slice_sample_adaptive_offset_flag[1] = get_bits1(gb);
        }

        sh->num_ref_idx_l0_active = 0;
        sh->num_ref_idx_l1_active = 0;
        if (sh->slice_type == P_SLICE || sh->slice_type == B_SLICE) {
            int NumPocTotalCurr;
            sh->num_ref_idx_l0_active = sc->pps->num_ref_idx_l0_default_active;
            if (sh->slice_type == B_SLICE)
                sh->num_ref_idx_l1_active = sc->pps->num_ref_idx_l1_default_active;
            sh->num_ref_idx_active_override_flag = get_bits1(gb);

            if (sh->num_ref_idx_active_override_flag) {
                sh->num_ref_idx_l0_active = get_ue_golomb(gb) + 1;
                if (sh->slice_type == B_SLICE)
                    sh->num_ref_idx_l1_active = get_ue_golomb(gb) + 1;
            }
            sh->ref_pic_list_modification_flag_lx[0] = 0;
            sh->ref_pic_list_modification_flag_lx[1] = 0;
            NumPocTotalCurr = ff_hevc_get_NumPocTotalCurr(s);
            if (sc->pps->lists_modification_present_flag && NumPocTotalCurr > 1) {
                sh->ref_pic_list_modification_flag_lx[0] = get_bits1(gb);
                if( sh->ref_pic_list_modification_flag_lx[0] == 1 )
                    for (i = 0; i < sh->num_ref_idx_l0_active; i++)
                        sh->list_entry_lx[0][i] = get_bits(gb, av_ceil_log2_c(NumPocTotalCurr));
                if (sh->slice_type == B_SLICE) {
                    sh->ref_pic_list_modification_flag_lx[1] = get_bits1(gb);
                    if (sh->ref_pic_list_modification_flag_lx[1] == 1)
                        for (i = 0; i < sh->num_ref_idx_l1_active; i++)
                            sh->list_entry_lx[1][i] = get_bits(gb, av_ceil_log2_c(NumPocTotalCurr));
                }
            }

            if (sh->slice_type == B_SLICE)
                sh->mvd_l1_zero_flag = get_bits1(gb);

            if (sc->pps->cabac_init_present_flag) {
                sh->cabac_init_flag = get_bits1(gb);
            }
            sh->collocated_ref_idx = 0;
            if (sh->slice_temporal_mvp_enabled_flag) {
                sh->collocated_from_l0_flag = 1;
                if (sh->slice_type == B_SLICE) {
                    sh->collocated_from_l0_flag = get_bits1(gb);
                }
                if (( sh->collocated_from_l0_flag && sh->num_ref_idx_l0_active > 1) ||
                    (!sh->collocated_from_l0_flag && sh->num_ref_idx_l1_active > 1)) {
                    sh->collocated_ref_idx = get_ue_golomb(gb);
                }
            }
            if ((sc->pps->weighted_pred_flag && sh->slice_type == P_SLICE) || (sc->pps->weighted_bipred_flag && sh->slice_type == B_SLICE)) {
                pred_weight_table(sc, gb);
            }

            sh->max_num_merge_cand = 5 - get_ue_golomb(gb);
        }
        ff_hevc_set_ref_poc_list(s);
        sh->slice_qp_delta = get_se_golomb(gb);
        if (sc->pps->pic_slice_level_chroma_qp_offsets_present_flag) {
            sh->slice_cb_qp_offset = get_se_golomb(gb);
            sh->slice_cr_qp_offset = get_se_golomb(gb);
        }
        if (sc->pps->deblocking_filter_control_present_flag) {
            int deblocking_filter_override_flag = 0;
            if (sc->pps->deblocking_filter_override_enabled_flag)
                deblocking_filter_override_flag = get_bits1(gb);
            if (deblocking_filter_override_flag) {
                sh->disable_deblocking_filter_flag = get_bits1(gb);
                if (!sh->disable_deblocking_filter_flag) {
                    sh->beta_offset = get_se_golomb(gb) * 2;
                    sh->tc_offset = get_se_golomb(gb) * 2;
                }
            } else {
                sh->disable_deblocking_filter_flag = sc->pps->pps_disable_deblocking_filter_flag;
            }
        }

        if (sc->pps->seq_loop_filter_across_slices_enabled_flag
            && (sh->slice_sample_adaptive_offset_flag[0] ||
                sh->slice_sample_adaptive_offset_flag[1] ||
                !sh->disable_deblocking_filter_flag)) {
            sh->slice_loop_filter_across_slices_enabled_flag = get_bits1(gb);
        } else {
            sh->slice_loop_filter_across_slices_enabled_flag =
            sc->pps->seq_loop_filter_across_slices_enabled_flag;
        }
    }

    ///

    sh->num_entry_point_offsets = 0;
    if( sc->pps->tiles_enabled_flag == 1 || sc->pps->entropy_coding_sync_enabled_flag == 1) {
        sh->num_entry_point_offsets = get_ue_golomb(gb);
        if(sh->num_entry_point_offsets >= MAX_ENTRIES) {
            av_log(s->avctx, AV_LOG_ERROR, "The number of entry points : %d is higher than the maximum number of entry points : %d \n", sh->num_entry_point_offsets, MAX_ENTRIES);
        }
        if( sh->num_entry_point_offsets > 0 ) {
            int offset_len = get_ue_golomb(gb)+1;
            int segments = offset_len >> 4;
            int rest = (offset_len & 15);
            av_freep(&sh->entry_point_offset);
            av_freep(&sh->offset);
            av_freep(&sh->size);
            sh->entry_point_offset = av_malloc(sh->num_entry_point_offsets*sizeof(int));
            sh->offset = av_malloc(sh->num_entry_point_offsets*sizeof(int));
            sh->size = av_malloc(sh->num_entry_point_offsets*sizeof(int));
            for( i = 0; i < sh->num_entry_point_offsets; i++ ) {
                int val = 0;
                for(j = 0;  j < segments; j++){
                    val <<= 16;
                    val += get_bits(gb, 16);
                }
                if(rest) {
                    val <<= rest;
                    val += get_bits(gb, rest);
                }
                sh->entry_point_offset[i] = val + 1; // +1; // +1 to get the size
            }
        }
    }


    if (sc->pps->slice_header_extension_present_flag) {
        int length = get_ue_golomb(gb);
        for (i = 0; i < length; i++)
            skip_bits(gb, 8); // slice_header_extension_data_byte
    }

    // Inferred parameters

    sh->slice_qp = 26 + sc->pps->pic_init_qp_minus26 + sh->slice_qp_delta;
    sh->slice_ctb_addr_rs = sh->slice_address;
    sh->slice_cb_addr_zs = sh->slice_address <<
                           (sc->sps->log2_diff_max_min_coding_block_size << 1);

    return 0;
}



#define CTB(tab, x, y) ((tab)[(y) * sc->sps->pic_width_in_ctbs + (x)])

#define set_sao(elem, value)                            \
    if (!sao_merge_up_flag && !sao_merge_left_flag) {   \
        sao->elem = value;                              \
    } else if (sao_merge_left_flag) {                   \
        sao->elem = CTB(sc->sao, rx-1, ry).elem;         \
    } else if (sao_merge_up_flag) {                     \
        sao->elem = CTB(sc->sao, rx, ry-1).elem;         \
    } else {                                            \
        sao->elem = 0;                                  \
    }

static int hls_sao_param(HEVCContext *s, int rx, int ry)
{
    int c_idx, i;
    int sao_merge_left_flag = 0;
    int sao_merge_up_flag = 0;
    HEVCSharedContext *sc = s->HEVCsc;
    HEVCLocalContext *lc = s->HEVClc;
    int shift = sc->sps->bit_depth - FFMIN(sc->sps->bit_depth, 10);

    SAOParams *sao = &CTB(sc->sao, rx, ry);

    if (rx > 0) {
        if (lc->ctb_left_flag)
            sao_merge_left_flag = ff_hevc_sao_merge_flag_decode(s);
    }
    if (ry > 0 && !sao_merge_left_flag) {
        if (lc->ctb_up_flag)
            sao_merge_up_flag = ff_hevc_sao_merge_flag_decode(s);
    }
    for (c_idx = 0; c_idx < 3; c_idx++) {

        if (!sc->sh.slice_sample_adaptive_offset_flag[c_idx])
            continue;

        if (c_idx == 2) {
            sao->type_idx[2] = sao->type_idx[1];
            sao->eo_class[2] = sao->eo_class[1];
        } else {
            set_sao(type_idx[c_idx], ff_hevc_sao_type_idx_decode(s));
        }

        if (sao->type_idx[c_idx] == SAO_NOT_APPLIED)
            continue;

        for (i = 0; i < 4; i++)
            set_sao(offset_abs[c_idx][i], ff_hevc_sao_offset_abs_decode(s));

        if (sao->type_idx[c_idx] == SAO_BAND) {
            for (i = 0; i < 4; i++) {
                if (sao->offset_abs[c_idx][i]) {
                    set_sao(offset_sign[c_idx][i], ff_hevc_sao_offset_sign_decode(s));
                } else {
                    sao->offset_sign[c_idx][i] = 0;
                }
            }
            set_sao(band_position[c_idx], ff_hevc_sao_band_position_decode(s));
        } else if (c_idx != 2) {
            set_sao(eo_class[c_idx], ff_hevc_sao_eo_class_decode(s));
        }

        // Inferred parameters
        for (i = 0; i < 4; i++) {
            sao->offset_val[c_idx][i+1] = sao->offset_abs[c_idx][i] << shift;
            if (sao->type_idx[c_idx] == SAO_EDGE) {
                if (i > 1)
                    sao->offset_val[c_idx][i+1] = -sao->offset_val[c_idx][i+1];
            } else if (sao->offset_sign[c_idx][i]) {
                sao->offset_val[c_idx][i+1] = -sao->offset_val[c_idx][i+1];
            }
        }
    }
    return 0;
}

#undef set_sao
#undef CTB

static av_always_inline int min_cb_addr_zs(HEVCSharedContext *sc, int x, int y)
{
    return sc->pps->min_cb_addr_zs[y * sc->sps->pic_width_in_min_cbs + x];
}

static void hls_residual_coding(HEVCContext *s, int x0, int y0, int log2_trafo_size, enum ScanType scan_idx, int c_idx)
{
#define GET_COORD(offset, n)                                    \
    do {                                                        \
        x_c = (scan_x_cg[offset >> 4] << 2) + scan_x_off[n];    \
        y_c = (scan_y_cg[offset >> 4] << 2) + scan_y_off[n];    \
    } while (0)
    HEVCSharedContext *sc = s->HEVCsc;
    HEVCLocalContext *lc = s->HEVClc;
    int i;

    int transform_skip_flag = 0;

    int last_significant_coeff_x, last_significant_coeff_y;
    int last_scan_pos;
    int n_end;
    int num_coeff = 0;
    int num_last_subset;
    int x_cg_last_sig, y_cg_last_sig;

    const uint8_t *scan_x_cg, *scan_y_cg, *scan_x_off, *scan_y_off;

    ptrdiff_t stride = sc->frame->linesize[c_idx];
    int hshift = sc->sps->hshift[c_idx];
    int vshift = sc->sps->vshift[c_idx];
    uint8_t *dst = &sc->frame->data[c_idx][(y0 >> vshift) * stride +
                                           ((x0 >> hshift) << sc->sps->pixel_shift)];
    DECLARE_ALIGNED( 16, int16_t, coeffs[MAX_TB_SIZE * MAX_TB_SIZE] )= { 0 };

    int trafo_size = 1 << log2_trafo_size;

    memset(lc->rc.significant_coeff_group_flag, 0, 8*8);


    if (sc->pps->transform_skip_enabled_flag && !lc->cu.cu_transquant_bypass_flag &&
        log2_trafo_size == 2) {
        transform_skip_flag = ff_hevc_transform_skip_flag_decode(s, c_idx);
    }

    last_significant_coeff_x =
    ff_hevc_last_significant_coeff_x_prefix_decode(s, c_idx, log2_trafo_size);
    last_significant_coeff_y =
    ff_hevc_last_significant_coeff_y_prefix_decode(s, c_idx, log2_trafo_size);


    if (last_significant_coeff_x > 3) {
        int suffix = ff_hevc_last_significant_coeff_suffix_decode(s, last_significant_coeff_x);
        last_significant_coeff_x = (1 << ((last_significant_coeff_x >> 1) - 1)) *
                                   (2 + (last_significant_coeff_x & 1)) +
                                   suffix;
    }
    if (last_significant_coeff_y > 3) {
        int suffix = ff_hevc_last_significant_coeff_suffix_decode(s, last_significant_coeff_y);
        last_significant_coeff_y = (1 << ((last_significant_coeff_y >> 1) - 1)) *
                                   (2 + (last_significant_coeff_y & 1)) +
                                   suffix;
    }

    if (scan_idx == SCAN_VERT)
        FFSWAP(int, last_significant_coeff_x, last_significant_coeff_y);

    x_cg_last_sig = last_significant_coeff_x >> 2;
    y_cg_last_sig = last_significant_coeff_y >> 2;

    switch (scan_idx) {
    case SCAN_DIAG: {
        int last_x_c = last_significant_coeff_x & 3;
        int last_y_c = last_significant_coeff_y & 3;

        scan_x_off = diag_scan4x4_x;
        scan_y_off = diag_scan4x4_y;
        num_coeff = diag_scan4x4_inv[last_y_c][last_x_c];
        if (trafo_size == 4) {
            scan_x_cg = scan_1x1;
            scan_y_cg = scan_1x1;
        } else if (trafo_size == 8) {
            num_coeff += diag_scan2x2_inv[y_cg_last_sig][x_cg_last_sig] << 4;
            scan_x_cg = diag_scan2x2_x;
            scan_y_cg = diag_scan2x2_y;
        } else if (trafo_size == 16) {
            num_coeff += diag_scan4x4_inv[y_cg_last_sig][x_cg_last_sig] << 4;
            scan_x_cg = diag_scan4x4_x;
            scan_y_cg = diag_scan4x4_y;
        } else { // trafo_size == 32
            num_coeff += diag_scan8x8_inv[y_cg_last_sig][x_cg_last_sig] << 4;
            scan_x_cg = diag_scan8x8_x;
            scan_y_cg = diag_scan8x8_y;
        }
        break;
    }
    case SCAN_HORIZ:
        scan_x_cg = horiz_scan2x2_x;
        scan_y_cg = horiz_scan2x2_y;
        scan_x_off = horiz_scan4x4_x;
        scan_y_off = horiz_scan4x4_y;
        num_coeff = horiz_scan8x8_inv[last_significant_coeff_y][last_significant_coeff_x];
        break;
    default: //SCAN_VERT
        scan_x_cg = horiz_scan2x2_y;
        scan_y_cg = horiz_scan2x2_x;
        scan_x_off = horiz_scan4x4_y;
        scan_y_off = horiz_scan4x4_x;
        num_coeff = horiz_scan8x8_inv[last_significant_coeff_x][last_significant_coeff_y];
        break;
    }
    num_coeff++;

    num_last_subset = (num_coeff - 1) >> 4;

    for (i = num_last_subset; i >= 0; i--) {
        int n, m;
        int first_nz_pos_in_cg, last_nz_pos_in_cg, num_sig_coeff, first_greater1_coeff_idx;
        int sign_hidden;
        int sum_abs;
        int x_cg, y_cg, x_c, y_c;
        int implicit_non_zero_coeff = 0;
        int trans_coeff_level;

        int offset = i << 4;

        uint8_t significant_coeff_flag_idx[16] = {0};
        uint8_t coeff_abs_level_greater1_flag[16] = {0};
        uint8_t coeff_abs_level_greater2_flag[16] = {0};
        uint16_t coeff_sign_flag;
        uint8_t nb_significant_coeff_flag = 0;

        int first_elem;

        x_cg = scan_x_cg[i];
        y_cg = scan_y_cg[i];

        if ((i < num_last_subset) && (i > 0)) {
            lc->rc.significant_coeff_group_flag[x_cg][y_cg] =
            ff_hevc_significant_coeff_group_flag_decode(s, c_idx, x_cg, y_cg,
                                                        log2_trafo_size);
            implicit_non_zero_coeff = 1;
        } else {
            lc->rc.significant_coeff_group_flag[x_cg][y_cg] =
            ((x_cg == x_cg_last_sig && y_cg == y_cg_last_sig) ||
             (x_cg == 0 && y_cg == 0));
        }

        last_scan_pos = num_coeff - offset - 1;

        if (i == num_last_subset) {
            n_end = last_scan_pos - 1;
            significant_coeff_flag_idx[0] = last_scan_pos;
            nb_significant_coeff_flag = 1;
        } else {
            n_end = 15;
        }

        for (n = n_end; n >= 0; n--) {
            GET_COORD(offset, n);

            if (lc->rc.significant_coeff_group_flag[x_cg][y_cg] &&
                (n > 0 || implicit_non_zero_coeff == 0)) {
                if (ff_hevc_significant_coeff_flag_decode(s, c_idx, x_c, y_c, log2_trafo_size, scan_idx) == 1) {
                    significant_coeff_flag_idx[nb_significant_coeff_flag] = n;
                    nb_significant_coeff_flag = nb_significant_coeff_flag + 1;
                    implicit_non_zero_coeff = 0;
                }
            } else {
                int last_cg = (x_c == (x_cg << 2) && y_c == (y_cg << 2));
                if (last_cg && implicit_non_zero_coeff && lc->rc.significant_coeff_group_flag[x_cg][y_cg]) {
                    significant_coeff_flag_idx[nb_significant_coeff_flag] = n;
                    nb_significant_coeff_flag = nb_significant_coeff_flag + 1;
                }
            }

        }

        n_end = nb_significant_coeff_flag;

        first_nz_pos_in_cg = 16;
        last_nz_pos_in_cg = -1;
        num_sig_coeff = 0;
        first_greater1_coeff_idx = -1;
        for (m = 0; m < n_end; m++) {
            n = significant_coeff_flag_idx[m];
            if (num_sig_coeff < 8) {
                coeff_abs_level_greater1_flag[n] =
                ff_hevc_coeff_abs_level_greater1_flag_decode(s, c_idx, i, n,
                                                             (num_sig_coeff == 0),
                                                             (i == num_last_subset));
                num_sig_coeff++;
                if (coeff_abs_level_greater1_flag[n] &&
                    first_greater1_coeff_idx == -1)
                    first_greater1_coeff_idx = n;
            }
            if (last_nz_pos_in_cg == -1)
                last_nz_pos_in_cg = n;
            first_nz_pos_in_cg = n;
        }

        sign_hidden = (last_nz_pos_in_cg - first_nz_pos_in_cg >= 4 &&
                       !lc->cu.cu_transquant_bypass_flag);
        if (first_greater1_coeff_idx != -1) {
            coeff_abs_level_greater2_flag[first_greater1_coeff_idx] =
            ff_hevc_coeff_abs_level_greater2_flag_decode(s, c_idx, i, first_greater1_coeff_idx);
        }
        if (!sc->pps->sign_data_hiding_flag || !sign_hidden ) {
            coeff_sign_flag = ff_hevc_coeff_sign_flag(s, nb_significant_coeff_flag) << (16 - nb_significant_coeff_flag);
        } else {
            coeff_sign_flag = ff_hevc_coeff_sign_flag(s, nb_significant_coeff_flag-1) << (16 - (nb_significant_coeff_flag - 1));
        }

        num_sig_coeff = 0;
        sum_abs = 0;
        first_elem = 1;
        for (m = 0; m < n_end; m++) {
            n = significant_coeff_flag_idx[m];
            GET_COORD(offset, n);
            trans_coeff_level = 1 + coeff_abs_level_greater1_flag[n] +
                                coeff_abs_level_greater2_flag[n];
            if (trans_coeff_level == ((num_sig_coeff < 8) ?
                                      ((n == first_greater1_coeff_idx) ? 3 : 2) : 1)) {
                trans_coeff_level += ff_hevc_coeff_abs_level_remaining(s, first_elem, trans_coeff_level);
                first_elem = 0;
            }
            if (sc->pps->sign_data_hiding_flag && sign_hidden) {
                sum_abs += trans_coeff_level;
                if (n == first_nz_pos_in_cg && ((sum_abs&1) == 1))
                    trans_coeff_level = -trans_coeff_level;
            }
            if (coeff_sign_flag >> 15)
                trans_coeff_level = -trans_coeff_level;
            coeff_sign_flag <<= 1;
            num_sig_coeff++;
            coeffs[y_c * trafo_size + x_c] = trans_coeff_level;

        }
    }

    if (lc->cu.cu_transquant_bypass_flag) {
        sc->hevcdsp.transquant_bypass[log2_trafo_size-2](dst, coeffs, stride);
    } else {
        int qp;
        int qp_y = lc->qp_y;
        static int qp_c[] = { 29, 30, 31, 32, 33, 33, 34, 34, 35, 35, 36, 36, 37, 37 };
        if (c_idx == 0) {
            qp = qp_y + sc->sps->qp_bd_offset;
        } else {
            int qp_i, offset;

            if (c_idx == 1) {
                offset = sc->pps->cb_qp_offset + sc->sh.slice_cb_qp_offset;
            } else {
                offset = sc->pps->cr_qp_offset + sc->sh.slice_cr_qp_offset;
            }
            qp_i = av_clip_c(qp_y + offset, - sc->sps->qp_bd_offset, 57);
            if (qp_i < 30) {
                qp = qp_i;
            } else if (qp_i > 43) {
                qp = qp_i - 6;
            } else {
                qp = qp_c[qp_i - 30];
            }

            qp += sc->sps->qp_bd_offset;

        }
        sc->hevcdsp.dequant[log2_trafo_size-2](coeffs, qp);
        if (transform_skip_flag) {
            sc->hevcdsp.transform_skip(dst, coeffs, stride);
        } else if (lc->cu.pred_mode == MODE_INTRA && c_idx == 0 && log2_trafo_size == 2) {
            sc->hevcdsp.transform_4x4_luma_add(dst, coeffs, stride);
        } else {
            sc->hevcdsp.transform_add[log2_trafo_size-2](dst, coeffs, stride);
        }
    }
}

static void hls_transform_unit(HEVCContext *s, int x0, int  y0, int xBase, int yBase, int cb_xBase, int cb_yBase,
                               int log2_cb_size, int log2_trafo_size, int trafo_depth, int blk_idx) {
    HEVCSharedContext *sc = s->HEVCsc;
    HEVCLocalContext *lc = s->HEVClc;
    int scan_idx = SCAN_DIAG;
    int scan_idx_c = SCAN_DIAG;
    if (lc->cu.pred_mode == MODE_INTRA) {
        sc->hpc.intra_pred(s, x0, y0, log2_trafo_size, 0);
        if (log2_trafo_size > 2) {
            sc->hpc.intra_pred(s, x0, y0, log2_trafo_size - 1, 1);
            sc->hpc.intra_pred(s, x0, y0, log2_trafo_size - 1, 2);
        } else if (blk_idx == 3) {
            sc->hpc.intra_pred(s, xBase, yBase, log2_trafo_size, 1);
            sc->hpc.intra_pred(s, xBase, yBase, log2_trafo_size, 2);
        }
    }

    if (lc->tt.cbf_luma ||
        SAMPLE_CBF(lc->tt.cbf_cb[trafo_depth], x0, y0) ||
        SAMPLE_CBF(lc->tt.cbf_cr[trafo_depth], x0, y0)) {
        if (sc->pps->cu_qp_delta_enabled_flag && !lc->tu.is_cu_qp_delta_coded) {
            lc->tu.cu_qp_delta = ff_hevc_cu_qp_delta_abs(s);
            if (lc->tu.cu_qp_delta != 0)
                if (ff_hevc_cu_qp_delta_sign_flag(s) == 1)
                    lc->tu.cu_qp_delta = -lc->tu.cu_qp_delta;
            lc->tu.is_cu_qp_delta_coded = 1;
            ff_hevc_set_qPy(s, x0, y0, cb_xBase, cb_yBase, log2_cb_size);
        }

        if (lc->cu.pred_mode == MODE_INTRA && log2_trafo_size < 4) {
            if (lc->tu.cur_intra_pred_mode >= 6 &&
                lc->tu.cur_intra_pred_mode <= 14) {
                scan_idx = SCAN_VERT;
            } else if (lc->tu.cur_intra_pred_mode >= 22 &&
                       lc->tu.cur_intra_pred_mode <= 30) {
                scan_idx = SCAN_HORIZ;
            }

            if (lc->pu.intra_pred_mode_c >= 6 &&
                lc->pu.intra_pred_mode_c <= 14) {
                scan_idx_c = SCAN_VERT;
            } else if (lc->pu.intra_pred_mode_c >= 22 &&
                       lc->pu.intra_pred_mode_c <= 30) {
                scan_idx_c = SCAN_HORIZ;
            }
        }

        if (lc->tt.cbf_luma)
            hls_residual_coding(s, x0, y0, log2_trafo_size, scan_idx, 0);
        if (log2_trafo_size > 2) {
            if (SAMPLE_CBF(lc->tt.cbf_cb[trafo_depth], x0, y0))
                hls_residual_coding(s, x0, y0, log2_trafo_size - 1, scan_idx_c, 1);
            if (SAMPLE_CBF(lc->tt.cbf_cr[trafo_depth], x0, y0))
                hls_residual_coding(s, x0, y0, log2_trafo_size - 1, scan_idx_c, 2);
        } else if (blk_idx == 3) {
            if (SAMPLE_CBF(lc->tt.cbf_cb[trafo_depth], xBase, yBase))
                hls_residual_coding(s, xBase, yBase, log2_trafo_size, scan_idx_c, 1);
            if (SAMPLE_CBF(lc->tt.cbf_cr[trafo_depth], xBase, yBase))
                hls_residual_coding(s, xBase, yBase, log2_trafo_size, scan_idx_c, 2);
        }
    }
}

static void set_deblocking_bypass(HEVCContext *s, int x0, int y0, int log2_cb_size)
{
    int i, j;
    int cb_size = 1 << log2_cb_size;
    int log2_min_pu_size = s->HEVCsc->sps->log2_min_pu_size;
    int pic_width_in_min_pu = s->HEVCsc->sps->pic_width_in_min_cbs * 4;
    for (j = (y0 >> log2_min_pu_size); j < ((y0 + cb_size) >> log2_min_pu_size); j++)
        for (i = (x0 >> log2_min_pu_size); i < ((x0 + cb_size) >> log2_min_pu_size); i++)
            s->HEVCsc->is_pcm[i + j * pic_width_in_min_pu] = 2;
}

static void hls_transform_tree(HEVCContext *s, int x0, int y0, int xBase, int yBase, int cb_xBase, int cb_yBase,
                               int log2_cb_size, int log2_trafo_size, int trafo_depth, int blk_idx)
{
    uint8_t split_transform_flag;
    HEVCSharedContext *sc = s->HEVCsc;
    HEVCLocalContext *lc = s->HEVClc;
    if (trafo_depth > 0 && log2_trafo_size == 2) {
        SAMPLE_CBF(lc->tt.cbf_cb[trafo_depth], x0, y0) =
        SAMPLE_CBF(lc->tt.cbf_cb[trafo_depth - 1], xBase, yBase);
        SAMPLE_CBF(lc->tt.cbf_cr[trafo_depth], x0, y0) =
        SAMPLE_CBF(lc->tt.cbf_cr[trafo_depth - 1], xBase, yBase);
    } else {
        SAMPLE_CBF(lc->tt.cbf_cb[trafo_depth], x0, y0) =
        SAMPLE_CBF(lc->tt.cbf_cr[trafo_depth], x0, y0) = 0;
    }

    if (lc->cu.intra_split_flag) {
        if (trafo_depth == 1)
            lc->tu.cur_intra_pred_mode = lc->pu.intra_pred_mode[blk_idx];
    } else {
        lc->tu.cur_intra_pred_mode = lc->pu.intra_pred_mode[0];
    }

    lc->tt.cbf_luma = 1;

    lc->tt.inter_split_flag = (sc->sps->max_transform_hierarchy_depth_inter == 0 &&
                               lc->cu.pred_mode == MODE_INTER &&
                               lc->cu.part_mode != PART_2Nx2N && trafo_depth == 0);

    if (log2_trafo_size <= sc->sps->log2_max_trafo_size &&
        log2_trafo_size > sc->sps->log2_min_transform_block_size &&
        trafo_depth < lc->cu.max_trafo_depth &&
        !(lc->cu.intra_split_flag && trafo_depth == 0)) {
        split_transform_flag =
        ff_hevc_split_transform_flag_decode(s, log2_trafo_size);
    } else {
        split_transform_flag =
        (log2_trafo_size > sc->sps->log2_max_trafo_size ||
         (lc->cu.intra_split_flag && (trafo_depth == 0)) ||
         lc->tt.inter_split_flag);
    }

    if (log2_trafo_size > 2) {
        if (trafo_depth == 0 || SAMPLE_CBF(lc->tt.cbf_cb[trafo_depth - 1], xBase, yBase)) {
            SAMPLE_CBF(lc->tt.cbf_cb[trafo_depth], x0, y0) =
            ff_hevc_cbf_cb_cr_decode(s, trafo_depth);
        }
        if (trafo_depth == 0 || SAMPLE_CBF(lc->tt.cbf_cr[trafo_depth - 1], xBase, yBase)) {
            SAMPLE_CBF(lc->tt.cbf_cr[trafo_depth], x0, y0) =
            ff_hevc_cbf_cb_cr_decode(s, trafo_depth);
        }
    }

    if (split_transform_flag) {
        int x1 = x0 + (( 1 << log2_trafo_size ) >> 1);
        int y1 = y0 + (( 1 << log2_trafo_size ) >> 1);

        hls_transform_tree(s, x0, y0, x0, y0, cb_xBase, cb_yBase, log2_cb_size,
                           log2_trafo_size - 1, trafo_depth + 1, 0);
        hls_transform_tree(s, x1, y0, x0, y0, cb_xBase, cb_yBase, log2_cb_size,
                           log2_trafo_size - 1, trafo_depth + 1, 1);
        hls_transform_tree(s, x0, y1, x0, y0, cb_xBase, cb_yBase, log2_cb_size,
                           log2_trafo_size - 1, trafo_depth + 1, 2);
        hls_transform_tree(s, x1, y1, x0, y0, cb_xBase, cb_yBase, log2_cb_size,
                           log2_trafo_size - 1, trafo_depth + 1, 3);
    } else {
        int i,j;
        int min_pu_size = 1 << sc->sps->log2_min_pu_size;
        int log2_min_pu_size = sc->sps->log2_min_pu_size;
        int pic_width_in_min_pu = sc->sps->pic_width_in_min_cbs * 4;
        if (lc->cu.pred_mode == MODE_INTRA || trafo_depth != 0 ||
            SAMPLE_CBF(lc->tt.cbf_cb[trafo_depth], x0, y0) ||
            SAMPLE_CBF(lc->tt.cbf_cr[trafo_depth], x0, y0)) {
            lc->tt.cbf_luma = ff_hevc_cbf_luma_decode(s, trafo_depth);
        }

        hls_transform_unit(s, x0, y0, xBase, yBase, cb_xBase, cb_yBase,
                log2_cb_size, log2_trafo_size, trafo_depth, blk_idx);

        // TODO: store cbf_luma somewhere else
        if (lc->tt.cbf_luma)
            for (i = 0; i < (1<<log2_trafo_size); i += min_pu_size)
                for (j = 0; j < (1<<log2_trafo_size); j += min_pu_size) {
                    int x_pu = (x0 + j) >> log2_min_pu_size;
                    int y_pu = (y0 + i) >> log2_min_pu_size;
                    sc->cbf_luma[y_pu * pic_width_in_min_pu + x_pu] = 1;
                }
        if (!sc->sh.disable_deblocking_filter_flag) {
            ff_hevc_deblocking_boundary_strengths(s, x0, y0, log2_trafo_size);
            if (sc->pps->transquant_bypass_enable_flag && lc->cu.cu_transquant_bypass_flag) {
                set_deblocking_bypass(s, x0, y0, log2_cb_size);
            }
        }
    }
}

static void hls_pcm_sample(HEVCContext *s, int x0, int y0, int log2_cb_size)
{
    //TODO: non-4:2:0 support
    int i, j;
    HEVCSharedContext *sc = s->HEVCsc ;
    int log2_min_pu_size = sc->sps->log2_min_pu_size;
    int pic_width_in_min_pu = sc->sps->pic_width_in_min_cbs * 4;
    GetBitContext gb;
    int cb_size = 1 << log2_cb_size;
    int stride0 = sc->frame->linesize[0];
    uint8_t *dst0 = &sc->frame->data[0][y0 * stride0 + x0];
    int stride1 = sc->frame->linesize[1];
    uint8_t *dst1 = &sc->frame->data[1][(y0 >> sc->sps->vshift[1]) * stride1 + (x0 >> sc->sps->hshift[1])];
    int stride2 = sc->frame->linesize[2];
    uint8_t *dst2 = &sc->frame->data[2][(y0 >> sc->sps->vshift[2]) * stride2 + (x0 >> sc->sps->hshift[2])];

    int length = cb_size * cb_size * 3 / 2 * sc->sps->pcm.bit_depth;
    const uint8_t *pcm = skip_bytes(s->HEVClc->cc, length >> 3);

    for (j = y0 >> log2_min_pu_size; j < ((y0 + cb_size) >> log2_min_pu_size); j++)
        for (i = x0 >> log2_min_pu_size; i < ((x0 + cb_size) >> log2_min_pu_size); i++)
            sc->is_pcm[i + j * pic_width_in_min_pu] = 1;
    if (sc->sh.disable_deblocking_filter_flag == 0) {
        if((y0 & 7) == 0)
            for(i = 0; i < cb_size; i+=4)
                sc->horizontal_bs[((x0 + i) + y0 * sc->bs_width) >> 2] = 2;
        if((x0 & 7) == 0)
            for(i = 0; i < cb_size; i+=4)
                sc->vertical_bs[(x0 >> 3) + ((y0 + i) * sc->bs_width) >> 2] = 2;
    }

    init_get_bits(&gb, pcm, length);

    sc->hevcdsp.put_pcm(dst0, stride0, cb_size, &gb, sc->sps->pcm.bit_depth);
    sc->hevcdsp.put_pcm(dst1, stride1, cb_size/2, &gb, sc->sps->pcm.bit_depth);
    sc->hevcdsp.put_pcm(dst2, stride2, cb_size/2, &gb, sc->sps->pcm.bit_depth);
}

static void hls_mvd_coding(HEVCContext *s, int x0, int y0, int log2_cb_size)
{
    int x = ff_hevc_abs_mvd_greater0_flag_decode(s);
    int y = ff_hevc_abs_mvd_greater0_flag_decode(s);
    HEVCLocalContext *lc = s->HEVClc;
    if (x)
        x += ff_hevc_abs_mvd_greater1_flag_decode(s);
    if (y)
        y += ff_hevc_abs_mvd_greater1_flag_decode(s);

    switch (x) {
        case 2:
            lc->pu.mvd.x = ff_hevc_mvd_decode(s);
            break;
        case 1:
            lc->pu.mvd.x = ff_hevc_mvd_sign_flag_decode(s);
            break;
        case 0:
            lc->pu.mvd.x = 0;
    }

    switch (y) {
        case 2:
            lc->pu.mvd.y = ff_hevc_mvd_decode(s);
            break;
        case 1:
            lc->pu.mvd.y = ff_hevc_mvd_sign_flag_decode(s);
            break;
        case 0:
            lc->pu.mvd.y = 0;
    }
    return;
}

/**
 * 8.5.3.2.2.1 Luma sample interpolation process
 *
 * @param s HEVC decoding context
 * @param dst target buffer for block data at block position
 * @param dststride stride of the dst buffer
 * @param ref reference picture buffer at origin (0, 0)
 * @param mv motion vector (relative to block position) to get pixel data from
 * @param x_off horizontal position of block from origin (0, 0)
 * @param y_off vertical position of block from origin (0, 0)
 * @param block_w width of block
 * @param block_h height of block
 */
static void luma_mc(HEVCContext *s, int16_t *dst, ptrdiff_t dststride, AVFrame *ref,
                    const Mv *mv, int x_off, int y_off, int block_w, int block_h)
{
    uint8_t *src = ref->data[0];
    ptrdiff_t srcstride = ref->linesize[0];
    HEVCSharedContext *sc = s->HEVCsc;
    int pic_width = sc->sps->pic_width_in_luma_samples;
    int pic_height = sc->sps->pic_height_in_luma_samples;

    int mx = mv->x & 3;
    int my = mv->y & 3;
    int extra_left = qpel_extra_before[mx];
    int extra_top = qpel_extra_before[my];

    x_off += mv->x >> 2;
    y_off += mv->y >> 2;
    src += y_off * srcstride + (x_off << sc->sps->pixel_shift);

    if (x_off < extra_left || x_off >= pic_width - block_w - qpel_extra_after[mx] ||
        y_off < extra_top || y_off >= pic_height - block_h - qpel_extra_after[my]) {
        int offset = extra_top * srcstride + (extra_left << sc->sps->pixel_shift);
        sc->vdsp.emulated_edge_mc(s->HEVClc->edge_emu_buffer, src - offset, srcstride,
                                  block_w + qpel_extra[mx], block_h + qpel_extra[my],
                                  x_off - extra_left, y_off - extra_top,
                                  pic_width, pic_height);
        src = s->HEVClc->edge_emu_buffer + offset;
    }
    sc->hevcdsp.put_hevc_qpel[my][mx](dst, dststride, src, srcstride, block_w, block_h);
}

/**
 * 8.5.3.2.2.2 Chroma sample interpolation process
 *
 * @param s HEVC decoding context
 * @param dst1 target buffer for block data at block position (U plane)
 * @param dst2 target buffer for block data at block position (V plane)
 * @param dststride stride of the dst1 and dst2 buffers
 * @param ref reference picture buffer at origin (0, 0)
 * @param mv motion vector (relative to block position) to get pixel data from
 * @param x_off horizontal position of block from origin (0, 0)
 * @param y_off vertical position of block from origin (0, 0)
 * @param block_w width of block
 * @param block_h height of block
 */
static void chroma_mc(HEVCContext *s, int16_t *dst1, int16_t *dst2, ptrdiff_t dststride, AVFrame *ref,
                      const Mv *mv, int x_off, int y_off, int block_w, int block_h)
{
    uint8_t *src1 = ref->data[1];
    uint8_t *src2 = ref->data[2];
    ptrdiff_t src1stride = ref->linesize[1];
    ptrdiff_t src2stride = ref->linesize[2];
    HEVCSharedContext *sc = s->HEVCsc;
    int pic_width = sc->sps->pic_width_in_luma_samples >> 1;
    int pic_height = sc->sps->pic_height_in_luma_samples >> 1;

    int mx = mv->x & 7;
    int my = mv->y & 7;

    x_off += mv->x >> 3;
    y_off += mv->y >> 3;
    src1 += y_off * src1stride + (x_off << sc->sps->pixel_shift);
    src2 += y_off * src2stride + (x_off << sc->sps->pixel_shift);

    if (x_off < epel_extra_before || x_off >= pic_width - block_w - epel_extra_after ||
        y_off < epel_extra_after || y_off >= pic_height - block_h - epel_extra_after) {
        int offset1 = epel_extra_before * (src1stride + (1 << sc->sps->pixel_shift));
        int offset2 = epel_extra_before * (src2stride + (1 << sc->sps->pixel_shift));
        sc->vdsp.emulated_edge_mc(s->HEVClc->edge_emu_buffer, src1 - offset1, src1stride,
                                  block_w + epel_extra, block_h + epel_extra,
                                  x_off - epel_extra_before, y_off - epel_extra_before,
                                  pic_width, pic_height);
        src1 = s->HEVClc->edge_emu_buffer + offset1;
        sc->hevcdsp.put_hevc_epel[!!my][!!mx](dst1, dststride, src1, src1stride, block_w, block_h, mx, my);

        sc->vdsp.emulated_edge_mc(s->HEVClc->edge_emu_buffer, src2 - offset2, src2stride,
                                  block_w + epel_extra, block_h + epel_extra,
                                  x_off - epel_extra_before, y_off - epel_extra_before,
                                  pic_width, pic_height);
        src2 = s->HEVClc->edge_emu_buffer + offset2;
        sc->hevcdsp.put_hevc_epel[!!my][!!mx](dst2, dststride, src2, src2stride, block_w, block_h, mx, my);
    } else {
        sc->hevcdsp.put_hevc_epel[!!my][!!mx](dst1, dststride, src1, src1stride, block_w, block_h, mx, my);
        sc->hevcdsp.put_hevc_epel[!!my][!!mx](dst2, dststride, src2, src2stride, block_w, block_h, mx, my);
    }
}
/*
static int identical_mvs(MvField *mv, RefPicList *refPicList) {
    if (mv->pred_flag[0] + mv->pred_flag[1] == 2)
        return (refPicList[0].list[mv->ref_idx[0]] == refPicList[1].list[mv->ref_idx[1]] && mv->mv[0].x == mv->mv[1].x && mv->mv[0].y == mv->mv[1].y);
    else
        return 0;
}
*/
static void hls_prediction_unit(HEVCContext *s, int x0, int y0, int nPbW, int nPbH, int log2_cb_size, int partIdx)
{
#define POS(c_idx, x, y)                                                              \
    &sc->frame->data[c_idx][((y) >> sc->sps->vshift[c_idx]) * sc->frame->linesize[c_idx] + \
                           (((x) >> sc->sps->hshift[c_idx]) << sc->sps->pixel_shift)]
    int merge_idx = 0;
    HEVCSharedContext *sc = s->HEVCsc;
    HEVCLocalContext *lc = s->HEVClc;
    enum InterPredIdc inter_pred_idc = PRED_L0;
    int ref_idx[2];
    int mvp_flag[2];
    struct MvField current_mv = {{{ 0 }}};
    int i, j;
    int x_pu, y_pu;
    int pic_width_in_min_pu = sc->sps->pic_width_in_min_cbs * 4;
    MvField *tab_mvf = sc->ref->tab_mvf;
    RefPicList  *refPicList =  sc->ref->refPicList;

    int tmpstride = MAX_PB_SIZE;

    uint8_t *dst0 = POS(0, x0, y0);
    uint8_t *dst1 = POS(1, x0, y0);
    uint8_t *dst2 = POS(2, x0, y0);
    int log2_min_cb_size = sc->sps->log2_min_coding_block_size;
    int pic_width_in_ctb = sc->sps->pic_width_in_luma_samples>>log2_min_cb_size;
    int x_cb             = x0 >> log2_min_cb_size;
    int y_cb             = y0 >> log2_min_cb_size;

    if (SAMPLE_CTB(sc->skip_flag, x_cb, y_cb)) {
        if (sc->sh.max_num_merge_cand > 1)
            merge_idx = ff_hevc_merge_idx_decode(s);
        else
            merge_idx = 0;

        ff_hevc_luma_mv_merge_mode(s, x0, y0, 1 << log2_cb_size, 1 << log2_cb_size, log2_cb_size, partIdx, merge_idx, &current_mv);
        x_pu = x0 >> sc->sps->log2_min_pu_size;
        y_pu = y0 >> sc->sps->log2_min_pu_size;
        for(i = 0; i < nPbW >> sc->sps->log2_min_pu_size; i++) {
            for(j = 0; j < nPbH >> sc->sps->log2_min_pu_size; j++) {
                tab_mvf[(y_pu + j) * pic_width_in_min_pu + x_pu + i] = current_mv;
            }
        }

    } else {/* MODE_INTER */
        lc->pu.merge_flag = ff_hevc_merge_flag_decode(s);
        if (lc->pu.merge_flag) {
            if (sc->sh.max_num_merge_cand > 1)
                merge_idx = ff_hevc_merge_idx_decode(s);
            else
                merge_idx = 0;

            ff_hevc_luma_mv_merge_mode(s, x0, y0, nPbW, nPbH, log2_cb_size, partIdx, merge_idx, &current_mv);
            x_pu = x0 >> sc->sps->log2_min_pu_size;
            y_pu = y0 >> sc->sps->log2_min_pu_size;
            for(i = 0; i < nPbW >> sc->sps->log2_min_pu_size; i++) {
                for(j = 0; j < nPbH >> sc->sps->log2_min_pu_size; j++) {
                    tab_mvf[(y_pu + j) * pic_width_in_min_pu + x_pu + i] = current_mv;
                }
            }
        } else {
            if (sc->sh.slice_type == B_SLICE) {
                inter_pred_idc = ff_hevc_inter_pred_idc_decode(s, nPbW, nPbH);
            }
            if (inter_pred_idc != PRED_L1) {
                if (sc->sh.num_ref_idx_l0_active > 1) {
                    ref_idx[0] = ff_hevc_ref_idx_lx_decode(s, sc->sh.num_ref_idx_l0_active);
                    current_mv.ref_idx[0] = ref_idx[0];
                }
                current_mv.pred_flag[0] = 1;
                hls_mvd_coding(s, x0, y0, 0 );
                mvp_flag[0] = ff_hevc_mvp_lx_flag_decode(s);
                ff_hevc_luma_mv_mvp_mode(s, x0, y0, nPbW, nPbH, log2_cb_size, partIdx, merge_idx, &current_mv, mvp_flag[0], 0);
                current_mv.mv[0].x += lc->pu.mvd.x;
                current_mv.mv[0].y += lc->pu.mvd.y;
            }
            if (inter_pred_idc != PRED_L0) {
                if (sc->sh.num_ref_idx_l1_active > 1) {
                    ref_idx[1] = ff_hevc_ref_idx_lx_decode(s, sc->sh.num_ref_idx_l1_active);
                    current_mv.ref_idx[1] = ref_idx[1];
                }
                if (sc->sh.mvd_l1_zero_flag == 1 && inter_pred_idc == PRED_BI) {
                    lc->pu.mvd.x = 0;
                    lc->pu.mvd.y = 0;
                    //mvd_l1[ x0 ][ y0 ][ 0 ] = 0
                    //mvd_l1[ x0 ][ y0 ][ 1 ] = 0
                } else {
                    hls_mvd_coding(s, x0, y0, 1);
                }
                current_mv.pred_flag[1] = 1;
                mvp_flag[1] = ff_hevc_mvp_lx_flag_decode(s);
                ff_hevc_luma_mv_mvp_mode(s, x0, y0, nPbW, nPbH, log2_cb_size, partIdx, merge_idx, &current_mv, mvp_flag[1], 1);
                current_mv.mv[1].x += lc->pu.mvd.x;
                current_mv.mv[1].y += lc->pu.mvd.y;
            }
            x_pu = x0 >> sc->sps->log2_min_pu_size;
            y_pu = y0 >> sc->sps->log2_min_pu_size;
            for(i = 0; i < nPbW >> sc->sps->log2_min_pu_size; i++) {
                for(j = 0; j < nPbH >> sc->sps->log2_min_pu_size; j++) {
                    tab_mvf[(y_pu + j) * pic_width_in_min_pu + x_pu + i] = current_mv;
                }
            }
        }
    }
    if (current_mv.pred_flag[0] && !current_mv.pred_flag[1]) {
        DECLARE_ALIGNED( 16, int16_t, tmp[MAX_PB_SIZE*MAX_PB_SIZE] );
        DECLARE_ALIGNED( 16, int16_t, tmp2[MAX_PB_SIZE * MAX_PB_SIZE] );

        if (! sc->pps->weighted_pred_flag){
            luma_mc(s, tmp, tmpstride,
                    sc->DPB[refPicList[0].idx[current_mv.ref_idx[0]]].frame,
                    &current_mv.mv[0], x0, y0, nPbW, nPbH);
            sc->hevcdsp.put_unweighted_pred(dst0, sc->frame->linesize[0], tmp, tmpstride, nPbW, nPbH);
            chroma_mc(s, tmp, tmp2, tmpstride,
                      sc->DPB[refPicList[0].idx[current_mv.ref_idx[0]]].frame,
                      &current_mv.mv[0], x0/2, y0/2, nPbW/2, nPbH/2);
            sc->hevcdsp.put_unweighted_pred(dst1, sc->frame->linesize[1], tmp, tmpstride, nPbW/2, nPbH/2);
            sc->hevcdsp.put_unweighted_pred(dst2, sc->frame->linesize[2], tmp2, tmpstride, nPbW/2, nPbH/2);
        } else {
            luma_mc(s, tmp, tmpstride,
                    sc->DPB[refPicList[0].idx[current_mv.ref_idx[0]]].frame,
                    &current_mv.mv[0], x0, y0, nPbW, nPbH);
            sc->hevcdsp.weighted_pred(sc->sh.luma_log2_weight_denom,
                                      sc->sh.luma_weight_l0[current_mv.ref_idx[0]],
                                      sc->sh.luma_offset_l0[current_mv.ref_idx[0]],
                                      dst0, sc->frame->linesize[0], tmp, tmpstride, nPbW, nPbH);
            chroma_mc(s, tmp, tmp2, tmpstride,
                      sc->DPB[refPicList[0].idx[current_mv.ref_idx[0]]].frame,
                      &current_mv.mv[0], x0/2, y0/2, nPbW/2, nPbH/2);
            sc->hevcdsp.weighted_pred(sc->sh.chroma_log2_weight_denom,
                                      sc->sh.chroma_weight_l0[current_mv.ref_idx[0]][0],
                                      sc->sh.chroma_offset_l0[current_mv.ref_idx[0]][0],
                                      dst1, sc->frame->linesize[1], tmp, tmpstride, nPbW/2, nPbH/2);
            sc->hevcdsp.weighted_pred(sc->sh.chroma_log2_weight_denom,
                                      sc->sh.chroma_weight_l0[current_mv.ref_idx[0]][1],
                                      sc->sh.chroma_offset_l0[current_mv.ref_idx[0]][1],
                                      dst2, sc->frame->linesize[2], tmp2, tmpstride, nPbW/2, nPbH/2);
        }

    } else if (!current_mv.pred_flag[0] && current_mv.pred_flag[1]) {
        DECLARE_ALIGNED( 16, int16_t, tmp[MAX_PB_SIZE*MAX_PB_SIZE] );
        DECLARE_ALIGNED( 16, int16_t, tmp2[MAX_PB_SIZE * MAX_PB_SIZE] );
        if (! sc->pps->weighted_pred_flag){
            luma_mc(s, tmp, tmpstride,
                    sc->DPB[refPicList[1].idx[current_mv.ref_idx[1]]].frame,
                    &current_mv.mv[1], x0, y0, nPbW, nPbH);
            sc->hevcdsp.put_unweighted_pred(dst0, sc->frame->linesize[0], tmp, tmpstride, nPbW, nPbH);
            chroma_mc(s, tmp, tmp2, tmpstride,
                      sc->DPB[refPicList[1].idx[current_mv.ref_idx[1]]].frame,
                      &current_mv.mv[1], x0/2, y0/2, nPbW/2, nPbH/2);
            sc->hevcdsp.put_unweighted_pred(dst1, sc->frame->linesize[1], tmp, tmpstride, nPbW/2, nPbH/2);
            sc->hevcdsp.put_unweighted_pred(dst2, sc->frame->linesize[2], tmp2, tmpstride, nPbW/2, nPbH/2);
        } else {
            luma_mc(s, tmp, tmpstride,
                    sc->DPB[refPicList[1].idx[current_mv.ref_idx[1]]].frame,
                    &current_mv.mv[1], x0, y0, nPbW, nPbH);
            sc->hevcdsp.weighted_pred(sc->sh.luma_log2_weight_denom,
                                      sc->sh.luma_weight_l1[current_mv.ref_idx[1]],
                                      sc->sh.luma_offset_l1[current_mv.ref_idx[1]],
                                      dst0, sc->frame->linesize[0], tmp, tmpstride, nPbW, nPbH);
            chroma_mc(s, tmp, tmp2, tmpstride,
                      sc->DPB[refPicList[1].idx[current_mv.ref_idx[1]]].frame,
                      &current_mv.mv[1], x0/2, y0/2, nPbW/2, nPbH/2);

            sc->hevcdsp.weighted_pred(sc->sh.chroma_log2_weight_denom,
                                      sc->sh.chroma_weight_l1[current_mv.ref_idx[1]][0],
                                      sc->sh.chroma_offset_l1[current_mv.ref_idx[1]][0],
                                      dst1, sc->frame->linesize[1], tmp, tmpstride, nPbW/2, nPbH/2);
            sc->hevcdsp.weighted_pred(sc->sh.chroma_log2_weight_denom,
                                      sc->sh.chroma_weight_l1[current_mv.ref_idx[1]][1],
                                      sc->sh.chroma_offset_l1[current_mv.ref_idx[1]][1],
                                      dst2, sc->frame->linesize[2], tmp2, tmpstride, nPbW/2, nPbH/2);
        }

    } else if (current_mv.pred_flag[0] && current_mv.pred_flag[1]) {
        DECLARE_ALIGNED( 16, int16_t, tmp[MAX_PB_SIZE*MAX_PB_SIZE] );
        DECLARE_ALIGNED( 16, int16_t, tmp2[MAX_PB_SIZE*MAX_PB_SIZE] );
        DECLARE_ALIGNED( 16, int16_t, tmp3[MAX_PB_SIZE*MAX_PB_SIZE] );
        DECLARE_ALIGNED( 16, int16_t, tmp4[MAX_PB_SIZE*MAX_PB_SIZE] );
        if (! sc->pps->weighted_bipred_flag){
            luma_mc(s, tmp, tmpstride,
                    sc->DPB[refPicList[0].idx[current_mv.ref_idx[0]]].frame,
                    &current_mv.mv[0], x0, y0, nPbW, nPbH);
            luma_mc(s, tmp2, tmpstride,
                    sc->DPB[refPicList[1].idx[current_mv.ref_idx[1]]].frame,
                    &current_mv.mv[1], x0, y0, nPbW, nPbH);
            sc->hevcdsp.put_weighted_pred_avg(dst0, sc->frame->linesize[0], tmp, tmp2, tmpstride, nPbW, nPbH);
            chroma_mc(s, tmp, tmp2, tmpstride,
                      sc->DPB[refPicList[0].idx[current_mv.ref_idx[0]]].frame,
                      &current_mv.mv[0], x0/2, y0/2, nPbW/2, nPbH/2);
            chroma_mc(s, tmp3, tmp4, tmpstride,
                      sc->DPB[refPicList[1].idx[current_mv.ref_idx[1]]].frame,
                      &current_mv.mv[1], x0/2, y0/2, nPbW/2, nPbH/2);
            sc->hevcdsp.put_weighted_pred_avg(dst1, sc->frame->linesize[1], tmp, tmp3, tmpstride, nPbW/2, nPbH/2);
            sc->hevcdsp.put_weighted_pred_avg(dst2, sc->frame->linesize[2], tmp2, tmp4, tmpstride, nPbW/2, nPbH/2);
        } else {
            luma_mc(s, tmp, tmpstride,
                    sc->DPB[refPicList[0].idx[current_mv.ref_idx[0]]].frame,
                    &current_mv.mv[0], x0, y0, nPbW, nPbH);
            luma_mc(s, tmp2, tmpstride,
                    sc->DPB[refPicList[1].idx[current_mv.ref_idx[1]]].frame,
                    &current_mv.mv[1], x0, y0, nPbW, nPbH);
            sc->hevcdsp.weighted_pred_avg(sc->sh.luma_log2_weight_denom,
                                          sc->sh.luma_weight_l0[current_mv.ref_idx[0]],
                                          sc->sh.luma_weight_l1[current_mv.ref_idx[1]],
                                          sc->sh.luma_offset_l0[current_mv.ref_idx[0]],
                                          sc->sh.luma_offset_l1[current_mv.ref_idx[1]],
                                          dst0, sc->frame->linesize[0], tmp, tmp2, tmpstride, nPbW, nPbH);
            chroma_mc(s, tmp, tmp2, tmpstride,
                      sc->DPB[refPicList[0].idx[current_mv.ref_idx[0]]].frame,
                      &current_mv.mv[0], x0/2, y0/2, nPbW/2, nPbH/2);
            chroma_mc(s, tmp3, tmp4, tmpstride,
                      sc->DPB[refPicList[1].idx[current_mv.ref_idx[1]]].frame,
                      &current_mv.mv[1], x0/2, y0/2, nPbW/2, nPbH/2);

            sc->hevcdsp.weighted_pred_avg(sc->sh.chroma_log2_weight_denom ,
                                          sc->sh.chroma_weight_l0[current_mv.ref_idx[0]][0],
                                          sc->sh.chroma_weight_l1[current_mv.ref_idx[1]][0],
                                          sc->sh.chroma_offset_l0[current_mv.ref_idx[0]][0],
                                          sc->sh.chroma_offset_l1[current_mv.ref_idx[1]][0],
                                          dst1, sc->frame->linesize[1], tmp, tmp3, tmpstride, nPbW/2, nPbH/2);
            sc->hevcdsp.weighted_pred_avg(sc->sh.chroma_log2_weight_denom ,
                                          sc->sh.chroma_weight_l0[current_mv.ref_idx[0]][1],
                                          sc->sh.chroma_weight_l1[current_mv.ref_idx[1]][1],
                                          sc->sh.chroma_offset_l0[current_mv.ref_idx[0]][1],
                                          sc->sh.chroma_offset_l1[current_mv.ref_idx[1]][1],
                                          dst2, sc->frame->linesize[2], tmp2, tmp4, tmpstride, nPbW/2, nPbH/2);
        }
    }
    return;
}


/**
 * 8.4.1
 */
static int luma_intra_pred_mode(HEVCContext *s, int x0, int y0, int pu_size,
                                int prev_intra_luma_pred_flag)
{
    int i, j;
    int candidate[3];
    int intra_pred_mode;
    HEVCSharedContext *sc = s->HEVCsc;
    HEVCLocalContext *lc = s->HEVClc;
    int x_pu = x0 >> sc->sps->log2_min_pu_size;
    int y_pu = y0 >> sc->sps->log2_min_pu_size;
    int pic_width_in_min_pu = sc->sps->pic_width_in_min_cbs << 2;
    int size_in_pus = pu_size >> sc->sps->log2_min_pu_size;
    int x0b = x0 & ((1 << sc->sps->log2_ctb_size) - 1);
    int y0b = y0 & ((1 << sc->sps->log2_ctb_size) - 1);

    int cand_up   = (lc->ctb_up_flag || y0b) ? sc->top_ipm[x_pu] : INTRA_DC ;
    int cand_left = (lc->ctb_left_flag || x0b) ? sc->left_ipm[y_pu] : INTRA_DC ;

    int y_ctb = (y0 >> (sc->sps->log2_ctb_size)) << (sc->sps->log2_ctb_size);
    MvField *tab_mvf = sc->ref->tab_mvf;

    // intra_pred_mode prediction does not cross vertical CTB boundaries
    if ((y0 - 1) < y_ctb)
        cand_up = INTRA_DC;


    if (cand_left == cand_up) {
        if (cand_left < 2) {
            candidate[0] = INTRA_PLANAR;
            candidate[1] = INTRA_DC;
            candidate[2] = INTRA_ANGULAR_26;
        } else {
            candidate[0] = cand_left;
            candidate[1] = 2 + ((cand_left - 2 - 1 + 32) & 31);
            candidate[2] = 2 + ((cand_left - 2 + 1) & 31);
        }
    } else {
        candidate[0] = cand_left;
        candidate[1] = cand_up;
        if (candidate[0] != INTRA_PLANAR && candidate[1] != INTRA_PLANAR) {
            candidate[2] = INTRA_PLANAR;
        } else if (candidate[0] != INTRA_DC && candidate[1] != INTRA_DC) {
            candidate[2] = INTRA_DC;
        } else {
            candidate[2] = INTRA_ANGULAR_26;
        }
    }

    if (prev_intra_luma_pred_flag) {
        intra_pred_mode = candidate[lc->pu.mpm_idx];
    } else {
        if (candidate[0] > candidate[1])
            FFSWAP(uint8_t, candidate[0], candidate[1]);
        if (candidate[0] > candidate[2])
            FFSWAP(uint8_t, candidate[0], candidate[2]);
        if (candidate[1] > candidate[2])
            FFSWAP(uint8_t, candidate[1], candidate[2]);

        intra_pred_mode = lc->pu.rem_intra_luma_pred_mode;
        for (i = 0; i < 3; i++) {
            if (intra_pred_mode >= candidate[i])
                intra_pred_mode++;
        }
    }
    memset(&sc->top_ipm[x_pu], intra_pred_mode, size_in_pus);
    memset(&sc->left_ipm[y_pu], intra_pred_mode, size_in_pus);

    /* write the intra prediction units into the mv array */
    for(i = 0; i <size_in_pus; i++) {
        for(j = 0; j <size_in_pus; j++) {
            tab_mvf[(y_pu+j)*pic_width_in_min_pu + x_pu+i].is_intra = 1;
            tab_mvf[(y_pu+j)*pic_width_in_min_pu + x_pu+i].pred_flag[0] = 0;
            tab_mvf[(y_pu+j)*pic_width_in_min_pu + x_pu+i].pred_flag[1] = 0;
            tab_mvf[(y_pu+j)*pic_width_in_min_pu + x_pu+i].ref_idx[0] = 0;
            tab_mvf[(y_pu+j)*pic_width_in_min_pu + x_pu+i].ref_idx[1] = 0;
            tab_mvf[(y_pu+j)*pic_width_in_min_pu + x_pu+i].mv[0].x = 0;
            tab_mvf[(y_pu+j)*pic_width_in_min_pu + x_pu+i].mv[0].y = 0;
            tab_mvf[(y_pu+j)*pic_width_in_min_pu + x_pu+i].mv[1].x = 0;
            tab_mvf[(y_pu+j)*pic_width_in_min_pu + x_pu+i].mv[1].y = 0;
        }
    }

    return intra_pred_mode;
}

static av_always_inline void set_ct_depth(HEVCContext *s, int x0, int y0,
                                          int log2_cb_size, int ct_depth)
{
    HEVCSharedContext *sc = s->HEVCsc;
    int length = (1 << log2_cb_size) >> sc->sps->log2_min_coding_block_size;
    int x_cb = x0 >> sc->sps->log2_min_coding_block_size;
    int y_cb = y0 >> sc->sps->log2_min_coding_block_size;

    memset(&sc->top_ct_depth[x_cb], ct_depth, length);
    memset(&sc->left_ct_depth[y_cb], ct_depth, length);
}

static void intra_prediction_unit(HEVCContext *s, int x0, int y0, int log2_cb_size)
{
    int i, j;
    uint8_t prev_intra_luma_pred_flag[4];
    int chroma_mode;
    static const uint8_t intra_chroma_table[4] = {0, 26, 10, 1};
    HEVCLocalContext *lc = s->HEVClc;
    int split = lc->cu.part_mode == PART_NxN;
    int pb_size = (1 << log2_cb_size) >> split;
    int side = split + 1;

    for (i = 0; i < side; i++)
        for (j = 0; j < side; j++) {
            prev_intra_luma_pred_flag[2*i+j] = ff_hevc_prev_intra_luma_pred_flag_decode(s);
        }

    for (i = 0; i < side; i++) {
        for (j = 0; j < side; j++) {
            if (prev_intra_luma_pred_flag[2*i+j]) {
                lc->pu.mpm_idx = ff_hevc_mpm_idx_decode(s);
            } else {
                lc->pu.rem_intra_luma_pred_mode = ff_hevc_rem_intra_luma_pred_mode_decode(s);
            }
            lc->pu.intra_pred_mode[2*i+j] =
            luma_intra_pred_mode(s, x0 + pb_size * j, y0 + pb_size * i, pb_size,
                                 prev_intra_luma_pred_flag[2*i+j]);
        }
    }

    chroma_mode = ff_hevc_intra_chroma_pred_mode_decode(s);
    if (chroma_mode != 4) {
        if (lc->pu.intra_pred_mode[0] == intra_chroma_table[chroma_mode]) {
            lc->pu.intra_pred_mode_c = 34;
        } else {
            lc->pu.intra_pred_mode_c = intra_chroma_table[chroma_mode];
        }
    } else {
        lc->pu.intra_pred_mode_c = lc->pu.intra_pred_mode[0];
    }

}
static void intra_prediction_unit_default_value(HEVCContext *s, int x0, int y0, int log2_cb_size)
{
    int i, j, k;
    HEVCLocalContext *lc = s->HEVClc;
    HEVCSharedContext *sc = s->HEVCsc;
    int split = lc->cu.part_mode == PART_NxN;
    int pb_size = (1 << log2_cb_size) >> split;
    int side = split + 1;
    int size_in_pus = pb_size >> sc->sps->log2_min_pu_size;
    int pic_width_in_min_pu  = sc->sps->pic_width_in_min_cbs * 4;
    MvField *tab_mvf = sc->ref->tab_mvf;
    for (i = 0; i < side; i++) {
        int x_pu = (x0 + pb_size * i) >> sc->sps->log2_min_pu_size;
        int y_pu = (y0 + pb_size * i) >> sc->sps->log2_min_pu_size;
        memset(&sc->top_ipm[x_pu], INTRA_DC, size_in_pus);
        memset(&sc->left_ipm[y_pu], INTRA_DC, size_in_pus);
        for(j = 0; j <size_in_pus; j++) {
            for(k = 0; k <size_in_pus; k++) {
                tab_mvf[(y_pu+j)*pic_width_in_min_pu + x_pu+k].is_intra = lc->cu.pred_mode == MODE_INTRA;
            }
        }
    }

}

static void hls_coding_unit(HEVCContext *s, int x0, int y0, int log2_cb_size)
{
    int cb_size          = 1 << log2_cb_size;
    HEVCSharedContext *sc = s->HEVCsc;
    HEVCLocalContext *lc = s->HEVClc;
    int log2_min_cb_size = sc->sps->log2_min_coding_block_size;
    int length           = cb_size >> log2_min_cb_size;
    int pic_width_in_ctb = sc->sps->pic_width_in_luma_samples >> log2_min_cb_size;
    int x_cb             = x0 >> log2_min_cb_size;
    int y_cb             = y0 >> log2_min_cb_size;
    int x, y;
    lc->cu.x = x0;
    lc->cu.y = y0;
    lc->cu.rqt_root_cbf = 1;

    lc->cu.pred_mode = MODE_INTRA;
    lc->cu.part_mode = PART_2Nx2N;
    lc->cu.intra_split_flag = 0;
    lc->cu.pcm_flag = 0;
    SAMPLE_CTB(sc->skip_flag, x_cb, y_cb) = 0;
    for (x = 0; x < 4; x++) {
        lc->pu.intra_pred_mode[x] = 1;
    }
    if (sc->pps->transquant_bypass_enable_flag)
        lc->cu.cu_transquant_bypass_flag = ff_hevc_cu_transquant_bypass_flag_decode(s);

    if (sc->sh.slice_type != I_SLICE) {
        uint8_t skip_flag = ff_hevc_skip_flag_decode(s, x0, y0, x_cb, y_cb);
        lc->cu.pred_mode = MODE_SKIP;
        x = y_cb * pic_width_in_ctb + x_cb;
        for (y = 0; y < length; y++) {
            memset(&sc->skip_flag[x], skip_flag, length);
            x += pic_width_in_ctb;
        }
        lc->cu.pred_mode = skip_flag ? MODE_SKIP : MODE_INTER;
    }

    if (SAMPLE_CTB(sc->skip_flag, x_cb, y_cb)) {
        hls_prediction_unit(s, x0, y0, cb_size, cb_size, log2_cb_size, 0);
        intra_prediction_unit_default_value(s, x0, y0, log2_cb_size);
        if (!sc->sh.disable_deblocking_filter_flag) {
            ff_hevc_deblocking_boundary_strengths(s, x0, y0, log2_cb_size);
            if (sc->pps->transquant_bypass_enable_flag && lc->cu.cu_transquant_bypass_flag) {
                set_deblocking_bypass(s, x, y, log2_cb_size);

            }
        }
    } else {
        if (sc->sh.slice_type != I_SLICE) {
            lc->cu.pred_mode = ff_hevc_pred_mode_decode(s);
        }
        if (lc->cu.pred_mode != MODE_INTRA ||
            log2_cb_size == sc->sps->log2_min_coding_block_size) {
            lc->cu.part_mode = ff_hevc_part_mode_decode(s, log2_cb_size);
            lc->cu.intra_split_flag = lc->cu.part_mode == PART_NxN &&
                                     lc->cu.pred_mode == MODE_INTRA;
        }

        if (lc->cu.pred_mode == MODE_INTRA) {
            if (lc->cu.part_mode == PART_2Nx2N && sc->sps->pcm_enabled_flag &&
                log2_cb_size >= sc->sps->pcm.log2_min_pcm_cb_size &&
                log2_cb_size <= sc->sps->pcm.log2_max_pcm_cb_size) {
                lc->cu.pcm_flag = ff_hevc_pcm_flag_decode(s);
            }
            if (lc->cu.pcm_flag) {
                hls_pcm_sample(s, x0, y0, log2_cb_size);
                intra_prediction_unit_default_value(s, x0, y0, log2_cb_size);
            } else {
                intra_prediction_unit(s, x0, y0, log2_cb_size);
            }
        } else {
            intra_prediction_unit_default_value(s, x0, y0, log2_cb_size);
            switch (lc->cu.part_mode) {
            case PART_2Nx2N:
                hls_prediction_unit(s, x0, y0, cb_size, cb_size, log2_cb_size, 0);
                break;
            case PART_2NxN:
                hls_prediction_unit(s, x0, y0, cb_size, cb_size / 2, log2_cb_size, 0);
                hls_prediction_unit(s, x0, y0 + cb_size / 2, cb_size, cb_size/2, log2_cb_size, 1);
                break;
            case PART_Nx2N:
                hls_prediction_unit(s, x0, y0, cb_size / 2, cb_size, log2_cb_size, 0);
                hls_prediction_unit(s, x0 + cb_size / 2, y0, cb_size / 2, cb_size, log2_cb_size, 1);
                break;
            case PART_2NxnU:
                hls_prediction_unit(s, x0, y0, cb_size, cb_size / 4, log2_cb_size, 0);
                hls_prediction_unit(s, x0, y0 + cb_size / 4, cb_size, cb_size * 3 / 4, log2_cb_size, 1);
                break;
            case PART_2NxnD:
                hls_prediction_unit(s, x0, y0, cb_size, cb_size * 3 / 4, log2_cb_size, 0);
                hls_prediction_unit(s, x0, y0 + cb_size * 3 / 4, cb_size, cb_size / 4, log2_cb_size, 1);
                break;
            case PART_nLx2N:
                hls_prediction_unit(s, x0, y0, cb_size / 4, cb_size, log2_cb_size,0);
                hls_prediction_unit(s, x0 + cb_size / 4, y0, cb_size * 3 / 4, cb_size, log2_cb_size, 1);
                break;
            case PART_nRx2N:
                hls_prediction_unit(s, x0, y0, cb_size * 3 / 4, cb_size, log2_cb_size,0);
                hls_prediction_unit(s, x0 + cb_size * 3 / 4, y0, cb_size/4, cb_size, log2_cb_size, 1);
                break;
            case PART_NxN:
                hls_prediction_unit(s, x0, y0, cb_size / 2, cb_size / 2, log2_cb_size, 0);
                hls_prediction_unit(s, x0 + cb_size / 2, y0, cb_size / 2, cb_size / 2, log2_cb_size, 1);
                hls_prediction_unit(s, x0, y0 + cb_size / 2, cb_size / 2, cb_size / 2, log2_cb_size, 2);
                hls_prediction_unit(s, x0 + cb_size / 2, y0 + cb_size / 2, cb_size / 2, cb_size / 2, log2_cb_size, 3);
                break;
            }
        }
        if (!lc->cu.pcm_flag) {
            if (lc->cu.pred_mode != MODE_INTRA &&
                !(lc->cu.part_mode == PART_2Nx2N && lc->pu.merge_flag)) {
                lc->cu.rqt_root_cbf = ff_hevc_no_residual_syntax_flag_decode(s);
            }
            if (lc->cu.rqt_root_cbf) {
                lc->cu.max_trafo_depth = lc->cu.pred_mode == MODE_INTRA ?
                                        sc->sps->max_transform_hierarchy_depth_intra + lc->cu.intra_split_flag :
                                        sc->sps->max_transform_hierarchy_depth_inter;
                hls_transform_tree(s, x0, y0, x0, y0, x0, y0, log2_cb_size,
                                   log2_cb_size, 0, 0);
            } else {
                if (!sc->sh.disable_deblocking_filter_flag) {
                    ff_hevc_deblocking_boundary_strengths(s, x0, y0, log2_cb_size);
                    if (sc->pps->transquant_bypass_enable_flag && lc->cu.cu_transquant_bypass_flag) {
                        set_deblocking_bypass(s, x, y, log2_cb_size);
                    }
                }
            }
        }
    }
    if( sc->pps->cu_qp_delta_enabled_flag && lc->tu.is_cu_qp_delta_coded == 0)
        ff_hevc_set_qPy(s, x0, y0, x0, y0, log2_cb_size);
    x = y_cb * pic_width_in_ctb + x_cb;
    for (y = 0; y < length; y++) {
        memset(&sc->qp_y_tab[x], lc->qp_y, length);
        x += pic_width_in_ctb;
    }
    set_ct_depth(s, x0, y0, log2_cb_size, lc->ct.depth);
}

static int hls_coding_quadtree(HEVCContext *s, int x0, int y0, int log2_cb_size, int cb_depth)
{
    HEVCSharedContext *sc = s->HEVCsc;
    HEVCLocalContext *lc = s->HEVClc;
    lc->ct.depth = cb_depth;
    if ((x0 + (1 << log2_cb_size) <= sc->sps->pic_width_in_luma_samples) &&
        (y0 + (1 << log2_cb_size) <= sc->sps->pic_height_in_luma_samples) &&
        log2_cb_size > sc->sps->log2_min_coding_block_size) {
        SAMPLE(sc->split_cu_flag, x0, y0) =
        ff_hevc_split_coding_unit_flag_decode(s, cb_depth, x0, y0);
    } else {
        SAMPLE(sc->split_cu_flag, x0, y0) =
        (log2_cb_size > sc->sps->log2_min_coding_block_size);
    }
    if( sc->pps->cu_qp_delta_enabled_flag
       && log2_cb_size >= sc->sps->log2_ctb_size - sc->pps->diff_cu_qp_delta_depth ) {
        lc->tu.is_cu_qp_delta_coded = 0;
        lc->tu.cu_qp_delta = 0;
    }

    if (SAMPLE(sc->split_cu_flag, x0, y0)) {
        int more_data = 0;
        int cb_size = (1 << (log2_cb_size)) >> 1;
        int x1 = x0 + cb_size;
        int y1 = y0 + cb_size;
        more_data = hls_coding_quadtree(s, x0, y0, log2_cb_size - 1, cb_depth + 1);

        if (more_data && x1 < sc->sps->pic_width_in_luma_samples)
            more_data = hls_coding_quadtree(s, x1, y0, log2_cb_size - 1, cb_depth + 1);
        if (more_data && y1 < sc->sps->pic_height_in_luma_samples)
            more_data = hls_coding_quadtree(s, x0, y1, log2_cb_size - 1, cb_depth + 1);
        if (more_data && x1 < sc->sps->pic_width_in_luma_samples &&
            y1 < sc->sps->pic_height_in_luma_samples) {
            return hls_coding_quadtree(s, x1, y1, log2_cb_size - 1, cb_depth + 1);
        }
        if (more_data)
            return ((x1 + cb_size) < sc->sps->pic_width_in_luma_samples ||
                    (y1 + cb_size) < sc->sps->pic_height_in_luma_samples);
        else
            return 0;
    } else {
        hls_coding_unit(s, x0, y0, log2_cb_size);
        if ((!((x0 + (1 << log2_cb_size)) %
               (1 << (sc->sps->log2_ctb_size))) ||
             (x0 + (1 << log2_cb_size) >= sc->sps->pic_width_in_luma_samples)) &&
            (!((y0 + (1 << log2_cb_size)) %
               (1 << (sc->sps->log2_ctb_size))) ||
             (y0 + (1 << log2_cb_size) >= sc->sps->pic_height_in_luma_samples))) {
                int end_of_slice_flag = ff_hevc_end_of_slice_flag_decode(s);
                return !end_of_slice_flag;
            } else {
                return 1;
            }
    }

    return 0;
}

/**
 * 7.3.4
 */

static void hls_decode_neighbour(HEVCContext *s, int x_ctb, int y_ctb, int ctb_addr_ts)
{
    HEVCSharedContext *sc = s->HEVCsc;
    HEVCLocalContext *lc = s->HEVClc;
    int ctb_size          = 1 << sc->sps->log2_ctb_size;
    int ctb_addr_rs       = sc->pps->ctb_addr_ts_to_rs[ctb_addr_ts];
    int ctb_addr_in_slice = ctb_addr_rs - sc->SliceAddrRs;
    if (sc->pps->entropy_coding_sync_enabled_flag) {
        if (x_ctb == 0 && (y_ctb&(ctb_size-1)) == 0)
            lc->isFirstQPgroup = 1;
        lc->end_of_tiles_x = sc->sps->pic_width_in_luma_samples;
    } else if (sc->pps->tiles_enabled_flag ) {
        if (ctb_addr_ts != 0 && sc->pps->tile_id[ctb_addr_ts] != sc->pps->tile_id[ctb_addr_ts-1]) {
            int idxX             = (x_ctb >> sc->sps->log2_ctb_size) / sc->pps->column_width[0];
            lc->start_of_tiles_x = x_ctb;
            lc->end_of_tiles_x   = x_ctb + (sc->pps->column_width[idxX]<< sc->sps->log2_ctb_size);
            lc->isFirstQPgroup   = 1;
        }
    } else {
        lc->end_of_tiles_x = sc->sps->pic_width_in_luma_samples;
    }
    lc->end_of_tiles_y = y_ctb + ctb_size;
    if (y_ctb + ctb_size >= sc->sps->pic_height_in_luma_samples)
        lc->end_of_tiles_y = sc->sps->pic_height_in_luma_samples;
    lc->ctb_left_flag = ((x_ctb > 0) && (ctb_addr_in_slice > 0) && (sc->pps->tile_id[ctb_addr_ts] == sc->pps->tile_id[sc->pps->ctb_addr_rs_to_ts[ctb_addr_rs-1]]));
    lc->ctb_up_flag   = ((y_ctb > 0)  && (ctb_addr_in_slice >= sc->sps->pic_width_in_ctbs) && (sc->pps->tile_id[ctb_addr_ts] == sc->pps->tile_id[sc->pps->ctb_addr_rs_to_ts[ctb_addr_rs - sc->sps->pic_width_in_ctbs]]));
    lc->ctb_up_right_flag = ((y_ctb > 0)  && (ctb_addr_in_slice+1 >= sc->sps->pic_width_in_ctbs) && (sc->pps->tile_id[ctb_addr_ts] == sc->pps->tile_id[sc->pps->ctb_addr_rs_to_ts[ctb_addr_rs+1 - sc->sps->pic_width_in_ctbs]]));
    lc->ctb_up_left_flag = ((x_ctb > 0) && (y_ctb > 0)  && (ctb_addr_in_slice-1 >= sc->sps->pic_width_in_ctbs) && (sc->pps->tile_id[ctb_addr_ts] == sc->pps->tile_id[sc->pps->ctb_addr_rs_to_ts[ctb_addr_rs-1 - sc->sps->pic_width_in_ctbs]]));
}

static int hls_decode_entry(AVCodecContext *avctxt, void *isFilterThread)
{
    HEVCContext *s  = avctxt->priv_data;
    HEVCSharedContext *sc = s->HEVCsc;

    int ctb_size    = 1 << sc->sps->log2_ctb_size;
    int more_data   = 1;
    int x_ctb       = 0;
    int y_ctb       = 0;
    int ctb_addr_ts = sc->pps->ctb_addr_rs_to_ts[sc->sh.slice_ctb_addr_rs];

    while (more_data) {
        int ctb_addr_rs       = sc->pps->ctb_addr_ts_to_rs[ctb_addr_ts];
        x_ctb = (ctb_addr_rs % ((sc->sps->pic_width_in_luma_samples + (ctb_size - 1))>> sc->sps->log2_ctb_size)) << sc->sps->log2_ctb_size;
        y_ctb = (ctb_addr_rs / ((sc->sps->pic_width_in_luma_samples + (ctb_size - 1))>> sc->sps->log2_ctb_size)) << sc->sps->log2_ctb_size;
        hls_decode_neighbour(s,x_ctb, y_ctb, ctb_addr_ts);
        ff_hevc_cabac_init(s, ctb_addr_ts);
        if (sc->sh.slice_sample_adaptive_offset_flag[0] || sc->sh.slice_sample_adaptive_offset_flag[1])
            hls_sao_param(s, x_ctb >> sc->sps->log2_ctb_size, y_ctb >> sc->sps->log2_ctb_size);
        sc->deblock[ctb_addr_rs].disable = sc->sh.disable_deblocking_filter_flag;
        sc->deblock[ctb_addr_rs].beta_offset = sc->sh.beta_offset;
        sc->deblock[ctb_addr_rs].tc_offset = sc->sh.tc_offset;
        more_data = hls_coding_quadtree(s, x_ctb, y_ctb, sc->sps->log2_ctb_size, 0);
        ctb_addr_ts++;
        save_states(s, ctb_addr_ts);
        hls_filters(s, x_ctb, y_ctb, ctb_size);
    }
    if (x_ctb + ctb_size >= sc->sps->pic_width_in_luma_samples && y_ctb + ctb_size >= sc->sps->pic_height_in_luma_samples)
        hls_filter(s, x_ctb, y_ctb);
    return ctb_addr_ts;
}

static int hls_slice_data(HEVCContext *s)
{
    int arg[2];
    int ret[2];
    HEVCSharedContext *sc = s->HEVCsc;

    arg[0] = 0;
    arg[1] = 1;
    if (sc->sh.first_slice_in_pic_flag == 1) {
        sc->SliceAddrRs = sc->sh.slice_address;
    } else {
        sc->SliceAddrRs = (sc->sh.dependent_slice_segment_flag == 0 ? sc->sh.slice_address : sc->SliceAddrRs);
    }
    av_atomic_int_set(&sc->coding_tree_count, 0);

    s->avctx->execute(s->avctx, hls_decode_entry, arg, ret , 1, sizeof(int));
    return ret[0];
}

#define SHIFT_CTB_WPP 2

static int hls_decode_entry_wpp(AVCodecContext *avctxt, void *input_ctb_row)
{
    HEVCContext *s  = avctxt->priv_data;
    HEVCSharedContext *sc = s->HEVCsc;
    HEVCLocalContext *lc;

    int ctb_size    = 1<< sc->sps->log2_ctb_size;
    int more_data   = 1;

    int *ctb_row    = input_ctb_row;
    int ctb_addr_rs = sc->sh.slice_ctb_addr_rs + (*ctb_row) * ((sc->sps->pic_width_in_luma_samples + (ctb_size - 1))>> sc->sps->log2_ctb_size);
    int ctb_addr_ts = sc->pps->ctb_addr_rs_to_ts[ctb_addr_rs];
    s = s->sList[(*ctb_row)%s->threads_number];
    lc = s->HEVClc;
    if(*ctb_row) {
        init_get_bits(lc->gb, sc->data+sc->sh.offset[(*ctb_row)-1], sc->sh.size[(*ctb_row)-1]*8);
        ff_init_cabac_decoder(lc->cc, sc->data+sc->sh.offset[(*ctb_row)-1], sc->sh.size[(*ctb_row)-1]);
    }
    while(more_data) {
        int x_ctb = (ctb_addr_rs % ((sc->sps->pic_width_in_luma_samples + (ctb_size - 1))>> sc->sps->log2_ctb_size)) << sc->sps->log2_ctb_size;
        int y_ctb = (ctb_addr_rs / ((sc->sps->pic_width_in_luma_samples + (ctb_size - 1))>> sc->sps->log2_ctb_size)) << sc->sps->log2_ctb_size;
        hls_decode_neighbour(s, x_ctb, y_ctb, ctb_addr_ts);
        while(*ctb_row && (av_atomic_int_get(&sc->ctb_entry_count[(*ctb_row)-1])-av_atomic_int_get(&sc->ctb_entry_count[(*ctb_row)]))<SHIFT_CTB_WPP);
        if (av_atomic_int_get(&sc->ERROR)){
        	av_atomic_int_add_and_fetch(&sc->ctb_entry_count[*ctb_row],SHIFT_CTB_WPP);
        	return 0;
        }
        ff_hevc_cabac_init(s, ctb_addr_ts);
        if (sc->sh.slice_sample_adaptive_offset_flag[0] ||
            sc->sh.slice_sample_adaptive_offset_flag[1])
            hls_sao_param(s, x_ctb >> sc->sps->log2_ctb_size, y_ctb >> sc->sps->log2_ctb_size);

        more_data = hls_coding_quadtree(s, x_ctb, y_ctb, sc->sps->log2_ctb_size, 0);
        ctb_addr_ts++;
        ctb_addr_rs       = sc->pps->ctb_addr_ts_to_rs[ctb_addr_ts];
        save_states(s, ctb_addr_ts);
        av_atomic_int_add_and_fetch(&sc->ctb_entry_count[*ctb_row],1);
        hls_filters(s, x_ctb, y_ctb, ctb_size);
        if (!more_data && (x_ctb+ctb_size) < sc->sps->pic_width_in_luma_samples && (y_ctb+ctb_size) < sc->sps->pic_height_in_luma_samples) {
        	av_atomic_int_set(&sc->ERROR,  1);
            av_atomic_int_add_and_fetch(&sc->ctb_entry_count[*ctb_row],SHIFT_CTB_WPP);
            return 0;
        }

        if (!more_data) {
            hls_filter(s, x_ctb, y_ctb);
            av_atomic_int_add_and_fetch(&sc->ctb_entry_count[*ctb_row],SHIFT_CTB_WPP);
            return ctb_addr_ts;
        }
        x_ctb+=ctb_size;

        if(x_ctb >= sc->sps->pic_width_in_luma_samples) {
            break;
        }
    }
    av_atomic_int_add_and_fetch(&sc->ctb_entry_count[*ctb_row],SHIFT_CTB_WPP);
    return 0;
}

static int hls_slice_data_wpp(HEVCContext *s, AVPacket *avpkt)
{
    HEVCSharedContext *sc = s->HEVCsc;
    HEVCLocalContext *lc = s->HEVClc;
    int *ret = av_malloc((sc->sh.num_entry_point_offsets+1)*sizeof(int));
    int *arg = av_malloc((sc->sh.num_entry_point_offsets+1)*sizeof(int));
    int i, j, res = 0;
    int offset;
#ifdef WPP1
    int startheader, cmpt = 0;
#endif

    if(!sc->ctb_entry_count) {
        sc->ctb_entry_count = av_malloc((sc->sh.num_entry_point_offsets+1)*sizeof(int));


        for(i=1; i< s->threads_number; i++) {
            s->sList[i] =  av_malloc(sizeof(HEVCContext));
            memcpy(s->sList[i], s, sizeof(HEVCContext));
            s->HEVClcList[i] = av_malloc(sizeof(HEVCLocalContext));
            for (j = 0; j < MAX_TRANSFORM_DEPTH; j++) {
                s->HEVClcList[i]->tt.cbf_cb[j] = av_malloc(MAX_CU_SIZE*MAX_CU_SIZE);
                s->HEVClcList[i]->tt.cbf_cr[j] = av_malloc(MAX_CU_SIZE*MAX_CU_SIZE);
                if (!s->HEVClcList[i]->tt.cbf_cb[j] || !s->HEVClcList[i]->tt.cbf_cr[j])
                    return AVERROR(ENOMEM);
            }
            s->HEVClcList[i]->gb = av_malloc(sizeof(GetBitContext));
            s->HEVClcList[i]->ctx_set = 0;
            s->HEVClcList[i]->greater1_ctx = 0;
            s->HEVClcList[i]->last_coeff_abs_level_greater1_flag = 0;
            s->HEVClcList[i]->cabac_state = av_malloc(HEVC_CONTEXTS);
            s->HEVClcList[i]->cc = av_malloc(sizeof(CABACContext));
            s->HEVClcList[i]->edge_emu_buffer = av_malloc((MAX_PB_SIZE + 7) * s->HEVCsc->frame->linesize[0]);
            s->sList[i]->HEVClc = s->HEVClcList[i];
        }
    }

    offset = (lc->gb->index>>3);

#ifdef WPP1
    for(j=0, cmpt = 0,startheader=offset+sc->sh.entry_point_offset[0]; j< sc->skipped_bytes; j++){
        if(sc->skipped_bytes_pos[j] >= offset && sc->skipped_bytes_pos[j] < startheader){
            startheader--;
            cmpt++;
        }
    }
#endif
    for(i=1; i< sc->sh.num_entry_point_offsets; i++) {
#ifdef WPP1
        offset += (sc->sh.entry_point_offset[i-1]-cmpt);
        for(j=0, cmpt=0, startheader=offset+sc->sh.entry_point_offset[i]; j< sc->skipped_bytes; j++){
            if(sc->skipped_bytes_pos[j] >= offset && sc->skipped_bytes_pos[j] < startheader){
                startheader--;
                cmpt++;
            }
        }
        sc->sh.size[i-1] = sc->sh.entry_point_offset[i]-cmpt;
#else
        offset += (sc->sh.entry_point_offset[i-1]);
        sc->sh.size[i-1] = sc->sh.entry_point_offset[i];
#endif
        sc->sh.offset[i-1] = offset;
    }
    if(sc->sh.num_entry_point_offsets!= 0) {
#ifdef WPP1
        offset += sc->sh.entry_point_offset[sc->sh.num_entry_point_offsets-1]-cmpt;
#else
        offset += sc->sh.entry_point_offset[sc->sh.num_entry_point_offsets-1];
#endif
        sc->sh.size[sc->sh.num_entry_point_offsets-1] = avpkt->size-offset;
        sc->sh.offset[sc->sh.num_entry_point_offsets-1] = offset;
    }


    for(i=1; i< s->threads_number; i++) {
        s->sList[i]->HEVClc->isFirstQPgroup = 1;
        s->sList[i]->HEVClc->qp_y = s->sList[0]->HEVClc->qp_y;

    }
    sc->data = avpkt->data;
    if (sc->sh.first_slice_in_pic_flag == 1) {
        sc->SliceAddrRs = sc->sh.slice_address;
    } else {
        sc->SliceAddrRs = (sc->sh.dependent_slice_segment_flag == 0 ? sc->sh.slice_address : sc->SliceAddrRs);
    }
    memset(sc->ctb_entry_count, 0, (sc->sh.num_entry_point_offsets+1)*sizeof(int));
    av_atomic_int_set(&sc->ERROR,  0);
    for(i=0; i<=sc->sh.num_entry_point_offsets; i++) {
        arg[i] = i;
        ret[i] = 0;
    }


    s->avctx->execute(s->avctx, hls_decode_entry_wpp, arg, ret ,sc->sh.num_entry_point_offsets+1, sizeof(int));
    for(i=0; i<=sc->sh.num_entry_point_offsets; i++)
        res += ret[i];
    av_free(ret);
    av_free(arg);
    return res;
}


/**
 * @return AVERROR_INVALIDDATA if the packet is not a valid NAL unit,
 * 0 if the unit should be skipped, 1 otherwise
 */
static int hls_nal_unit(HEVCContext *s)
{
    GetBitContext *gb = s->HEVClc->gb;
    int nuh_layer_id;

    if (get_bits1(gb) != 0)
        return AVERROR_INVALIDDATA;

    s->HEVCsc->nal_unit_type = get_bits(gb, 6);

    nuh_layer_id = get_bits(gb, 6);
    s->HEVCsc->temporal_id = get_bits(gb, 3) - 1;
    if (s->HEVCsc->temporal_id < 0)
        return AVERROR_INVALIDDATA;

    av_log(s->avctx, AV_LOG_DEBUG,
           "nal_unit_type: %d, nuh_layer_id: %dtemporal_id: %d\n",
           s->HEVCsc->nal_unit_type, nuh_layer_id, s->HEVCsc->temporal_id);

    return (nuh_layer_id == 0);
}

static void printf_ref_pic_list(HEVCContext *s)
{
    RefPicList  *refPicList = s->HEVCsc->ref->refPicList;

    uint8_t i, list_idx;
    if (s->HEVCsc->sh.slice_type == I_SLICE)
        printf("\nPOC %4d TId: %1d ( I-SLICE, QP%3d ) ", s->HEVCsc->poc, s->HEVCsc->temporal_id, s->HEVCsc->sh.slice_qp);
    else if (s->HEVCsc->sh.slice_type == B_SLICE)
        printf("\nPOC %4d TId: %1d ( B-SLICE, QP%3d ) ", s->HEVCsc->poc, s->HEVCsc->temporal_id, s->HEVCsc->sh.slice_qp);
    else
        printf("\nPOC %4d TId: %1d ( P-SLICE, QP%3d ) ", s->HEVCsc->poc, s->HEVCsc->temporal_id, s->HEVCsc->sh.slice_qp);

    for ( list_idx = 0; list_idx < 2; list_idx++) {
        printf("[L%d ",list_idx);
        for(i = 0; i < refPicList[list_idx].numPic; i++) {
            int currIsLongTerm = refPicList[list_idx].isLongTerm[i];
//            if (currIsLongTerm)
//                printf("%d* ",refPicList[list_idx].list[i]);
//            else
                printf("%d ",refPicList[list_idx].list[i]);
        }
        printf("] ");
    }
}

static void print_md5(int poc, uint8_t md5[3][16]) {
    int i, j;
    printf("\n[MD5:");
    for (j = 0; j < 3; j++) {
        printf("\n");
        for (i = 0; i < 16; i++)
            printf("%02x", md5[j][i]);
    }
    printf("\n]");

}

static void calc_md5(uint8_t *md5, uint8_t* src, int stride, int width, int height) {
    uint8_t *buf;
    int y,x;
    buf = av_malloc(width * height);

    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++)
            buf[y * width + x] = src[x];

        src += stride;
    }
    av_md5_sum(md5, buf, width * height);
    av_free(buf);
}


static int hevc_decode_frame(AVCodecContext *avctx, void *data, int *got_output,
                             AVPacket *avpkt)
{
    HEVCContext *s = avctx->priv_data;
    HEVCSharedContext *sc = s->HEVCsc;
    HEVCLocalContext *lc = s->HEVClc;
    GetBitContext *gb = lc->gb;


    int poc_display;

    int ctb_addr_ts;
    int ret;

    if (!avpkt->size) {
        if ((ret = ff_hevc_find_display(s, data, 1, &poc_display)) < 0)
            return ret;

        *got_output = ret;
        return 0;
    }

    init_get_bits(gb, avpkt->data, avpkt->size*8);
    av_log(s->avctx, AV_LOG_DEBUG, "=================\n");

    ret = hls_nal_unit(s);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "Invalid NAL unit %d, skipping.\n", sc->nal_unit_type);
        return avpkt->size;
    } else if (!ret)
        return avpkt->size;

    switch (sc->nal_unit_type) {
    case NAL_VPS:
        ff_hevc_decode_nal_vps(s);
        break;
    case NAL_SPS:
        ff_hevc_decode_nal_sps(s);
        break;
    case NAL_PPS:
        ff_hevc_decode_nal_pps(s);
        break;
    case NAL_SEI_PREFIX:
    case NAL_SEI_SUFFIX:
        ff_hevc_decode_nal_sei(s);
        break;
    case NAL_TRAIL_R:
    case NAL_TRAIL_N:
    case NAL_TSA_N:
    case NAL_TSA_R:
    case NAL_STSA_N:
    case NAL_STSA_R:
    case NAL_BLA_W_LP:
    case NAL_BLA_W_RADL:
    case NAL_BLA_N_LP:
    case NAL_IDR_W_RADL:
    case NAL_IDR_N_LP:
    case NAL_CRA_NUT:
    case NAL_RADL_N:
    case NAL_RADL_R:
    case NAL_RASL_N:
    case NAL_RASL_R:
        ret = hls_slice_header(s);
        lc->isFirstQPgroup = !sc->sh.dependent_slice_segment_flag;
        
        if (ret < 0)
            if (ret == AVERROR_INVALIDDATA)
                return avpkt->size;
            else
                    return ret;
        if (sc->max_ra == INT_MAX) {
            if (sc->nal_unit_type == NAL_CRA_NUT ||
                    sc->nal_unit_type == NAL_BLA_W_LP ||
                    sc->nal_unit_type == NAL_BLA_N_LP ||
                    sc->nal_unit_type == NAL_BLA_N_LP) {
                sc->max_ra = sc->poc;
            } else {
                if (sc->nal_unit_type == NAL_IDR_W_RADL || sc->nal_unit_type == NAL_IDR_N_LP)
                    sc->max_ra = INT_MIN;
            }
        }

        if (sc->nal_unit_type == NAL_RASL_R && sc->poc <= sc->max_ra) {
            sc->is_decoded = 0;
            printf("not decoded %d %d\n", sc->poc, sc->max_ra);
            break;
        } else {
            if (sc->nal_unit_type == NAL_RASL_R && sc->poc > sc->max_ra)
                sc->max_ra = INT_MIN;
        }

        if(sc->sh.first_slice_in_pic_flag) {
            int pic_width_in_min_pu  = sc->sps->pic_width_in_min_cbs * 4;
            int pic_height_in_min_pu = sc->sps->pic_height_in_min_cbs * 4;
            memset(sc->horizontal_bs, 0, 2 * sc->bs_width * sc->bs_height);
            memset(sc->vertical_bs, 0, sc->bs_width * 2 * sc->bs_height);
            memset(sc->cbf_luma, 0 , pic_width_in_min_pu * pic_height_in_min_pu);
            memset(sc->is_pcm, 0 , pic_width_in_min_pu * pic_height_in_min_pu);
            lc->start_of_tiles_x = 0;
            if (sc->pps->tiles_enabled_flag )
	            lc->end_of_tiles_x   = sc->pps->column_width[0]<< sc->sps->log2_ctb_size;
        }
        if( !sc->pps->cu_qp_delta_enabled_flag )
            lc->qp_y = ((sc->sh.slice_qp + 52 + 2 * sc->sps->qp_bd_offset) %
                    (52 + sc->sps->qp_bd_offset)) - sc->sps->qp_bd_offset;

        if (sc->sh.first_slice_in_pic_flag) {
            if (sc->sps->sample_adaptive_offset_enabled_flag) {
                av_frame_unref(sc->tmp_frame);
                if ((ret = ff_reget_buffer(s->avctx, sc->tmp_frame)) < 0)
                    return ret;
                sc->frame = sc->tmp_frame;
                if ((ret = ff_hevc_set_new_ref(s, &sc->sao_frame, sc->poc))< 0)
                    return ret;
            } else {
                if ((ret = ff_hevc_set_new_ref(s, &sc->frame, sc->poc))< 0)
                    return ret;
            }
        }
        if (!lc->edge_emu_buffer)
            lc->edge_emu_buffer = av_malloc((MAX_PB_SIZE + 7) * sc->frame->linesize[0]);
        if (!lc->edge_emu_buffer)
            return -1;
        if(s->threads_number>1 && sc->sh.num_entry_point_offsets > 0 ) {
            ctb_addr_ts = hls_slice_data_wpp(s, avpkt);
        } else {
            //ctb_addr_ts = hls_slice_data_wpp(s, avpkt);
            ctb_addr_ts = hls_slice_data(s);
        }
        if (s->decode_checksum_sei && ctb_addr_ts >= (sc->sps->pic_width_in_ctbs * sc->sps->pic_height_in_ctbs)) {
#ifdef POC_DISPLAY_MD5
            AVFrame *frame = (AVFrame *) data;
            int poc        = poc_display;
#else
            AVFrame *frame = sc->ref->frame;
            int poc        = sc->poc;
#endif
            calc_md5(sc->md5[0], frame->data[0], frame->linesize[0], frame->width  , frame->height  );
            calc_md5(sc->md5[1], frame->data[1], frame->linesize[1], frame->width/2, frame->height/2);
            calc_md5(sc->md5[2], frame->data[2], frame->linesize[2], frame->width/2, frame->height/2);
            sc->is_decoded = 1;
            //printf_ref_pic_list(s);
            //print_md5(poc, sc->md5);
        }

        if (sc->sh.first_slice_in_pic_flag) {
            if ((ret = ff_hevc_find_display(s, data, 0, &poc_display)) < 0)
                return ret;
            sc->frame->pict_type = AV_PICTURE_TYPE_I;
            sc->frame->key_frame = 1;
            *got_output = ret;
        } else {
            *got_output = 0;
        }
        break;
    case NAL_AUD:
    case NAL_EOS_NUT:
    case NAL_EOB_NUT:
    case NAL_FD_NUT:
        return avpkt->size;
    default:
        av_log(s->avctx, AV_LOG_INFO, "Skipping NAL unit %d\n", sc->nal_unit_type);
        return avpkt->size;
    }

    av_log(s->avctx, AV_LOG_DEBUG, "%d bits left in unit\n", get_bits_left(gb));
    return avpkt->size;
}

static av_cold int hevc_decode_init(AVCodecContext *avctx)
{
    int i;
    HEVCContext *s = avctx->priv_data;

    HEVCSharedContext *sc;
    HEVCLocalContext *lc;

    s->avctx = avctx;
    s->HEVCsc = av_mallocz(sizeof(HEVCSharedContext));
    s->HEVClc = av_mallocz(sizeof(HEVCLocalContext));
    lc = s->HEVClcList[0] = s->HEVClc;
    sc = s->HEVCsc;
    s->sList[0] = s;


    sc->tmp_frame = av_frame_alloc();
    sc->cabac_state = av_malloc(HEVC_CONTEXTS);




    lc->gb = av_malloc(sizeof(GetBitContext));
    lc->cc = av_malloc(sizeof(CABACContext));
    lc->cabac_state = av_malloc(HEVC_CONTEXTS);
    lc->ctx_set = 0;
    lc->greater1_ctx = 0;
    lc->last_coeff_abs_level_greater1_flag = 0;
    if (!sc->tmp_frame)
        return AVERROR(ENOMEM);
    sc->max_ra = INT_MAX;
    for (i = 0; i < FF_ARRAY_ELEMS(sc->DPB); i++) {
        sc->DPB[i].frame = av_frame_alloc();
        if (!sc->DPB[i].frame)
            return AVERROR(ENOMEM);
    }
    memset(sc->vps_list, 0, sizeof(sc->vps_list));
    memset(sc->sps_list, 0, sizeof(sc->sps_list));
    memset(sc->pps_list, 0, sizeof(sc->pps_list));
    sc->ctb_entry_count  = NULL;
    for (i = 0; i < MAX_TRANSFORM_DEPTH; i++) {
        lc->tt.cbf_cb[i] = av_malloc(MAX_CU_SIZE*MAX_CU_SIZE);
        lc->tt.cbf_cr[i] = av_malloc(MAX_CU_SIZE*MAX_CU_SIZE);
        if (!lc->tt.cbf_cb[i] || !lc->tt.cbf_cr[i])
            return AVERROR(ENOMEM);
    }
    sc->skipped_buf_size = 0;
    s->threads_number = 1;
    return 0;
}

static av_cold int hevc_decode_free(AVCodecContext *avctx)
{
    int i, j;
    HEVCContext *s = avctx->priv_data;
    HEVCSharedContext *sc = s->HEVCsc;
    HEVCLocalContext *lc = s->HEVClc;

    av_free(sc->skipped_bytes_pos);
    av_frame_free(&sc->tmp_frame);
    av_free(sc->cabac_state);

    av_free(lc->cabac_state);
    av_free(lc->gb);
    av_free(lc->cc);
    av_free(lc->edge_emu_buffer);

    for (i = 0; i < MAX_TRANSFORM_DEPTH; i++) {
        av_freep(&lc->tt.cbf_cb[i]);
        av_freep(&lc->tt.cbf_cr[i]);
    }

    if(sc->ctb_entry_count) {
        av_freep(&sc->sh.entry_point_offset);
        av_freep(&sc->sh.offset);
        av_freep(&sc->sh.size);
        for (i = 1; i < s->threads_number; i++) {
            lc = s->HEVClcList[i];
            av_free(lc->gb);
            av_free(lc->cc);
            av_free(lc->edge_emu_buffer);
            for (j = 0; j < MAX_TRANSFORM_DEPTH; j++) {
                av_freep(&lc->tt.cbf_cb[j]);
                av_freep(&lc->tt.cbf_cr[j]);
            }
            av_free(lc->cabac_state);
            av_free(lc);
        }
        av_free(sc->ctb_entry_count);
        av_free(s->HEVClcList[i]);
    }
    for (i = 0; i < FF_ARRAY_ELEMS(sc->DPB); i++) {
        av_frame_free(&sc->DPB[i].frame);
    }
    for (i = 0; i < MAX_VPS_COUNT; i++) {
        av_freep(&sc->vps_list[i]);
    }
    for (i = 0; i < MAX_SPS_COUNT; i++) {
        av_freep(&sc->sps_list[i]);
    }
    for (i = 0; i < MAX_PPS_COUNT; i++) {
        if (sc->pps_list[i]) {
            av_freep(&sc->pps_list[i]->column_width);
            av_freep(&sc->pps_list[i]->row_height);
            av_freep(&sc->pps_list[i]->col_bd);
            av_freep(&sc->pps_list[i]->row_bd);
            av_freep(&sc->pps_list[i]->ctb_addr_rs_to_ts);
            av_freep(&sc->pps_list[i]->ctb_addr_ts_to_rs);
            av_freep(&sc->pps_list[i]->tile_id);
            av_freep(&sc->pps_list[i]->min_cb_addr_zs);
            av_freep(&sc->pps_list[i]->min_tb_addr_zs);
        }
        av_freep(&sc->pps_list[i]);
    }
    av_freep(&s->HEVClc);
    av_freep(&sc);
    pic_arrays_free(s);

    return 0;
}

static void hevc_decode_flush(AVCodecContext *avctx)
{
    HEVCContext *s = avctx->priv_data;
    ff_hevc_clean_refs(s);
    s->HEVCsc->max_ra = INT_MAX;
}

#define OFFSET(x) offsetof(HEVCContext, x)
#define PAR (AV_OPT_FLAG_DECODING_PARAM | AV_OPT_FLAG_VIDEO_PARAM)
static const AVOption options[] = {
    { "decode-checksum", "decode picture checksum SEI message", OFFSET(decode_checksum_sei),
        AV_OPT_TYPE_INT, {.i64 = 0}, 0, 1, PAR },
    { NULL },
};

static const AVClass hevc_decoder_class = {
    .class_name = "HEVC decoder",
    .item_name  = av_default_item_name,
    .option     = options,
    .version    = LIBAVUTIL_VERSION_INT,
};

AVCodec ff_hevc_decoder = {
    .name           = "hevc",
    .type           = AVMEDIA_TYPE_VIDEO,
    .id             = AV_CODEC_ID_HEVC,
    .priv_data_size = sizeof(HEVCContext),
    .priv_class     = &hevc_decoder_class,
    .init           = hevc_decode_init,
    .close          = hevc_decode_free,
    .decode         = hevc_decode_frame,
    .capabilities   = CODEC_CAP_DR1 | CODEC_CAP_DELAY | CODEC_CAP_SLICE_THREADS, // Add by wassim to support multi-threading
    .flush          = hevc_decode_flush,
    .long_name      = NULL_IF_CONFIG_SMALL("HEVC (High Efficiency Video Coding)"),
};
