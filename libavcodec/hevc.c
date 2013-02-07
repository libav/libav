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
// #define DEBUG

#include "libavutil/attributes.h"
#include "libavutil/common.h"
#include "libavutil/pixdesc.h"
#include "libavutil/internal.h"
#include "cabac_functions.h"
#include "golomb.h"
#include "hevcdata.h"
#include "hevc.h"

/**
 * NOTE: Each function hls_foo correspond to the function foo in the
 * specification (HLS stands for High Level Syntax).
 */

/**
 * Section 5.7
 */
#define INVERSE_RASTER_SCAN(a, b, c, d, e) ((e) ? ((a) / (ROUNDED_DIV(d, b)))*(c) : ((a)%(ROUNDED_DIV(d, b)))*(b))

static int pic_arrays_init(HEVCContext *s)
{
    int i;
    int pic_size = s->sps->pic_width_in_luma_samples * s->sps->pic_height_in_luma_samples;
    int ctb_count = s->sps->pic_width_in_ctbs * s->sps->pic_height_in_ctbs;
    int pic_width_in_min_pu  = s->sps->pic_width_in_min_cbs * 4;
    int pic_height_in_min_pu = s->sps->pic_height_in_min_cbs * 4;

    s->sao = av_mallocz(ctb_count * sizeof(*s->sao));

    s->split_coding_unit_flag = av_malloc(pic_size);
    s->cu.skip_flag = av_malloc(pic_size);

    s->cu.left_ct_depth = av_malloc(s->sps->pic_height_in_min_cbs);
    s->cu.top_ct_depth = av_malloc(s->sps->pic_width_in_min_cbs);

    s->pu.left_ipm = av_malloc(pic_height_in_min_pu);
    s->pu.top_ipm = av_malloc(pic_width_in_min_pu);
    s->pu.tab_mvf = av_malloc(pic_width_in_min_pu*pic_height_in_min_pu*sizeof(MvField));

    for( i =0; i<pic_width_in_min_pu*pic_height_in_min_pu ; i++ ) {
        s->pu.tab_mvf[i].ref_idx =  -1;
        s->pu.tab_mvf[i].mv.x = 0 ;
        s->pu.tab_mvf[i].mv.y =0;
        s->pu.tab_mvf[i].pred_flag =0;
        s->pu.tab_mvf[i].is_intra =0;
    }

    if (!s->sao || !s->split_coding_unit_flag || !s->cu.skip_flag ||
        !s->cu.left_ct_depth || !s->cu.top_ct_depth ||
        !s->pu.left_ipm || !s->pu.top_ipm || !s->pu.tab_mvf)
        return -1;

    for (i = 0; i < MAX_TRANSFORM_DEPTH; i++) {
        s->tt.cbf_cb[i] = av_malloc(MAX_CU_SIZE*MAX_CU_SIZE);
        s->tt.cbf_cr[i] = av_malloc(MAX_CU_SIZE*MAX_CU_SIZE);
        if (!s->tt.cbf_cb[i] ||
            !s->tt.cbf_cr[i])
            return -1;
    }

    return 0;
}

static void pic_arrays_free(HEVCContext *s)
{
    int i;
    av_freep(&s->sao);

    av_freep(&s->split_coding_unit_flag);
    av_freep(&s->cu.skip_flag);

    av_freep(&s->cu.left_ct_depth);
    av_freep(&s->cu.top_ct_depth);

    av_freep(&s->pu.left_ipm);
    av_freep(&s->pu.top_ipm);
    av_freep(&s->pu.tab_mvf);

    for (i = 0; i < MAX_TRANSFORM_DEPTH; i++) {
        av_freep(&s->tt.cbf_cb[i]);
        av_freep(&s->tt.cbf_cr[i]);
    }
    if (s->sh.entry_point_offset) {
        av_freep(&s->sh.entry_point_offset);
    }
}

static void compute_POC(HEVCContext *s, int iPOClsb)
{
    int iMaxPOClsb  = 1 << s->sps->log2_max_poc_lsb;
    int iPrevPOClsb = s->poc % iMaxPOClsb;
    int iPrevPOCmsb = s->poc - iPrevPOClsb;
    int iPOCmsb;
    if ((iPOClsb < iPrevPOClsb) && ((iPrevPOClsb - iPOClsb) >= (iMaxPOClsb / 2))) {
        iPOCmsb = iPrevPOCmsb + iMaxPOClsb;
    } else if ((iPOClsb > iPrevPOClsb) && ((iPOClsb - iPrevPOClsb) > (iMaxPOClsb / 2))) {
        iPOCmsb = iPrevPOCmsb - iMaxPOClsb;
    } else {
        iPOCmsb = iPrevPOCmsb;
    }
    s->poc = iPOCmsb + iPOClsb;
}
static void set_ref_pic_list(HEVCContext *s)
{
    SliceHeader *sh = &s->sh;
    RefPicList  *refPocList = s->sh.refPocList;
    RefPicList  *refPicList = s->sh.refPicList;

    uint8_t num_ref_idx_lx_act[2];
    uint8_t cIdx;
    uint8_t num_poc_total_curr;
    uint8_t num_rps_curr_lx;
    uint8_t first_list;
    uint8_t sec_list;
    uint8_t i, list_idx;

    num_ref_idx_lx_act[0] = sh->num_ref_idx_l0_active;
    num_ref_idx_lx_act[1] = sh->num_ref_idx_l1_active;
    for ( list_idx = 0; list_idx < 2; list_idx++) {
        /* The order of the elements is
         * ST_CURR_BEF - ST_CURR_AFT - LT_CURR for the RefList0 and
         * ST_CURR_AFT - ST_CURR_BEF - LT_CURR for the RefList1
         */
        first_list = list_idx == 0 ? ST_CURR_BEF : ST_CURR_AFT;
        sec_list   = list_idx == 0 ? ST_CURR_AFT : ST_CURR_BEF;

        /* even if num_ref_idx_lx_act is inferior to num_poc_total_curr we fill in
         * all the element from the Rps because we might reorder the list. If
         * we reorder the list might need a reference picture located after
         * num_ref_idx_lx_act.
         */
        num_poc_total_curr = refPocList[ST_CURR_BEF].numPic + refPocList[ST_CURR_AFT].numPic + refPocList[LT_CURR].numPic;
        num_rps_curr_lx    = num_poc_total_curr<num_ref_idx_lx_act[list_idx] ? num_poc_total_curr : num_ref_idx_lx_act[list_idx];
        cIdx = 0;
        while(cIdx < num_rps_curr_lx) {
            for(i = 0; i < refPocList[first_list].numPic; i++) {
                refPicList[list_idx].list[cIdx] = refPocList[first_list].list[i];
                cIdx++;
            }
            for(i = 0; i < refPicList[sec_list].numPic; i++) {
                refPicList[list_idx].list[cIdx] = refPocList[sec_list].list[i];
                cIdx++;
            }
            for(i = 0; i < refPocList[LT_CURR].numPic; i++) {
                refPicList[list_idx].list[cIdx] = refPocList[LT_CURR].list[i];
                cIdx++;
            }
        }
        refPicList[list_idx].numPic = cIdx;
    }
}
static void set_ref_poc_list(HEVCContext *s)
{
    int i;
    int j = 0;
    int k = 0;
    ShortTermRPS *rps        = s->sh.short_term_rps;
    RefPicList   *refPocList = s->sh.refPocList;
    if (rps != NULL) {
        for (i = 0; i < rps->num_negative_pics; i ++) {
            if ( rps->used[i] == 1 ) {
                refPocList[ST_CURR_BEF].list[j] = s->poc + rps->delta_poc[i];
                j++;
            } else {
                refPocList[ST_FOLL].list[k] = s->poc + rps->delta_poc[i];
                k++;
            }
        }
        refPocList[ST_CURR_BEF].numPic = j;
        j = 0;
        for( i = rps->num_negative_pics; i < rps->num_delta_pocs; i ++ ) {
            if (rps->used[i] ==1) {
                refPocList[ST_CURR_AFT].list[j] = s->poc + rps->delta_poc[i];
                j++;
            } else {
                refPocList[ST_FOLL].list[k] = s->poc + rps->delta_poc[i];
                k++;
            }
        }
        refPocList[ST_CURR_AFT].numPic = j;
        refPocList[ST_FOLL].numPic = k;
        refPocList[LT_CURR].numPic = 0;
        refPocList[LT_FOLL].numPic = 0;
        //set_ref_pic_list(s);
    }
}
static int hls_slice_header(HEVCContext *s)
{
    int i;
    GetBitContext *gb = &s->gb;
    SliceHeader *sh = &s->sh;
    int slice_address_length = 0;

    av_dlog(s->avctx, "Decoding slice\n");


    // Coded parameters

    sh->first_slice_in_pic_flag = get_bits1(gb);
    if (s->nal_unit_type >= 16 && s->nal_unit_type <= 23)
        sh->no_output_of_prior_pics_flag = get_bits1(gb);

    sh->pps_id = get_ue_golomb(gb);
    if (sh->pps_id >= MAX_PPS_COUNT || !s->pps_list[sh->pps_id]) {
        av_log(s->avctx, AV_LOG_ERROR, "PPS id out of range: %d\n", sh->pps_id);
        return -1;
    }
    s->pps = s->pps_list[sh->pps_id];
    if (s->sps != s->sps_list[s->pps->sps_id]) {
        s->sps = s->sps_list[s->pps->sps_id];
        s->vps = s->vps_list[s->sps->vps_id];
        //TODO: Handle switching between different SPS better
        pic_arrays_free(s);
        if (pic_arrays_init(s) < 0) {
            pic_arrays_free(s);
            return -1;
        }

        s->avctx->width = s->sps->pic_width_in_luma_samples;
        s->avctx->height = s->sps->pic_height_in_luma_samples;
        if (s->sps->chroma_format_idc == 0 || s->sps->separate_colour_plane_flag) {
            av_log(s->avctx, AV_LOG_ERROR,
                   "TODO: s->sps->chroma_format_idc == 0 || "
                   "s->sps->separate_colour_plane_flag\n");
            return -1;
        }

        if (s->sps->chroma_format_idc == 1) {
            switch (s->sps->bit_depth) {
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
            return -1;
        }
        s->sps->hshift[0] = s->sps->vshift[0] = 0;
        s->sps->hshift[2] =
        s->sps->hshift[1] = av_pix_fmt_descriptors[s->avctx->pix_fmt].log2_chroma_w;
        s->sps->vshift[2] =
        s->sps->vshift[1] = av_pix_fmt_descriptors[s->avctx->pix_fmt].log2_chroma_h;

        s->sps->pixel_shift = s->sps->bit_depth > 8;

        ff_hevc_pred_init(&s->hpc, s->sps->bit_depth);
        ff_hevc_dsp_init(&s->hevcdsp, s->sps->bit_depth);

        ff_dsputil_init(&s->dsp, s->avctx);
    }

    if (!sh->first_slice_in_pic_flag) {
        if (s->pps->dependent_slice_segments_enabled_flag)
            sh->dependent_slice_segment_flag = get_bits1(gb);

        slice_address_length = av_ceil_log2_c(s->sps->pic_width_in_ctbs *
                                              s->sps->pic_height_in_ctbs);
        sh->slice_address = get_bits(gb, slice_address_length);
    }

    if (!s->pps->dependent_slice_segments_enabled_flag) {
        for(i = 0; i < s->pps->num_extra_slice_header_bits; i++)
            skip_bits(gb, 1); // slice_reserved_undetermined_flag[]
        sh->slice_type = get_ue_golomb(gb);
        if (s->pps->output_flag_present_flag)
            sh->pic_output_flag = get_bits1(gb);

        if (s->sps->separate_colour_plane_flag == 1)
            sh->colour_plane_id = get_bits(gb, 2);

        if (s->nal_unit_type != NAL_IDR_W_DLP) {
            int short_term_ref_pic_set_sps_flag;
            sh->pic_order_cnt_lsb = get_bits(gb, s->sps->log2_max_poc_lsb);
            compute_POC(s, sh->pic_order_cnt_lsb);
            short_term_ref_pic_set_sps_flag = get_bits1(gb);
            if (!short_term_ref_pic_set_sps_flag) {
                ff_hevc_decode_short_term_rps(s, MAX_SHORT_TERM_RPS_COUNT, s->sps);
                sh->short_term_rps = &s->sps->short_term_rps_list[MAX_SHORT_TERM_RPS_COUNT];
            } else {
                int numbits = 0;
                int short_term_ref_pic_set_idx;
                while ((1 << numbits) < s->sps->num_short_term_ref_pic_sets)
                    numbits++;
                if (numbits > 0)
                    short_term_ref_pic_set_idx = get_bits(gb, numbits);
                else
                    short_term_ref_pic_set_idx = 0;
            }
            if (s->sps->long_term_ref_pics_present_flag) {
                av_log(s->avctx, AV_LOG_ERROR, "TODO: long_term_ref_pics_present_flag\n");
                return -1;
            }
        } else {
            s->poc = 0;
        }
        if (!s->pps) {
            av_log(s->avctx, AV_LOG_ERROR, "No PPS active while decoding slice\n");
            return -1;
        }

        if (s->sps->sample_adaptive_offset_enabled_flag) {
            sh->slice_sample_adaptive_offset_flag[0] = get_bits1(gb);
            sh->slice_sample_adaptive_offset_flag[2] =
            sh->slice_sample_adaptive_offset_flag[1] = get_bits1(gb);
        }

        sh->num_ref_idx_l0_active = 0;
        sh->num_ref_idx_l1_active = 0;
        if (s->sps->sps_temporal_mvp_enabled_flag && s->nal_unit_type != NAL_IDR_W_DLP) {
            sh->slice_temporal_mvp_enable_flag = get_bits1(gb);
        }
        if (sh->slice_type == P_SLICE || sh->slice_type == B_SLICE) {
            sh->num_ref_idx_l0_active = s->pps->num_ref_idx_l0_default_active;
            if (sh->slice_type == B_SLICE)
                sh->num_ref_idx_l1_active = s->pps->num_ref_idx_l1_default_active;
            sh->num_ref_idx_active_override_flag = get_bits1(gb);

            if (sh->num_ref_idx_active_override_flag) {
                sh->num_ref_idx_l0_active = get_ue_golomb(gb) + 1;
                if (sh->slice_type == B_SLICE)
                    sh->num_ref_idx_l1_active = get_ue_golomb(gb) + 1;
            }
            if (s->pps->lists_modification_present_flag) {
                av_log(s->avctx, AV_LOG_ERROR, "TODO: ref_pic_list_modification() \n");
                return -1;
            }

            if (sh->slice_type == B_SLICE)
                sh->mvd_l1_zero_flag = get_bits1(gb);

            if (s->pps->cabac_init_present_flag) {
                sh->cabac_init_flag = get_bits1(gb);
            }
            if (sh->slice_temporal_mvp_enable_flag) {
                if (sh->slice_type == B_SLICE) {
                    sh->collocated_from_l0_flag = get_bits1(gb);
                }
                if ((sh->collocated_from_l0_flag &&
                     sh->num_ref_idx_l0_active > 1) ||
                    (!sh->collocated_from_l0_flag &&
                     sh->num_ref_idx_l0_active > 1)) {
                    sh->collocated_ref_idx = get_ue_golomb(gb);
                }
            }


            sh->max_num_merge_cand = 5 - get_ue_golomb(gb);
        }
        set_ref_poc_list(s);
        sh->slice_qp_delta = get_se_golomb(gb);
        if (s->pps->pic_slice_level_chroma_qp_offsets_present_flag) {
            sh->slice_cb_qp_offset = get_se_golomb(gb);
            sh->slice_cr_qp_offset = get_se_golomb(gb);
        }
        if (s->pps->deblocking_filter_control_present_flag) {
            int deblocking_filter_override_flag = 0;
            if (s->pps->deblocking_filter_override_enabled_flag)
                deblocking_filter_override_flag = get_bits1(gb);
            if (deblocking_filter_override_flag) {
                sh->disable_deblocking_filter_flag = get_bits1(gb);
                if (!sh->disable_deblocking_filter_flag) {
                    sh->beta_offset = get_se_golomb(gb) * 2;
                    sh->tc_offset = get_se_golomb(gb) * 2;
                }
            } else {
                sh->disable_deblocking_filter_flag = s->pps->pps_disable_deblocking_filter_flag;
            }
        }

        if (s->pps->seq_loop_filter_across_slices_enabled_flag
            && (sh->slice_sample_adaptive_offset_flag[0] ||
                sh->slice_sample_adaptive_offset_flag[1] ||
                !sh->disable_deblocking_filter_flag)) {
            sh->slice_loop_filter_across_slices_enabled_flag = get_bits1(gb);
        } else {
            sh->slice_loop_filter_across_slices_enabled_flag =
            s->pps->seq_loop_filter_across_slices_enabled_flag;
        }
    }
    if( s->pps->tiles_enabled_flag == 1 || s->pps->entropy_coding_sync_enabled_flag == 1) {
        int num_entry_point_offsets = get_ue_golomb(gb);
        if( num_entry_point_offsets > 0 ) {
            int offset_len = get_ue_golomb(gb)+1;
            if (sh->entry_point_offset) {
                av_freep(&sh->entry_point_offset);
            }
            sh->entry_point_offset = av_malloc(num_entry_point_offsets);
            for( i = 0; i < num_entry_point_offsets; i++ ) {
                sh->entry_point_offset[i] = get_bits(gb, offset_len);
            }
        }
    }

    if (s->pps->slice_header_extension_present_flag) {
        int length = get_ue_golomb(gb);
        for (i = 0; i < length; i++)
            skip_bits(gb, 8); // slice_header_extension_data_byte
    }

    // Inferred parameters
    sh->slice_qp = 26 + s->pps->pic_init_qp_minus26 + sh->slice_qp_delta;
    sh->slice_ctb_addr_rs = sh->slice_address;
    sh->slice_cb_addr_zs = sh->slice_address <<
                           (s->sps->log2_diff_max_min_coding_block_size << 1);

    return 0;
}

#define CTB(tab, x, y) ((tab)[(y) * s->sps->pic_width_in_ctbs + (x)])

#define set_sao(elem, value)                            \
    if (!sao_merge_up_flag && !sao_merge_left_flag) {   \
        sao->elem = value;                              \
    } else if (sao_merge_left_flag) {                   \
        sao->elem = CTB(s->sao, rx-1, ry).elem;         \
    } else if (sao_merge_up_flag) {                     \
        sao->elem = CTB(s->sao, rx, ry-1).elem;         \
    } else {                                            \
        sao->elem = 0;                                  \
    }

static int hls_sao_param(HEVCContext *s, int rx, int ry)
{
    int c_idx, i;
    int sao_merge_left_flag = 0;
    int sao_merge_up_flag = 0;
    int shift = s->sps->bit_depth - FFMIN(s->sps->bit_depth, 10);

    SAOParams *sao = &CTB(s->sao, rx, ry);

    if (rx > 0) {
        int left_ctb_in_slice = s->ctb_addr_in_slice > 0;
        int left_ctb_in_tile = 1 ;//TODO: s->pps->tile_id[s->ctb_addr_ts] == s->pps->tile_id[s->pps->ctb_addr_rs_to_ts[s->ctb_addr_rs - 1]];
        if (left_ctb_in_slice && left_ctb_in_tile)
            sao_merge_left_flag = ff_hevc_sao_merge_flag_decode(s);
    }
    if (ry > 0 && !sao_merge_left_flag) {
        int up_ctb_in_slice = 1; //TODO: (s->ctb_addr_ts - s->pps->ctb_addr_rs_to_ts[s->ctb_addr_rs - s->sps->pic_width_in_ctbs]) <= s->ctb_addr_in_slice;
        int up_ctb_in_tile = 1; //TODO: (s->pps->tile_id[s->ctb_addr_ts] == s->pps->tile_id[s->pps->ctb_addr_rs_to_ts[s->ctb_addr_rs - s->sps->pic_width_in_ctbs]]);
        if (up_ctb_in_slice && up_ctb_in_tile)
            sao_merge_up_flag = ff_hevc_sao_merge_flag_decode(s);
    }
    for (c_idx = 0; c_idx < 3; c_idx++) {

        if (!s->sh.slice_sample_adaptive_offset_flag[c_idx])
            continue;

        if (c_idx == 2) {
            sao->type_idx[2] = sao->type_idx[1];
            sao->eo_class[2] = sao->eo_class[1];
        } else {
            set_sao(type_idx[c_idx], ff_hevc_sao_type_idx_decode(s));
        }
        av_log(s->avctx, AV_LOG_DEBUG, "sao_type_idx: %d\n",
               sao->type_idx[c_idx]);

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

static void sao_filter(HEVCContext *s)
{
    //TODO: This should be easily parallelizable
    //TODO: skip CBs when (cu_transquant_bypass_flag || (pcm_loop_filter_disable_flag && pcm_flag))
    int c_idx, y_ctb, x_ctb;
    for (c_idx = 0; c_idx < 3; c_idx++) {
        int stride = s->frame.linesize[c_idx];
        int ctb_size = (1 << (s->sps->log2_ctb_size)) >> s->sps->hshift[c_idx];
        for (y_ctb = 0; y_ctb < s->sps->pic_height_in_ctbs; y_ctb++) {
            for (x_ctb = 0; x_ctb < s->sps->pic_width_in_ctbs; x_ctb++) {
                struct SAOParams *sao = &CTB(s->sao, x_ctb, y_ctb);
                int x = x_ctb * ctb_size;
                int y = y_ctb * ctb_size;
                int width = FFMIN(ctb_size,
                                  (s->sps->pic_width_in_luma_samples >> s->sps->hshift[c_idx]) - x);
                int height = FFMIN(ctb_size,
                                   (s->sps->pic_height_in_luma_samples >> s->sps->vshift[c_idx]) - y);
                uint8_t *src = &s->frame.data[c_idx][y * stride + x];
                uint8_t *dst = &s->sao_frame.data[c_idx][y * stride + x];
                switch (sao->type_idx[c_idx]) {
                case SAO_BAND:
                    s->hevcdsp.sao_band_filter(dst, src, stride, sao->offset_val[c_idx],
                                               sao->band_position[c_idx], width, height);
                    break;
                case SAO_EDGE: {
                    int top    = y_ctb == 0;
                    int bottom = y_ctb == (s->sps->pic_height_in_ctbs - 1);
                    int left   = x_ctb == 0;
                    int right  = x_ctb == (s->sps->pic_width_in_ctbs - 1);
                    s->hevcdsp.sao_edge_filter(dst, src, stride, sao->offset_val[c_idx],
                                               sao->eo_class[c_idx],
                                               top, bottom, left, right, width, height);
                    break;
                }
                }
            }
        }
    }
}

#undef CTB


static av_always_inline int min_cb_addr_zs(HEVCContext *s, int x, int y)
{
    return s->pps->min_cb_addr_zs[y * s->sps->pic_width_in_min_cbs + x];
}

static void hls_residual_coding(HEVCContext *s, int x0, int y0, int log2_trafo_size, enum ScanType scan_idx, int c_idx)
{
#define GET_COORD(offset, n)                                    \
    do {                                                        \
        x_c = (scan_x_cg[offset >> 4] << 2) + scan_x_off[n];    \
        y_c = (scan_y_cg[offset >> 4] << 2) + scan_y_off[n];    \
    } while (0)

    int i;

    int transform_skip_flag = 0;

    int last_significant_coeff_x, last_significant_coeff_y;
    int last_scan_pos;
    int n_end;
    int num_coeff = 0;
    int num_last_subset;
    int x_cg_last_sig, y_cg_last_sig;

    const uint8_t *scan_x_cg, *scan_y_cg, *scan_x_off, *scan_y_off;

    ptrdiff_t stride = s->frame.linesize[c_idx];
    int hshift = s->sps->hshift[c_idx];
    int vshift = s->sps->vshift[c_idx];
    uint8_t *dst = &s->frame.data[c_idx][(y0 >> vshift) * stride +
                                         ((x0 >> hshift) << s->sps->pixel_shift)];

    int16_t coeffs[MAX_TB_SIZE * MAX_TB_SIZE] = { 0 };
    int trafo_size = 1 << log2_trafo_size;

    av_dlog(s->avctx, "scan_idx: %d, c_idx: %d\n",
           scan_idx, c_idx);
    memset(s->rc.significant_coeff_group_flag, 0, 8*8);

    if (log2_trafo_size == 1) {
        log2_trafo_size = 2;
    }


    if (s->pps->transform_skip_enabled_flag && !s->cu.cu_transquant_bypass_flag &&
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

    av_dlog(s->avctx, "last_significant_coeff_x: %d\n",
           last_significant_coeff_x);
    av_dlog(s->avctx, "last_significant_coeff_y: %d\n",
           last_significant_coeff_y);

    x_cg_last_sig = last_significant_coeff_x >> 2;
    y_cg_last_sig = last_significant_coeff_y >> 2;

    switch (scan_idx) {
    case SCAN_DIAG: {
        int last_x_c = last_significant_coeff_x % 4;
        int last_y_c = last_significant_coeff_y % 4;

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
    av_dlog(s->avctx, "num_coeff: %d\n",
           num_coeff);

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
            s->rc.significant_coeff_group_flag[x_cg][y_cg] =
            ff_hevc_significant_coeff_group_flag_decode(s, c_idx, x_cg, y_cg,
                                                        log2_trafo_size);
            implicit_non_zero_coeff = 1;
        } else {
            s->rc.significant_coeff_group_flag[x_cg][y_cg] =
            ((x_cg == x_cg_last_sig && y_cg == y_cg_last_sig) ||
             (x_cg == 0 && y_cg == 0));
        }
        av_dlog(s->avctx, "significant_coeff_group_flag[%d][%d]: %d\n",
               x_cg, y_cg, s->rc.significant_coeff_group_flag[x_cg][y_cg]);

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

            if (s->rc.significant_coeff_group_flag[x_cg][y_cg] &&
                (n > 0 || implicit_non_zero_coeff == 0)) {
                if (ff_hevc_significant_coeff_flag_decode(s, c_idx, x_c, y_c, log2_trafo_size, scan_idx) == 1) {
                    significant_coeff_flag_idx[nb_significant_coeff_flag] = n;
                    nb_significant_coeff_flag = nb_significant_coeff_flag + 1;
                    implicit_non_zero_coeff = 0;
                }
            } else {
                int last_cg = (x_c == (x_cg << 2) && y_c == (y_cg << 2));
                if (last_cg && implicit_non_zero_coeff && s->rc.significant_coeff_group_flag[x_cg][y_cg]) {
                    significant_coeff_flag_idx[nb_significant_coeff_flag] = n;
                    nb_significant_coeff_flag = nb_significant_coeff_flag + 1;
                }
            }
            av_dlog(s->avctx, "significant_coeff_flag(%d, %d): %d\n",
                   x_c, y_c, significant_coeff_flag_idx[n]);

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
                av_dlog(s->avctx, "coeff_abs_level_greater1_flag[%d]: %d\n",
                       n, coeff_abs_level_greater1_flag[n]);
            }

        sign_hidden = (last_nz_pos_in_cg - first_nz_pos_in_cg >= 4 &&
                       !s->cu.cu_transquant_bypass_flag);
        if (first_greater1_coeff_idx != -1) {
            coeff_abs_level_greater2_flag[first_greater1_coeff_idx] =
            ff_hevc_coeff_abs_level_greater2_flag_decode(s, c_idx, i, first_greater1_coeff_idx);
            av_dlog(s->avctx, "coeff_abs_level_greater2_flag[%d]: %d\n",
                   first_greater1_coeff_idx,
                   coeff_abs_level_greater2_flag[first_greater1_coeff_idx]);
        }
        if (!s->pps->sign_data_hiding_flag || !sign_hidden ) {
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
                if (s->pps->sign_data_hiding_flag && sign_hidden) {
                    sum_abs += trans_coeff_level;
                    if (n == first_nz_pos_in_cg && (sum_abs%2 == 1))
                        trans_coeff_level = -trans_coeff_level;
                }
            if (coeff_sign_flag >> 15)
                    trans_coeff_level = -trans_coeff_level;
            coeff_sign_flag <<= 1;
                num_sig_coeff++;
                av_dlog(s->avctx, "trans_coeff_level: %d\n",
                       trans_coeff_level);
                coeffs[y_c * trafo_size + x_c] = trans_coeff_level;

            }
        }

    if (s->cu.cu_transquant_bypass_flag) {
        s->hevcdsp.transquant_bypass(dst, coeffs, stride, log2_trafo_size);
    } else {
        int qp;
        //TODO: handle non-constant QP
        int qp_y_pred = s->sh.slice_qp;
        int qp_y = ((qp_y_pred + s->tu.cu_qp_delta + 52 + 2 * s->sps->qp_bd_offset) %
                    (52 + s->sps->qp_bd_offset)) - s->sps->qp_bd_offset;
        static int qp_c[] = { 29, 30, 31, 32, 33, 33, 34, 34, 35, 35, 36, 36, 37, 37 };
        if (c_idx == 0) {
            qp = qp_y + s->sps->qp_bd_offset;
        } else {
            int qp_i, offset;

            if (c_idx == 1) {
                offset = s->pps->cb_qp_offset + s->sh.slice_cb_qp_offset;
            } else {
                offset = s->pps->cr_qp_offset + s->sh.slice_cr_qp_offset;
            }
            qp_i = av_clip_c(qp_y + offset, - s->sps->qp_bd_offset, 57);
            if (qp_i < 30) {
                qp = qp_i;
            } else if (qp_i > 43) {
                qp = qp_i - 6;
            } else {
                qp = qp_c[qp_i - 30];
            }

            qp += s->sps->qp_bd_offset;

        }

        s->hevcdsp.dequant(coeffs, log2_trafo_size, qp);
        if (transform_skip_flag) {
            s->hevcdsp.transform_skip(dst, coeffs, stride);
        } else if (s->cu.pred_mode == MODE_INTRA && c_idx == 0 && log2_trafo_size == 2) {
            s->hevcdsp.transform_4x4_luma_add(dst, coeffs, stride);
        } else {
            s->hevcdsp.transform_add[log2_trafo_size-2](dst, coeffs, stride);
        }
    }
}

static void hls_transform_unit(HEVCContext *s, int x0, int  y0, int xBase, int yBase,
                               int log2_trafo_size, int trafo_depth, int blk_idx) {
    int scan_idx = SCAN_DIAG;
    int scan_idx_c = SCAN_DIAG;
    if (s->cu.pred_mode == MODE_INTRA) {
        s->hpc.intra_pred(s, x0, y0, log2_trafo_size, 0);
        if (log2_trafo_size > 2) {
            s->hpc.intra_pred(s, x0, y0, log2_trafo_size - 1, 1);
            s->hpc.intra_pred(s, x0, y0, log2_trafo_size - 1, 2);
        } else if (blk_idx == 3) {
            s->hpc.intra_pred(s, xBase, yBase, log2_trafo_size, 1);
            s->hpc.intra_pred(s, xBase, yBase, log2_trafo_size, 2);
        }
    }

    if (s->tt.cbf_luma ||
        SAMPLE_CBF(s->tt.cbf_cb[trafo_depth], x0, y0) ||
        SAMPLE_CBF(s->tt.cbf_cr[trafo_depth], x0, y0)) {
        if (s->pps->cu_qp_delta_enabled_flag && !s->tu.is_cu_qp_delta_coded) {
            av_log(s->avctx, AV_LOG_ERROR, "TODO: cu_qp_delta_enabled_flag\n");
            s->tu.is_cu_qp_delta_coded = 1;
        }

        if (s->cu.pred_mode == MODE_INTRA && log2_trafo_size < 4) {
            if (s->tu.cur_intra_pred_mode >= 6 &&
                s->tu.cur_intra_pred_mode <= 14) {
                scan_idx = SCAN_VERT;
            } else if (s->tu.cur_intra_pred_mode >= 22 &&
                       s->tu.cur_intra_pred_mode <= 30) {
                scan_idx = SCAN_HORIZ;
            }

            if (s->pu.intra_pred_mode_c >= 6 &&
                s->pu.intra_pred_mode_c <= 14) {
                scan_idx_c = SCAN_VERT;
            } else if (s->pu.intra_pred_mode_c >= 22 &&
                       s->pu.intra_pred_mode_c <= 30) {
                scan_idx_c = SCAN_HORIZ;
            }
        }

        if (s->tt.cbf_luma)
            hls_residual_coding(s, x0, y0, log2_trafo_size, scan_idx, 0);
        if (log2_trafo_size > 2) {
            if (SAMPLE_CBF(s->tt.cbf_cb[trafo_depth], x0, y0))
                hls_residual_coding(s, x0, y0, log2_trafo_size - 1, scan_idx_c, 1);
            if (SAMPLE_CBF(s->tt.cbf_cr[trafo_depth], x0, y0))
                hls_residual_coding(s, x0, y0, log2_trafo_size - 1, scan_idx_c, 2);
        } else if (blk_idx == 3) {
            if (SAMPLE_CBF(s->tt.cbf_cb[trafo_depth], xBase, yBase))
                hls_residual_coding(s, xBase, yBase, log2_trafo_size, scan_idx_c, 1);
            if (SAMPLE_CBF(s->tt.cbf_cr[trafo_depth], xBase, yBase))
                hls_residual_coding(s, xBase, yBase, log2_trafo_size, scan_idx_c, 2);
        }
    }
}

static void hls_transform_tree(HEVCContext *s, int x0, int y0,
                               int xBase, int yBase, int log2_cb_size, int log2_trafo_size,
                               int trafo_depth, int blk_idx)
{

    uint8_t split_transform_flag;
    SAMPLE_CBF(s->tt.cbf_cb[trafo_depth], x0, y0) =
    SAMPLE_CBF(s->tt.cbf_cb[trafo_depth - 1], xBase, yBase);
    SAMPLE_CBF(s->tt.cbf_cr[trafo_depth], x0, y0) =
    SAMPLE_CBF(s->tt.cbf_cr[trafo_depth - 1], xBase, yBase);

    if (s->cu.intra_split_flag) {
        if (trafo_depth == 1)
            s->tu.cur_intra_pred_mode = s->pu.intra_pred_mode[blk_idx];
    } else {
        s->tu.cur_intra_pred_mode = s->pu.intra_pred_mode[0];
    }

    s->tt.cbf_luma = 1;

    s->tt.inter_split_flag = (s->sps->max_transform_hierarchy_depth_inter == 0 &&
                              s->cu.pred_mode == MODE_INTER &&
                              s->cu.part_mode != PART_2Nx2N && trafo_depth == 0);

    if (log2_trafo_size <= s->sps->log2_min_transform_block_size +
        s->sps->log2_diff_max_min_coding_block_size &&
        log2_trafo_size > s->sps->log2_min_transform_block_size &&
        trafo_depth < s->cu.max_trafo_depth &&
        !(s->cu.intra_split_flag && trafo_depth == 0)) {
        split_transform_flag =
        ff_hevc_split_transform_flag_decode(s, log2_trafo_size);
        av_dlog(s->avctx,
                "split_transform_flag: %d\n", split_transform_flag);
    } else {
        split_transform_flag =
        (log2_trafo_size >
         s->sps->log2_min_transform_block_size +
         s->sps->log2_diff_max_min_coding_block_size ||
         (s->cu.intra_split_flag && trafo_depth == 0) ||
         s->tt.inter_split_flag);
    }

    if (trafo_depth == 0 || log2_trafo_size > 2) {
        if (trafo_depth == 0 || SAMPLE_CBF(s->tt.cbf_cb[trafo_depth - 1], xBase, yBase)) {
            SAMPLE_CBF(s->tt.cbf_cb[trafo_depth], x0, y0) =
            ff_hevc_cbf_cb_cr_decode(s, trafo_depth);
            av_dlog(s->avctx,
                   "cbf_cb: %d\n", SAMPLE_CBF(s->tt.cbf_cb[trafo_depth], x0, y0));
        }
        if (trafo_depth == 0 || SAMPLE_CBF(s->tt.cbf_cr[trafo_depth - 1], xBase, yBase)) {
            SAMPLE_CBF(s->tt.cbf_cr[trafo_depth], x0, y0) =
            ff_hevc_cbf_cb_cr_decode(s, trafo_depth);
            av_dlog(s->avctx,
                   "cbf_cr: %d\n", SAMPLE_CBF(s->tt.cbf_cr[trafo_depth], x0, y0));
        }
    }

    if (split_transform_flag) {
        int x1 = x0 + (( 1 << log2_trafo_size ) >> 1);
        int y1 = y0 + (( 1 << log2_trafo_size ) >> 1);

        hls_transform_tree(s, x0, y0, x0, y0, log2_cb_size,
                           log2_trafo_size - 1, trafo_depth + 1, 0);
        hls_transform_tree(s, x1, y0, x0, y0, log2_cb_size,
                           log2_trafo_size - 1, trafo_depth + 1, 1);
        hls_transform_tree(s, x0, y1, x0, y0, log2_cb_size,
                           log2_trafo_size - 1, trafo_depth + 1, 2);
        hls_transform_tree(s, x1, y1, x0, y0, log2_cb_size,
                           log2_trafo_size - 1, trafo_depth + 1, 3);
    } else {
        if (s->cu.pred_mode == MODE_INTRA || trafo_depth != 0 ||
            SAMPLE_CBF(s->tt.cbf_cb[trafo_depth], x0, y0) ||
            SAMPLE_CBF(s->tt.cbf_cr[trafo_depth], x0, y0)) {
            s->tt.cbf_luma = ff_hevc_cbf_luma_decode(s, trafo_depth);
            av_dlog(s->avctx,
                    "cbf_luma: %d\n", s->tt.cbf_luma);
        }

        hls_transform_unit(s, x0, y0, xBase,
                           yBase, log2_trafo_size, trafo_depth, blk_idx);
    }

}

static void hls_pcm_sample(HEVCContext *s, int x0, int y0, int log2_cb_size)
{
    //TODO: non-4:2:0 support

    GetBitContext gb;
    int cb_size = 1 << log2_cb_size;
    int stride0 = s->frame.linesize[0];
    uint8_t *dst0 = &s->frame.data[0][y0 * stride0 + x0];
    int stride1 = s->frame.linesize[1];
    uint8_t *dst1 = &s->frame.data[1][(y0 >> s->sps->vshift[1]) * stride1 + (x0 >> s->sps->hshift[1])];
    int stride2 = s->frame.linesize[2];
    uint8_t *dst2 = &s->frame.data[2][(y0 >> s->sps->vshift[2]) * stride2 + (x0 >> s->sps->hshift[2])];

    int length = cb_size * cb_size * 3 / 2 * s->sps->pcm.bit_depth;
    uint8_t *pcm = skip_bytes(&s->cc, length / 8);

    init_get_bits(&gb, pcm, length);

    s->hevcdsp.put_pcm(dst0, stride0, cb_size, &gb, s->sps->pcm.bit_depth);
    s->hevcdsp.put_pcm(dst1, stride1, cb_size/2, &gb, s->sps->pcm.bit_depth);
    s->hevcdsp.put_pcm(dst2, stride2, cb_size/2, &gb, s->sps->pcm.bit_depth);
}

static void hls_mvd_coding(HEVCContext *s, int x0, int y0, int log2_cb_size)
{

    uint16_t abs_mvd_greater0_flag[2];
    uint16_t abs_mvd_greater1_flag[2] = { 0, 0 };
    uint16_t abs_mvd_minus2[2];
    uint8_t mvd_sign_flag[2];
    abs_mvd_greater0_flag[0] = ff_hevc_abs_mvd_greater0_flag_decode(s);
    av_dlog(s->avctx, "abs_mvd_greater0_flag[0]: %d\n",
            abs_mvd_greater0_flag[0]);
    abs_mvd_greater0_flag[1] = ff_hevc_abs_mvd_greater0_flag_decode(s);
    av_dlog(s->avctx, "abs_mvd_greater0_flag[1]: %d\n",
            abs_mvd_greater0_flag[1]);
    if (abs_mvd_greater0_flag[0])
        abs_mvd_greater1_flag[0] = ff_hevc_abs_mvd_greater1_flag_decode(s);

    if (abs_mvd_greater0_flag[1])
        abs_mvd_greater1_flag[1] = ff_hevc_abs_mvd_greater1_flag_decode(s);

    if (abs_mvd_greater0_flag[0]) {
        abs_mvd_minus2[0] = -1;
        if (abs_mvd_greater1_flag[0])
            abs_mvd_minus2[0] = ff_hevc_abs_mvd_minus2_decode(s);
        mvd_sign_flag[0] = ff_hevc_mvd_sign_flag_decode(s);
    }
    if (abs_mvd_greater0_flag[1]) {
        abs_mvd_minus2[1] = -1;
        if (abs_mvd_greater1_flag[1])
            abs_mvd_minus2[1] = ff_hevc_abs_mvd_minus2_decode(s);
        mvd_sign_flag[1] = ff_hevc_mvd_sign_flag_decode(s);
    }
    s->pu.mvd.x = abs_mvd_greater0_flag[0] * (abs_mvd_minus2[0] + 2) * (1 - (mvd_sign_flag[0] << 1));
    s->pu.mvd.y = abs_mvd_greater0_flag[1] * (abs_mvd_minus2[1] + 2) * (1 - (mvd_sign_flag[1] << 1));
    return;
}

/*
 * 6.4.1 Derivation process for z-scan order block availability
 */
static int z_scan_block_avail(HEVCContext *s, int xCurr, int yCurr, int xN, int yN)
{

#define MIN_TB_ADDR_ZS(x, y)                                            \
    s->pps->min_tb_addr_zs[(y) * s->sps->pic_width_in_min_tbs + (x)]
    int availableN = 0;
    int minBlockAddrCurr =  MIN_TB_ADDR_ZS((xCurr >> s->sps->log2_min_transform_block_size), (yCurr >> s->sps->log2_min_transform_block_size));

    int minBlockAddrN;

    if ((xN < 0) || (yN < 0) || (xN >= s->sps->pic_width_in_luma_samples) || (yN >= s->sps->pic_height_in_luma_samples)) {
        minBlockAddrN = -1;
    } else {
        minBlockAddrN = MIN_TB_ADDR_ZS((xN >> s->sps->log2_min_transform_block_size), (yN >> s->sps->log2_min_transform_block_size));
    }
    if (s->sh.slice_address != 0 || s->pps->tiles_enabled_flag != 0)
        av_log(s->avctx, AV_LOG_ERROR, "TODO : check for different slices and tiles \n");

    //TODO : check for different slices and tiles
    if ((minBlockAddrN < 0) || (minBlockAddrN > minBlockAddrCurr)) {
        availableN = 0;
    } else {
        availableN = 1;
    }
    return availableN;
}

/*
 * 6.4.2 Derivation process for prediction block availability
 */
static int check_prediction_block_available(HEVCContext *s, int log2_cb_size, int x0, int y0, int nPbW, int nPbH, int xA1, int yA1, int partIdx)
{
    int sameCb = 0;
    int availableN = 0;

    if ((s->cu.x < xA1) && (s->cu.y < yA1) && ((s->cu.x + (1 << log2_cb_size))> xA1) && ((s->cu.y + (1 << log2_cb_size))> yA1)) {
        sameCb = 1;
    } else {
        sameCb = 0;
    }

    if (sameCb == 0) {
        availableN = z_scan_block_avail(s, x0, y0, xA1, yA1);
    } else {
        if ((nPbW << 1 == (1 << log2_cb_size)) && ((nPbH << 1) == (1 << log2_cb_size)) && (partIdx ==1) && ((s->cu.x + nPbW) > xA1) && ((s->cu.y + nPbH) <= yA1)) {
            availableN = 0;
        } else {
            availableN = 1;
        }
    }
    return availableN;
}

//check if the two luma locations belong to the same mostion estimation region
static int isDiffMER(HEVCContext *s, int xN, int yN, int xP, int yP)
{
    uint8_t plevel = s->pps->log2_parallel_merge_level;
    if (((xN >> plevel) == (xP >> plevel)) && ((yN >> plevel) == (yP >> plevel))) {
        return 1;
    }
    return 0;
}

// check if the mv's and refidx are the same between A and B
static int compareMVrefidx(struct MvField A, struct MvField B)
{
    if((A.ref_idx == B.ref_idx) && (A.mv.x == B.mv.x) && (A.mv.y == B.mv.y))
        return 1;
    else
        return 0;
}

/*
 * 8.5.3.1.2  Derivation process for spatial merging candidates
 */
static void derive_spatial_merge_candidates(HEVCContext *s, int x0, int y0, int nPbW, int nPbH, int log2_cb_size, int singleMCLFlag, int part_idx,  struct MvField mergecandlist[])
{
    
    int available_a1_flag=0;
    int available_b1_flag=0;
    int available_b0_flag=0;
    int available_a0_flag=0;
    int available_b2_flag=0;
    struct MvField spatialCMVS[MRG_MAX_NUM_CANDS];
    
    //first left spatial merge candidate
    int xA1 = x0-1;
    int yA1 = y0 + nPbH - 1;
    int isAvailableA1 =0;
    int check_A1 = check_prediction_block_available (s,log2_cb_size, x0, y0, nPbW, nPbH, xA1, yA1, part_idx);
    int pic_width_in_min_pu  = s->sps->pic_width_in_min_cbs * 4;
    
    int check_B1;
    int xB1, yB1;
    int is_available_b1;
    int xB1_pu;
    int yB1_pu;
    
    int check_B0;
    int xB0, yB0;
    int isAvailableB0;
    int xB0_pu;
    int yB0_pu;
    
    int check_A0;
    int xA0, yA0;
    int isAvailableA0;
    int xA0_pu;
    int yA0_pu;
    
    int check_B2;
    int xB2, yB2;
    int isAvailableB2;
    int xB2_pu;
    int yB2_pu;
    
    int  availableFlagCol = 0;
    struct MvField temporal_cand;
    int mergearray_index = 0;
    
    struct MvField zerovector;
    int numRefIdx;
    int zeroIdx = 0;
    
    int numMergeCand, numOrigMergeCand, numInputMergeCand, sumcandidates;
    
    int xA1_pu = xA1 >> s->sps->log2_min_pu_size;
    int yA1_pu = yA1 >> s->sps->log2_min_pu_size;
    
    // struct MvField A1 = s->pu.tab_mvf[(x_pu-1)*pic_width_in_min_pu + y_pu]
    if((xA1_pu >= 0) && !(s->pu.tab_mvf[(xA1_pu) * pic_width_in_min_pu + yA1_pu].is_intra) && check_A1) {
        isAvailableA1 = 1;
    } else {
        isAvailableA1 = 0;
    }
    
    if((singleMCLFlag == 0) &&  (part_idx == 1) &&
       ((s->cu.part_mode == PART_Nx2N) || (s->cu.part_mode == PART_nLx2N) || (s->cu.part_mode == PART_nRx2N))
       || isDiffMER(s, xA1, yA1, x0, y0) ) {
        isAvailableA1 = 0;
    }
    
    if (isAvailableA1) {
        available_a1_flag = 1;
        spatialCMVS[0] = s->pu.tab_mvf[(xA1_pu) * pic_width_in_min_pu + yA1_pu];
    } else {
        available_a1_flag = 0;
        spatialCMVS[0].ref_idx = -1;
        spatialCMVS[0].mv.x = 0;
        spatialCMVS[0].mv.y = 0;
        spatialCMVS[0].pred_flag = 0;
        spatialCMVS[0].is_intra = 0;
    }
    
    // above spatial merge candidate
    
    xB1 = x0 + nPbW - 1;
    yB1 = y0 - 1;
    xB1_pu = xB1 >> s->sps->log2_min_pu_size;
    yB1_pu = yB1 >> s->sps->log2_min_pu_size;
    
    is_available_b1 = 0;
    check_B1 = check_prediction_block_available(s, log2_cb_size, x0, y0, nPbW, nPbH, xB1, yB1, part_idx);
    
    if((yB1_pu >= 0) && !(s->pu.tab_mvf[(xB1_pu) * pic_width_in_min_pu + yB1_pu].is_intra) && check_B1) {
        is_available_b1 = 1;
    } else {
        is_available_b1 = 0;
    }
    
    if((singleMCLFlag == 0) && (part_idx == 1) &&
       ((s->cu.part_mode == PART_2NxN) || (s->cu.part_mode == PART_2NxnU) || (s->cu.part_mode == PART_2NxnD))
       || isDiffMER(s, xB1, yB1, x0, y0)) {
        is_available_b1 = 0;
    }
    
    if (is_available_b1 && !(compareMVrefidx(s->pu.tab_mvf[(xB1_pu) * pic_width_in_min_pu + yB1_pu], s->pu.tab_mvf[(xA1_pu) * pic_width_in_min_pu + yA1_pu] ))) {
        available_b1_flag = 1;
        spatialCMVS[1] = s->pu.tab_mvf[(xB1_pu) * pic_width_in_min_pu + yB1_pu];
    } else {
        available_b1_flag = 0;
        spatialCMVS[1].ref_idx = -1;
        spatialCMVS[1].mv.x = 0;
        spatialCMVS[1].mv.y = 0;
        spatialCMVS[1].pred_flag = 0;
        spatialCMVS[1].is_intra = 0;
    }
    
    // above right spatial merge candidate
    xB0 = x0 + nPbW;
    yB0 = y0 - 1;
    xB0_pu = xB0 >> s->sps->log2_min_pu_size;
    yB0_pu = yB0 >> s->sps->log2_min_pu_size;
    isAvailableB0 = 0;
    check_B0 = check_prediction_block_available(s, log2_cb_size, x0, y0, nPbW, nPbH, xB0, yB0, part_idx);
    if((yB0_pu >= 0) && !(s->pu.tab_mvf[(xB0_pu) * pic_width_in_min_pu + yB0_pu].is_intra) && check_B0) {
        isAvailableB0 = 1;
    } else {
        isAvailableB0 = 0;
    }
    
    if((isDiffMER(s, xB0, yB0, x0, y0))) {
        isAvailableB0 = 0;
    }
    
    if (isAvailableB0 && !(compareMVrefidx(s->pu.tab_mvf[(xB0_pu) * pic_width_in_min_pu + yB0_pu], s->pu.tab_mvf[(xB1_pu) * pic_width_in_min_pu + yB1_pu]))) {
        available_b0_flag = 1;
        spatialCMVS[2] = s->pu.tab_mvf[(xB0_pu) * pic_width_in_min_pu + yB0_pu];
    } else {
        available_b0_flag = 0;
        spatialCMVS[2].ref_idx = -1;
        spatialCMVS[2].mv.x = 0;
        spatialCMVS[2].mv.y = 0;
        spatialCMVS[2].pred_flag = 0;
        spatialCMVS[2].is_intra = 0;
    }
    
    // left bottom spatial merge candidate
    xA0 = x0 - 1;
    yA0 = y0 + nPbH;
    xA0_pu = xA0 >> s->sps->log2_min_pu_size;
    yA0_pu = yA0 >> s->sps->log2_min_pu_size;
    isAvailableA0 = 0;
    check_A0 = check_prediction_block_available(s, log2_cb_size, x0, y0, nPbW, nPbH, xA0, yA0, part_idx);
    
    if((xA0_pu >= 0) && !(s->pu.tab_mvf[(xA0_pu) * pic_width_in_min_pu + yA0_pu].is_intra) && check_A0) {
        isAvailableA0 = 1;
    } else {
        isAvailableA0 = 0;
    }
    
    if((isDiffMER(s, xA0, yA0, x0, y0))) {
        isAvailableA0 = 0;
    }
    
    if (isAvailableA0 && !(compareMVrefidx(s->pu.tab_mvf[(xA0_pu) * pic_width_in_min_pu + yA0_pu], s->pu.tab_mvf[(xA1_pu) * pic_width_in_min_pu + yA1_pu]))) {
        available_a0_flag = 1;
        spatialCMVS[3] = s->pu.tab_mvf[(xA0_pu) * pic_width_in_min_pu + yA0_pu];
    } else {
        available_a0_flag = 0;
        spatialCMVS[3].ref_idx = -1;
        spatialCMVS[3].mv.x = 0;
        spatialCMVS[3].mv.y = 0;
        spatialCMVS[3].pred_flag = 0;
        spatialCMVS[3].is_intra = 0;
    }
    
    // above left spatial merge candidate
    xB2 = x0 - 1;
    yB2 = y0 - 1;
    xB2_pu = xB2 >> s->sps->log2_min_pu_size;
    yB2_pu = yB2 >> s->sps->log2_min_pu_size;
    isAvailableB2 = 0;
    check_B2 = check_prediction_block_available(s, log2_cb_size, x0, y0, nPbW, nPbH, xB2, yB2, part_idx);
    
    if((xB2_pu >= 0) && (yB2_pu >= 0) && !(s->pu.tab_mvf[(xB2_pu) * pic_width_in_min_pu + yB2_pu].is_intra) && check_B2) {
        isAvailableB2 = 1;
    } else {
        isAvailableB2 = 0;
    }
    
    if((isDiffMER(s, xB2, yB2, x0, y0))) {
        isAvailableB2 = 0;
    }
    
    sumcandidates = available_a1_flag + available_b1_flag + available_b0_flag + available_a0_flag;
    
    if (isAvailableB2 && !(compareMVrefidx(s->pu.tab_mvf[(xB2_pu) * pic_width_in_min_pu + yB2_pu], s->pu.tab_mvf[(xA1_pu) * pic_width_in_min_pu + yA1_pu])) &&
        !(compareMVrefidx(s->pu.tab_mvf[(xB2_pu) * pic_width_in_min_pu + yB2_pu], s->pu.tab_mvf[(xB1_pu) * pic_width_in_min_pu + yB1_pu])) && sumcandidates != 4) {
        available_b2_flag =1;
        spatialCMVS[4] = s->pu.tab_mvf[(xB2_pu) * pic_width_in_min_pu + yB2_pu];
    } else {
        available_b2_flag = 0;
        spatialCMVS[4].ref_idx = -1;
        spatialCMVS[4].mv.x = 0;
        spatialCMVS[4].mv.y = 0;
        spatialCMVS[4].pred_flag = 0;
        spatialCMVS[4].is_intra = 0;
    }
    
    //TODO : temporal motion vector candidate
    if (available_a1_flag) {
        mergecandlist[mergearray_index] = spatialCMVS[0];
        mergearray_index++;
    }
    if (available_b1_flag) {
        mergecandlist[mergearray_index] = spatialCMVS[1];
        mergearray_index++;
    }
    if (available_b0_flag) {
        mergecandlist[mergearray_index] = spatialCMVS[2];
        mergearray_index++;
    }
    if (available_a0_flag) {
        mergecandlist[mergearray_index] = spatialCMVS[3];
        mergearray_index++;
    }
    if (available_b2_flag) {
        mergecandlist[mergearray_index] = spatialCMVS[4];
        mergearray_index++;
    }
    if (availableFlagCol) {
        mergecandlist[mergearray_index] = temporal_cand;
        mergearray_index++;
    }
    numMergeCand = mergearray_index;
    numOrigMergeCand = mergearray_index;
    
    // TODO: derive combined bi-predictive merge candidates  (applies for B slices)
    
    
    
    /*
     * append Zero motion vector candidates
     */
    if(s->sh.slice_type == P_SLICE) {
        numRefIdx = s->sh.num_ref_idx_l0_active;
    } else if(s->sh.slice_type == B_SLICE) {
        numRefIdx = s->sh.num_ref_idx_l0_active > s->sh.num_ref_idx_l1_active ? s->sh.num_ref_idx_l1_active : s->sh.num_ref_idx_l0_active;
    }
    numInputMergeCand = numMergeCand;
    while(numMergeCand != MRG_MAX_NUM_CANDS) {
        if(s->sh.slice_type == P_SLICE) {
            zerovector.ref_idx = (zeroIdx < numRefIdx) ? zeroIdx : 0;
            zerovector.pred_flag = 0;
            zerovector.mv.x = 0;
            zerovector.mv.y = 0;
            zerovector.is_intra = 0;
        } else if(s->sh.slice_type == B_SLICE) {
            zerovector.ref_idx = (zeroIdx < numRefIdx) ? zeroIdx : 0;
            zerovector.pred_flag =1;
            zerovector.mv.x = 0;
            zerovector.mv.y = 0;
            zerovector.is_intra = 0;
        }
        mergecandlist[numMergeCand] = zerovector;
        numMergeCand++;
        zeroIdx++;
    }
}

/*
 * 8.5.3.1.1 Derivation process of luma Mvs for merge mode
 */
static void luma_mv_merge_mode(HEVCContext *s, int x0, int y0, int nPbW, int nPbH, int log2_cb_size, int part_idx, int merge_idx, MvField *mv)
{
    int singleMCLFlag = 0;
    int nCS = 1 << log2_cb_size;
    struct MvField mergecand_list[MRG_MAX_NUM_CANDS];
    
    if ((s->pps->log2_parallel_merge_level -2 > 0) && (nCS == 8)) {
        singleMCLFlag = 1;
    }
    
    if (singleMCLFlag == 1) {
        x0 = s->cu.x;
        y0 = s->cu.y;
        nPbW = nCS;
        nPbH = nCS;
    }
    derive_spatial_merge_candidates(s, x0, y0, nPbW, nPbH, log2_cb_size, singleMCLFlag, part_idx, mergecand_list);
    mv->mv.x = mergecand_list[merge_idx].mv.x;
    mv->mv.y = mergecand_list[merge_idx].mv.y;
    
}

static void luma_mv_mvp_mode(HEVCContext *s, int x0, int y0, int nPbW, int nPbH, int log2_cb_size, int part_idx, int merge_idx, MvField *mv , int mvp_lx_flag)
{
    int isScaledFlag_L0 =0;
    int availableFlagLXA0 = 0;
    int availableFlagLXB0 = 0;
    int availableFlagLXCol = 0;
    int numMVPCandLX  =0;
    int pic_width_in_min_pu  = s->sps->pic_width_in_min_cbs * 4;
    int xA0, yA0;
    int xA0_pu, yA0_pu;
    int isAvailableA0;
    
    int xA1, yA1;
    int xA1_pu, yA1_pu;
    int isAvailableA1;
    
    int xB0, yB0;
    int xB0_pu, yB0_pu;
    int isAvailableB0;
    
    int xB1, yB1;
    int xB1_pu, yB1_pu;
    int is_available_b1;
    
    int xB2, yB2;
    int xB2_pu, yB2_pu;
    int isAvailableB2;
    struct MvField mvpcand_list[2];
    int check_A0, check_A1, check_B0, check_B1, check_B2;
    MvField mxA;
    MvField mxB;
    
    // left bottom spatial candidate
    xA0 = x0 - 1;
    yA0 = y0 + nPbH;
    xA0_pu = xA0 >> s->sps->log2_min_pu_size;
    yA0_pu = yA0 >> s->sps->log2_min_pu_size;
    isAvailableA0 = 0;
    check_A0 = check_prediction_block_available(s, log2_cb_size, x0, y0, nPbW, nPbH, xA0, yA0, part_idx);
    
    if((xA0_pu >= 0) && !(s->pu.tab_mvf[(xA0_pu) * pic_width_in_min_pu + yA0_pu].is_intra) && check_A0) {
        isAvailableA0 = 1;
    } else {
        isAvailableA0 = 0;
    }
    
    
    //left spatial merge candidate
    xA1 = x0-1;
    yA1 = y0 + nPbH - 1;
    xA1_pu = xA1 >> s->sps->log2_min_pu_size;
    yA1_pu = yA1 >> s->sps->log2_min_pu_size;
    isAvailableA1 =0;
    check_A1 = check_prediction_block_available (s,log2_cb_size, x0, y0, nPbW, nPbH, xA1, yA1, part_idx);
    if((xA1_pu >= 0) && !(s->pu.tab_mvf[(xA1_pu) * pic_width_in_min_pu + yA1_pu].is_intra) && check_A1) {
        isAvailableA1 = 1;
    } else {
        isAvailableA1 = 0;
    }
    
    if((isAvailableA0) || (isAvailableA1)) {
        isScaledFlag_L0 =1;
    }
    // XA0
    if((isAvailableA0) && !(s->pu.tab_mvf[(xA0_pu) * pic_width_in_min_pu + yA0_pu].is_intra) && (availableFlagLXA0 == 0)) {
        if((s->pu.tab_mvf[(xA0_pu) * pic_width_in_min_pu + yA0_pu].pred_flag == 1) && 1 ) { // 1 is used in the if because For first P frame
            // DiffPicOrderCnt( RefPicListX[ RefIdxLX[ xAk ][ yAk ] ], RefPicListX[ refIdxLX ]// iS considered to be equal to 0)
            availableFlagLXA0 =1;
            mxA = s->pu.tab_mvf[(xA0_pu) * pic_width_in_min_pu + yA0_pu];
            // mvpcand_list[numMVPCandLX] = s->pu.tab_mvf[(xA0_pu) * pic_width_in_min_pu + yA0_pu];
        }
    }
    //XA1
    if((isAvailableA1) && !(s->pu.tab_mvf[(xA1_pu) * pic_width_in_min_pu + yA1_pu].is_intra) && (availableFlagLXA0==0)) {
        if((s->pu.tab_mvf[(xA1_pu) * pic_width_in_min_pu + yA1_pu].pred_flag == 1) && 1 ) { // 1 is used in the if because For first P frame
            // DiffPicOrderCnt( RefPicListX[ RefIdxLX[ xAk ][ yAk ] ], RefPicListX[ refIdxLX ]// iS considered to be equal to 0)
            availableFlagLXA0 =1;
            mxA = s->pu.tab_mvf[(xA1_pu) * pic_width_in_min_pu + yA1_pu];
            // mvpcand_list[numMVPCandLX] = s->pu.tab_mvf[(xA1_pu) * pic_width_in_min_pu + yA1_pu];
            
        }
    }
    // TODO Step 7 in 8.5.3.1.6.
    
    // B candidates
    // above right spatial merge candidate
    xB0 = x0 + nPbW;
    yB0 = y0 - 1;
    xB0_pu = xB0 >> s->sps->log2_min_pu_size;
    yB0_pu = yB0 >> s->sps->log2_min_pu_size;
    isAvailableB0 = 0;
    check_B0 = check_prediction_block_available(s, log2_cb_size, x0, y0, nPbW, nPbH, xB0, yB0, part_idx);
    
    if((yB0_pu >= 0) && !(s->pu.tab_mvf[(xB0_pu) * pic_width_in_min_pu + yB0_pu].is_intra) && check_B0) {
        isAvailableB0 = 1;
    } else {
        isAvailableB0 = 0;
    }
    
    if((isAvailableB0 == 1) && (availableFlagLXB0 == 0)) {
        if((s->pu.tab_mvf[(xB0_pu) * pic_width_in_min_pu + yB0_pu].pred_flag == 1) && 1 ) { // 1 is used in the if because For first P frame
            // DiffPicOrderCnt( RefPicListX[ RefIdxLX[ xAk ][ yAk ] ], RefPicListX[ refIdxLX ]// iS considered to be equal to 0)
            availableFlagLXB0 =1;
            mxB = s->pu.tab_mvf[(xB0_pu) * pic_width_in_min_pu + yB0_pu];
            // mvpcand_list[numMVPCandLX] = s->pu.tab_mvf[(xB0_pu) * pic_width_in_min_pu + yB0_pu];
        }
    }
    
    if(!availableFlagLXB0) {
        // above spatial merge candidate
        xB1 = x0 + nPbW - 1;
        yB1 = y0 - 1;
        xB1_pu = xB1 >> s->sps->log2_min_pu_size;
        yB1_pu = yB1 >> s->sps->log2_min_pu_size;
        is_available_b1 = 0;
        check_B1 = check_prediction_block_available(s, log2_cb_size, x0, y0, nPbW, nPbH, xB1, yB1, part_idx);
        
        if((yB1_pu >= 0) && !(s->pu.tab_mvf[(xB1_pu) * pic_width_in_min_pu + yB1_pu].is_intra) && check_B1) {
            is_available_b1 = 1;
        } else {
            is_available_b1 = 0;
        }
        
        if((is_available_b1 == 1) && (availableFlagLXB0 == 0)) {
            if((s->pu.tab_mvf[(xB1_pu) * pic_width_in_min_pu + yB1_pu].pred_flag == 1) && 1 ) { // 1 is used in the if because For first P frame
                // DiffPicOrderCnt( RefPicListX[ RefIdxLX[ xAk ][ yAk ] ], RefPicListX[ refIdxLX ]// iS considered to be equal to 0)
                availableFlagLXB0 =1;
                // mvpcand_list[numMVPCandLX] = s->pu.tab_mvf[(xB1_pu) * pic_width_in_min_pu + yB1_pu];
                mxB = s->pu.tab_mvf[(xB1_pu) * pic_width_in_min_pu + yB1_pu];
                
            }
        }
    }
    
    if(!availableFlagLXB0) {
        // above left spatial merge candidate
        xB2 = x0 - 1;
        yB2 = y0 - 1;
        xB2_pu = xB2 >> s->sps->log2_min_pu_size;
        yB2_pu = yB2 >> s->sps->log2_min_pu_size;
        isAvailableB2 = 0;
        check_B2 = check_prediction_block_available(s, log2_cb_size, x0, y0, nPbW, nPbH, xB2, yB2, part_idx);
        
        if((xB2_pu >= 0) && (yB2_pu >= 0) && !(s->pu.tab_mvf[(xB2_pu) * pic_width_in_min_pu + yB2_pu].is_intra) && check_B2) {
            isAvailableB2 = 1;
        } else {
            isAvailableB2 = 0;
        }
        
        if((isAvailableB2 == 1) && (availableFlagLXB0 == 0)) {
            if((s->pu.tab_mvf[(xB2_pu) * pic_width_in_min_pu + yB2_pu].pred_flag == 1) && 1 ) { // 1 is used in the if because For first P frame
                // DiffPicOrderCnt( RefPicListX[ RefIdxLX[ xAk ][ yAk ] ], RefPicListX[ refIdxLX ]// iS considered to be equal to 0)
                availableFlagLXB0 =1;
                // mvpcand_list[numMVPCandLX] = s->pu.tab_mvf[(xB2_pu) * pic_width_in_min_pu + yB2_pu];
                mxB = s->pu.tab_mvf[(xB2_pu) * pic_width_in_min_pu + yB2_pu];
                
            }
        }
    }
    
    if(isScaledFlag_L0 == 0 && availableFlagLXB0) {
        availableFlagLXA0 =1;
        mxA = mxB;
    }
    if(isScaledFlag_L0 == 0) {
        availableFlagLXB0 =0;
    }
    if(availableFlagLXA0) {
        mvpcand_list[numMVPCandLX] = mxA;
        numMVPCandLX++;
    }
    if(availableFlagLXB0) {
        mvpcand_list[numMVPCandLX] = mxB;
        numMVPCandLX++;
    }
    
    
    
    // TODO Step 5 for B candidates  in 8.5.3.1.6.
    if (availableFlagLXA0 && availableFlagLXB0 && ((mvpcand_list[0].mv.x != mvpcand_list[1].mv.x) || (mvpcand_list[0].mv.y != mvpcand_list[1].mv.y))) {
        availableFlagLXCol = 0 ;
    } else {
        //TODO section 8.5.3.1.7 temporal motion vector prediction
    }
    if ((mvpcand_list[0].mv.x == mvpcand_list[1].mv.x) && (mvpcand_list[0].mv.y == mvpcand_list[1].mv.y)) {
        numMVPCandLX--;
    }
    while (numMVPCandLX < 2) { // insert zero motion vectors when the number of available candidates are less than 2
        mvpcand_list[numMVPCandLX].mv.x =0;
        mvpcand_list[numMVPCandLX].mv.y =0;
        numMVPCandLX++;
    }
    mv->mv.x  = mvpcand_list[mvp_lx_flag].mv.x;
    mv->mv.y  = mvpcand_list[mvp_lx_flag].mv.y;
    
}

/**
 * 8.5.3.2.2.1 Luma sample interpolation process
 *
 * @param s HEVC decoding context
 * @param dst target buffer for block data at block position
 * @param dststride stride of the dst buffer
 * @param mv motion vector (relative to block position) to get pixel data from
 * @param x_off horizontal position of block from origin (0, 0)
 * @param y_off vertical position of block from origin (0, 0)
 * @param block_w width of block
 * @param block_h height of block
 */
static void luma_mc(HEVCContext *s, uint8_t *dst, ptrdiff_t dststride,
                    const Mv *mv, int x_off, int y_off, int block_w, int block_h)
{
    uint8_t *src = s->frame.data[0];
    ptrdiff_t srcstride = s->frame.linesize[0];
    int pic_width = s->sps->pic_width_in_luma_samples;
    int pic_height = s->sps->pic_height_in_luma_samples;

    int mx = mv->x & 3;
    int my = mv->y & 3;
    int extra_left = qpel_extra_before[mx];
    int extra_top = qpel_extra_before[my];

    x_off += mv->x >> 2;
    y_off += mv->y >> 2;
    src += y_off * srcstride + (x_off << s->sps->pixel_shift);

    if (x_off < extra_left || x_off >= pic_width - block_w - qpel_extra_after[mx] ||
        y_off < extra_top || y_off >= pic_height - block_h - qpel_extra_after[my]) {
        int offset = extra_top * srcstride + (extra_left << s->sps->pixel_shift);
        s->dsp.emulated_edge_mc(s->edge_emu_buffer, src - offset, srcstride,
                                block_w + qpel_extra[mx], block_h + qpel_extra[my],
                                x_off - extra_left, y_off - extra_top,
                                pic_width, pic_height);
        src = s->edge_emu_buffer + offset;
    }
    s->hevcdsp.put_hevc_qpel[my][mx](dst, dststride, src, srcstride, block_w, block_h);
}

/**
 * 8.5.3.2.2.2 Chroma sample interpolation process
 *
 * @param s HEVC decoding context
 * @param dst1 target buffer for block data at block position (U plane)
 * @param dst2 target buffer for block data at block position (V plane)
 * @param dststride stride of the dst1 and dst2 buffers
 * @param mv motion vector (relative to block position) to get pixel data from
 * @param x_off horizontal position of block from origin (0, 0)
 * @param y_off vertical position of block from origin (0, 0)
 * @param block_w width of block
 * @param block_h height of block
 */
static void chroma_mc(HEVCContext *s, uint8_t *dst1, uint8_t *dst2, ptrdiff_t dststride,
                      const Mv *mv, int x_off, int y_off, int block_w, int block_h)
{
    uint8_t *src1 = s->frame.data[1];
    uint8_t *src2 = s->frame.data[2];
    ptrdiff_t srcstride = s->frame.linesize[1];
    int pic_width = s->sps->pic_width_in_luma_samples >> 1;
    int pic_height = s->sps->pic_height_in_luma_samples >> 1;

    int mx = mv->x & 7;
    int my = mv->y & 7;

    x_off += mv->x >> 3;
    y_off += mv->y >> 3;
    src1 += y_off * srcstride + (x_off << s->sps->pixel_shift);
    src2 += y_off * srcstride + (x_off << s->sps->pixel_shift);

    if (x_off < epel_extra_before || x_off >= pic_width - block_w - epel_extra_after ||
        y_off < epel_extra_after || y_off >= pic_height - block_h - epel_extra_after) {
        int offset = epel_extra_before * (srcstride + (1 << s->sps->pixel_shift));
        s->dsp.emulated_edge_mc(s->edge_emu_buffer, src1 - offset, srcstride,
                                block_w + epel_extra, block_h + epel_extra,
                                x_off - epel_extra_before, y_off - epel_extra_before,
                                pic_width, pic_height);
        src1 = s->edge_emu_buffer + offset;
        s->hevcdsp.put_hevc_epel[!!my][!!mx](dst1, dststride, src1, srcstride, block_w, block_h, mx, my);

        s->dsp.emulated_edge_mc(s->edge_emu_buffer, src2 - offset, srcstride,
                                block_w + epel_extra, block_h + epel_extra,
                                x_off - epel_extra_before, y_off - epel_extra_before,
                                pic_width, pic_height);
        src2 = s->edge_emu_buffer + offset;
        s->hevcdsp.put_hevc_epel[!!my][!!mx](dst2, dststride, src2, srcstride, block_w, block_h, mx, my);
    } else {
        s->hevcdsp.put_hevc_epel[!!my][!!mx](dst1, dststride, src1, srcstride, block_w, block_h, mx, my);
        s->hevcdsp.put_hevc_epel[!!my][!!mx](dst2, dststride, src2, srcstride, block_w, block_h, mx, my);
    }
}

static void hls_prediction_unit(HEVCContext *s, int x0, int y0, int nPbW, int nPbH, int log2_cb_size, int partIdx)
{
    int merge_idx;
    enum InterPredIdc inter_pred_idc = PRED_L0;
    int ref_idx_l0;
    int ref_idx_l1;
    int mvp_l0_flag;
    int mvp_l1_flag;
    struct MvField current_mv = {{0, 0}};
    int i, j;
    int x_pu, y_pu;
    int pic_width_in_min_pu = s->sps->pic_width_in_min_cbs * 4;

    current_mv.pred_flag=1;

    if (SAMPLE(s->cu.skip_flag, x0, y0)) {
        if (s->sh.max_num_merge_cand > 1) {
            merge_idx = ff_hevc_merge_idx_decode(s);
            av_dlog(s->avctx,
                    "merge_idx: %d\n", merge_idx);
            luma_mv_merge_mode(s, x0, y0, 1 << log2_cb_size, 1 << log2_cb_size, log2_cb_size, partIdx, merge_idx, &current_mv);
            x_pu = x0 >> s->sps->log2_min_pu_size;
            y_pu = y0 >> s->sps->log2_min_pu_size;
            for(i = 0; i < nPbW >> s->sps->log2_min_pu_size; i++) {
                for(j = 0; j < nPbH >> s->sps->log2_min_pu_size; j++) {
                    s->pu.tab_mvf[(x_pu + i) * pic_width_in_min_pu + y_pu + j] = current_mv;
                }
            }
        }
    } else {/* MODE_INTER */
        s->pu.merge_flag = ff_hevc_merge_flag_decode(s);
        av_dlog(s->avctx,
                "merge_flag: %d\n", s->pu.merge_flag);
        if (s->pu.merge_flag) {
            if (s->sh.max_num_merge_cand > 1) {
                merge_idx = ff_hevc_merge_idx_decode(s);
                av_dlog(s->avctx,
                        "merge_idx: %d\n", merge_idx);
                luma_mv_merge_mode(s, x0, y0, nPbW, nPbH, log2_cb_size, partIdx, merge_idx, &current_mv);
                x_pu = x0 >> s->sps->log2_min_pu_size;
                y_pu = y0 >> s->sps->log2_min_pu_size;
                for(i = 0; i < nPbW >> s->sps->log2_min_pu_size; i++) {
                    for(j = 0; j < nPbH >> s->sps->log2_min_pu_size; j++) {
                        s->pu.tab_mvf[(x_pu + i) * pic_width_in_min_pu + y_pu + j] = current_mv;                    }
                }
            }
        } else {
            if (s->sh.slice_type == B_SLICE)
                inter_pred_idc = ff_hevc_inter_pred_idc_decode(s, nPbW, nPbH);
            if (inter_pred_idc != PRED_L1) {
                if (s->sh.num_ref_idx_l0_active > 1) {
                    ref_idx_l0 = ff_hevc_ref_idx_lx_decode(s, s->sh.num_ref_idx_l0_active);
                    av_dlog(s->avctx, "ref_idx_l0: %d\n",
                            ref_idx_l0);
                }
                hls_mvd_coding(s, x0, y0, 0 );
                mvp_l0_flag = ff_hevc_mvp_lx_flag_decode(s);
                luma_mv_mvp_mode(s, x0, y0, nPbW, nPbH, log2_cb_size, partIdx, merge_idx, &current_mv, mvp_l0_flag);
            }
            if (inter_pred_idc != PRED_L0) {
                if (s->sh.num_ref_idx_l1_active > 1) {
                    ref_idx_l1 = ff_hevc_ref_idx_lx_decode(s, s->sh.num_ref_idx_l1_active);
                    av_dlog(s->avctx, "ref_idx_l0: %d\n",
                            ref_idx_l0);
                }
                if (s->sh.mvd_l1_zero_flag == 1 && inter_pred_idc == PRED_BI) {
                    //mvd_l1[ x0 ][ y0 ][ 0 ] = 0
                    //mvd_l1[ x0 ][ y0 ][ 1 ] = 0
                } else {
                    hls_mvd_coding(s, x0, y0, 1 );
                }
                mvp_l1_flag = ff_hevc_mvp_lx_flag_decode(s);
                luma_mv_mvp_mode(s, x0, y0, nPbW, nPbH, log2_cb_size, partIdx, merge_idx, &current_mv, mvp_l1_flag);
            }
            current_mv.mv.x += s->pu.mvd.x;
            current_mv.mv.y += s->pu.mvd.y;
            x_pu = x0 >> s->sps->log2_min_pu_size;
            y_pu = y0 >> s->sps->log2_min_pu_size;
            for(i = 0; i < nPbW >> s->sps->log2_min_pu_size; i++) {
                for(j = 0; j < nPbH >> s->sps->log2_min_pu_size; j++) {
                    s->pu.tab_mvf[(x_pu + i) * pic_width_in_min_pu + y_pu + j] = current_mv;
                }
            }

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

    int x_pu = x0 >> s->sps->log2_min_pu_size;
    int y_pu = y0 >> s->sps->log2_min_pu_size;
    int size_in_pus = pu_size >> s->sps->log2_min_pu_size;
    int pic_width_in_min_pu = s->sps->pic_width_in_min_cbs * 4;

    int cand_up   = y_pu > 0 ? s->pu.top_ipm[x_pu] : INTRA_DC ;
    int cand_left = x_pu > 0 ? s->pu.left_ipm[y_pu] : INTRA_DC ;

    int y_ctb = (y0 >> (s->sps->log2_ctb_size)) << (s->sps->log2_ctb_size);

    // intra_pred_mode prediction does not cross vertical CTB boundaries
    if ((y0 - 1) < y_ctb)
        cand_up = INTRA_DC;

    av_dlog(s->avctx, "cand_left: %d, cand_up: %d\n",
           cand_left, cand_up);

    if (cand_left == cand_up) {
        if (cand_left < 2) {
            candidate[0] = INTRA_PLANAR;
            candidate[1] = INTRA_DC;
            candidate[2] = INTRA_ANGULAR_26;
        } else {
            candidate[0] = cand_left;
            candidate[1] = 2 + ((cand_left - 2 - 1 + 32) % 32);
            candidate[2] = 2 + ((cand_left - 2 + 1) % 32);
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
        intra_pred_mode = candidate[s->pu.mpm_idx];
    } else {
        if (candidate[0] > candidate[1])
            FFSWAP(uint8_t, candidate[0], candidate[1]);
        if (candidate[0] > candidate[2])
            FFSWAP(uint8_t, candidate[0], candidate[2]);
        if (candidate[1] > candidate[2])
            FFSWAP(uint8_t, candidate[1], candidate[2]);

        intra_pred_mode = s->pu.rem_intra_luma_pred_mode;
        for (i = 0; i < 3; i++) {
            av_dlog(s->avctx, "candidate[%d] = %d\n",
                   i, candidate[i]);
            if (intra_pred_mode >= candidate[i])
                intra_pred_mode++;
        }
    }

    memset(&s->pu.top_ipm[x_pu], intra_pred_mode, size_in_pus);
    memset(&s->pu.left_ipm[y_pu], intra_pred_mode, size_in_pus);
    /* write the intra prediction units into the mv array */
    for(i = 0; i <size_in_pus; i++) {
            for(j = 0; j <size_in_pus; j++) {
                s->pu.tab_mvf[(x_pu+i)*pic_width_in_min_pu + y_pu+j].is_intra = 1;
            }
        }

    av_dlog(s->avctx, "intra_pred_mode: %d\n",
           intra_pred_mode);
    return intra_pred_mode;
}

static av_always_inline void set_ct_depth(HEVCContext *s, int x0, int y0,
                                          int log2_cb_size, int ct_depth)
{
    int length = (1 << log2_cb_size) >> s->sps->log2_min_coding_block_size;
    int x_cb = x0 >> s->sps->log2_min_coding_block_size;
    int y_cb = y0 >> s->sps->log2_min_coding_block_size;

    memset(&s->cu.top_ct_depth[x_cb], ct_depth, length);
    memset(&s->cu.left_ct_depth[y_cb], ct_depth, length);
}

static void intra_prediction_unit(HEVCContext *s, int x0, int y0, int log2_cb_size)
{
    int i, j;
    uint8_t prev_intra_luma_pred_flag[4];
    int chroma_mode;
    static const uint8_t intra_chroma_table[4] = {0, 26, 10, 1};

    int split = s->cu.part_mode == PART_NxN;
    int pb_size = (1 << log2_cb_size) >> split;
    int side = split + 1;

    for (i = 0; i < side; i++)
        for (j = 0; j < side; j++) {
            prev_intra_luma_pred_flag[2*i+j] = ff_hevc_prev_intra_luma_pred_flag_decode(s);
            av_dlog(s->avctx, "prev_intra_luma_pred_flag: %d\n", prev_intra_luma_pred_flag[2*i+j]);
        }

    for (i = 0; i < side; i++) {
        for (j = 0; j < side; j++) {
            if (prev_intra_luma_pred_flag[2*i+j]) {
                s->pu.mpm_idx = ff_hevc_mpm_idx_decode(s);
                av_dlog(s->avctx, "mpm_idx: %d\n", s->pu.mpm_idx);
            } else {
                s->pu.rem_intra_luma_pred_mode = ff_hevc_rem_intra_luma_pred_mode_decode(s);
                av_dlog(s->avctx, "rem_intra_luma_pred_mode: %d\n", s->pu.rem_intra_luma_pred_mode);
            }
            s->pu.intra_pred_mode[2*i+j] =
            luma_intra_pred_mode(s, x0 + pb_size * j, y0 + pb_size * i, pb_size,
                                 prev_intra_luma_pred_flag[2*i+j]);
        }
    }

    chroma_mode = ff_hevc_intra_chroma_pred_mode_decode(s);
    if (chroma_mode != 4) {
        if (s->pu.intra_pred_mode[0] == intra_chroma_table[chroma_mode]) {
            s->pu.intra_pred_mode_c = 34;
        } else {
            s->pu.intra_pred_mode_c = intra_chroma_table[chroma_mode];
        }
    } else {
        s->pu.intra_pred_mode_c = s->pu.intra_pred_mode[0];
    }

    av_dlog(s->avctx, "intra_pred_mode_c: %d\n",
           s->pu.intra_pred_mode_c);
}
static void intra_prediction_unit_default_value(HEVCContext *s, int x0, int y0, int log2_cb_size)
{
    int i;
    int split = s->cu.part_mode == PART_NxN;
    int pb_size = (1 << log2_cb_size) >> split;
    int side = split + 1;
    int size_in_pus = pb_size >> s->sps->log2_min_pu_size;
    for (i = 0; i < side; i++) {
        int x_pu = (x0 + pb_size * i) >> s->sps->log2_min_pu_size;
        int y_pu = (y0 + pb_size * i) >> s->sps->log2_min_pu_size;
        memset(&s->pu.top_ipm[x_pu], INTRA_DC, size_in_pus);
        memset(&s->pu.left_ipm[y_pu], INTRA_DC, size_in_pus);
    }
}

static void hls_coding_unit(HEVCContext *s, int x0, int y0, int log2_cb_size)
{
    int cb_size = 1 << log2_cb_size;

    int log2_min_cb_size = s->sps->log2_min_coding_block_size;
    int length = cb_size >> log2_min_cb_size;
    int x_cb = x0 >> log2_min_cb_size;
    int y_cb = y0 >> log2_min_cb_size;
    int x, y;

    s->cu.x = x0;
    s->cu.y = y0;
    s->cu.no_residual_data_flag = 1;

    s->cu.pred_mode = MODE_INTRA;
    s->cu.part_mode = PART_2Nx2N;
    s->cu.intra_split_flag = 0;
    SAMPLE(s->cu.skip_flag, x0, y0) = 0;
    for (x = 0; x < 4; x++) {
        s->pu.intra_pred_mode[x] = 1;
    }
    if (s->pps->transquant_bypass_enable_flag)
        s->cu.cu_transquant_bypass_flag = ff_hevc_cu_transquant_bypass_flag_decode(s);

    if (s->sh.slice_type != I_SLICE) {
        s->cu.pred_mode = MODE_SKIP;
        SAMPLE(s->cu.skip_flag, x0, y0) = ff_hevc_skip_flag_decode(s, x_cb, y_cb);
        for (x = 0; x < length; x++) {
            for (y = 0; y < length; y++) {
                SAMPLE(s->cu.skip_flag, x_cb+x, y_cb+y) = SAMPLE(s->cu.skip_flag, x0, y0);
            }
        }
        s->cu.pred_mode = s->cu.skip_flag ? MODE_SKIP : MODE_INTER;
    }

    if (SAMPLE(s->cu.skip_flag, x0, y0)) {
        hls_prediction_unit(s, x0, y0, cb_size, cb_size, log2_cb_size, 0);
        intra_prediction_unit_default_value(s, x0, y0, log2_cb_size);
    } else {
        if (s->sh.slice_type != I_SLICE) {
            s->cu.pred_mode = ff_hevc_pred_mode_decode(s);
        }
        if (s->cu.pred_mode != MODE_INTRA ||
            log2_cb_size == s->sps->log2_min_coding_block_size) {
            s->cu.part_mode = ff_hevc_part_mode_decode(s, log2_cb_size);
            av_dlog(s->avctx, "part_mode: %d\n", s->cu.part_mode);
            s->cu.intra_split_flag = s->cu.part_mode == PART_NxN &&
                                     s->cu.pred_mode == MODE_INTRA;
        }

        if (s->cu.pred_mode == MODE_INTRA) {
            if (s->cu.part_mode == PART_2Nx2N && s->sps->pcm_enabled_flag &&
                log2_cb_size >= s->sps->pcm.log2_min_pcm_cb_size &&
                log2_cb_size <= s->sps->pcm.log2_max_pcm_cb_size) {
                s->cu.pcm_flag = ff_hevc_pcm_flag_decode(s);
                av_dlog(s->avctx, "pcm_flag: %d\n", s->cu.pcm_flag);
           }
            if (s->cu.pcm_flag) {
                hls_pcm_sample(s, x0, y0, log2_cb_size);
                intra_prediction_unit_default_value(s, x0, y0, log2_cb_size);
            } else {
                intra_prediction_unit(s, x0, y0, log2_cb_size);
            }
        } else {
            intra_prediction_unit_default_value(s, x0, y0, log2_cb_size);
            switch (s->cu.part_mode) {
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
        if (!s->cu.pcm_flag) {
            if (s->cu.pred_mode != MODE_INTRA &&
                !(s->cu.part_mode == PART_2Nx2N && s->pu.merge_flag)) {
                s->cu.no_residual_data_flag = ff_hevc_no_residual_syntax_flag_decode(s);
            }
            if (s->cu.no_residual_data_flag) {
                s->cu.max_trafo_depth = s->cu.pred_mode == MODE_INTRA ?
                                        s->sps->max_transform_hierarchy_depth_intra + s->cu.intra_split_flag :
                                        s->sps->max_transform_hierarchy_depth_inter;
                hls_transform_tree(s, x0, y0, x0, y0, log2_cb_size,
                                   log2_cb_size, 0, 0);
            }
        }
    }

    set_ct_depth(s, x0, y0, log2_cb_size, s->ct.depth);
}

static int hls_coding_tree(HEVCContext *s, int x0, int y0, int log2_cb_size, int cb_depth)
{
    s->ct.depth = cb_depth;
    if ((x0 + (1 << log2_cb_size) <= s->sps->pic_width_in_luma_samples) &&
        (y0 + (1 << log2_cb_size) <= s->sps->pic_height_in_luma_samples) &&
        min_cb_addr_zs(s, x0 >> s->sps->log2_min_coding_block_size,
                       y0 >> s->sps->log2_min_coding_block_size) >= s->sh.slice_cb_addr_zs &&
        log2_cb_size > s->sps->log2_min_coding_block_size) {
        SAMPLE(s->split_coding_unit_flag, x0, y0) =
        ff_hevc_split_coding_unit_flag_decode(s, cb_depth, x0, y0);
    } else {
        SAMPLE(s->split_coding_unit_flag, x0, y0) =
        (log2_cb_size > s->sps->log2_min_coding_block_size);
    }
    av_dlog(s->avctx, "split_coding_unit_flag: %d\n",
           SAMPLE(s->split_coding_unit_flag, x0, y0));

    if (SAMPLE(s->split_coding_unit_flag, x0, y0)) {
        int more_data = 0;
        int cb_size = (1 << (log2_cb_size)) >> 1;
        int x1 = x0 + cb_size;
        int y1 = y0 + cb_size;

        more_data = hls_coding_tree(s, x0, y0, log2_cb_size - 1, cb_depth + 1);

        if (more_data && x1 < s->sps->pic_width_in_luma_samples)
            more_data = hls_coding_tree(s, x1, y0, log2_cb_size - 1, cb_depth + 1);
        if (more_data && y1 < s->sps->pic_height_in_luma_samples)
            more_data = hls_coding_tree(s, x0, y1, log2_cb_size - 1, cb_depth + 1);
        if (more_data && x1 < s->sps->pic_width_in_luma_samples &&
           y1 < s->sps->pic_height_in_luma_samples) {
            return hls_coding_tree(s, x1, y1, log2_cb_size - 1, cb_depth + 1);
        }
        return ((x1 + cb_size) < s->sps->pic_width_in_luma_samples ||
                (y1 + cb_size) < s->sps->pic_height_in_luma_samples);
    } else {
        hls_coding_unit(s, x0, y0, log2_cb_size);

        av_dlog(s->avctx, "x0: %d, y0: %d, cb: %d, %d\n",
               x0, y0, (1 << log2_cb_size), (1 << (s->sps->log2_ctb_size)));
        if ((!((x0 + (1 << log2_cb_size)) %
               (1 << (s->sps->log2_ctb_size))) ||
             (x0 + (1 << log2_cb_size) >= s->sps->pic_width_in_luma_samples)) &&
            (!((y0 + (1 << log2_cb_size)) %
               (1 << (s->sps->log2_ctb_size))) ||
             (y0 + (1 << log2_cb_size) >= s->sps->pic_height_in_luma_samples))) {
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
static int hls_slice_data(HEVCContext *s)
{
    int ctb_size = 1 << s->sps->log2_ctb_size;
    int pic_size = s->sps->pic_width_in_luma_samples * s->sps->pic_height_in_luma_samples;
    int more_data = 1;
    int x_ctb, y_ctb;

    memset(s->cu.skip_flag, 0, pic_size);

    s->ctb_addr_rs = s->sh.slice_ctb_addr_rs;
    s->ctb_addr_ts = s->pps->ctb_addr_rs_to_ts[s->ctb_addr_rs];

    while (more_data) {
        x_ctb = INVERSE_RASTER_SCAN(s->ctb_addr_rs, ctb_size, ctb_size, s->sps->pic_width_in_luma_samples, 0);
        y_ctb = INVERSE_RASTER_SCAN(s->ctb_addr_rs, ctb_size, ctb_size, s->sps->pic_width_in_luma_samples, 1);
        s->ctb_addr_in_slice = s->ctb_addr_rs - s->sh.slice_address;
        if (s->sh.slice_sample_adaptive_offset_flag[0] ||
            s->sh.slice_sample_adaptive_offset_flag[1])
            hls_sao_param(s, x_ctb >> s->sps->log2_ctb_size, y_ctb >> s->sps->log2_ctb_size);

        more_data = hls_coding_tree(s, x_ctb, y_ctb, s->sps->log2_ctb_size, 0);
        if (!more_data)
            return 0;

        s->ctb_addr_ts++;
        s->ctb_addr_rs = s->pps->ctb_addr_ts_to_rs[s->ctb_addr_ts];

        if (more_data) {
            if ((s->pps->tiles_enabled_flag &&
                 s->pps->tile_id[s->ctb_addr_ts] !=
                 s->pps->tile_id[s->ctb_addr_ts - 1]) ||
                (s->pps->entropy_coding_sync_enabled_flag &&
                ((s->ctb_addr_ts % s->sps->pic_width_in_ctbs) == 0))) {
                //ff_hevc_end_of_sub_stream_one_bit_decode(s);
                ff_hevc_cabac_reinit(s);
                load_states(s);
            }
            if (s->pps->entropy_coding_sync_enabled_flag &&
                    ((s->ctb_addr_ts % s->sps->pic_width_in_ctbs) == 2)) {
                save_states(s);
            }
        }
    }

    return 0;
}

/**
 * @return AVERROR_INVALIDDATA if the packet is not a valid NAL unit,
 * 0 if the unit should be skipped, 1 otherwise
 */
static int hls_nal_unit(HEVCContext *s)
{
    GetBitContext *gb = &s->gb;
    int ret;

    if (get_bits1(gb) != 0)
        return AVERROR_INVALIDDATA;

    s->nal_unit_type = get_bits(gb, 6);

    s->temporal_id = get_bits(gb, 3) - 1;
    ret = (get_bits(gb, 6) != 0);

    av_log(s->avctx, AV_LOG_DEBUG,
           "nal_ref_flag: %d, nal_unit_type: %d, temporal_id: %d\n",
           s->nal_ref_flag, s->nal_unit_type, s->temporal_id);

    return ret;
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

    if (hls_nal_unit(s) <= 0) {
        av_dlog(s->avctx, AV_LOG_INFO, "Skipping NAL unit\n");
        return avpkt->size;
    }

    switch (s->nal_unit_type) {
    case NAL_VPS:
        ff_hevc_decode_nal_vps(s);
        break;
    case NAL_SPS:
        ff_hevc_decode_nal_sps(s);
        break;
    case NAL_PPS:
        ff_hevc_decode_nal_pps(s);
        break;
    case NAL_SEI:
        ff_hevc_decode_nal_sei(s);
        break;
    case NAL_TRAIL_R:
        // fall-through
    case NAL_TRAIL_N: {
        int pic_height_in_min_pu = s->sps->pic_height_in_min_cbs * 4;
        int pic_width_in_min_pu = s->sps->pic_width_in_min_cbs * 4;

        memset(s->pu.left_ipm, INTRA_DC, pic_height_in_min_pu);
        memset(s->pu.top_ipm, INTRA_DC, pic_width_in_min_pu);
        // fall-through
    }
    case NAL_IDR_W_DLP:
        if (hls_slice_header(s) < 0)
            return -1;

        if (s->sao_frame.data[0])
            s->avctx->release_buffer(s->avctx, &s->sao_frame);
        if (s->frame.data[0])
            s->avctx->release_buffer(s->avctx, &s->frame);
        if (s->avctx->get_buffer(s->avctx, &s->frame) < 0) {
            av_log(avctx, AV_LOG_ERROR, "get_buffer() failed\n");
            return -1;
        }
        if (!s->edge_emu_buffer)
            s->edge_emu_buffer = av_malloc((MAX_PB_SIZE + 7) * s->frame.linesize[0]);

        ff_hevc_cabac_init(s);
        if (hls_slice_data(s) < 0)
            return -1;

        if (s->sps->sample_adaptive_offset_enabled_flag) {
            if (s->avctx->get_buffer(s->avctx, &s->sao_frame) < 0) {
                av_log(avctx, AV_LOG_ERROR, "get_buffer() failed\n");
                return -1;
            }
            av_picture_copy((AVPicture*)&s->sao_frame, (AVPicture*)&s->frame,
                            s->avctx->pix_fmt, s->avctx->width, s->avctx->height);
            sao_filter(s);
            *(AVFrame*)data = s->sao_frame;
        } else {
            *(AVFrame*)data = s->frame;
        }

        s->frame.pict_type = AV_PICTURE_TYPE_I;
        s->frame.key_frame = 1;
        *data_size = sizeof(AVFrame);
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

    return 0;
}

static av_cold int hevc_decode_free(AVCodecContext *avctx)
{
    int i;
    HEVCContext *s = avctx->priv_data;

    if (s->frame.data[0])
        s->avctx->release_buffer(s->avctx, &s->frame);
    if (s->sao_frame.data[0])
        s->avctx->release_buffer(s->avctx, &s->sao_frame);

    av_freep(&s->edge_emu_buffer);

    for (i = 0; i < MAX_VPS_COUNT; i++) {
        av_freep(&s->vps_list[i]);
    }

    for (i = 0; i < MAX_SPS_COUNT; i++) {
        av_freep(&s->sps_list[i]);
    }

    for (i = 0; i < MAX_PPS_COUNT; i++) {
        if (s->pps_list[i]) {
            av_freep(&s->pps_list[i]->column_width);
            av_freep(&s->pps_list[i]->row_height);
            av_freep(&s->pps_list[i]->col_bd);
            av_freep(&s->pps_list[i]->row_bd);
            av_freep(&s->pps_list[i]->ctb_addr_rs_to_ts);
            av_freep(&s->pps_list[i]->ctb_addr_ts_to_rs);
            av_freep(&s->pps_list[i]->tile_id);
            av_freep(&s->pps_list[i]->min_cb_addr_zs);
            av_freep(&s->pps_list[i]->min_tb_addr_zs);
        }
        av_freep(&s->pps_list[i]);
    }

    pic_arrays_free(s);
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
    .long_name      = NULL_IF_CONFIG_SMALL("HEVC (High Efficiency Video Coding)"),
};
