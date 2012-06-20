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
 * Value of the luma sample at position (x,y) in the 2D array tab.
 */
#define SAMPLE(tab,x,y) ((tab)[(x) * s->sps->pic_height_in_luma_samples + (y)])

static int pic_arrays_init(HEVCContext *s)
{
    int pic_size = s->sps->pic_width_in_luma_samples * s->sps->pic_height_in_luma_samples;

    for (int i = 0; i < 3; i++) {
        s->sao_type_idx[i] = av_mallocz(pic_size * sizeof(enum SAOType));
        s->sao_band_position[i] = av_mallocz(pic_size * sizeof(int));
        if (s->sao_type_idx[i] == NULL || s->sao_band_position[i] == NULL)
            return -1;
    }

    s->split_coding_unit_flag = av_mallocz(pic_size * sizeof(uint8_t));
    s->cu.skip_flag = av_mallocz(pic_size * sizeof(uint8_t));
    s->pu.prev_intra_luma_pred_flag = av_mallocz(pic_size * sizeof(uint8_t));
    s->pu.mpm_idx = av_mallocz(pic_size * sizeof(int));
    s->pu.rem_intra_luma_pred_mode = av_mallocz(pic_size * sizeof(uint8_t));
    s->pu.intra_chroma_pred_mode = av_mallocz(pic_size * sizeof(int));

    if (s->split_coding_unit_flag == NULL || s->cu.skip_flag == NULL ||
        s->pu.prev_intra_luma_pred_flag == NULL || s->pu.mpm_idx == NULL ||
        s->pu.rem_intra_luma_pred_mode == NULL || s->pu.intra_chroma_pred_mode == NULL)
        return -1;

    for (int i = 0; i < MAX_TRANSFORM_DEPTH; i++) {
        s->tt.split_transform_flag[i] = av_mallocz(pic_size * sizeof(uint8_t));
        s->tt.cbf_cb[i] = av_mallocz(pic_size * sizeof(uint8_t));
        s->tt.cbf_cr[i] = av_mallocz(pic_size * sizeof(uint8_t));
        if (s->tt.split_transform_flag[i] == NULL || s->tt.cbf_cb[i] == NULL ||
            s->tt.cbf_cr[i] == NULL)
            return -1;
    }

    return 0;
}

static void pic_arrays_free(HEVCContext *s)
{
    for (int i = 0; i < 3; i++) {
        av_freep(&s->sao_type_idx[i]);
        av_freep(&s->sao_band_position[i]);
    }

    av_freep(&s->split_coding_unit_flag);
    av_freep(&s->cu.skip_flag);
    av_freep(&s->pu.prev_intra_luma_pred_flag);
    av_freep(&s->pu.mpm_idx);
    av_freep(&s->pu.rem_intra_luma_pred_mode);
    av_freep(&s->pu.intra_chroma_pred_mode);

    for (int i = 0; i < MAX_TRANSFORM_DEPTH; i++) {
        av_freep(&s->tt.split_transform_flag[i]);
        av_freep(&s->tt.cbf_cb[i]);
        av_freep(&s->tt.cbf_cr[i]);
    }
}

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
        if (s->sps != s->sps_list[s->pps->sps_id]) {
            s->sps = s->sps_list[s->pps->sps_id];
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
            //TODO: take into account the luma and chroma bit depths
            if (s->sps->chroma_format_idc == 1) {
                s->avctx->pix_fmt = PIX_FMT_YUV420P;
            } else if (s->sps->chroma_format_idc == 2) {
                s->avctx->pix_fmt = PIX_FMT_YUV422P;
            } else {
                s->avctx->pix_fmt = PIX_FMT_YUV444P;
            }
        }

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
            sh->slice_sample_adaptive_offset_flag = get_bits1(gb);
            if (sh->slice_sample_adaptive_offset_flag) {
                sh->sao_cb_enable_flag = get_bits1(gb);
                sh->sao_cr_enable_flag = get_bits1(gb);
            }
        }

        if (s->sps->scaling_list_enable_flag ||
            s->sps->deblocking_filter_in_aps_enabled_flag ||
            s->sps->sample_adaptive_offset_enabled_flag ||
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
    SAMPLE(s->sao_type_idx[c_idx], rx, ry) = ff_hevc_cabac_decode(s, SAO_TYPE_IDX);
    av_log(s->avctx, AV_LOG_DEBUG, "sao_type_idx: %d\n",
           SAMPLE(s->sao_type_idx[c_idx], rx, ry));

    if (SAMPLE(s->sao_type_idx[c_idx], rx, ry) == SAO_BAND)
        SAMPLE(s->sao_band_position[c_idx], rx, ry) = ff_hevc_cabac_decode(s, SAO_BAND_POSITION);

    if (SAMPLE(s->sao_type_idx[c_idx], rx, ry) != SAO_NOT_APPLIED)
        av_log(s->avctx, AV_LOG_ERROR, "TODO: sao_type_idx != 0\n");
    return 0;
}

/**
 * 7.3.4.1
 */
static int sao_unit_cabac(HEVCContext *s, int rx, int ry, int c_idx)
{
    s->sao_merge_left_flag = 0;
    s->sao_merge_up_flag = 0;
    if (rx > 0 &&
        s->sps->tile_id[s->ctb_addr_ts] ==
        s->sps->tile_id[s->sps->ctb_addr_rs_to_ts[s->ctb_addr_rs - 1]])
        s->sao_merge_left_flag = ff_hevc_cabac_decode(s, SAO_MERGE_LEFT_FLAG);
    if (!s->sao_merge_left_flag) {
        if (ry > 0 &&
            (s->ctb_addr_ts -
             s->sps->ctb_addr_rs_to_ts[s->ctb_addr_rs - s->sps->PicWidthInCtbs]) <=
            s->ctb_addr_in_slice &&
            s->sps->tile_id[s->ctb_addr_ts] ==
            s->sps->tile_id[s->sps->ctb_addr_rs_to_ts[s->ctb_addr_rs - s->sps->PicWidthInCtbs]])
            s->sao_merge_up_flag = ff_hevc_cabac_decode(s, SAO_MERGE_UP_FLAG);
        if (!s->sao_merge_up_flag)
            sao_offset_cabac(s, rx, ry, c_idx);
    }
    //TODO: infer sao_type_idx and sao_band_position when they are not present
    return 0;
}


static av_always_inline int min_cb_addr_zs(HEVCContext *s, int x, int y)
{
    return s->sps->min_cb_addr_zs[s->sps->pic_height_in_min_cbs * x + y];
}

/**
 * 7.3.10
 */
static void residual_coding(int x0, int y0, int log2TrafoWidth, int log2TrafoHeight,
                            int scanIdx, int cIdx)
{
}

/**
 * 7.3.9
 */
static void transform_unit(HEVCContext *s, int x0L, int  y0L, int x0C, int y0C,
                           int log2_trafo_width, int log2_trafo_height,
                           int trafo_depth, int blk_idx) {
    int log2_trafo_size = (log2_trafo_width + log2_trafo_height) >> 1;
    int scan_idx = 0;
    int scan_idx_c = 0;

    if (s->tt.cbf_luma ||
        SAMPLE(s->tt.cbf_cb[trafo_depth], x0L, y0L) ||
        SAMPLE(s->tt.cbf_cr[trafo_depth], x0L, y0L)) {
        if ((s->pps->diff_cu_qp_delta_depth > 0) && !s->tu.is_cu_qp_delta_coded) {
            s->tu.cu_qp_delta = ff_hevc_cabac_decode(s, CU_QP_DELTA);
            s->tu.is_cu_qp_delta_coded = 1;
        }

        if (s->cu.pred_mode == MODE_INTRA) {
            //TODO
            //if (s->tu.intra_pred_mode != 0) {
                //scan_idx = ScanType[log2_trafo_size - 2][IntraPredMode];
                //scan_idx_c = ScanType[log2_trafo_size - 2][IntraPredModeC];
            //}
        }

        if (s->tt.cbf_luma)
            residual_coding(x0L, y0L, log2_trafo_width, log2_trafo_height,
                             scan_idx, 0);
        if (log2_trafo_size > 2) {
            if (SAMPLE(s->tt.cbf_cb[trafo_depth], x0C, y0C))
                residual_coding(x0C, y0C, log2_trafo_width - 1, log2_trafo_height - 1,
                                scan_idx_c, 1);
            if (SAMPLE(s->tt.cbf_cr[trafo_depth], x0C, y0C))
                residual_coding( x0C, y0C, log2_trafo_width - 1, log2_trafo_height - 1,
                                 scan_idx_c, 2);
        } else if (blk_idx == 3) {
            if (SAMPLE(s->tt.cbf_cb[trafo_depth], x0C, y0C))
                residual_coding( x0C, y0C, log2_trafo_width, log2_trafo_height,
                                 scan_idx_c, 1);
            if (SAMPLE(s->tt.cbf_cr[trafo_depth], x0C, y0C))
                residual_coding( x0C, y0C, log2_trafo_width, log2_trafo_height,
                                 scan_idx_c, 2);
        }
    }
}

/**
 * 7.3.8
 */
static void transform_tree(HEVCContext *s, int x0L, int y0L, int x0C, int y0C,
                           int xBase, int yBase, int log2_cb_size, int log2_trafo_width,
                           int log2_trafo_height, int trafo_depth, int blk_idx)
{
    int log2_trafo_size = (log2_trafo_width + log2_trafo_height) >> 1;
    int log2_max_trafo_size = s->sps->log2_min_transform_block_size +
                              s->sps->log2_diff_max_min_transform_block_size;
    int trafo_height = 1 << log2_trafo_height;
    int trafo_width = 1 << log2_trafo_width;
    int x1L, y1L, x2L, y2L, x3L, y3L;
    int x1C, y1C, x2C, y2C, x3C, y3C;

    //6.6
    if (s->sps->nsrqt_enabled_flag &&
        ((log2_trafo_size == log2_max_trafo_size
          || (log2_trafo_size < log2_max_trafo_size &&
              trafo_depth == 0))
         && log2_trafo_size > (s->sps->log2_min_transform_block_size + 1)
         && (s->cu.part_mode == PART_2NxN
             || s->cu.part_mode == PART_2NxnU
             || s->cu.part_mode == PART_2NxnD))
        || (log2_trafo_size == (s->sps->log2_min_transform_block_size + 1)
            && log2_trafo_width < log2_trafo_height)) {
        s->tt.inter_tb_split_direction_l = 0;
    } else if (s->sps->nsrqt_enabled_flag
               && ((log2_trafo_size == log2_max_trafo_size
                    || (log2_trafo_size < log2_max_trafo_size && trafo_depth == 0))
                   && log2_trafo_size > (s->sps->log2_min_transform_block_size + 1)
                   && (s->cu.part_mode == PART_Nx2N || s->cu.part_mode == PART_nLx2N
                       || s->cu.part_mode == PART_nRx2N))
               || (log2_trafo_size == (s->sps->log2_min_transform_block_size + 1)
                   && log2_trafo_width > log2_trafo_height)) {
        s->tt.inter_tb_split_direction_l = 1;
    } else {
        s->tt.inter_tb_split_direction_l = 2;
    }

    if (trafo_depth > 0 && log2_trafo_size == 2) {
        SAMPLE(s->tt.cbf_cb[trafo_depth], x0C, y0C) =
            SAMPLE(s->tt.cbf_cb[trafo_depth - 1], xBase, yBase);
        SAMPLE(s->tt.cbf_cr[trafo_depth], x0C, y0C) =
            SAMPLE(s->tt.cbf_cb[trafo_depth - 1], xBase, yBase);
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
        SAMPLE(s->tt.split_transform_flag[trafo_depth], x0L, y0L) =
            ff_hevc_cabac_decode(s, SPLIT_TRANSFORM_FLAG);
    } else {
        SAMPLE(s->tt.split_transform_flag[trafo_depth], x0L, y0L) =
            (log2_trafo_size >
             s->sps->log2_min_transform_block_size +
             s->sps->log2_diff_max_min_coding_block_size ||
             (s->cu.intra_split_flag && trafo_depth == 0) ||
             s->tt.inter_split_flag);
    }

    if (trafo_depth == 0 || log2_trafo_size > 2) {
        if (trafo_depth == 0 || SAMPLE(s->tt.cbf_cb[trafo_depth - 1], xBase, yBase))
            SAMPLE(s->tt.cbf_cb[trafo_depth], x0C, y0C) =
                ff_hevc_cabac_decode(s, CBF_CB_CR);
        if (trafo_depth == 0 || SAMPLE(s->tt.cbf_cr[trafo_depth - 1], xBase, yBase))
            SAMPLE(s->tt.cbf_cr[trafo_depth], x0C, y0C) =
                ff_hevc_cabac_decode(s, CBF_CB_CR);
    }

    if (SAMPLE(s->tt.split_transform_flag[trafo_depth], x0L, y0L)) {
        if (s->tt.inter_tb_split_direction_l == 2) {
            x1L = x0L + (trafo_width >> 1);
            y1L = y0L;
            x2L = x0L;
            y2L = y0L + (trafo_height >> 1);
            x3L = x1L;
            y3L = y2L;
        } else {
            x1L = x0L + (trafo_width >> 2) *
                  s->tt.inter_tb_split_direction_l;
            y1L = y0L + (trafo_height >> 2) *
                  (1 - s->tt.inter_tb_split_direction_l);
            x2L = x1L + (trafo_width >> 2) *
                  s->tt.inter_tb_split_direction_l;
            y2L = y1L + (trafo_height >> 2)
                          * (1 - s->tt.inter_tb_split_direction_l);
            x3L = x2L + (trafo_width >> 2) *
                  s->tt.inter_tb_split_direction_l;
            y3L = y2L + (trafo_height >> 2) *
                  (1 - s->tt.inter_tb_split_direction_l);
        }
        if (s->tt.inter_tb_split_direction_c == 2 && log2_trafo_size > 3) {
            x1C = x0C + (trafo_width >> 1);
            y1C = y0C;
            x2C = x0C;
            y2C = y0C + (trafo_height >> 1);
            x3C = x1C;
            y3C = y2C;
        } else if (log2_trafo_size > 3) {
            x1C = x0C + (trafo_width >> 2) *
                  s->tt.inter_tb_split_direction_c;
            y1C = y0C + (trafo_height >> 2) *
                  (1 - s->tt.inter_tb_split_direction_c);
            x2C = x1C + (trafo_width >> 2) *
                  s->tt.inter_tb_split_direction_c;
            y2C = y1C + (trafo_height >> 2) *
                  (1 - s->tt.inter_tb_split_direction_c);
            x3C = x2C + (trafo_width >> 2) *
                  s->tt.inter_tb_split_direction_c;
            y3C = y2C + (trafo_height >> 2) *
                  (1 - s->tt.inter_tb_split_direction_c);
        } else {
            x1C = x0C;
            y1C = y0C;
            x2C = x0C;
            y2C = y0C;
            x3C = x0C;
            y3C = y0C;
        }
        if (s->tt.inter_tb_split_direction_l != 2) {
            log2_trafo_width = log2_trafo_width - 2 * s->tt.inter_tb_split_direction_l + 1;
            log2_trafo_height = log2_trafo_height + 2 * s->tt.inter_tb_split_direction_l - 1;
        }
        transform_tree(s, x0L, y0L, x0C, y0C, x0L, y0L, log2_cb_size,
                       log2_trafo_width - 1, log2_trafo_height - 1, trafo_depth + 1, 0);
        transform_tree(s, x1L, y1L, x1C, y1C, x0L, y0L, log2_cb_size,
                       log2_trafo_width - 1, log2_trafo_height - 1, trafo_depth + 1, 1);
        transform_tree(s, x2L, y2L, x2C, y2C, x0L, y0L, log2_cb_size,
                       log2_trafo_width - 1, log2_trafo_height - 1, trafo_depth + 1, 2);
        transform_tree(s, x3L, y3L, x3C, y3C, x0L, y0L, log2_cb_size,
                       log2_trafo_width - 1, log2_trafo_height - 1, trafo_depth + 1, 3);
    } else {
        if (s->cu.pred_mode == MODE_INTRA || trafo_depth != 0 ||
            SAMPLE(s->tt.cbf_cb[trafo_depth], x0C, y0C) ||
            SAMPLE(s->tt.cbf_cr[trafo_depth], x0C, y0C))
            s->tt.cbf_luma = ff_hevc_cabac_decode(s, CBF_LUMA);

        transform_unit(s, x0L, y0L, x0C,
                       y0C, log2_trafo_width, log2_trafo_height, trafo_depth, blk_idx);
    }

}

/**
 * 7.3.7.2
 */
static void pcm_sample(HEVCContext *s, int x0, int y0, int log2_cb_size)
{
    GetBitContext *gb = &s->gb;
    int cb_size = 1 << log2_cb_size;

    // Directly fill the current frame (section 8.4)
    for (int j = 0; j < cb_size; j++)
        for (int i = 0; i < cb_size; i++)
            s->frame.data[0][(x0 + i) * s->frame.linesize[0] + (y0 + j)]
                = get_bits(gb, s->sps->pcm.bit_depth_luma) <<
                (s->sps->bit_depth_luma - s->sps->pcm.bit_depth_luma);

    //TODO: put the samples at the correct place in the frame
    for (int i = 0; i < (1 << (log2_cb_size << 1)) >> 1; i++)
        get_bits(gb, s->sps->pcm.bit_depth_chroma);

    s->num_pcm_block--;
}

/**
 * 7.3.7
 */
static void prediction_unit(HEVCContext *s, int x0, int y0, int log2_cb_size)
{
    s->pu.pcm_flag = 0;
    if (SAMPLE(s->cu.skip_flag, x0, y0)) {
        av_log(s->avctx, AV_LOG_ERROR, "TODO: slice_type != I_SLICE\n");
        return;
    } else if (s->cu.pred_mode == MODE_INTRA) {
        if (s->cu.part_mode == PART_2Nx2N && s->sps->pcm_enabled_flag &&
            log2_cb_size >= s->sps->pcm.log2_min_pcm_coding_block_size &&
            log2_cb_size <= (s->sps->pcm.log2_min_pcm_coding_block_size +
                             s->sps->pcm.log2_diff_max_min_pcm_coding_block_size)) {
            s->pu.pcm_flag = ff_hevc_cabac_decode(s, PCM_FLAG);
            av_log(s->avctx, AV_LOG_ERROR, "pcm_flag: %d\n", s->pu.pcm_flag);
        }
        if (s->pu.pcm_flag) {
            s->num_pcm_block = 1;
            while (s->num_pcm_block < 4 && get_bits1(&s->gb))
                s->num_pcm_block++;

            align_get_bits(&s->gb);
            pcm_sample(s, x0, y0, log2_cb_size);
        } else {
            //TODO: based on JCTVC-I0302 which doesn't seem to handle all cases
            //to be checked again once it's part of the spec
            int d0 = 1 << log2_cb_size;
            if (!(x0 % d0) && !(y0 % d0)) {
                int d1 = s->cu.part_mode == PART_2Nx2N ? d0 : (d0 >> 1);
                for (int i = 0; i < d0; i += d1)
                    for (int j = 0; j < d0; j += d1)
                        SAMPLE(s->pu.prev_intra_luma_pred_flag, x0 + j, y0 + i) =
                            ff_hevc_cabac_decode(s, PREV_INTRA_LUMA_PRED_FLAG);

                for (int i = 0; i < d0; i += d1) {
                    for (int j = 0; j < d0; j += d1) {
                        if (SAMPLE(s->pu.prev_intra_luma_pred_flag, x0 + j, y0 + i)) {
                            SAMPLE(s->pu.mpm_idx, x0 + j, y0 + i) =
                                ff_hevc_cabac_decode(s, MPM_IDX);
                        } else {
                            SAMPLE(s->pu.rem_intra_luma_pred_mode, x0 + j, y0 + i) =
                                ff_hevc_cabac_decode(s, REM_INTRA_LUMA_PRED_MODE);
                        }
                    }
                }
            }
            if (s->cu.part_mode == PART_2Nx2N) {
                SAMPLE(s->pu.intra_chroma_pred_mode, x0, y0) =
                    ff_hevc_cabac_decode(s, INTRA_CHROMA_PRED_MODE);
                av_log(s->avctx, AV_LOG_ERROR, "intra_chroma: %d\n",
                       SAMPLE(s->pu.intra_chroma_pred_mode, x0, y0));
            } else if ((x0 % d0) && (y0 % d0)) {
                SAMPLE(s->pu.intra_chroma_pred_mode, x0 - (d0 >> 1), y0 - (d0 >> 1)) =
                    ff_hevc_cabac_decode(s, INTRA_CHROMA_PRED_MODE);
            }
        }
    } else {
        av_log(s->avctx, AV_LOG_ERROR, "TODO: pred_mode != MODE_INTRA\n");
        return;
    }
}

/**
 * 7.3.6
 */
static void coding_unit(HEVCContext *s, int x0, int y0, int log2_cb_size)
{
    int cb_size = 1 << log2_cb_size;
    int x1, y1, x2, y2, x3, y3;

    s->cu.no_residual_data_flag = 0;

    s->cu.pred_mode = MODE_INTRA;
    s->cu.part_mode = PART_2Nx2N;
    s->cu.intra_split_flag = 0;

    if (s->sh.slice_type != I_SLICE) {
        av_log(s->avctx, AV_LOG_ERROR, "TODO: slice_type != I_SLICE\n");
        return;
        //SAMPLE(s->cu.skip_flag, x0, y0) = ff_hevc_cabac_decode(s, SKIP_FLAG);
        s->cu.pred_mode = s->cu.skip_flag ? MODE_SKIP : MODE_INTER;
    }

    if (SAMPLE(s->cu.skip_flag, x0, y0)) {
        prediction_unit(s, x0, y0, log2_cb_size);
    } else {
        if (s->sh.slice_type != I_SLICE || s->cu.pred_mode != MODE_INTRA) {
            av_log(s->avctx, AV_LOG_ERROR, "TODO: slice_type != I_SLICE\n");
            return;
        }
        x1 = x0 + (cb_size >> 1);
        y1 = y0 + (cb_size >> 1);
        x2 = x1 - (cb_size >> 2);
        y2 = y1 - (cb_size >> 2);
        x3 = x1 + (cb_size >> 2);
        y3 = y1 + (cb_size >> 2);

        switch (s->cu.part_mode) {
        case PART_2Nx2N:
            prediction_unit(s, x0, y0, log2_cb_size);
            break;
        case PART_2NxN:
            prediction_unit(s, x0, y0, log2_cb_size);
            prediction_unit(s, x0, y1, log2_cb_size);
            break;
        case PART_Nx2N:
            prediction_unit(s, x0, y0, log2_cb_size);
            prediction_unit(s, x1, y0, log2_cb_size);
            break;
        case PART_2NxnU:
            prediction_unit(s, x0, y0, log2_cb_size);
            prediction_unit(s, x0, y2, log2_cb_size);
            break;
        case PART_2NxnD:
            prediction_unit(s, x0, y0, log2_cb_size);
            prediction_unit(s, x0, y3, log2_cb_size);
            break;
        case PART_nLx2N:
            prediction_unit(s, x0, y0, log2_cb_size);
            prediction_unit(s, x2, y0, log2_cb_size);
            break;
        case PART_nRx2N:
            prediction_unit(s, x0, y0, log2_cb_size);
            prediction_unit(s, x3, y0, log2_cb_size);
            break;
        case PART_NxN:
            prediction_unit(s, x0, y0, log2_cb_size);
            prediction_unit(s, x1, y0, log2_cb_size);
            prediction_unit(s, x0, y1, log2_cb_size);
            prediction_unit(s, x1, y1, log2_cb_size);
            break;
        }

        if (!s->pu.pcm_flag) {
            if (s->cu.pred_mode != MODE_INTRA &&
                !(s->cu.part_mode == PART_2Nx2N && s->pu.merge_flag)) {
                av_log(s->avctx, AV_LOG_ERROR, "TODO: pred_mode != MODE_INTRA\n");
                return;
            }
            if (!s->cu.no_residual_data_flag) {
                s->cu.max_trafo_depth = s->cu.pred_mode == MODE_INTRA ?
                                        s->sps->max_transform_hierarchy_depth_intra + s->cu.intra_split_flag :
                                        s->sps->max_transform_hierarchy_depth_inter;
                transform_tree(s, x0, y0, x0, y0, x0, y0, log2_cb_size,
                               log2_cb_size, log2_cb_size, 0, 0);
            }
        }
    }
}

/**
 * 7.3.5
 */
static int coding_tree(HEVCContext *s, int x0, int y0, int log2_cb_size, int cb_depth)
{
    if ((x0 + (1 << log2_cb_size) <= s->sps->pic_width_in_luma_samples) &&
        (y0 + (1 << log2_cb_size) <= s->sps->pic_height_in_luma_samples) &&
        min_cb_addr_zs(s, x0 >> s->sps->log2_min_coding_block_size,
                       y0 >> s->sps->log2_min_coding_block_size) >= s->sh.slice_cb_addr_zs &&
        log2_cb_size > s->sps->log2_min_coding_block_size && s->num_pcm_block == 0) {
        SAMPLE(s->split_coding_unit_flag, x0, y0) = ff_hevc_cabac_decode(s, SPLIT_CODING_UNIT_FLAG);
        av_log(s->avctx, AV_LOG_DEBUG, "split_coding_unit_flag: %d\n",
               SAMPLE(s->split_coding_unit_flag, x0, y0));
    }

    //TODO

    if (SAMPLE(s->split_coding_unit_flag, x0, y0)) {
        av_log(s->avctx, AV_LOG_ERROR, "TODO: split_coding_unit_flag\n");
    } else {
        //TODO

        if (s->num_pcm_block == 0) {
            coding_unit(s, x0, y0, log2_cb_size);
        } else {
            pcm_sample(s, x0, y0, log2_cb_size);
        }
    }

    //TODO
    return 0;
}

/**
 * 7.3.4
 */
static int decode_nal_slice_data(HEVCContext *s)
{
    int ctb_size = 1 << s->sps->Log2CtbSize;
    int more_data = 1;
    int x_ctb, y_ctb;

    s->ctb_addr_rs = s->sh.slice_ctb_addr_rs;
    s->ctb_addr_ts = s->sps->ctb_addr_rs_to_ts[s->ctb_addr_rs];

    while (more_data) {
        x_ctb = INVERSE_RASTER_SCAN(s->ctb_addr_rs, ctb_size, ctb_size, s->sps->pic_width_in_luma_samples, 0);
        y_ctb = INVERSE_RASTER_SCAN(s->ctb_addr_rs, ctb_size, ctb_size, s->sps->pic_width_in_luma_samples, 1);
        s->num_pcm_block = 0;
        s->ctb_addr_in_slice = s->ctb_addr_ts - s->sps->ctb_addr_rs_to_ts[s->sh.slice_ctb_addr_rs];
        for (int i = 0; i < 3; i++) {
            if (s->sh.slice_sample_adaptive_offset_flag)
                sao_unit_cabac(s, x_ctb, y_ctb, i);
        }

        more_data = coding_tree(s, x_ctb, y_ctb, s->sps->Log2CtbSize, 0);
        s->ctb_addr_ts++;
        s->ctb_addr_rs = s->sps->ctb_addr_ts_to_rs[s->ctb_addr_ts];
        //TODO
    }

    return 0;
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
    case NAL_SLICE: {
        if (decode_nal_slice_header(s) < 0) {
            return -1;
        }

        if (s->frame.data[0] != NULL)
            s->avctx->release_buffer(s->avctx, &s->frame);
        if (s->avctx->get_buffer(s->avctx, &s->frame) < 0) {
            av_log(avctx, AV_LOG_ERROR, "get_buffer() failed\n");
            return -1;
        }

        ff_hevc_cabac_init(s);
        if (decode_nal_slice_data(s) < 0) {
            return -1;
        }

        s->frame.pict_type = AV_PICTURE_TYPE_I;
        s->frame.key_frame = 1;
        *(AVFrame*)data = s->frame;
        *data_size = sizeof(AVFrame);
        break;
    }
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

    if (s->frame.data[0] != NULL)
        s->avctx->release_buffer(s->avctx, &s->frame);

    for (int i = 0; i < MAX_SPS_COUNT; i++) {
        if (s->sps_list[i]) {
            for (int j = 0; j < MAX_SHORT_TERM_RPS_COUNT; j++)
                av_freep(&s->sps_list[i]->short_term_rps_list[j]);
            av_freep(&s->sps_list[i]->column_width);
            av_freep(&s->sps_list[i]->row_height);
            av_freep(&s->sps_list[i]->col_bd);
            av_freep(&s->sps_list[i]->row_bd);
            av_freep(&s->sps_list[i]->ctb_addr_rs_to_ts);
            av_freep(&s->sps_list[i]->ctb_addr_ts_to_rs);
            av_freep(&s->sps_list[i]->tile_id);
            av_freep(&s->sps_list[i]->min_cb_addr_zs);
        }
        av_freep(&s->sps_list[i]);
    }

    for (int i = 0; i < MAX_PPS_COUNT; i++)
        av_freep(&s->pps_list[i]);

    for (int i = 0; i < MAX_APS_COUNT; i++)
        av_freep(&s->aps_list[i]);

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
    .long_name      = NULL_IF_CONFIG_SMALL("HEVC"),
};
