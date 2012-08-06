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
#include "golomb.h"
#include "hevcdata.h"
#include "hevc.h"

/**
 * Section 5.7
 */
#define INVERSE_RASTER_SCAN(a,b,c,d,e) ((e) ? ((a)/(ROUNDED_DIV(d, b)))*(c) : ((a)%(ROUNDED_DIV(d, b)))*(b))

/**
 * Value of the luma sample at position (x,y) in the 2D array tab.
 */
#define SAMPLE(tab,x,y) ((tab)[(x) * s->sps->pic_height_in_luma_samples + (y)])

static void clear_pu(struct PUContent *pu_band, int start, int end)
{
    for (int i = start; i < end; i++) {
        pu_band[i].intra_pred_mode = INTRA_DC;
        pu_band[i].ct_depth = -1;
    }
}

static int pic_arrays_init(HEVCContext *s)
{
    int pic_size = s->sps->pic_width_in_luma_samples * s->sps->pic_height_in_luma_samples;
    int ctb_count = s->sps->PicWidthInCtbs * s->sps->PicHeightInCtbs;
    int pic_width_in_min_pu = s->sps->pic_width_in_min_cbs * 4;
    int pic_height_in_min_pu = s->sps->pic_height_in_min_cbs * 4;

    s->sao = av_mallocz(ctb_count * sizeof(struct SAOParams));

    s->split_coding_unit_flag = av_mallocz(pic_size * sizeof(uint8_t));
    s->cu.skip_flag = av_mallocz(pic_size * sizeof(uint8_t));

    s->cu.left_cb_available = av_mallocz(s->sps->pic_height_in_min_cbs);
    s->cu.top_cb_available = av_mallocz(s->sps->pic_width_in_min_cbs);

    s->pu.pu_vert = av_malloc(pic_height_in_min_pu * sizeof(struct PUContent));
    s->pu.pu_horiz = av_malloc(pic_width_in_min_pu * sizeof(struct PUContent));

    if (s->split_coding_unit_flag == NULL || s->cu.skip_flag == NULL ||
        s->pu.pu_vert == NULL || s->pu.pu_horiz == NULL)
        return -1;

    clear_pu(s->pu.pu_vert, 0, pic_height_in_min_pu);
    clear_pu(s->pu.pu_horiz, 0, pic_width_in_min_pu);

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
    av_freep(&s->sao);

    av_freep(&s->split_coding_unit_flag);
    av_freep(&s->cu.skip_flag);

    av_freep(&s->pu.pu_vert);
    av_freep(&s->pu.pu_horiz);

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
    sh->pps_id = get_ue_golomb(gb);
    if (sh->pps_id >= MAX_PPS_COUNT || s->pps_list[sh->pps_id] == NULL) {
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
        //TODO: take into account the luma and chroma bit depths
        if (s->sps->chroma_format_idc == 1) {
            s->avctx->pix_fmt = PIX_FMT_YUV420P;
        } else if (s->sps->chroma_format_idc == 2) {
            s->avctx->pix_fmt = PIX_FMT_YUV422P;
        } else {
            s->avctx->pix_fmt = PIX_FMT_YUV444P;
        }
        ff_hevc_pred_init(&s->hpc, s->sps->bit_depth_luma);
        ff_hevc_dsp_init(&s->hevcdsp, s->sps->bit_depth_luma);
    }

    if (!sh->first_slice_in_pic_flag) {
        slice_address_length = av_ceil_log2_c(s->sps->PicWidthInCtbs *
                                              s->sps->PicHeightInCtbs) +
                               s->pps->SliceGranularity;
        sh->slice_address = get_bits(gb, slice_address_length);
    }

    sh->slice_type = get_ue_golomb(gb);

    sh->dependent_slice_flag = get_bits1(gb);

    if (!sh->dependent_slice_flag) {
        if (s->pps->output_flag_present_flag)
            sh->pic_output_flag = get_bits1(gb);

        if (s->sps->separate_colour_plane_flag == 1)
            sh->colour_plane_id = get_bits(gb, 2);

        if (s->nal_unit_type >= 4 && s->nal_unit_type <= 8) {
            sh->rap_pic_id                   = get_ue_golomb(gb);
            sh->no_output_of_prior_pics_flag = get_bits1(gb);
        }
        if (s->nal_unit_type != NAL_IDR_SLICE) {
            av_log(s->avctx, AV_LOG_ERROR, "TODO: nal_unit_type != NAL_IDR_SLICE\n");
            return -1;
        }

        if (s->sps->sample_adaptive_offset_enabled_flag) {
            sh->slice_sample_adaptive_offset_flag[0] = get_bits1(gb);
            if (sh->slice_sample_adaptive_offset_flag[0]) {
                sh->slice_sample_adaptive_offset_flag[1] = get_bits1(gb);
                sh->slice_sample_adaptive_offset_flag[2] = get_bits1(gb);
            }
        }

        if (s->sps->adaptive_loop_filter_enabled_flag ||
#if REFERENCE_ENCODER_QUIRKS
            s->sps->sample_adaptive_offset_enabled_flag) {
#else
            0) {
#endif
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

        if (s->pps == NULL) {
            av_log(s->avctx, AV_LOG_ERROR, "No PPS active while decoding slice\n");
            return -1;
        }

        if (s->pps->cabac_init_present_flag && sh->slice_type != I_SLICE)
            sh->cabac_init_flag = get_bits1(gb);

        sh->slice_qp_delta = get_se_golomb(gb);
        if (s->pps->deblocking_filter_control_present_flag) {
            av_log(s->avctx, AV_LOG_ERROR,
                   "TODO: deblocking_filter_control_present_flag\n");
            return -1;
        }
#if !REFERENCE_ENCODER_QUIRKS
        if (sh->slice_type != I_SLICE)
#endif
            sh->max_num_merge_cand = 5 - get_ue_golomb(gb);

        if (s->sps->adaptive_loop_filter_enabled_flag) {
            for (int i = 0; i < 3; i++)
                sh->slice_alf_flag[i] = get_bits1(gb);
        }

        if (s->sps->seq_loop_filter_across_slices_enabled_flag
            && (sh->slice_alf_flag[0] || sh->slice_alf_flag[1] ||
                sh->slice_alf_flag[2] || sh->slice_sample_adaptive_offset_flag ||
                !sh->disable_deblocking_filter_flag)) {
            sh->slice_loop_filter_across_slices_enabled_flag = get_bits1(gb);
        } else {
            sh->slice_loop_filter_across_slices_enabled_flag =
                s->sps->seq_loop_filter_across_slices_enabled_flag;
        }
    }

    if (s->pps->slice_header_extension_present_flag) {
        int length = get_ue_golomb(gb);
        for (int i = 0; i < length; i++)
            skip_bits(gb, 8); // slice_header_extension_data_byte
    }

    align_get_bits(gb);

    // Inferred parameters
    sh->slice_qp = 26 + s->pps->pic_init_qp_minus26 + sh->slice_qp_delta;
    sh->slice_ctb_addr_rs = sh->slice_address >> s->pps->SliceGranularity;
    sh->slice_cb_addr_zs = sh->slice_address <<
                        ((s->sps->log2_diff_max_min_coding_block_size -
                          s->pps->SliceGranularity) << 1);

    return 0;
}

#define CTB(tab,x,y) ((tab)[(x) * s->sps->PicHeightInCtbs + (y)])

#define set_sao(elem, value)\
    if (!sao_merge_up_flag && !sao_merge_left_flag) {\
        sao->elem = value;\
    } else if (sao_merge_left_flag) {\
        sao->elem = CTB(s->sao, rx-1, ry).elem;\
    } else if (sao_merge_up_flag) {\
        sao->elem = CTB(s->sao, rx, ry-1).elem;\
    }

/**
 * 7.3.4.1
 */
static int sao_param(HEVCContext *s, int rx, int ry, int c_idx)
{
    int bit_depth = c_idx ? s->sps->bit_depth_chroma: s->sps->bit_depth_luma;
    int shift = bit_depth - FFMIN(bit_depth, 10);

    int sao_merge_left_flag = 0;
    int sao_merge_up_flag = 0;

    struct SAOParams *sao = &CTB(s->sao, rx, ry);

    sao->type_idx[c_idx] = 0;

    if (rx > 0) {
        int left_ctb_in_slice = (s->ctb_addr_in_slice > 0);
        int left_ctb_in_tile =
            (s->pps->tile_id[s->ctb_addr_ts] ==
             s->pps->tile_id[s->pps->ctb_addr_rs_to_ts[s->ctb_addr_rs - 1]]);
        if (left_ctb_in_slice && left_ctb_in_tile)
            sao_merge_left_flag = ff_hevc_sao_merge_left_flag_decode(s, c_idx);
    }
    if (ry > 0 && !sao_merge_left_flag) {
        int up_ctb_in_slice =
            (s->ctb_addr_ts - s->pps->ctb_addr_rs_to_ts[s->ctb_addr_rs - s->sps->PicWidthInCtbs])
            <=  s->ctb_addr_in_slice;
        int up_ctb_in_tile = (s->pps->tile_id[s->ctb_addr_ts] ==
                              s->pps->tile_id[s->pps->ctb_addr_rs_to_ts[s->ctb_addr_rs - s->sps->PicWidthInCtbs]]);
        if (up_ctb_in_slice && up_ctb_in_tile)
            sao_merge_up_flag = ff_hevc_cabac_decode(s, SAO_MERGE_UP_FLAG);
    }
    set_sao(type_idx[c_idx], ff_hevc_cabac_decode(s, SAO_TYPE_IDX));
    av_log(s->avctx, AV_LOG_DEBUG, "sao_type_idx: %d\n",
           sao->type_idx[c_idx]);
    if (sao->type_idx[c_idx] == SAO_BAND)
        set_sao(band_position[c_idx], ff_hevc_cabac_decode(s, SAO_BAND_POSITION));

    if (sao->type_idx[c_idx] != SAO_NOT_APPLIED)
        for (int i = 0; i < 4; i++)
            set_sao(offset_abs[c_idx][i], ff_hevc_sao_offset_abs_decode(s, bit_depth));

    if (sao->type_idx[c_idx] == SAO_BAND)
        for (int i = 0; i < 4; i++)
            if (sao->offset_abs[c_idx][i])
                set_sao(offset_sign[c_idx][i], ff_hevc_sao_offset_sign_decode(s));

    // Inferred parameters
    for (int i = 0; i < 4; i++) {
        sao->offset_val[c_idx][i+1] = sao->offset_abs[c_idx][i] << shift;
        if (sao->type_idx[c_idx] != SAO_BAND) {
            if (i > 1)
                sao->offset_val[c_idx][i+1] = -sao->offset_val[c_idx][i+1];
        } else if (sao->offset_sign[c_idx][i+1]) {
            sao->offset_val[c_idx][i+1] = -sao->offset_val[c_idx][i+1];
        }
    }
    return 0;
}

#undef CTB
#undef set_sao


static av_always_inline int min_cb_addr_zs(HEVCContext *s, int x, int y)
{
    return s->pps->min_cb_addr_zs[s->sps->pic_height_in_min_cbs * x + y];
}

static av_always_inline void get_coord(enum ScanType scan_idx, const uint8_t *scan_x,
                                       const uint8_t *scan_y, int width, int height,
                                       int offset, int n, int *x, int *y)
{
    int pos = n + offset;
    switch (scan_idx) {
    case SCAN_DIAG:
        *x = (scan_x[offset >> 4] << 2) + diag_scan4x4_x[n];
        *y = (scan_y[offset >> 4] << 2) + diag_scan4x4_y[n];
        break;
    case SCAN_HORIZ:
        *y = pos / width;
        *x = pos % width;
        break;
    default: // SCAN_VERT
        *x = pos / height;
        *y = pos % height;
        break;
    }
}

/**
 * 7.3.10
 */
static void residual_coding(HEVCContext *s, int x0, int y0, int log2_trafo_width,
                            int log2_trafo_height, enum ScanType scan_idx, int c_idx)
{
    int last_significant_coeff_x, last_significant_coeff_y;
    int num_coeff = 0;
    int num_last_subset;
    int num_greater1 = 0;
    int trafo_height, trafo_width;
    int x_cg_last_sig, y_cg_last_sig;

    const uint8_t *scan_x, *scan_y;

    int stride = s->frame.linesize[c_idx];
    int hshift = c_idx ? av_pix_fmt_descriptors[s->avctx->pix_fmt].log2_chroma_w : 0;
    int vshift = c_idx ? av_pix_fmt_descriptors[s->avctx->pix_fmt].log2_chroma_h : 0;
    uint8_t *dst = &s->frame.data[c_idx][(y0 >> vshift) * stride + (x0 >> hshift)];

    int16_t coeffs[MAX_TB_SIZE * MAX_TB_SIZE] = { 0 };
    int size = 1 << log2_trafo_width;

    av_log(s->avctx, AV_LOG_DEBUG, "scan_idx: %d, c_idx: %d\n",
           scan_idx, c_idx);

    //TODO
    memset(s->rc.significant_coeff_group_flag, 0, 64*64);

    if (log2_trafo_width == 1 || log2_trafo_height == 1) {
        log2_trafo_width = 2;
        log2_trafo_height = 2;
    }
    trafo_width = 1 << log2_trafo_width;
    trafo_height = 1 << log2_trafo_height;

    if (s->sps->transform_skip_enabled_flag && !s->cu.cu_transquant_bypass_flag &&
        (s->cu.pred_mode == MODE_INTRA) &&
        (log2_trafo_width == 2) && (log2_trafo_height == 2)) {
        av_log(s->avctx, AV_LOG_ERROR, "TODO: transform_skip_flag\n");
        return;
    }

    last_significant_coeff_x =
        ff_hevc_last_significant_coeff_prefix_decode(s, c_idx, log2_trafo_width, 1);
    last_significant_coeff_y =
        ff_hevc_last_significant_coeff_prefix_decode(s, c_idx, log2_trafo_height, 0);


    if (last_significant_coeff_x > 3) {
        int suffix =
            ff_hevc_last_significant_coeff_suffix_decode(s, last_significant_coeff_x, 1);
        last_significant_coeff_x = (1 << ((last_significant_coeff_x >> 1) - 1)) *
                                   (2 + (last_significant_coeff_x & 1)) +
                                   suffix;
    }
    if (last_significant_coeff_y > 3) {
        int suffix =
            ff_hevc_last_significant_coeff_suffix_decode(s, last_significant_coeff_y, 0);
        last_significant_coeff_y = (1 << ((last_significant_coeff_y >> 1) - 1)) *
                                   (2 + (last_significant_coeff_y & 1)) +
                                   suffix;
    }

    if (scan_idx == SCAN_VERT)
        FFSWAP(int, last_significant_coeff_x, last_significant_coeff_y);

    av_log(s->avctx, AV_LOG_DEBUG, "last_significant_coeff_x: %d\n",
           last_significant_coeff_x);
    av_log(s->avctx, AV_LOG_DEBUG, "last_significant_coeff_y: %d\n",
           last_significant_coeff_y);

    if (log2_trafo_width == 3 && log2_trafo_height == 3 && scan_idx != SCAN_DIAG) {
        if (scan_idx == SCAN_HORIZ) {
            x_cg_last_sig = 0;
            y_cg_last_sig = last_significant_coeff_y >> 1;
        } else { //SCAN_VERT
            x_cg_last_sig = last_significant_coeff_x >> 1;
            y_cg_last_sig = 0;
        }
    } else {
        x_cg_last_sig = last_significant_coeff_x >> 2;
        y_cg_last_sig = last_significant_coeff_y >> 2;
    }

    switch (scan_idx) {
    case SCAN_DIAG: {
        int last_x_c = last_significant_coeff_x % 4;
        int last_y_c = last_significant_coeff_y % 4;
        num_coeff = diag_scan4x4_inv[last_y_c][last_x_c];
        if (trafo_width == trafo_height) {
            if (trafo_width == 4) {
                scan_x = scan_1x1;
                scan_y = scan_1x1;
            } else if (trafo_width == 8) {
                num_coeff += diag_scan2x2_inv[y_cg_last_sig][x_cg_last_sig] << 4;
                scan_x = diag_scan2x2_x;
                scan_y = diag_scan2x2_y;
            } else if (trafo_width == 16) {
                num_coeff += diag_scan4x4_inv[y_cg_last_sig][x_cg_last_sig] << 4;
                scan_x = diag_scan4x4_x;
                scan_y = diag_scan4x4_y;
            } else { // trafo_width == 32
                num_coeff += diag_scan8x8_inv[y_cg_last_sig][x_cg_last_sig] << 4;
                scan_x = diag_scan8x8_x;
                scan_y = diag_scan8x8_y;
            }
        } else {
            if (trafo_width == 4) { // 4x16
                num_coeff += y_cg_last_sig << 4;
                scan_x = scan_1x1;
                scan_y = diag_scan1x4_y;
            } else if (trafo_width == 8) { // 8x32
                num_coeff += diag_scan2x8_inv[y_cg_last_sig][x_cg_last_sig] << 4;
                scan_x = diag_scan2x8_x;
                scan_y = diag_scan2x8_y;
            } else if (trafo_width == 16) { // 16x4
                num_coeff += x_cg_last_sig << 4;
                scan_x = diag_scan4x1_x;
                scan_y = scan_1x1;
            } else { //32x8
                num_coeff += diag_scan8x2_inv[y_cg_last_sig][x_cg_last_sig] << 4;
                scan_x = diag_scan8x2_x;
                scan_y = diag_scan8x2_y;
            }
        }
        break;
    }
    case SCAN_HORIZ:
        num_coeff = last_significant_coeff_y * trafo_width + last_significant_coeff_x;
        break;
    default: //SCAN_VERT
        num_coeff = last_significant_coeff_x * trafo_height + last_significant_coeff_y;
        break;
    }
    num_coeff++;
    av_log(s->avctx, AV_LOG_DEBUG, "num_coeff: %d\n",
           num_coeff);

    num_last_subset = (num_coeff - 1) >> 4;

    for(int i = num_last_subset; i >= 0; i--) {
        int first_nz_pos_in_cg, last_nz_pos_in_cg, num_sig_coeff, first_greater1_coeff_idx;
        int sign_hidden;
        int sum_abs;
        int x_cg, y_cg, x_c, y_c;
        int implicit_non_zero_coeff = 0;

        int offset = i << 4;

        uint8_t significant_coeff_flag[16] = {0};
        uint8_t coeff_abs_level_greater1_flag[16] = {0};
        uint8_t coeff_abs_level_greater2_flag[16] = {0};
        uint8_t coeff_sign_flag[16] = {0};

        int first_elem;

        if (log2_trafo_width == 3 && log2_trafo_height == 3 && scan_idx != SCAN_DIAG) {
            if (scan_idx == SCAN_HORIZ) {
                x_cg = 0;
                y_cg = i;
            } else { //SCAN_VERT
                x_cg = i;
                y_cg = 0;
            }
        } else {
            get_coord(scan_idx, scan_x, scan_y, trafo_width, trafo_height,
                      i << 4, 0, &x_cg, &y_cg);
            x_cg >>= 2;
            y_cg >>= 2;
        }

        if ((i < num_last_subset) && (i > 0)) {
            s->rc.significant_coeff_group_flag[x_cg][y_cg] =
                ff_hevc_significant_coeff_group_flag_decode(s, c_idx, x_cg, y_cg,
                                                            log2_trafo_width,
                                                            log2_trafo_height,
                                                            scan_idx);
            implicit_non_zero_coeff = 1;
        } else {
            s->rc.significant_coeff_group_flag[x_cg][y_cg] =
                ((x_cg == x_cg_last_sig && y_cg == y_cg_last_sig) ||
                 (x_cg == 0 && y_cg == 0));
        }
        av_log(s->avctx, AV_LOG_DEBUG, "significant_coeff_group_flag[%d][%d]: %d\n",
               x_cg, y_cg, s->rc.significant_coeff_group_flag[x_cg][y_cg]);

        for(int n = 15; n >= 0; n--) {
            get_coord(scan_idx, scan_x, scan_y, trafo_width, trafo_height,
                      offset, n, &x_c, &y_c);

            if ((n + offset) < (num_coeff - 1) &&
                s->rc.significant_coeff_group_flag[x_cg][y_cg] &&
                (n > 0 || implicit_non_zero_coeff == 0)) {
                significant_coeff_flag[n] =
                    ff_hevc_significant_coeff_flag_decode(s, c_idx, x_c, y_c, log2_trafo_width,
                                                          log2_trafo_height);
                if (significant_coeff_flag[n] == 1)
                    implicit_non_zero_coeff = 0;
            } else {
                int last_cg = 0;
                if (log2_trafo_width == 3 && log2_trafo_height == 3 && scan_idx != SCAN_DIAG) {
                    if (scan_idx == SCAN_HORIZ) {
                        last_cg = (y_c == (y_cg << 1));
                    } else { //SCAN_VERT
                        last_cg = (x_c == (x_cg << 1));
                    }
                } else {
                    last_cg = (x_c == (x_cg << 2) && y_c == (y_cg << 2));
                }
                significant_coeff_flag[n] =
                    ((n + offset) == (num_coeff - 1) ||
                     (last_cg && implicit_non_zero_coeff &&
                      s->rc.significant_coeff_group_flag[x_cg][y_cg])); // not in spec
            }
            av_log(s->avctx, AV_LOG_DEBUG, "significant_coeff_flag(%d,%d): %d\n",
                   x_c, y_c, significant_coeff_flag[n]);

        }

        first_nz_pos_in_cg = 16;
        last_nz_pos_in_cg = -1;
        num_sig_coeff = 0;
        first_greater1_coeff_idx = -1;
        for(int n = 15; n >= 0; n--) {
            if (significant_coeff_flag[n]) {
                if (num_sig_coeff < 8) {
                    coeff_abs_level_greater1_flag[n] =
                        ff_hevc_coeff_abs_level_greater1_flag_decode(s, c_idx, i, n,
                                                                     (num_sig_coeff == 0),
                                                                     (i == num_last_subset),
                                                                     num_greater1);
                    num_sig_coeff++;
                    if (coeff_abs_level_greater1_flag[n] &&
                        first_greater1_coeff_idx == -1)
                        first_greater1_coeff_idx = n;
                }
                if (last_nz_pos_in_cg == -1)
                    last_nz_pos_in_cg = n;
                first_nz_pos_in_cg = n;
                av_log(s->avctx, AV_LOG_DEBUG, "coeff_abs_level_greater1_flag[%d]: %d\n",
                       n, coeff_abs_level_greater1_flag[n]);
            }
        }

        sign_hidden = (last_nz_pos_in_cg - first_nz_pos_in_cg >= 4 &&
                       !s->cu.cu_transquant_bypass_flag);
        if (first_greater1_coeff_idx != -1) {
            coeff_abs_level_greater2_flag[first_greater1_coeff_idx] =
                ff_hevc_coeff_abs_level_greater2_flag_decode(s, c_idx, i, first_greater1_coeff_idx);
            av_log(s->avctx, AV_LOG_DEBUG, "coeff_abs_level_greater2_flag[%d]: %d\n",
                   first_greater1_coeff_idx,
                   coeff_abs_level_greater2_flag[first_greater1_coeff_idx]);
        }

        for(int n = 15; n >= 0; n--) {
            if (significant_coeff_flag[n] &&
                (!s->pps->sign_data_hiding_flag || !sign_hidden ||
                 n != first_nz_pos_in_cg)) {
                coeff_sign_flag[n] = ff_hevc_coeff_sign_flag(s);
                av_log(s->avctx, AV_LOG_DEBUG, "coeff_sign_flag[%d]: %d\n",
                       n, coeff_sign_flag[n]);
            }
        }

        num_sig_coeff = 0;
        sum_abs = 0;
        num_greater1 >>= 1;
        first_elem = 1;
        for(int n = 15; n >= 0; n--) {
            int trans_coeff_level = 0;
            get_coord(scan_idx, scan_x, scan_y, trafo_width, trafo_height,
                      offset, n, &x_c, &y_c);

            if (significant_coeff_flag[n]) {
                trans_coeff_level = 1 + coeff_abs_level_greater1_flag[n] +
                                 coeff_abs_level_greater2_flag[n];
                if (trans_coeff_level == ((num_sig_coeff < 8) ? ((n == first_greater1_coeff_idx) ? 3 : 2
                                                                 ):1)) {
                    trans_coeff_level +=
                        ff_hevc_coeff_abs_level_remaining(s, first_elem, trans_coeff_level);
                    first_elem = 0;
                }
                if (trans_coeff_level > 1)
                    num_greater1++;
                if (s->pps->sign_data_hiding_flag && sign_hidden) {
                    sum_abs += trans_coeff_level;
                    if (n == first_nz_pos_in_cg && (sum_abs%2 == 1))
                        trans_coeff_level = -trans_coeff_level;
                }
                if (coeff_sign_flag[n])
                    trans_coeff_level = -trans_coeff_level;
                num_sig_coeff++;
                av_log(s->avctx, AV_LOG_DEBUG, "trans_coeff_level: %d\n",
                       trans_coeff_level);
            }

            if (s->cu.cu_transquant_bypass_flag) {
                dst[y_c * stride + x_c] += trans_coeff_level;
            } else {
                coeffs[y_c * size + x_c] = trans_coeff_level;
            }
        }
    }

    if (!s->cu.cu_transquant_bypass_flag) {
        int bit_depth = c_idx ? s->sps->bit_depth_chroma : s->sps->bit_depth_luma;
        //TODO: don't hardcode QP
        int qp = c_idx ? 31 : 32;
        s->hevcdsp.dequant(coeffs, log2_trafo_width, qp, bit_depth);
        if (c_idx == 0 && log2_trafo_width == 2) {
            s->hevcdsp.transform_4x4_luma_add(dst, coeffs, stride, s->sps->bit_depth_luma);
        } else {
            s->hevcdsp.transform_add[log2_trafo_width-2](dst, coeffs, stride, bit_depth);
        }
    }
}

/**
 * 7.3.9
 */
static void transform_unit(HEVCContext *s, int x0L, int  y0L, int x0C, int y0C,
                           int log2_trafo_width, int log2_trafo_height,
                           int trafo_depth, int blk_idx) {
    int log2_trafo_size = (log2_trafo_width + log2_trafo_height) >> 1;
    int scan_idx = SCAN_DIAG;
    int scan_idx_c = SCAN_DIAG;

    if (s->cu.pred_mode == MODE_INTRA) {
        s->hpc.intra_pred(s, x0L, y0L, log2_trafo_width, 0);
        if (log2_trafo_size > 2) {
            s->hpc.intra_pred(s, x0C, y0C, log2_trafo_width - 1, 1);
            s->hpc.intra_pred(s, x0C, y0C, log2_trafo_width - 1, 2);
        } else if (blk_idx == 3) {
            s->hpc.intra_pred(s, x0C, y0C, log2_trafo_width, 1);
            s->hpc.intra_pred(s, x0C, y0C, log2_trafo_width, 2);
        }
    }

    if (s->tt.cbf_luma ||
        SAMPLE(s->tt.cbf_cb[trafo_depth], x0C, y0C) ||
        SAMPLE(s->tt.cbf_cr[trafo_depth], x0C, y0C)) {
        if ((s->pps->diff_cu_qp_delta_depth > 0) && !s->tu.is_cu_qp_delta_coded) {
            s->tu.cu_qp_delta = ff_hevc_cabac_decode(s, CU_QP_DELTA);
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
            residual_coding(s, x0L, y0L, log2_trafo_width, log2_trafo_height,
                             scan_idx, 0);
        if (log2_trafo_size > 2) {
            if (SAMPLE(s->tt.cbf_cb[trafo_depth], x0C, y0C))
                residual_coding(s, x0C, y0C, log2_trafo_width - 1, log2_trafo_height - 1,
                                scan_idx_c, 1);
            if (SAMPLE(s->tt.cbf_cr[trafo_depth], x0C, y0C))
                residual_coding(s, x0C, y0C, log2_trafo_width - 1, log2_trafo_height - 1,
                                 scan_idx_c, 2);
        } else if (blk_idx == 3) {
            if (SAMPLE(s->tt.cbf_cb[trafo_depth], x0C, y0C))
                residual_coding(s, x0C, y0C, log2_trafo_width, log2_trafo_height,
                                 scan_idx_c, 1);
            if (SAMPLE(s->tt.cbf_cr[trafo_depth], x0C, y0C))
                residual_coding(s, x0C, y0C, log2_trafo_width, log2_trafo_height,
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
            SAMPLE(s->tt.cbf_cr[trafo_depth - 1], xBase, yBase);
    }

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
    s->tt.inter_tb_split_direction_c = 2;

    if (log2_trafo_size <= s->sps->log2_min_transform_block_size +
        s->sps->log2_diff_max_min_coding_block_size &&
        log2_trafo_size > s->sps->log2_min_transform_block_size &&
        trafo_depth < s->cu.max_trafo_depth &&
        !(s->cu.intra_split_flag && trafo_depth == 0)) {
        SAMPLE(s->tt.split_transform_flag[trafo_depth], x0L, y0L) =
            ff_hevc_split_transform_flag_decode(s, trafo_depth);
    } else {
        SAMPLE(s->tt.split_transform_flag[trafo_depth], x0L, y0L) =
            (log2_trafo_size >
             s->sps->log2_min_transform_block_size +
             s->sps->log2_diff_max_min_coding_block_size ||
             (s->cu.intra_split_flag && trafo_depth == 0) ||
             s->tt.inter_split_flag);
    }

    if (trafo_depth == 0 || log2_trafo_size > 2) {
        if (trafo_depth == 0 || SAMPLE(s->tt.cbf_cb[trafo_depth - 1], xBase, yBase)) {
            SAMPLE(s->tt.cbf_cb[trafo_depth], x0C, y0C) =
                ff_hevc_cbf_cb_cr_decode(s, trafo_depth);
            av_log(s->avctx, AV_LOG_DEBUG,
                   "cbf_cb: %d\n", SAMPLE(s->tt.cbf_cb[trafo_depth], x0C, y0C));
        }
        if (trafo_depth == 0 || SAMPLE(s->tt.cbf_cr[trafo_depth - 1], xBase, yBase)) {
            SAMPLE(s->tt.cbf_cr[trafo_depth], x0C, y0C) =
                ff_hevc_cbf_cb_cr_decode(s, trafo_depth);
            av_log(s->avctx, AV_LOG_DEBUG,
                   "cbf_cr: %d\n", SAMPLE(s->tt.cbf_cr[trafo_depth], x0C, y0C));
        }
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
        transform_tree(s, x0L, y0L, x0C, y0C, x0C, y0C, log2_cb_size,
                       log2_trafo_width - 1, log2_trafo_height - 1, trafo_depth + 1, 0);
        transform_tree(s, x1L, y1L, x1C, y1C, x0C, y0C, log2_cb_size,
                       log2_trafo_width - 1, log2_trafo_height - 1, trafo_depth + 1, 1);
        transform_tree(s, x2L, y2L, x2C, y2C, x0C, y0C, log2_cb_size,
                       log2_trafo_width - 1, log2_trafo_height - 1, trafo_depth + 1, 2);
        transform_tree(s, x3L, y3L, x3C, y3C, x0C, y0C, log2_cb_size,
                       log2_trafo_width - 1, log2_trafo_height - 1, trafo_depth + 1, 3);
    } else {
        if (s->cu.pred_mode == MODE_INTRA || trafo_depth != 0 ||
            SAMPLE(s->tt.cbf_cb[trafo_depth], x0C, y0C) ||
            SAMPLE(s->tt.cbf_cr[trafo_depth], x0C, y0C))
            s->tt.cbf_luma = ff_hevc_cbf_luma_decode(s, trafo_depth, log2_trafo_size,
                                                     log2_max_trafo_size);

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
            s->frame.data[0][(y0 + j) * s->frame.linesize[0] + (x0 + i)]
                = get_bits(gb, s->sps->pcm.bit_depth_luma) <<
                (s->sps->bit_depth_luma - s->sps->pcm.bit_depth_luma);

    //TODO: put the samples at the correct place in the frame
    for (int i = 0; i < (1 << (log2_cb_size << 1)) >> 1; i++)
        get_bits(gb, s->sps->pcm.bit_depth_chroma);

    s->num_pcm_block--;
}

/**
 * 8.4.1
 */
static int luma_intra_pred_mode(HEVCContext *s, int x0, int y0, int pu_size,
                                uint8_t prev_intra_luma_pred_flag)
{
    int candidate[3];
    enum IntraPredMode intra_pred_mode;

    int x_pu = x0 >> s->sps->log2_min_pu_size;
    int y_pu = y0 >> s->sps->log2_min_pu_size;
    int next_x_pu = (x0 + pu_size) >> s->sps->log2_min_pu_size;
    int next_y_pu = (y0 + pu_size) >> s->sps->log2_min_pu_size;

    int cand_up = s->pu.pu_horiz[x_pu].intra_pred_mode;
    int cand_left = s->pu.pu_vert[y_pu].intra_pred_mode;

    int y_ctb = (y0 >>
                 (s->sps->log2_min_coding_block_size +
                  s->sps->log2_diff_max_min_coding_block_size))
                << (s->sps->log2_min_coding_block_size +
                    s->sps->log2_diff_max_min_coding_block_size);

    // intra_pred_mode prediction does not cross vertical CTB boundaries
    if ((y0 - 1) < y_ctb)
        cand_up = INTRA_DC;

    av_log(s->avctx, AV_LOG_DEBUG, "cand_left: %d, cand_up: %d\n",
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
            FFSWAP(enum IntraPredMode, candidate[0], candidate[1]);
        if (candidate[0] > candidate[2])
            FFSWAP(enum IntraPredMode, candidate[0], candidate[2]);
        if (candidate[1] > candidate[2])
            FFSWAP(enum IntraPredMode, candidate[1], candidate[2]);

        intra_pred_mode = s->pu.rem_intra_luma_pred_mode;
        for (int i = 0; i < 3; i++) {
            av_log(s->avctx, AV_LOG_DEBUG, "candidate[%d] = %d\n",
                   i, candidate[i]);
            if (intra_pred_mode >= candidate[i])
                intra_pred_mode++;
        }
    }

    for (int i = x_pu; i < next_x_pu; i++) {
        s->pu.pu_horiz[i].intra_pred_mode = intra_pred_mode;
        s->pu.pu_horiz[i].ct_depth = s->ct.depth;
    }

    for (int i = y_pu; i < next_y_pu; i++) {
        s->pu.pu_vert[i].intra_pred_mode = intra_pred_mode;
        s->pu.pu_vert[i].ct_depth = s->ct.depth;
    }

    av_log(s->avctx, AV_LOG_DEBUG, "intra_pred_mode: %d\n",
           intra_pred_mode);
    return intra_pred_mode;
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
            uint8_t prev_intra_luma_pred_flag[4];
            int d0 = 1 << log2_cb_size;
            int d1 = d0 >> 1;
            if (!(x0 % d0) && !(y0 % d0)) {
                int part = s->cu.part_mode == PART_2Nx2N ? 1 : 2;
                for (int i = 0; i < part; i++)
                    for (int j = 0; j < part; j++)
                        prev_intra_luma_pred_flag[2*i+j] =
                            ff_hevc_cabac_decode(s, PREV_INTRA_LUMA_PRED_FLAG);

                for (int i = 0; i < part; i++) {
                    for (int j = 0; j < part; j++) {
                        if (prev_intra_luma_pred_flag[2*i+j]) {
                            s->pu.mpm_idx =
                                ff_hevc_cabac_decode(s, MPM_IDX);
                            av_log(s->avctx, AV_LOG_DEBUG, "mpm_idx: %d\n", s->pu.mpm_idx);
                        } else {
                            s->pu.rem_intra_luma_pred_mode =
                                ff_hevc_cabac_decode(s, REM_INTRA_LUMA_PRED_MODE);
                            av_log(s->avctx, AV_LOG_DEBUG, "rem_intra_luma_pred_mode: %d\n", s->pu.rem_intra_luma_pred_mode);
                        }
                        s->pu.intra_pred_mode[2*i+j] =
                            luma_intra_pred_mode(s, x0 + d1*j, y0 + d1*i,
                                                 (s->cu.part_mode == PART_2Nx2N) ? d0 : d1,
                                                 prev_intra_luma_pred_flag[2*i+j]);
                    }
                }
            }
            if (s->cu.part_mode == PART_2Nx2N || ((x0 % d0) && (y0 % d0))) {
                int intra_chroma_pred_mode =
                    ff_hevc_cabac_decode(s, INTRA_CHROMA_PRED_MODE);
                switch (intra_chroma_pred_mode) {
                case 0:
                    s->pu.intra_pred_mode_c = (s->pu.intra_pred_mode[0] == 0) ? 34 : 0;
                    break;
                case 1:
                    s->pu.intra_pred_mode_c = (s->pu.intra_pred_mode[0] == 26) ? 34 : 26;
                    break;
                case 2:
                    s->pu.intra_pred_mode_c = (s->pu.intra_pred_mode[0] == 10) ? 34 : 10;
                    break;
                case 3:
                    s->pu.intra_pred_mode_c = (s->pu.intra_pred_mode[0] == 1) ? 34 : 1;
                    break;
                case 4:
                    if (s->sps->chroma_pred_from_luma_enabled_flag == 1) {
                        s->pu.intra_pred_mode_c = INTRA_FROM_LUMA;
                        break;
                    }
                case 5:
                    s->pu.intra_pred_mode_c = s->pu.intra_pred_mode[0];
                    break;
                }
                av_log(s->avctx, AV_LOG_DEBUG, "intra_pred_mode_c: %d\n",
                       s->pu.intra_pred_mode_c);
            }
        }
    } else {
        av_log(s->avctx, AV_LOG_ERROR, "TODO: pred_mode != MODE_INTRA\n");
        return;
    }
}

static av_always_inline void set_cu_available(HEVCContext *s, int x0, int y0,
                                              int log2_cb_size, int available)
{
    int length = (1 << log2_cb_size) >> s->sps->log2_min_coding_block_size;
    int x_cb = x0 >> s->sps->log2_min_coding_block_size;
    int y_cb = y0 >> s->sps->log2_min_coding_block_size;

    memset(&s->cu.top_cb_available[x_cb], available, length);
    memset(&s->cu.left_cb_available[y_cb], available, length);
}

/**
 * 7.3.6
 */
static void coding_unit(HEVCContext *s, int x0, int y0, int log2_cb_size)
{
    int cb_size = 1 << log2_cb_size;
    int x1, y1, x2, y2, x3, y3;
    int available = (s->cu.pred_mode == MODE_INTRA ||
                     !s->pps->constrained_intra_pred_flag);

    s->cu.x = x0;
    s->cu.y = y0;
    s->cu.no_residual_data_flag = 0;

    s->cu.pred_mode = MODE_INTRA;
    s->cu.part_mode = PART_2Nx2N;
    s->cu.intra_split_flag = 0;

    if (s->pps->transquant_bypass_enable_flag)
        s->cu.cu_transquant_bypass_flag = ff_hevc_cu_transquant_bypass_flag_decode(s);

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
        if (s->cu.pred_mode != MODE_INTRA ||
            log2_cb_size == s->sps->log2_min_coding_block_size) {
            s->cu.part_mode = ff_hevc_part_mode_decode(s, log2_cb_size);
            av_log(s->avctx, AV_LOG_DEBUG, "part_mode: %d\n", s->cu.part_mode);
            s->cu.intra_split_flag = (s->cu.part_mode == PART_NxN &&
                                      s->cu.pred_mode == MODE_INTRA);
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

    set_cu_available(s, x0, y0, log2_cb_size, available);
}

/**
 * 7.3.5
 */
static int coding_tree(HEVCContext *s, int x0, int y0, int log2_cb_size, int cb_depth)
{
    s->ct.depth = cb_depth;
    if ((x0 + (1 << log2_cb_size) <= s->sps->pic_width_in_luma_samples) &&
        (y0 + (1 << log2_cb_size) <= s->sps->pic_height_in_luma_samples) &&
        min_cb_addr_zs(s, x0 >> s->sps->log2_min_coding_block_size,
                       y0 >> s->sps->log2_min_coding_block_size) >= s->sh.slice_cb_addr_zs &&
        log2_cb_size > s->sps->log2_min_coding_block_size && s->num_pcm_block == 0) {
        SAMPLE(s->split_coding_unit_flag, x0, y0) =
            ff_hevc_split_coding_unit_flag_decode(s, cb_depth, x0, y0);
    } else {
        SAMPLE(s->split_coding_unit_flag, x0, y0) =
            (log2_cb_size > s->sps->log2_min_coding_block_size);
    }
    av_log(s->avctx, AV_LOG_DEBUG, "split_coding_unit_flag: %d\n",
           SAMPLE(s->split_coding_unit_flag, x0, y0));

    //TODO

    if (SAMPLE(s->split_coding_unit_flag, x0, y0)) {
        int more_data = 0;
        int cb_size = (1 << (log2_cb_size)) >> 1;
        int x1 = x0 + cb_size;
        int y1 = y0 + cb_size;
        int x0_pu = x0 >> s->sps->log2_min_pu_size;
        int x1_pu = x1 >> s->sps->log2_min_pu_size;
        int x2_pu = (x1 + cb_size) >> s->sps->log2_min_pu_size;
        int y0_pu = y0 >> s->sps->log2_min_pu_size;
        int y1_pu = y1 >> s->sps->log2_min_pu_size;
        int y2_pu = (y1 + cb_size) >> s->sps->log2_min_pu_size;

        more_data = coding_tree(s, x0, y0, log2_cb_size - 1, cb_depth + 1);

        if(more_data && x1 < s->sps->pic_width_in_luma_samples) {
            more_data = coding_tree(s, x1, y0, log2_cb_size - 1, cb_depth + 1);
        } else if (x1 < s->sps->pic_width_in_luma_samples) {
            set_cu_available(s, x1, y0, log2_cb_size - 1, 0);
            clear_pu(s->pu.pu_vert, y0_pu, y1_pu);
            clear_pu(s->pu.pu_horiz, x1_pu, x2_pu);
        }
        if (more_data && y1 < s->sps->pic_height_in_luma_samples) {
            more_data = coding_tree(s, x0, y1, log2_cb_size - 1, cb_depth + 1);
        } else if (y1 < s->sps->pic_height_in_luma_samples) {
            set_cu_available(s, x0, y1, log2_cb_size - 1, 0);
            clear_pu(s->pu.pu_vert, y1_pu, y2_pu);
            clear_pu(s->pu.pu_horiz, x0_pu, x1_pu);
        }
        if(more_data && x1 < s->sps->pic_width_in_luma_samples &&
           y1 < s->sps->pic_height_in_luma_samples) {
            return coding_tree(s, x1, y1, log2_cb_size - 1, cb_depth + 1);
        } else if (x1 < s->sps->pic_width_in_luma_samples &&
                   y1 < s->sps->pic_height_in_luma_samples) {
            set_cu_available(s, x1, y1, log2_cb_size - 1, 0);
            clear_pu(s->pu.pu_vert, y1_pu, y2_pu);
            clear_pu(s->pu.pu_horiz, x1_pu, x2_pu);
        }
        return ((x1 + cb_size) < s->sps->pic_width_in_luma_samples ||
                (y1 + cb_size) < s->sps->pic_height_in_luma_samples);
    } else {
        //TODO

        if (s->num_pcm_block == 0) {
            coding_unit(s, x0, y0, log2_cb_size);
        } else {
            pcm_sample(s, x0, y0, log2_cb_size);
        }

        av_log(s->avctx, AV_LOG_DEBUG, "x0: %d, y0: %d, cb: %d, %d\n",
               x0, y0, (1 << log2_cb_size), (1 << (s->sps->log2_min_coding_block_size
                                               + s->sps->log2_diff_max_min_coding_block_size)));
        if ((!((x0 + (1 << log2_cb_size)) %
               ((1 << (s->sps->log2_min_coding_block_size +
                       s->sps->log2_diff_max_min_coding_block_size)) >>
                s->pps->slice_granularity)) ||
             (x0 + (1 << log2_cb_size) >= s->sps->pic_width_in_luma_samples)) &&
            (!((y0 + (1 << log2_cb_size)) %
               ((1 << (s->sps->log2_min_coding_block_size +
                       s->sps->log2_diff_max_min_coding_block_size)) >>
                s->pps->slice_granularity)) ||
             (y0 + (1 << log2_cb_size) >= s->sps->pic_height_in_luma_samples)) &&
            s->num_pcm_block == 0) {
            int end_of_slice_flag = ff_hevc_cabac_decode(s, END_OF_SLICE_FLAG);
            return !end_of_slice_flag;
        } else {
            return 1;
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
    s->ctb_addr_ts = s->pps->ctb_addr_rs_to_ts[s->ctb_addr_rs];

    while (1) {
        x_ctb = INVERSE_RASTER_SCAN(s->ctb_addr_rs, ctb_size, ctb_size, s->sps->pic_width_in_luma_samples, 0);
        y_ctb = INVERSE_RASTER_SCAN(s->ctb_addr_rs, ctb_size, ctb_size, s->sps->pic_width_in_luma_samples, 1);
        s->num_pcm_block = 0;
        s->ctb_addr_in_slice = s->ctb_addr_rs - (s->sh.slice_address >> s->pps->SliceGranularity);
        for (int i = 0; i < 3; i++) {
            if (s->sh.slice_sample_adaptive_offset_flag[i])
                sao_param(s, x_ctb >> s->sps->Log2CtbSize, y_ctb >> s->sps->Log2CtbSize, i);
        }
        for (int i = 0; i < 3; i++) {
            if (s->sh.slice_alf_flag[i]) {
                av_log(s->avctx, AV_LOG_ERROR, "TODO: slice_alf_flag\n");
                return -1;
            }
        }

        more_data = coding_tree(s, x_ctb, y_ctb, s->sps->Log2CtbSize, 0);
        if (!more_data)
            return 0;

        s->ctb_addr_ts++;
        s->ctb_addr_rs = s->pps->ctb_addr_ts_to_rs[s->ctb_addr_ts];

        if (more_data && ((s->pps->tiles_or_entropy_coding_sync_idc == 1) &&
                          s->pps->tile_id[s->ctb_addr_ts] !=
                          s->pps->tile_id[s->ctb_addr_ts - 1]) ||
            (s->pps->tiles_or_entropy_coding_sync_idc == 2 &&
             ((s->ctb_addr_ts % s->sps->PicWidthInCtbs) == 0)))
            align_get_bits(&s->gb);
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
    case NAL_VPS:
        ff_hevc_decode_nal_vps(s);
        break;
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
        }
        av_freep(&s->sps_list[i]);
    }

    for (int i = 0; i < MAX_PPS_COUNT; i++) {
        if (s->pps_list[i]) {
            av_freep(&s->pps_list[i]->column_width);
            av_freep(&s->pps_list[i]->row_height);
            av_freep(&s->pps_list[i]->col_bd);
            av_freep(&s->pps_list[i]->row_bd);
            av_freep(&s->pps_list[i]->ctb_addr_rs_to_ts);
            av_freep(&s->pps_list[i]->ctb_addr_ts_to_rs);
            av_freep(&s->pps_list[i]->tile_id);
            av_freep(&s->pps_list[i]->min_cb_addr_zs);
        }
        av_freep(&s->pps_list[i]);
    }

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
