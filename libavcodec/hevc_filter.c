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
#include "bit_depth_template.c"

#define LUMA 0
#define CB 1
#define CR 2

static int chroma_tc(HEVCContext *s, int qp_y, int c_idx)
{
    static int qp_c[] = { 29, 30, 31, 32, 33, 33, 34, 34, 35, 35, 36, 36, 37, 37 };
    int qp_i, offset;
    int qp;
    int idxt;

    // slice qp offset is not used for deblocking
    if (c_idx == 1)
        offset = s->pps->cb_qp_offset;
    else
        offset = s->pps->cr_qp_offset;

    qp_i = av_clip_c(qp_y + offset, - s->sps->qp_bd_offset, 57);
    if (qp_i < 30)
        qp = qp_i;
    else if (qp_i > 43)
        qp = qp_i - 6;
    else
        qp = qp_c[qp_i - 30];

    qp += s->sps->qp_bd_offset;

    idxt = av_clip_c(qp + DEFAULT_INTRA_TC_OFFSET + s->sh.tc_offset, 0, 53);
    return tctable[idxt];
}
static int get_qPy_pred(HEVCContext *s, int xC, int yC)
{
    int Log2CtbSizeY         = s->sps->log2_ctb_size;
    int Log2MinTrafoSize     = s->sps->log2_min_transform_block_size;
    int Log2MinCuQpDeltaSize = Log2CtbSizeY - s->pps->diff_cu_qp_delta_depth;
    int xQg                  = xC - ( xC & ( ( 1 << Log2MinCuQpDeltaSize) - 1 ) );
    int yQg                  = yC - ( yC & ( ( 1 << Log2MinCuQpDeltaSize) - 1 ) );
    int x                    = xC >> Log2MinCuQpDeltaSize;
    int y                    = yC >> Log2MinCuQpDeltaSize;
    int qPy_pred;
    int qPy_a;
    int qPy_b;
    int availableA           = ff_hevc_z_scan_block_avail(s, xC, yC, xQg-1, yQg  );
    int availableB           = ff_hevc_z_scan_block_avail(s, xC, yC, xQg  , yQg-1);
    int minTbAddrA;
    int minTbAddrB;
    int ctbAddrA = 0;
    int ctbAddrB = 0;
    if (availableA != 0) {
        int tmpX   = (xQg-1) >> Log2MinTrafoSize;
        int tmpY   =  yQg    >> Log2MinTrafoSize;
        minTbAddrA = s->pps->min_tb_addr_zs[ tmpX + tmpY * s->sps->pic_width_in_min_tbs];
        ctbAddrA   = ( minTbAddrA >> 2 ) * (Log2CtbSizeY - Log2MinTrafoSize);
    }
    if (availableB != 0) {
        int tmpX   =  xQg    >> Log2MinTrafoSize;
        int tmpY   = (yQg-1) >> Log2MinTrafoSize;
        minTbAddrB = s->pps->min_tb_addr_zs[ tmpX + tmpY * s->sps->pic_width_in_min_tbs];
        ctbAddrB   = ( minTbAddrB >> 2 ) * (Log2CtbSizeY - Log2MinTrafoSize);
    }
    // qPy_pred
    if (s->isFirstQPgroup != 0) {
        s->isFirstQPgroup = 0;
        qPy_pred = s->sh.slice_qp;
    } else {
        qPy_pred = s->qp_y;
    }
    // qPy_a
    if ( (availableA == 0) || (ctbAddrA != s->ctb_addr_ts) ) {
        qPy_a = qPy_pred;
    } else {
        qPy_a = s->qp_y_tab[(x-1) + y * s->sps->pic_width_in_min_tbs];
    }
    // qPy_b
    if ( (availableB == 0) || (ctbAddrB != s->ctb_addr_ts) ) {
        qPy_b = qPy_pred;
    } else {
        qPy_b = s->qp_y_tab[x + (y-1) * s->sps->pic_width_in_min_tbs];
    }
    return (qPy_a + qPy_b + 1) >> 1;
}
void ff_hevc_set_qPy(HEVCContext *s, int xC, int yC)
{
    int Log2CtbSizeY         = s->sps->log2_ctb_size;
    int Log2MinCuQpDeltaSize = Log2CtbSizeY - s->pps->diff_cu_qp_delta_depth;
    int x                    = xC >> Log2MinCuQpDeltaSize;
    int y                    = yC >> Log2MinCuQpDeltaSize;
    if (s->tu.cu_qp_delta != 0) {
        s->qp_y = ((get_qPy_pred(s, xC, yC) + s->tu.cu_qp_delta + 52 + 2 * s->sps->qp_bd_offset) %
                (52 + s->sps->qp_bd_offset)) - s->sps->qp_bd_offset;
    } else {
        s->qp_y = get_qPy_pred(s, xC, yC);
    }
    s->qp_y_tab[x + y * s->sps->pic_width_in_min_tbs] = s->qp_y;
}
static int get_qPy(HEVCContext *s, int xC, int yC)
{
    int Log2CtbSizeY         = s->sps->log2_ctb_size;
    int Log2MinCuQpDeltaSize = Log2CtbSizeY - s->pps->diff_cu_qp_delta_depth;
    int x                    = xC >> Log2MinCuQpDeltaSize;
    int y                    = yC >> Log2MinCuQpDeltaSize;
    return s->qp_y_tab[x + y * s->sps->pic_width_in_min_tbs];
}
#ifndef DEBLOCKING_IN_LOOP
void ff_hevc_deblocking_filter(HEVCContext *s)
{
    uint8_t *src;
    int x, y;
    int pixel = 1 + !!(s->sps->bit_depth - 8); // sizeof(pixel)
    int pic_width_in_min_pu = s->sps->pic_width_in_min_cbs * 4;
    int min_pu_size = 1 << (s->sps->log2_min_pu_size - 1);
    int log2_min_pu_size = s->sps->log2_min_pu_size - 1;

    // vertical filtering
    for (y = 0; y < s->sps->pic_height_in_luma_samples; y += 4) {
        for (x = 8; x < s->sps->pic_width_in_luma_samples; x += 8) {
            int bs = s->vertical_bs[(x >> 3) + (y >> 2) * s->bs_width];
            if (bs) {
                int qp_y = get_qPy(s,x,y);
                int qp = qp_y + s->sps->qp_bd_offset;
                const int idxb = av_clip_c(qp + s->sh.beta_offset, 0, MAX_QP);
                const int beta = betatable[idxb];
                int no_p = 0;
                int no_q = 0;
                const int idxt = av_clip_c(qp + DEFAULT_INTRA_TC_OFFSET * (bs - 1) + s->sh.tc_offset, 0, MAX_QP + DEFAULT_INTRA_TC_OFFSET);
                const int tc = tctable[idxt];
                if(s->sps->pcm_enabled_flag && s->sps->pcm.loop_filter_disable_flag) {
                    int xp_pu = (x - 1) / min_pu_size;
                    int xq_pu = x >> log2_min_pu_size;
                    int y_pu = y >> log2_min_pu_size;
                    if (s->is_pcm[y_pu * pic_width_in_min_pu + xp_pu])
                        no_p = 1;
                    if (s->is_pcm[y_pu * pic_width_in_min_pu + xq_pu])
                        no_q = 1;
                }
                src = &s->frame->data[LUMA][y * s->frame->linesize[LUMA] + x];
                s->hevcdsp.hevc_loop_filter_luma(src, pixel, s->frame->linesize[LUMA], no_p, no_q, beta, tc);
                if ((x & 15) == 0 && (y & 7) == 0 && bs == 2) {
                    src = &s->frame->data[CB][(y / 2) * s->frame->linesize[CB] + (x / 2)];
                    s->hevcdsp.hevc_loop_filter_chroma(src, pixel, s->frame->linesize[CB], no_p, no_q, chroma_tc(s, qp_y, CB));
                    src = &s->frame->data[CR][(y / 2) * s->frame->linesize[CR] + (x / 2)];
                    s->hevcdsp.hevc_loop_filter_chroma(src, pixel, s->frame->linesize[CR], no_p, no_q, chroma_tc(s, qp_y, CR));
                }
            }
        }
    }
    // horizontal filtering
    for (y = 8; y < s->sps->pic_height_in_luma_samples; y += 8) {
        int yp_pu = (y - 1) / min_pu_size;
        int yq_pu = y >> log2_min_pu_size;
        for (x = 0; x < s->sps->pic_width_in_luma_samples; x += 4) {
            int bs = s->horizontal_bs[(x + y * s->bs_width) >> 2];
            if (bs) {
                int qp_y = get_qPy(s,x,y);
                int qp = qp_y + s->sps->qp_bd_offset;
                const int idxb = av_clip_c(qp + s->sh.beta_offset, 0, MAX_QP);
                const int beta = betatable[idxb];
                int no_p = 0;
                int no_q = 0;
                const int idxt = av_clip_c(qp + DEFAULT_INTRA_TC_OFFSET * (bs - 1) + s->sh.tc_offset, 0, MAX_QP + DEFAULT_INTRA_TC_OFFSET);
                const int tc = tctable[idxt];
                if(s->sps->pcm_enabled_flag && s->sps->pcm.loop_filter_disable_flag) {
                    int x_pu = x >> log2_min_pu_size;
                    if (s->is_pcm[yp_pu * pic_width_in_min_pu + x_pu])
                        no_p = 1;
                    if (s->is_pcm[yq_pu * pic_width_in_min_pu + x_pu])
                        no_q = 1;
                }
                src = &s->frame->data[LUMA][y * s->frame->linesize[LUMA] + x];
                s->hevcdsp.hevc_loop_filter_luma(src, s->frame->linesize[LUMA], pixel, no_p, no_q, beta, tc);
                if ((x & 7) == 0 && (y & 15) == 0 && bs == 2) {
                    src = &s->frame->data[CB][(y / 2) * s->frame->linesize[CB] + (x / 2)];
                    s->hevcdsp.hevc_loop_filter_chroma(src, s->frame->linesize[CB], pixel, no_p, no_q, chroma_tc(s, qp_y, CB));
                    src = &s->frame->data[CR][(y / 2) * s->frame->linesize[CR] + (x / 2)];
                    s->hevcdsp.hevc_loop_filter_chroma(src, s->frame->linesize[CR], pixel, no_p, no_q, chroma_tc(s, qp_y, CR));
                }
            }
        }
    }
}
#endif


#define CTB(tab, x, y) ((tab)[(y) * s->sps->pic_width_in_ctbs + (x)])
#ifndef SAO_IN_LOOP
void ff_hevc_sao_filter(HEVCContext *s)
{
    //TODO: This should be easily parallelizable
    //TODO: skip CBs when (cu_transquant_bypass_flag || (pcm_loop_filter_disable_flag && pcm_flag))
    int c_idx, y_ctb, x_ctb;
    int c_idx_min = s->sh.slice_sample_adaptive_offset_flag[0] != 0 ? 0 : 1;
    int c_idx_max = s->sh.slice_sample_adaptive_offset_flag[1] != 0 ? 3 : 1;
    for (c_idx = c_idx_min; c_idx < c_idx_max; c_idx++) {
        int stride = s->frame->linesize[c_idx];
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
#ifdef DEBLOCKING_IN_LOOP
                uint8_t *src = &s->dbf_frame->data[c_idx][y * stride + x];
#else
                uint8_t *src = &s->frame->data[c_idx][y * stride + x];
#endif
                uint8_t *dst = &s->sao_frame->data[c_idx][y * stride + x];
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
#else
void ff_hevc_sao_filter(HEVCContext *s, int x0, int y0)
{
    //TODO: This should be easily parallelizable
    //TODO: skip CBs when (cu_transquant_bypass_flag || (pcm_loop_filter_disable_flag && pcm_flag))
    int c_idx;
    int c_idx_min = s->sh.slice_sample_adaptive_offset_flag[0] != 0 ? 0 : 1;
    int c_idx_max = s->sh.slice_sample_adaptive_offset_flag[1] != 0 ? 3 : 1;
    int x_ctb = x0 >> s->sps->log2_ctb_size;
    int y_ctb = y0 >> s->sps->log2_ctb_size;
    int x, y, x_end, y_end;
    x_end = x0+(1<<s->sps->log2_ctb_size);
    if (x_end > s->sps->pic_width_in_luma_samples)
        x_end = s->sps->pic_width_in_luma_samples;
    y_end = y0+(1<<s->sps->log2_ctb_size);
    if (y_end > s->sps->pic_height_in_luma_samples)
        y_end = s->sps->pic_height_in_luma_samples;

    for (y = y0; y < y_end; y++) {
        for (x = x0; x < x_end; x ++) {
            s->sao_frame->data[LUMA][y * s->sao_frame->linesize[LUMA] + x] = s->frame->data[LUMA][y * s->frame->linesize[LUMA] + x];
        }
    }
    for (y = y0/2; y < y_end/2; y++) {
        for (x = x0/2; x < x_end/2; x ++) {
            s->sao_frame->data[CB][y * s->sao_frame->linesize[CB] + x] = s->frame->data[CB][y * s->frame->linesize[CB] + x];
            s->sao_frame->data[CR][y * s->sao_frame->linesize[CR] + x] = s->frame->data[CR][y * s->frame->linesize[CR] + x];
        }
    }

    printf("ff_hevc_sao_filter(s, %d, %d) : (%d, %d)\n", x0, y0, x_ctb, y_ctb);
    for (c_idx = c_idx_min; c_idx < c_idx_max; c_idx++) {
        int stride = s->frame->linesize[c_idx];
        int ctb_size = (1 << (s->sps->log2_ctb_size)) >> s->sps->hshift[c_idx];
        struct SAOParams *sao = &CTB(s->sao, x_ctb, y_ctb);
        int x = x_ctb * ctb_size;
        int y = y_ctb * ctb_size;

        int width = FFMIN(ctb_size,
                                  (s->sps->pic_width_in_luma_samples >> s->sps->hshift[c_idx]) - x);
        int height = FFMIN(ctb_size,
                                    (s->sps->pic_height_in_luma_samples >> s->sps->vshift[c_idx]) - y);
        if (c_idx==0)
        printf("sao(%d -> %d, %d -> %d)\n",x,x+width, y, y+height);
        uint8_t *src = &s->frame->data[c_idx][y * stride + x];
        uint8_t *dst = &s->sao_frame->data[c_idx][y * stride + x];
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
#endif
#undef CTB

static int boundary_strength(HEVCContext *s, MvField *curr, uint8_t curr_cbf_luma, MvField *neigh, uint8_t neigh_cbf_luma, int tu_border)
{
    if (tu_border) {
        if (curr->is_intra || neigh->is_intra)
            return 2;
        if (curr_cbf_luma || neigh_cbf_luma)
            return 1;
    }

    if (s->sh.slice_type == P_SLICE) {
        if (abs(neigh->mv[0].x - curr->mv[0].x) >= 4 || abs(neigh->mv[0].y - curr->mv[0].y) >= 4 ||
            s->ref->refPicList[0].list[neigh->ref_idx[0]] != s->ref->refPicList[0].list[curr->ref_idx[0]])
            return 1;
        else
            return 0;
    } else if (s->sh.slice_type == B_SLICE) {
        int mvs = curr->pred_flag[0] + curr->pred_flag[1];
        if (mvs == neigh->pred_flag[0] + neigh->pred_flag[1]) {
            if (mvs == 2) {
                // same L0 and L1
                if (s->ref->refPicList[0].list[curr->ref_idx[0]] == s->ref->refPicList[0].list[neigh->ref_idx[0]]
                    && s->ref->refPicList[0].list[curr->ref_idx[0]] == s->ref->refPicList[1].list[curr->ref_idx[1]]
                    && s->ref->refPicList[0].list[neigh->ref_idx[0]] == s->ref->refPicList[1].list[neigh->ref_idx[1]]) {
                    if ((abs(neigh->mv[0].x - curr->mv[0].x) >= 4 || abs(neigh->mv[0].y - curr->mv[0].y) >= 4 ||
                        abs(neigh->mv[1].x - curr->mv[1].x) >= 4 || abs(neigh->mv[1].y - curr->mv[1].y) >= 4) &&
                        (abs(neigh->mv[1].x - curr->mv[0].x) >= 4 || abs(neigh->mv[1].y - curr->mv[0].y) >= 4 ||
                        abs(neigh->mv[0].x - curr->mv[1].x) >= 4 || abs(neigh->mv[0].y - curr->mv[1].y) >= 4))
                        return 1;
                    else
                        return 0;
                }
                else if (s->ref->refPicList[0].list[neigh->ref_idx[0]] == s->ref->refPicList[0].list[curr->ref_idx[0]]
                         && s->ref->refPicList[1].list[neigh->ref_idx[1]] == s->ref->refPicList[1].list[curr->ref_idx[1]]) {
                    if (abs(neigh->mv[0].x - curr->mv[0].x) >= 4 || abs(neigh->mv[0].y - curr->mv[0].y) >= 4 ||
                        abs(neigh->mv[1].x - curr->mv[1].x) >= 4 || abs(neigh->mv[1].y - curr->mv[1].y) >= 4)
                        return 1;
                    else
                        return 0;
                }
                else if (s->ref->refPicList[1].list[neigh->ref_idx[1]] == s->ref->refPicList[0].list[curr->ref_idx[0]]
                        && s->ref->refPicList[0].list[neigh->ref_idx[0]] == s->ref->refPicList[1].list[curr->ref_idx[1]]) {
                    if (abs(neigh->mv[1].x - curr->mv[0].x) >= 4 || abs(neigh->mv[1].y - curr->mv[0].y) >= 4 ||
                        abs(neigh->mv[0].x - curr->mv[1].x) >= 4 || abs(neigh->mv[0].y - curr->mv[1].y) >= 4)
                        return 1;
                    else
                        return 0;
                } else {
                    return 1;
                }
            } else { // 1 MV
                Mv A, B;
                int ref_A;
                int ref_B;
                if (curr->pred_flag[0]) {
                    A = curr->mv[0];
                    ref_A = s->ref->refPicList[0].list[curr->ref_idx[0]];
                }
                else {
                    A = curr->mv[1];
                    ref_A = s->ref->refPicList[1].list[curr->ref_idx[1]];
                }
                if (neigh->pred_flag[0]) {
                    B = neigh->mv[0];
                    ref_B = s->ref->refPicList[0].list[neigh->ref_idx[0]];
                } else {
                    B = neigh->mv[1];
                    ref_B = s->ref->refPicList[1].list[neigh->ref_idx[1]];
                }
                if (ref_A == ref_B) {
                    if (abs(A.x - B.x) >= 4 || abs(A.y - B.y) >= 4)
                        return 1;
                    else
                        return 0;
                } else
                    return 1;
            }
        }
        else
            return 1;
    }
    return 0;
}
void ff_hevc_deblocking_boundary_strengths(HEVCContext *s, int x0, int y0, int log2_trafo_size)
{
    int log2_min_pu_size = s->sps->log2_min_pu_size;
    int min_pu_size = 1 << s->sps->log2_min_pu_size;
    int pic_width_in_min_pu = s->sps->pic_width_in_min_cbs * 4;
    int i, j;
    int bs;
    MvField *tab_mvf = s->ref->tab_mvf;
    if ((y0 & 7) == 0) {
        int yp_pu = (y0 - 1) / min_pu_size;
        int yq_pu = y0 >> log2_min_pu_size;
        for (i = 0; i < (1<<log2_trafo_size); i+=4) {
            int x_pu = (x0 + i) >> log2_min_pu_size;
            MvField *top = &tab_mvf[yp_pu * pic_width_in_min_pu + x_pu];
            MvField *curr = &tab_mvf[yq_pu * pic_width_in_min_pu + x_pu];
            uint8_t top_cbf_luma = s->cbf_luma[yp_pu * pic_width_in_min_pu + x_pu];
            uint8_t curr_cbf_luma = s->cbf_luma[yq_pu * pic_width_in_min_pu + x_pu];
            bs = boundary_strength(s, curr, curr_cbf_luma, top, top_cbf_luma, 1);
            if (bs)
                s->horizontal_bs[((x0 + i) + y0 * s->bs_width) >> 2] = bs;
        }
    }
    // bs for TU internal horizontal PU boundaries
    if (log2_trafo_size > s->sps->log2_min_pu_size && s->sh.slice_type != I_SLICE)
        for (j = 8; j < (1<<log2_trafo_size); j += 8) {
            int yp_pu = (y0 + j - 1) >> log2_min_pu_size;
            int yq_pu = (y0 + j) >> log2_min_pu_size;
            for (i = 0; i < (1<<log2_trafo_size); i += 4) {
                int x_pu = (x0 + i) >> log2_min_pu_size;
                MvField *top = &tab_mvf[yp_pu * pic_width_in_min_pu + x_pu];
                MvField *curr = &tab_mvf[yq_pu * pic_width_in_min_pu + x_pu];
                uint8_t top_cbf_luma = s->cbf_luma[yp_pu * pic_width_in_min_pu + x_pu];
                uint8_t curr_cbf_luma = s->cbf_luma[yq_pu * pic_width_in_min_pu + x_pu];
                bs = boundary_strength(s, curr, curr_cbf_luma, top, top_cbf_luma, 0);
                if (bs)
                    s->horizontal_bs[((x0 + i) + (y0 + j) * s->bs_width) >> 2] = bs;
            }
        }
    // bs for vertical TU boundaries
    if ((x0 & 7) == 0) {
        int xp_pu = (x0 - 1) / min_pu_size;
        int xq_pu = x0 >> log2_min_pu_size;
        for (i = 0; i < (1<<log2_trafo_size); i+=4) {
            int y_pu = (y0 + i) >> log2_min_pu_size;
            MvField *left = &tab_mvf[y_pu * pic_width_in_min_pu + xp_pu];
            MvField *curr = &tab_mvf[y_pu * pic_width_in_min_pu + xq_pu];
            uint8_t left_cbf_luma = s->cbf_luma[y_pu * pic_width_in_min_pu + xp_pu];
            uint8_t curr_cbf_luma = s->cbf_luma[y_pu * pic_width_in_min_pu + xq_pu];
            bs = boundary_strength(s, curr, curr_cbf_luma, left, left_cbf_luma, 1);
            if (bs)
                s->vertical_bs[(x0 >> 3) + ((y0 + i) >> 2) * s->bs_width] = bs;
        }
    }
    // bs for TU internal vertical PU boundaries
    if (log2_trafo_size > s->sps->log2_min_pu_size && s->sh.slice_type != I_SLICE)
        for (j = 0; j < (1<<log2_trafo_size); j += 4) {
            int y_pu = (y0 + j) >> log2_min_pu_size;
            for (i = 8; i < (1<<log2_trafo_size); i += 8) {
                int xp_pu = (x0 + i - 1) >> log2_min_pu_size;
                int xq_pu = (x0 + i) >> log2_min_pu_size;
                MvField *left = &tab_mvf[y_pu * pic_width_in_min_pu + xp_pu];
                MvField *curr = &tab_mvf[y_pu * pic_width_in_min_pu + xq_pu];
                uint8_t left_cbf_luma = s->cbf_luma[y_pu * pic_width_in_min_pu + xp_pu];
                uint8_t curr_cbf_luma = s->cbf_luma[y_pu * pic_width_in_min_pu + xq_pu];
                bs = boundary_strength(s, curr, curr_cbf_luma, left, left_cbf_luma, 0);
                if (bs)
                    s->vertical_bs[((x0 + i) >> 3) + ((y0 + j) >> 2) * s->bs_width] = bs;
            }
        }
}
#ifdef DEBLOCKING_IN_LOOP
void ff_hevc_deblocking_filter(HEVCContext *s, int x0, int y0, int log2_ctb_size)
{
    uint8_t *src;
    int x, y, x_end, y_end;
    int pixel = 1 + !!(s->sps->bit_depth - 8); // sizeof(pixel)
    int pic_width_in_min_pu = s->sps->pic_width_in_min_cbs * 4;
    int min_pu_size = 1 << (s->sps->log2_min_pu_size - 1);
    int log2_min_pu_size = s->sps->log2_min_pu_size - 1;

    x_end = x0+(1<<log2_ctb_size) + 8;
    if (x_end > s->sps->pic_width_in_luma_samples)
        x_end = s->sps->pic_width_in_luma_samples;
    y_end = y0+(1<<log2_ctb_size) + 4;
    if (y_end > s->sps->pic_height_in_luma_samples)
        y_end = s->sps->pic_height_in_luma_samples;

    // vertical filtering
    for (x = x0+8; x < x_end; x += 8) {
        for (y = y0==0?0:y0+4; y < y_end; y += 4) {
            int bs = s->vertical_bs[(x >> 3) + (y >> 2) * s->bs_width];
            if (bs) {
                int qp_y = s->qp_y;
                int qp = qp_y + s->sps->qp_bd_offset;
                const int idxb = av_clip_c(qp + s->sh.beta_offset, 0, MAX_QP);
                const int beta = betatable[idxb];
                int no_p = 0;
                int no_q = 0;
                const int idxt = av_clip_c(qp + DEFAULT_INTRA_TC_OFFSET * (bs - 1) + s->sh.tc_offset, 0, MAX_QP + DEFAULT_INTRA_TC_OFFSET);
                const int tc = tctable[idxt];
                if(s->sps->pcm_enabled_flag && s->sps->pcm.loop_filter_disable_flag) {
                    int xp_pu = (x - 1) / min_pu_size;
                    int xq_pu = x >> log2_min_pu_size;
                    int y_pu = y >> log2_min_pu_size;
                    if (s->is_pcm[y_pu * pic_width_in_min_pu + xp_pu])
                        no_p = 1;
                    if (s->is_pcm[y_pu * pic_width_in_min_pu + xq_pu])
                        no_q = 1;
                }
                src = &s->frame->data[LUMA][y * s->frame->linesize[LUMA] + x];
                s->hevcdsp.hevc_loop_filter_luma(src, pixel, s->frame->linesize[LUMA], no_p, no_q, beta, tc);
                if ((x & 15) == 0 && (y & 7) == 0 && bs == 2) {
                    src = &s->frame->data[CB][(y / 2) * s->frame->linesize[CB] + (x / 2)];
                    s->hevcdsp.hevc_loop_filter_chroma(src, pixel, s->frame->linesize[CB], no_p, no_q, chroma_tc(s, qp_y, CB));
                    src = &s->frame->data[CR][(y / 2) * s->frame->linesize[CR] + (x / 2)];
                    s->hevcdsp.hevc_loop_filter_chroma(src, pixel, s->frame->linesize[CR], no_p, no_q, chroma_tc(s, qp_y, CR));
                }
            }
        }
    }
    // horizontal filtering
    if (x_end != s->sps->pic_width_in_luma_samples)
        x_end -= 8;
    if (y_end != s->sps->pic_height_in_luma_samples)
        y_end += 4;
    for (y = y0+8; y < y_end; y += 8) {
        int yp_pu = (y - 1) / min_pu_size;
        int yq_pu = y >> log2_min_pu_size;
        for (x = x0; x < x_end; x += 4) {
            int bs = s->horizontal_bs[(x + y * s->bs_width) >> 2];
            if (bs) {
                int qp_y = s->qp_y;
                int qp = qp_y + s->sps->qp_bd_offset;
                const int idxb = av_clip_c(qp + s->sh.beta_offset, 0, MAX_QP);
                const int beta = betatable[idxb];
                int no_p = 0;
                int no_q = 0;
                const int idxt = av_clip_c(qp + DEFAULT_INTRA_TC_OFFSET * (bs - 1) + s->sh.tc_offset, 0, MAX_QP + DEFAULT_INTRA_TC_OFFSET);
                const int tc = tctable[idxt];
                if(s->sps->pcm_enabled_flag && s->sps->pcm.loop_filter_disable_flag) {
                    int x_pu = x >> log2_min_pu_size;
                    if (s->is_pcm[yp_pu * pic_width_in_min_pu + x_pu])
                        no_p = 1;
                    if (s->is_pcm[yq_pu * pic_width_in_min_pu + x_pu])
                        no_q = 1;
                }
                src = &s->frame->data[LUMA][y * s->frame->linesize[LUMA] + x];
                s->hevcdsp.hevc_loop_filter_luma(src, s->frame->linesize[LUMA], pixel, no_p, no_q, beta, tc);
                if ((x & 7) == 0 && (y & 15) == 0 && bs == 2) {
                    src = &s->frame->data[CB][(y / 2) * s->frame->linesize[CB] + (x / 2)];
                    s->hevcdsp.hevc_loop_filter_chroma(src, s->frame->linesize[CB], pixel, no_p, no_q, chroma_tc(s, qp_y, CB));
                    src = &s->frame->data[CR][(y / 2) * s->frame->linesize[CR] + (x / 2)];
                    s->hevcdsp.hevc_loop_filter_chroma(src, s->frame->linesize[CR], pixel, no_p, no_q, chroma_tc(s, qp_y, CR));
                }
            }
        }
    }
}
#endif
#undef LUMA
#undef CB
#undef CR
