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

static int chroma_tc(HEVCSharedContext *s, int qp_y, int c_idx, int tc_offset)
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

    idxt = av_clip_c(qp + DEFAULT_INTRA_TC_OFFSET + tc_offset, 0, 53);
    return tctable[idxt];
}
static int get_qPy_pred(HEVCContext *s, int xC, int yC, int xBase, int yBase)
{
    HEVCSharedContext *sc = s->HEVCsc;
    HEVCLocalContext *lc = s->HEVClc;
    int Log2CtbSizeY         = sc->sps->log2_ctb_size;
    int MinCuQpDeltaSizeMask     = (1 << (Log2CtbSizeY - sc->pps->diff_cu_qp_delta_depth)) - 1;
    int xQg                  = xC    - ( xC    & MinCuQpDeltaSizeMask );
    int yQg                  = yC    - ( yC    & MinCuQpDeltaSizeMask );
    int xQgBase              = xBase - ( xBase & MinCuQpDeltaSizeMask );
    int yQgBase              = yBase - ( yBase & MinCuQpDeltaSizeMask );
    int log2_min_cb_size     = sc->sps->log2_min_coding_block_size;
    int pic_width            = sc->sps->pic_width_in_luma_samples>>log2_min_cb_size;
    int x                    = xQg >> log2_min_cb_size;
    int y                    = yQg >> log2_min_cb_size;
    int qPy_pred;
    int qPy_a;
    int qPy_b;
    int availableA           = (xQg & ((1<<Log2CtbSizeY)-1)) != 0 && xQg == xQgBase;
    int availableB           = (yQg & ((1<<Log2CtbSizeY)-1)) != 0 && yQg == yQgBase;

    // qPy_pred
    if (lc->isFirstQPgroup != 0) {
        lc->isFirstQPgroup = 0;
        qPy_pred = sc->sh.slice_qp;
    } else
        qPy_pred = lc->qp_y;
        if ( log2_cb_size < sc->sps->log2_ctb_size - sc->pps->diff_cu_qp_delta_depth )
        {
            int offsetX[8][8] = {
                    {-1, 1, 3, 1, 7, 1, 3, 1},
                    { 0, 0, 0, 0, 0, 0, 0, 0},
                    { 1, 3, 1, 3, 1, 3, 1, 3},
                    { 2, 2, 2, 2, 2, 2, 2, 2},
                    { 3, 5, 7, 5, 3, 5, 7, 5},
                    { 4, 4, 4, 4, 4, 4, 4, 4},
                    { 5, 7, 5, 7, 5, 7, 5, 7},
                    { 6, 6, 6, 6, 6, 6, 6, 6}
            };
            int offsetY[8][8] = {
                    { 7, 0, 1, 2, 3, 4, 5, 6},
                    { 0, 1, 2, 3, 4, 5, 6, 7},
                    { 1, 0, 3, 2, 5, 4, 7, 6},
                    { 0, 1, 2, 3, 4, 5, 6, 7},
                    { 3, 0, 1, 2, 7, 4, 5, 6},
                    { 0, 1, 2, 3, 4, 5, 6, 7},
                    { 1, 0, 3, 2, 5, 4, 7, 6},
                    { 0, 1, 2, 3, 4, 5, 6, 7}
            };
            int xC0b = (xC - (xC & ctb_size_mask)) >> sc->sps->log2_min_coding_block_size;
            int yC0b = (yC - (yC & ctb_size_mask)) >> sc->sps->log2_min_coding_block_size;
            int idxX = (xQgBase & ctb_size_mask)>>sc->sps->log2_min_coding_block_size;
            int idxY = (yQgBase & ctb_size_mask)>>sc->sps->log2_min_coding_block_size;
            int idxMask = ctb_size_mask >> sc->sps->log2_min_coding_block_size;
            int x, y;

            x = FFMIN(xC0b+offsetX[idxX][idxY], pic_width - 1);
            y = FFMIN(yC0b+(offsetY[idxX][idxY]&idxMask), pic_height - 1);

            if (xC0b == (lc->start_of_tiles_x >> sc->sps->log2_min_coding_block_size) && offsetX[idxX][idxY] == -1) {
                x = (lc->end_of_tiles_x >> sc->sps->log2_min_coding_block_size) - 1;
                y = yC0b - 1;
            }
            qPy_pred = sc->qp_y_tab[y * pic_width + x];
        }
    }
    // qPy_a
    if (availableA == 0)
        qPy_a = qPy_pred;
    else
        qPy_a = sc->qp_y_tab[(x-1) + y * pic_width];

    // qPy_b
    if (availableB == 0)
        qPy_b = qPy_pred;
    else
        qPy_b = sc->qp_y_tab[x + (y-1) * pic_width];
    return (qPy_a + qPy_b + 1) >> 1;
}
void ff_hevc_set_qPy(HEVCContext *s, int xC, int yC, int xBase, int yBase)
{
    if (s->HEVClc->tu.cu_qp_delta != 0)
        s->HEVClc->qp_y = ((get_qPy_pred(s, xC, yC, xBase, yBase) + s->HEVClc->tu.cu_qp_delta + 52 + 2 * s->HEVCsc->sps->qp_bd_offset) %
                (52 + s->HEVCsc->sps->qp_bd_offset)) - s->HEVCsc->sps->qp_bd_offset;
    else
        s->HEVClc->qp_y = get_qPy_pred(s, xC, yC, xBase, yBase);
}
static int get_qPy(HEVCContext *s, int xC, int yC)
{
    HEVCSharedContext *sc = s->HEVCsc;
    int log2_min_cb_size = sc->sps->log2_min_coding_block_size;
    int pic_width        = sc->sps->pic_width_in_luma_samples>>log2_min_cb_size;
    int x                = xC >> log2_min_cb_size;
    int y                = yC >> log2_min_cb_size;
    return sc->qp_y_tab[x + y * pic_width];
}

static void copy_CTB(uint8_t *dst, uint8_t *src, int width, int height, int stride){
    int i;
    
    for(i=0; i< height; i++){
        memcpy(dst, src, width);
        dst += stride;
        src += stride;
    }
}

#define CTB(tab, x, y) ((tab)[(y) * sc->sps->pic_width_in_ctbs + (x)])

void ff_hevc_sao_filter_CTB(HEVCSharedContext *sc, int x, int y, int c_idx_min, int c_idx_max)
{
    //  TODO: This should be easily parallelizable
    //  TODO: skip CBs when (cu_transquant_bypass_flag || (pcm_loop_filter_disable_flag && pcm_flag))
    
    int c_idx = 0;
    int class=1, class_index;
    int  edges[4]; // 0 left 1 top 2 right 3 bottom
    SAOParams *sao[4];
    int classes[4];
    int x_shift = 0, y_shift = 0;
    int x_ctb = x>>sc->sps->log2_ctb_size;
    int y_ctb = y>>sc->sps->log2_ctb_size;
    sao[0]= &CTB(sc->sao, x_ctb, y_ctb);
    edges[0]   =  x_ctb == 0;
    edges[1]   =  y_ctb == 0;
    edges[2]   =  x_ctb == (sc->sps->pic_width_in_ctbs - 1);
    edges[3]   =  y_ctb == (sc->sps->pic_height_in_ctbs - 1);
    classes[0] = 0;
    
    if(!edges[0]) {
        sao[class]= &CTB(sc->sao, x_ctb-1, y_ctb);
        classes[class] = 2;
        class++;
        x_shift = 8;
    }
    if(!edges[1]) {
        sao[class]= &CTB(sc->sao, x_ctb, y_ctb-1);
        classes[class] = 1;
        class++;
        y_shift = 4;
        if(!edges[0]) {
            classes[class] = 3;
            sao[class]= &CTB(sc->sao, x_ctb-1, y_ctb-1);
            class++;
        }
    }
    for(c_idx=0; c_idx<3; c_idx++)  {
        int chroma = c_idx ? 1 : c_idx;
        int x0 = x>>chroma;
        int y0 = y>>chroma;
        int stride = sc->frame->linesize[c_idx];
        int ctb_size = (1 << (sc->sps->log2_ctb_size)) >> sc->sps->hshift[c_idx];
        int width = FFMIN(ctb_size,
                          (sc->sps->pic_width_in_luma_samples >> sc->sps->hshift[c_idx]) - x0);
        int height = FFMIN(ctb_size,
                           (sc->sps->pic_height_in_luma_samples >> sc->sps->vshift[c_idx]) - y0);
        
        uint8_t *src = &sc->frame->data[c_idx][y0 * stride + x0];
        uint8_t *dst = &sc->sao_frame->data[c_idx][y0 * stride + x0];
        int offset = (y_shift>>chroma) * stride + (x_shift>>chroma);
        
        copy_CTB(dst-offset, src-offset, edges[2] ? (width+(x_shift>>chroma)):width , edges[3] ? height+(y_shift>>chroma):height, stride );
        for(class_index = 0; class_index < class && c_idx>=c_idx_min && c_idx<c_idx_max; class_index++)    {
            switch (sao[class_index]->type_idx[c_idx]) {
                case SAO_BAND:
                    sc->hevcdsp.sao_band_filter(dst, src, stride,  sao[class_index], edges, width, height, c_idx, classes[class_index]);
                    
                    break;
                case SAO_EDGE: {
                    sc->hevcdsp.sao_edge_filter(dst, src, stride, sao[class_index],  edges, width, height, c_idx, classes[class_index]);
                    break;
                }
            }
        }
    }
}

void ff_hevc_deblocking_filter_CTB(HEVCContext *s, int x0, int y0)
{
    uint8_t *src;
    int x, y;
    HEVCSharedContext *sc = s->HEVCsc;
    int pixel = 1 + !!(sc->sps->bit_depth - 8); // sizeof(pixel)

    int pic_width_in_min_pu = sc->sps->pic_width_in_min_cbs * 4;
    int min_pu_size = 1 << sc->sps->log2_min_pu_size;
    int log2_min_pu_size = sc->sps->log2_min_pu_size;
    int log2_ctb_size =  sc->sps->log2_ctb_size;
    int x_end, y_end;
    int ctb_size = 1<<log2_ctb_size;
    int ctb = (x0 >> log2_ctb_size) + (y0 >> log2_ctb_size) * sc->sps->pic_width_in_ctbs;
    int tc_offset = sc->deblock[ctb].tc_offset;
    int beta_offset = sc->deblock[ctb].beta_offset;
    if (sc->deblock[ctb].disable)
        return;

    x_end = x0+ctb_size;
    if (x_end > sc->sps->pic_width_in_luma_samples)
        x_end = sc->sps->pic_width_in_luma_samples;
    y_end = y0+ctb_size ;
    if (y_end > sc->sps->pic_height_in_luma_samples)
        y_end = sc->sps->pic_height_in_luma_samples;

    // vertical filtering
    for (x = x0 ? x0:8; x < x_end; x += 8) {
        for (y = y0; y < y_end; y += 4) {
            int bs = sc->vertical_bs[(x >> 3) + (y >> 2) * sc->bs_width];
            if (bs) {
                int qp = (get_qPy(s, x - 1, y) + get_qPy(s, x, y) + 1) >> 1;
                const int idxb = av_clip_c(qp + ((beta_offset >> 1) << 1), 0, MAX_QP);
                const int beta = betatable[idxb];
                int no_p = 0;
                int no_q = 0;
                const int idxt = av_clip_c(qp + DEFAULT_INTRA_TC_OFFSET * (bs - 1) + ((tc_offset >> 1) << 1), 0, MAX_QP + DEFAULT_INTRA_TC_OFFSET);
                const int tc = tctable[idxt];
                if((sc->sps->pcm_enabled_flag && sc->sps->pcm.loop_filter_disable_flag) || sc->pps->transquant_bypass_enable_flag) {
                    int y_pu = y >> log2_min_pu_size;
                    int xp_pu = (x - 1) / min_pu_size;
                    int xq_pu = x >> log2_min_pu_size;
                    if (sc->is_pcm[y_pu * pic_width_in_min_pu + xp_pu])
                        no_p = 1;
                    if (sc->is_pcm[y_pu * pic_width_in_min_pu + xq_pu])
                        no_q = 1;
                }
                src = &sc->frame->data[LUMA][y * sc->frame->linesize[LUMA] + x];
                sc->hevcdsp.hevc_loop_filter_luma(src, pixel, sc->frame->linesize[LUMA], no_p, no_q, beta, tc);

                if ((x & 15) == 0 && (y & 7) == 0 && bs == 2) {
                    src = &sc->frame->data[CB][(y / 2) * sc->frame->linesize[CB] + (x / 2)];
                    sc->hevcdsp.hevc_loop_filter_chroma(src, pixel, sc->frame->linesize[CB], no_p, no_q, chroma_tc(sc, qp, CB, tc_offset));
                    src = &sc->frame->data[CR][(y / 2) * sc->frame->linesize[CR] + (x / 2)];
                    sc->hevcdsp.hevc_loop_filter_chroma(src, pixel, sc->frame->linesize[CR], no_p, no_q, chroma_tc(sc, qp, CR, tc_offset));
                }
            }
        }
    }
    // horizontal filtering
    if (x_end != sc->sps->pic_width_in_luma_samples)
        x_end -= 8;
    for (y = y0 ? y0:8; y < y_end; y += 8) {
        for (x = x0 ? x0-8:0; x < x_end; x += 4) {
            int bs = sc->horizontal_bs[(x + y * sc->bs_width) >> 2];
            if (bs) {
                int qp = (get_qPy(s, x, y - 1) + get_qPy(s, x, y) + 1) >> 1;
                const int idxb = av_clip_c(qp + ((beta_offset >> 1) << 1), 0, MAX_QP);
                const int beta = betatable[idxb];
                int no_p = 0;
                int no_q = 0;
                const int idxt = av_clip_c(qp + DEFAULT_INTRA_TC_OFFSET * (bs - 1) + ((tc_offset >> 1) << 1), 0, MAX_QP + DEFAULT_INTRA_TC_OFFSET);
                const int tc = tctable[idxt];
                if((sc->sps->pcm_enabled_flag && sc->sps->pcm.loop_filter_disable_flag) || sc->pps->transquant_bypass_enable_flag) {
                    int yp_pu = (y - 1) / min_pu_size;
                    int yq_pu = y >> log2_min_pu_size;
                    int x_pu = x >> log2_min_pu_size;
                    if (sc->is_pcm[yp_pu * pic_width_in_min_pu + x_pu])
                        no_p = 1;
                    if (sc->is_pcm[yq_pu * pic_width_in_min_pu + x_pu])
                        no_q = 1;
                }
                src = &sc->frame->data[LUMA][y * sc->frame->linesize[LUMA] + x];
                sc->hevcdsp.hevc_loop_filter_luma(src, sc->frame->linesize[LUMA], pixel, no_p, no_q, beta, tc);
                if ((x & 7) == 0 && (y & 15) == 0 && bs == 2) {
                    src = &sc->frame->data[CB][(y / 2) * sc->frame->linesize[CB] + (x / 2)];
                    sc->hevcdsp.hevc_loop_filter_chroma(src, sc->frame->linesize[CB], pixel, no_p, no_q, chroma_tc(sc, qp, CB, tc_offset));
                    src = &sc->frame->data[CR][(y / 2) * sc->frame->linesize[CR] + (x / 2)];
                    sc->hevcdsp.hevc_loop_filter_chroma(src, sc->frame->linesize[CR], pixel, no_p, no_q, chroma_tc(sc, qp, CR, tc_offset));
                }
            }
        }
    }
}


static int boundary_strength(HEVCSharedContext *sc, MvField *curr, uint8_t curr_cbf_luma, MvField *neigh, uint8_t neigh_cbf_luma, int tu_border)
{
    if (tu_border) {
        if (curr->is_intra || neigh->is_intra)
            return 2;
        if (curr_cbf_luma || neigh_cbf_luma)
            return 1;
    }

    if (sc->sh.slice_type == P_SLICE) {
        if (abs(neigh->mv[0].x - curr->mv[0].x) >= 4 || abs(neigh->mv[0].y - curr->mv[0].y) >= 4 ||
            sc->ref->refPicList[0].list[neigh->ref_idx[0]] != sc->ref->refPicList[0].list[curr->ref_idx[0]])
            return 1;
        else
            return 0;
    } else if (sc->sh.slice_type == B_SLICE) {
        int mvs = curr->pred_flag[0] + curr->pred_flag[1];
        if (mvs == neigh->pred_flag[0] + neigh->pred_flag[1]) {
            if (mvs == 2) {
                // same L0 and L1
                if (sc->ref->refPicList[0].list[curr->ref_idx[0]] == sc->ref->refPicList[0].list[neigh->ref_idx[0]]
                    && sc->ref->refPicList[0].list[curr->ref_idx[0]] == sc->ref->refPicList[1].list[curr->ref_idx[1]]
                    && sc->ref->refPicList[0].list[neigh->ref_idx[0]] == sc->ref->refPicList[1].list[neigh->ref_idx[1]]) {
                    if ((abs(neigh->mv[0].x - curr->mv[0].x) >= 4 || abs(neigh->mv[0].y - curr->mv[0].y) >= 4 ||
                        abs(neigh->mv[1].x - curr->mv[1].x) >= 4 || abs(neigh->mv[1].y - curr->mv[1].y) >= 4) &&
                        (abs(neigh->mv[1].x - curr->mv[0].x) >= 4 || abs(neigh->mv[1].y - curr->mv[0].y) >= 4 ||
                        abs(neigh->mv[0].x - curr->mv[1].x) >= 4 || abs(neigh->mv[0].y - curr->mv[1].y) >= 4))
                        return 1;
                    else
                        return 0;
                }
                else if (sc->ref->refPicList[0].list[neigh->ref_idx[0]] == sc->ref->refPicList[0].list[curr->ref_idx[0]]
                         && sc->ref->refPicList[1].list[neigh->ref_idx[1]] == sc->ref->refPicList[1].list[curr->ref_idx[1]]) {
                    if (abs(neigh->mv[0].x - curr->mv[0].x) >= 4 || abs(neigh->mv[0].y - curr->mv[0].y) >= 4 ||
                        abs(neigh->mv[1].x - curr->mv[1].x) >= 4 || abs(neigh->mv[1].y - curr->mv[1].y) >= 4)
                        return 1;
                    else
                        return 0;
                }
                else if (sc->ref->refPicList[1].list[neigh->ref_idx[1]] == sc->ref->refPicList[0].list[curr->ref_idx[0]]
                        && sc->ref->refPicList[0].list[neigh->ref_idx[0]] == sc->ref->refPicList[1].list[curr->ref_idx[1]]) {
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
                    ref_A = sc->ref->refPicList[0].list[curr->ref_idx[0]];
                }
                else {
                    A = curr->mv[1];
                    ref_A = sc->ref->refPicList[1].list[curr->ref_idx[1]];
                }
                if (neigh->pred_flag[0]) {
                    B = neigh->mv[0];
                    ref_B = sc->ref->refPicList[0].list[neigh->ref_idx[0]];
                } else {
                    B = neigh->mv[1];
                    ref_B = sc->ref->refPicList[1].list[neigh->ref_idx[1]];
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
    HEVCSharedContext *sc = s->HEVCsc; 
    int log2_min_pu_size = sc->sps->log2_min_pu_size;
    int min_pu_size = 1 << sc->sps->log2_min_pu_size;
    int pic_width_in_min_pu = sc->sps->pic_width_in_min_cbs * 4;
    int i, j;
    int bs;
    MvField *tab_mvf = sc->ref->tab_mvf;
    if ((y0 & 7) == 0) {
        int yp_pu = (y0 - 1) / min_pu_size;
        int yq_pu = y0 >> log2_min_pu_size;
        for (i = 0; i < (1<<log2_trafo_size); i+=4) {
            int x_pu = (x0 + i) >> log2_min_pu_size;
            MvField *top = &tab_mvf[yp_pu * pic_width_in_min_pu + x_pu];
            MvField *curr = &tab_mvf[yq_pu * pic_width_in_min_pu + x_pu];
            uint8_t top_cbf_luma = sc->cbf_luma[yp_pu * pic_width_in_min_pu + x_pu];
            uint8_t curr_cbf_luma = sc->cbf_luma[yq_pu * pic_width_in_min_pu + x_pu];
            bs = boundary_strength(sc, curr, curr_cbf_luma, top, top_cbf_luma, 1);
            if (sc->sh.slice_loop_filter_across_slices_enabled_flag == 0 && (y0 % (1 << sc->sps->log2_ctb_size)) == 0 && !s->HEVClc->ctb_up_flag)

                bs = 0;
            if (sc->sh.disable_deblocking_filter_flag == 1)
                bs = 0;
            if (bs)
                sc->horizontal_bs[((x0 + i) + y0 * sc->bs_width) >> 2] = bs;
        }
    }
    // bs for TU internal horizontal PU boundaries
    if (log2_trafo_size > sc->sps->log2_min_pu_size && sc->sh.slice_type != I_SLICE)
        for (j = 8; j < (1<<log2_trafo_size); j += 8) {
            int yp_pu = (y0 + j - 1) >> log2_min_pu_size;
            int yq_pu = (y0 + j) >> log2_min_pu_size;
            for (i = 0; i < (1<<log2_trafo_size); i += 4) {
                int x_pu = (x0 + i) >> log2_min_pu_size;
                MvField *top = &tab_mvf[yp_pu * pic_width_in_min_pu + x_pu];
                MvField *curr = &tab_mvf[yq_pu * pic_width_in_min_pu + x_pu];
                uint8_t top_cbf_luma = sc->cbf_luma[yp_pu * pic_width_in_min_pu + x_pu];
                uint8_t curr_cbf_luma = sc->cbf_luma[yq_pu * pic_width_in_min_pu + x_pu];
                bs = boundary_strength(sc, curr, curr_cbf_luma, top, top_cbf_luma, 0);
                if (sc->sh.disable_deblocking_filter_flag == 1)
                    bs = 0;
                if (bs)
                    sc->horizontal_bs[((x0 + i) + (y0 + j) * sc->bs_width) >> 2] = bs;
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

            uint8_t left_cbf_luma = sc->cbf_luma[y_pu * pic_width_in_min_pu + xp_pu];
            uint8_t curr_cbf_luma = sc->cbf_luma[y_pu * pic_width_in_min_pu + xq_pu];
            bs = boundary_strength(sc, curr, curr_cbf_luma, left, left_cbf_luma, 1);
            if (sc->sh.slice_loop_filter_across_slices_enabled_flag == 0 && (x0 % (1 << sc->sps->log2_ctb_size)) == 0 && !s->HEVClc->ctb_left_flag)

                bs = 0;
            if (sc->sh.disable_deblocking_filter_flag == 1)
                bs = 0;
            if (bs)
                sc->vertical_bs[(x0 >> 3) + ((y0 + i) >> 2) * sc->bs_width] = bs;
        }
    }
    // bs for TU internal vertical PU boundaries
    if (log2_trafo_size > sc->sps->log2_min_pu_size && sc->sh.slice_type != I_SLICE)
        for (j = 0; j < (1<<log2_trafo_size); j += 4) {
            int y_pu = (y0 + j) >> log2_min_pu_size;
            for (i = 8; i < (1<<log2_trafo_size); i += 8) {
                int xp_pu = (x0 + i - 1) >> log2_min_pu_size;
                int xq_pu = (x0 + i) >> log2_min_pu_size;
                MvField *left = &tab_mvf[y_pu * pic_width_in_min_pu + xp_pu];
                MvField *curr = &tab_mvf[y_pu * pic_width_in_min_pu + xq_pu];
                uint8_t left_cbf_luma = sc->cbf_luma[y_pu * pic_width_in_min_pu + xp_pu];
                uint8_t curr_cbf_luma = sc->cbf_luma[y_pu * pic_width_in_min_pu + xq_pu];
                bs = boundary_strength(sc, curr, curr_cbf_luma, left, left_cbf_luma, 0);
                if (sc->sh.disable_deblocking_filter_flag == 1)
                    bs = 0;
                if (bs)
                    sc->vertical_bs[((x0 + i) >> 3) + ((y0 + j) >> 2) * sc->bs_width] = bs;
            }
        }
}
#undef LUMA
#undef CB
#undef CR

void hls_filter(HEVCContext *s, int x, int y)
{
    int c_idx_min = s->HEVCsc->sh.slice_sample_adaptive_offset_flag[0] != 0 ? 0 : 1;
    int c_idx_max = s->HEVCsc->sh.slice_sample_adaptive_offset_flag[1] != 0 ? 3 : 1;
    ff_hevc_deblocking_filter_CTB(s, x, y);
    if(s->HEVCsc->sps->sample_adaptive_offset_enabled_flag)
        ff_hevc_sao_filter_CTB(s->HEVCsc, x, y, c_idx_min, c_idx_max);
}
void hls_filters(HEVCContext *s, int x_ctb, int y_ctb, int ctb_size)
{
    if(y_ctb && x_ctb) {
        hls_filter(s, x_ctb-ctb_size, y_ctb-ctb_size);
        if(x_ctb >= (s->HEVCsc->sps->pic_width_in_luma_samples - ctb_size))
            hls_filter(s, x_ctb, y_ctb-ctb_size);
        if(y_ctb >= (s->HEVCsc->sps->pic_height_in_luma_samples - ctb_size))
            hls_filter(s, x_ctb-ctb_size, y_ctb);
    }
}
