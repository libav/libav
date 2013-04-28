/*
 * HEVC video Decoder
 *
 * Copyright (C) 2012 Guillaume Martres
 * Copyright (C) 2013 Anand Meher Kotra
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
#include "hevcdata.h"

/*
 * 6.4.1 Derivation process for z-scan order block availability
 */
int ff_hevc_z_scan_block_avail(HEVCContext *s, int xCurr, int yCurr, int xN, int yN)
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
        availableN = ff_hevc_z_scan_block_avail(s, x0, y0, xA1, yA1);
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
    if(A.pred_flag[0] && A.pred_flag[1] && B.pred_flag[0] && B.pred_flag[1])
        return ((A.ref_idx[0] == B.ref_idx[0])  && (A.mv[0].x == B.mv[0].x) && (A.mv[0].y == B.mv[0].y) &&
                (A.ref_idx[1] == B.ref_idx[1]) && (A.mv[1].x == B.mv[1].x) && (A.mv[1].y == B.mv[1].y));
    else if (A.pred_flag[0] && !A.pred_flag[1] && B.pred_flag[0] && !B.pred_flag[1])
        return ((A.ref_idx[0] == B.ref_idx[0])  && (A.mv[0].x == B.mv[0].x) && (A.mv[0].y == B.mv[0].y));
    else if (!A.pred_flag[0] && A.pred_flag[1] && !B.pred_flag[0] && B.pred_flag[1])
        return ((A.ref_idx[1] == B.ref_idx[1]) && (A.mv[1].x == B.mv[1].x) && (A.mv[1].y == B.mv[1].y));
    else
        return 0;
}

static int DiffPicOrderCnt(int A, int B)
{
    return A-B;
}

// derive the motion vectors section 8.5.3.1.8
static int derive_temporal_colocated_mvs(HEVCContext *s, MvField temp_col, int refIdxLx, Mv* mvLXCol, int X, int colPic, RefPicList refPicList_col[])
{
    int availableFlagLXCol = 0;
    Mv mvCol;
    int listCol;
    int refidxCol;
    int check_mvset =0;
    RefPicList  *refPicList =  s->ref->refPicList;


    if(temp_col.is_intra) {
        mvLXCol->x = 0;
        mvLXCol->y = 0;
        availableFlagLXCol = 0;
    } else {
        if(temp_col.pred_flag[0] == 0) {
            mvCol = temp_col.mv[1];
            refidxCol = temp_col.ref_idx[1];
            listCol = L1;
            check_mvset =1;
        } else if((temp_col.pred_flag[0] == 1) && (temp_col.pred_flag[1] == 0)) {
            mvCol = temp_col.mv[0];
            refidxCol = temp_col.ref_idx[0];
            listCol = L0;
            check_mvset = 1;
        } else if((temp_col.pred_flag[0] == 1) && (temp_col.pred_flag[1] == 1)) {
            int check_diffpicount = 0;
            int i = 0;
            for (i = 0; i < refPicList[0].numPic; i++) {
                if(DiffPicOrderCnt(refPicList[0].list[i], s->poc) > 0)
                    check_diffpicount++;
            }
            for (i = 0; i < refPicList[1].numPic; i++) {
                if(DiffPicOrderCnt(refPicList[1].list[i], s->poc) > 0)
                    check_diffpicount++;
            }
            if ((check_diffpicount == 0) && (X == 0)) {
                mvCol = temp_col.mv[0];
                refidxCol = temp_col.ref_idx[0];
                listCol = L0;
            } else if ((check_diffpicount == 0) && (X == 1)) {
                mvCol = temp_col.mv[1];
                refidxCol = temp_col.ref_idx[1];
                listCol = L1;
            } else {
                if(s->sh.collocated_from_l0_flag == 0) {
                    mvCol = temp_col.mv[0];
                    refidxCol = temp_col.ref_idx[0];
                    listCol = L0;
                } else {
                    mvCol = temp_col.mv[1];
                    refidxCol = temp_col.ref_idx[1];
                    listCol = L1;
                }
            }
            check_mvset = 1;
        }
        // Assuming no long term pictures in version 1 of the decoder
        if(check_mvset == 1) {
            int colPocDiff = DiffPicOrderCnt(colPic, refPicList_col[listCol].list[refidxCol]);
            int curPocDiff = DiffPicOrderCnt(s->poc, refPicList[X].list[refIdxLx]);
            availableFlagLXCol = 1;
            if (colPocDiff == curPocDiff) {
                mvLXCol->x = mvCol.x;
                mvLXCol->y = mvCol.y;
            } else {
                int td = av_clip_c(colPocDiff, -128, 127);
                int tb = av_clip_c(curPocDiff, -128, 127);
                int tx = (0x4000 + abs(td/2)) / td;
                int distScaleFactor = av_clip_c((tb * tx + 32) >> 6,
                                                -4096, 4095);
                mvLXCol->x = av_clip_c((distScaleFactor * mvCol.x + 127 + (distScaleFactor * mvCol.x < 0)) >> 8,
                                       -32768, 32767);
                mvLXCol->y = av_clip_c((distScaleFactor * mvCol.y + 127 + (distScaleFactor * mvCol.y < 0)) >> 8,
                                       -32768, 32767);
            }
        }
    }
    return availableFlagLXCol;
}
/*
 * 8.5.3.1.7  temporal luma motion vector prediction
 */
static int temporal_luma_motion_vector(HEVCContext *s, int x0, int y0, int nPbW, int nPbH, int refIdxLx, Mv* mvLXCol, int X)
{
    MvField *coloc_tab_mvf = NULL;
    MvField temp_col;
    RefPicList  *refPicList =  s->ref->refPicList;
    int xPRb, yPRb;
    int xPRb_pu;
    int yPRb_pu;
    int xPCtr, yPCtr;
    int xPCtr_pu;
    int yPCtr_pu;
    int pic_width_in_min_pu  = s->sps->pic_width_in_min_cbs * 4;
    int short_ref_idx = 0;
    int availableFlagLXCol = 0;
    int colPic;

    if((s->sh.slice_type == B_SLICE) && (s->sh.collocated_from_l0_flag == 0)) {
        short_ref_idx = refPicList[1].idx[s->sh.collocated_ref_idx];
        coloc_tab_mvf = s->short_refs[short_ref_idx].tab_mvf;
        colPic = s->short_refs[short_ref_idx].poc;
    }
    else if(((s->sh.slice_type == B_SLICE) && (s->sh.collocated_from_l0_flag == 1))
            || (s->sh.slice_type == P_SLICE)) {
        short_ref_idx = refPicList[0].idx[s->sh.collocated_ref_idx];
        coloc_tab_mvf = s->short_refs[short_ref_idx].tab_mvf;
        colPic = s->short_refs[short_ref_idx].poc;
    }
    //bottom right collocated motion vector
    xPRb = x0 + nPbW;
    yPRb = y0 + nPbH;
    if (((y0 >> s->sps->log2_ctb_size) == (yPRb >> s->sps->log2_ctb_size))
        && (yPRb < s->sps->pic_height_in_luma_samples) && (xPRb < s->sps->pic_width_in_luma_samples)) {
        xPRb = ((xPRb >> 4) << 4);
        yPRb = ((yPRb >> 4) << 4);
        xPRb_pu = xPRb >> s->sps->log2_min_pu_size;
        yPRb_pu = yPRb >> s->sps->log2_min_pu_size;
        temp_col = coloc_tab_mvf[(yPRb_pu) * pic_width_in_min_pu + xPRb_pu];
        availableFlagLXCol = derive_temporal_colocated_mvs(s, temp_col, refIdxLx, mvLXCol, X, colPic, s->short_refs[short_ref_idx].refPicList);
    } else {
        mvLXCol->x = 0;
        mvLXCol->y = 0;
        availableFlagLXCol = 0;
    }

    // derive center collocated motion vector
    if (availableFlagLXCol == 0) {
        xPCtr = x0 + (nPbW >> 1);
        yPCtr = y0 + (nPbH >> 1);
        xPCtr = ((xPCtr >> 4) << 4);
        yPCtr = ((yPCtr >> 4) << 4);
        xPCtr_pu = xPCtr >> s->sps->log2_min_pu_size;
        yPCtr_pu = yPCtr >> s->sps->log2_min_pu_size;
        temp_col = coloc_tab_mvf[(yPCtr_pu) * pic_width_in_min_pu + xPCtr_pu];
        availableFlagLXCol = derive_temporal_colocated_mvs(s, temp_col, refIdxLx, mvLXCol, X, colPic, s->short_refs[short_ref_idx].refPicList);
    }
    return availableFlagLXCol;
}
/*
 * 8.5.3.1.2  Derivation process for spatial merging candidates
 */
static void derive_spatial_merge_candidates(HEVCContext *s, int x0, int y0, int nPbW, int nPbH, int log2_cb_size, int singleMCLFlag, int part_idx,  struct MvField mergecandlist[])
{

    RefPicList  *refPicList =  s->ref->refPicList;
    MvField *tab_mvf = s->ref->tab_mvf;
    int available_a1_flag=0;
    int available_b1_flag=0;
    int available_b0_flag=0;
    int available_a0_flag=0;
    int available_b2_flag=0;
    struct MvField spatialCMVS[MRG_MAX_NUM_CANDS];
    struct MvField l0Cand   = {{{ 0 }}};
    struct MvField l1Cand   = {{{ 0 }}};
    struct MvField combCand = {{{ 0 }}};
    struct MvField TMVPCand = {{{ 0 }}};
    struct Mv  mvL0Col = {0};
    struct Mv  mvL1Col = {0};

    //first left spatial merge candidate
    int xA1 = x0 - 1;
    int yA1 = y0 + nPbH - 1;
    int isAvailableA1 =0;
    int check_A1 = check_prediction_block_available (s,log2_cb_size, x0, y0, nPbW, nPbH, xA1, yA1, part_idx);
    int pic_width_in_min_pu  = s->sps->pic_width_in_min_cbs * 4;
    int check_MER = 1;
    int check_MER_1 =1;

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
    int mergearray_index = 0;

    struct MvField zerovector;
    int numRefIdx;
    int zeroIdx = 0;

    int numMergeCand =0;
    int numOrigMergeCand = 0;
    int numInputMergeCand = 0;
    int sumcandidates = 0;
    int combIdx  = 0;
    int combStop = 0;
    int l0CandIdx = 0;
    int l1CandIdx = 0;

    int refIdxL0Col = 0;
    int refIdxL1Col = 0;
    int availableFlagLXCol = 0;
    int availableFlagL0Col = 0;
    int availableFlagL1Col = 0;

    int xA1_pu = xA1 >> s->sps->log2_min_pu_size;
    int yA1_pu = yA1 >> s->sps->log2_min_pu_size;

    if((xA1_pu >= 0) && !(tab_mvf[(yA1_pu) * pic_width_in_min_pu + xA1_pu].is_intra) && check_A1) {
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
        spatialCMVS[0] = tab_mvf[(yA1_pu) * pic_width_in_min_pu + xA1_pu];
    } else {
        available_a1_flag = 0;
        spatialCMVS[0].ref_idx[0] = -1;
        spatialCMVS[0].ref_idx[1] = -1;
        spatialCMVS[0].mv[0].x = 0;
        spatialCMVS[0].mv[0].y = 0;
        spatialCMVS[0].mv[1].x = 0;
        spatialCMVS[0].mv[1].y = 0;
        spatialCMVS[0].pred_flag[0] = 0;
        spatialCMVS[0].pred_flag[1] = 0;
        spatialCMVS[0].is_intra = 0;
    }

    // above spatial merge candidate

    xB1 = x0 + nPbW - 1;
    yB1 = y0 - 1;
    xB1_pu = xB1 >> s->sps->log2_min_pu_size;
    yB1_pu = yB1 >> s->sps->log2_min_pu_size;

    is_available_b1 = 0;
    check_B1 = check_prediction_block_available(s, log2_cb_size, x0, y0, nPbW, nPbH, xB1, yB1, part_idx);

    if((yB1_pu >= 0) && !(tab_mvf[(yB1_pu) * pic_width_in_min_pu + xB1_pu].is_intra) && check_B1) {
        is_available_b1 = 1;
    } else {
        is_available_b1 = 0;
    }

    if((singleMCLFlag == 0) && (part_idx == 1) &&
       ((s->cu.part_mode == PART_2NxN) || (s->cu.part_mode == PART_2NxnU) || (s->cu.part_mode == PART_2NxnD))
       || isDiffMER(s, xB1, yB1, x0, y0)) {
        is_available_b1 = 0;
    }
    if (isAvailableA1 && is_available_b1) {
        check_MER = !(compareMVrefidx(tab_mvf[(yB1_pu) * pic_width_in_min_pu + xB1_pu], tab_mvf[(yA1_pu) * pic_width_in_min_pu + xA1_pu]));
    }

    if (is_available_b1 && check_MER) {
        available_b1_flag = 1;
        spatialCMVS[1] = tab_mvf[(yB1_pu) * pic_width_in_min_pu + xB1_pu];
    } else {
        available_b1_flag = 0;
        spatialCMVS[1].ref_idx[0] = -1;
        spatialCMVS[1].ref_idx[1] = -1;
        spatialCMVS[1].mv[0].x = 0;
        spatialCMVS[1].mv[0].y = 0;
        spatialCMVS[1].mv[1].x = 0;
        spatialCMVS[1].mv[1].y = 0;
        spatialCMVS[1].pred_flag[0] = 0;
        spatialCMVS[1].pred_flag[1] =0;
        spatialCMVS[1].is_intra = 0;
    }

    // above right spatial merge candidate
    xB0 = x0 + nPbW;
    yB0 = y0 - 1;
    check_MER = 1;
    xB0_pu = xB0 >> s->sps->log2_min_pu_size;
    yB0_pu = yB0 >> s->sps->log2_min_pu_size;
    isAvailableB0 = 0;
    check_B0 = check_prediction_block_available(s, log2_cb_size, x0, y0, nPbW, nPbH, xB0, yB0, part_idx);
    if((yB0_pu >= 0) && !(tab_mvf[(yB0_pu) * pic_width_in_min_pu + xB0_pu].is_intra) && check_B0) {
        isAvailableB0 = 1;
    } else {
        isAvailableB0 = 0;
    }

    if((isDiffMER(s, xB0, yB0, x0, y0))) {
        isAvailableB0 = 0;
    }

    if (is_available_b1 && isAvailableB0) {
        check_MER = !(compareMVrefidx(tab_mvf[(yB0_pu) * pic_width_in_min_pu + xB0_pu], tab_mvf[(yB1_pu) * pic_width_in_min_pu + xB1_pu]));
    }

    if (isAvailableB0 && check_MER) {
        available_b0_flag = 1;
        spatialCMVS[2] = tab_mvf[(yB0_pu) * pic_width_in_min_pu + xB0_pu];
    } else {
        available_b0_flag = 0;
        spatialCMVS[2].ref_idx[0] = -1;
        spatialCMVS[2].ref_idx[1] = -1;
        spatialCMVS[2].mv[0].x = 0;
        spatialCMVS[2].mv[0].y = 0;
        spatialCMVS[2].mv[1].x = 0;
        spatialCMVS[2].mv[1].y = 0;
        spatialCMVS[2].pred_flag[0] = 0;
        spatialCMVS[2].pred_flag[1] =0;
        spatialCMVS[2].is_intra = 0;
    }

    // left bottom spatial merge candidate
    xA0 = x0 - 1;
    yA0 = y0 + nPbH;
    check_MER = 1;
    xA0_pu = xA0 >> s->sps->log2_min_pu_size;
    yA0_pu = yA0 >> s->sps->log2_min_pu_size;
    isAvailableA0 = 0;
    check_A0 = check_prediction_block_available(s, log2_cb_size, x0, y0, nPbW, nPbH, xA0, yA0, part_idx);

    if((xA0_pu >= 0) && !(tab_mvf[(yA0_pu) * pic_width_in_min_pu + xA0_pu].is_intra) && check_A0) {
        isAvailableA0 = 1;
    } else {
        isAvailableA0 = 0;
    }

    if((isDiffMER(s, xA0, yA0, x0, y0))) {
        isAvailableA0 = 0;
    }
    if (isAvailableA1 && isAvailableA0) {
        check_MER = !(compareMVrefidx(tab_mvf[(yA0_pu) * pic_width_in_min_pu + xA0_pu], tab_mvf[(yA1_pu) * pic_width_in_min_pu + xA1_pu]));
    }


    if (isAvailableA0 && check_MER) {
        available_a0_flag = 1;
        spatialCMVS[3] = tab_mvf[(yA0_pu) * pic_width_in_min_pu + xA0_pu];
    } else {
        available_a0_flag = 0;
        spatialCMVS[3].ref_idx[0] = -1;
        spatialCMVS[3].ref_idx[1] = -1;
        spatialCMVS[3].mv[0].x = 0;
        spatialCMVS[3].mv[0].y = 0;
        spatialCMVS[3].mv[1].x = 0;
        spatialCMVS[3].mv[1].y = 0;
        spatialCMVS[3].pred_flag[0] = 0;
        spatialCMVS[3].pred_flag[1] =0;
        spatialCMVS[3].is_intra = 0;
    }

    // above left spatial merge candidate
    xB2 = x0 - 1;
    yB2 = y0 - 1;
    check_MER = 1;
    xB2_pu = xB2 >> s->sps->log2_min_pu_size;
    yB2_pu = yB2 >> s->sps->log2_min_pu_size;
    isAvailableB2 = 0;
    check_B2 = check_prediction_block_available(s, log2_cb_size, x0, y0, nPbW, nPbH, xB2, yB2, part_idx);

    if((xB2_pu >= 0) && (yB2_pu >= 0) && !(tab_mvf[(yB2_pu) * pic_width_in_min_pu + xB2_pu].is_intra) && check_B2) {
        isAvailableB2 = 1;
    } else {
        isAvailableB2 = 0;
    }

    if((isDiffMER(s, xB2, yB2, x0, y0))) {
        isAvailableB2 = 0;
    }
    if (isAvailableA1 && isAvailableB2) {
        check_MER = !(compareMVrefidx(tab_mvf[(yB2_pu) * pic_width_in_min_pu + xB2_pu], tab_mvf[(yA1_pu) * pic_width_in_min_pu + xA1_pu]));
    }
    if (is_available_b1 && isAvailableB2) {
        check_MER_1 = !(compareMVrefidx(tab_mvf[(yB2_pu) * pic_width_in_min_pu + xB2_pu], tab_mvf[(yB1_pu) * pic_width_in_min_pu + xB1_pu]));
    }

    sumcandidates = available_a1_flag + available_b1_flag + available_b0_flag + available_a0_flag;


    if (isAvailableB2 && check_MER && check_MER_1 && sumcandidates != 4) {
        available_b2_flag =1;
        spatialCMVS[4] = tab_mvf[(yB2_pu) * pic_width_in_min_pu + xB2_pu];
    } else {
        available_b2_flag = 0;
        spatialCMVS[4].ref_idx[0] = -1;
        spatialCMVS[4].ref_idx[1] = -1;
        spatialCMVS[4].mv[0].x = 0;
        spatialCMVS[4].mv[0].y = 0;
        spatialCMVS[4].mv[1].x = 0;
        spatialCMVS[4].mv[1].y = 0;
        spatialCMVS[4].pred_flag[0] = 0;
        spatialCMVS[4].pred_flag[1] =0;
        spatialCMVS[4].is_intra = 0;
    }

    // temporal motion vector candidate
    // one optimization is that do temporal checking only if the number of
    // available candidates < MRG_MAX_NUM_CANDS
    if(s->sh.slice_temporal_mvp_enabled_flag == 0) {
        availableFlagLXCol = 0;
    } else {
        availableFlagL0Col = temporal_luma_motion_vector(s, x0, y0, nPbW, nPbH, refIdxL0Col, &mvL0Col, 0);
        // one optimization is that l1 check can be done only when the current slice type is B_SLICE
        if(s->sh.slice_type == B_SLICE) {
            availableFlagL1Col = temporal_luma_motion_vector(s, x0, y0, nPbW, nPbH, refIdxL1Col, &mvL1Col, 1);
        }
        availableFlagLXCol = availableFlagL0Col || availableFlagL1Col;
        if(availableFlagLXCol) {
            TMVPCand.is_intra = 0;
            TMVPCand.pred_flag[0] = availableFlagL0Col;
            TMVPCand.pred_flag[1] = availableFlagL1Col;
            if (TMVPCand.pred_flag[0]) {
                TMVPCand.mv[0] = mvL0Col;
                TMVPCand.ref_idx[0]  = refIdxL0Col;
            }
            if (TMVPCand.pred_flag[1]) {
                TMVPCand.mv[1] = mvL1Col;
                TMVPCand.ref_idx[1]  = refIdxL1Col;
            }
        }
    }

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
    if (availableFlagLXCol && (mergearray_index < MRG_MAX_NUM_CANDS)) {
        mergecandlist[mergearray_index] = TMVPCand;
        mergearray_index++;
    }
    numMergeCand = mergearray_index;
    numOrigMergeCand = mergearray_index;

    // combined bi-predictive merge candidates  (applies for B slices)
    if (s->sh.slice_type == B_SLICE) {
        if((numOrigMergeCand > 1) && (numOrigMergeCand < MRG_MAX_NUM_CANDS)) {

            numInputMergeCand = numMergeCand;
            combIdx           = 0;
            combStop          = 0;
            while (combStop != 1)
            {
                l0CandIdx = l0_l1_cand_idx[combIdx][0];
                l1CandIdx = l0_l1_cand_idx[combIdx][1];
                l0Cand = mergecandlist[l0CandIdx];
                l1Cand = mergecandlist[l1CandIdx];
                if ((l0Cand.pred_flag[0] == 1) && (l1Cand.pred_flag[1] == 1)
                    && (((DiffPicOrderCnt(refPicList[0].list[l0Cand.ref_idx[0]], refPicList[1].list[l1Cand.ref_idx[1]])) != 0)
                        || ((l0Cand.mv[0].x != l1Cand.mv[1].x) || (l0Cand.mv[0].y != l1Cand.mv[1].y)))) {

                        combCand.ref_idx[0] = l0Cand.ref_idx[0];
                        combCand.ref_idx[1] = l1Cand.ref_idx[1];
                        combCand.pred_flag[0] = 1;
                        combCand.pred_flag[1] = 1;
                        combCand.mv[0].x = l0Cand.mv[0].x;
                        combCand.mv[0].y = l0Cand.mv[0].y;
                        combCand.mv[1].x = l1Cand.mv[1].x;
                        combCand.mv[1].y = l1Cand.mv[1].y;
                        combCand.is_intra = 0;
                        mergecandlist[numMergeCand] = combCand;
                        numMergeCand++;
                    }
                combIdx++;
                if((combIdx == numOrigMergeCand * (numOrigMergeCand-1)) || (numMergeCand == MRG_MAX_NUM_CANDS)) {
                    combStop = 1;
                    break;
                }
            }
        }
    }


    /*
     * append Zero motion vector candidates
     */
    if(s->sh.slice_type == P_SLICE) {
        numRefIdx = s->sh.num_ref_idx_l0_active;
    } else if(s->sh.slice_type == B_SLICE) {
        numRefIdx = s->sh.num_ref_idx_l0_active > s->sh.num_ref_idx_l1_active ? s->sh.num_ref_idx_l1_active : s->sh.num_ref_idx_l0_active;
    }
    numInputMergeCand = numMergeCand;
    while(numMergeCand < MRG_MAX_NUM_CANDS) {
        if(s->sh.slice_type == P_SLICE) {
            zerovector.ref_idx[0] = (zeroIdx < numRefIdx) ? zeroIdx : 0;
            zerovector.ref_idx[1] = -1;
            zerovector.pred_flag[0] = 1;
            zerovector.pred_flag[1] = 0;
            zerovector.mv[0].x = 0;
            zerovector.mv[0].y = 0;
            zerovector.mv[1].x = 0;
            zerovector.mv[1].y = 0;
            zerovector.is_intra = 0;
        } else if(s->sh.slice_type == B_SLICE) {
            zerovector.ref_idx[0] = (zeroIdx < numRefIdx) ? zeroIdx : 0;
            zerovector.ref_idx[1] = (zeroIdx < numRefIdx) ? zeroIdx : 0;
            zerovector.pred_flag[0] =1;
            zerovector.pred_flag[1] =1;
            zerovector.mv[0].x = 0;
            zerovector.mv[0].y = 0;
            zerovector.mv[1].x = 0;
            zerovector.mv[1].y = 0;
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
void ff_hevc_luma_mv_merge_mode(HEVCContext *s, int x0, int y0, int nPbW, int nPbH, int log2_cb_size, int part_idx, int merge_idx, MvField *mv)
{
    int singleMCLFlag = 0;
    int nCS = 1 << log2_cb_size;
    struct MvField mergecand_list[MRG_MAX_NUM_CANDS] = {{{{ 0 }}}};

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
    if ((mergecand_list[merge_idx].pred_flag[0] == 1) &&
        (mergecand_list[merge_idx].pred_flag[1] == 1) &&
        ((nPbW + nPbH) == 12)) {
        mergecand_list[merge_idx].ref_idx[1] = -1;
        mergecand_list[merge_idx].pred_flag[1] = 0;
    }
    mv->mv[0].x = mergecand_list[merge_idx].mv[0].x;
    mv->mv[0].y = mergecand_list[merge_idx].mv[0].y;
    mv->mv[1].x = mergecand_list[merge_idx].mv[1].x;
    mv->mv[1].y = mergecand_list[merge_idx].mv[1].y;
    mv->ref_idx[0] = mergecand_list[merge_idx].ref_idx[0];
    mv->ref_idx[1] = mergecand_list[merge_idx].ref_idx[1];
    mv->pred_flag[0] = mergecand_list[merge_idx].pred_flag[0];
    mv->pred_flag[1] = mergecand_list[merge_idx].pred_flag[1];
}

#define TAB_MVF(x, y)                                                              \
    tab_mvf[(y) * pic_width_in_min_pu + x]

static av_always_inline void dist_scale(HEVCContext *s, Mv * mv, int pic_width_in_min_pu, int x_pu, int y_pu, int elist, int ref_idx_curr, int ref_idx) {
    RefPicList  *refPicList =  s->ref->refPicList;
    MvField *tab_mvf = s->ref->tab_mvf;
    if((DiffPicOrderCnt(refPicList[elist].list[TAB_MVF(x_pu, y_pu).ref_idx[elist]], refPicList[ref_idx_curr].list[ref_idx]))!=0) {
        // *** Assuming there are no long term pictures in version 1 of the decoder and the pictures are short term pictures ***
        int td = av_clip_int8_c((DiffPicOrderCnt(s->poc,refPicList[elist].list[(TAB_MVF(x_pu, y_pu).ref_idx[elist])])));
        int tb = av_clip_int8_c((DiffPicOrderCnt(s->poc,refPicList[ref_idx_curr].list[ref_idx])));
        int tx = (0x4000 + abs(td/2)) / td;
        int distScaleFactor = av_clip_c((tb * tx + 32) >> 6, -4096, 4095);
        mv->x = av_clip_int16_c((distScaleFactor * mv->x + 127 + (distScaleFactor * mv->x < 0)) >> 8);
        mv->y = av_clip_int16_c((distScaleFactor * mv->y + 127 + (distScaleFactor * mv->y < 0)) >> 8);
    }
}

void ff_hevc_luma_mv_mvp_mode(HEVCContext *s, int x0, int y0, int nPbW, int nPbH, int log2_cb_size, int part_idx, int merge_idx, MvField *mv , int mvp_lx_flag, int LX)
{
    RefPicList  *refPicList =  s->ref->refPicList;
    MvField *tab_mvf = s->ref->tab_mvf;
    int isScaledFlag_L0 = 0;
    int availableFlagLXA0 = 0;
    int availableFlagLXB0 = 0;
    int availableFlagLXCol = 0;
    int numMVPCandLX = 0;
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
    int is_available_b1 = 0;

    int xB2, yB2;
    int xB2_pu, yB2_pu;
    int isAvailableB2 = 0;
    Mv mvpcand_list[2] = { {0} };
    int check_A0, check_A1, check_B0, check_B1, check_B2;
    Mv mxA = {0};
    Mv mxB = {0};
    Mv mvLXCol = {0};
    int ref_idx_curr = 0;
    int ref_idx = 0;
    int pi_L0=0; //pred_flag_index_l0
    int pi_L1=0; //pred_flag_index_l1

    if(LX == 0) {
        ref_idx_curr = 0; //l0
        ref_idx = mv->ref_idx[0];
        pi_L0=0;
        pi_L1=1;
    } else if (LX == 1){
        ref_idx_curr = 1; // l1
        ref_idx = mv->ref_idx[1];
        pi_L0=1;
        pi_L1=0;
    }

    // left bottom spatial candidate
    xA0 = x0 - 1;
    yA0 = y0 + nPbH;
    xA0_pu = xA0 >> s->sps->log2_min_pu_size;
    yA0_pu = yA0 >> s->sps->log2_min_pu_size;
    isAvailableA0 = 0;
    check_A0 = check_prediction_block_available(s, log2_cb_size, x0, y0, nPbW, nPbH, xA0, yA0, part_idx);

    isAvailableA0 = ((xA0_pu >= 0) && !(TAB_MVF(xA0_pu, yA0_pu).is_intra) && check_A0);

    //left spatial merge candidate
    xA1 = x0 - 1;
    yA1 = y0 + nPbH - 1;
    xA1_pu = xA1 >> s->sps->log2_min_pu_size;
    yA1_pu = yA1 >> s->sps->log2_min_pu_size;
    isAvailableA1 = 0;
    check_A1 = check_prediction_block_available (s,log2_cb_size, x0, y0, nPbW, nPbH, xA1, yA1, part_idx);

    isAvailableA1 = ((xA1_pu >= 0) && !(TAB_MVF(xA1_pu, yA1_pu).is_intra) && check_A1);

    if ((isAvailableA0) || (isAvailableA1)) {
        isScaledFlag_L0 = 1;
    }

    // XA0 and L1
    if ((isAvailableA0) && !(TAB_MVF(xA0_pu, yA0_pu).is_intra) && (availableFlagLXA0 == 0)) {
        if ((TAB_MVF(xA0_pu, yA0_pu).pred_flag[pi_L0] == 1) &&
            (DiffPicOrderCnt(refPicList[pi_L0].list[(TAB_MVF(xA0_pu, yA0_pu).ref_idx[pi_L0])], refPicList[ref_idx_curr].list[ref_idx])) == 0) {
            availableFlagLXA0 = 1;
            mxA = TAB_MVF(xA0_pu, yA0_pu).mv[pi_L0];
        }
    }

    // XA0 and L0
    if ((isAvailableA0) && !(TAB_MVF(xA0_pu, yA0_pu).is_intra) && (availableFlagLXA0 == 0)) {
        if ((TAB_MVF(xA0_pu, yA0_pu).pred_flag[pi_L1] == 1) &&
            (DiffPicOrderCnt(refPicList[pi_L1].list[(TAB_MVF(xA0_pu, yA0_pu).ref_idx[pi_L1])], refPicList[ref_idx_curr].list[ref_idx])) == 0) {
            availableFlagLXA0 = 1;
            mxA = TAB_MVF(xA0_pu, yA0_pu).mv[pi_L1];
        }
    }

    //XA1 and L1
    if ((isAvailableA1) && !(TAB_MVF(xA1_pu, yA1_pu).is_intra) && (availableFlagLXA0 == 0)) {
        if ((TAB_MVF(xA1_pu, yA1_pu).pred_flag[pi_L0] == 1) &&
            (DiffPicOrderCnt(refPicList[pi_L0].list[(TAB_MVF(xA1_pu, yA1_pu).ref_idx[pi_L0])], refPicList[ref_idx_curr].list[ref_idx])) == 0) {
            availableFlagLXA0 = 1;
            mxA = TAB_MVF(xA1_pu, yA1_pu).mv[pi_L0];
        }
    }

    //XA1 and L0
    if ((isAvailableA1) && !(TAB_MVF(xA1_pu, yA1_pu).is_intra) && (availableFlagLXA0 == 0)) {
        if ((TAB_MVF(xA1_pu, yA1_pu).pred_flag[pi_L1] == 1) &&
            (DiffPicOrderCnt(refPicList[pi_L1].list[(TAB_MVF(xA1_pu, yA1_pu).ref_idx[pi_L1])], refPicList[ref_idx_curr].list[ref_idx])) == 0) {
            availableFlagLXA0 =1;
            mxA = TAB_MVF(xA1_pu, yA1_pu).mv[pi_L1];
        }
    }

    if ((isAvailableA0) && !(TAB_MVF(xA0_pu, yA0_pu).is_intra) && (availableFlagLXA0 == 0)) {
        // XA0 and L1
        if (TAB_MVF(xA0_pu, yA0_pu).pred_flag[pi_L0] == 1) {
            availableFlagLXA0 = 1;
            mxA = TAB_MVF(xA0_pu, yA0_pu).mv[pi_L0];
            dist_scale(s, &mxA, pic_width_in_min_pu, xA0_pu, yA0_pu, pi_L0, ref_idx_curr, ref_idx);
        }
    }

    if ((isAvailableA0) && !(TAB_MVF(xA0_pu, yA0_pu).is_intra) && (availableFlagLXA0 == 0)) {
        // XA0 and L0
        if (TAB_MVF(xA0_pu, yA0_pu).pred_flag[pi_L1] == 1) {
            availableFlagLXA0 = 1;
            mxA = TAB_MVF(xA0_pu, yA0_pu).mv[pi_L1];
            dist_scale(s, &mxA, pic_width_in_min_pu, xA0_pu, yA0_pu, pi_L1, ref_idx_curr, ref_idx);
        }
    }

    if ((isAvailableA1) && !(TAB_MVF(xA1_pu, yA1_pu).is_intra) && (availableFlagLXA0 == 0)) {
        //XA1 and L1
        if (TAB_MVF(xA1_pu, yA1_pu).pred_flag[pi_L0] == 1) {
            availableFlagLXA0 = 1;
            mxA = TAB_MVF(xA1_pu, yA1_pu).mv[pi_L0];
            dist_scale(s, &mxA, pic_width_in_min_pu, xA1_pu, yA1_pu, pi_L0, ref_idx_curr, ref_idx);
        }
    }

    if ((isAvailableA1) && !(TAB_MVF(xA1_pu, yA1_pu).is_intra) && (availableFlagLXA0 == 0)) {
        //XA1 and L0
        if (TAB_MVF(xA1_pu, yA1_pu).pred_flag[pi_L1] == 1) {
            availableFlagLXA0 = 1;
            mxA = TAB_MVF(xA1_pu, yA1_pu).mv[pi_L1];
            dist_scale(s, &mxA, pic_width_in_min_pu, xA1_pu, yA1_pu, pi_L1, ref_idx_curr, ref_idx);
        }
    }

    // B candidates
    // above right spatial merge candidate
    xB0 = x0 + nPbW;
    yB0 = y0 - 1;
    xB0_pu = xB0 >> s->sps->log2_min_pu_size;
    yB0_pu = yB0 >> s->sps->log2_min_pu_size;
    isAvailableB0 = 0;
    check_B0 = check_prediction_block_available(s, log2_cb_size, x0, y0, nPbW, nPbH, xB0, yB0, part_idx);

    isAvailableB0 = ((yB0_pu >= 0) && !(TAB_MVF(xB0_pu, yB0_pu).is_intra) && check_B0);

    // XB0 and L1
    if ((isAvailableB0) && !(TAB_MVF(xB0_pu, yB0_pu).is_intra) && (availableFlagLXB0 == 0)) {
        if ((TAB_MVF(xB0_pu, yB0_pu).pred_flag[pi_L0] == 1) &&
            (DiffPicOrderCnt(refPicList[pi_L0].list[TAB_MVF(xB0_pu, yB0_pu).ref_idx[pi_L0]], refPicList[ref_idx_curr].list[ref_idx])) == 0) {
            availableFlagLXB0 = 1;
            mxB = TAB_MVF(xB0_pu, yB0_pu).mv[pi_L0];
        }
    }

    // XB0 and L0
    if ((isAvailableB0) && !(TAB_MVF(xB0_pu, yB0_pu).is_intra) && (availableFlagLXB0 == 0)) {
        if ((TAB_MVF(xB0_pu, yB0_pu).pred_flag[pi_L1] == 1) &&
            (DiffPicOrderCnt(refPicList[pi_L1].list[(TAB_MVF(xB0_pu, yB0_pu).ref_idx[pi_L1])], refPicList[ref_idx_curr].list[ref_idx])) == 0) {
            availableFlagLXB0 = 1;
            mxB = TAB_MVF(xB0_pu, yB0_pu).mv[pi_L1];
        }
    }

    if (!availableFlagLXB0) {
        // above spatial merge candidate
        xB1 = x0 + nPbW - 1;
        yB1 = y0 - 1;
        xB1_pu = xB1 >> s->sps->log2_min_pu_size;
        yB1_pu = yB1 >> s->sps->log2_min_pu_size;
        is_available_b1 = 0;
        check_B1 = check_prediction_block_available(s, log2_cb_size, x0, y0, nPbW, nPbH, xB1, yB1, part_idx);
        is_available_b1 = ((yB1_pu >= 0) && !(TAB_MVF(xB1_pu, yB1_pu).is_intra) && check_B1);

        // XB1 and L1
        if ((is_available_b1) && !(TAB_MVF(xB1_pu, yB1_pu).is_intra) && (availableFlagLXB0 == 0)) {
            // XB1 and L1
            if ((TAB_MVF(xB1_pu, yB1_pu).pred_flag[pi_L0] == 1) &&
                (DiffPicOrderCnt(refPicList[pi_L0].list[(TAB_MVF(xB1_pu, yB1_pu).ref_idx[pi_L0])], refPicList[ref_idx_curr].list[ref_idx])) == 0) {
                availableFlagLXB0 =1;
                mxB = TAB_MVF(xB1_pu, yB1_pu).mv[pi_L0];
            }
        }
        // XB1 and L0
        if ((is_available_b1) && !(TAB_MVF(xB1_pu, yB1_pu).is_intra) && (availableFlagLXB0 == 0)) {
            if ((TAB_MVF(xB1_pu, yB1_pu).pred_flag[pi_L1] == 1) &&
                (DiffPicOrderCnt(refPicList[pi_L1].list[(TAB_MVF(xB1_pu, yB1_pu).ref_idx[pi_L1])], refPicList[ref_idx_curr].list[ref_idx])) == 0) {
                availableFlagLXB0 = 1;
                mxB = TAB_MVF(xB1_pu, yB1_pu).mv[pi_L1];
            }
        }
    }

    if (!availableFlagLXB0) {
        // above left spatial merge candidate
        xB2 = x0 - 1;
        yB2 = y0 - 1;
        xB2_pu = xB2 >> s->sps->log2_min_pu_size;
        yB2_pu = yB2 >> s->sps->log2_min_pu_size;
        isAvailableB2 = 0;
        check_B2 = check_prediction_block_available(s, log2_cb_size, x0, y0, nPbW, nPbH, xB2, yB2, part_idx);

        isAvailableB2 = ((xB2_pu >= 0) && (yB2_pu >= 0) && !(TAB_MVF(xB2_pu, yB2_pu).is_intra) && check_B2);

        // XB2 and L1
        if ((isAvailableB2) && !(TAB_MVF(xB2_pu, yB2_pu).is_intra) && (availableFlagLXB0 == 0)) {
            if ((TAB_MVF(xB2_pu, yB2_pu).pred_flag[pi_L0] == 1) &&
                (DiffPicOrderCnt(refPicList[pi_L0].list[(TAB_MVF(xB2_pu, yB2_pu).ref_idx[pi_L0])], refPicList[ref_idx_curr].list[ref_idx]))== 0) {
                availableFlagLXB0 = 1;
                mxB = TAB_MVF(xB2_pu, yB2_pu).mv[pi_L0];
            }
        }

        // XB2 and L0
        if ((isAvailableB2) && !(TAB_MVF(xB2_pu, yB2_pu).is_intra) && (availableFlagLXB0 == 0)) {
            if((TAB_MVF(xB2_pu, yB2_pu).pred_flag[pi_L1] == 1) &&
               (DiffPicOrderCnt(refPicList[pi_L1].list[(TAB_MVF(xB2_pu, yB2_pu).ref_idx[pi_L1])], refPicList[ref_idx_curr].list[ref_idx]))==0) {
                availableFlagLXB0 =1;
                mxB = TAB_MVF(xB2_pu, yB2_pu).mv[pi_L1];
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

    // XB0 and L1
    if ((isAvailableB0) && !(TAB_MVF(xB0_pu, yB0_pu).is_intra) && (availableFlagLXB0 == 0)) {
        if ((TAB_MVF(xB0_pu, yB0_pu).pred_flag[pi_L0] == 1) && (isScaledFlag_L0 == 0)) {
            availableFlagLXB0 =1;
            mxB = TAB_MVF(xB0_pu, yB0_pu).mv[pi_L0];
            dist_scale(s, &mxB, pic_width_in_min_pu, xB0_pu, yB0_pu, pi_L0, ref_idx_curr, ref_idx);
        }
    }
    // XB0 and L0
    if ((isAvailableB0) && !(TAB_MVF(xB0_pu, yB0_pu).is_intra) && (availableFlagLXB0 == 0)) {
        if ((TAB_MVF(xB0_pu, yB0_pu).pred_flag[pi_L1] == 1) && (isScaledFlag_L0 == 0)) {
            availableFlagLXB0 = 1;
            mxB = TAB_MVF(xB0_pu, yB0_pu).mv[pi_L1];
            dist_scale(s, &mxB, pic_width_in_min_pu, xB0_pu, yB0_pu, pi_L1, ref_idx_curr, ref_idx);
        }
    }


    if ((is_available_b1) && !(TAB_MVF(xB1_pu, yB1_pu).is_intra) && (availableFlagLXB0 == 0)) {
        // XB1 and L1
        if((TAB_MVF(xB1_pu, yB1_pu).pred_flag[pi_L0] == 1) && (isScaledFlag_L0 == 0)) {
            availableFlagLXB0 =1;
            mxB = TAB_MVF(xB1_pu, yB1_pu).mv[pi_L0];
            dist_scale(s, &mxB, pic_width_in_min_pu, xB1_pu, yB1_pu, pi_L0, ref_idx_curr, ref_idx);
        }
    }

    if ((is_available_b1) && !(TAB_MVF(xB1_pu, yB1_pu).is_intra) && (availableFlagLXB0 == 0)) {
        // XB1 and L0
        if((TAB_MVF(xB1_pu, yB1_pu).pred_flag[pi_L1] == 1) && (isScaledFlag_L0 == 0)) {
            availableFlagLXB0 = 1;
            mxB = TAB_MVF(xB1_pu, yB1_pu).mv[pi_L1];
            dist_scale(s, &mxB, pic_width_in_min_pu, xB1_pu, yB1_pu, pi_L1, ref_idx_curr, ref_idx);
        }
    }


    if ((isAvailableB2) && !(TAB_MVF(xB2_pu, yB2_pu).is_intra) && (availableFlagLXB0 == 0)) {
        // XB2 and L1
        if ((TAB_MVF(xB2_pu, yB2_pu).pred_flag[pi_L0] == 1)  && (isScaledFlag_L0 == 0)) {
            availableFlagLXB0 = 1;
            mxB = TAB_MVF(xB2_pu, yB2_pu).mv[pi_L0];
            dist_scale(s, &mxB, pic_width_in_min_pu, xB2_pu, yB2_pu, pi_L0, ref_idx_curr, ref_idx);
        }
    }

    if ((isAvailableB2) && !(TAB_MVF(xB2_pu, yB2_pu).is_intra) && (availableFlagLXB0 == 0)) {
        // XB2 and L0
        if ((TAB_MVF(xB2_pu, yB2_pu).pred_flag[pi_L1] == 1) && (isScaledFlag_L0 == 0)) {
            availableFlagLXB0 = 1;
            mxB = TAB_MVF(xB2_pu, yB2_pu).mv[pi_L1];
            dist_scale(s, &mxB, pic_width_in_min_pu, xB2_pu, yB2_pu, pi_L1, ref_idx_curr, ref_idx);
        }
    }


    if (availableFlagLXA0 && availableFlagLXB0 && ((mxA.x != mxB.x) || (mxA.y != mxB.y))) {
        availableFlagLXCol = 0 ;
    } else {
        //temporal motion vector prediction candidate
        if(s->sh.slice_temporal_mvp_enabled_flag == 0) {
            availableFlagLXCol = 0;
        } else {
            availableFlagLXCol = temporal_luma_motion_vector(s, x0, y0, nPbW, nPbH, ref_idx, &mvLXCol, LX);
        }
    }

    if(availableFlagLXA0) {
        mvpcand_list[numMVPCandLX] = mxA;
        numMVPCandLX++;
    }
    if(availableFlagLXB0) {
        mvpcand_list[numMVPCandLX] = mxB;
        numMVPCandLX++;
    }

    if(availableFlagLXA0 && availableFlagLXB0 && ((mxA.x == mxB.x) && (mxA.y == mxB.y))) {
        numMVPCandLX--;
    }

    if (availableFlagLXCol && numMVPCandLX < 2) {
        mvpcand_list[numMVPCandLX] = mvLXCol;
        numMVPCandLX++;
    }

    while (numMVPCandLX < 2) { // insert zero motion vectors when the number of available candidates are less than 2
        mvpcand_list[numMVPCandLX].x = 0;
        mvpcand_list[numMVPCandLX].y = 0;
        numMVPCandLX++;
    }

    if (LX == 0) {
        mv->mv[0].x  = mvpcand_list[mvp_lx_flag].x;
        mv->mv[0].y  = mvpcand_list[mvp_lx_flag].y;
    }
    if (LX == 1) {
        mv->mv[1].x  = mvpcand_list[mvp_lx_flag].x;
        mv->mv[1].y  = mvpcand_list[mvp_lx_flag].y;
    }
}
