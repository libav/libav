/*
 * H.26L/H.264/AVC/JVT/14496-10/... encoder/decoder
 * Copyright (c) 2003 Michael Niedermayer <michaelni@gmx.at>
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

/**
 * @file
 * Context Adaptive Binary Arithmetic Coder.
 */

#include <string.h>

#include "libavutil/common.h"
#include "get_bits.h"
#include "cabac.h"
#include "cabac_functions.h"

uint8_t ff_h264_cabac_tables[512 + 4*2*64 + 4*64 + 63] = {
 9,8,7,7,6,6,6,6,5,5,5,5,5,5,5,5,
 4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,
 3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,
 3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,
 2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
 2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
 2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
 2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
};

static uint8_t h264_mps_state[2 * 64];

static const uint8_t last_coeff_flag_offset_8x8[63] = {
 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4,
 5, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8
};

/**
 *
 * @param buf_size size of buf in bits
 */
void ff_init_cabac_encoder(CABACContext *c, uint8_t *buf, int buf_size){
    init_put_bits(&c->pb, buf, buf_size);

    c->low= 0;
    c->range= 0x1FE;
    c->outstanding_count= 0;
    c->pb.bit_left++; //avoids firstBitFlag
}

/**
 *
 * @param buf_size size of buf in bits
 */
void ff_init_cabac_decoder(CABACContext *c, const uint8_t *buf, int buf_size){
    c->bytestream_start=
    c->bytestream= buf;
    c->bytestream_end= buf + buf_size;

#if CABAC_BITS == 16
    c->low =  (*c->bytestream++)<<18;
    c->low+=  (*c->bytestream++)<<10;
#else
    c->low =  (*c->bytestream++)<<10;
#endif
    c->low+= ((*c->bytestream++)<<2) + 2;
    c->range= 0x1FE;
}

void ff_init_cabac_states(CABACContext *c){
    int i, j;

    for(i=0; i<64; i++){
        for(j=0; j<4; j++){ //FIXME check if this is worth the 1 shift we save
            ff_h264_lps_range[j*2*64+2*i+0]=
            ff_h264_lps_range[j*2*64+2*i+1]= lps_range[i][j];
        }

        ff_h264_mlps_state[128+2*i+0]=
        h264_mps_state[2 * i + 0] = 2 * mps_state[i] + 0;
        ff_h264_mlps_state[128+2*i+1]=
        h264_mps_state[2 * i + 1] = 2 * mps_state[i] + 1;

        if( i ){
            ff_h264_mlps_state[128-2*i-1]= 2*lps_state[i]+0;
            ff_h264_mlps_state[128-2*i-2]= 2*lps_state[i]+1;
        }else{
            ff_h264_mlps_state[128-2*i-1]= 1;
            ff_h264_mlps_state[128-2*i-2]= 0;
        }
    }
    for(i=0; i< 63; i++){
      ff_h264_last_coeff_flag_offset_8x8[i] = last_coeff_flag_offset_8x8[i];
    }
}

#ifdef TEST
#define SIZE 10240

#include "libavutil/lfg.h"
#include "avcodec.h"
#include "cabac.h"

static inline void put_cabac_bit(CABACContext *c, int b){
    put_bits(&c->pb, 1, b);
    for(;c->outstanding_count; c->outstanding_count--){
        put_bits(&c->pb, 1, 1-b);
    }
}

static inline void renorm_cabac_encoder(CABACContext *c){
    while(c->range < 0x100){
        //FIXME optimize
        if(c->low<0x100){
            put_cabac_bit(c, 0);
        }else if(c->low<0x200){
            c->outstanding_count++;
            c->low -= 0x100;
        }else{
            put_cabac_bit(c, 1);
            c->low -= 0x200;
        }

        c->range+= c->range;
        c->low += c->low;
    }
}

static void put_cabac(CABACContext *c, uint8_t * const state, int bit){
    int RangeLPS= ff_h264_lps_range[2*(c->range&0xC0) + *state];

    if(bit == ((*state)&1)){
        c->range -= RangeLPS;
        *state    = h264_mps_state[*state];
    }else{
        c->low += c->range - RangeLPS;
        c->range = RangeLPS;
    }

    renorm_cabac_encoder(c);
}

/**
 * @param bit 0 -> write zero bit, !=0 write one bit
 */
static void put_cabac_bypass(CABACContext *c, int bit){
    c->low += c->low;

    if(bit){
        c->low += c->range;
    }
//FIXME optimize
    if(c->low<0x200){
        put_cabac_bit(c, 0);
    }else if(c->low<0x400){
        c->outstanding_count++;
        c->low -= 0x200;
    }else{
        put_cabac_bit(c, 1);
        c->low -= 0x400;
    }
}

/**
 *
 * @return the number of bytes written
 */
static int put_cabac_terminate(CABACContext *c, int bit){
    c->range -= 2;

    if(!bit){
        renorm_cabac_encoder(c);
    }else{
        c->low += c->range;
        c->range= 2;

        renorm_cabac_encoder(c);

        assert(c->low <= 0x1FF);
        put_cabac_bit(c, c->low>>9);
        put_bits(&c->pb, 2, ((c->low>>7)&3)|1);

        flush_put_bits(&c->pb); //FIXME FIXME FIXME XXX wrong
    }

    return (put_bits_count(&c->pb)+7)>>3;
}

int main(void){
    CABACContext c;
    uint8_t b[9*SIZE];
    uint8_t r[9*SIZE];
    int i;
    uint8_t state[10]= {0};
    AVLFG prng;

    av_lfg_init(&prng, 1);
    ff_init_cabac_encoder(&c, b, SIZE);
    ff_init_cabac_states(&c);

    for(i=0; i<SIZE; i++){
        r[i] = av_lfg_get(&prng) % 7;
    }

    for(i=0; i<SIZE; i++){
START_TIMER
        put_cabac_bypass(&c, r[i]&1);
STOP_TIMER("put_cabac_bypass")
    }

    for(i=0; i<SIZE; i++){
START_TIMER
        put_cabac(&c, state, r[i]&1);
STOP_TIMER("put_cabac")
    }

    put_cabac_terminate(&c, 1);

    ff_init_cabac_decoder(&c, b, SIZE);

    memset(state, 0, sizeof(state));

    for(i=0; i<SIZE; i++){
START_TIMER
        if( (r[i]&1) != get_cabac_bypass(&c) )
            av_log(NULL, AV_LOG_ERROR, "CABAC bypass failure at %d\n", i);
STOP_TIMER("get_cabac_bypass")
    }

    for(i=0; i<SIZE; i++){
START_TIMER
        if( (r[i]&1) != get_cabac(&c, state) )
            av_log(NULL, AV_LOG_ERROR, "CABAC failure at %d\n", i);
STOP_TIMER("get_cabac")
    }
    if(!get_cabac_terminate(&c))
        av_log(NULL, AV_LOG_ERROR, "where's the Terminator?\n");

    return 0;
}

#endif /* TEST */
