/*
 * HEVC Supplementary Enhancement Information messages
 *
 * Copyright (C) 2012 Guillaume Martres
 * Copyright (C) 2013 Vittorio Giovara
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

static void decode_nal_sei_decoded_picture_hash(HEVCContext *s, int payload_size)
{
    int cIdx, i;
    int hash_type;
    int picture_md5;
    int picture_crc;
    int picture_checksum;
    GetBitContext *gb = &s->gb;
    hash_type = get_bits(gb, 8);

    for( cIdx = 0; cIdx < 3/*((s->sps->chroma_format_idc == 0) ? 1 : 3)*/; cIdx++ ) {
        if ( hash_type == 0 ) {
            for( i = 0; i < 16; i++) {
                picture_md5 = get_bits(gb, 8);
            }
        } else if( hash_type == 1 ) {
            picture_crc = get_bits(gb, 16);
        } else if( hash_type == 2 ) {
            picture_checksum = get_bits(gb, 32);
        }
    }
}

static int decode_nal_sei_message(HEVCContext *s)
{
    GetBitContext *gb = &s->gb;

    int payload_type = 0;
    int payload_size = 0;
    int byte = 0xFF;
    av_log(s->avctx, AV_LOG_DEBUG, "Decoding SEI\n");

    while (byte == 0xFF) {
        byte = get_bits(gb, 8);
        payload_type += byte;
    }
    byte = 0xFF;
    while (byte == 0xFF) {
        byte = get_bits(gb, 8);
        payload_size += byte;
    }
    if (s->nal_unit_type == NAL_SEI_PREFIX) {
        if (payload_type == 256 && s->decode_checksum_sei)
            decode_nal_sei_decoded_picture_hash(s, payload_size);
        else {
            av_log(s->avctx, AV_LOG_DEBUG, "Skipped PREFIX SEI %d\n", payload_type);
            skip_bits(gb, 8*payload_size);
        }
    } else { /* nal_unit_type == NAL_SEI_SUFFIX */
        if (payload_type == 132 && s->decode_checksum_sei)
            decode_nal_sei_decoded_picture_hash(s, payload_size);
        else {
            av_log(s->avctx, AV_LOG_DEBUG, "Skipped SUFFIX SEI %d\n", payload_type);
            skip_bits(gb, 8*payload_size);
        }
    }
    return 0;
}

static int more_rbsp_data(GetBitContext *gb)
{
    return get_bits_left(gb) > 0 && show_bits(gb, 8) != 0x80;
}

int ff_hevc_decode_nal_sei(HEVCContext *s)
{
    do {
        decode_nal_sei_message(s);
    } while (more_rbsp_data(&s->gb));
    return 0;
}

