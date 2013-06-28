/*
 * HEVC Annex B format parser
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

#include "libavutil/common.h"
#include "parser.h"
#include "hevc.h"


#define START_CODE 0x000001 ///< start_code_prefix_one_3bytes
#define EMULATION_CODE 0x03 ///< emulation_prevention_three_byte
#define MAX_SKIPPED_BUFFER_SIZE 64
typedef struct HEVCParserContext {
    ParseContext pc;
    uint8_t *nal_buffer;
    unsigned int nal_buffer_size;
} HEVCParserContext;

/**
 * Annex B.1: Byte stream NAL unit syntax and semantics
 */
static int hevc_parse_nal_unit(HEVCParserContext *hpc, HEVCContext *cct, uint8_t **poutbuf,
		int *poutbuf_size, const uint8_t *buf,
		int buf_size)
{
    int i;
    ParseContext *pc = &hpc->pc;
    int mask = 0xFFFFFF;
    int skipped = 0;
    int header = 0;
    HEVCSharedContext *sc = cct->HEVCsc;

    // skip leading zeroes
    if (!pc->frame_start_found) {
        for (i = 0; i < buf_size; i++) {
            pc->state = (pc->state << 8) | buf[i];
            if ((pc->state & mask) == START_CODE) {
                pc->frame_start_found = 1;
                // the frame starts one byte after the start code
                header = i + 1;
                break;
            } else if (buf[i] != 0) {
                return AVERROR_INVALIDDATA;
            }
        }
    }
    sc->skipped_bytes = 0;

    buf      += header;
    buf_size -= header;
    *poutbuf = (uint8_t*)buf;

    // Remove emulation bytes and find the frame end
    if (pc->frame_start_found) {
        for (i = 0; i < buf_size; i++) {
            pc->state = (pc->state << 8) | buf[i];
            switch (pc->state & mask) {
            case START_CODE:
                *poutbuf_size = FFMAX(i - 2 - skipped, 0);
                return header + i + 1;
            case EMULATION_CODE:
                skipped++;
                if(skipped > sc->skipped_buf_size)  {
                    int *temp = sc->skipped_bytes_pos;
                    sc->skipped_bytes_pos = av_malloc((MAX_SKIPPED_BUFFER_SIZE+sc->skipped_buf_size)*sizeof(int));
                    memcpy(sc->skipped_bytes_pos, temp, sc->skipped_buf_size*sizeof(int));
                    av_free(temp);
                    sc->skipped_buf_size += MAX_SKIPPED_BUFFER_SIZE;
                }
                sc->skipped_bytes_pos[skipped-1] = i-skipped;
                if (*poutbuf != hpc->nal_buffer) {
                    hpc->nal_buffer = av_fast_realloc(hpc->nal_buffer,
                                                      &hpc->nal_buffer_size,
                                                      buf_size - skipped);
                    if (!hpc->nal_buffer)
                        return END_NOT_FOUND;
                    *poutbuf = hpc->nal_buffer;
                    memcpy(*poutbuf, buf, i - skipped + 1);
                }
                break;
            default:
                if (*poutbuf == hpc->nal_buffer)
                    (*poutbuf)[i-skipped] = buf[i];
            }
        }
    }

    sc->skipped_bytes = skipped;
	*poutbuf_size = buf_size - skipped;
	if (buf_size == 0)
		return 0;
    return END_NOT_FOUND;
}

// Each parsed packet is a NAL unit with the emulation bytes removed and may
// have trailing zero bytes.
static int hevc_parse(AVCodecParserContext *s,
                      AVCodecContext *avctx,
                      const uint8_t **poutbuf, int *poutbuf_size,
                      const uint8_t *buf, int buf_size)
{
    HEVCParserContext *hpc = s->priv_data;
    HEVCContext *cct = avctx->priv_data;
    ParseContext *pc = &hpc->pc;
    int combine_next = 0;
    int next = hevc_parse_nal_unit(hpc, cct, (uint8_t**)poutbuf, poutbuf_size, buf, buf_size);

    if (next == AVERROR_INVALIDDATA) {
        av_log(NULL, AV_LOG_ERROR, "Data fed to parser isn't a NAL unit\n");
        return buf_size;
    }

    // next is an offset in buf, but we want to combine frames from *poutbuf
    combine_next = (next != END_NOT_FOUND) ? *poutbuf_size : next;

    if (ff_combine_frame(pc, combine_next, poutbuf, poutbuf_size) < 0 ||
        *poutbuf_size == 0) {
        *poutbuf      = NULL;
        *poutbuf_size = 0;
        return buf_size;
    }

    return next;
}

static int hevc_init(AVCodecParserContext *s)
{
    HEVCParserContext *hpc = s->priv_data;
    hpc->nal_buffer_size = 0;
    hpc->nal_buffer      = NULL;
    return 0;
}

static void hevc_close(AVCodecParserContext *s)
{
    HEVCParserContext *hpc = s->priv_data;
    av_free(hpc->nal_buffer);

    ff_parse_close(s);
}


AVCodecParser ff_hevc_parser = {
    .codec_ids      = { AV_CODEC_ID_HEVC },
    .priv_data_size = sizeof(HEVCParserContext),
    .parser_init    = hevc_init,
    .parser_parse   = hevc_parse,
    .parser_close   = hevc_close
};
