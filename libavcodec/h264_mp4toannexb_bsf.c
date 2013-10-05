/*
 * H.264 MP4 to Annex B byte stream format filter
 * Copyright (c) 2007 Benoit Fouet <benoit.fouet@free.fr>
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

#include <string.h>

#include "libavutil/intreadwrite.h"
#include "libavutil/mem.h"
#include "avcodec.h"

typedef struct H264BSFContext {
    uint8_t  length_size;
    uint8_t  first_idr;
    int      extradata_parsed;
} H264BSFContext;

static int alloc_and_copy(uint8_t **poutbuf, int *poutbuf_size,
                          const uint8_t *sps_pps, uint32_t sps_pps_size,
                          const uint8_t *in, uint32_t in_size)
{
    uint32_t offset         = *poutbuf_size;
    uint8_t nal_header_size = offset ? 3 : 4;
    void *tmp;

    *poutbuf_size += sps_pps_size + in_size + nal_header_size;
    tmp = av_realloc(*poutbuf, *poutbuf_size + FF_INPUT_BUFFER_PADDING_SIZE);
    if (!tmp)
        return AVERROR(ENOMEM);
    *poutbuf = tmp;
    if (sps_pps)
        memcpy(*poutbuf + offset, sps_pps, sps_pps_size);
    memcpy(*poutbuf + sps_pps_size + nal_header_size + offset, in, in_size);
    if (!offset) {
        AV_WB32(*poutbuf + sps_pps_size, 1);
    } else {
        (*poutbuf + offset + sps_pps_size)[0] =
        (*poutbuf + offset + sps_pps_size)[1] = 0;
        (*poutbuf + offset + sps_pps_size)[2] = 1;
    }

    return 0;
}

static int h264_extradata_to_annexb(AVCodecContext *avctx, const int padding)
{
    uint16_t unit_size;
    uint64_t total_size                 = 0;
    uint8_t *out                        = NULL, unit_nb, sps_done = 0,
             sps_seen                   = 0, pps_seen = 0;
    const uint8_t *extradata            = avctx->extradata + 4;
    static const uint8_t nalu_header[4] = { 0, 0, 0, 1 };
    int length_size = (*extradata++ & 0x3) + 1; // retrieve length coded size

    if (length_size == 3)
        return AVERROR(EINVAL);

    /* retrieve sps and pps unit(s) */
    unit_nb = *extradata++ & 0x1f; /* number of sps unit(s) */
    if (!unit_nb) {
        unit_nb = *extradata++; /* number of pps unit(s) */
        sps_done++;

        if (unit_nb)
            pps_seen = 1;
    } else {
        sps_seen = 1;
    }

    while (unit_nb--) {
        void *tmp;

        unit_size   = AV_RB16(extradata);
        total_size += unit_size + 4;
        if (total_size > INT_MAX - padding ||
            extradata + 2 + unit_size > avctx->extradata +
            avctx->extradata_size) {
            av_free(out);
            return AVERROR(EINVAL);
        }
        tmp = av_realloc(out, total_size + padding);
        if (!tmp) {
            av_free(out);
            return AVERROR(ENOMEM);
        }
        out = tmp;
        memcpy(out + total_size - unit_size - 4, nalu_header, 4);
        memcpy(out + total_size - unit_size, extradata + 2, unit_size);
        extradata += 2 + unit_size;

        if (!unit_nb && !sps_done++) {
            unit_nb = *extradata++; /* number of pps unit(s) */
            if (unit_nb)
                pps_seen = 1;
        }
    }

    if (out)
        memset(out + total_size, 0, FF_INPUT_BUFFER_PADDING_SIZE);

    if (!sps_seen)
        av_log(avctx, AV_LOG_WARNING,
               "Warning: SPS NALU missing or invalid. "
               "The resulting stream may not play.\n");

    if (!pps_seen)
        av_log(avctx, AV_LOG_WARNING,
               "Warning: PPS NALU missing or invalid. "
               "The resulting stream may not play.\n");

    av_free(avctx->extradata);
    avctx->extradata      = out;
    avctx->extradata_size = total_size;

    return length_size;
}

static int h264_mp4toannexb_filter(AVBitStreamFilterContext *bsfc,
                                   AVCodecContext *avctx, const char *args,
                                   uint8_t **poutbuf, int *poutbuf_size,
                                   const uint8_t *buf, int buf_size,
                                   int keyframe)
{
    H264BSFContext *ctx = bsfc->priv_data;
    uint8_t unit_type;
    int32_t nal_size;
    uint32_t cumul_size    = 0;
    const uint8_t *buf_end = buf + buf_size;
    int ret = 0;

    /* nothing to filter */
    if (!avctx->extradata || avctx->extradata_size < 6) {
        *poutbuf      = (uint8_t *)buf;
        *poutbuf_size = buf_size;
        return 0;
    }

    /* retrieve sps and pps NAL units from extradata */
    if (!ctx->extradata_parsed) {
        ret = h264_extradata_to_annexb(avctx, FF_INPUT_BUFFER_PADDING_SIZE);
        if (ret < 0)
            return ret;
        ctx->length_size      = ret;
        ctx->first_idr        = 1;
        ctx->extradata_parsed = 1;
    }

    *poutbuf_size = 0;
    *poutbuf      = NULL;
    do {
        if (buf + ctx->length_size > buf_end)
            goto fail;

        if (ctx->length_size == 1) {
            nal_size = buf[0];
        } else if (ctx->length_size == 2) {
            nal_size = AV_RB16(buf);
        } else
            nal_size = AV_RB32(buf);

        buf      += ctx->length_size;
        unit_type = *buf & 0x1f;

        if (buf + nal_size > buf_end || nal_size < 0)
            goto fail;

        /* prepend only to the first type 5 NAL unit of an IDR picture */
        if (ctx->first_idr && unit_type == 5) {
            if (alloc_and_copy(poutbuf, poutbuf_size,
                               avctx->extradata, avctx->extradata_size,
                               buf, nal_size) < 0)
                goto fail;
            ctx->first_idr = 0;
        } else {
            if (alloc_and_copy(poutbuf, poutbuf_size,
                               NULL, 0, buf, nal_size) < 0)
                goto fail;
            if (!ctx->first_idr && unit_type == 1)
                ctx->first_idr = 1;
        }

        buf        += nal_size;
        cumul_size += nal_size + ctx->length_size;
    } while (cumul_size < buf_size);

    return 1;

fail:
    av_freep(poutbuf);
    *poutbuf_size = 0;
    return AVERROR(EINVAL);
}

AVBitStreamFilter ff_h264_mp4toannexb_bsf = {
    "h264_mp4toannexb",
    sizeof(H264BSFContext),
    h264_mp4toannexb_filter,
};
