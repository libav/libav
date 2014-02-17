/*
 * libx265 encoder
 *
 * Copyright (c) 2013-2014 Derek Buitenhuis
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

#include <x265.h>

#include "libavutil/internal.h"
#include "libavutil/common.h"
#include "libavutil/opt.h"
#include "libavutil/pixdesc.h"
#include "avcodec.h"
#include "internal.h"

typedef struct libx265Context {
    const AVClass *class;

    x265_encoder *encoder;
    x265_param   *params;
    uint8_t      *header;
    int           header_size;

    char *preset;
    char *tune;
    char *x265_opts;
} libx265Context;

static int is_keyframe(NalUnitType naltype)
{
    switch (naltype) {
    case NAL_UNIT_CODED_SLICE_BLA_W_LP:
    case NAL_UNIT_CODED_SLICE_BLA_W_RADL:
    case NAL_UNIT_CODED_SLICE_BLA_N_LP:
    case NAL_UNIT_CODED_SLICE_IDR_W_RADL:
    case NAL_UNIT_CODED_SLICE_IDR_N_LP:
    case NAL_UNIT_CODED_SLICE_CRA:
        return 1;
    default:
        return 0;
    }
}

static av_cold int libx265_encode_close(AVCodecContext *avctx)
{
    libx265Context *ctx = avctx->priv_data;

    av_frame_free(&avctx->coded_frame);
    av_freep(&ctx->header);

    x265_param_free(ctx->params);

    if (ctx->encoder)
        x265_encoder_close(ctx->encoder);

    return 0;
}

static av_cold int libx265_encode_init(AVCodecContext *avctx)
{
    libx265Context *ctx = avctx->priv_data;
    x265_nal *nal;
    uint8_t *buf;
    int nnal;
    int ret;
    int i;

    avctx->coded_frame = av_frame_alloc();
    if (!avctx->coded_frame) {
        av_log(avctx, AV_LOG_ERROR, "Could not allocate frame.\n");
        return AVERROR(ENOMEM);
    }

    ctx->params = x265_param_alloc();
    if (!ctx->params) {
        av_log(avctx, AV_LOG_ERROR, "Could not allocate x265 param structure.\n");
        return AVERROR(ENOMEM);
    }

    if (x265_param_default_preset(ctx->params, ctx->preset, ctx->tune) < 0) {
        av_log(avctx, AV_LOG_ERROR, "Invalid preset or tune.\n");
        return AVERROR(EINVAL);
    }

    ctx->params->frameNumThreads = avctx->thread_count;
    ctx->params->frameRate       = (int) (avctx->time_base.den / avctx->time_base.num);
    ctx->params->sourceWidth     = avctx->width;
    ctx->params->sourceHeight    = avctx->height;
    //ctx->params->inputBitDepth   = av_pix_fmt_desc_get(avctx->pix_fmt)->comp[0].depth_minus1 + 1;
    ctx->params->internalBitDepth   = av_pix_fmt_desc_get(avctx->pix_fmt)->comp[0].depth_minus1 + 1;   // for symbol match x265 library ver.6162:291b3a3

    if (avctx->bit_rate > 0) {
        ctx->params->rc.bitrate         = avctx->bit_rate / 1000;
        ctx->params->rc.rateControlMode = X265_RC_ABR;
    }

    if (ctx->x265_opts) {
        AVDictionary *dict    = NULL;
        AVDictionaryEntry *en = NULL;

        if (!av_dict_parse_string(&dict, ctx->x265_opts, "=", ":", 0)) {
            while ((en = av_dict_get(dict, "", en, AV_DICT_IGNORE_SUFFIX))) {
                int parse_ret = x265_param_parse(ctx->params, en->key, en->value);

                switch (parse_ret) {
                case X265_PARAM_BAD_NAME:
                    av_log(avctx, AV_LOG_WARNING,
                          "Unknown option: %s.\n", en->key);
                    break;
                case X265_PARAM_BAD_VALUE:
                    av_log(avctx, AV_LOG_WARNING,
                          "Invalid value for %s: %s.\n", en->key, en->value);
                    break;
                default:
                    break;
                }
            }
            av_dict_free(&dict);
        }
    }

    ctx->encoder = x265_encoder_open(ctx->params);
    if (!ctx->encoder) {
        av_log(avctx, AV_LOG_ERROR, "Cannot open libx265 encoder.\n");
        libx265_encode_close(avctx);
        return AVERROR_INVALIDDATA;
    }

    ret = x265_encoder_headers(ctx->encoder, &nal, &nnal);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "Cannot encode headers.\n");
        libx265_encode_close(avctx);
        return AVERROR_INVALIDDATA;
    }

    for (i = 0; i < nnal; i++)
        ctx->header_size += nal[i].sizeBytes;

    ctx->header = av_malloc(ctx->header_size);
    if (!ctx->header) {
        av_log(avctx, AV_LOG_ERROR,
               "Cannot allocate HEVC header of size %d.\n", ctx->header_size);
        libx265_encode_close(avctx);
        return AVERROR(ENOMEM);
    }

    buf = ctx->header;
    for (i = 0; i < nnal; i++) {
        memcpy(buf, nal[i].payload, nal[i].sizeBytes);
        buf += nal[i].sizeBytes;
    }

    return 0;
}

static int libx265_encode_frame(AVCodecContext *avctx, AVPacket *pkt,
                                const AVFrame *pic, int *got_packet)
{
    libx265Context *ctx = avctx->priv_data;
    x265_picture x265pic;
    x265_picture x265pic_out = { { 0 } };
    x265_nal *nal;
    uint8_t *dst;
    int payload = 0;
    int nnal;
    int ret;
    int i;

    x265_picture_init(ctx->params, &x265pic);

    if (pic) {
        for (i = 0; i < 3; i++) {
           x265pic.planes[i] = pic->data[i];
           x265pic.stride[i] = pic->linesize[i];
        }

        x265pic.pts = pic->pts;
    }

    ret = x265_encoder_encode(ctx->encoder, &nal, &nnal,
                              pic ? &x265pic : NULL, &x265pic_out);
    if (ret < 0)
        return AVERROR_UNKNOWN;

    if (!nnal)
        return 0;

    for (i = 0; i < nnal; i++)
        payload += nal[i].sizeBytes;

    payload += ctx->header_size;

    ret = ff_alloc_packet(pkt, payload);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "Error getting output packet.\n");
        return ret;
    }
    dst = pkt->data;

    if (ctx->header) {
        memcpy(dst, ctx->header, ctx->header_size);
        dst += ctx->header_size;

        av_freep(&ctx->header);
        ctx->header_size = 0;
    }

    for (i = 0; i < nnal; i++) {
        memcpy(dst, nal[i].payload, nal[i].sizeBytes);
        dst += nal[i].sizeBytes;

        if (is_keyframe(nal[i].type))
            pkt->flags |= AV_PKT_FLAG_KEY;
    }

    pkt->pts = x265pic_out.pts;
    pkt->dts = x265pic_out.dts;

    *got_packet = 1;
    return 0;
}

static const enum AVPixelFormat x265_csp_eight[] = {
    AV_PIX_FMT_YUV420P,
    AV_PIX_FMT_NONE
};

static const enum AVPixelFormat x265_csp_twelve[] = {
    AV_PIX_FMT_YUV420P,
    AV_PIX_FMT_YUV420P10,
    AV_PIX_FMT_NONE
};

static av_cold void libx265_encode_init_csp(AVCodec *codec)
{
    if (x265_max_bit_depth == 8)
        codec->pix_fmts = x265_csp_eight;
    else if (x265_max_bit_depth == 12)
        codec->pix_fmts = x265_csp_twelve;
}

#define OFFSET(x) offsetof(libx265Context, x)
#define VE AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_ENCODING_PARAM
static const AVOption options[] = {
    { "preset",      "set the x265 preset",                                                         OFFSET(preset),    AV_OPT_TYPE_STRING, { 0 }, 0, 0, VE },
    { "tune",        "set the x265 tune parameter",                                                 OFFSET(tune),      AV_OPT_TYPE_STRING, { 0 }, 0, 0, VE },
    { "x265-params", "set the x265 configuration using a :-separated list of key=value parameters", OFFSET(x265_opts), AV_OPT_TYPE_STRING, { 0 }, 0, 0, VE },
    { NULL }
};

static const AVClass class = {
    .class_name = "libx265",
    .item_name  = av_default_item_name,
    .option     = options,
    .version    = LIBAVUTIL_VERSION_INT,
};

AVCodec ff_libx265_encoder = {
    .name             = "libx265",
    .long_name        = NULL_IF_CONFIG_SMALL("libx265 H.265 / HEVC"),
    .type             = AVMEDIA_TYPE_VIDEO,
    .id               = AV_CODEC_ID_HEVC,
    .init             = libx265_encode_init,
    .init_static_data = libx265_encode_init_csp,
    .encode2          = libx265_encode_frame,
    .close            = libx265_encode_close,
    .priv_data_size   = sizeof(libx265Context),
    .priv_class       = &class,
    .capabilities     = CODEC_CAP_DELAY | CODEC_CAP_AUTO_THREADS,
};
