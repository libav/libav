/*
 * RAW HEVC video demuxer
 * Copyright (c) 2012 Guillaume Martres <smarter@ubuntu.com>
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

#include "avformat.h"
#include "rawdec.h"

static int hevc_probe(AVProbeData *p)
{
    return AVPROBE_SCORE_MAX/2;
}

FF_DEF_RAWVIDEO_DEMUXER(hevc, "raw HEVC video format", hevc_probe, "hevc,h265,265", AV_CODEC_ID_HEVC)
