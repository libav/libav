/*
 * Copyright (C) 2007 Marc Hoffman <marc.hoffman@analog.com>
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

#include "libavutil/attributes.h"
#include "libavcodec/avcodec.h"
#include "libavcodec/vp3dsp.h"
#include "libavcodec/dsputil.h"
#include "dsputil_bfin.h"
#include "vp3_bfin.h"

/* Intra iDCT offset 128 */
static void bfin_vp3_idct_put(uint8_t *dest, int line_size, int16_t *block)
{
    const uint8_t *cm = ff_cropTbl + MAX_NEG_CROP + 128;
    int i,j;

    ff_bfin_vp3_idct (block);

    for (i=0;i<8;i++)
        for (j=0;j<8;j++)
            dest[line_size*i + j] = cm[block[j*8 + i]];

    memset(block, 0, 128);
}

/* Inter iDCT */
static void bfin_vp3_idct_add(uint8_t *dest, int line_size, int16_t *block)
{
    const uint8_t *cm = ff_cropTbl + MAX_NEG_CROP;
    int i, j;

    ff_bfin_vp3_idct (block);
    for (i = 0; i < 8; i++)
        for (j = 0; j < 8; j++)
            dest[line_size*i + j] = cm[dest[line_size*i + j] + block[j*8 + i]];

    memset(block, 0, 128);
}

av_cold void ff_vp3dsp_init_bfin(VP3DSPContext *c, int flags)
{
    if (!(flags & CODEC_FLAG_BITEXACT)) {
        c->idct_add = bfin_vp3_idct_add;
        c->idct_put = bfin_vp3_idct_put;
    }
}
