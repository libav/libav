/*
 * HQX DSP routines
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
 * HQX DSP routines
 */

#ifndef AVCODEC_HQXDSP_H
#define AVCODEC_HQXDSP_H

#include <stdint.h>

typedef struct HQXDSPContext {
    void (*idct_put)(uint16_t *dst, ptrdiff_t stride,
                     int16_t *block, const uint8_t *quant);
} HQXDSPContext;

void ff_hqxdsp_init(HQXDSPContext *c);

#endif /* AVCODEC_HQXDSP_H */
