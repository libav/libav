/*
 * ARM optimized DSP utils
 * Copyright (c) 2001 Lionel Ulmer
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

#include "libavutil/attributes.h"
#include "libavutil/arm/cpu.h"
#include "dsputil_arm.h"

void ff_j_rev_dct_arm(int16_t *data);
void ff_simple_idct_arm(int16_t *data);

/* XXX: local hack */
static void (*ff_put_pixels_clamped)(const int16_t *block, uint8_t *pixels, int line_size);
static void (*ff_add_pixels_clamped)(const int16_t *block, uint8_t *pixels, int line_size);

void ff_add_pixels_clamped_arm(const int16_t *block, uint8_t *dest,
                               int line_size);

/* XXX: those functions should be suppressed ASAP when all IDCTs are
   converted */
static void j_rev_dct_arm_put(uint8_t *dest, int line_size, int16_t *block)
{
    ff_j_rev_dct_arm (block);
    ff_put_pixels_clamped(block, dest, line_size);
}
static void j_rev_dct_arm_add(uint8_t *dest, int line_size, int16_t *block)
{
    ff_j_rev_dct_arm (block);
    ff_add_pixels_clamped(block, dest, line_size);
}
static void simple_idct_arm_put(uint8_t *dest, int line_size, int16_t *block)
{
    ff_simple_idct_arm (block);
    ff_put_pixels_clamped(block, dest, line_size);
}
static void simple_idct_arm_add(uint8_t *dest, int line_size, int16_t *block)
{
    ff_simple_idct_arm (block);
    ff_add_pixels_clamped(block, dest, line_size);
}

av_cold void ff_dsputil_init_arm(DSPContext *c, AVCodecContext *avctx)
{
    int cpu_flags = av_get_cpu_flags();

    ff_put_pixels_clamped = c->put_pixels_clamped;
    ff_add_pixels_clamped = c->add_pixels_clamped;

    if (avctx->bits_per_raw_sample <= 8) {
        if(avctx->idct_algo == FF_IDCT_AUTO ||
           avctx->idct_algo == FF_IDCT_ARM){
            c->idct_put              = j_rev_dct_arm_put;
            c->idct_add              = j_rev_dct_arm_add;
            c->idct                  = ff_j_rev_dct_arm;
            c->idct_permutation_type = FF_LIBMPEG2_IDCT_PERM;
        } else if (avctx->idct_algo == FF_IDCT_SIMPLEARM){
            c->idct_put              = simple_idct_arm_put;
            c->idct_add              = simple_idct_arm_add;
            c->idct                  = ff_simple_idct_arm;
            c->idct_permutation_type = FF_NO_IDCT_PERM;
        }
    }

    c->add_pixels_clamped = ff_add_pixels_clamped_arm;

    if (have_armv5te(cpu_flags)) ff_dsputil_init_armv5te(c, avctx);
    if (have_armv6(cpu_flags))   ff_dsputil_init_armv6(c, avctx);
    if (have_neon(cpu_flags))    ff_dsputil_init_neon(c, avctx);
}
