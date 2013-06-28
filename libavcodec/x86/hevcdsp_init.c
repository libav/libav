/*
 * Copyright (c) 2013 Seppo Tomperi
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

#include "libavutil/cpu.h"
#include "libavutil/x86/asm.h"
#include "libavutil/x86/cpu.h"
#include "libavcodec/get_bits.h" /* required for hevcdsp.h GetBitContext */
#include "libavcodec/hevcdsp.h"


/***********************************/
/* deblocking */

#define LFC_FUNC(DIR, DEPTH, OPT)                                        \
void ff_hevc_ ## DIR ## _loop_filter_chroma_ ## DEPTH ## _ ## OPT(uint8_t *_pix, ptrdiff_t _stride, int *_tc, uint8_t *_no_p, uint8_t *_no_q);

#define LFL_FUNC(DIR, DEPTH, OPT)                                        \
void ff_hevc_ ## DIR ## _loop_filter_luma_ ## DEPTH ## _ ## OPT(uint8_t *_pix, ptrdiff_t stride, int *_beta, int *_tc, \
uint8_t *_no_p, uint8_t *_no_q);

#define LFC_FUNCS(type, depth) \
LFC_FUNC(h, depth, sse2)  \
LFC_FUNC(v, depth, sse2)

#define LFL_FUNCS(type, depth) \
LFL_FUNC(h, depth, ssse3)  \
LFL_FUNC(v, depth, ssse3)


LFC_FUNCS(uint8_t,   8)
LFL_FUNCS(uint8_t,   8)

//LF_FUNCS(uint16_t, 10)


void ff_hevcdsp_init_x86(HEVCDSPContext *c, const int bit_depth, const int pcm_deblock)
{
    int mm_flags = av_get_cpu_flags();

    if (bit_depth == 8) {
        if (EXTERNAL_MMX(mm_flags)) {
            /*if (mm_flags & AV_CPU_FLAG_CMOV)
                c->h264_luma_dc_dequant_idct = ff_h264_luma_dc_dequant_idct_mmx; */

            if (EXTERNAL_MMXEXT(mm_flags)) {
#if ARCH_X86_32 && HAVE_MMXEXT_EXTERNAL
                /* MMEXT optimizations */
#endif /* ARCH_X86_32 && HAVE_MMXEXT_EXTERNAL */

                if (EXTERNAL_SSE2(mm_flags)) {
                    if (!pcm_deblock) {
                        c->hevc_v_loop_filter_chroma = ff_hevc_v_loop_filter_chroma_8_sse2;
                        c->hevc_h_loop_filter_chroma = ff_hevc_h_loop_filter_chroma_8_sse2;
                    }
                }
                if (EXTERNAL_SSSE3(mm_flags)) {
#if ARCH_X86_64
                        c->hevc_v_loop_filter_luma = ff_hevc_v_loop_filter_luma_8_ssse3;
                        c->hevc_h_loop_filter_luma = ff_hevc_h_loop_filter_luma_8_ssse3;
#endif

                }
                if (EXTERNAL_AVX(mm_flags)) {
                }
            }
        }
    } else if (bit_depth == 10) {
        if (EXTERNAL_MMX(mm_flags)) {
            if (EXTERNAL_MMXEXT(mm_flags)) {
#if ARCH_X86_32
#endif /* ARCH_X86_32 */
                if (EXTERNAL_SSE2(mm_flags)) {
#if HAVE_ALIGNED_STACK
                    /*stuff that requires aligned stack */
#endif /* HAVE_ALIGNED_STACK */
                }
                if (EXTERNAL_SSE4(mm_flags)) {
                }
                if (EXTERNAL_AVX(mm_flags)) {
                }
            }
        }
    }
}
