;*****************************************************************************
;* SSE2-optimized HEVC deblocking code
;*****************************************************************************
;* Copyright (C) 2013 VTT
;*
;* Authors: Seppo Tomperi <seppo.tomperi@vtt.fi>
;*
;* This file is part of Libav.
;*
;* Libav is free software; you can redistribute it and/or
;* modify it under the terms of the GNU Lesser General Public
;* License as published by the Free Software Foundation; either
;* version 2.1 of the License, or (at your option) any later version.
;*
;* Libav is distributed in the hope that it will be useful,
;* but WITHOUT ANY WARRANTY; without even the implied warranty of
;* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
;* Lesser General Public License for more details.
;*
;* You should have received a copy of the GNU Lesser General Public
;* License along with Libav; if not, write to the Free Software
;* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
;******************************************************************************

%include "libavutil/x86/x86util.asm"

SECTION_RODATA

SECTION .text
INIT_XMM sse2

; expands to [base],...,[base+7*stride]
%define PASS8ROWS(base, base3, stride, stride3) \
    [base], [base+stride], [base+stride*2], [base3], \
    [base3+stride], [base3+stride*2], [base3+stride3], [base3+stride*4]

%define PASS8ROWS(base, base3, stride, stride3, offset) \
    PASS8ROWS(base+offset, base3+offset, stride, stride3)

; in: 8 rows of 4 bytes in %4..%11
; out: 4 rows of 8 words in m0..m3
%macro TRANSPOSE4x8_LOAD 8
    movd       m0, %1
    movd       m2, %2
    movd       m1, %3
    movd       m3, %4

    punpcklbw  m0, m2
    punpcklbw  m1, m3
    punpcklwd  m0, m1


    movd       m4, %5
    movd       m6, %6
    movd       m5, %7
    movd       m7, %8

    punpcklbw  m4, m6
    punpcklbw  m5, m7
    punpcklwd  m4, m5

    movdqa     m2, m0
    punpckldq  m0, m4
    punpckhdq  m2, m4
    movdqa     m1, m0
    movdqa     m3, m2

    pxor m5, m5
    punpcklbw  m0, m5
    punpckhbw  m1, m5
    punpcklbw  m2, m5
    punpckhbw  m3, m5
%endmacro

; in: 4 rows of 8 words in m0..m3
; out: 8 rows of 4 bytes in %1..%8
%macro TRANSPOSE8x4B_STORE 8
    packuswb   m0, m0
    packuswb   m1, m1
    packuswb   m2, m2
    packuswb   m3, m3

    punpcklbw m0, m1
    punpcklbw m2, m3

    movdqa     m6, m0

    punpcklwd m0, m2
    punpckhwd m6, m2

    movd       %1, m0
    pshufd     m0, m0, 0x39
    movd       %2, m0
    pshufd     m0, m0, 0x39
    movd       %3, m0
    pshufd     m0, m0, 0x39
    movd       %4, m0

    movd       %5, m6
    pshufd     m6, m6, 0x39
    movd       %6, m6
    pshufd     m6, m6, 0x39
    movd       %7, m6
    pshufd     m6, m6, 0x39
    movd       %8, m6
%endmacro

; in: 8 rows of 8 bytes in %1..%8
; out: 8 rows of 8 words in m0..m7
%macro TRANSPOSE8x8_LOAD 8
    movq       m7, %1
    movq       m2, %2
    movq       m1, %3
    movq       m3, %4

    punpcklbw  m7, m2
    punpcklbw  m1, m3
    movdqa     m3, m7
    punpcklwd  m3, m1
    punpckhwd  m7, m1

    movq       m4, %5
    movq       m6, %6
    movq       m5, %7
    movq      m15, %8

    punpcklbw  m4, m6
    punpcklbw  m5, m15
    movdqa     m9, m4
    punpcklwd  m9, m5
    punpckhwd  m4, m5

    movdqa     m1, m3
    punpckldq  m1, m9;  0, 1
    punpckhdq  m3, m9;  2, 3

    movdqa     m5, m7
    punpckldq  m5, m4;  4, 5
    punpckhdq  m7, m4;  6, 7

    pxor m13, m13

    movdqa     m0, m1
    punpcklbw  m0, m13; 0 in 16 bit
    punpckhbw  m1, m13; 1 in 16 bit

    movdqa     m2, m3;
    punpcklbw  m2, m13; 2
    punpckhbw  m3, m13; 3

    movdqa     m4, m5;
    punpcklbw  m4, m13; 4
    punpckhbw  m5, m13; 5

    movdqa     m6, m7
    punpcklbw  m6, m13; 6
    punpckhbw  m7, m13; 7
%endmacro


; in: 8 rows of 8 words in m0..m8
; out: 8 rows of 8 bytes in %1..%8
%macro TRANSPOSE8x8B_STORE 8

    movdqa m13, m11
    movd   r9, m13;
    and    r9, 0xffff; 1dq3
    pshufd m13, m13, 0x39
    movd   r10, m13;
    shr    r10, 16; 1dq0
    add    r9, r10 ; 1dq0 + 1dq3
    pshufd m13, m13, 0x39
    movd   r10, m13;
    and    r10, 0xffff; 0dq3
    pshufd m13, m13, 0x39
    movd   r11, m13;
    shr    r11, 16; 0dq0
    add    r10, r11; 0dq0 + 0dq3

    packuswb   m0, m0
    packuswb   m1, m1
    packuswb   m2, m2
    packuswb   m3, m3
    packuswb   m4, m4
    packuswb   m5, m5
    packuswb   m6, m6
    packuswb   m7, m7

    punpcklbw m0, m1
    punpcklbw m2, m3

    movdqa    m8, m0
    punpcklwd m0, m2
    punpckhwd m8, m2


    punpcklbw m4, m5
    punpcklbw m6, m7

    movdqa    m9, m4
    punpcklwd m4, m6
    punpckhwd m9, m6

    movdqa     m10, m0
    punpckldq  m0, m4;   0, 1
    punpckhdq   m10, m4; 2, 3

    movdqa     m11, m8
    punpckldq  m11, m9;  4, 5
    punpckhdq  m8, m9;   6, 7
    movq       %1, m0
    pshufd     m0, m0, 0x4E
    movq       %2, m0
    movq       %3, m10
    pshufd     m10, m10, 0x4E
    movq       %4, m10
    movq       %5, m11
    pshufd     m11, m11, 0x4E
    movq       %6, m11
    movq       %7, m8
    pshufd     m8, m8, 0x4E
    movq       %8, m8
%endmacro



; in: %2 clobbered
; out: %1
; mask in m11
; clobbers m10
%macro MASKED_COPY 2
    pand  %2, m11 ; and mask
    movdqa m10, m11
    pandn m10, %1; and -mask
    por %2, m10
    movdqa %1, %2
%endmacro

; in: %2 clobbered
; out: %1
; mask in %3, will be clobbered
%macro MASKED_COPY2 3
    pand  %2, %3 ; and mask
    pandn %3, %1; and -mask
    por %2, %3
    movdqa %1, %2
%endmacro

ALIGN 16
; input in m0 ... m3 and tcs in r2. Output in m1 and m2
ff_hevc_chroma_deblock_body:
    movdqa    m4, m2; temp copy of q0
    movdqa    m5, m0; temp copy of p1
    psubw     m4, m1; q0 - p0
    psubw     m5, m3; p1 - q1
    psllw     m4, 2; << 2
    paddw     m5, m4;

    ;tc calculations
    movd      m6, [r2]; tc0
    add       r2, 4;
    punpcklwd m6, m6
    movd      m7, [r2]; tc1
    punpcklwd m7, m7
    shufps    m6, m7, 0; tc0, tc1
    movdqa    m4, m6
    pcmpeqw   m7, m7; set all bits to 1
    pxor      m4, m7; flip all bits of first reg
    psrlw     m7, 15; 1 in every cell
    paddw     m4, m7; -tc0, -tc1
    ;end tc calculations

    psllw     m7, 2; 4 in every cell
    paddw     m5, m7; +4
    psraw     m5, 3; >> 3

    pmaxsw    m5, m4
    pminsw    m5, m6
    paddw     m1, m5; p0 + delta0
    psubw     m2, m5; q0 - delta0
    ret

INIT_XMM ssse3
ALIGN 16
; input in m0 ... m7, betas in r2 tcs in r3. Output in m1...m6
ff_hevc_luma_deblock_body:
    movdqa m9, m2
    psllw  m9, 1; *2
    movdqa   m10, m1
    psubw  m10, m9
    paddw  m10, m3
    pabsw  m10, m10 ; 0dp0, 0dp3 , 1dp0, 1dp3

    movdqa m9, m5
    psllw  m9, 1; *2
    movdqa m11, m6
    psubw  m11, m9
    paddw  m11, m4
    pabsw  m11, m11 ; 0dq0, 0dq3 , 1dq0, 1dq3

    ;beta calculations
    movd      m13, [r2]; beta0
    mov       r11, [r2];
    add       r2, 4;
    punpcklwd m13, m13
    movd      m14, [r2]; beta1
    mov       r12, [r2];
    punpcklwd m14, m14
    pshufd    m13, m14, 0; beta0, beta1
    ;end beta calculations

    movdqa m9, m10
    paddw  m9, m11;   0d0, 0d3  ,  1d0, 1d3

    pshufhw m14, m9, 0x0f ;0b00001111;  0d3 0d3 0d0 0d0 in high
    pshuflw m14, m14, 0x0f ;0b00001111;  1d3 1d3 1d0 1d0 in low

    pshufhw m9, m9, 0xf0 ;0b11110000; 0d0 0d0 0d3 0d3
    pshuflw m9, m9, 0xf0 ;0b11110000; 1d0 1d0 1d3 1d3

    paddw m14, m9; 0d0+0d3, 1d0+1d3
    movdqa m15, m13; beta0, beta1

    ;compare
    pcmpgtw m15, m14
    movmskps r13, m15 ;filtering mask 0d0 + 0d3 < beta0 (bit 2 or 3) , 1d0 + 1d3 < beta1 (bit 0 or 1)
    cmp r13, 0
    je bypasswrite

    ;weak / strong decision compare to beta_2
    movdqa m15, m13; beta0, beta1
    psraw  m15, 2;   beta >> 2
    movdqa m8, m9;
    psllw  m8, 1;
    pcmpgtw m15, m8; (d0 << 1) < beta_2, (d3 << 1) < beta_2
    movmskps r14, m15;
    ;end weak / strong decision

    ; weak filter nd_p/q calculation
    pshufd  m8, m10, 0x31
    psrld   m8, 16
    paddw   m8, m10
    movd    r7, m8
    and     r7, 0xffff; 1dp0 + 1dp3
    pshufd  m8, m8, 0x4E
    movd    r8, m8
    and     r8, 0xffff; 0dp0 + 0dp3

    pshufd  m8, m11, 0x31
    psrld   m8, 16
    paddw   m8, m11
    movd    r9, m8
    and     r9, 0xffff; 1dq0 + 1dq3
    pshufd  m8, m8, 0x4E
    movd   r10, m8
    and    r10, 0xffff; 0dq0 + 0dq3
    ; end calc for weak filter

    ; filtering mask
    mov      r2, r13
    shr      r2, 3
    movd     m15, r2
    and      r13, 1
    movd      m11, r13
    shufps   m11, m15, 0
    shl       r2, 1
    or        r13, r2

    pcmpeqd m15, m15; set all bits to 1
    psrld   m15, 31; set to 32bit 1
    pcmpeqd  m11, m15; filtering mask

    ;decide between strong and weak filtering
    ;tc25 calculations
    movd      m8, [r3]; tc0
    mov       r2, [r3];
    add       r3, 4;
    punpcklwd m8, m8
    movd      m9, [r3]; tc1
    add       r2, [r3]; tc0 + tc1
    cmp       r2, 0;
    je        bypasswrite
    punpcklwd m9, m9
    shufps    m8, m9, 0; tc0, tc1
    movdqa    m9, m8
    psllw     m8, 2; tc << 2
    pavgw     m8, m9; tc25 = ((tc * 5 + 1) >> 1)
    ;end tc25 calculations

    ;----beta_3 comparison-----
    movdqa    m12, m0;      p3
    psubw     m12, m3;      p3 - p0
    pabsw     m12, m12; abs(p3 - p0)

    movdqa    m15, m7;      q3
    psubw     m15, m4;      q3 - q0
    pabsw     m15, m15; abs(q3 - q0)

    paddw     m12, m15; abs(p3 - p0) + abs(q3 - q0)

    pshufhw   m12, m12, 0xf0 ;0b11110000;
    pshuflw   m12, m12, 0xf0 ;0b11110000;

    psraw     m13, 3; beta >> 3
    pcmpgtw   m13, m12;
    movmskps   r2, m13;
    and       r14, r2; strong mask , beta_2 and beta_3 comparisons
    ;----beta_3 comparison end-----
    ;----tc25 comparison---
    movdqa    m12, m3;      p0
    psubw     m12, m4;      p0 - q0
    pabsw     m12, m12; abs(p0 - q0)

    pshufhw   m12, m12, 0xf0 ;0b11110000;
    pshuflw   m12, m12, 0xf0 ;0b11110000;

    pcmpgtw    m8, m12; tc25 comparisons
    movmskps   r2, m8;
    and       r14, r2; strong mask, beta_2, beta_3 and tc25 comparisons
    ;----tc25 comparison end---
    mov        r2, r14;
    shr        r2, 1;
    and       r14, r2; strong mask, bits 2 and 0

    pcmpeqw   m13, m13; set all bits to 1
    movdqa    m14, m9; tc
    pxor      m14, m13; invert bits
    psrlw     m13, 15; 1 in every cell
    paddw     m14, m13; -tc

    psllw      m9, 1;  tc * 2
    psllw     m14, 1; -tc * 2

    and      r14, 5; 0b101
    mov       r2, r14; strong mask
    shr      r14, 2;
    movd     m12, r14; store to xmm for mask generation
    shl      r14, 1
    and       r2, 1
    movd     m10, r2; store to xmm for mask generation
    or       r14, r2; final strong mask, bits 1 and 0
    cmp      r14, 0;
    je      weakfilter

    shufps   m10, m12, 0

    pcmpeqd m12, m12; set all bits to 1
    psrld   m12, 31; set to 32bit 1
    pcmpeqd  m10, m12; strong mask

    psllw    m13, 2; 4 in every cell
    pand     m11, m10; combine filtering mask and strong mask
    movdqa   m12, m2;          p1
    paddw    m12, m3;          p1 +   p0
    paddw    m12, m4;          p1 +   p0 +   q0
    movdqa   m10, m12; copy
    psllw    m12, 1;         2*p1 + 2*p0 + 2*q0
    paddw    m12, m1;   p2 + 2*p1 + 2*p0 + 2*q0
    paddw    m12, m5;   p2 + 2*p1 + 2*p0 + 2*q0 + q1
    paddw    m12, m13;  p2 + 2*p1 + 2*p0 + 2*q0 + q1 + 4
    psraw    m12, 3;  ((p2 + 2*p1 + 2*p0 + 2*q0 + q1 + 4) >> 3)
    psubw    m12, m3; ((p2 + 2*p1 + 2*p0 + 2*q0 + q1 + 4) >> 3) - p0
    pmaxsw   m12, m14
    pminsw   m12, m9; av_clip( , -2 * tc, 2 * tc)
    paddw    m12, m3; p0'

    movdqa   m15, m1;  p2
    paddw    m15, m10; p2 + p1 + p0 + q0
    psrlw    m13, 1; 2 in every cell
    paddw    m15, m13; p2 + p1 + p0 + q0 + 2
    psraw    m15, 2;  (p2 + p1 + p0 + q0 + 2) >> 2
    psubw    m15, m2;((p2 + p1 + p0 + q0 + 2) >> 2) - p1
    pmaxsw   m15, m14
    pminsw   m15, m9; av_clip( , -2 * tc, 2 * tc)
    paddw    m15, m2; p1'

    movdqa   m8, m1;            p2
    paddw    m8, m0;     p3 +   p2
    psllw    m8, 1;    2*p3 + 2*p2
    paddw    m8, m1;   2*p3 + 3*p2
    paddw    m8, m10;  2*p3 + 3*p2 + p1 + p0 + q0
    psllw    m13, 1; 4 in every cell
    paddw    m8, m13;  2*p3 + 3*p2 + p1 + p0 + q0 + 4
    psraw    m8, 3;   (2*p3 + 3*p2 + p1 + p0 + q0 + 4) >> 3
    psubw    m8, m1; ((2*p3 + 3*p2 + p1 + p0 + q0 + 4) >> 3) - p2
    pmaxsw   m8, m14
    pminsw   m8, m9; av_clip( , -2 * tc, 2 * tc)
    paddw    m8, m1; p2'
    MASKED_COPY m1, m8

    movdqa   m8, m3;         p0
    paddw    m8, m4;         p0 +   q0
    paddw    m8, m5;         p0 +   q0 +   q1
    psllw    m8, 1;        2*p0 + 2*q0 + 2*q1
    paddw    m8, m2;  p1 + 2*p0 + 2*q0 + 2*q1
    paddw    m8, m6;  p1 + 2*p0 + 2*q0 + 2*q1 + q2
    paddw    m8, m13; p1 + 2*p0 + 2*q0 + 2*q1 + q2 + 4
    psraw    m8, 3;  (p1 + 2*p0 + 2*q0 + 2*q1 + q2 + 4) >>3
    psubw    m8, m4;
    pmaxsw   m8, m14
    pminsw   m8, m9; av_clip( , -2 * tc, 2 * tc)
    paddw    m8, m4; q0'
    MASKED_COPY m2, m15

    movdqa   m15, m3;   p0
    paddw    m15, m4;   p0 + q0
    paddw    m15, m5;   p0 + q0 + q1
    movdqa   m10, m15;
    paddw    m15, m6;   p0 + q0 + q1 + q2
    psrlw    m13, 1; 2 in every cell
    paddw    m15, m13;  p0 + q0 + q1 + q2 + 2
    psraw    m15, 2;   (p0 + q0 + q1 + q2 + 2) >> 2
    psubw    m15, m5; ((p0 + q0 + q1 + q2 + 2) >> 2) - q1
    pmaxsw   m15, m14
    pminsw   m15, m9; av_clip( , -2 * tc, 2 * tc)
    paddw    m15, m5; q1'

    paddw    m13, m7;      q3 + 2
    paddw    m13, m6;      q3 +  q2 + 2
    psllw    m13, 1;     2*q3 + 2*q2 + 4
    paddw    m13, m6;    2*q3 + 3*q2 + 4
    paddw    m13, m10;   2*q3 + 3*q2 + q1 + q0 + p0 + 4
    psraw    m13, 3;    (2*q3 + 3*q2 + q1 + q0 + p0 + 4) >> 3
    psubw    m13, m6;  ((2*q3 + 3*q2 + q1 + q0 + p0 + 4) >> 3) - q2
    pmaxsw   m13, m14
    pminsw   m13, m9; av_clip( , -2 * tc, 2 * tc)
    paddw    m13, m6; q2'

    MASKED_COPY m6, m13
    MASKED_COPY m5, m15
    MASKED_COPY m4, m8
    MASKED_COPY m3, m12

weakfilter:
    not      r14; strong mask -> weak mask
    and      r14, r13; final weak filtering mask, bits 0 and 1
    cmp      r14, 0;
    je      ready

    ; weak filtering mask
    mov       r2, r14
    shr       r2, 1
    movd     m12, r2
    and      r14, 1
    movd     m11, r14
    shufps   m11, m12, 0

    pcmpeqd m12, m12; set all bits to 1
    psrld   m12, 31; set to 32bit 1
    pcmpeqd  m11, m12; filtering mask

    mov r13, r11; beta0
    shr r13, 1;
    add r11, r13
    shr r11, 3; ((beta0+(beta0>>1))>>3))

    mov r13, r12; beta1
    shr r13, 1;
    add r12, r13
    shr r12, 3; ((beta1+(beta1>>1))>>3))

    pcmpeqw   m13, m13; set all bits to 1
    psrlw     m13, 15; 1 in every cell
    psllw     m13, 3; 8 in every cell

    movdqa    m12, m4 ; q0
    psubw     m12, m3 ; q0 - p0
    movdqa    m10, m12
    psllw     m10, 3; 8 * (q0 - p0)
    paddw     m12, m10 ; 9 * (q0 - p0)

    movdqa    m10, m5 ; q1
    psubw     m10, m2 ; q1 - p1
    movdqa    m8, m10
    psllw     m8, 1; 2 * ( q1 - p1 )
    paddw     m10, m8; 3 * ( q1 - p1 )
    psubw     m12, m10; 9 * (q0 - p0) - 3 * ( q1 - p1 )
    paddw     m12, m13; + 8
    psraw     m12, 4; >> 4 , delta0
    pabsw     m13, m12; abs(delta0)


    movdqa    m10, m9; 2*tc
    psllw     m10, 2; 8 * tc
    paddw     m10, m9; 10 * tc
    pcmpgtw   m10, m13
    pand      m11, m10

    psraw     m9, 1;   tc * 2 -> tc
    psraw     m14, 1; -tc * 2 -> -tc

    pmaxsw    m12, m14
    pminsw    m12, m9;  av_clip(delta0, -tc, tc)

    pcmpeqw   m13, m13; set all bits to 1
    psraw     m9, 1;   tc -> tc / 2
    movdqa    m14, m9;
    pxor      m14, m13; complement -tc
    psrlw     m13, 15; set all cells to 1
    paddw     m14, m13; add 1, -tc / 2

    movdqa    m15, m1;    p2
    pavgw     m15, m3;   (p2 + p0 + 1) >> 1
    psubw     m15, m2;  ((p2 + p0 + 1) >> 1) - p1
    paddw     m15, m12; ((p2 + p0 + 1) >> 1) - p1 + delta0
    psraw     m15, 1;   (((p2 + p0 + 1) >> 1) - p1 + delta0) >> 1
    pmaxsw    m15, m14
    pminsw    m15, m9; av_clip(deltap1, -tc/2, tc/2)
    paddw     m15, m2; p1'

    ;beta calculations
    movd      m10, r11; beta0
    punpcklwd m10, m10
    movd      m13, r12; beta1
    punpcklwd m13, m13
    shufps    m10, m13, 0; betax0, betax1

    movd      m13, r7; 1dp0 + 1dp3
    movd       m8, r8; 0dp0 + 0dp3
    punpcklwd  m8, m8
    punpcklwd m13, m13
    shufps    m13, m8, 0;
    movdqa     m8, m10; copy of beta
    pcmpgtw   m8, m13
    pand      m8, m11
    ;end beta calculations
    MASKED_COPY2 m2, m15, m8; write p1'

    movdqa     m8, m6;    q2
    pavgw      m8, m4;   (q2 + q0 + 1) >> 1
    psubw      m8, m5;  ((q2 + q0 + 1) >> 1) - q1
    psubw      m8, m12; ((q2 + q0 + 1) >> 1) - q1 - delta0)
    psraw      m8, 1;   ((q2 + q0 + 1) >> 1) - q1 - delta0) >> 1
    pmaxsw     m8, m14
    pminsw     m8, m9; av_clip(deltaq1, -tc/2, tc/2)
    paddw      m8, m5; q1'

    movd      m13, r9;
    movd      m15, r10;
    punpcklwd m15, m15
    punpcklwd m13, m13
    shufps    m13, m15, 0; dq0 + dq3

    pcmpgtw   m10, m13; compare to ((beta+(beta>>1))>>3)
    pand      m10, m11
    MASKED_COPY2 m5, m8, m10; write q1'

    movdqa   m15, m3 ; p0
    paddw    m15, m12 ; p0 + delta0
    MASKED_COPY m3, m15

    movdqa   m8, m4 ; q0
    psubw    m8, m12 ; q0 - delta0
    MASKED_COPY m4, m8
ready:
    mov r4, 0
    ret
bypasswrite:
    mov r4, 1
    ret

INIT_XMM sse2
;-----------------------------------------------------------------------------
; void ff_hevc_v_loop_filter_chroma(uint8_t *_pix, ptrdiff_t _stride, int *_tc, uint8_t *_no_p, uint8_t *_no_q)
;-----------------------------------------------------------------------------
cglobal hevc_v_loop_filter_chroma_8, 3, 6, 8
    sub    r0, 2
    lea    r5, [3*r1]
    mov    r4, r0
    add    r0, r5
    TRANSPOSE4x8_LOAD  PASS8ROWS(r4, r0, r1, r5)
    call ff_hevc_chroma_deblock_body
    TRANSPOSE8x4B_STORE PASS8ROWS(r4, r0, r1, r5)
    RET

;-----------------------------------------------------------------------------
; void ff_hevc_h_loop_filter_chroma(uint8_t *_pix, ptrdiff_t _stride, int *_tc, uint8_t *_no_p, uint8_t *_no_q
;-----------------------------------------------------------------------------
cglobal hevc_h_loop_filter_chroma_8, 3, 6, 8
    mov       r5, r0; pix
    sub       r5, r1
    sub       r5, r1
    movq      m0, [r5];    p1
    movq      m1, [r5+r1]; p0
    movq      m2, [r0];    q0
    movq      m3, [r0+r1]; q1
    pxor      m5, m5; zeros reg
    punpcklbw m0, m5
    punpcklbw m1, m5
    punpcklbw m2, m5
    punpcklbw m3, m5
    call ff_hevc_chroma_deblock_body
    packuswb m1, m1 ; p0' packed in bytes on low quadword
    packuswb m2, m2 ; q0' packed in bytes on low quadword
    movq [r5+r1], m1
    movq [r0], m2
    RET

INIT_XMM ssse3
;-----------------------------------------------------------------------------
;    void ff_hevc_v_loop_filter_luma(uint8_t *_pix, ptrdiff_t _stride, int *_beta, int *_tc, uint8_t *_no_p, uint8_t *_no_q);
;-----------------------------------------------------------------------------
cglobal hevc_v_loop_filter_luma_8, 4, 15, 16
    sub    r0, 4
    lea    r5, [3*r1]
    mov    r6, r0
    add    r0, r5
    TRANSPOSE8x8_LOAD  PASS8ROWS(r6, r0, r1, r5)
    call ff_hevc_luma_deblock_body
    cmp r4, 1
    je bypassvluma
    TRANSPOSE8x8B_STORE PASS8ROWS(r6, r0, r1, r5)
bypassvluma:
    RET

;-----------------------------------------------------------------------------
;    void ff_hevc_h_loop_filter_luma(uint8_t *_pix, ptrdiff_t _stride, int *_beta, int *_tc, uint8_t *_no_p, uint8_t *_no_q);
;-----------------------------------------------------------------------------
cglobal hevc_h_loop_filter_luma_8, 4, 15, 16
    lea       r6, [3*r1]
    mov       r5, r0
    sub       r5, r6
    sub       r5, r1
    movq      m0, [r5];       p3
    movq      m1, [r5+r1];    p2
    movq      m2, [r5+2*r1];  p1
    movq      m3, [r5+r6];    p0
    movq      m4, [r0];       q0
    movq      m5, [r0+r1];    q1
    movq      m6, [r0+2*r1];  q2
    movq      m7, [r0+r6];    q3
    pxor m8, m8
    punpcklbw m0, m8
    punpcklbw m1, m8
    punpcklbw m2, m8
    punpcklbw m3, m8
    punpcklbw m4, m8
    punpcklbw m5, m8
    punpcklbw m6, m8
    punpcklbw m7, m8
    call ff_hevc_luma_deblock_body
    cmp r4, 1
    je bypasshluma
    packuswb m1, m1; p2
    packuswb m2, m2; p1
    packuswb m3, m3; p0
    packuswb m4, m4; q0
    packuswb m5, m5; q1
    packuswb m6, m6; q2
    movq     [r5+r1],   m1;  p2
    movq     [r5+2*r1], m2;  p1
    movq     [r5+r6],   m3;  p0
    movq     [r0],      m4;  q0
    movq     [r0+r1],   m5;  q1
    movq     [r0+2*r1], m6;  q2
bypasshluma:
    RET
