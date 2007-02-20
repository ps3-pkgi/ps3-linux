/*
 *  PS3 hvcall interface.
 *
 *  Copyright (C) 2006 Sony Computer Entertainment Inc.
 *  Copyright 2006 Sony Corp.
 *  Copyright 2003, 2004 (c) MontaVista Software, Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include "ppc_asm.h"

/*
 *  wrapper           lv1
 *  32-bit (h,l)      64-bit
 *
 *  1: r3,r4          <-> r3
 *  2: r5,r6          <-> r4
 *  3: r7,r8          <-> r5
 *  4: r9,r10         <-> r6
 *  5: 8(r1),12(r1)   <-> r7
 *  6: 16(r1),20(r1)  <-> r8
 *  7: 24(r1),28(r1)  <-> r9
 *  8: 32(r1),36(r1)  <-> r10
 *
 */

#define _GLOBAL(n) .section ".text"; .align 2; .globl n; n:
#define lv1call .long 0x44000022; extsw r3, r3;

#define LR_OFFSET 4
#define SAVE_LR() mflr r0; stw r0, LR_OFFSET(r1);
#define LOAD_LR() lwz r0, LR_OFFSET(r1); mtlr r0;

#define LOAD_64_REG(_a,_b,_r) sldi r11, _a, 32; or _r, r11, _b;
#define LOAD_64_STACK(_o,_r) ld _r,_o(r1);

#define LOAD_r3() LOAD_64_REG(r3,r4,r3)
#define LOAD_r4() LOAD_64_REG(r5,r6,r4)
#define LOAD_r5() LOAD_64_REG(r7,r8,r5)
#define LOAD_r6() LOAD_64_REG(r9,r10,r6)
#define LOAD_r7() LOAD_64_STACK(8,r7)
#define LOAD_r8() LOAD_64_STACK(16,r8)
#define LOAD_r9() LOAD_64_STACK(24,r9)
#define LOAD_r10() LOAD_64_STACK(32,r10)

#define LOAD_1() LOAD_r3();
#define LOAD_6()               \
	LOAD_r3(); LOAD_r4();  \
	LOAD_r5(); LOAD_r6();  \
	LOAD_r7(); LOAD_r8();
#define LOAD_7() LOAD_6(); LOAD_r9();
#define LOAD_8() LOAD_7(); LOAD_r10();

#define STORE_6_1() lwz r11, 24(r1); std r4, 0(r11);

#define LV1_N_IN_0_OUT(API_NAME, API_NUMBER)	\
_GLOBAL(_##API_NAME)				\
	SAVE_LR();				\
	LOAD_8();				\
	li      r11, API_NUMBER;		\
	lv1call;				\
	LOAD_LR();				\
	blr

#define LV1_0_IN_0_OUT LV1_N_IN_0_OUT
#define LV1_1_IN_0_OUT LV1_N_IN_0_OUT
#define LV1_2_IN_0_OUT LV1_N_IN_0_OUT
#define LV1_3_IN_0_OUT LV1_N_IN_0_OUT
#define LV1_4_IN_0_OUT LV1_N_IN_0_OUT
#define LV1_5_IN_0_OUT LV1_N_IN_0_OUT
#define LV1_6_IN_0_OUT LV1_N_IN_0_OUT
#define LV1_7_IN_0_OUT LV1_N_IN_0_OUT

#define LV1_0_IN_1_OUT(API_NAME, API_NUMBER)	\
_GLOBAL(_##API_NAME)				\
	b ps3_no_support

#define LV1_0_IN_2_OUT(API_NAME, API_NUMBER)	\
_GLOBAL(_##API_NAME)				\
	b ps3_no_support

#define LV1_0_IN_3_OUT(API_NAME, API_NUMBER)	\
_GLOBAL(_##API_NAME)				\
	b ps3_no_support

#define LV1_0_IN_7_OUT(API_NAME, API_NUMBER)	\
_GLOBAL(_##API_NAME)				\
	b ps3_no_support

#define LV1_1_IN_1_OUT(API_NAME, API_NUMBER)	\
_GLOBAL(_##API_NAME)				\
	b ps3_no_support

#define LV1_1_IN_2_OUT(API_NAME, API_NUMBER)	\
_GLOBAL(_##API_NAME)				\
	b ps3_no_support

#define LV1_1_IN_3_OUT(API_NAME, API_NUMBER)	\
_GLOBAL(_##API_NAME)				\
	b ps3_no_support

#define LV1_1_IN_4_OUT(API_NAME, API_NUMBER)	\
_GLOBAL(_##API_NAME)				\
	b ps3_no_support

#define LV1_1_IN_5_OUT(API_NAME, API_NUMBER)	\
_GLOBAL(_##API_NAME)				\
	b ps3_no_support

#define LV1_1_IN_6_OUT(API_NAME, API_NUMBER)	\
_GLOBAL(_##API_NAME)				\
	b ps3_no_support

#define LV1_1_IN_7_OUT(API_NAME, API_NUMBER)	\
_GLOBAL(_##API_NAME)				\
	b ps3_no_support

#define LV1_2_IN_1_OUT(API_NAME, API_NUMBER)	\
_GLOBAL(_##API_NAME)				\
	b ps3_no_support

#define LV1_2_IN_2_OUT(API_NAME, API_NUMBER)	\
_GLOBAL(_##API_NAME)				\
	b ps3_no_support

#define LV1_2_IN_3_OUT(API_NAME, API_NUMBER)	\
_GLOBAL(_##API_NAME)				\
	b ps3_no_support

#define LV1_2_IN_4_OUT(API_NAME, API_NUMBER)	\
_GLOBAL(_##API_NAME)				\
	b ps3_no_support

#define LV1_2_IN_5_OUT(API_NAME, API_NUMBER)	\
_GLOBAL(_##API_NAME)				\
	b ps3_no_support

#define LV1_3_IN_1_OUT(API_NAME, API_NUMBER)	\
_GLOBAL(_##API_NAME)				\
	b ps3_no_support

#define LV1_3_IN_2_OUT(API_NAME, API_NUMBER)	\
_GLOBAL(_##API_NAME)				\
	b ps3_no_support

#define LV1_3_IN_3_OUT(API_NAME, API_NUMBER)	\
_GLOBAL(_##API_NAME)				\
	b ps3_no_support

#define LV1_4_IN_1_OUT(API_NAME, API_NUMBER)	\
_GLOBAL(_##API_NAME)				\
	b ps3_no_support

#define LV1_4_IN_2_OUT(API_NAME, API_NUMBER)	\
_GLOBAL(_##API_NAME)				\
	b ps3_no_support

#define LV1_4_IN_3_OUT(API_NAME, API_NUMBER)	\
_GLOBAL(_##API_NAME)				\
	b ps3_no_support

#define LV1_5_IN_1_OUT(API_NAME, API_NUMBER)	\
_GLOBAL(_##API_NAME)				\
	b ps3_no_support

#define LV1_5_IN_2_OUT(API_NAME, API_NUMBER)	\
_GLOBAL(_##API_NAME)				\
	b ps3_no_support

#define LV1_5_IN_3_OUT(API_NAME, API_NUMBER)	\
_GLOBAL(_##API_NAME)				\
	b ps3_no_support

#define LV1_6_IN_1_OUT(API_NAME, API_NUMBER)	\
_GLOBAL(_##API_NAME)				\
	SAVE_LR();				\
	LOAD_6();				\
	li      r11, API_NUMBER;		\
	lv1call;				\
	STORE_6_1();				\
	LOAD_LR();				\
	blr

#define LV1_6_IN_2_OUT(API_NAME, API_NUMBER)	\
_GLOBAL(_##API_NAME)				\
	b ps3_no_support

#define LV1_6_IN_3_OUT(API_NAME, API_NUMBER)	\
_GLOBAL(_##API_NAME)				\
	b ps3_no_support

#define LV1_7_IN_1_OUT(API_NAME, API_NUMBER)	\
_GLOBAL(_##API_NAME)				\
	b ps3_no_support

#define LV1_7_IN_6_OUT(API_NAME, API_NUMBER)	\
_GLOBAL(_##API_NAME)				\
	b ps3_no_support

#define LV1_8_IN_1_OUT(API_NAME, API_NUMBER)	\
_GLOBAL(_##API_NAME)				\
	b ps3_no_support

/* the lv1 underscored call definitions expand here */

#define LV1_CALL(name, in, out, num) LV1_##in##_IN_##out##_OUT(lv1_##name, num)
#include "lv1call.h"

