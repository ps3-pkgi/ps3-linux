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

.macro global name
	.section ".text"
	.balign 4
	.globl \name
\name:
.endm

.macro no_support name
	GLOBAL \name
	b ps3_no_support
.endm

.macro hvcall
	.long 0x44000022
	extsw r3, r3
.endm

.macro save_lr offset=4
	mflr r0
	stw r0, \offset(r1)
.endm

.macro load_lr offset=4
	lwz r0, \offset(r1)
	mtlr r0
.endm

.macro load_64_reg target,high,low
	sldi r11, \high, 32
	or \target, r11, \low
.endm

.macro load_64_stack target,offset
	ld \target, \offset(r1)
.endm

.macro load_r3
	LOAD_64_REG r3,r3,r4
.endm

.macro load_r4
	LOAD_64_REG r4,r5,r6
.endm

.macro load_r5
	LOAD_64_REG r5,r7,r8
.endm

.macro load_r6
	LOAD_64_REG r6,r9,r10
.endm

.macro load_r7
	LOAD_64_STACK r7,8
.endm

.macro load_r8
	LOAD_64_STACK r8,16
.endm

.macro load_r9
	LOAD_64_STACK r9,24
.endm

.macro load_r10
	LOAD_64_STACK r10,32
.endm

.macro load_regs_6
	LOAD_R3
	LOAD_R4
	LOAD_R5
	LOAD_R6
	LOAD_R7
	LOAD_R8
.endm

.macro load_regs_8
	LOAD_REGS_6
	LOAD_R9
	LOAD_R10
.endm

.macro store_regs_6_1
	lwz r11, 24(r1)
	std r4, 0(r11)
.endm

#define LV1_N_IN_0_OUT(API_NAME, API_NUMBER)	\
GLOBAL _##API_NAME;				\
	SAVE_LR;				\
	LOAD_REGS_8;				\
	li      r11, API_NUMBER;		\
	HVCALL;					\
	LOAD_LR;				\
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
	NO_SUPPORT _##API_NAME

#define LV1_0_IN_2_OUT(API_NAME, API_NUMBER)	\
	NO_SUPPORT _##API_NAME

#define LV1_0_IN_3_OUT(API_NAME, API_NUMBER)	\
	NO_SUPPORT _##API_NAME

#define LV1_0_IN_7_OUT(API_NAME, API_NUMBER)	\
	NO_SUPPORT _##API_NAME

#define LV1_1_IN_1_OUT(API_NAME, API_NUMBER)	\
	NO_SUPPORT _##API_NAME

#define LV1_1_IN_2_OUT(API_NAME, API_NUMBER)	\
	NO_SUPPORT _##API_NAME

#define LV1_1_IN_3_OUT(API_NAME, API_NUMBER)	\
	NO_SUPPORT _##API_NAME

#define LV1_1_IN_4_OUT(API_NAME, API_NUMBER)	\
	NO_SUPPORT _##API_NAME

#define LV1_1_IN_5_OUT(API_NAME, API_NUMBER)	\
	NO_SUPPORT _##API_NAME

#define LV1_1_IN_6_OUT(API_NAME, API_NUMBER)	\
	NO_SUPPORT

#define LV1_1_IN_7_OUT(API_NAME, API_NUMBER)	\
	NO_SUPPORT _##API_NAME

#define LV1_2_IN_1_OUT(API_NAME, API_NUMBER)	\
	NO_SUPPORT _##API_NAME

#define LV1_2_IN_2_OUT(API_NAME, API_NUMBER)	\
	NO_SUPPORT _##API_NAME

#define LV1_2_IN_3_OUT(API_NAME, API_NUMBER)	\
	NO_SUPPORT _##API_NAME

#define LV1_2_IN_4_OUT(API_NAME, API_NUMBER)	\
	NO_SUPPORT _##API_NAME

#define LV1_2_IN_5_OUT(API_NAME, API_NUMBER)	\
	NO_SUPPORT _##API_NAME

#define LV1_3_IN_1_OUT(API_NAME, API_NUMBER)	\
	NO_SUPPORT _##API_NAME

#define LV1_3_IN_2_OUT(API_NAME, API_NUMBER)	\
	NO_SUPPORT _##API_NAME

#define LV1_3_IN_3_OUT(API_NAME, API_NUMBER)	\
	NO_SUPPORT _##API_NAME

#define LV1_4_IN_1_OUT(API_NAME, API_NUMBER)	\
	NO_SUPPORT _##API_NAME

#define LV1_4_IN_2_OUT(API_NAME, API_NUMBER)	\
	NO_SUPPORT _##API_NAME

#define LV1_4_IN_3_OUT(API_NAME, API_NUMBER)	\
	NO_SUPPORT _##API_NAME

#define LV1_5_IN_1_OUT(API_NAME, API_NUMBER)	\
	NO_SUPPORT _##API_NAME

#define LV1_5_IN_2_OUT(API_NAME, API_NUMBER)	\
	NO_SUPPORT _##API_NAME

#define LV1_5_IN_3_OUT(API_NAME, API_NUMBER)	\
	NO_SUPPORT _##API_NAME

#define LV1_6_IN_1_OUT(API_NAME, API_NUMBER)	\
GLOBAL _##API_NAME;				\
	SAVE_LR;				\
	LOAD_REGS_6;				\
	li      r11, API_NUMBER;		\
	HVCALL;					\
	STORE_REGS_6_1;				\
	LOAD_LR;				\
	blr

#define LV1_6_IN_2_OUT(API_NAME, API_NUMBER)	\
	NO_SUPPORT _##API_NAME

#define LV1_6_IN_3_OUT(API_NAME, API_NUMBER)	\
	NO_SUPPORT _##API_NAME

#define LV1_7_IN_1_OUT(API_NAME, API_NUMBER)	\
	NO_SUPPORT _##API_NAME

#define LV1_7_IN_6_OUT(API_NAME, API_NUMBER)	\
	NO_SUPPORT _##API_NAME

#define LV1_8_IN_1_OUT(API_NAME, API_NUMBER)	\
	NO_SUPPORT _##API_NAME

/* the lv1 underscored call definitions expand here */

#define LV1_CALL(name, in, out, num) LV1_##in##_IN_##out##_OUT(lv1_##name, num)
#include "lv1call.h"

