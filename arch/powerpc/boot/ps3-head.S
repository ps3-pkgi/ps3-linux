/*
 *  PS3 first stage entry.
 *
 *  Copyright (C) 2007 Sony Computer Entertainment Inc.
 *  Copyright 2007 Sony Corp.
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
 * __start_stage1 - The PS3 first stage entry.
 *
 * On entry the MMU is off and the bootwrapper image has been loaded to lpar
 * address zero.  The bootwrapper is entered at 0x100 (System Reset) by both
 * proceesor threads.
 *
 * __start_stage1 runs the bootwrapper code on thread 0 to prepare the
 * wrapped kernel image.  Thread 0 then enters the kernel at the end of the
 * bootwrapper code.  __start_stage1 sends thread 1 to smp_secondary_hold()
 * to wait until the bootwrapper code is finished running.  When signled,
 * thread 1 enters the kernel directly from smp_secondary_hold().
 */

	.section .vectors,"ax"

	. = 0x100

	.globl __start_stage1
__start_stage1:

	/* switch to 32-bit mode */

	mfmsr	r9
	clrldi	r9,r9,1
	mtmsrd	r9

	/* setup stack */

	lis	r1,__wrapper_stack@ha
	addi	r1,r1,__wrapper_stack@l

	bl	__construct_console

	/* test thread number and branch */

	mfspr	r3, 0x88
	cntlzw.	r3, r3
	beq	1f
	bl	__marker_1
	b	smp_secondary_hold
1:
	bl	__marker_0
	b	_zimage_start


	.section .text

#define _GLOBAL(n) \
 .section ".text"; \
 .align 2 ;        \
 .globl n;         \
n:


_GLOBAL(_lv1_construct_logical_console)
	mflr 0;
	std 0, 16(1);
	li 11, 105;
	.long 0x44000022;
	extsw 3, 3;
	ld 0, 16(1);
	mtlr 0;
	blr

_GLOBAL(_lv1_send_logical_console)
	mflr 0;
	std 0, 16(1);
	stdu 9, -8(1);
	li 11, 107;
	.long 0x44000022;
	extsw 3, 3;
	addi 1, 1, 8;
	ld 11, -8(1);
	std 4, 0(11);
	ld 0, 16(1);
	mtlr 0;
	blr

_GLOBAL(_lv1_flush_logical_console)
	mflr 0;
	std 0, 16(1);
	li 11, 109;
	.long 0x44000022;
	extsw 3, 3;
	ld 0, 16(1);
	mtlr 0;
	blr


#define CONSOLE_CID 3

	.globl __construct_console
__construct_console:

	mflr r13
	li r3,CONSOLE_CID
	li r4,0
	li r5,0
	li r6,4080
	li r7,4080
	li r8,0
	li r9,0
	li r11, 105 /* lv1_construct_logical_console */
	.long 0x44000022
	nop
	mtlr r13
	blr

	.globl __marker_0
__marker_0:

	mflr r13
	li r3,CONSOLE_CID
	li r4,8
	lis r5,0x7468
	ori r5,r5,0x7265
	rldicr r5,r5,32,31
	oris r5,r5,0x6164
	ori r5,r5,0x300a
	li r6,0
	li r7,0
	li r8,0
	li r11, 107 /* lv1_send_logical_console */
	.long 0x44000022;
	nop
	li r3,CONSOLE_CID
	li r11, 109 /* lv1_flush_logical_console */
	.long 0x44000022
	nop
	mtlr r13
	blr

	.globl __marker_1
__marker_1:

	mflr r13
	li r3,CONSOLE_CID
	li r4,8
	lis r5,0x7468
	ori r5,r5,0x7265
	rldicr r5,r5,32,31
	oris r5,r5,0x6164
	ori r5,r5,0x310a
	li r6,0
	li r7,0
	li r8,0
	li r11, 107 /* lv1_send_logical_console */
	.long 0x44000022;
	nop
	li r3,CONSOLE_CID
	li r11, 109 /* lv1_flush_logical_console */
	.long 0x44000022
	nop
	mtlr r13
	blr

	.section .bss
	. = . + 4096
	.globl __wrapper_stack
__wrapper_stack:
