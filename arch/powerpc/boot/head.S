/*
 *  zImage exception vector entry.
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
 * __system_reset_entry - For platforms entering at the reset vector.
 *
 * __system_reset_entry runs the bootwrapper code on cpu 0 to prepare the
 * wrapped kernel image.  Cpu 0 then enters the kernel at the end of the
 * bootwrapper code.  .secondary_start sends secondary cpus to
 * smp_secondary_hold() to wait until the bootwrapper code is finished
 * running.  When signled, secondary cpus enter the kernel directly from
 * smp_secondary_hold().
 *
 * This implementation supports only one secondary cpu.
 *
 * For the PS3 first stage entry the MMU is off and the bootwrapper image
 * has been loaded to lpar address zero.  The bootwrapper is entered at
 * 0x100 (system reset) by both proceesor threads.
 */

	.section .vectors,"ax"
	. = 0x100
	.globl __system_reset_entry
__system_reset_entry:

	/* switch to 32-bit mode */

	mfmsr	r9
	clrldi	r9,r9,1
	mtmsrd	r9
	nop

	li	r4, 0
	li	r5, 0

	/* test thread number and branch */

	mfspr	r3, 0x88
	cntlzw.	r3, r3
	beq	__primary_start

__secondary_start:

	/* setup secondary's stack */

	lis	r1, __debug_stack_1@ha
	addi	r1, r1, __debug_stack_1@l
	subi	r1, r1, 16

	b	smp_secondary_hold

__primary_start:

	/* setup primary's stack */

	lis	r1, _platform_stack@ha
	addi	r1, r1, _platform_stack@l
	subi	r1, r1, 16

	li	r3, 0
	b	_zimage_start

	 .section .stack,"aw",@nobits
	. = . + 4096
__debug_stack_1:
