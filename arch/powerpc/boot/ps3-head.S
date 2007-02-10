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

	mfspr	r3, 0x88
	cntlzw.	r3, r3
	beq	1f
	b	smp_secondary_hold
1:
	b	_zimage_start
