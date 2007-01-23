/*
 *  PS3 bootrom support.
 *
 *  Copyright (C) 2006 Sony Computer Entertainment Inc.
 *  Copyright 2006 Sony Corp.
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
 *  For ps3:
 *   1. The MMU is off
 *   2. The image is loaded at lpar address zero
 *   3. The image is entered at 0x100 (system reset)
 */
	.text

	/* Secondary processors spin on this value until it goes to 1. */
	.globl  __secondary_hold_spinloop
__secondary_hold_spinloop:
	.llong	0x0

	. = 0x60
/*
 * The following code is used on pSeries to hold secondary processors
 * in a spin loop after they have been freed from OpenFirmware, but
 * before the bulk of the kernel has been relocated.  This code
 * is relocated to physical address 0x60 before prom_init is run.
 * All of it must fit below the first exception vector at 0x100.
 */

	.globl __secondary_hold
__secondary_hold:
	mfmsr	r24
	ori	r24,r24,2
	mtmsrd	r24			/* RI on */

	/* Grab our physical cpu number */
	mr	r24,r3

	/* All secondary cpus wait here until told to start. */
100:	ld	r4,__secondary_hold_spinloop@l(0)
	cmpdi	0,r4,1
	bne	100b

/*	ld	r4,_vmlinux_start */
	mtctr	r4
	mr	r3,r24
	bctr

	. = 0x100
	.globl _zimage_start
_zimage_start:

	/*
	 * We start up with both threads running.  Start up with thread 0
	 * and send the other off to __secondary_hold with the Linux processor
	 * number in r3.
	 */
	mfspr	r3, 0x88
	cntlzw.	r3, r3
	beq	1f
	b	__secondary_hold
1:
	/* current thread is thread 0 */
	b start
