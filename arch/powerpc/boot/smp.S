/*
 *  zImage SMP support.
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

	.data
	.balign 4
smp_secondary_sync:
	.long 0

	.text

/**
 * smp_secondary_hold - Hold the secondary cpu until the kernel is ready to enter.
 *
 * The secondary cpu is sent here.  It sets smp_secondary_sync to
 * signal to the primary that it has entered smp_secondary_hold.  The
 * secondary then loops here until smp_secondary_sync is cleared by
 * the primary.  The primary does this with smp_secondary_release when
 * the kernel has been setup and is ready to enter.  The secondary then
 * branches to the well known address of __secondary_hold (0x60) in the
 * relocated kernel.
 */

	.balign 4
	.globl smp_secondary_hold
smp_secondary_hold:

	/* Set smp_secondary_sync. */

	lis	r5, smp_secondary_sync@ha
	addi	r5, r5, smp_secondary_sync@l
	li 	r4, 1
	stw	r4, 0(r5)
	sync

	/* Loop while smp_secondary_sync. */
1:
	lwz	r4, 0(r5)
	cmpwi	r4, 0
	bne	1b

	/* Branch to __secondary_hold in kernel. */

	li	r3, 1 /* for debug!!!!!!!!!!!!!!! */
	li	r4, 0x60
	mtctr	r4
	bctr

/**
 * smp_secondary_wait - Wait for the secondary cpu to enter smp_secondary_hold.
 *
 * The primary cpu waits here until the secondary enters smp_secondary_hold
 * and sets smp_secondary_sync to signal it is safe to reclaim low memory.
 */

	.balign 4
	.globl smp_secondary_wait
smp_secondary_wait:

	/* Loop while NOT smp_secondary_sync. */

	lis	r9, smp_secondary_sync@ha
	addi	r9, r9, smp_secondary_sync@l
1:
	lwz	r8, 0(r9)
	cmpwi	r8, 0
	beq	1b
	blr

/**
 * smp_secondary_release - Release the secondary cpu.
 *
 * Called by the primary cpu after the kernel is setup and ready for
 * the secondary to enter.  Signals the secondary to enter the kerenl
 * by clearing smp_secondary_sync.  It is then safe for the primary
 * to enter the kerenl at __start.
 */

	.balign 4
	.globl smp_secondary_release
smp_secondary_release:

#define USE_SECONDARY_HOLD_ACKNOWLEDGE 0

#if (USE_SECONDARY_HOLD_ACKNOWLEDGE)
	/* Clear __secondary_hold_acknowledge. */

	li	r5, 0x10 /* __secondary_hold_acknowledge */
	li	r4, 0
	stw	r4, 0(r5)
	sync
#endif

	/* Clear smp_secondary_sync */

	lis	r5, smp_secondary_sync@ha
	addi	r5, r5, smp_secondary_sync@l
	li 	r4, 0
	stw	r4, 0(r5)
	sync

#if (USE_SECONDARY_HOLD_ACKNOWLEDGE)
	/* Loop while NOT __secondary_hold_acknowledge. */

	li	r5, 0x10 /* __secondary_hold_acknowledge */
1:
	lwz	r4, 0(r5)
	cmpwi	r4, 0
	beq	1b
#endif
	blr
