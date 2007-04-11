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
smp_secondary_entry:
	.long 0

	.text

/**
 * smp_secondary_hold - Hold any secondary cpus until kernel is ready to enter.
 * !!!Need to make this do a kexec entry!!!
 */

	.balign 4
	.globl smp_secondary_hold
smp_secondary_hold:

	lis	r9, smp_secondary_entry@ha
	addi	r9, r9, smp_secondary_entry@l
	lwz	r9, 0(r9)
	cmpwi	r9, 0
	beq	smp_secondary_hold
	li	r3, 0
	li	r4, 0
	li	r5, 0
	mtctr	r9
	bctr

/**
 * smp_secondary_release - Release any secondary cpus.
 * @kentry: The kernel entry for secondary cpus.
 *
 * Typically called by the primary cpu after the kernel is ready for entry.
 */

	.balign 4
	.globl smp_secondary_release
smp_secondary_release:
	lis	r9, smp_secondary_entry@ha
	addi	r9, r9, smp_secondary_entry@l
	stw	r3, 0(r9)
	sync
	blr
