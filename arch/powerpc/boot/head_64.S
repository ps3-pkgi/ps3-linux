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
 */

	.section .vectors,"ax"
	. = 0x100
	.globl __system_reset_entry
__system_reset_entry:

	/* Switch to 32-bit mode. */
	mfmsr	r9
	clrldi	r9,r9,1
	mtmsrd	r9
	nop

	li	r3, 0
	li	r4, 0
	li	r5, 0
	b	_zimage_start
