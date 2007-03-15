/*
 *  PS3 bootwrapper support.
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

#include <stdarg.h>
#include <stddef.h>
#include "types.h"
#include "elf.h"
#include "string.h"
#include "stdio.h"
#include "page.h"
#include "ops.h"
#include "lv1call.h"

#if defined(DEBUG)
#define DBG(fmt...) printf(fmt)
#else
static inline int __attribute__ ((format (printf, 1, 2))) DBG(
	const char *fmt, ...) {return 0;}
#endif

extern char _start[];
extern char _end[];
extern char _dtb_start[];
extern char _dtb_end[];

PLATFORM_STACK(4096);

static void ps3_console_write(const char *buf, int len)
{
}

static void ps3_exit(void)
{
	printf("ps3_exit\n");
	lv1_panic(0); /* zero = no reboot */
	while(1);
}

int platform_init(void)
{
	const u32 heapsize = 0x4000000 - (u32)_end; /* 64M */

	console_ops.write = ps3_console_write;
	platform_ops.secondary_release = smp_secondary_release;
	platform_ops.exit = ps3_exit;

	printf("\n-- PS3 bootwrapper --\n");

	simple_alloc_init(_end, heapsize, 32, 64);
	platform_ops.vmlinux_alloc = platform_ops.malloc;

	ft_init(_dtb_start, 0, 4);
	return 0;
}

void ps3_no_support(void)
{
	printf("\n*** bootwrapper BUG: ps3_no_support() called!\n");
	ps3_exit();
}
