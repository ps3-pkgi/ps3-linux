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

extern char _end[];

#define PLATFORM_STACK(size) \
	char __attribute__((section(".stack"))) _platform_stack[size]; \
	void *_platform_stack_top = _platform_stack + sizeof(_platform_stack);

PLATFORM_STACK(4096)

static void ps3_console_write(const char *buf, int len)
{
}

static void *ps3_malloc(u32 size)
{
	static unsigned long addr = (unsigned long)_end;
	const unsigned long ret = _ALIGN_UP(addr, 0x100000);

	addr = ret + size;

	printf("%s:%d: %xh bytes @ %lxh\n", __func__, __LINE__, size, ret);

	return (void *)ret;
}

static void ps3_exit(void)
{
	printf("ps3_exit\n");
	while(1);
}

int platform_init(void *promptr, char *dt_blob_start, char *dt_blob_end)
{
	console_ops.write = ps3_console_write;

	printf("\n-- PS3 bootwrapper --\n");
	printf("%s %s\n", __TIME__, __DATE__);

	platform_ops.malloc = ps3_malloc;
	platform_ops.exit = ps3_exit;

	//printf("%s:%d:\n", __func__, __LINE__);

	return 0;
}

int platform_init_new(unsigned int cpu_id)
{
	console_ops.write = ps3_console_write;

	printf("\n-- PS3 bootwrapper, cpu (%u) --\n", cpu_id);
	printf("%s %s\n", __TIME__, __DATE__);

	platform_ops.malloc = ps3_malloc;
	platform_ops.exit = ps3_exit;

	//printf("%s:%d:\n", __func__, __LINE__);

	return 0;
}

void ps3_no_support(void)
{
	printf("\n*** bootwrapper BUG: ps3_no_support() called!\n");
	while(1)
		(void)0;
}
