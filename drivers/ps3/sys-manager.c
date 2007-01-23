/*
 *  PS3 Platform System Manager.
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

#define DEBUG 1

#include <linux/kernel.h>
#include <linux/module.h>
#include <asm/ps3.h>

#include "vuart.h"

MODULE_AUTHOR("Sony Corporation");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("PS3 Platform system manager");

/**
 * System manager
 *
 */

static int
ps3_sys_manager_probe (struct ps3_vuart_port_device *dev)
{
	pr_debug(" -> %s:%d\n", __func__, __LINE__);
	pr_debug(" <- %s:%d\n", __func__, __LINE__);
	return 0;
}

static int
ps3_sys_manager_remove (struct ps3_vuart_port_device *dev)
{
	pr_debug(" -> %s:%d\n", __func__, __LINE__);
	pr_debug(" <- %s:%d\n", __func__, __LINE__);
	return 0;
}

static struct ps3_vuart_port_driver ps3_sys_manager = {
	.match_id = PS3_MATCH_ID_SYSTEM_MANAGER,
	.core = {
		.name = "ps3_sys_manager",
	},
	.probe = ps3_sys_manager_probe,
	.remove = ps3_sys_manager_remove,
};

static int __init
ps3_sys_manager_init (void)
{
	int result;

	pr_debug(" -> %s:%d\n", __func__, __LINE__);

	result = ps3_vuart_port_driver_register(&ps3_sys_manager);

	pr_debug(" <- %s:%d\n", __func__, __LINE__);
	return result;
}

static void __exit
ps3_sys_manager_exit (void)
{
	pr_debug(" -> %s:%d\n", __func__, __LINE__);
	ps3_vuart_port_driver_unregister(&ps3_sys_manager);
	pr_debug(" <- %s:%d\n", __func__, __LINE__);
}

module_init (ps3_sys_manager_init);
module_exit (ps3_sys_manager_exit);

