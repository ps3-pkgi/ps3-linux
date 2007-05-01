/*
 * PS3 ROM Storage Driver
 *
 * Copyright (C) 2007 Sony Computer Entertainment Inc.
 * Copyright 2007 Sony Corp.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published
 * by the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#define DEBUG

#include "storage.h"


#define DEVICE_NAME		"ps3rom"


static int ps3rom_probe(struct ps3_system_bus_device *dev)
{
	dev_dbg(&dev->core, "%s:%u\n", __func__, __LINE__);
	return 0;
}

static int ps3rom_remove(struct ps3_system_bus_device *dev)
{
	dev_dbg(&dev->core, "%s:%u\n", __func__, __LINE__);
	return 0;
}


static struct ps3_system_bus_driver ps3rom = {
	.match_id	= PS3_MATCH_ID_STOR_ROM,
	.core.name	= DEVICE_NAME,
	.probe		= ps3rom_probe,
	.remove		= ps3rom_remove
};


static int __init ps3rom_init(void)
{
	pr_debug("%s:%u\n", __func__, __LINE__);
	return ps3_system_bus_driver_register(&ps3rom, PS3_IOBUS_SB);
}

static void __exit ps3rom_exit(void)
{
	pr_debug("%s:%u\n", __func__, __LINE__);
	return ps3_system_bus_driver_unregister(&ps3rom);
}

module_init(ps3rom_init);
module_exit(ps3rom_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("PS3 ROM Storage Driver");
MODULE_AUTHOR("Sony Corporation");
