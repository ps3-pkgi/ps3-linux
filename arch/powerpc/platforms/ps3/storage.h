/*
 * PS3 Storage Bus
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

#include "platform.h"


struct ps3_storage_region {
	unsigned int id;
	u64 start;
	u64 size;
};

struct ps3_storage_device {
	struct ps3_system_bus_device sbd;

	enum ps3_dev_type dev_type;
	struct ps3_dma_region dma;
	unsigned int irq;
	u64 port;
	u64 blk_size;
	u64 num_blocks;

	u64 tag;
	int lv1_res;
	u64 lv1_tag;
	u64 lv1_status;
	struct completion done;

	unsigned long bounce_size;
	void *bounce_buf;
	u64 bounce_lpar;
	dma_addr_t bounce_dma;
	u64 dma_region;
	struct mutex mutex;

	// FIXME ps3 disk only?
	spinlock_t lock;
	struct request_queue *queue;
	struct gendisk *gendisk;
	struct task_struct *thread;

	// FIXME Should we keep the (single?) accessible region only?
	unsigned int num_regions;
	unsigned long accessible_regions;
	struct ps3_storage_region regions[0];	/* Must be last */
};

static inline struct ps3_storage_device *to_ps3_storage_device(struct device *dev)
{
	return container_of(dev, struct ps3_storage_device, sbd.core);
}

