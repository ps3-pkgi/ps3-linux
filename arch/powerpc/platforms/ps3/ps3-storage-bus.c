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

#define DEBUG

#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/freezer.h>
#include <linux/kthread.h>
#include <linux/module.h>

#include <asm/firmware.h>
#include <asm/lv1call.h>

#include "storage.h"


extern struct dma_mapping_ops ps3_sb_dma_ops;	// FIXME

#ifdef DEBUG
static const char *ps3_stor_dev_type(enum ps3_dev_type dev_type)
{
	switch (dev_type) {
	case PS3_DEV_TYPE_STOR_DISK:
		return "disk";

	case PS3_DEV_TYPE_STOR_ROM:
		return "rom";

	case PS3_DEV_TYPE_STOR_FLASH:
		return "flash";

	case PS3_DEV_TYPE_NONE:
		return "not present";

	default:
		return "unknown";
	}
}
#else
static inline const char *ps3_stor_dev_type(enum ps3_dev_type dev_type)
{
    return NULL;
}
#endif /* DEBUG */

#define NOTIFICATION_DEVID ((u64)(-1L))

static u64 ps3_stor_wait_for_completion(u64 devid, u64 tag,
					unsigned int timeout)
{
	unsigned int retries = 0;
	u64 res = -1, status;

	for (retries = 0; retries < timeout; retries++) {
		res = lv1_storage_check_async_status(NOTIFICATION_DEVID, tag,
						     &status);
		if (!res)
			break;
		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(1);
	}
	if (res)
		pr_debug("%s:%u: check_async_status returns %ld status %lx\n",
			 __func__, __LINE__, res, status);

	return res;
}

static int ps3_stor_probe_notification(struct ps3_storage_device *dev)
{
	int error = -ENODEV, res;
	u64 *buf;
	u64 lpar;

	pr_info("%s:%u: Requesting notification\n", __func__, __LINE__);

	buf = kzalloc(512, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	lpar = ps3_mm_phys_to_lpar(__pa(buf));

	/* 2-1) open special event device */
	res = lv1_open_device(dev->sbd.did.bus_id, NOTIFICATION_DEVID, 0);
	if (res) {
		printk(KERN_ERR "%s:%u: open notification device failed %d\n",
		       __func__, __LINE__, res);
		goto fail_free;
	}

	/* 2-2) write info to request notify */
	buf[0] = 0;
	buf[1] = (1 << 1); /* region update info only */
	res = lv1_storage_write(NOTIFICATION_DEVID, 0, 0, 1, 0, lpar,
				&dev->tag);
	if (res) {
		printk(KERN_ERR "%s:%u: notify request write failed %d\n",
		       __func__, __LINE__, res);
		goto fail_close;
	}

	/* wait for completion in one second */
	res = ps3_stor_wait_for_completion(NOTIFICATION_DEVID, dev->tag, HZ);
	if (res) {
		/* write not completed */
		printk(KERN_ERR "%s:%u: write not completed %d\n", __func__,
		       __LINE__, res);
		goto fail_close;
	}

	/* 2-3) read to wait region notification for each device */
	while (1) {
		memset(buf, 0, 512);
		lv1_storage_read(NOTIFICATION_DEVID, 0, 0, 1, 0, lpar,
				 &dev->tag);
		res = ps3_stor_wait_for_completion(NOTIFICATION_DEVID,
						   dev->tag, HZ);
		if (res) {
			/* read not completed */
			printk(KERN_ERR "%s:%u: read not completed %d\n",
			       __func__, __LINE__, res);
			break;
		}

		/* 2-4) verify the notification */
		if (buf[0] != 1 || buf[1] != dev->sbd.did.bus_id) {
			/* other info notified */
			pr_debug("%s:%u: notification info %ld dev=%lx type=%lx\n",
				 __func__, __LINE__, buf[0], buf[2], buf[3]);
			break;
		}

		if (buf[2] == dev->sbd.did.dev_id && buf[3] == dev->dev_type) {
			pr_debug("%s:%u: device ready\n", __func__, __LINE__);
			error = 0;
			break;
		}
	}

fail_close:
	lv1_close_device(dev->sbd.did.bus_id, NOTIFICATION_DEVID);

fail_free:
	kfree(buf);
	return error;
}

static irqreturn_t ps3_stor_probe_interrupt(int irq, void *data)
{
	struct ps3_storage_device *dev = data;

	dev->lv1_res = lv1_storage_get_async_status(dev->sbd.did.dev_id,
						    &dev->lv1_tag,
						    &dev->lv1_status);
	complete(&dev->done);

	return IRQ_HANDLED;
}

static int ps3_stor_probe_access(struct ps3_storage_device *dev)
{
	int res, error;
	unsigned int irq, i;
	void *buf;
	dma_addr_t dma;
	u64 lpar;

	if (dev->sbd.match_id == PS3_MATCH_ID_STOR_ROM) {
		/*
		 * special case
		 * cd-rom is assumed always accessible
		 */
		dev->accessible_regions = 1;
		return 0;
	}

	/*
	 * 1. open the device
	 * 2. register irq for the device
	 * 3. connect irq
	 * 4. map dma region
	 * 5. do read
	 * 6. umap dma region
	 * 7. disconnect irq
	 * 8. unregister irq
	 * 9. close the device
	 */

	res = lv1_open_device(dev->sbd.did.bus_id, dev->sbd.did.dev_id, 0);
	if (res) {
		printk(KERN_ERR "%s:%u: open device %u:%u failed %d\n",
		       __func__, __LINE__, dev->sbd.did.bus_id,
		       dev->sbd.did.dev_id, res);
		return -ENODEV;
	}

	error = ps3_sb_event_receive_port_setup(PS3_BINDING_CPU_ANY,
						&dev->sbd.did,
						dev->sbd.interrupt_id, &irq);
	if (error) {
		printk(KERN_ERR
		       "%s:%u: ps3_sb_event_receive_port_setup failed %d\n",
		       __func__, __LINE__, error);
		goto fail_close_device;
	}

	error = request_irq(irq, ps3_stor_probe_interrupt, IRQF_DISABLED,
			    "ps3_stor_probe", dev);
	if (error) {
		printk(KERN_ERR "%s:%u: request_irq failed %d\n", __func__,
		       __LINE__, error);
		goto fail_event_receive_port_destroy;
	}

	/* PAGE_SIZE >= 4 KiB buffer for fail safe of large sector devices */
	buf = (void *)__get_free_page(GFP_KERNEL);
	if (!buf) {
		printk(KERN_ERR "%s:%u: no memory while probing", __func__,
		       dev->sbd.did.dev_id);
		error = -ENOMEM;
		goto fail_free_irq;
	};

	ps3_dma_region_init(&dev->dma, &dev->sbd.did, PS3_DMA_4K,
			    PS3_DMA_OTHER, buf, PAGE_SIZE, PS3_IOBUS_SB);
	res = ps3_dma_region_create(&dev->dma);
	if (res) {
		printk(KERN_ERR "%s:%u: cannot create DMA region\n", __func__,
		       __LINE__);
		error = -ENOMEM;
		goto fail_free_buf;
	}

	lpar = ps3_mm_phys_to_lpar(__pa(buf));

	dma = dma_map_single(&dev->sbd.core, buf, PAGE_SIZE, DMA_FROM_DEVICE);
	if (!dma) {
		printk(KERN_ERR "%s:%u: map DMA region failed\n", __func__,
		       __LINE__);
		error = -ENODEV;
		goto fail_free_dma;
	}

	error = -EPERM;
	for (i = 0; i < dev->num_regions; i++) {
		pr_debug("%s:%u: checking accessibility of region %u\n",
			 __func__, __LINE__, i);

		init_completion(&dev->done);
		res = lv1_storage_read(dev->sbd.did.dev_id, dev->regions[i].id,
				       0, /* start sector */
				       1, /* sector count */
				       0, /* flags */
				       lpar, &dev->tag);
		if (res) {
			pr_debug("%s:%u: read failed %d, region %u is not accessible\n",
				 __func__, __LINE__, res, i);
			continue;
		}

		wait_for_completion(&dev->done);

		if (dev->lv1_res || dev->lv1_status) {
			pr_debug("%s:%u: read failed, region %u is not accessible\n",
				 __func__, __LINE__, i);
			continue;
		}

		if (dev->lv1_tag != dev->tag) {
			printk(KERN_ERR
			       "%s:%u: tag mismatch, got %lx, expected %lx\n",
			       __func__, __LINE__, dev->lv1_tag, dev->tag);
			break;
		}

		pr_debug("%s:%u: region %u is accessible\n", __func__,
			 __LINE__, i);
		set_bit(i, &dev->accessible_regions);

		/* We can access at least one region */
		error = 0;
	}
	if (hweight_long(dev->accessible_regions) > 1)
		pr_info("%s:%u: %lu accessible regions found. Only the first "
			"one will be used", __func__, __LINE__,
			hweight_long(dev->accessible_regions));

	dma_unmap_single(&dev->sbd.core, dma, PAGE_SIZE, DMA_FROM_DEVICE);
fail_free_dma:
	ps3_dma_region_free(&dev->dma);
fail_free_buf:
	free_page((unsigned long)buf);
fail_free_irq:
	free_irq(irq, dev);
fail_event_receive_port_destroy:
	ps3_sb_event_receive_port_destroy(&dev->sbd.did, dev->sbd.interrupt_id,
					  irq);
fail_close_device:
	lv1_close_device(dev->sbd.did.bus_id, dev->sbd.did.dev_id);

	return error;
}

static int ps3_stor_probe_dev(struct ps3_repository_device *repo)
{
	int error;
	u64 port, blk_size, num_blocks;
	unsigned int num_regions, i;
	struct ps3_storage_device *dev;
	enum ps3_dev_type dev_type;
	enum ps3_match_id match_id;

	pr_info("%s:%u: Probing new storage device %u\n", __func__, __LINE__,
		 repo->dev_index);

	error = ps3_repository_read_dev_id(repo->bus_index, repo->dev_index,
					   &repo->did.dev_id);
	if (error) {
		pr_debug("%s:%u: read_dev_id failed %d\n", __func__, __LINE__,
			 error);
		return -ENODEV;
	}

	error = ps3_repository_read_dev_type(repo->bus_index, repo->dev_index,
					     &dev_type);
	if (error) {
		pr_debug("%s:%u: read_dev_type failed %d\n", __func__,
			__LINE__, error);
		return -ENODEV;
	}

	pr_debug("%s:%u: index %u:%u: id %u:%u dev_type %u (%s)\n", __func__,
		 __LINE__, repo->bus_index, repo->dev_index,
		 repo->did.bus_id, repo->did.dev_id, dev_type,
		 ps3_stor_dev_type(dev_type));

	switch (dev_type) {
	case PS3_DEV_TYPE_STOR_DISK:
		match_id = PS3_MATCH_ID_STOR_DISK;
		break;

	case PS3_DEV_TYPE_STOR_ROM:
		match_id = PS3_MATCH_ID_STOR_ROM;
		break;

	case PS3_DEV_TYPE_STOR_FLASH:
		match_id = PS3_MATCH_ID_STOR_FLASH;
		break;

	case PS3_DEV_TYPE_NONE:
		return 0;

	default:
		return 0;
	}

	error = ps3_repository_read_stor_dev_info(repo->bus_index,
						  repo->dev_index, &port,
						  &blk_size, &num_blocks,
						  &num_regions);
	if (error) {
		pr_debug("%s:%u: _read_stor_dev_info failed %d\n", __func__,
			 __LINE__, error);
		return -ENODEV;
	}
	pr_debug("%s:%u: index %u:%u: port %lu blk_size %lu num_blocks %lu "
		 "num_regions %u\n",
		 __func__, __LINE__, repo->bus_index, repo->dev_index, port,
		 blk_size, num_blocks, num_regions);

	dev = kzalloc(sizeof(struct ps3_storage_device)+
		      num_regions*sizeof(struct ps3_storage_region),
		      GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->sbd.match_id = match_id;
	dev->sbd.did = repo->did;
	dev->sbd.d_region = &dev->dma;
	dev->dev_type = dev_type;
	dev->port = port;
	dev->blk_size = blk_size;
	dev->num_blocks = num_blocks;
	dev->num_regions = num_regions;

	error = ps3_repository_find_interrupt(repo,
					      PS3_INTERRUPT_TYPE_EVENT_PORT,
					      &dev->sbd.interrupt_id);
	if (error) {
		pr_debug("%s:%u: find_interrupt failed %d\n", __func__,
			__LINE__, error);
		goto cleanup;
	}

#ifdef CONFIG_PS3_STORAGE
	switch (dev->dev_type) {
	case PS3_DEV_TYPE_STOR_FLASH:
	case PS3_DEV_TYPE_STOR_DISK:
		break;

	default:
		/*
		 * FIXME As this driver conflicts with the old storage driver,
		 *	 we cannot do a full probe here
		 */
		printk(KERN_ERR
		       "Ignoring storage device, let the old driver handle it\n");
		goto cleanup;
	}
#endif

	/* FIXME Do we really need this? I guess for kboot only? */
	error = ps3_stor_probe_notification(dev);
	if (error) {
		pr_debug("%s:%u: probe_notification failed %d\n", __func__,
			 __LINE__, error);
		goto cleanup;
	}

	for (i = 0; i < num_regions; i++) {
		unsigned int id;
		u64 start, size;

		error = ps3_repository_read_stor_dev_region(repo->bus_index,
							    repo->dev_index, i,
							    &id, &start,
							    &size);
		if (error) {
			pr_debug("%s:%u: read_stor_dev_region failed %d\n",
				 __func__, __LINE__, error);
			goto cleanup;
		}
		pr_debug("%s:%u: region %u: id %u start %lu size %lu\n",
			 __func__, __LINE__, i, id, start, size);

		dev->regions[i].id = id;
		dev->regions[i].start = start;
		dev->regions[i].size = size;
	}

	// FIXME ps3_system_bus_device_register() does this, too, but too late
	// FIXME Perhaps we should do the probing in each storage driver
	//       itself?
	dev->sbd.core.archdata.dma_ops = &ps3_sb_dma_ops;
	error = ps3_stor_probe_access(dev);
	if (error) {
		pr_debug("%s:%u: probe_access failed %d\n", __func__, __LINE__,
			 error);
		goto cleanup;
	}

	// FIXME Prevent the system bus from messing with our DMA
	dev->sbd.d_region = NULL;

	error = ps3_system_bus_device_register(&dev->sbd, PS3_IOBUS_SB);
	if (error) {
		pr_debug("%s:%u: ps3_system_register failed %d\n", __func__,
			 __LINE__, error);
		goto cleanup;
	}
	return 0;

cleanup:
	kfree(dev);
	return -ENODEV;
}

static int ps3_storage_thread(void *data)
{
	struct ps3_repository_device *repo = data;
	int error;
	unsigned int n, ms = 250;

	pr_debug("%s:%u: kthread started\n", __func__, __LINE__);

	do {
		try_to_freeze();

//		pr_debug("%s:%u: Checking for new storage devices...\n",
//			 __func__, __LINE__);
		error = ps3_repository_read_bus_num_dev(repo->bus_index, &n);
		if (error) {
			printk(KERN_ERR "%s:%u: read_bus_num_dev failed %d\n",
			       __func__, __LINE__, error);
			break;
		}

		if (n > repo->dev_index) {
			pr_debug("%s:%u: Found %u storage devices (%u new)\n",
				 __func__, __LINE__, n, n - repo->dev_index);

			while (repo->dev_index < n && !error) {
				error = ps3_stor_probe_dev(repo);
				repo->dev_index++;
			}

			if (error)
				pr_debug("%s:%u: ps3_stor_probe_dev failed %d\n",
					 __func__, __LINE__, error);
			ms = 250;
		}

		msleep_interruptible(ms);
		if (ms < 60000)
			ms <<= 1;
	} while (!kthread_should_stop());

	pr_debug("%s:%u: kthread finished\n", __func__, __LINE__);

	return 0;
}


static int __init ps3_storage_init(void)
{
	int error;
	static struct ps3_repository_device repo;
	struct task_struct *task;

	pr_debug("%s:%u\n", __func__, __LINE__);

	if (!firmware_has_feature(FW_FEATURE_PS3_LV1))
		return -ENODEV;

	error = ps3_repository_find_bus(PS3_BUS_TYPE_STORAGE, 0,
					&repo.bus_index);
	if (error) {
		printk(KERN_ERR "%s: Cannot find storage bus (%d)\n", __func__,
		       error);
		return -ENODEV;
	}
	pr_debug("%s:%u: Storage bus has index %u\n", __func__, __LINE__,
		 repo.bus_index);

	error = ps3_repository_read_bus_id(repo.bus_index, &repo.did.bus_id);
	if (error) {
		printk(KERN_ERR "%s: read_bus_id failed %d\n", __func__,
		       error);
		return -ENODEV;
	}

	pr_debug("%s:%u: Storage bus has id %u\n", __func__, __LINE__,
		 repo.did.bus_id);

	task = kthread_run(ps3_storage_thread, &repo, "ps3stor-probe");
	BUG_ON(IS_ERR(task));

	return 0;
}

postcore_initcall(ps3_storage_init);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("PS3 Storage Bus Driver");
MODULE_AUTHOR("Sony Corporation");
