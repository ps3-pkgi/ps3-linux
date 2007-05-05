
#define PS3DISK_DIRECT_IO		1
#define PS3DISK_BOUNCE			2
#define PS3DISK_SG			3
#define PS3DISK_TCQ			4

#define PS3DISK_DEFAULT_STRATEGY	PS3DISK_SG

/*
 * PS3 Disk Storage Driver
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

#include <linux/dma-mapping.h>
#include <linux/blkdev.h>
#include <linux/freezer.h>
#include <linux/hdreg.h>
#include <linux/kthread.h>

#include <asm/lv1call.h>

#include "../../arch/powerpc/platforms/ps3/storage.h"	// FIXME


#define DEVICE_NAME		"ps3disk"

#define BOUNCE_SIZE		(64*1024)

// FIXME Use a fixed major assigned by LANANA?
#define PS3DISK_MAJOR		0

#define PS3DISK_MAX_DISKS	16
#define PS3DISK_MINORS		16

#define KERNEL_SECTOR_SIZE	512

// FIXME SCSI uses H=64, S=32. Can we still repartition disks partitioned using
//       the old driver?
#define PS3DISK_HEADS		255
#define PS3DISK_SECTORS	63

#define PS3DISK_NAME		"ps3d%c"


static int ps3disk_major = PS3DISK_MAJOR;

static int read_only;
module_param(read_only, bool, 0);
static int strategy = PS3DISK_DEFAULT_STRATEGY;
module_param(strategy, int, 0);

static ssize_t ps3disk_read_write_sectors(struct ps3_storage_device *dev,
					  u64 lpar, u64 start_sector,
					  u64 sectors, int write)
{
	unsigned int idx = ffs(dev->accessible_regions)-1;
	unsigned int region_id = dev->regions[idx].id;
	int res = 0;
	const char *op = write ? "write" : "read";

	dev_dbg(&dev->sbd.core, "%s:%u: %s %lu sectors starting at %lu\n",
		__func__, __LINE__, op, sectors, start_sector);

	init_completion(&dev->done);
	if (write) {
		if (read_only) {
			dev_err(&dev->sbd.core, "NOT writing to disk!!\n");
			return sectors;
		}
		res = lv1_storage_write(dev->sbd.did.dev_id, region_id,
					start_sector, sectors,
					0, /* flags */
					lpar, &dev->tag);
	} else {
		res = lv1_storage_read(dev->sbd.did.dev_id, region_id,
				       start_sector, sectors,
				       0, /* flags */
				       lpar, &dev->tag);
	}
	if (res) {
		dev_err(&dev->sbd.core, "%s:%u: %s failed %d\n", __func__,
			__LINE__, op, res);
		return -EIO;
	}

	wait_for_completion(&dev->done);
	if (dev->lv1_status) {
		dev_err(&dev->sbd.core, "%s:%u: %s failed 0x%lx\n", __func__,
			__LINE__, op, dev->lv1_status);
		return -EIO;
	}

	dev_dbg(&dev->sbd.core, "%s:%u: %s completed\n", __func__, __LINE__,
		op);

	return sectors;
}

static int ps3disk_open(struct inode *inode, struct file *file)
{
	struct ps3_storage_device *dev = inode->i_bdev->bd_disk->private_data;

	dev_dbg(&dev->sbd.core, "%s:%u\n", __func__, __LINE__);

	file->private_data = dev;
	return 0;
}

static int ps3disk_release(struct inode *inode, struct file *file)
{
	struct ps3_storage_device *dev = inode->i_bdev->bd_disk->private_data;

	dev_dbg(&dev->sbd.core, "%s:%u\n", __func__, __LINE__);
	return 0;
}

static int ps3disk_getgeo(struct block_device *bdev, struct hd_geometry *geo)
{
	geo->heads = PS3DISK_HEADS;
	geo->sectors = PS3DISK_SECTORS;
	geo->cylinders = min(65535UL,
			     get_capacity(bdev->bd_disk) /
			     (PS3DISK_HEADS * PS3DISK_SECTORS));
	return 0;
}


static struct block_device_operations ps3disk_fops = {
	.owner		= THIS_MODULE,
	.open		= ps3disk_open,
	.release	= ps3disk_release,
	.getgeo		= ps3disk_getgeo,
};

static ssize_t ps3disk_read_write(struct ps3_storage_device *dev, void *buffer,
				  u64 start_sector, u64 sectors, int write)
{
	ssize_t res = 0;
	u64 bytes, lpar, hw_start_sector, hw_sectors;
#ifdef DEBUG
	const char *op = write ? "write" : "read";
#endif

	dev_dbg(&dev->sbd.core, "%s:%u: %s buffer 0x%p sector %lu nr %lu\n",
		__func__, __LINE__, op, buffer, start_sector, sectors);

	bytes = sectors * KERNEL_SECTOR_SIZE;
	// FIXME Precalculate dev->blk_size/KERNEL_SECTOR_SIZE somewhere
	hw_start_sector = start_sector * (dev->blk_size/KERNEL_SECTOR_SIZE);
	hw_sectors = sectors * (dev->blk_size/KERNEL_SECTOR_SIZE);

	BUG_ON(bytes > dev->bounce_size);

	switch (strategy) {
	case PS3DISK_DIRECT_IO:
		lpar = ps3_mm_phys_to_lpar(__pa(buffer));
		res = ps3disk_read_write_sectors(dev, lpar,
						 hw_start_sector, hw_sectors,
						 write);
		break;

	case PS3DISK_BOUNCE:
		if (write)
			memcpy(dev->bounce_buf, buffer, bytes);

		lpar = dev->bounce_lpar;
		res = ps3disk_read_write_sectors(dev, lpar, hw_start_sector,
						 hw_sectors, write);

		if (!write && res > 0)
			memcpy(buffer, dev->bounce_buf, bytes);
		break;

	case PS3DISK_SG:
		lpar = dev->bounce_lpar;
		res = ps3disk_read_write_sectors(dev, lpar, hw_start_sector,
						 hw_sectors, write);
		break;

	case PS3DISK_TCQ:
		// FIXME
		break;
	}

	return res < 0 ? res : sectors;
}

static irqreturn_t ps3disk_interrupt(int irq, void *data)
{
	struct ps3_storage_device *dev = data;

	dev->lv1_res = lv1_storage_get_async_status(dev->sbd.did.dev_id,
						    &dev->lv1_tag,
						    &dev->lv1_status);
	/*
	 * lv1_status = -1 may mean that ATAPI transport completed OK, but
	 * ATAPI command itself resulted CHECK CONDITION
	 * so, upper layer should issue REQUEST_SENSE to check the sense data
	 */
	if (dev->lv1_tag != dev->tag)
		dev_err(&dev->sbd.core,
			"%s:%u: tag mismatch, got %lx, expected %lx\n",
			__func__, __LINE__, dev->lv1_tag, dev->tag);
	if (dev->lv1_res)
		dev_err(&dev->sbd.core, "%s:%u: res=%d status=0x%lx\n",
			__func__, __LINE__, dev->lv1_res, dev->lv1_status);
	else
		complete(&dev->done);
	return IRQ_HANDLED;
}

static void ps3disk_handle_request_single(struct ps3_storage_device *dev,
					  struct request *req)
{
	ssize_t sectors_done;
	int uptodate;

	dev_dbg(&dev->sbd.core, "%s:%u\n", __func__, __LINE__);

	sectors_done = ps3disk_read_write(dev, req->buffer, req->sector,
					  req->current_nr_sectors,
					  rq_data_dir(req));
	if (sectors_done != req->current_nr_sectors) {
		dev_err(&dev->sbd.core, "%s:%u: request failed: %Zd\n",
			__func__, __LINE__, sectors_done);
		uptodate = 0;
	} else
		uptodate = 1;
	spin_lock_irq(&dev->lock);
	end_request(req, uptodate);
	spin_unlock_irq(&dev->lock);
}

static void ps3disk_scatter_gather(struct ps3_storage_device *dev,
				   struct request *req, int gather)
{
	unsigned int sectors = 0, offset = 0;
	struct bio *bio;
	sector_t sector;
	struct bio_vec *bvec;
	unsigned int i = 0, j;
	size_t size;
	void *buf;

	rq_for_each_bio(bio, req) {
		sector = bio->bi_sector;
		dev_dbg(&dev->sbd.core,
			"%s:%u: bio %u: %u segs %u sectors from %lu\n",
			__func__, __LINE__, i, bio_segments(bio),
			bio_sectors(bio), sector);
		bio_for_each_segment(bvec, bio, j) {
			BUG_ON(offset >= dev->bounce_size);
			size = bio_cur_sectors(bio)*KERNEL_SECTOR_SIZE;
			buf = __bio_kmap_atomic(bio, j, KM_USER0);
			if (gather)
				memcpy(dev->bounce_buf+offset, buf, size);
			else
				memcpy(buf, dev->bounce_buf+offset, size);
			offset += size;
			__bio_kunmap_atomic(bio, KM_USER0);
		}
		sectors += bio_sectors(bio);
		i++;
	}
	BUG_ON(sectors != req->nr_sectors);
}

static void ps3disk_handle_request_sg(struct ps3_storage_device *dev,
				      struct request *req)
{
	int uptodate = 1;
	int write = rq_data_dir(req);
	ssize_t res;

#ifdef DEBUG
	unsigned int n = 0;
	struct bio *bio;
	const char *op = write ? "write" : "read";
	rq_for_each_bio(bio, req)
		n++;
	dev_dbg(&dev->sbd.core,
		"%s:%u: %s req has %u bios for %lu sectors %lu hard sectors\n",
		__func__, __LINE__, op, n, req->nr_sectors,
		req->hard_nr_sectors);
#endif

	if (write)
		ps3disk_scatter_gather(dev, req, 1);

	res = ps3disk_read_write(dev, NULL, req->sector, req->nr_sectors,
				 write);
	if (res < 0)
		uptodate = 0;
	else if (!write)
		ps3disk_scatter_gather(dev, req, 0);

	spin_lock_irq(&dev->lock);
	if (!end_that_request_first(req, uptodate, req->nr_sectors)) {
		blkdev_dequeue_request(req);
		end_that_request_last(req, uptodate);
	}
	spin_unlock_irq(&dev->lock);
}

static int ps3disk_thread(void *data)
{
	struct ps3_storage_device *dev = data;
	request_queue_t *q = dev->queue;
	struct request *req;

	dev_dbg(&dev->sbd.core, "%s:%u\n", __func__, __LINE__);

	while (!kthread_should_stop()) {
		try_to_freeze();
		spin_lock_irq(&dev->lock);
		set_current_state(TASK_INTERRUPTIBLE);
		req = elv_next_request(q);
		if (!req) {
			spin_unlock_irq(&dev->lock);
			schedule();
			continue;
		}
		if (!blk_fs_request(req)) {
			blk_dump_rq_flags(req, DEVICE_NAME " bad request");
			end_request(req, 0);
			spin_unlock_irq(&dev->lock);
			continue;
		}
		spin_unlock_irq(&dev->lock);
		switch (strategy) {
		case PS3DISK_DIRECT_IO:
		case PS3DISK_BOUNCE:
			ps3disk_handle_request_single(dev, req);
			break;

		case PS3DISK_SG:
			ps3disk_handle_request_sg(dev, req);
			break;

		case PS3DISK_TCQ:
			// FIXME
			break;
		}
	}

	dev_dbg(&dev->sbd.core, "%s:%u\n", __func__, __LINE__);
	return 0;
}

static void ps3disk_request(request_queue_t *q)
{
	struct ps3_storage_device *dev = q->queuedata;
	wake_up_process(dev->thread);
}

static unsigned long ps3disk_mask;

static int ps3disk_probe(struct ps3_system_bus_device *_dev)
{
	struct ps3_storage_device *dev = to_ps3_storage_device(&_dev->core);
	int res, error;
	unsigned int idx, devidx;
	struct request_queue *queue;
	struct gendisk *gendisk;
	struct task_struct *task;

	dev_dbg(&dev->sbd.core, "%s:%u\n", __func__, __LINE__);

	dev_dbg(&dev->sbd.core, "=============================================\n");
	dev_dbg(&dev->sbd.core, "        Engaging new DISK driver\n");
	dev_dbg(&dev->sbd.core, "=============================================\n");

	BUILD_BUG_ON(PS3DISK_MAX_DISKS > BITS_PER_LONG);
	devidx = find_first_zero_bit(&ps3disk_mask, PS3DISK_MAX_DISKS);
	if (devidx >= PS3DISK_MAX_DISKS) {
		dev_err(&dev->sbd.core, "%s:%u: Too many disks\n", __func__,
			__LINE__);
		return -ENOSPC;
	}
	__set_bit(devidx, &ps3disk_mask);

	mutex_init(&dev->mutex);	// FIXME unused?
	spin_lock_init(&dev->lock);

	idx = ffs(dev->accessible_regions)-1;
	dev_dbg(&dev->sbd.core,
		"First accessible region has index %u start %lu size %lu\n",
		idx, dev->regions[idx].start, dev->regions[idx].size);

	if (dev->blk_size < KERNEL_SECTOR_SIZE) {
		dev_err(&dev->sbd.core,
			"%s:%u: cannot handle block size %lu\n", __func__,
			__LINE__, dev->blk_size);
		error = -EINVAL;
		goto fail;
	}

	dev->bounce_size = BOUNCE_SIZE;
	dev->bounce_buf = kmalloc(BOUNCE_SIZE, GFP_DMA);
	if (!dev->bounce_buf) {
		error = -ENOMEM;
		goto fail;
	}

	error = ps3_open_hv_device(&dev->sbd.did);

	if (error) {
		dev_dbg(&dev->sbd.core, "%s:%d: ps3_open_hv_device failed %d\n",
			__func__, __LINE__, error);
		goto fail_free;
	}

	error = ps3_sb_event_receive_port_setup(PS3_BINDING_CPU_ANY,
						&dev->sbd.did,
						dev->sbd.interrupt_id,
						&dev->irq);
	if (error) {
		dev_err(&dev->sbd.core,
			"%s:%u: ps3_sb_event_receive_port_setup failed %d\n",
		       __func__, __LINE__, error);
		goto fail_close_device;
	}

	error = request_irq(dev->irq, ps3disk_interrupt, IRQF_DISABLED,
			    DEVICE_NAME, dev);
	if (error) {
		dev_err(&dev->sbd.core, "%s:%u: request_irq failed %d\n",
			__func__, __LINE__, error);
		goto fail_sb_event_receive_port_destroy;
	}

	dev->bounce_lpar = ps3_mm_phys_to_lpar(__pa(dev->bounce_buf));

	dev->sbd.d_region = &dev->dma;
	switch (strategy) {
	case PS3DISK_DIRECT_IO:
	case PS3DISK_TCQ:
		// All RAM must be accessible
		ps3_dma_region_init(&dev->dma, &dev->sbd.did, PS3_DMA_4K,
				    PS3_DMA_OTHER, NULL, 0, PS3_IOBUS_SB);
		break;

	case PS3DISK_BOUNCE:
	case PS3DISK_SG:
		// Only the bounce buffer must be accessible
		ps3_dma_region_init(&dev->dma, &dev->sbd.did, PS3_DMA_4K,
				    PS3_DMA_OTHER, dev->bounce_buf,
				    dev->bounce_size, PS3_IOBUS_SB);
		break;
	}
	res = ps3_dma_region_create(&dev->dma);
	if (res) {
		printk(KERN_ERR "%s:%u: cannot create DMA region\n", __func__,
		       __LINE__);
		error = -ENOMEM;
		goto fail_free_irq;
	}

	dev->bounce_dma = dma_map_single(&dev->sbd.core, dev->bounce_buf,
					 dev->bounce_size, DMA_BIDIRECTIONAL);
	if (!dev->bounce_dma) {
		dev_err(&dev->sbd.core, "%s:%u: map DMA region failed\n",
			__func__, __LINE__);
		error = -ENODEV;
		goto fail_free_dma;
	}

	queue = blk_init_queue(ps3disk_request, &dev->lock);
	if (!queue) {
		dev_err(&dev->sbd.core, "%s:%u: blk_init_queue failed\n",
			__func__, __LINE__);
		error = -ENOMEM;
		goto fail_unmap_dma;
	}

	dev->queue = queue;
	queue->queuedata = dev;

	blk_queue_bounce_limit(queue, BLK_BOUNCE_HIGH);	// FIXME no highmem

	blk_queue_max_sectors(queue, dev->bounce_size/KERNEL_SECTOR_SIZE);
	blk_queue_segment_boundary(queue, -1UL);
	blk_queue_dma_alignment(queue, dev->blk_size-1);
	blk_queue_hardsect_size(queue, dev->blk_size);

	blk_queue_ordered(queue, 0, NULL);	// FIXME no barriers

	// FIXME Tagged Command Queueing?
	blk_queue_max_phys_segments(queue, -1);
	blk_queue_max_hw_segments(queue, -1);
	blk_queue_max_segment_size(queue, dev->bounce_size);

	gendisk = alloc_disk(PS3DISK_MINORS);
	if (!gendisk) {
		dev_err(&dev->sbd.core, "%s:%u: alloc_disk failed\n", __func__,
			__LINE__);
		error = -ENOMEM;
		goto fail_cleanup_queue;
	}

	dev->gendisk = gendisk;
	gendisk->major = ps3disk_major;
	gendisk->first_minor = devidx * PS3DISK_MINORS;
	gendisk->fops = &ps3disk_fops;
	gendisk->queue = queue;
	gendisk->private_data = dev;
	snprintf(gendisk->disk_name, sizeof(gendisk->disk_name), PS3DISK_NAME,
		 devidx+'a');
	set_capacity(gendisk,
		     dev->regions[idx].size*(dev->blk_size/KERNEL_SECTOR_SIZE));

	task = kthread_run(ps3disk_thread, dev, DEVICE_NAME);
	if (IS_ERR(task)) {
		error = PTR_ERR(task);
		goto fail_free_disk;
	}
	dev->thread = task;

	add_disk(gendisk);
	return 0;

fail_free_disk:
	put_disk(dev->gendisk);
fail_cleanup_queue:
	blk_cleanup_queue(queue);
fail_unmap_dma:
	dma_unmap_single(&dev->sbd.core, dev->bounce_dma, dev->bounce_size,
			 DMA_BIDIRECTIONAL);
fail_free_dma:
	ps3_dma_region_free(&dev->dma);
	// FIXME Prevent the system bus from messing with our DMA
	dev->sbd.d_region = NULL;
fail_free_irq:
	free_irq(dev->irq, dev);
fail_sb_event_receive_port_destroy:
	ps3_sb_event_receive_port_destroy(&dev->sbd.did, dev->sbd.interrupt_id,
					  dev->irq);
fail_close_device:
	ps3_close_hv_device(&dev->sbd.did);
fail_free:
	kfree(dev->bounce_buf);
	dev->bounce_buf = NULL;
fail:
	__clear_bit(devidx, &ps3disk_mask);
	return error;
}

static int ps3disk_remove(struct ps3_system_bus_device *_dev)
{
	struct ps3_storage_device *dev = to_ps3_storage_device(&_dev->core);
	int error;

	dev_dbg(&dev->sbd.core, "%s:%u\n", __func__, __LINE__);

	if (dev->thread)
		kthread_stop(dev->thread);
	if (dev->gendisk) {
		__clear_bit(dev->gendisk->first_minor / PS3DISK_MINORS,
			    &ps3disk_mask);
		del_gendisk(dev->gendisk);
		put_disk(dev->gendisk);
	}
	if (dev->queue)
		blk_cleanup_queue(dev->queue);
	dma_unmap_single(&dev->sbd.core, dev->bounce_dma, dev->bounce_size,
			 DMA_BIDIRECTIONAL);
	ps3_dma_region_free(&dev->dma);
	// FIXME Prevent the system bus from messing with our DMA
	dev->sbd.d_region = NULL;

	free_irq(dev->irq, dev);

	error = ps3_sb_event_receive_port_destroy(&dev->sbd.did,
						  dev->sbd.interrupt_id,
						  dev->irq);
	if (error)
		dev_err(&dev->sbd.core,
			"%s:%u: destroy event receive port failed %d\n",
			__func__, __LINE__, error);

	error = ps3_close_hv_device(&dev->sbd.did);
	if (error)
		dev_err(&dev->sbd.core,
			"%s:%u: ps3_close_hv_device failed %d\n",
			__func__, __LINE__, error);

	kfree(dev->bounce_buf);
	return 0;
}


static struct ps3_system_bus_driver ps3disk = {
	.match_id	= PS3_MATCH_ID_STOR_DISK,
	.core.name	= DEVICE_NAME,
	.probe		= ps3disk_probe,
	.remove		= ps3disk_remove,
	.shutdown	= ps3disk_remove,
};


static int __init ps3disk_init(void)
{
	int error;

	pr_debug("%s:%u\n", __func__, __LINE__);

	error = register_blkdev(ps3disk_major, DEVICE_NAME);
	if (error <= 0) {
		printk(KERN_ERR "%s:%u: register_blkdev failed %d\n", __func__,
		       __LINE__, error);
		return error;
	}
	if (!ps3disk_major)
		ps3disk_major = error;

	pr_info("%s:%u: registered block device major %d\n", __func__,
		__LINE__, ps3disk_major);

	if (read_only)
		pr_info("%s:%u: disk driver is READ-ONLY!\n", __func__,
			__LINE__);

	switch (strategy) {
	case PS3DISK_DIRECT_IO:
		pr_info("%s:%u: using direct I/O\n", __func__, __LINE__);
		break;

	case PS3DISK_BOUNCE:
		pr_info("%s:%u: using a bounce buffer of %u bytes\n", __func__,
			__LINE__, BOUNCE_SIZE);
		break;

	case PS3DISK_SG:
		pr_info("%s:%u: using a bounce buffer of %u bytes with "
			"scatter-gather\n",
			__func__, __LINE__, BOUNCE_SIZE);
		break;

	case PS3DISK_TCQ:
		pr_info("%s:%u: using tagged command queuing\n", __func__,
			__LINE__);
		printk(KERN_ERR "%s:%u: NOT YET IMPLEMENTED\n", __func__,
		       __LINE__);
		return -EINVAL;	// FIXME Not yet implemented

	default:
		printk(KERN_ERR "%s:%u: INVALID STRATEGY %d\n", __func__,
		       __LINE__, strategy);
		return -EINVAL;
	}

	return ps3_system_bus_driver_register(&ps3disk, PS3_IOBUS_SB);
}

static void __exit ps3disk_exit(void)
{
	pr_debug("%s:%u\n", __func__, __LINE__);

	unregister_blkdev(ps3disk_major, DEVICE_NAME);

	return ps3_system_bus_driver_unregister(&ps3disk);
}

module_init(ps3disk_init);
module_exit(ps3disk_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("PS3 Disk Storage Driver");
MODULE_AUTHOR("Sony Corporation");

