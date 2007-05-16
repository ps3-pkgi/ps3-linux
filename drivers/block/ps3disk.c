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
#include <linux/interrupt.h>
#include <linux/kthread.h>

#include <asm/lv1call.h>
#include <asm/ps3stor.h>


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
#define PS3DISK_SECTORS		63

#define PS3DISK_NAME		"ps3d%c"


struct ps3disk_private {
	spinlock_t lock;
	struct task_struct *thread;
	struct request_queue *queue;
	struct gendisk *gendisk;
};
#define ps3disk_priv(dev)	((dev)->sbd.core.driver_data)

static int ps3disk_major = PS3DISK_MAJOR;

static int ps3disk_open(struct inode *inode, struct file *file)
{
	struct ps3_storage_device *dev = inode->i_bdev->bd_disk->private_data;

	file->private_data = dev;
	return 0;
}

static int ps3disk_release(struct inode *inode, struct file *file)
{
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

static ssize_t ps3disk_read_write(struct ps3_storage_device *dev,
				  u64 start_sector, u64 sectors, int write)
{
	u64 bytes, hw_start_sector, hw_sectors, res;

	bytes = sectors * KERNEL_SECTOR_SIZE;

	// FIXME Precalculate dev->blk_size/KERNEL_SECTOR_SIZE somewhere
	hw_start_sector = start_sector * (dev->blk_size/KERNEL_SECTOR_SIZE);
	hw_sectors = sectors * (dev->blk_size/KERNEL_SECTOR_SIZE);

	res = ps3stor_read_write_sectors(dev, dev->bounce_lpar,
					 hw_start_sector, hw_sectors, write);
	return res ? -EIO : sectors;
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
}

static void ps3disk_handle_request_sg(struct ps3_storage_device *dev,
				      struct request *req)
{
	struct ps3disk_private *priv = ps3disk_priv(dev);
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

	res = ps3disk_read_write(dev, req->sector, req->nr_sectors, write);
	if (res < 0)
		uptodate = 0;
	else if (!write)
		ps3disk_scatter_gather(dev, req, 0);

	spin_lock_irq(&priv->lock);
	if (!end_that_request_first(req, uptodate, req->nr_sectors)) {
		blkdev_dequeue_request(req);
		end_that_request_last(req, uptodate);
	}
	spin_unlock_irq(&priv->lock);
}

static int ps3disk_thread(void *data)
{
	struct ps3_storage_device *dev = data;
	struct ps3disk_private *priv = ps3disk_priv(dev);
	request_queue_t *q = priv->queue;
	struct request *req;

	dev_dbg(&dev->sbd.core, "%s thread init\n", __func__);

	current->flags |= PF_NOFREEZE;

	while (!kthread_should_stop()) {
		spin_lock_irq(&priv->lock);
		set_current_state(TASK_INTERRUPTIBLE);
		req = elv_next_request(q);
		if (!req) {
			spin_unlock_irq(&priv->lock);
			schedule();
			continue;
		}
		if (!blk_fs_request(req)) {
			blk_dump_rq_flags(req, DEVICE_NAME " bad request");
			end_request(req, 0);
			spin_unlock_irq(&priv->lock);
			continue;
		}
		spin_unlock_irq(&priv->lock);
		ps3disk_handle_request_sg(dev, req);
	}

	dev_dbg(&dev->sbd.core, "%s thread exit\n", __func__);
	return 0;
}

static void ps3disk_request(request_queue_t *q)
{
	struct ps3_storage_device *dev = q->queuedata;
	struct ps3disk_private *priv = ps3disk_priv(dev);
	wake_up_process(priv->thread);
}

static unsigned long ps3disk_mask;

static int ps3disk_probe(struct ps3_system_bus_device *_dev)
{
	struct ps3_storage_device *dev = to_ps3_storage_device(&_dev->core);
	struct ps3disk_private *priv;
	int res, error;
	unsigned int idx, devidx;
	struct request_queue *queue;
	struct gendisk *gendisk;
	struct task_struct *task;

	BUILD_BUG_ON(PS3DISK_MAX_DISKS > BITS_PER_LONG);
	devidx = find_first_zero_bit(&ps3disk_mask, PS3DISK_MAX_DISKS);
	if (devidx >= PS3DISK_MAX_DISKS) {
		dev_err(&dev->sbd.core, "%s:%u: Too many disks\n", __func__,
			__LINE__);
		return -ENOSPC;
	}
	__set_bit(devidx, &ps3disk_mask);

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

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		error = -ENOMEM;
		goto fail;
	}

	ps3disk_priv(dev) = priv;
	spin_lock_init(&priv->lock);

	dev->bounce_size = BOUNCE_SIZE;
	dev->bounce_buf = kmalloc(BOUNCE_SIZE, GFP_DMA);
	if (!dev->bounce_buf) {
		error = -ENOMEM;
		goto fail_free_priv;
	}

	error = ps3_open_hv_device(&dev->sbd.did);
	if (error) {
		dev_err(&dev->sbd.core,
			"%s:%u: ps3_open_hv_device failed %d\n", __func__,
			__LINE__, error);
		goto fail_free_bounce;
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

	error = request_irq(dev->irq, ps3stor_interrupt, IRQF_DISABLED,
			    DEVICE_NAME, dev);
	if (error) {
		dev_err(&dev->sbd.core, "%s:%u: request_irq failed %d\n",
			__func__, __LINE__, error);
		goto fail_sb_event_receive_port_destroy;
	}

	dev->bounce_lpar = ps3_mm_phys_to_lpar(__pa(dev->bounce_buf));

	dev->sbd.d_region = &dev->dma;
	ps3_dma_region_init(&dev->dma, &dev->sbd.did, PS3_DMA_4K,
			    PS3_DMA_OTHER, dev->bounce_buf, dev->bounce_size,
			    PS3_IOBUS_SB);
	res = ps3_dma_region_create(&dev->dma);
	if (res) {
		dev_err(&dev->sbd.core, "%s:%u: cannot create DMA region\n",
			__func__, __LINE__);
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

	queue = blk_init_queue(ps3disk_request, &priv->lock);
	if (!queue) {
		dev_err(&dev->sbd.core, "%s:%u: blk_init_queue failed\n",
			__func__, __LINE__);
		error = -ENOMEM;
		goto fail_unmap_dma;
	}

	priv->queue = queue;
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

	priv->gendisk = gendisk;
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
	priv->thread = task;

	add_disk(gendisk);
	return 0;

fail_free_disk:
	put_disk(priv->gendisk);
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
fail_free_bounce:
	kfree(dev->bounce_buf);
fail_free_priv:
	kfree(priv);
fail:
	__clear_bit(devidx, &ps3disk_mask);
	return error;
}

static int ps3disk_remove(struct ps3_system_bus_device *_dev)
{
	struct ps3_storage_device *dev = to_ps3_storage_device(&_dev->core);
	struct ps3disk_private *priv = ps3disk_priv(dev);
	int error;

	if (priv->thread)
		kthread_stop(priv->thread);
	if (priv->gendisk) {
		__clear_bit(priv->gendisk->first_minor / PS3DISK_MINORS,
			    &ps3disk_mask);
		del_gendisk(priv->gendisk);
		put_disk(priv->gendisk);
	}
	if (priv->queue)
		blk_cleanup_queue(priv->queue);
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
			"%s:%u: ps3_close_hv_device failed %d\n", __func__,
			__LINE__, error);

	kfree(dev->bounce_buf);
	kfree(priv);
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

	return ps3_system_bus_driver_register(&ps3disk, PS3_IOBUS_SB);
}

static void __exit ps3disk_exit(void)
{
	unregister_blkdev(ps3disk_major, DEVICE_NAME);

	return ps3_system_bus_driver_unregister(&ps3disk);
}

module_init(ps3disk_init);
module_exit(ps3disk_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("PS3 Disk Storage Driver");
MODULE_AUTHOR("Sony Corporation");

