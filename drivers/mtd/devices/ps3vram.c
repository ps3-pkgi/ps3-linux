/**
 * ps3vram - Use extra PS3 video ram as MTD block device.
 *
 * Copyright (c) 2007-2008 Jim Paris <jim@jtan.com>
 * Added support RSX DMA Vivien Chappelier <vivien.chappelier@free.fr>
 */

#include <asm/io.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/mtd/mtd.h>
#include <linux/version.h>
#include <linux/gfp.h>
#include <linux/delay.h>

#include <asm/lv1call.h>
#include <asm/ps3.h>

#define XDR_BUF_SIZE (2 * 1024 * 1024) /* XDR buffer (must be 1MB aligned) */
#define XDR_IOIF 0x0c000000

#define FIFO_BASE XDR_IOIF
#define FIFO_SIZE (64 * 1024)

#define DMA_PAGE_SIZE (4 * 1024)

#define CACHE_PAGE_SIZE (256 * 1024)
#define CACHE_PAGE_COUNT ((XDR_BUF_SIZE - FIFO_SIZE) / CACHE_PAGE_SIZE)

#define CACHE_OFFSET CACHE_PAGE_SIZE
#define FIFO_OFFSET 0

#define CTRL_PUT 0x10
#define CTRL_GET 0x11
#define CTRL_TOP 0x15

#define BEGIN_RING(chan, tag, size) \
	OUT_RING(((size) << 18) | ((chan) << 13) | (tag))
#define OUT_RING(data) *(priv->fifo_ptr)++ = (data)
#define FIRE_RING() ps3vram_fire_ring(mtd)

#define UPLOAD_SUBCH	1
#define DOWNLOAD_SUBCH	2

#define NV_MEMORY_TO_MEMORY_FORMAT_OFFSET_IN	0x0000030c
#define NV_MEMORY_TO_MEMORY_FORMAT_NOTIFY	0x00000104

#define L1GPU_CONTEXT_ATTRIBUTE_FB_BLIT 0x601

struct mtd_info ps3vram_mtd;

#define CACHE_PAGE_PRESENT 1
#define CACHE_PAGE_DIRTY   2

#define dbg(fmt, args...)					\
{								\
	if(debug)						\
		printk(KERN_INFO "%s:%d " fmt "\n",		\
		       __FUNCTION__, __LINE__, ## args);	\
} while(0)

struct ps3vram_tag {
	unsigned int address;
	unsigned int flags;
};

struct ps3vram_cache {
	unsigned int page_count;
	unsigned int page_size;
	struct ps3vram_tag *tags;
	int hit;
	int miss;
};

struct ps3vram_priv {
	uint64_t memory_handle;
	uint64_t context_handle;
	uint8_t *base;
	uint8_t *real_base;
	uint32_t *ctrl;
	uint32_t *reports;
	uint8_t *xdr_buf;

	uint32_t *fifo_base;
	uint32_t *fifo_ptr;

	uint32_t reserved;

	struct ps3vram_cache cache;

	struct mutex lock;
};

#define DMA_NOTIFIER_HANDLE_BASE 0x66604200 /* first DMA notifier handle */
#define DMA_NOTIFIER_OFFSET_BASE 0x1000     /* first DMA notifier offset */
#define DMA_NOTIFIER_SIZE        0x40

#define NOTIFIER 8	/* notifier used for completion report */

/* XXX: reserved space in GDDR RAM, used by framebuffer */
#define SKIP_SIZE (16 * 1024 * 1024)

int debug;
module_param(debug, bool, 0);
MODULE_PARM_DESC(debug, "debug mode");

char *size = "256M";
module_param(size, charp, 0);
MODULE_PARM_DESC(size, "memory size");

static void ps3vram_notifier_reset(struct mtd_info *mtd)
{
	struct ps3vram_priv *priv = mtd->priv;
	uint32_t *notify = (uint32_t *) ((void *) priv->reports +
		DMA_NOTIFIER_OFFSET_BASE + DMA_NOTIFIER_SIZE * NOTIFIER);
	notify[0] = notify[1] = notify[2] = notify[3] = 0xffffffff;
}

static int ps3vram_notifier_wait(struct mtd_info *mtd, int timeout_ms)
{
	struct ps3vram_priv *priv = mtd->priv;
	uint32_t *notify = (uint32_t *) ((void *) priv->reports +
		DMA_NOTIFIER_OFFSET_BASE + DMA_NOTIFIER_SIZE * NOTIFIER);

	timeout_ms *= 1000;

	do {
		if (notify[3] == 0)
			return 0;

		if (timeout_ms)
			udelay(1);
	} while(timeout_ms--);

	return -1;
}

static void ps3vram_dump_ring(struct mtd_info *mtd)
{
	struct ps3vram_priv *priv = mtd->priv;
	uint32_t *fifo;

	printk("PUT = %08x GET = %08x\n",
	       priv->ctrl[CTRL_PUT],
	       priv->ctrl[CTRL_GET]);
	for (fifo = priv->fifo_base; fifo < priv->fifo_ptr; fifo++) {
		printk("%p: %08x\n", fifo, *fifo);
	}
}

static void ps3vram_dump_reports(struct mtd_info *mtd)
{
	struct ps3vram_priv *priv = mtd->priv;
	int i;

	for (i = 0; i < 16; i++) {
		uint32_t *n = (uint32_t *) ((void *) priv->reports + 0x1000
			+ 0x40 * i);
		printk("%p: %08x\n", n, *n);
	}
}

static void ps3vram_init_ring(struct mtd_info *mtd)
{
	struct ps3vram_priv *priv = mtd->priv;

	priv->ctrl[CTRL_PUT] = FIFO_BASE + FIFO_OFFSET;
	priv->ctrl[CTRL_GET] = FIFO_BASE + FIFO_OFFSET;
}

static int ps3vram_wait_ring(struct mtd_info *mtd, int timeout)
{
	struct ps3vram_priv *priv = mtd->priv;

	/* wait until setup commands are processed */
	timeout *= 1000;
	while (--timeout) {
		if (priv->ctrl[CTRL_PUT] == priv->ctrl[CTRL_GET])
			break;
		udelay(1);
	}
	if (timeout == 0) {
		printk(KERN_ERR "FIFO timeout (%08x/%08x/%08x)\n",
		       priv->ctrl[CTRL_PUT], priv->ctrl[CTRL_GET],
				priv->ctrl[CTRL_TOP]);
		return -ETIMEDOUT;
	}

        return 0;
}

static void ps3vram_rewind_ring(struct mtd_info *mtd)
{
	struct ps3vram_priv *priv = mtd->priv;
	u64 status;

	OUT_RING(0x20000000 | (FIFO_BASE + FIFO_OFFSET));

	priv->ctrl[CTRL_PUT] = FIFO_BASE + FIFO_OFFSET;

	/* asking the HV for a blit will kick the fifo */
	status = lv1_gpu_context_attribute(priv->context_handle,
					   L1GPU_CONTEXT_ATTRIBUTE_FB_BLIT,
					   0, 0, 0, 0);
	if (status) {
		printk(KERN_ERR
		       "ps3vram: lv1_gpu_context_attribute FB_BLIT failed\n");
	}

	priv->fifo_ptr = priv->fifo_base;
}

static void ps3vram_fire_ring(struct mtd_info *mtd)
{
	struct ps3vram_priv *priv = mtd->priv;
	u64 status;

	priv->ctrl[CTRL_PUT] = FIFO_BASE + FIFO_OFFSET +
		(priv->fifo_ptr - priv->fifo_base) * 4;

	/* asking the HV for a blit will kick the fifo */
	status = lv1_gpu_context_attribute(priv->context_handle,
					   L1GPU_CONTEXT_ATTRIBUTE_FB_BLIT,
					   0, 0, 0, 0);
	if (status) {
		printk(KERN_ERR
		       "ps3vram: lv1_gpu_context_attribute FB_BLIT failed\n");
	}

	if ((priv->fifo_ptr - priv->fifo_base) * 4 > FIFO_SIZE - 1024) {
		dbg("fifo full, rewinding");
		ps3vram_wait_ring(mtd, 200);
		ps3vram_rewind_ring(mtd);
	}
}

static void ps3vram_bind(struct mtd_info *mtd)
{
	struct ps3vram_priv *priv = mtd->priv;

	BEGIN_RING(UPLOAD_SUBCH, 0, 1);
	OUT_RING(0x31337303);
	BEGIN_RING(UPLOAD_SUBCH, 0x180, 3);
	OUT_RING(DMA_NOTIFIER_HANDLE_BASE + NOTIFIER);      // DMA notifier
	OUT_RING(0xfeed0001);      // DMA source from DMA system RAM instance
	OUT_RING(0xfeed0000);      // DMA dest to DMA video RAM instance

	BEGIN_RING(DOWNLOAD_SUBCH, 0, 1);
	OUT_RING(0x3137c0de);
	BEGIN_RING(DOWNLOAD_SUBCH, 0x180, 3);
	OUT_RING(DMA_NOTIFIER_HANDLE_BASE + NOTIFIER);      // DMA notifier
	OUT_RING(0xfeed0000);      // DMA dest to DMA video RAM instance
	OUT_RING(0xfeed0001);      // DMA source from DMA system RAM instance

	FIRE_RING();
}

static int ps3vram_upload(struct mtd_info *mtd,
			  unsigned int src_offset,
			  unsigned int dst_offset, int len, int count)
{
	struct ps3vram_priv *priv = mtd->priv;

	BEGIN_RING(UPLOAD_SUBCH, NV_MEMORY_TO_MEMORY_FORMAT_OFFSET_IN, 8);
	OUT_RING(XDR_IOIF + src_offset);
	OUT_RING(dst_offset);
	OUT_RING(len);
	OUT_RING(len);
	OUT_RING(len);
	OUT_RING(count);
	OUT_RING((1 << 8) | 1);
	OUT_RING(0);

	ps3vram_notifier_reset(mtd);
	BEGIN_RING(UPLOAD_SUBCH, NV_MEMORY_TO_MEMORY_FORMAT_NOTIFY, 1);
	OUT_RING(0);
	BEGIN_RING(UPLOAD_SUBCH, 0x100, 1);
	OUT_RING(0);
	FIRE_RING();
	if (ps3vram_notifier_wait(mtd, 200) < 0) {
		printk(KERN_ERR "notifier timeout\n");
		ps3vram_dump_ring(mtd);
		ps3vram_dump_reports(mtd);
		return -1;
	}

	return 0;
}

static int ps3vram_download(struct mtd_info *mtd,
			    unsigned int src_offset,
			    unsigned int dst_offset, int len, int count)
{
	struct ps3vram_priv *priv = mtd->priv;

	BEGIN_RING(DOWNLOAD_SUBCH, NV_MEMORY_TO_MEMORY_FORMAT_OFFSET_IN, 8);
	OUT_RING(src_offset);
	OUT_RING(XDR_IOIF + dst_offset);
	OUT_RING(len);
	OUT_RING(len);
	OUT_RING(len);
	OUT_RING(count);
	OUT_RING((1 << 8) | 1);
	OUT_RING(0);

	ps3vram_notifier_reset(mtd);
	BEGIN_RING(DOWNLOAD_SUBCH, NV_MEMORY_TO_MEMORY_FORMAT_NOTIFY, 1);
	OUT_RING(0);
	BEGIN_RING(DOWNLOAD_SUBCH, 0x100, 1);
	OUT_RING(0);
	FIRE_RING();
	if (ps3vram_notifier_wait(mtd, 200) < 0) {
		printk(KERN_ERR "notifier timeout\n");
		ps3vram_dump_ring(mtd);
		ps3vram_dump_reports(mtd);
		return -1;
	}

	return 0;
}

static void ps3vram_cache_evict(struct mtd_info *mtd, int entry)
{
	struct ps3vram_priv *priv = mtd->priv;
	struct ps3vram_cache *cache = &priv->cache;

	if (cache->tags[entry].flags & CACHE_PAGE_DIRTY) {
		dbg("flushing %d : 0x%08x",
		    entry, (unsigned int) cache->tags[entry].address);
		if (ps3vram_upload(mtd,
				   CACHE_OFFSET + entry * cache->page_size,
				   priv->reserved + cache->tags[entry].address,
				   DMA_PAGE_SIZE,
				   cache->page_size / DMA_PAGE_SIZE) < 0) {
			printk(KERN_ERR
			       "failed to upload from 0x%x to 0x%x size 0x%x\n",
			       entry * cache->page_size,
			       priv->reserved + cache->tags[entry].address,
			       cache->page_size);
		}
		cache->tags[entry].flags &= ~CACHE_PAGE_DIRTY;
	}
}

static void ps3vram_cache_load(struct mtd_info *mtd, int entry,
	unsigned int address)
{
	struct ps3vram_priv *priv = mtd->priv;
	struct ps3vram_cache *cache = &priv->cache;

	dbg("fetching %d : 0x%08x", entry, address);
	if (ps3vram_download(mtd,
			     priv->reserved + address,
			     CACHE_OFFSET + entry * cache->page_size,
			     DMA_PAGE_SIZE,
			     cache->page_size / DMA_PAGE_SIZE) < 0) {
		printk(KERN_ERR
		       "failed to download from 0x%x to 0x%x size 0x%x\n",
		       priv->reserved + address,
		       entry * cache->page_size,
		       cache->page_size);
	}

	cache->tags[entry].address = address;
	cache->tags[entry].flags |= CACHE_PAGE_PRESENT;
}


static void ps3vram_cache_flush(struct mtd_info *mtd)
{
	struct ps3vram_priv *priv = mtd->priv;
	struct ps3vram_cache *cache = &priv->cache;
	int i;

	dbg("FLUSH");
	for (i = 0; i < cache->page_count; i++) {
		ps3vram_cache_evict(mtd, i);
		cache->tags[i].flags = 0;
	}
}

static unsigned int ps3vram_cache_match(struct mtd_info *mtd, loff_t address)
{
	struct ps3vram_priv *priv = mtd->priv;
	struct ps3vram_cache *cache = &priv->cache;
	unsigned int base;
	unsigned int offset;
	int i;
	static int counter = 0;

	offset = (unsigned int) (address & (cache->page_size - 1));
	base = (unsigned int) (address - offset);

	/* fully associative check */
	for (i = 0; i < cache->page_count; i++) {
		if ((cache->tags[i].flags & CACHE_PAGE_PRESENT) &&
		    cache->tags[i].address == base) {
			dbg("found entry %d : 0x%08x",
			    i, cache->tags[i].address);
			cache->hit++;
			return i;
		}
	}

	/* choose a random entry */
	i = (jiffies + (counter++)) % cache->page_count;
	dbg("using cache entry %d", i);

	ps3vram_cache_evict(mtd, i);
	ps3vram_cache_load(mtd, i, base);

	cache->miss++;
	return i;
}

static int ps3vram_cache_init(struct mtd_info *mtd)
{
	struct ps3vram_priv *priv = mtd->priv;

	printk(KERN_INFO "creating cache: %d entries, %d bytes pages\n",
	       CACHE_PAGE_COUNT, CACHE_PAGE_SIZE);

	priv->cache.page_count = CACHE_PAGE_COUNT;
	priv->cache.page_size = CACHE_PAGE_SIZE;
	priv->cache.tags = kzalloc(sizeof(struct ps3vram_tag) *
				   CACHE_PAGE_COUNT, GFP_KERNEL);
	if (priv->cache.tags == NULL) {
		printk(KERN_ERR "could not allocate cache tags\n");
		return -ENOMEM;
	}

	return 0;
}

static void ps3vram_cache_cleanup(struct mtd_info *mtd)
{
	struct ps3vram_priv *priv = mtd->priv;

	ps3vram_cache_flush(mtd);
	kfree(priv->cache.tags);
}

static int ps3vram_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	struct ps3vram_priv *priv = mtd->priv;

        if (instr->addr + instr->len > mtd->size)
                return -EINVAL;

	mutex_lock(&priv->lock);

	ps3vram_cache_flush(mtd);

        /* Set bytes to 0xFF */
        memset(priv->base + instr->addr, 0xFF, instr->len);

	mutex_unlock(&priv->lock);

        instr->state = MTD_ERASE_DONE;
        mtd_erase_callback(instr);

        return 0;
}


static int ps3vram_read(struct mtd_info *mtd, loff_t from, size_t len,
                size_t *retlen, u_char *buf)
{
	struct ps3vram_priv *priv = mtd->priv;
	unsigned int cached, count;

	dbg("from = 0x%08x len = 0x%x",
	    (unsigned int) from, (unsigned int) len);

	if (from >= mtd->size)
		return -EINVAL;

	if (len > mtd->size - from)
		len = mtd->size - from;

        /* Copy from vram to buf */
	count = len;
	while (count) {
		unsigned int offset, avail;
		unsigned int entry;

		offset = (unsigned int) (from & (priv->cache.page_size - 1));
		avail  = priv->cache.page_size - offset;

		mutex_lock(&priv->lock);

		entry = ps3vram_cache_match(mtd, from);
		cached = CACHE_OFFSET + entry * priv->cache.page_size + offset;

		dbg("from=%08x cached=%08x offset=%08x avail=%08x "
			"count=%08x", (unsigned ) from, cached, offset, avail,
			(unsigned) count);

		if (avail > count)
			avail = count;
		memcpy(buf, priv->xdr_buf + cached, avail);

		mutex_unlock(&priv->lock);

		buf += avail;
		count -= avail;
		from += avail;
	}

        *retlen = len;
        return 0;
}

static int ps3vram_write(struct mtd_info *mtd, loff_t to, size_t len,
                size_t *retlen, const u_char *buf)
{
	struct ps3vram_priv *priv = mtd->priv;
	unsigned int cached, count;

	if (to >= mtd->size)
		return -EINVAL;

	if (len > mtd->size - to)
		len = mtd->size - to;

	/* Copy from buf to vram */
	count = len;
	while (count) {
		unsigned int offset, avail;
		unsigned int entry;

		offset = (unsigned int) (to & (priv->cache.page_size - 1));
		avail  = priv->cache.page_size - offset;

		mutex_lock(&priv->lock);

		entry = ps3vram_cache_match(mtd, to);
		cached = CACHE_OFFSET + entry * priv->cache.page_size + offset;

		dbg("to=%08x cached=%08x offset=%08x avail=%08x count=%08x",
		    (unsigned ) to, cached, offset, avail, (unsigned) count);

		if (avail > count)
			avail = count;
		memcpy(priv->xdr_buf + cached, buf, avail);

		priv->cache.tags[entry].flags |= CACHE_PAGE_DIRTY;

		mutex_unlock(&priv->lock);

		buf += avail;
		count -= avail;
		to += avail;
	}

        *retlen = len;
        return 0;
}

/* XXX: Fake structure so we can call ps3_{open,close}_hv_device */
static struct ps3_system_bus_device fake_dev = {
	.match_id = PS3_MATCH_ID_GRAPHICS,
	.dev_type = PS3_DEVICE_TYPE_IOC0,
};

static void unregister_device(void)
{
	struct ps3vram_priv *priv;

	priv = ps3vram_mtd.priv;
	if (priv == NULL)
		return;

	del_mtd_device(&ps3vram_mtd);
	ps3vram_cache_cleanup(&ps3vram_mtd);
	iounmap(priv->reports);
	iounmap(priv->ctrl);
	iounmap(priv->real_base);
	lv1_gpu_context_free(priv->context_handle);
	lv1_gpu_memory_free(priv->memory_handle);
	ps3_close_hv_device(&fake_dev);
	free_pages((unsigned long) priv->xdr_buf, get_order(XDR_BUF_SIZE));
	kfree(priv);

	printk(KERN_INFO "ps3vram mtd device unregistered\n");
}

static int register_device(void)
{
	struct ps3vram_priv *priv;
	uint64_t status;
	uint64_t ddr_lpar, ddr_size, ctrl_lpar, info_lpar, reports_lpar;
	uint64_t reports_size;
	int ret = -ENOMEM;
	char *rest;

	ret = -EIO;
	ps3vram_mtd.priv = kzalloc(sizeof(struct ps3vram_priv), GFP_KERNEL);
	if (!ps3vram_mtd.priv)
		goto out;
	priv = ps3vram_mtd.priv;

	mutex_init(&priv->lock);

	/* Allocate XDR buffer (1MB aligned) */
	priv->xdr_buf = (uint8_t *) __get_free_pages(GFP_KERNEL,
						     get_order(XDR_BUF_SIZE));
	if (priv->xdr_buf == NULL) {
		printk(KERN_ERR "ps3vram: could not allocate XDR buffer\n");
		ret = -ENOMEM;
		goto out_free_priv;
	}

	/* Put FIFO at begginning of XDR buffer */
	priv->fifo_base = priv->fifo_ptr =
		(uint32_t *) (priv->xdr_buf + FIFO_OFFSET);

	/* XXX: Need to open GPU, in case ps3fb or snd_ps3 aren't loaded */
	if (ps3_open_hv_device(&fake_dev)) {
		printk(KERN_ERR "ps3vram: ps3_open_hv_device failed\n");
		ret = -EAGAIN;
		goto out_close_gpu;
	}

	/* Request memory */
	status = -1;
	ddr_size = memparse(size, &rest);
	if (ddr_size < SKIP_SIZE)
		ddr_size = SKIP_SIZE;
	ddr_size = (ddr_size + 1024 * 1024 - 1) & ~(1024 * 1024 - 1);

	while (ddr_size > SKIP_SIZE) {
		status = lv1_gpu_memory_allocate(ddr_size, 0, 0, 0, 0,
					 &priv->memory_handle, &ddr_lpar);
		if (status == 0)
			break;
		ddr_size -= 1024*1024;
	}
	if (status != 0 || ddr_size <= SKIP_SIZE) {
		printk(KERN_ERR "ps3vram: lv1_gpu_memory_allocate failed\n");
		ret = -ENOMEM;
		goto out_free_xdr_buf;
        }
	printk(KERN_INFO "ps3vram: allocated %dMB of DDR memory\n",
	       (unsigned int) (ddr_size / 1024 / 1024));

	/* Request context */
	status = lv1_gpu_context_allocate(priv->memory_handle,
					  0,
					  &priv->context_handle,
					  &ctrl_lpar,
					  &info_lpar,
					  &reports_lpar,
					  &reports_size);
	if (status) {
		printk(KERN_ERR "ps3vram: lv1_gpu_context_allocate failed\n");
		ret = -ENOMEM;
		goto out_free_memory;
	}

	/* Map XDR buffer to RSX */
	status = lv1_gpu_context_iomap(priv->context_handle, XDR_IOIF,
				       ps3_mm_phys_to_lpar(__pa(priv->xdr_buf)),
				       XDR_BUF_SIZE, 0);
	if (status) {
		printk(KERN_ERR "ps3vram: lv1_gpu_context_iomap failed\n");
		ret = -ENOMEM;
		goto out_free_memory;
        }

	priv->base = priv->real_base = ioremap(ddr_lpar, ddr_size);
	if (!priv->real_base) {
		printk(KERN_ERR "ps3vram: ioremap failed\n");
		ret = -ENOMEM;
		goto out_free_context;
	}

	/* XXX: Skip beginning GDDR ram that might belong to the framebuffer. */
	priv->reserved = SKIP_SIZE;
	priv->base += priv->reserved;
	ddr_size -= priv->reserved;

	priv->ctrl = ioremap(ctrl_lpar, 64 * 1024);
	if (!priv->ctrl) {
		printk(KERN_ERR "ps3vram: ioremap failed\n");
		ret = -ENOMEM;
		goto out_unmap_vram;
	}

	priv->reports = ioremap(reports_lpar, reports_size);
	if (!priv->reports) {
		printk(KERN_ERR "ps3vram: ioremap failed\n");
		ret = -ENOMEM;
		goto out_unmap_ctrl;
	}

	ps3vram_init_ring(&ps3vram_mtd);

	ps3vram_mtd.name = "ps3vram";
	ps3vram_mtd.size = ddr_size;
	ps3vram_mtd.flags = MTD_CAP_RAM;
	ps3vram_mtd.erase = ps3vram_erase;
	ps3vram_mtd.point = NULL;
	ps3vram_mtd.unpoint = NULL;
	ps3vram_mtd.read = ps3vram_read;
	ps3vram_mtd.write = ps3vram_write;
	ps3vram_mtd.owner = THIS_MODULE;
	ps3vram_mtd.type = MTD_RAM;
	ps3vram_mtd.erasesize = CACHE_PAGE_SIZE;
	ps3vram_mtd.writesize = 1;

	ps3vram_bind(&ps3vram_mtd);

	if (ps3vram_wait_ring(&ps3vram_mtd, 100) < 0) {
		printk(KERN_ERR "failed to initialize channels\n");
		ret = -ETIMEDOUT;
		goto out_unmap_reports;
	}

	ps3vram_cache_init(&ps3vram_mtd);

	if (add_mtd_device(&ps3vram_mtd)) {
		printk(KERN_ERR "ps3vram: failed to register device\n");
		ret = -EAGAIN;
		goto out_cache_cleanup;
	}

	printk(KERN_INFO "ps3vram mtd device registered, %ld bytes\n", ddr_size);

	return 0;

out_cache_cleanup:
	ps3vram_cache_cleanup(&ps3vram_mtd);
out_unmap_reports:
	iounmap(priv->reports);
out_unmap_ctrl:
	iounmap(priv->ctrl);
out_unmap_vram:
	iounmap(priv->real_base);
out_free_context:
	lv1_gpu_context_free(priv->context_handle);
out_free_memory:
	lv1_gpu_memory_free(priv->memory_handle);
out_close_gpu:
	ps3_close_hv_device(&fake_dev);
out_free_xdr_buf:
	free_pages((unsigned long) priv->xdr_buf, get_order(XDR_BUF_SIZE));
out_free_priv:
	kfree(ps3vram_mtd.priv);
	ps3vram_mtd.priv = NULL;
out:
	return ret;
}

static int __init init_ps3vram(void)
{
	return register_device();
}

static void __exit cleanup_ps3vram(void)
{
	unregister_device();
}

module_init(init_ps3vram);
module_exit(cleanup_ps3vram);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jim Paris <jim@jtan.com>");
MODULE_DESCRIPTION("MTD driver for PS3 video RAM");
