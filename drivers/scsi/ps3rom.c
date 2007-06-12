//#define NO_ATAPI_FOR_READ_WRITE
#define ALWAYS_USE_DMA_PROTO		// FIXME always safe to use DMA_PROTO?

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

#include <linux/cdrom.h>
#include <linux/highmem.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>

#include <scsi/scsi.h>
#include <scsi/scsi_cmnd.h>
#include <scsi/scsi_dbg.h>
#include <scsi/scsi_device.h>
#include <scsi/scsi_host.h>

#include <asm/lv1call.h>
#include <asm/ps3stor.h>


#define DEVICE_NAME			"ps3rom"

#define BOUNCE_SIZE			(64*1024)

#define PS3ROM_MAX_SECTORS		(BOUNCE_SIZE / CD_FRAMESIZE)

#define LV1_STORAGE_SEND_ATAPI_COMMAND	(1)


struct ps3rom_private {
	struct ps3_storage_device *dev;
	struct scsi_cmnd *cmd;
};
#define ps3rom_priv(dev)	((dev)->sbd.core.driver_data)

struct lv1_atapi_cmnd_block {
	u8	pkt[32];	/* packet command block           */
	u32	pktlen;		/* should be 12 for ATAPI 8020    */
	u32	blocks;
	u32	block_size;
	u32	proto;		/* transfer mode                  */
	u32	in_out;		/* transfer direction             */
	u64	buffer;		/* parameter except command block */
	u32	arglen;		/* length above                   */
};

enum lv1_atapi_proto {
	NON_DATA_PROTO     = 0,
	PIO_DATA_IN_PROTO  = 1,
	PIO_DATA_OUT_PROTO = 2,
	DMA_PROTO = 3
};

enum lv1_atapi_in_out {
	DIR_WRITE = 0, /* memory -> device */
	DIR_READ = 1 /* device -> memory */
};


static int ps3rom_slave_configure(struct scsi_device *scsi_dev)
{
	struct ps3rom_private *priv;
	struct ps3_storage_device *dev;

	priv = (struct ps3rom_private *)scsi_dev->host->hostdata;
	dev = priv->dev;
	dev_dbg(&dev->sbd.core, "%s:%u: id %u, lun %u, channel %u\n", __func__,
		__LINE__, scsi_dev->id, scsi_dev->lun, scsi_dev->channel);

	/*
	 * ATAPI SFF8020 devices use MODE_SENSE_10,
	 * so we can prohibit MODE_SENSE_6
	 */
	scsi_dev->use_10_for_ms = 1;

	return 0;
}

/*
 * copy data from device into scatter/gather buffer
 */
static int fill_from_dev_buffer(struct scsi_cmnd *cmd, const void *buf)
{
	int k, req_len, act_len, len, active;
	void *kaddr;
	struct scatterlist *sgpnt;
	unsigned int buflen;

	buflen = cmd->request_bufflen;
	if (!buflen)
		return 0;

	if (!cmd->request_buffer)
		return DID_ERROR << 16;

	if (cmd->sc_data_direction != DMA_BIDIRECTIONAL &&
	    cmd->sc_data_direction != DMA_FROM_DEVICE)
		return DID_ERROR << 16;

	sgpnt = cmd->request_buffer;
	active = 1;
	for (k = 0, req_len = 0, act_len = 0; k < cmd->use_sg; ++k, ++sgpnt) {
		if (active) {
			kaddr = kmap_atomic(sgpnt->page, KM_USER0);
			if (!kaddr)
				return DID_ERROR << 16;
			len = sgpnt->length;
			if ((req_len + len) > buflen) {
				active = 0;
				len = buflen - req_len;
			}
			memcpy(kaddr + sgpnt->offset, buf + req_len, len);
			kunmap_atomic(kaddr, KM_USER0);
			act_len += len;
		}
		req_len += sgpnt->length;
	}
	cmd->resid = req_len - act_len;
	return 0;
}

/*
 * copy data from scatter/gather into device's buffer
 */
static int fetch_to_dev_buffer(struct scsi_cmnd *cmd, void *buf)
{
	int k, req_len, len, fin;
	void *kaddr;
	struct scatterlist *sgpnt;
	unsigned int buflen;

	buflen = cmd->request_bufflen;
	if (!buflen)
		return 0;

	if (!cmd->request_buffer)
		return -1;

	if (cmd->sc_data_direction != DMA_BIDIRECTIONAL &&
	    cmd->sc_data_direction != DMA_TO_DEVICE)
		return -1;

	sgpnt = cmd->request_buffer;
	for (k = 0, req_len = 0, fin = 0; k < cmd->use_sg; ++k, ++sgpnt) {
		kaddr = kmap_atomic(sgpnt->page, KM_USER0);
		if (!kaddr)
			return -1;
		len = sgpnt->length;
		if ((req_len + len) > buflen) {
			len = buflen - req_len;
			fin = 1;
		}
		memcpy(buf + req_len, kaddr + sgpnt->offset, len);
		kunmap_atomic(kaddr, KM_USER0);
		if (fin)
			return req_len + len;
		req_len += sgpnt->length;
	}
	return req_len;
}

static int decode_lv1_status(u64 status, unsigned char *sense_key,
			     unsigned char *asc, unsigned char *ascq)
{
	if (((status >> 24) & 0xff) != SAM_STAT_CHECK_CONDITION)
		return -1;

	*sense_key = (status >> 16) & 0xff;
	*asc       = (status >>  8) & 0xff;
	*ascq      =  status        & 0xff;
	return 0;
}

static inline unsigned int srb6_lba(const struct scsi_cmnd *cmd)
{
	BUG_ON(cmd->cmnd[1] & 0xe0);	// FIXME lun == 0
	return cmd->cmnd[1] << 16 | cmd->cmnd[2] << 8 | cmd->cmnd[3];
}

static inline unsigned int srb6_len(const struct scsi_cmnd *cmd)
{
	return cmd->cmnd[4];
}

static inline unsigned int srb10_lba(const struct scsi_cmnd *cmd)
{
	return cmd->cmnd[2] << 24 | cmd->cmnd[3] << 16 | cmd->cmnd[4] << 8 |
	       cmd->cmnd[5];
}

static inline unsigned int srb10_len(const struct scsi_cmnd *cmd)
{
	return cmd->cmnd[7] << 8 | cmd->cmnd[8];
}

static void ps3rom_end_command(struct scsi_cmnd *cmd, int result)
{
	struct ps3rom_private *priv;
	struct ps3_storage_device *dev;

	priv = (struct ps3rom_private *)cmd->device->host->hostdata;
	dev = priv->dev;

	dev_dbg(&dev->sbd.core, "%s:%u: command 0x%02x result 0x%x\n",
		__func__, __LINE__, cmd->cmnd[0], result);

	cmd->result = result;
	priv->cmd = NULL;
	cmd->scsi_done(cmd);
}

static int ps3rom_send_atapi_command(struct ps3_storage_device *dev,
				     struct lv1_atapi_cmnd_block *cmd)
{
	int res;

	dev_dbg(&dev->sbd.core, "%s:%u: send ATAPI command 0x%02x\n", __func__,
		__LINE__, cmd->pkt[0]);

	res = lv1_storage_send_device_command(dev->sbd.dev_id,
					      LV1_STORAGE_SEND_ATAPI_COMMAND,
					      ps3_mm_phys_to_lpar(__pa(cmd)),
					      sizeof(*cmd), cmd->buffer,
					      cmd->arglen, &dev->tag);
	if (res == LV1_DENIED_BY_POLICY)
		dev_dbg(&dev->sbd.core,
			"%s:%u: ATAPI command 0x%02x denied by policy\n",
			__func__, __LINE__, cmd->pkt[0]);
	else if (res)
		dev_err(&dev->sbd.core,
			"%s:%u: ATAPI command 0x%02x failed %d\n", __func__,
			__LINE__, cmd->pkt[0], res);

	return res;
}

static int ps3rom_atapi_request(struct ps3_storage_device *dev,
				struct scsi_cmnd *cmd)
{
	struct lv1_atapi_cmnd_block atapi_cmnd;
	unsigned char *cmnd = cmd->cmnd;
	int res;

	memset(&atapi_cmnd, 0, sizeof(struct lv1_atapi_cmnd_block));
	memcpy(&atapi_cmnd.pkt, cmnd, 12);
	atapi_cmnd.pktlen = 12;

	switch (cmd->sc_data_direction) {
	case DMA_FROM_DEVICE:
#ifdef ALWAYS_USE_DMA_PROTO
		atapi_cmnd.proto = DMA_PROTO;
#else
		if (cmd->cmnd[0] == GPCMD_READ_CD)
			atapi_cmnd.proto = DMA_PROTO;
		else
			atapi_cmnd.proto = PIO_DATA_IN_PROTO;
#endif
		atapi_cmnd.in_out = DIR_READ;
		break;

	case DMA_TO_DEVICE:
#ifdef ALWAYS_USE_DMA_PROTO
		// FIXME always safe to use DMA_PROTO?
		atapi_cmnd.proto = DMA_PROTO;
#else
		atapi_cmnd.proto = PIO_DATA_OUT_PROTO;
#endif
		atapi_cmnd.in_out = DIR_WRITE;
		// FIXME check error
		fetch_to_dev_buffer(cmd, dev->bounce_buf);
		break;

	default:
		atapi_cmnd.proto = NON_DATA_PROTO;
		break;
	}

	atapi_cmnd.block_size = 1; /* transfer size is block_size * blocks */

	atapi_cmnd.blocks = atapi_cmnd.arglen = cmd->request_bufflen;
	atapi_cmnd.buffer = dev->bounce_lpar;

	res = ps3rom_send_atapi_command(dev, &atapi_cmnd);
	if (res)
		return DID_ERROR << 16;

	return 0;
}

#ifdef NO_ATAPI_FOR_READ_WRITE
static int ps3rom_read_request(struct ps3_storage_device *dev,
			       struct scsi_cmnd *cmd, u32 start_sector,
			       u32 sectors)
{
	int res;

	dev_dbg(&dev->sbd.core, "%s:%u: read %u sectors starting at %u\n",
		__func__, __LINE__, sectors, start_sector);

	res = lv1_storage_read(dev->sbd.dev_id,
			       dev->regions[dev->region_idx].id, start_sector,
			       sectors, 0, dev->bounce_lpar, &dev->tag);
	if (res) {
		dev_err(&dev->sbd.core, "%s:%u: read failed %d\n", __func__,
			__LINE__, res);
		return DID_ERROR << 16;
	}

	return 0;
}

static int ps3rom_write_request(struct ps3_storage_device *dev,
				struct scsi_cmnd *cmd, u32 start_sector,
				u32 sectors)
{
	int res;

	dev_dbg(&dev->sbd.core, "%s:%u: write %u sectors starting at %u\n",
		__func__, __LINE__, sectors, start_sector);

	// FIXME check error
	fetch_to_dev_buffer(cmd, dev->bounce_buf);

	res = lv1_storage_write(dev->sbd.dev_id,
				dev->regions[dev->region_idx].id, start_sector,
				sectors, 0, dev->bounce_lpar, &dev->tag);
	if (res) {
		dev_err(&dev->sbd.core, "%s:%u: write failed %d\n", __func__,
			__LINE__, res);
		return DID_ERROR << 16;
	}

	return 0;
}
#endif // NO_ATAPI_FOR_READ_WRITE

static irqreturn_t ps3rom_interrupt(int irq, void *data)
{
	struct ps3_storage_device *dev = data;
	struct Scsi_Host *host;
	struct ps3rom_private *priv;
	struct scsi_cmnd *cmd;
	unsigned char opcode, sense_key, asc, ascq;

	dev->lv1_res = lv1_storage_get_async_status(dev->sbd.dev_id,
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
	if (dev->lv1_res) {
		dev_err(&dev->sbd.core, "%s:%u: res=%d status=0x%lx\n",
			__func__, __LINE__, dev->lv1_res, dev->lv1_status);
		return IRQ_HANDLED;
	}

	host = ps3rom_priv(dev);
	priv = (struct ps3rom_private *)host->hostdata;
	cmd = priv->cmd;
	opcode = cmd->cmnd[0];

#ifdef NO_ATAPI_FOR_READ_WRITE
	if (opcode == READ_6 || opcode == READ_10) {
		/* read request */
		if (dev->lv1_status) {
			memset(cmd->sense_buffer, 0, SCSI_SENSE_BUFFERSIZE);
			decode_lv1_status(dev->lv1_status,
					  &cmd->sense_buffer[2],
					  &cmd->sense_buffer[12],
					  &cmd->sense_buffer[13]);
			cmd->sense_buffer[7] = 16 - 6;	// FIXME hardcoded numbers?
			ps3rom_end_command(cmd, SAM_STAT_CHECK_CONDITION);
		} else {
			// FIXME check error
			fill_from_dev_buffer(cmd, dev->bounce_buf);

			ps3rom_end_command(cmd, DID_OK << 16);
		}
	} else if (opcode == WRITE_6 || opcode == WRITE_10) {
		/* write request */
		if (dev->lv1_status) {
			memset(cmd->sense_buffer, 0, SCSI_SENSE_BUFFERSIZE);
			decode_lv1_status(dev->lv1_status,
					  &cmd->sense_buffer[2],
					  &cmd->sense_buffer[12],
					  &cmd->sense_buffer[13]);
			cmd->sense_buffer[7] = 16 - 6;	// FIXME hardcoded numbers?
			ps3rom_end_command(cmd, SAM_STAT_CHECK_CONDITION);
		} else {
			ps3rom_end_command(cmd, DID_OK << 16);
		}
	} else
#endif // NO_ATAPI_FOR_READ_WRITE
	{
		/* ATAPI request */
		if (!dev->lv1_status) {
			/* OK, completed */
			if (cmd->sc_data_direction == DMA_FROM_DEVICE) {
				// FIXME check error
				fill_from_dev_buffer(cmd, dev->bounce_buf);
			}
			ps3rom_end_command(cmd, DID_OK << 16);
		} else if (opcode == REQUEST_SENSE) {
			/* scsi spec says request sense should never get error */
			dev_err(&dev->sbd.core,
				"%s:%u: end error without autosense\n",
				__func__, __LINE__);
			ps3rom_end_command(cmd,
					   DID_ERROR << 16 |
					   CHECK_CONDITION << 1);
		} else if (decode_lv1_status(dev->lv1_status, &sense_key, &asc,
					     &ascq)) {
			/* FIXME: is better other error code ? */
			ps3rom_end_command(cmd, DID_ERROR << 16);
		} else {
			/* lv1 may have issued autosense ... */
			cmd->sense_buffer[0]  = 0x70;
			cmd->sense_buffer[2]  = sense_key;
			cmd->sense_buffer[7]  = 16 - 6;
			cmd->sense_buffer[12] = asc;
			cmd->sense_buffer[13] = ascq;
			ps3rom_end_command(cmd, SAM_STAT_CHECK_CONDITION);
		}
	}
	return IRQ_HANDLED;
}

static int ps3rom_queuecommand(struct scsi_cmnd *cmd,
			       void (*done)(struct scsi_cmnd *))
{
	struct ps3rom_private *priv;
	struct ps3_storage_device *dev;
	unsigned char opcode;
	int res;

#ifdef DEBUG
	scsi_print_command(cmd);
#endif

	priv = (struct ps3rom_private *)cmd->device->host->hostdata;
	dev = priv->dev;
	priv->cmd = cmd;
	cmd->scsi_done = done;

	opcode = cmd->cmnd[0];

	switch (opcode) {
#ifdef NO_ATAPI_FOR_READ_WRITE
	case READ_6:
		res = ps3rom_read_request(dev, cmd, srb6_lba(cmd),
					  srb6_len(cmd));
		break;

	case READ_10:
		res = ps3rom_read_request(dev, cmd, srb10_lba(cmd),
					  srb10_len(cmd));
		break;

	case WRITE_6:
		res = ps3rom_write_request(dev, cmd, srb6_lba(cmd),
					   srb6_len(cmd));
		break;

	case WRITE_10:
		res = ps3rom_write_request(dev, cmd, srb10_lba(cmd),
					   srb10_len(cmd));
		break;
#endif // NO_ATAPI_FOR_READ_WRITE

	default:
		res = ps3rom_atapi_request(dev, cmd);
		break;
	}

	if (res) {
		memset(cmd->sense_buffer, 0, SCSI_SENSE_BUFFERSIZE);
		cmd->sense_buffer[0] = 0x70;
		cmd->sense_buffer[2] = ILLEGAL_REQUEST;
		ps3rom_end_command(cmd, res);
	}

	return 0;
}


static struct scsi_host_template ps3rom_host_template = {
	.name =			DEVICE_NAME,
	.slave_configure =	ps3rom_slave_configure,
	.queuecommand =		ps3rom_queuecommand,
	.can_queue =		1,
	.this_id =		7,
	.sg_tablesize =		SG_ALL,
	.cmd_per_lun =		1,
	.emulated =             1,		/* only sg driver uses this */
	.max_sectors =		PS3ROM_MAX_SECTORS,
	.use_clustering =	ENABLE_CLUSTERING,
	.module =		THIS_MODULE,
};


static int __devinit ps3rom_probe(struct ps3_system_bus_device *_dev)
{
	struct ps3_storage_device *dev = to_ps3_storage_device(&_dev->core);
	int error;
	struct Scsi_Host *host;
	struct ps3rom_private *priv;

	if (dev->blk_size != CD_FRAMESIZE) {
		dev_err(&dev->sbd.core,
			"%s:%u: cannot handle block size %lu\n", __func__,
			__LINE__, dev->blk_size);
		return -EINVAL;
	}

	dev->bounce_size = BOUNCE_SIZE;
	dev->bounce_buf = kmalloc(BOUNCE_SIZE, GFP_DMA);
	if (!dev->bounce_buf) {
		return -ENOMEM;
	}

	error = ps3stor_setup(dev);
	if (error)
		goto fail_free_bounce;

	/* override the interrupt handler */
	free_irq(dev->irq, dev);
	error = request_irq(dev->irq, ps3rom_interrupt, IRQF_DISABLED,
			    dev->sbd.core.driver->name, dev);
	if (error) {
		dev_err(&dev->sbd.core, "%s:%u: request_irq failed %d\n",
			__func__, __LINE__, error);
		goto fail_teardown;
	}

	host = scsi_host_alloc(&ps3rom_host_template,
			       sizeof(struct ps3rom_private));
	if (!host) {
		dev_err(&dev->sbd.core, "%s:%u: scsi_host_alloc failed\n",
			__func__, __LINE__);
		goto fail_teardown;
	}

	priv = (struct ps3rom_private *)host->hostdata;
	ps3rom_priv(dev) = host;
	priv->dev = dev;

	/* One device/LUN per SCSI bus */
	host->max_id = 1;
	host->max_lun = 1;

	error = scsi_add_host(host, &dev->sbd.core);
	if (error) {
		dev_err(&dev->sbd.core, "%s:%u: scsi_host_alloc failed %d\n",
			__func__, __LINE__, error);
		error = -ENODEV;
		goto fail_host_put;
	}

	scsi_scan_host(host);
	return 0;

fail_host_put:
	scsi_host_put(host);
fail_teardown:
	ps3stor_teardown(dev);
fail_free_bounce:
	kfree(dev->bounce_buf);
	return error;
}

static int ps3rom_remove(struct ps3_system_bus_device *_dev)
{
	struct ps3_storage_device *dev = to_ps3_storage_device(&_dev->core);
	struct Scsi_Host *host = ps3rom_priv(dev);

	scsi_remove_host(host);
	ps3stor_teardown(dev);
	kfree(dev->bounce_buf);
	scsi_host_put(host);
	return 0;
}


static struct ps3_system_bus_driver ps3rom = {
	.match_id	= PS3_MATCH_ID_STOR_ROM,
	.core.name	= DEVICE_NAME,
	.core.owner	= THIS_MODULE,
	.probe		= ps3rom_probe,
	.remove		= ps3rom_remove
};


static int __init ps3rom_init(void)
{
	return ps3_system_bus_driver_register(&ps3rom);
}

static void __exit ps3rom_exit(void)
{
	ps3_system_bus_driver_unregister(&ps3rom);
}

module_init(ps3rom_init);
module_exit(ps3rom_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("PS3 ROM Storage Driver");
MODULE_AUTHOR("Sony Corporation");
MODULE_ALIAS(PS3_MODULE_ALIAS_STOR_ROM);
