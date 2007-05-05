/*
 * Audio support for PS3
 * Copyright (C) 2007 Sony Computer Entertainment Inc.
 * All rights reserved.
 * Copyright 2006, 2007 Sony Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2 of the Licence.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#define DEBUG
#undef _SND_PS3_DEV_ATTR

#include <linux/init.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <sound/driver.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/asound.h>
#include <sound/memalloc.h>
#include <sound/pcm_params.h>
#include <sound/control.h>
#include <linux/dmapool.h>
#include <linux/dma-mapping.h>
#include <asm/firmware.h>
#include <asm/io.h>
#include <asm/dma.h>
#include <asm/lv1call.h>
#include <asm/ps3.h>
#include <asm/ps3av.h>

#include "snd_ps3_reg.h"
#include "snd_ps3.h"

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("PS3 sound driver");
MODULE_AUTHOR("Sony Computer Entertainment Inc.");

static int index = SNDRV_DEFAULT_IDX1;
static char *id = SNDRV_DEFAULT_STR1;

module_param(index, int, 0444);
MODULE_PARM_DESC(index, "Index value for PS3 soundchip.");
module_param(id, charp, 0444);
MODULE_PARM_DESC(id, "ID string for PS3 soundchip.");


module_init(snd_ps3_init);
module_exit(snd_ps3_exit);


#ifdef _SND_PS3_DEV_ATTR
static DEVICE_ATTR(start_delay,
		   S_IRUGO | S_IWUSR,
		   snd_ps3_get_start_delay,
		   snd_ps3_set_start_delay);
#endif

/*
 * global
 */
struct snd_ps3_card_info the_card;

static int snd_ps3_start_delay = CONFIG_SND_PS3_DEFAULT_START_DELAY;

module_param_named(start_delay, snd_ps3_start_delay, int, 0444);
MODULE_PARM_DESC(start_delay, "time to insert silent data in milisec");

/*
 * PS3 audio register access macros
 */

/*
 * chip: pointer to snd_ps3_card_info
 * name: register offset value; PS3_AUDIO_XXXX
 */
#define AUDIOREGPTR(chip, name) \
	(volatile uint32_t *)(chip->mapped_mmio_vaddr + name)

#define AUDIOREG(chip, name) *(AUDIOREGPTR(chip, name))

/*
 * ALSA defs
 */
const static struct snd_pcm_hardware snd_ps3_pcm_hw = {
        .info = (SNDRV_PCM_INFO_MMAP |
                 SNDRV_PCM_INFO_NONINTERLEAVED |
                 SNDRV_PCM_INFO_MMAP_VALID),
        .formats = (SNDRV_PCM_FMTBIT_S16_BE |
		    SNDRV_PCM_FMTBIT_S24_BE),
        .rates = (SNDRV_PCM_RATE_44100 |
		  SNDRV_PCM_RATE_48000 |
		  SNDRV_PCM_RATE_88200 |
		  SNDRV_PCM_RATE_96000),
        .rate_min = 44100,
        .rate_max = 96000,

        .channels_min = 2, /* stereo only */
        .channels_max = 2,

        .buffer_bytes_max = PS3_AUDIO_FIFO_SIZE * 64,

	/* interrupt by four stages */
        .period_bytes_min = PS3_AUDIO_FIFO_STAGE_SIZE * 4,
        .period_bytes_max = PS3_AUDIO_FIFO_STAGE_SIZE * 4,

        .periods_min = 16,
	.periods_max = 32, /* buffer_size_max/ period_bytes_max */

	.fifo_size = PS3_AUDIO_FIFO_SIZE
};

static struct snd_pcm_ops snd_ps3_pcm_spdif_ops =
{
	.open = snd_ps3_pcm_open,
	.close = snd_ps3_pcm_close,
	.prepare = snd_ps3_pcm_prepare,
	.ioctl = snd_pcm_lib_ioctl,
	.trigger = snd_ps3_pcm_trigger,
	.pointer = snd_ps3_pcm_pointer,
	.hw_params = snd_ps3_pcm_hw_params,
	.hw_free = snd_ps3_pcm_hw_free
};

static struct snd_kcontrol_new snd_ps3_vol_control =
{
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "PCM Playback Volume",
	.index = 0,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.private_value = SND_PS3_MAX_VOL,/* not used */
	.info = snd_ps3_info_vol_control,
	.put = snd_ps3_put_vol_control,
	.get = snd_ps3_get_vol_control
};

static int snd_ps3_verify_dma_stop(struct snd_ps3_card_info * card,
				   int count, int force_stop)
{
	int dma_ch, done, retries, stop_forced = 0;
	uint32_t status;

	for (dma_ch = 0; dma_ch < 8; dma_ch ++) {
		retries = count;
		do {
			status = AUDIOREG(card, PS3_AUDIO_KICK(dma_ch)) &
				PS3_AUDIO_KICK_STATUS_MASK;
			switch (status) {
			case PS3_AUDIO_KICK_STATUS_DONE:
			case PS3_AUDIO_KICK_STATUS_NOTIFY:
			case PS3_AUDIO_KICK_STATUS_CLEAR:
			case PS3_AUDIO_KICK_STATUS_ERROR:
				done = 1;
				break;
			default:
				done = 0;
				udelay(10);
			}
		} while (!done && --retries);
		if (!retries && force_stop) {
			printk(KERN_ERR "%s: DMA ch %d is not stopped.",
			       __func__, dma_ch);
			/* last resort. force to stop dma.
			 *  NOTE: this cause DMA done interrupts
			 */
			AUDIOREG(card, PS3_AUDIO_CONFIG) |=
				PS3_AUDIO_CONFIG_CLEAR;
			stop_forced = 1;
		}
	}
	return stop_forced;
}

/*
 * wait for all dma is done.
 * NOTE: caller should reset card->running before call.
 *       If not, the interrupt handler will re-start DMA,
 *       then DMA is never stopped.
 */
static void snd_ps3_wait_for_dma_stop(struct snd_ps3_card_info * card)
{
	int stop_forced;
	/*
	 * wait for the last dma is done
	 */

	/*
	 * expected maximum DMA done time is 5.7ms + something (DMA itself).
	 * 5.7ms is from 16bit/sample 2ch 44.1Khz; the time next
	 * DMA kick event would occur.
	 */
	stop_forced = snd_ps3_verify_dma_stop(card, 700, 1);

	/*
	 * clear outstanding interrupts.
	 */
	AUDIOREG(card, PS3_AUDIO_INTR_0) |= 0;
	AUDIOREG(card, PS3_AUDIO_AX_IS) |= 0;

	/*
	 *revert CLEAR bit since it will not reset automatically after DMA stop
	 */
	if (stop_forced) {
		AUDIOREG(card, PS3_AUDIO_CONFIG) &= ~PS3_AUDIO_CONFIG_CLEAR;
	}
	mb();
}

static void snd_ps3_kick_dma(struct snd_ps3_card_info * card)
{

	AUDIOREG(card, PS3_AUDIO_KICK(0)) |= PS3_AUDIO_KICK_REQUEST;
	mb();
}

/*
 * convert virtual addr to ioif bus addr.
 */
static dma_addr_t v_to_bus(struct snd_ps3_card_info * card,
			   void * paddr,
			   int ch)
{
	return card->dma_start_bus_addr[ch] +
		(paddr - card->dma_start_vaddr[ch]);
};


/*
 * increment ring buffer pointer.
 * NOTE: caller must hold write spinlock
 */
static void snd_ps3_bump_buffer(struct snd_ps3_card_info * card,
				enum snd_ps3_ch ch, size_t byte_count,
				int stage)
{
	if (!stage)
		card->dma_last_transfer_vaddr[ch] =
			card->dma_next_transfer_vaddr[ch];
	card->dma_next_transfer_vaddr[ch] += byte_count;
	if ((card->dma_start_vaddr[ch] + (card->dma_buffer_size / 2)) <=
	    card->dma_next_transfer_vaddr[ch]) {
		card->dma_next_transfer_vaddr[ch] = card->dma_start_vaddr[ch];
	}
}
/*
 * setup dmac to send data to audio and attenuate samples on the ring buffer
 */
static int snd_ps3_program_dma(struct snd_ps3_card_info * card,
			       enum snd_ps3_dma_filltype filltype)
{
	/* this dmac does not support over 4G */
	uint32_t dma_addr;
	int fill_stages, dma_ch, stage;
	enum snd_ps3_ch ch;
	uint32_t ch0_kick_event = 0; /* initialize to mute gcc */
	void * start_vaddr;
	unsigned long irqsave;
	int silent = 0;

	switch (filltype) {
	case SND_PS3_DMA_FILLTYPE_SILENT_FIRSTFILL:
		silent = 1;
		/* intentionally fall thru */
	case SND_PS3_DMA_FILLTYPE_FIRSTFILL:
		ch0_kick_event = PS3_AUDIO_KICK_EVENT_ALWAYS;
		break;

	case SND_PS3_DMA_FILLTYPE_SILENT_RUNNING:
		silent = 1;
		/* intentionally fall thru */
	case SND_PS3_DMA_FILLTYPE_RUNNING:
		ch0_kick_event = PS3_AUDIO_KICK_EVENT_SERIALOUT0_EMPTY;
		break;
	}

	snd_ps3_verify_dma_stop(card, 700, 0);
	fill_stages = 4;
	write_lock_irqsave(&card->dma_lock, irqsave);
	if (likely(!silent))
		snd_ps3_soft_attenuate(card,
				       card->dma_next_transfer_vaddr[0],
				       card->dma_next_transfer_vaddr[1],
				       PS3_AUDIO_DMAC_BLOCK_SIZE * 4);
	for (ch = 0; ch < 2; ch++) {
		start_vaddr = card->dma_next_transfer_vaddr[0];
		for (stage = 0; stage < fill_stages; stage ++) {
			//dma_ch = fill_stages * ch + stage;
			dma_ch = stage * 2 + ch;
			if (silent)
				dma_addr = card->null_buffer_start_dma_addr;
			else
				dma_addr =
				v_to_bus(card,
					 card->dma_next_transfer_vaddr[ch],
					 ch);

			AUDIOREG(card, PS3_AUDIO_SOURCE(dma_ch)) =
				(PS3_AUDIO_SOURCE_TARGET_SYSTEM_MEMORY |
				 dma_addr);

			/* dst: fixed to 3wire#0 */
			if (ch == 0)
				AUDIOREG(card, PS3_AUDIO_DEST(dma_ch)) =
					(PS3_AUDIO_DEST_TARGET_AUDIOFIFO |
					 PS3_AUDIO_AO_3W_LDATA(0));
			else
				AUDIOREG(card, PS3_AUDIO_DEST(dma_ch)) =
					(PS3_AUDIO_DEST_TARGET_AUDIOFIFO |
					 PS3_AUDIO_AO_3W_RDATA(0));

			/* count always 1 DMA block (1/2 stage = 128 bytes) */
			AUDIOREG(card, PS3_AUDIO_DMASIZE(dma_ch)) = 0;
			/* bump pointer if needed */
			if (!silent)
				snd_ps3_bump_buffer(card, ch,
						    PS3_AUDIO_DMAC_BLOCK_SIZE,
						    stage);

			/* kick event  */
			if (dma_ch == 0) {
				AUDIOREG(card, PS3_AUDIO_KICK(dma_ch)) =
					ch0_kick_event;
			} else {
				AUDIOREG(card, PS3_AUDIO_KICK(dma_ch)) =
					(PS3_AUDIO_KICK_EVENT_AUDIO_DMA(dma_ch -
									1) |
					 PS3_AUDIO_KICK_REQUEST);
			}
		}
	}
	mb();
	write_unlock_irqrestore(&card->dma_lock, irqsave);

	mb();
	return 0;
}

/*
 * audio mute on/off
 * mute_on : 0 output enabled
 *           1 mute
 */
static int snd_ps3_mute(int mute_on)
{
	return ps3av_audio_mute(mute_on);
}

/*
 * PCM operators
 */
static int snd_ps3_pcm_open(struct snd_pcm_substream * substream)
{
	struct snd_pcm_runtime * runtime = substream->runtime;
	struct snd_ps3_card_info * card = snd_pcm_substream_chip(substream);
	int pcm_index;

	pcm_index = substream->pcm->device;
	/* to retrieve substream/runtime in interrupt handler */
	card->substream = substream;

	runtime->hw = snd_ps3_pcm_hw;

	/* mute off */
	snd_ps3_mute(0); // this function sleep

	snd_pcm_hw_constraint_step(runtime, 0, SNDRV_PCM_HW_PARAM_BUFFER_BYTES,
				   PS3_AUDIO_FIFO_STAGE_SIZE * 4 * 2);
	return 0;
};

static int snd_ps3_pcm_hw_params(struct snd_pcm_substream * substream,
				 struct snd_pcm_hw_params * hw_params)
{
	size_t size;

	/* alloc transport buffer */
	size = params_buffer_bytes(hw_params);
	snd_pcm_lib_malloc_pages(substream, size);
	return 0;
};

static int snd_ps3_delay_to_bytes(struct snd_pcm_substream * substream,
				  unsigned int delay_ms)
{
	int ret;
	int rate ;

	rate = substream->runtime->rate;
	ret = snd_pcm_format_size(substream->runtime->format,
				  rate * delay_ms / 1000)
		* substream->runtime->channels;

	pr_debug(KERN_ERR "%s: time=%d rate=%d bytes=%ld, frames=%d, ret=%d\n",
		 __func__,
		 delay_ms,
		 rate,
		 snd_pcm_format_size(substream->runtime->format, rate),
		 rate * delay_ms / 1000,
		 ret);

	return ret;
};

static int snd_ps3_pcm_prepare(struct snd_pcm_substream * substream)
{
	struct snd_pcm_runtime * runtime = substream->runtime;
	struct snd_ps3_card_info * card = snd_pcm_substream_chip(substream);
	unsigned long irqsave;

	if (!snd_ps3_set_avsetting(substream)) {
		/* some parameter changed */
		AUDIOREG(card, PS3_AUDIO_AX_IE) = (PS3_AUDIO_AX_IE_ASOBEIE(0) |
						   PS3_AUDIO_AX_IE_ASOBUIE(0));
		/*
		 * let SPDIF device re-lock with SPDIF signal,
		 * start with some silence
		 */
		read_lock(&card->start_delay_lock);
		card->silent = snd_ps3_delay_to_bytes(substream,
						      card->start_delay) /
			(PS3_AUDIO_FIFO_STAGE_SIZE * 4); /* every 4 times */
		read_unlock(&card->start_delay_lock);
	}

	/* restart ring buffer pointer */
	write_lock_irqsave(&card->dma_lock, irqsave);
	{
		card->dma_buffer_size = runtime->dma_bytes;

		card->dma_last_transfer_vaddr[SND_PS3_CH_L] =
			card->dma_next_transfer_vaddr[SND_PS3_CH_L] =
			card->dma_start_vaddr[SND_PS3_CH_L] =
			runtime->dma_area;
		card->dma_start_bus_addr[SND_PS3_CH_L] = runtime->dma_addr;

		card->dma_last_transfer_vaddr[SND_PS3_CH_R] =
			card->dma_next_transfer_vaddr[SND_PS3_CH_R] =
			card->dma_start_vaddr[SND_PS3_CH_R] =
			runtime->dma_area + (runtime->dma_bytes / 2);
		card->dma_start_bus_addr[SND_PS3_CH_R] =
			runtime->dma_addr + (runtime->dma_bytes / 2);

		pr_debug("%s: vaddr=%p bus=%#lx\n", __func__,
			 card->dma_start_vaddr[SND_PS3_CH_L],
			 card->dma_start_bus_addr[SND_PS3_CH_L]);

	}
	write_unlock_irqrestore(&card->dma_lock, irqsave);

	mb();

	return 0;
};

static int snd_ps3_pcm_trigger(struct snd_pcm_substream * substream,
			       int cmd)
{
	struct snd_ps3_card_info * card = snd_pcm_substream_chip(substream);
	int ret = 0;
	unsigned long irqsave;

	switch (cmd)
	{
	case SNDRV_PCM_TRIGGER_START:
		/* clear outstanding interrupts  */
		AUDIOREG(card, PS3_AUDIO_AX_IS) |= 0;

		write_lock_irqsave(&card->dma_lock, irqsave);
		{
			card->running = 1;
		}
		write_unlock_irqrestore(&card->dma_lock, irqsave);

		snd_ps3_program_dma(card,
				    SND_PS3_DMA_FILLTYPE_SILENT_FIRSTFILL);
		snd_ps3_kick_dma(card);
		while (AUDIOREG(card, PS3_AUDIO_KICK(7)) &
		       PS3_AUDIO_KICK_STATUS_MASK) {
			udelay(1);
		}
		snd_ps3_program_dma(card, SND_PS3_DMA_FILLTYPE_SILENT_RUNNING);
		snd_ps3_kick_dma(card);
		break;

	case SNDRV_PCM_TRIGGER_STOP:
		write_lock_irqsave(&card->dma_lock, irqsave);
		{
			card->running = 0;
		}
		write_unlock_irqrestore(&card->dma_lock, irqsave);
		snd_ps3_wait_for_dma_stop(card);
		break;
	default:
		break;

	}

	return ret;
};

/*
 * report current pointer
 */
static snd_pcm_uframes_t snd_ps3_pcm_pointer(
	struct snd_pcm_substream * substream)
{
	struct snd_ps3_card_info * card = snd_pcm_substream_chip(substream);
	unsigned long irqsave;
	size_t bytes;
	snd_pcm_uframes_t ret;

	read_lock_irqsave(&card->dma_lock, irqsave);
	{
		bytes = (size_t)(card->dma_last_transfer_vaddr[SND_PS3_CH_L] -
				 card->dma_start_vaddr[SND_PS3_CH_L]);
	}
	read_unlock_irqrestore(&card->dma_lock, irqsave);

	ret = bytes_to_frames(substream->runtime, bytes * 2);

	return ret;
};

static int snd_ps3_pcm_hw_free(struct snd_pcm_substream * substream)
{
	int ret;
	ret = snd_pcm_lib_free_pages(substream);
	return ret;
};

static int snd_ps3_pcm_close(struct snd_pcm_substream * substream)
{
	/* mute on */
	snd_ps3_mute(1);
	return 0;
};

static void snd_ps3_audio_fixup(struct snd_ps3_card_info * card)
{
	/*
	 * avsetting driver seems to never change the followings
	 * so, init them here once
	 */

	/* no dma interrupt needed */
	AUDIOREG(card, PS3_AUDIO_INTR_EN_0) = 0;

	/* use every 4 buffer empty interrupt */
	AUDIOREG(card, PS3_AUDIO_AX_IC) = ((AUDIOREG(card, PS3_AUDIO_AX_IC) &
					    PS3_AUDIO_AX_IC_AASOIMD_MASK) |
					   PS3_AUDIO_AX_IC_AASOIMD_EVERY4);

	/* enable 3wire clocks */
	AUDIOREG(card, PS3_AUDIO_AO_3WMCTRL) &=
		~(PS3_AUDIO_AO_3WMCTRL_ASOBCLKD_DISABLED |
		  PS3_AUDIO_AO_3WMCTRL_ASOLRCKD_DISABLED);
	AUDIOREG(card, PS3_AUDIO_AO_3WMCTRL) |=
		PS3_AUDIO_AO_3WMCTRL_ASOPLRCK_DEFAULT;
}

/*
 * av setting
 * NOTE: calling this function may generate audio interrupt.
 */
static int snd_ps3_change_avsetting(struct snd_ps3_card_info * card)
{
	int ret, retries, i;

	ret = ps3av_set_audio_mode(card->avs.avs_audio_ch,
				  card->avs.avs_audio_rate,
				  card->avs.avs_audio_width,
				  card->avs.avs_audio_format,
				  card->avs.avs_audio_source);
	/*
	 * Reset the following unwanted settings:
	 */

	/* disable all 3wire buffers */
	AUDIOREG(card, PS3_AUDIO_AO_3WMCTRL) &=
		~(PS3_AUDIO_AO_3WMCTRL_ASOEN(0) |
		  PS3_AUDIO_AO_3WMCTRL_ASOEN(1) |
		  PS3_AUDIO_AO_3WMCTRL_ASOEN(2) |
		  PS3_AUDIO_AO_3WMCTRL_ASOEN(3));
	mb();
	/* wait for actually stopped */
	retries = 1000;
	while ((AUDIOREG(card, PS3_AUDIO_AO_3WMCTRL) &
		(PS3_AUDIO_AO_3WMCTRL_ASORUN(0) |
		 PS3_AUDIO_AO_3WMCTRL_ASORUN(1) |
		 PS3_AUDIO_AO_3WMCTRL_ASORUN(2) |
		 PS3_AUDIO_AO_3WMCTRL_ASORUN(3))) &&
	       --retries) {
		udelay(1);
	}
	mb();
	/* reset buffer pointer */
	for (i = 0; i < 4; i++) {
		AUDIOREG(card, PS3_AUDIO_AO_3WCTRL(i)) |=
			PS3_AUDIO_AO_3WCTRL_ASOBRST_RESET;
		udelay(10);
	}
	mb();

	/* enable 3wire#0 buffer */
	AUDIOREG(card, PS3_AUDIO_AO_3WMCTRL) |= PS3_AUDIO_AO_3WMCTRL_ASOEN(0);
	mb();

	/* In 24bit mode,ALSA inserts a zero byte at first byte of per sample */
	AUDIOREG(card, PS3_AUDIO_AO_3WCTRL(0)) =
		((AUDIOREG(card, PS3_AUDIO_AO_3WCTRL(0)) &
		  ~PS3_AUDIO_AO_3WCTRL_ASODF) |
		 PS3_AUDIO_AO_3WCTRL_ASODF_LSB);
	AUDIOREG(card, PS3_AUDIO_AO_SPDCTRL(0)) =
		((AUDIOREG(card, PS3_AUDIO_AO_SPDCTRL(0)) &
		  ~PS3_AUDIO_AO_SPDCTRL_SPODF) |
		 PS3_AUDIO_AO_SPDCTRL_SPODF_LSB);
	mb();
	/* avsetting driver altered AX_IE, caller must reset it if you want */
	return ret;
}

static int snd_ps3_init_avsetting(struct snd_ps3_card_info * card)
{
	int ret;

	card->avs.avs_audio_ch = PS3AV_CMD_AUDIO_NUM_OF_CH_2;
	card->avs.avs_audio_rate = PS3AV_CMD_AUDIO_FS_48K;
	card->avs.avs_audio_width = PS3AV_CMD_AUDIO_WORD_BITS_16;
	card->avs.avs_audio_format = PS3AV_CMD_AUDIO_FORMAT_PCM;
	card->avs.avs_audio_source = PS3AV_CMD_AUDIO_SOURCE_SERIAL;

	ret = snd_ps3_change_avsetting(card);

	snd_ps3_audio_fixup(card);

	/* to start to generate SPDIF signal, fill data */
	snd_ps3_program_dma(card, SND_PS3_DMA_FILLTYPE_SILENT_FIRSTFILL);
	snd_ps3_kick_dma(card);
	return ret;
}

/*
 *  set sampling rate according to the substream
 */
static int snd_ps3_set_avsetting(struct snd_pcm_substream * substream)
{
	struct snd_ps3_card_info * card = snd_pcm_substream_chip(substream);
	struct snd_ps3_avsetting_info avs;

	avs = card->avs;

	pr_debug("%s: called freq=%d width=%d\n", __func__,
		 substream->runtime->rate,
		 snd_pcm_format_width(substream->runtime->format));

	pr_debug("%s: before freq=%d width=%d\n", __func__,
		 card->avs.avs_audio_rate, card->avs.avs_audio_width);

	/* sample rate */
	switch (substream->runtime->rate)
	{
	case 44100:
		avs.avs_audio_rate = PS3AV_CMD_AUDIO_FS_44K;
		break;
	case 48000:
		avs.avs_audio_rate = PS3AV_CMD_AUDIO_FS_48K;
		break;
	case 88200:
		avs.avs_audio_rate = PS3AV_CMD_AUDIO_FS_88K;
		break;
	case 96000:
		avs.avs_audio_rate = PS3AV_CMD_AUDIO_FS_96K;
		break;
	default:
		printk(KERN_ERR "%s: invalid rate %d\n", __func__,
		       substream->runtime->rate);
		return 1;
	}

	/* width */
	switch (snd_pcm_format_width(substream->runtime->format))
	{
	case 16:
		avs.avs_audio_width = PS3AV_CMD_AUDIO_WORD_BITS_16;
		break;
	case 24:
		avs.avs_audio_width = PS3AV_CMD_AUDIO_WORD_BITS_24;
		break;
	default:
		printk(KERN_ERR "%s: invalid width %d\n", __func__,
		       snd_pcm_format_width(substream->runtime->format));
		return 1;
	}

	if ((card->avs.avs_audio_width != avs.avs_audio_width) ||
	    (card->avs.avs_audio_rate != avs.avs_audio_rate)) {
		card->avs = avs;
		snd_ps3_change_avsetting(card);

		pr_debug("%s: after freq=%d width=%d\n", __func__,
			 card->avs.avs_audio_rate, card->avs.avs_audio_width);

		return 0;
	} else
		return 1;
}



static int snd_ps3_map_mmio(void)
{
	the_card.mapped_mmio_vaddr =
		ioremap(the_card.ps3_dev->m_region->bus_addr,
			the_card.ps3_dev->m_region->len);

	if (!the_card.mapped_mmio_vaddr) {
		printk(KERN_ERR "%s: ioremap 0 failed p=%#lx l=%#lx \n",
		       __func__, the_card.ps3_dev->m_region->lpar_addr,
		       the_card.ps3_dev->m_region->len);
		return -ENXIO;
	}

	return 0;
};

static void snd_ps3_unmap_mmio(void)
{
	iounmap(the_card.mapped_mmio_vaddr);
	the_card.mapped_mmio_vaddr = 0;
}

static int snd_ps3_allocate_irq(void)
{
	int ret;
	u64 lpar_addr, lpar_size;
	u64 * mapped;

	/* get irq outlet */
	ret = lv1_gpu_device_map(1, &lpar_addr, &lpar_size);
	if (ret) {
		printk(KERN_ERR "%s: device map 1 failed %d\n", __func__,
		       ret);
		return -ENXIO;
	}

	mapped = ioremap(lpar_addr, lpar_size);

	if (!mapped) {
		printk(KERN_ERR "%s: ioremap 1 failed \n", __func__);
		return -ENXIO;
	}

	the_card.audio_irq_outlet = *mapped;

	iounmap(mapped);
	lv1_gpu_device_unmap(1);

	/* irq */
	ret = ps3_irq_plug_setup(PS3_BINDING_CPU_ANY,
				 the_card.audio_irq_outlet,
				 &the_card.irq_no);
	if (ret) {
		printk("%s:ps3_alloc_irq failed (%d)\n", __func__, ret);
		return ret;
	}

	ret = request_irq(the_card.irq_no, snd_ps3_interrupt, IRQF_DISABLED,
			  SND_PS3_DRIVER_NAME, &the_card);
	if (ret) {
		printk("%s: request_irq failed (%d)\n", __func__, ret);
		goto cleanup_irq;
	}

	return 0;

 cleanup_irq:
	ps3_irq_plug_destroy(the_card.irq_no);
	return ret;
};

static void snd_ps3_free_irq(void)
{
	free_irq(the_card.irq_no, &the_card);
	ps3_irq_plug_destroy(the_card.irq_no);
}

static void snd_ps3_audio_set_base_addr(uint64_t ioaddr_start)
{
	uint64_t val;
	int ret;

	val = (ioaddr_start & (0x0fUL << 32)) >> (32 - 20) |
		(0x03UL << 24) |
		(0x0fUL << 12) |
		(PS3_AUDIO_IOID);

	ret = lv1_gpu_attribute(0x100, 0x007, val, 0, 0);
	if (ret)
		printk(KERN_ERR "%s: gpu_attribute failed %d\n", __func__,
		       ret);
}

static int __init snd_ps3_driver_probe(struct ps3_system_bus_device * dev)
{
	int ret;
	u64 lpar_addr, lpar_size;
	struct ps3_device_id null_id = {0, 0};

	BUG_ON(!firmware_has_feature(FW_FEATURE_PS3_LV1));
	BUG_ON(dev->match_id != PS3_MATCH_ID_SOUND);

	the_card.ps3_dev = dev;

	ret = ps3av_dev_open();
	if (ret)
		return -ENXIO;

	/* setup MMIO */
	ret = lv1_gpu_device_map(2, &lpar_addr, &lpar_size);
	if (ret) {
		printk(KERN_ERR "%s: device map 2 failed %d\n", __func__, ret);
		goto clean_open;
	}
	ps3_mmio_region_init(dev->m_region,
			     &null_id,
			     lpar_addr,
			     lpar_size,
			     PAGE_SHIFT,
			     PS3_IOBUS_IOC0);

	if ((ret = snd_ps3_map_mmio())) {
		goto clean_dev_map;
	}

	/* setup DMA area */
	ps3_dma_region_init(dev->d_region,
			    &null_id,
			    PAGE_SHIFT, /* use system page size */
			    0, /* dma type; not used */
			    NULL,
			    _ALIGN_UP(SND_PS3_DMA_REGION_SIZE, PAGE_SIZE),
			    PS3_IOBUS_IOC0);
	dev->d_region->ioid = PS3_AUDIO_IOID;

	ret = ps3_dma_region_create(dev->d_region);
	if (ret) {
		printk("%s: region_create\n", __func__);
		goto clean_mmio;
	}

	snd_ps3_audio_set_base_addr(dev->d_region->bus_addr);

	/* CONFIG_SND_PS3_DEFAULT_START_DELAY */
	the_card.start_delay = snd_ps3_start_delay;
	//the_card.platform_device = dev->dev;

	/* irq */
	if (snd_ps3_allocate_irq()) {
		ret = -ENXIO;
		goto clean_dma_region;
	}

	/* create card instance */
	the_card.card = snd_card_new(index, id, THIS_MODULE, 0);
	if (!the_card.card) {
		ret = -ENXIO;
		goto clean_irq;
	}

	//snd_card_set_dev(the_card.card, &dev->core);

	strcpy(the_card.card->driver, "snd_ps3");
	strcpy(the_card.card->shortname, "PS3");
	strcpy(the_card.card->longname, "PS3 sound");
	/* create PCM devices instance */
	/* NOTE:this driver works assuming pcm:substream = 1:1 */
	ret = snd_pcm_new(the_card.card,
			  "SPDIF",
			  0, /* instance index, will be stored pcm.device*/
			  1, /* output substream */
			  0, /* input substream */
			  &(the_card.pcm));
	if (ret)
		goto clean_card;

	the_card.pcm->private_data = &the_card;
	strcpy(the_card.pcm->name, "SPDIF");

	/* set pcm ops */
	snd_pcm_set_ops(the_card.pcm, SNDRV_PCM_STREAM_PLAYBACK,
			&snd_ps3_pcm_spdif_ops);

	the_card.pcm->info_flags = SNDRV_PCM_INFO_NONINTERLEAVED;
	/* pre-alloc PCM DMA buffer*/
	ret = snd_pcm_lib_preallocate_pages_for_all(the_card.pcm,
					SNDRV_DMA_TYPE_DEV,
					&dev->core,
					SND_PS3_PCM_PREALLOC_SIZE,
					SND_PS3_PCM_PREALLOC_SIZE);
	if (ret < 0) {
		printk(KERN_ERR "%s: prealloc failed\n", __func__);
		goto clean_card;
	}

	/*
	 * allocate null buffer
	 * its size should be lager than PS3_AUDIO_FIFO_STAGE_SIZE * 2
	 * PAGE_SIZE is enogh
	 */
	if (!(the_card.null_buffer_start_vaddr =
	      dma_alloc_coherent(&the_card.ps3_dev->core,
				 PAGE_SIZE,
				 &the_card.null_buffer_start_dma_addr,
				 GFP_KERNEL))) {
		printk(KERN_ERR "%s: nullbuffer alloc failed\n", __func__);
		goto clean_preallocate;
	}
	pr_debug("%s: null vaddr=%p dma=%#lx\n", __func__,
		 the_card.null_buffer_start_vaddr,
		 the_card.null_buffer_start_dma_addr);
	/* set default sample rate/word width */
	snd_ps3_init_avsetting(&the_card);

	/* add volume control */
	the_card.vol_control = snd_ctl_new1(&snd_ps3_vol_control, &the_card);
	if ((ret = snd_ctl_add(the_card.card, the_card.vol_control)) < 0) {
		goto clean_dma_map;
	}
	/* register the card */
	ret = snd_card_register(the_card.card);
	if (ret < 0)
		goto clean_ctl_add;

	//platform_set_drvdata(device, &the_card);

	printk("%s started. start_delay=%dms\n",
	       the_card.card->longname, the_card.start_delay);
	return 0;

clean_ctl_add:
	/* no need call to snd_control_free_one() here*/
	snd_ctl_remove(the_card.card, the_card.vol_control);
clean_dma_map:
	dma_free_coherent(&the_card.ps3_dev->core,
			  PAGE_SIZE,
			  the_card.null_buffer_start_vaddr,
			  the_card.null_buffer_start_dma_addr);
clean_preallocate:
	snd_pcm_lib_preallocate_free_for_all(the_card.pcm);
clean_card:
	snd_card_free(the_card.card);
clean_irq:
	snd_ps3_free_irq();
clean_dma_region:
	ps3_dma_region_free(dev->d_region);
clean_mmio:
	snd_ps3_unmap_mmio();
clean_dev_map:
	lv1_gpu_device_unmap(2);
clean_open:
	ps3av_dev_close();
	/*
	 * there is no destructor function to pcm.
	 * midlayer automatically releases if the card removed
	 */
	return ret;
}; /* snd_ps3_probe */

/* called when module removal */
static int snd_ps3_driver_remove(struct ps3_system_bus_device * device)
{
	int ret;

	if (!firmware_has_feature(FW_FEATURE_PS3_LV1))
		return -ENXIO;

	if (device->match_id != PS3_MATCH_ID_SOUND)
		return -ENXIO;

	ret = snd_ctl_remove(the_card.card, the_card.vol_control);
	if (ret)
		printk("%s: ctl remove=%d\n", __func__,ret);
	ret = snd_pcm_lib_preallocate_free_for_all(the_card.pcm);
	if (ret)
		printk("%s: ctl freepage=%d\n", __func__,ret);
	ret = snd_card_free(the_card.card);
	if (ret)
		printk("%s: ctl freecard=%d\n", __func__,ret);

	dma_free_coherent(&the_card.ps3_dev->core,
			  PAGE_SIZE,
			  the_card.null_buffer_start_vaddr,
			  the_card.null_buffer_start_dma_addr);

	ps3_dma_region_free(the_card.ps3_dev->d_region);

	snd_ps3_free_irq();
	snd_ps3_unmap_mmio();

	lv1_gpu_device_unmap(2);
	ps3av_dev_close();
	return 0;
} /* snd_ps3_remove */

static struct ps3_system_bus_driver snd_ps3_bus_driver_info = {
	.match_id = PS3_MATCH_ID_SOUND,
	.probe = snd_ps3_driver_probe,
	.remove = snd_ps3_driver_remove,
	.shutdown = snd_ps3_driver_remove,
	.core = {
		.name = SND_PS3_DRIVER_NAME,
	},
};


/*
 * Interrupt handler
 */
static irqreturn_t snd_ps3_interrupt(int irq, void * dev_id)
{

	uint32_t port_intr;
	int underflow_occured = 0;
	struct snd_ps3_card_info * card = dev_id;

	if (!card->running) {
		AUDIOREG(card, PS3_AUDIO_AX_IS) |= 0;
		AUDIOREG(card, PS3_AUDIO_INTR_0) |= 0;
		return IRQ_HANDLED;
	}

	port_intr = AUDIOREG(card, PS3_AUDIO_AX_IS);
	/*
	 *serial buffer empty detected (every 4 times),
	 *program next dma and kick it
	 */
	if (port_intr & PS3_AUDIO_AX_IE_ASOBEIE(0)) {
		AUDIOREG(card, PS3_AUDIO_AX_IS) = PS3_AUDIO_AX_IE_ASOBEIE(0);
		if (port_intr & PS3_AUDIO_AX_IE_ASOBUIE(0)) {
			AUDIOREG(card, PS3_AUDIO_AX_IS) = port_intr;
			underflow_occured = 1;
		}
		if (card->silent) {
			/* we are still in silent time */
			snd_ps3_program_dma(card,
				(underflow_occured) ?
				SND_PS3_DMA_FILLTYPE_SILENT_FIRSTFILL :
				SND_PS3_DMA_FILLTYPE_SILENT_RUNNING);
			snd_ps3_kick_dma(card);
			card->silent --;
		} else {
			snd_ps3_program_dma(card,
				(underflow_occured) ?
				SND_PS3_DMA_FILLTYPE_FIRSTFILL :
				SND_PS3_DMA_FILLTYPE_RUNNING);
			snd_ps3_kick_dma(card);
			snd_pcm_period_elapsed(card->substream);
		}
	} else if (port_intr & PS3_AUDIO_AX_IE_ASOBUIE(0)) {
		AUDIOREG(card, PS3_AUDIO_AX_IS) = PS3_AUDIO_AX_IE_ASOBUIE(0);
                /*
		 * serial out underflow, but buffer empty not detected.
		 * in this case, fill fifo with 0 to recover.  After
		 * filling dummy data, serial automatically start to
		 * consume them and then will generate normal buffer
		 * empty interrupts.
		 * If both buffer underflow and buffer empty are occured,
		 * it is better to do nomal data transfer than empty one
		 */
		snd_ps3_program_dma(card,
				    SND_PS3_DMA_FILLTYPE_SILENT_FIRSTFILL);
		snd_ps3_kick_dma(card);
		snd_ps3_program_dma(card,
				    SND_PS3_DMA_FILLTYPE_SILENT_FIRSTFILL);
		snd_ps3_kick_dma(card);
	}
	/* clear interrupt cause */
	return IRQ_HANDLED;
};

#ifdef _SND_PS3_DEV_ATTR
/*
 * sysfs
 */
static ssize_t snd_ps3_get_start_delay(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct platform_device * plat_dev =
		container_of(dev, struct platform_device, dev);
	struct snd_ps3_card_info * card = platform_get_drvdata(plat_dev);
	ssize_t ret;

	read_lock(&card->start_delay_lock);
	ret = sprintf(buf, "%u\n", card->start_delay);
	read_unlock(&card->start_delay_lock);
	return ret;
}

static ssize_t snd_ps3_set_start_delay(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf,
				       size_t count)
{
	unsigned int start_delay;
	struct platform_device * plat_dev =
		container_of(dev, struct platform_device, dev);
	struct snd_ps3_card_info * card = platform_get_drvdata(plat_dev);

	if (sscanf(buf, "%u", &start_delay) > 0) {
		write_lock(&card->start_delay_lock);
		card->start_delay = start_delay;
		write_unlock(&card->start_delay_lock);
		return strlen(buf);
	}
	return -EINVAL;
}

#endif

static int snd_ps3_info_vol_control(struct snd_kcontrol * kcontrol,
				    struct snd_ctl_elem_info * uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = SND_PS3_CH_MAX; /* stereo */
	uinfo->value.integer.min = SND_PS3_MIN_VOL;
	uinfo->value.integer.max = SND_PS3_MAX_VOL;
	return 0;
};

static int snd_ps3_get_vol_control(struct snd_kcontrol * kcontrol,
				   struct snd_ctl_elem_value * ucontrol)
{
	struct snd_ps3_card_info * card = snd_kcontrol_chip(kcontrol);
	int i;
	for (i = 0; i < SND_PS3_CH_MAX; i++)
		ucontrol->value.integer.value[i] =
			SND_PS3_MAX_VOL - card->attenuater[i];
	return 0;
};

static int snd_ps3_put_vol_control(struct snd_kcontrol * kcontrol,
				   struct snd_ctl_elem_value * ucontrol)
{
	struct snd_ps3_card_info * card = snd_kcontrol_chip(kcontrol);
	int i;
	int changed = 0;

	for (i = 0; i < SND_PS3_CH_MAX; i++)
		if ((ucontrol->value.integer.value[i] < SND_PS3_MIN_VOL) ||
		    (SND_PS3_MAX_VOL < ucontrol->value.integer.value[i]))
			return -EINVAL;

	for (i = 0; i < SND_PS3_CH_MAX; i++)
		if ((SND_PS3_MAX_VOL - card->attenuater[i]) !=
		    ucontrol->value.integer.value[i]) {
			card->attenuater[i] = SND_PS3_MAX_VOL -
				ucontrol->value.integer.value[i];
			changed = 1;
		}
	return changed;
};

typedef struct
{
	unsigned char numerator;
	unsigned char denominator;
} attenuater_divisor;

static const attenuater_divisor
attenuater_divisor_array[SND_PS3_MAX_VOL - SND_PS3_MIN_VOL + 1] =
{
	[ 0] = {   1,   1}, /* 0db; not used */
	[ 1] = { 177, 250}, /*  -1.5 db 0.708 */
	[ 2] = {   1,   2}, /*  -3.0 db 0.501 */
	[ 3] = {  71, 200}, /*  -4.5 db 0.355 */
	[ 4] = {   1,   4}, /*  -6.0 db 0.251 */
	[ 5] = {  45, 250}, /*  -7.5 db 0.178 */
	[ 6] = {   1,   8}, /*  -9.0 db 0.126 */
	[ 7] = {  22, 250}, /* -10.5 db 89.1m */
	[ 8] = {   1,  16}, /* -12.0 db 63.1m */
	[ 9] = {  11, 250}, /* -13.5 db 44.7m */
	[10] = {   1,  32}, /* -15.0 db 31.6m */
	[11] = {   5, 250}, /* -16.5 db 22.4m */
	[12] = {   1,  64}, /* -18.0 db 15.8m */
	[13] = {   2, 178}, /* -19.5 db 11.2m */
	[14] = {   1, 128}, /* -21.0 db  7.94m*/
	[15] = {   0,   1} /* mute; not used */
};

/*
 * software volume control
 */
static void snd_ps3_do_attenuate_16(int attenuate, signed short int * start,
				    int samples)
{
	int i;

	if (unlikely(attenuate == SND_PS3_MIN_ATT)) {
		return;
	} else if (attenuate == SND_PS3_MAX_ATT) {
		memset(start, 0, sizeof(short int) * samples);
		return;
	} else {
		for (i = 0; i < samples; i++) {
			start[i] = start[i] *
				attenuater_divisor_array[attenuate].numerator /
				attenuater_divisor_array[attenuate].denominator;
		}
	}
}

static void snd_ps3_do_attenuate_24(int attenuate, uint32_t * start,
				    int samples)
{
	int i;
	int32_t temp, temp2;

	if (unlikely(attenuate == SND_PS3_MIN_ATT)) {
		return;
	} else if (attenuate == SND_PS3_MAX_ATT) {
		memset(start, 0, sizeof(uint32_t) * samples);
		return;
	} else {
		for (i = 0; i < samples; i++) {
			/* 24bit -> 32bit */
			temp = (int32_t)(start[i] << 8);
			/* shift alithmetic */
			temp2 = temp >> 8;
			/*
			 * Since upper 8 bits will be disposed of by the
			 * hardware, leave it untouched.
			 */
			start[i] = temp2 *
				attenuater_divisor_array[attenuate].numerator /
				attenuater_divisor_array[attenuate].denominator;
		}
	}
}

static int snd_ps3_soft_attenuate(struct snd_ps3_card_info *card,
				  void * start_l, void * start_r, int bytes)
{

	switch(snd_pcm_format_width(card->substream->runtime->format)) {
	case 16:
		snd_ps3_do_attenuate_16(card->attenuater[SND_PS3_CH_L],
					start_l, bytes / 2);
		snd_ps3_do_attenuate_16(card->attenuater[SND_PS3_CH_R],
					start_r, bytes / 2);
		break;
	case 24:
		snd_ps3_do_attenuate_24(card->attenuater[SND_PS3_CH_L],
					start_l, bytes / 4);
		snd_ps3_do_attenuate_24(card->attenuater[SND_PS3_CH_R],
					start_r, bytes / 4);
		break;
	default:
		printk(KERN_ERR "%s: invalid width %d\n", __func__,
		       snd_pcm_format_width(card->substream->runtime->format));
		return -EINVAL;
	}
	return 0;
}

/*
 * module/subsystem initialize/terminate
 */
static int __init snd_ps3_init(void)
{
	int ret;

	if (!firmware_has_feature(FW_FEATURE_PS3_LV1))
		return -ENXIO;

	memset(&the_card, 0, sizeof(the_card));
	rwlock_init(&the_card.dma_lock);
	rwlock_init(&the_card.start_delay_lock);

	/* register systembus DRIVER, this calls our probe() func */
	ret = ps3_system_bus_driver_register(&snd_ps3_bus_driver_info,
					     PS3_IOBUS_IOC0);

	return ret;
}

static void __exit snd_ps3_exit(void)
{
	pr_debug("%s:start\n", __func__);
	ps3_system_bus_driver_unregister(&snd_ps3_bus_driver_info);
	pr_debug("%s:end\n", __func__);
}
