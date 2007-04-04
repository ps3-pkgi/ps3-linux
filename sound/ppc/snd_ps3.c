/*
 * Audio support for PS3
 * Copyright (C) 2006 Sony Computer Entertainment Inc.
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

#include <sound/driver.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/asound.h>
#include <sound/memalloc.h>
#include <sound/pcm_params.h>
#include <sound/control.h>
#include <linux/dmapool.h>
#include <linux/dma-mapping.h>
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

static DEVICE_ATTR(start_delay,
		   S_IRUGO | S_IWUSR,
		   snd_ps3_get_start_delay,
		   snd_ps3_set_start_delay);

/* system memory info */
extern unsigned long ps3_rm_limit, ps3_2nd_mem_base;
extern unsigned long ps3_2nd_mem_size, ps3_mem_total;

/*
 * global
 */
struct snd_ps3_card_info the_card;

static struct ioif_map_info * ioif_map_info_array;
static int ioif_map_info_count;
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
#define AUDIOREGPTR(chip, name) (volatile uint32_t *)(chip->mapped_vaddr + name)

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

/*
 * PCM operators
 */
static int snd_ps3_pcm_open(struct snd_pcm_substream * substream)
{
	struct snd_pcm_runtime * runtime = substream->runtime;
	struct snd_ps3_card_info * card =
		(struct snd_ps3_card_info *) snd_pcm_substream_chip(substream);
	int pcm_index;

	_SF;
	pcm_index = substream->pcm->device;
	/* to retrieve substream/runtime in interrupt handler */
	card->substream = substream;

	runtime->hw = snd_ps3_pcm_hw;

	/* mute off */
	snd_ps3_mute(substream, 0); // this function sleep

	snd_pcm_hw_constraint_step(runtime, 0, SNDRV_PCM_HW_PARAM_BUFFER_BYTES,
				   PS3_AUDIO_FIFO_STAGE_SIZE * 4 * 2);
	_EF;
	return 0;
};

static int snd_ps3_pcm_hw_params(struct snd_pcm_substream * substream,
				 struct snd_pcm_hw_params * hw_params)
{
	size_t size;

	_SF;
	/* alloc transport buffer */
	size = params_buffer_bytes(hw_params);
	snd_pcm_lib_malloc_pages(substream, size);
	_EF;
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
#if defined(_SND_PS3_DEBUG)
	printk(KERN_ERR "%s: time=%d rate=%d bytes=%ld, frames=%d, ret=%d\n",
	       __FUNCTION__,
	       delay_ms,
	       rate,
	       snd_pcm_format_size(substream->runtime->format, rate),
	       rate * delay_ms / 1000,
	       ret);
#endif
	return ret;
};

static int snd_ps3_pcm_prepare(struct snd_pcm_substream * substream)
{
	struct snd_pcm_runtime * runtime = substream->runtime;
	struct snd_ps3_card_info * card =
		(struct snd_ps3_card_info *) snd_pcm_substream_chip(substream);
	unsigned long irqsave;
	_SF;

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
		card->dma_last_transfer_vaddr[SND_PS3_CH_L] =
			card->dma_next_transfer_vaddr[SND_PS3_CH_L] =
			card->dma_start_vaddr[SND_PS3_CH_L] = runtime->dma_area;

		card->dma_buffer_size = runtime->dma_bytes;

		card->dma_last_transfer_vaddr[SND_PS3_CH_R] =
			card->dma_next_transfer_vaddr[SND_PS3_CH_R] =
			card->dma_start_vaddr[SND_PS3_CH_R] =
			runtime->dma_area + (runtime->dma_bytes / 2);
	}
	write_unlock_irqrestore(&card->dma_lock, irqsave);

	mb();

	_EF;
	return 0;
};

/*
 * atomic
 */
static int snd_ps3_pcm_trigger(struct snd_pcm_substream * substream,
			       int cmd)
{
	struct snd_ps3_card_info * card =
		(struct snd_ps3_card_info *) snd_pcm_substream_chip(substream);
	int ret = 0;
	unsigned long irqsave;

	_SF;
	switch (cmd)
	{
	case SNDRV_PCM_TRIGGER_START:
		/* clear outstanding interrupts  */
		AUDIOREG(card, PS3_AUDIO_AX_IS) = -1;

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

	_EF;
	return ret;
};

/*
 * report current pointer
 */
static snd_pcm_uframes_t snd_ps3_pcm_pointer(
	struct snd_pcm_substream * substream)
{
	struct snd_ps3_card_info * card =
		(struct snd_ps3_card_info *) snd_pcm_substream_chip(substream);
	unsigned long irqsave;
	size_t bytes;
	snd_pcm_uframes_t ret;

	_SF;

 	read_lock_irqsave(&card->dma_lock, irqsave);
	{
		bytes = (size_t)(card->dma_last_transfer_vaddr[SND_PS3_CH_L] -
				 card->dma_start_vaddr[SND_PS3_CH_L]);
	}
 	read_unlock_irqrestore(&card->dma_lock, irqsave);

	ret = bytes_to_frames(substream->runtime, bytes * 2);
	_EF;
	return ret;
};

static int snd_ps3_pcm_hw_free(struct snd_pcm_substream * substream)
{
	int ret;
	_SF;
	ret = snd_pcm_lib_free_pages(substream);
	_EF;
	return ret;
};

static int snd_ps3_pcm_close(struct snd_pcm_substream * substream)
{
	_SF;
	/* mute on */
	snd_ps3_mute(substream, 1); // this function sleep
	_EF;
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
	_SF;

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
	_EF;
	return ret;
}

static int snd_ps3_init_avsetting(struct snd_ps3_card_info * card)
{
	int ret;

	_SF;
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
	_EF;
	return ret;
}

/*
 *  set sampling rate according to the substream
 */
static int snd_ps3_set_avsetting(struct snd_pcm_substream * substream)
{
	struct snd_ps3_card_info * card =
		(struct snd_ps3_card_info *) snd_pcm_substream_chip(substream);
	struct snd_ps3_avsetting_info avs;

	avs = card->avs;

#if defined(_SND_PS3_DEBUG)
	printk(KERN_ERR "%s: called freq=%d width=%d\n", __FUNCTION__,
	       substream->runtime->rate,
	       snd_pcm_format_width(substream->runtime->format));

	printk(KERN_ERR "%s: before freq=%d width=%d\n", __FUNCTION__,
	       card->avs.avs_audio_rate, card->avs.avs_audio_width);

#endif
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
		printk(KERN_ERR "%s: invalid rate %d\n", __FUNCTION__,
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
		printk(KERN_ERR "%s: invalid width %d\n", __FUNCTION__,
		       snd_pcm_format_width(substream->runtime->format));
		return 1;
	}

	if ((card->avs.avs_audio_width != avs.avs_audio_width) ||
	    (card->avs.avs_audio_rate != avs.avs_audio_rate)) {
		card->avs = avs;
		snd_ps3_change_avsetting(card);
#if defined(_SND_PS3_DEBUG)
		printk(KERN_ERR "%s: after freq=%d width=%d\n", __FUNCTION__,
		       card->avs.avs_audio_rate, card->avs.avs_audio_width);
#endif
		return 0;
	} else
		return 1;
}

/*
 * audio mute on/off
 * mute_on : 0 output enabled
 *           1 mute
 */
static int snd_ps3_mute(struct snd_pcm_substream * substream, int mute_on)
{
	(void) substream;

	return ps3av_audio_mute(mute_on);
}

static int snd_ps3_kick_dma(struct snd_ps3_card_info * card)
{

	/* kick dma */
	AUDIOREG(card, PS3_AUDIO_KICK(0)) |= PS3_AUDIO_KICK_REQUEST;
	mb();

	return 0;
}

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
			       __FUNCTION__, dma_ch);
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
	_SF;
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
	AUDIOREG(card, PS3_AUDIO_INTR_0) = -1;
	AUDIOREG(card, PS3_AUDIO_AX_IS) = -1;

	/*
	 *revert CLEAR bit since it will not reset automatically after DMA stop
	 */
	if (stop_forced) {
		AUDIOREG(card, PS3_AUDIO_CONFIG) &= ~PS3_AUDIO_CONFIG_CLEAR;
	}
	mb();
	_EF;
}

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
			if (silent) {
				dma_addr =
				p_to_dma(__pa(card->null_buffer_start_vaddr));
			}
			else {
				dma_addr =
				p_to_dma(__pa(card->dma_next_transfer_vaddr[ch]));
			}

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

/****************************************************************************
 *
 * real initializer. called once
 *
 */
static int __init snd_ps3_driver_probe(struct platform_device * device)
{
	int ret;
	_SF;

	memset(&the_card, 0, sizeof(the_card));
	rwlock_init(&the_card.dma_lock);
	rwlock_init(&the_card.start_delay_lock);

	/* CONFIG_SND_PS3_DEFAULT_START_DELAY */
	the_card.start_delay = snd_ps3_start_delay;
	the_card.platform_device = device;

	/* map audio register, etc */
	if ((ret = snd_ps3_init_audio())) {
		return -ENOMEM;
	}

	/* create card instance */
	the_card.card = snd_card_new(index, id, THIS_MODULE, 0);
	if (!the_card.card) {
		goto error0;
	}
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
		goto error1;

	the_card.pcm->private_data = &the_card;
	strcpy(the_card.pcm->name, "SPDIF");

	/* set pcm ops */
	snd_pcm_set_ops(the_card.pcm, SNDRV_PCM_STREAM_PLAYBACK,
			&snd_ps3_pcm_spdif_ops);

	the_card.pcm->info_flags = SNDRV_PCM_INFO_NONINTERLEAVED;
	/* pre-alloc buffer */
	ret = snd_pcm_lib_preallocate_pages_for_all(the_card.pcm,
					SNDRV_DMA_TYPE_CONTINUOUS,
					snd_dma_continuous_data(GFP_KERNEL),
					PS3_AUDIO_DMAC_BLOCK_SIZE *
					PS3_AUDIO_DMAC_MAX_BLOCKS * 4,
					PS3_AUDIO_DMAC_BLOCK_SIZE *
					PS3_AUDIO_DMAC_MAX_BLOCKS * 4);

	if (ret < 0) {
		printk(KERN_ERR "%s: prealloc failed\n", __FUNCTION__);
		goto error1;
	}

	/*
	 * allocate null buffer
	 * its size should be lager than PS3_AUDIO_FIFO_STAGE_SIZE * 2
	 */
	if (!(the_card.null_buffer_start_vaddr = get_zeroed_page(GFP_KERNEL))) {
		printk(KERN_ERR "%s: nullbuffer alloc failed\n", __FUNCTION__);
		goto error2;
	}
	/* set default sample rate/word width */
	snd_ps3_init_avsetting(&the_card);

	/* add volume control */
	the_card.vol_control = snd_ctl_new1(&snd_ps3_vol_control, &the_card);
	if ((ret = snd_ctl_add(the_card.card, the_card.vol_control)) < 0) {
		goto error3;
	}
	/* register the card */
	ret = snd_card_register(the_card.card);
	if (ret < 0)
		goto error4;

	platform_set_drvdata(device, &the_card);

	printk("%s started. start_delay=%dms\n",
	       the_card.card->longname, the_card.start_delay);
	_EF;
	return 0;

 error4:
	/* no need call to snd_control_free_one() here*/
	snd_ctl_remove(the_card.card, the_card.vol_control);
 error3:
	free_page(the_card.null_buffer_start_vaddr);
 error2:
	snd_pcm_lib_preallocate_free_for_all(the_card.pcm);
 error1:
	snd_card_free(the_card.card);
 error0:
	snd_ps3_free_audio();
	/*
	 * there is no destructor function to pcm.
	 * midlayer automatically releases if the card removed
	 */
	_EF1("error");
	return ret;
}; /* snd_ps3_probe */

/* called when system shutdown */
static void snd_ps3_driver_shutdown(struct platform_device * device)
{
	_SF;
	snd_ps3_driver_remove(device);
	_EF;
}

/* called when module removal */
static int snd_ps3_driver_remove(struct platform_device * device)
{
	_SF;

	platform_set_drvdata(device, NULL);
	snd_ctl_remove(the_card.card, the_card.vol_control);
	snd_pcm_lib_preallocate_free_for_all(the_card.pcm);
	snd_card_free(the_card.card);
	free_page(the_card.null_buffer_start_vaddr);
	snd_ps3_free_audio();

	_EF;
	return 0;
} /* snd_ps3_remove */

static struct platform_driver snd_ps3_platform_driver =
{
	.probe = snd_ps3_driver_probe,
	.remove = snd_ps3_driver_remove,
	.shutdown = snd_ps3_driver_shutdown,
	.driver = {
		.name = SND_PS3_DRIVER_NAME
	}
};

static struct platform_device * the_platform_device;

/*
 * module initialize/terminate
 */
static int __init snd_ps3_init(void)
{
	int ret;

	ret = ps3av_dev_open();

	if (ret) {
		printk(KERN_ERR "%s: open failed %d\n", __FUNCTION__, ret);
		return ret;
	}

	ret = platform_driver_register(&snd_ps3_platform_driver);
	if (ret < 0)
		return ret;

	the_platform_device =
		platform_device_register_simple(SND_PS3_DRIVER_NAME,
						0, NULL, 0);
	if (IS_ERR(the_platform_device)) {
		ret = PTR_ERR(the_platform_device);
		goto error0;
	}

	ret = device_create_file(&(the_platform_device->dev),
				 &dev_attr_start_delay);

	return ret;
 error0:
	platform_driver_unregister(&snd_ps3_platform_driver);
	return ret;
}

static void __exit snd_ps3_exit(void)
{
	device_remove_file(&(the_platform_device->dev), &dev_attr_start_delay);
	platform_device_unregister(the_platform_device);
	platform_driver_unregister(&snd_ps3_platform_driver);
}


/*
 * create iosegment and iopte for DMA area
 * map audio audio registers
 * allocate irq
 */

static int snd_ps3_init_audio(void)
{
	int ret, map;

	if ((ret = snd_ps3_create_iopt()))
		return ret;

 	/* map registers and irq info */
	map = 1;
	ret = lv1_gpu_device_map(map, &the_card.audio_lpar_addr,
				 &the_card.audio_lpar_size);
	if (ret) {
		printk(KERN_ERR "%s: device map 1 failed %d\n", __FUNCTION__,
		       ret);
		ret = -ENXIO;
		goto cleanup;
	}

	the_card.mapped_vaddr = ioremap(the_card.audio_lpar_addr,
					the_card.audio_lpar_size);

	if (!the_card.mapped_vaddr) {
		printk(KERN_ERR "%s: ioremap 1 failed \n", __FUNCTION__);
		ret = -ENXIO;
		goto cleanup_1;
	}


	the_card.audio_irq_outlet = *(uint64_t*)the_card.mapped_vaddr;

	/* no more needed */
	iounmap(the_card.mapped_vaddr);
	lv1_gpu_device_unmap(map);

	map = 2;
	ret = lv1_gpu_device_map(map, &the_card.audio_lpar_addr,
				 &the_card.audio_lpar_size);
	if (ret) {
		printk(KERN_ERR "%s: device map 2 failed %d\n", __FUNCTION__,
		       ret);
		ret = -ENXIO;
		goto cleanup;
	}

	the_card.mapped_vaddr = ioremap(the_card.audio_lpar_addr,
					the_card.audio_lpar_size);

	if (!the_card.mapped_vaddr) {
		printk(KERN_ERR "%s: ioremap 0 failed \n", __FUNCTION__);
		ret = -ENXIO;
		goto cleanup_1;
	}

	/* irq */
	ret = ps3_irq_plug_setup(PS3_BINDING_CPU_ANY, the_card.audio_irq_outlet,
				 &the_card.irq_no);
	if (ret) {
		printk("%s:%u: ps3_alloc_irq failed (%d)\n", __FUNCTION__,
		       __LINE__, ret);
		goto cleanup_2;
	}

	ret = request_irq(the_card.irq_no, snd_ps3_interrupt, IRQF_DISABLED,
			  SND_PS3_DRIVER_NAME, &the_card);
	if (ret) {
		printk("%s:%u: request_irq failed (%d)\n", __FUNCTION__,
		       __LINE__, ret);
		goto cleanup_3;
	}

	/*
	 * OK, PPU side setup done,
	 * tell io address for DMA to audio controller
	 */
	snd_ps3_audio_set_base_addr(ioif_map_info_array[0].ioif_addr);

	_EF;

	return 0;
 cleanup_3:
	ps3_irq_plug_destroy(the_card.irq_no);
 cleanup_2:
	iounmap(the_card.mapped_vaddr);

 cleanup_1:
	lv1_gpu_device_unmap(map);

 cleanup:

	return ret;
};


static void snd_ps3_free_audio(void)
{
	int ret;

	_SF;
	/* irq */
	free_irq(the_card.irq_no, &the_card);
	ps3_irq_plug_destroy(the_card.irq_no);

	iounmap(the_card.mapped_vaddr);

	/* unmap registers */
	ret = lv1_gpu_device_unmap(2);
	if (ret)
		printk(KERN_ERR "%s: device unmap failed %d\n", __FUNCTION__,
		       ret);

	ps3av_dev_close();

	/* iopte */
	snd_ps3_destruct_iopt();
	_EF;

};



/*
 * request iopte for device dma
 */
static void snd_ps3_audio_set_base_addr(uint64_t ioaddr_start)
{
	uint64_t val;
	int ret;

	_SF;

	val = (ioaddr_start & (0x0fUL << 32)) >> (32 - 20) |
		(0x03UL << 24) |
		(0x0fUL << 12) |
		(1);

	ret = lv1_gpu_attribute(0x100, 0x007, val, 0, 0);
	if (ret)
		printk(KERN_ERR "%s: gpu_attribute failed %d\n", __FUNCTION__,
		       ret);
	_EF;
}

#define PS3_AUDIO_IOID       (1UL)
#define IO_PAGESIZE_4K_SHIFT   (12)
#define IO_PAGESIZE_64K_SHIFT  (16)
#define IO_PAGESIZE_1M_SHIFT   (20)
#define IO_PAGESIZE_16M_SHIFT  (24)
#define IO_PAGESIZE_SHIFT      IO_PAGESIZE_16M_SHIFT
#define IO_PAGESIZE            (1UL << IO_PAGESIZE_SHIFT)
#define IO_SEGMENTSIZE_SHIFT   (28)
#define IO_SEGMENTSIZE         (1UL << IO_SEGMENTSIZE_SHIFT)
#define IOPTE_READONLY         (1UL << 62)
#define IOPTE_READWRITE        (3UL << 62)
#define IOPTE_INVALID          (0UL << 62)

#define IOPTE_COHERENT         (1UL << 61)
#define IOPTE_STRICT_ORDER     (3UL << 59)
#define IOPTE_HINT             (1UL << 11)
/*
 * convert physical addr to ioif bus addr.
 * Since we mapped physical addr 0 as seg[0].ioif_addr
 * no need to convert to lpar address here.
 * NOTE: no boudary check performed
 */
static uint64_t p_to_dma(uint64_t paddr)
{
	int seg;
#if defined(CONFIG_PS3_USE_LPAR_ADDR)
	if (ps3_rm_limit <= paddr)
		paddr = paddr - ps3_2nd_mem_base + ps3_rm_limit;
#endif
	seg = paddr >> IO_SEGMENTSIZE_SHIFT;


	return ioif_map_info_array[seg].ioif_addr + (paddr & ~IO_SEGMENTSIZE);
};

static uint64_t inc_paddr(uint64_t curpos, uint64_t offset)
{
	uint64_t ret = curpos + offset;

#if defined(CONFIG_PS3_USE_LPAR_ADDR)
	if ((ps3_rm_limit <= ret) && (ret < ps3_2nd_mem_base))
		ret = ps3_2nd_mem_base;
#endif
	return ret;
}
/*
 * create io segments for DMA and iopte
 * and also AUDIO ioif setup
 * Note: segments cover whole mememory area Linux uses.
 *       See ps3_get_memsize for LPAR memory layout.
 * we assume the followings:
 *   o total system memory size is multiple of 16MB
 *   o hvc allocates all io address under 4G
 */
static int snd_ps3_create_iopt(void)
{
	int ret, pages_remain, current_segment, current_page;
	uint64_t current_paddr;
	_SF;
	/*
	 * since we allocated memory from hvc as pagesize 16MB,
	 * total allocated size should be multiple of 16MB
	 */
	if (ps3_mem_total % IO_PAGESIZE) {
		printk(KERN_ERR "%s: pagesize differ %lx!!\n", __FUNCTION__,
		       ps3_mem_total);
		panic("!");
	}
	/*
	 * calc how many segment needed
	 * assumed 1 or 2 for current ps3 memory size
	 */
	ioif_map_info_count = (ps3_mem_total >> IO_SEGMENTSIZE_SHIFT) + 1;

	if (!(ioif_map_info_array =
	      kzalloc(sizeof(struct ioif_map_info) * ioif_map_info_count,
		      GFP_KERNEL))) {
		printk(KERN_ERR "%s: no memory\n", __FUNCTION__);
		return -ENOMEM;
	}

	pages_remain = ps3_mem_total >> IO_PAGESIZE_SHIFT;
	/* physical address start from 0 */
	current_paddr = 0;
	for (current_segment = 0;
	     current_segment < ioif_map_info_count;
	     current_segment++) {

		ioif_map_info_array[current_segment].start_paddr =
			current_paddr;
		ioif_map_info_array[current_segment].start_lpar_addr =
			ps3_mm_phys_to_lpar(current_paddr);
		ioif_map_info_array[current_segment].area_size = IO_SEGMENTSIZE;

		ret = lv1_allocate_io_segment(0, /* io space */
			IO_SEGMENTSIZE, /* segment size */
			IO_PAGESIZE_SHIFT, /* io page size */
			&(ioif_map_info_array[current_segment].ioif_addr));

		if (ret) {
			printk(KERN_ERR "%s: alloc_io_seg %d failed %d\n",
			       __FUNCTION__, current_segment, ret);
			goto cleanup0;
		}

		if (ioif_map_info_array[current_segment].ioif_addr >> 32) {
			printk(KERN_CRIT "%s: io addr is alloc above 4G! %lx\n",
			       __FUNCTION__,
			       ioif_map_info_array[current_segment].ioif_addr);
			panic("!");
		}
		/* create iopte for this segment */
		for (current_page = 0;
		     current_page < (IO_SEGMENTSIZE / IO_PAGESIZE) &&
			     pages_remain;
		     current_page ++, current_paddr =
			     inc_paddr(current_paddr, IO_PAGESIZE)) {
			ret = lv1_put_iopte(0, /* io address space id */
				ioif_map_info_array[current_segment].ioif_addr +
				current_page * IO_PAGESIZE, /* ioif addr */
				ps3_mm_phys_to_lpar(current_paddr),
				PS3_AUDIO_IOID,
				IOPTE_READONLY | IOPTE_COHERENT |
					    IOPTE_STRICT_ORDER);
			if (ret) {
				printk(KERN_ERR "%s: put_iopte failed (%d) " \
				       "seg=%d paddr=%#lx lpar=%#lx page=%d\n",
				       __FUNCTION__, ret, current_segment,
				       current_paddr,
				       ps3_mm_phys_to_lpar(current_paddr),
				       current_page);
				printk(KERN_ERR "%s: rm_limit=%#lx " \
				       "2nd_base=%#lx 2nd_size=%#lx "\
				       "total=%#lx\n",
				       __FUNCTION__,
				       ps3_rm_limit, ps3_2nd_mem_base,
				       ps3_2nd_mem_size, ps3_mem_total);
				current_segment ++;
				goto cleanup0;
			}
			pages_remain --;
		}
	}

	if (0 < pages_remain) {
		panic("unmaped page remain\n");
	}

	_EF;
	return 0;

 cleanup0:
	snd_ps3_destruct_iopt_helper();
	return -ENOMEM;
}

/*
 * invalidate all iopte and free io segment
 */
static void snd_ps3_destruct_iopt_helper(void)
{
	int ret, pages_remain, current_segment, current_page;
	uint64_t current_paddr;

	_SF;

	pages_remain = ps3_mem_total >> IO_PAGESIZE_SHIFT;
	/* physical address start from 0 */
	current_paddr = 0;

	/*
	 * invalidate iopte first , then release io segment
	 */
	for (current_segment = 0;
	     current_segment < ioif_map_info_count; current_segment++) {
		/* create iopte for this segment */
		for (current_page = 0;
		     current_page < (IO_SEGMENTSIZE / IO_PAGESIZE) &&
			     pages_remain;
		     current_page ++, current_paddr =
			     inc_paddr(current_paddr,IO_PAGESIZE)) {
			ret = lv1_put_iopte(0, /* io address space id */
				ioif_map_info_array[current_segment].ioif_addr +
					    current_page *
					    IO_PAGESIZE, /* ioif addr */
				ps3_mm_phys_to_lpar(current_paddr),
				PS3_AUDIO_IOID, IOPTE_INVALID);
			if (ret) {
				printk(KERN_ERR "%s: put_iopte failed (%d) "\
				       "seg=%d paddr=%#lx lpar=%#lx page=%d\n",
				       __FUNCTION__, ret, current_segment,
				       current_paddr,
				       ps3_mm_phys_to_lpar(current_paddr),
				       current_page);
				printk(KERN_ERR "%s: rm_limit=%#lx "\
				       "2nd_base=%#lx 2nd_size=%#lx "\
				       "total=%#lx\n", __FUNCTION__,
				       ps3_rm_limit, ps3_2nd_mem_base,
				       ps3_2nd_mem_size, ps3_mem_total);
				current_segment ++;
			}
			pages_remain --;
		}

		ret = lv1_release_io_segment(0, /* io space */
			     ioif_map_info_array[current_segment].ioif_addr);

		if (ret)
			printk(KERN_ERR "%s: release_io_seg %d failed %d\n",
			       __FUNCTION__, current_segment, ret);
	}

	kfree(ioif_map_info_array);
	ioif_map_info_array = 0;
	_EF;
}

static void snd_ps3_destruct_iopt(void)
{
	snd_ps3_destruct_iopt_helper();
}

/*
 * Interrupt handler
 */
static irqreturn_t snd_ps3_interrupt(int irq, void * dev_id)
{

	uint32_t port_intr;
	int underflow_occured = 0;
	struct snd_ps3_card_info * card = (struct snd_ps3_card_info * )dev_id;



	if (!card->running) {
		AUDIOREG(card, PS3_AUDIO_AX_IS) = -1;
		AUDIOREG(card, PS3_AUDIO_INTR_0) = -1;
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
		printk(KERN_ERR "%s: invalid width %d\n", __FUNCTION__,
		       snd_pcm_format_width(card->substream->runtime->format));
		return -EINVAL;
	}
	return 0;
}
