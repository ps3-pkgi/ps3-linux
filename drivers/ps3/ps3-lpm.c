/*
 * PS3 Logical Performance Monitor.
 *
 *  Copyright (C) 2007 Sony Computer Entertainment Inc.
 *  Copyright 2007 Sony Corp.
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
#define DEBUG
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/uaccess.h>
#include <asm/ps3.h>
#include <asm/lv1call.h>
#include <asm/cell-pmu.h>

#define PS3_SIZE_OF_PM_INTERNAL_TRACE_BUFFER        0x4000
#define PS3_SIZE_OF_PM_DEFAULT_TRACE_BUFFER_CACHE   0x4000

/**
 * struct ps3_lpm_priv - private lpm device data.
 *
 * @mutex: Open/close mutex.
 * @rights: lpm rigths granted by system policy module.
 * @pu_id: lv1's BE prosessor id.
 * @outlet_id: outlet created by lv1 for the lpm instance.
 * @lpm_id: lv1 lpm instance id
 * @constructed: the lpm driver has been opened -- can we just use (lpm_id == ???)
 * @sizeof_tb: lv1's trace buffer size
 * @tb_cache: Trace buffer cache
 * @sizeof_traced_data: Traced data size
 * @sbd: the struct ps3_system_bus_device attached to this driver
 */

struct ps3_lpm_priv {
	struct mutex mutex;
	u64 rights;
	u64 pu_id;
	u64 outlet_id;
	u64 lpm_id;
	int constructed;
	void *default_tb_cache;
	u64 sizeof_tb;
	void *tb_cache;
	u64 sizeof_tb_cache;
	u64 sizeof_traced_data;
	u64 sizeof_total_copied_data;
	u64 shadow_pm_control;
	u64 shadow_pm_start_stop;
	u64 shadow_pm_interval;
	u64 shadow_group_control;
	u64 shadow_debug_bus_control;
	struct ps3_system_bus_device *sbd;
};

/**
 * lpm_priv - Static instance of the lpm data.
 *
 * Since the exported routines don't support the notion of a device
 * instance, we need to keep this static variable to hold the instance
 * and not allow more than one instance to be created at any one time.
 */

static struct ps3_lpm_priv *lpm_priv;

/*
 * USE_START_STOP_BOOKMARK enables the PPU bookmark trace.
 * And it enables PPU bookmark triggers ONLY if the other triggers are not set.
 * The start/stop bookmarks are inserted at ps3_enable_pm() and ps3_disable_pm()
 * to start/stop LPM.
 *
 * This macro is used to get good quality of the performance counter.
 */
#define USE_START_STOP_BOOKMARK

/* BOOKMARK tag macros */
#define PS3_PM_BOOKMARK_START                    0x8000000000000000ULL
#define PS3_PM_BOOKMARK_STOP                     0x4000000000000000ULL
#define PS3_PM_BOOKMARK_TAG_KERNEL               0x1000000000000000ULL
#define PS3_PM_BOOKMARK_TAG_USER                 0x3000000000000000ULL
#define PS3_PM_BOOKMARK_TAG_MASK_HI              0xF000000000000000ULL
#define PS3_PM_BOOKMARK_TAG_MASK_LO              0x0F00000000000000ULL

/* CBE PM CONTROL register macros */
#define PS3_PM_CONTROL_PPU_TH0_BOOKMARK          0x00001000
#define PS3_PM_CONTROL_PPU_TH1_BOOKMARK          0x00000800
#define PS3_PM_CONTROL_PPU_COUNT_MODE_MASK       0x000C0000
#define PS3_PM_CONTROL_PPU_COUNT_MODE_PROBLEM    0x00080000
#define PS3_WRITE_PM_MASK                        0xFFFFFFFFFFFFFFFFULL

/* CBE PM START STOP register macros */
#define PS3_PM_START_STOP_PPU_TH0_BOOKMARK_START 0x02000000
#define PS3_PM_START_STOP_PPU_TH1_BOOKMARK_START 0x01000000
#define PS3_PM_START_STOP_PPU_TH0_BOOKMARK_STOP  0x00020000
#define PS3_PM_START_STOP_PPU_TH1_BOOKMARK_STOP  0x00010000
#define PS3_PM_START_STOP_START_MASK             0xFF000000
#define PS3_PM_START_STOP_STOP_MASK              0x00FF0000

/* CBE PM COUNTER register macres */
#define PS3_PM_COUNTER_MASK_HI                   0xFFFFFFFF00000000ULL
#define PS3_PM_COUNTER_MASK_LO                   0x00000000FFFFFFFFULL

/* BASE SIGNAL GROUP NUMBER macros */
#define PM_ISLAND2_BASE_SIGNAL_GROUP_NUMBER  0
#define PM_ISLAND2_SIGNAL_GROUP_NUMBER1      6
#define PM_ISLAND2_SIGNAL_GROUP_NUMBER2      7
#define PM_ISLAND3_BASE_SIGNAL_GROUP_NUMBER  7
#define PM_ISLAND4_BASE_SIGNAL_GROUP_NUMBER  15
#define PM_SPU_TRIGGER_SIGNAL_GROUP_NUMBER   17
#define PM_SPU_EVENT_SIGNAL_GROUP_NUMBER     18
#define PM_ISLAND5_BASE_SIGNAL_GROUP_NUMBER  18
#define PM_ISLAND6_BASE_SIGNAL_GROUP_NUMBER  24
#define PM_ISLAND7_BASE_SIGNAL_GROUP_NUMBER  49
#define PM_ISLAND8_BASE_SIGNAL_GROUP_NUMBER  52
#define PM_SIG_GROUP_SPU                     41
#define PM_SIG_GROUP_SPU_TRIGGER             42
#define PM_SIG_GROUP_SPU_EVENT               43
#define PM_SIG_GROUP_MFC_MAX                 60

/* shadow register macros */
#define PS3_SHADOW_REG_INIT_VALUE          0xFFFFFFFF00000000ULL

/* bookmark spr address */
#define BOOKMARK_SPR_ADDR 1020

inline void ps3_set_bookmark(u64 bookmark)
{
	/*
	 * To avoid bookmark lost, the following nops are added.
	 */
	asm volatile("nop;nop;nop;nop;nop;nop;nop;nop;nop;");
	mtspr(BOOKMARK_SPR_ADDR, bookmark);
	asm volatile("nop;nop;nop;nop;nop;nop;nop;nop;nop;");
}
EXPORT_SYMBOL_GPL(ps3_set_bookmark);

inline void ps3_set_pm_bookmark(u64 tag, u64 incident, u64 th_id)
{
	u64 bookmark;

	bookmark = (get_tb() & 0x00000000FFFFFFFFULL) |
		PS3_PM_BOOKMARK_TAG_KERNEL;
	bookmark = ((tag << 56) & PS3_PM_BOOKMARK_TAG_MASK_LO) |
		(incident << 48) | (th_id << 32) | bookmark;
	ps3_set_bookmark(bookmark);
}
EXPORT_SYMBOL_GPL(ps3_set_pm_bookmark);

/*
 * Read physical counter registers.
 * Each physical counter can act as one 32-bit counter or two 16-bit counters.
 */
u32 ps3_read_phys_ctr(u32 cpu, u32 phys_ctr)
{
	u32 val = 0;
	u64 counter0415;
	u64 counter2637;
	int ret;

	if (phys_ctr < NR_PHYS_CTRS) {
		ret = lv1_set_lpm_counter(lpm_priv->lpm_id, 0, 0, 0, 0,
					  &counter0415, &counter2637);
		switch (phys_ctr) {
		case 0:
			val = (u32)(counter0415 >> 32);
			break;
		case 1:
			val = (u32)(counter0415 & PS3_PM_COUNTER_MASK_LO);
			break;
		case 2:
			val = (u32)(counter2637 >> 32);
			break;
		case 3:
			val = (u32)(counter2637 & PS3_PM_COUNTER_MASK_LO);
			break;
		default:
			val = 0;
			break;
		}
		if (ret)
			dev_err(&lpm_priv->sbd->core,
				"%s:%u: cnum:%d error:%d\n", __func__,
				__LINE__, phys_ctr, ret);
	}
	return val;
}
EXPORT_SYMBOL_GPL(ps3_read_phys_ctr);

/*
 * Write physical counter registers.
 * Each physical counter can act as one 32-bit counter or two 16-bit counters.
 */
void ps3_write_phys_ctr(u32 cpu, u32 phys_ctr, u32 val)
{
	u64 counter0415;
	u64 counter0415_mask;
	u64 counter2637;
	u64 counter2637_mask;
	int ret;
	u64 tmp;

	ret = 0;
	tmp = val;
	if (phys_ctr < NR_PHYS_CTRS) {
		switch (phys_ctr) {
		case 0:
			counter0415 = tmp << 32;
			counter0415_mask = PS3_PM_COUNTER_MASK_HI;
			counter2637 = 0x0;
			counter2637_mask = 0x0;
			break;
		case 1:
			counter0415 = tmp;
			counter0415_mask = PS3_PM_COUNTER_MASK_LO;
			counter2637 = 0x0;
			counter2637_mask = 0x0;
			break;
		case 2:
			counter0415 = 0x0;
			counter0415_mask = 0x0;
			counter2637 = tmp << 32;
			counter2637_mask = PS3_PM_COUNTER_MASK_HI;
			break;
		case 3:
			counter0415 = 0x0;
			counter0415_mask = 0x0;
			counter2637 = tmp;
			counter2637_mask = PS3_PM_COUNTER_MASK_LO;
			break;
		default:
			return ;
		}

		ret = lv1_set_lpm_counter(lpm_priv->lpm_id,
					  counter0415, counter0415_mask,
					  counter2637, counter2637_mask,
					  &counter0415, &counter2637);
		if (ret)
			dev_err(&lpm_priv->sbd->core,
				"%s:%u: cnum:%d value:0x%x error:%d\n",
				__func__, __LINE__, phys_ctr, val, ret);
	}
}
EXPORT_SYMBOL_GPL(ps3_write_phys_ctr);

/*
 * read 16-bits or 32-bits depending on the
 * current size of the counter. Counters 4 - 7 are always 16-bit.
 */
u32 ps3_read_ctr(u32 cpu, u32 ctr)
{
	u32 val;
	u32 phys_ctr = ctr & (NR_PHYS_CTRS - 1);

	val = ps3_read_phys_ctr(cpu, phys_ctr);

	if (ps3_get_ctr_size(cpu, phys_ctr) == 16)
		val = (ctr < NR_PHYS_CTRS) ? (val >> 16) : (val & 0xffff);

	return val;
}
EXPORT_SYMBOL_GPL(ps3_read_ctr);

/*
 * write 16-bits or 32-bits depending on the
 * current size of the counter. Counters 4 - 7 are always 16-bit.
 */
void ps3_write_ctr(u32 cpu, u32 ctr, u32 val)
{
	u32 phys_ctr;
	u32 phys_val;

	phys_ctr = ctr & (NR_PHYS_CTRS - 1);

	if (ps3_get_ctr_size(cpu, phys_ctr) == 16) {
		phys_val = ps3_read_phys_ctr(cpu, phys_ctr);

		if (ctr < NR_PHYS_CTRS)
			val = (val << 16) | (phys_val & 0xffff);
		else
			val = (val & 0xffff) | (phys_val & 0xffff0000);
	}

	ps3_write_phys_ctr(cpu, phys_ctr, val);
}
EXPORT_SYMBOL_GPL(ps3_write_ctr);

/*
 * Read Counter-control registers.
 * Each "logical" counter has a corresponding control register.
 */
u32 ps3_read_pm07_control(u32 cpu, u32 ctr)
{
	return 0;
}
EXPORT_SYMBOL_GPL(ps3_read_pm07_control);

/*
 * Write Counter-control registers.
 * Each "logical" counter has a corresponding control register.
 */
void ps3_write_pm07_control(u32 cpu, u32 ctr, u32 val)
{
	u64 mask;
	u64 old_value;
	int ret;

	if (ctr < NR_CTRS) {
		mask = 0xFFFFFFFFFFFFFFFFULL;
		ret = lv1_set_lpm_counter_control(lpm_priv->lpm_id, ctr,
						  val, mask, &old_value);
		if (ret)
			dev_err(&lpm_priv->sbd->core,
				"%s:%u: cnum:%d value:0x%x error:%d\n",
				__func__, __LINE__, ctr, val, ret);
	}
}
EXPORT_SYMBOL_GPL(ps3_write_pm07_control);

/*
 * Read Other LPM control registers.
 */
u32 ps3_read_pm(u32 cpu, enum pm_reg_name reg)
{
	u32 val = 0;

	switch (reg) {
	case pm_control:
		val = lpm_priv->shadow_pm_control;
		break;
	case trace_address:
		val = CBE_PM_TRACE_BUF_EMPTY;
		break;
	case pm_start_stop:
		val = lpm_priv->shadow_pm_start_stop;
		break;
	default:
		val = 0;
		break;
	}
	return val;
}
EXPORT_SYMBOL_GPL(ps3_read_pm);

/*
 * Write Other LPM control registers.
 */
void ps3_write_pm(u32 cpu, enum pm_reg_name reg, u32 val)
{
	int ret;
	u64 dummy;

	ret = 0;
	switch (reg) {
	case group_control:
		if (val != lpm_priv->shadow_group_control)
			ret = lv1_set_lpm_group_control(lpm_priv->lpm_id, val,
							PS3_WRITE_PM_MASK,
							&dummy);
		lpm_priv->shadow_group_control = val;
		break;

	case debug_bus_control:
		if (val != lpm_priv->shadow_debug_bus_control)
			ret = lv1_set_lpm_debug_bus_control(lpm_priv->lpm_id, val,
 PS3_WRITE_PM_MASK,
							    &dummy);
		lpm_priv->shadow_debug_bus_control = val;
		break;

	case pm_control:
		/*
		 * count mode is always problem-mode.
		 * because lv-1 lpm allows only problem-mode.
		 */
		val = (val & ~PS3_PM_CONTROL_PPU_COUNT_MODE_MASK) |
			PS3_PM_CONTROL_PPU_COUNT_MODE_PROBLEM;
#ifdef USE_START_STOP_BOOKMARK
		val = val | PS3_PM_CONTROL_PPU_TH0_BOOKMARK |
			PS3_PM_CONTROL_PPU_TH1_BOOKMARK ;
#endif
		if (val != lpm_priv->shadow_pm_control)
			ret = lv1_set_lpm_general_control(lpm_priv->lpm_id, val,
							  PS3_WRITE_PM_MASK,
							  0, 0,
							  &dummy, &dummy);
		lpm_priv->shadow_pm_control = val;
		break;

	case pm_interval:
		if (val != lpm_priv->shadow_pm_interval)
			ret = lv1_set_lpm_interval(lpm_priv->lpm_id, val,
						   PS3_WRITE_PM_MASK, &dummy);
		lpm_priv->shadow_pm_interval = val;
		break;

	case pm_start_stop:
		if (val != lpm_priv->shadow_pm_start_stop)
			ret = lv1_set_lpm_trigger_control(lpm_priv->lpm_id, val,
							  PS3_WRITE_PM_MASK,
							  &dummy);
		lpm_priv->shadow_pm_start_stop = val;
		break;
	default:
		ret = 0;
		break;
	}

	if (ret)
		dev_err(&lpm_priv->sbd->core,
			"%s:%u: reg:%d value:0x%x error:%d\n", __func__,
			__LINE__, reg, val, ret);
}
EXPORT_SYMBOL_GPL(ps3_write_pm);

/*
 * Get the size of a physical counter to either 16 or 32 bits.
 */
u32 ps3_get_ctr_size(u32 cpu, u32 phys_ctr)
{
	u32 pm_ctrl, size = 0;

	if (phys_ctr < NR_PHYS_CTRS) {
		pm_ctrl = ps3_read_pm(cpu, pm_control);
		size = (pm_ctrl & CBE_PM_16BIT_CTR(phys_ctr)) ? 16 : 32;
	}

	return size;
}
EXPORT_SYMBOL_GPL(ps3_get_ctr_size);

/*
 * Set the size of a physical counter to either 16 or 32 bits.
 */
void ps3_set_ctr_size(u32 cpu, u32 phys_ctr, u32 ctr_size)
{
	u32 pm_ctrl;

	if (phys_ctr < NR_PHYS_CTRS) {
		pm_ctrl = ps3_read_pm(cpu, pm_control);
		switch (ctr_size) {
		case 16:
			pm_ctrl |= CBE_PM_16BIT_CTR(phys_ctr);
			break;

		case 32:
			pm_ctrl &= ~CBE_PM_16BIT_CTR(phys_ctr);
			break;
		}
		ps3_write_pm(cpu, pm_control, pm_ctrl);
	}
}
EXPORT_SYMBOL_GPL(ps3_set_ctr_size);

static inline u64 pm_translate_signal_group_number_on_island2(
	u64 subgroup)
{

	if (subgroup == 2)
		subgroup = 3;

	if (subgroup <= 6)
		return PM_ISLAND2_BASE_SIGNAL_GROUP_NUMBER + subgroup;
	else if (subgroup == 7)
		return PM_ISLAND2_SIGNAL_GROUP_NUMBER1;
	else
		return PM_ISLAND2_SIGNAL_GROUP_NUMBER2;
}

static inline u64 pm_translate_signal_group_number_on_island3(
	u64 subgroup)
{

	switch (subgroup) {
	case 2:
	case 3:
	case 4:
		subgroup += 2;
		break;
	case 5:
		subgroup = 8;
		break;
	default:
		break;
	}
	return PM_ISLAND3_BASE_SIGNAL_GROUP_NUMBER + subgroup;
}

static inline u64 pm_translate_signal_group_number_on_island4(
	u64 subgroup) {
	return PM_ISLAND4_BASE_SIGNAL_GROUP_NUMBER + subgroup;
}

static inline u64 pm_translate_signal_group_number_on_island5(
	u64 subgroup)
{

	switch (subgroup) {
	case 3:
		subgroup = 4;
		break;
	case 4:
		subgroup = 6;
		break;
	default:
		break;
	}
	return PM_ISLAND5_BASE_SIGNAL_GROUP_NUMBER + subgroup;
}

static inline u64 pm_translate_signal_group_number_on_island6(
	u64 subgroup, u64 subsubgroup)
{
	switch (subgroup) {
	case 3:
	case 4:
	case 5:
		subgroup += 1;
		break;
	default:
		break;
	}

	switch (subsubgroup) {
	case 4:
	case 5:
	case 6:
		subsubgroup += 2;
		break;
	case 7:
	case 8:
	case 9:
	case 10:
		subsubgroup += 4;
		break;
	case 11:
	case 12:
	case 13:
		subsubgroup += 5;
		break;
	default:
		break;
	}

	if (subgroup <= 5)
		return (PM_ISLAND6_BASE_SIGNAL_GROUP_NUMBER + subgroup);
	else
		return (PM_ISLAND6_BASE_SIGNAL_GROUP_NUMBER + subgroup
			+ subsubgroup - 1);
}

static inline u64 pm_translate_signal_group_number_on_island7(
	u64 subgroup)
{
	return PM_ISLAND7_BASE_SIGNAL_GROUP_NUMBER + subgroup;
}

static inline u64 pm_translate_signal_group_number_on_island8(
	u64 subgroup)
{
	return PM_ISLAND8_BASE_SIGNAL_GROUP_NUMBER + subgroup;
}

static u64 pm_signal_group_to_ps3_lv1_signal_group(u64 group)
{
	u64 island;
	u64 subgroup;
	u64 subsubgroup;
	u64 lv1_signal_group;

	subgroup = 0;
	subsubgroup = 0;
	if (group < 1000) {
		if (group < 100) {
			if (20 <= group && group < 30) {
				island = 2;
				subgroup = group - 20;
			} else if (30 <= group && group < 40) {
				island = 3;
				subgroup = group - 30;
			} else if (40 <= group && group < 50) {
				island = 4;
				subgroup = group - 40;
			} else if (50 <= group && group < 60) {
				island = 5;
				subgroup = group - 50;
			} else if (60 <= group && group < 70) {
				island = 6;
				subgroup = group - 60;
			} else if (70 <= group && group < 80) {
				island = 7;
				subgroup = group - 70;
			} else if (80 <= group && group < 90) {
				island = 8;
				subgroup = group - 80;
			} else {
				island = 0;
			}
		} else if (200 <= group && group < 300) {
			island = 2;
			subgroup = group - 200;
		} else if (600 <= group && group < 700) {
			island = 6;
			subgroup = 5;
			subsubgroup = group - 650;
		} else {
			island = 0;
		}
	} else if (6000 <= group && group < 7000) {
		island = 6;
		subgroup = 5;
		subsubgroup = group - 6500;
	} else {
		island = 0;
	}

	switch (island) {
	case 2:
		lv1_signal_group =
 pm_translate_signal_group_number_on_island2(subgroup);
		break;
	case 3:
		lv1_signal_group =
 pm_translate_signal_group_number_on_island3(subgroup);
		break;
	case 4:
		lv1_signal_group =
 pm_translate_signal_group_number_on_island4(subgroup);
		break;
	case 5:
		lv1_signal_group =
 pm_translate_signal_group_number_on_island5(subgroup);
		break;
	case 6:
		lv1_signal_group =
			pm_translate_signal_group_number_on_island6(
				subgroup, subsubgroup);
		break;
	case 7:
		lv1_signal_group =
 pm_translate_signal_group_number_on_island7(subgroup);
		break;
	case 8:
		lv1_signal_group =
 pm_translate_signal_group_number_on_island8(subgroup);
		break;
	default:
		lv1_signal_group = 0;
		break;
	}
	return lv1_signal_group;
}

static u64 pm_bus_word_to_ps3_lv1_bus_word(u8 word)
{

	switch (word) {
	case 1:
		return 0xF000;
	case 2:
		return 0x0F00;
	case 4:
		return 0x00F0;
	case 8:
	default:
		return 0x000F;
	}
}

static int __ps3_set_signal(u64 lv1_signal_group, u64 bus_select,
			    u64 signal_select, u64 attr1, u64 attr2, u64 attr3)
{
	int ret;

	ret = lv1_set_lpm_signal(lpm_priv->lpm_id, lv1_signal_group, bus_select,
				 signal_select, attr1, attr2, attr3);
	if (ret)
		dev_err(&lpm_priv->sbd->core,
			"%s:%u: error:%d 0x%lx 0x%lx 0x%lx 0x%lx 0x%lx 0x%lx\n",
			__func__, __LINE__, ret, lv1_signal_group, bus_select,
			signal_select, attr1, attr2, attr3);

	return ret;
}

int ps3_set_signal(u64 signal_group, u8 signal_bit, u16 sub_unit,
		   u8 bus_word)
{
	int ret;
	u64 lv1_signal_group;
	u64 bus_select;
	u64 signal_select;
	u64 attr1, attr2, attr3;

	if (signal_group == 0)
		return __ps3_set_signal(0, 0, 0, 0, 0, 0);

	lv1_signal_group =
		pm_signal_group_to_ps3_lv1_signal_group(signal_group);
	bus_select = pm_bus_word_to_ps3_lv1_bus_word(bus_word);

	switch (signal_group) {
	case PM_SIG_GROUP_SPU_TRIGGER:
		signal_select = 1;
		signal_select = signal_select << (63 - signal_bit);
		break;
	case PM_SIG_GROUP_SPU_EVENT:
		signal_select = 1;
		signal_select = (signal_select << (63 - signal_bit)) | 0x3;
		break;
	default:
		signal_select = 0;
		break;
	}

	/*
	 * 0: physical object.
	 * 1: logical object.
	 * This parameter is only used for the PPE and SPE signals.
	 */
	attr1 = 1;

	/*
	 * This parameter is used to specify the target physical/logical
	 * PPE/SPE object.
	 */
	if (PM_SIG_GROUP_SPU <= signal_group &&
	    signal_group < PM_SIG_GROUP_MFC_MAX) {
		attr2 = sub_unit;
	} else {
		attr2 = lpm_priv->pu_id;;
	}

	/*
	 * This parameter is only used for setting the SPE signal.
	 */
	attr3 = 0;

	ret = __ps3_set_signal(lv1_signal_group,
			       bus_select,
			       signal_select,
			       attr1,
			       attr2,
			       attr3);
	if (ret)
		dev_err(&lpm_priv->sbd->core, "%s:%u: error:%d\n", __func__,
			__LINE__, ret);

	return ret;
}
EXPORT_SYMBOL_GPL(ps3_set_signal);

inline u32 ps3_get_hw_thread_id(int cpu)
{
	return cpu;
}
EXPORT_SYMBOL_GPL(ps3_get_hw_thread_id);


/*
 * Enable the entire performance monitoring unit.
 * When we enable the LPM, all pending writes to counters get committed.
 */
void ps3_enable_pm(u32 cpu)
{
	int ret;
	u64 tmp;
	u32 tb;

#ifdef USE_START_STOP_BOOKMARK
	int insert_bookmark = 0;
	if (!(lpm_priv->shadow_pm_start_stop &
	      (PS3_PM_START_STOP_START_MASK | PS3_PM_START_STOP_STOP_MASK))) {
		ret = lv1_set_lpm_trigger_control(
			lpm_priv->lpm_id,
			(PS3_PM_START_STOP_PPU_TH0_BOOKMARK_START |
			 PS3_PM_START_STOP_PPU_TH1_BOOKMARK_START |
			 PS3_PM_START_STOP_PPU_TH0_BOOKMARK_STOP |
			 PS3_PM_START_STOP_PPU_TH1_BOOKMARK_STOP),
			0xFFFFFFFFFFFFFFFFULL,
			&tmp);
		insert_bookmark = 1;
	}
#endif
	ret = lv1_start_lpm(lpm_priv->lpm_id);
	if (ret)
		dev_err(&lpm_priv->sbd->core, "%s:%u: Lv-1 lpm: start lpm error\n",
			__func__, __LINE__);

#ifdef USE_START_STOP_BOOKMARK
	if (insert_bookmark) {
		tb = get_tb();
		ps3_set_bookmark(PS3_PM_BOOKMARK_START | tb);
	}
#endif
}
EXPORT_SYMBOL_GPL(ps3_enable_pm);

/*
 * Disable the entire performance monitoring unit.
 */
void ps3_disable_pm(u32 cpu)
{
	int ret;
	u64 param = 0;
	u32 tb;

	tb = get_tb();
	ps3_set_bookmark(PS3_PM_BOOKMARK_STOP | tb);

	ret = lv1_stop_lpm(lpm_priv->lpm_id, &param);
	if (!ret) {
		lpm_priv->sizeof_traced_data = param;
		lpm_priv->sizeof_total_copied_data = 0;
	}
}
EXPORT_SYMBOL_GPL(ps3_disable_pm);

/*
 * Copy the trace buffer.
 */
static u64 _ps3_copy_trace_buffer(u64 offset, u64 size, u64 *to, int to_user)
{
	int ret;
	u64 sizeof_copied_data;

	if (offset >= lpm_priv->sizeof_traced_data)
		return 0;

	ret = lv1_copy_lpm_trace_buffer(lpm_priv->lpm_id, offset, size,
					&sizeof_copied_data);
	if (ret) {
		dev_err(&lpm_priv->sbd->core, "%s:%u: lv1_copy_lpm_trace_buffer error:%d "
			"offset:0x%lx size:0x%lx\n", __func__, __LINE__, ret,
			offset, size);
		return 0;
	}

	if (to_user) {
		if (copy_to_user((void __user *)to, lpm_priv->tb_cache,
				 sizeof_copied_data)) {
			dev_err(&lpm_priv->sbd->core, "%s:%u: copy_to_user() error. "
				"offset:0x%lx size:0x%lx dest:0x%p src:0x%p\n",
				__func__, __LINE__, offset, sizeof_copied_data,
				to, lpm_priv->tb_cache);
			return 0;
		}
	} else
		memcpy(to, lpm_priv->tb_cache, sizeof_copied_data);

	return sizeof_copied_data;
}
u64 ps3_copy_trace_buffer(u64 offset, u64 size, void *to, int to_user)
{
	u64 sz;
	u64 cp_size;
	u64 total_cp_size;

	if (!lpm_priv->tb_cache)
		return 0;

	cp_size = size;
	if (cp_size > lpm_priv->sizeof_tb_cache)
		cp_size = lpm_priv->sizeof_tb_cache;

	total_cp_size = 0;
	while (total_cp_size < size) {
		sz = _ps3_copy_trace_buffer(offset, cp_size, to, to_user);
		if (!sz)
			break;

		total_cp_size += sz;
		offset += sz;
		to = ((u8 *)to + sz);
	}
	return total_cp_size;
}

/*
 * Clearing interrupts for the entire performance monitoring unit.
 */
u32 ps3_get_and_clear_pm_interrupts(u32 cpu)
{
	/* Reading pm_status clears the interrupt bits. */
	return ps3_read_pm(cpu, pm_status);
}
EXPORT_SYMBOL_GPL(ps3_get_and_clear_pm_interrupts);

/*
 * Enabling interrupts for the entire performance monitoring unit.
 */
void ps3_enable_pm_interrupts(u32 cpu, u32 thread, u32 mask)
{
	/* Enable the interrupt bits in the pm_status register. */
	if (mask)
		ps3_write_pm(cpu, pm_status, mask);
}
EXPORT_SYMBOL_GPL(ps3_enable_pm_interrupts);

/*
 * Disabling interrupts for the entire performance monitoring unit.
 */
void ps3_disable_pm_interrupts(u32 cpu)
{
	ps3_get_and_clear_pm_interrupts(cpu);
	ps3_write_pm(cpu, pm_status, 0);
}
EXPORT_SYMBOL_GPL(ps3_disable_pm_interrupts);

int ps3_lpm_open(int is_default_tb_cache, void *tb_cache, u64 tb_cache_size,
	u64 tb_type)
{
	int ret;
	u64 cbe_node_id;
	u64 tb_size;
	u64 ctrl_opt;
	u64 tb_cache_lpar_addr;
	u64 lpm_id;
	u64 outlet_id;
	u64 used_tb_size;

	if (!lpm_priv) {
		BUG();
		return -ENODEV;
	}

	mutex_lock(&lpm_priv->mutex);

	if (lpm_priv->constructed) {
		dev_err(&lpm_priv->sbd->core, "%s:%u: construct Lv-1 lpm error. context state error.\n",
			__func__, __LINE__);
		ret = -EBUSY;
		goto unlock;
	}

	if (is_default_tb_cache) {
		if (!lpm_priv->default_tb_cache) {
			ret = -ENOMEM;
			goto unlock;
		}

		dev_dbg(&lpm_priv->sbd->core, "%s:%u: Use default TB cache\n",
			__func__, __LINE__);
		tb_cache = lpm_priv->default_tb_cache;
		tb_cache_size = PS3_SIZE_OF_PM_DEFAULT_TRACE_BUFFER_CACHE;
	}

	cbe_node_id = 0;
	if (tb_cache) {
		if (tb_type == 0) {
			/* no trace buffer */
			tb_type = 0;
			tb_size = 0;
			ctrl_opt = 0;
			tb_cache_lpar_addr = 0;
		} else if (tb_type == 1) {
			/* internal trace buffer */
			tb_size = PS3_SIZE_OF_PM_INTERNAL_TRACE_BUFFER;
			ctrl_opt = 0;
		} else {
			dev_err(&lpm_priv->sbd->core, "%s:%u: Unkown TB type:0x%lx\n",
			__func__, __LINE__, tb_type);
			ret = -EINVAL;
			goto unlock;
		}
		tb_cache_lpar_addr = (u64)ps3_mm_phys_to_lpar(__pa(tb_cache));
	} else {
		/* no trace buffer */
		tb_type = 0;
		tb_size = 0;
		ctrl_opt = 0;
		tb_cache_lpar_addr = 0;
	}

	ret = lv1_construct_lpm(cbe_node_id, tb_type, tb_size, ctrl_opt,
				tb_cache_lpar_addr, tb_cache_size,
				&lpm_id, &outlet_id, &used_tb_size);

	if (ret) {
		dev_err(&lpm_priv->sbd->core, "%s:%u: construct Lv-1 lpm error:%d\n",
			__func__, __LINE__, ret);
		ret = -EINVAL;
		goto unlock;
	}

	lpm_priv->constructed = 1;
	lpm_priv->tb_cache = tb_cache;
	lpm_priv->sizeof_tb_cache = tb_cache_size;
	lpm_priv->lpm_id = lpm_id;
	lpm_priv->outlet_id = outlet_id;
	lpm_priv->sizeof_tb = used_tb_size;
	lpm_priv->shadow_pm_control = PS3_SHADOW_REG_INIT_VALUE;
	lpm_priv->shadow_pm_start_stop = PS3_SHADOW_REG_INIT_VALUE;
	lpm_priv->shadow_pm_interval = PS3_SHADOW_REG_INIT_VALUE;
	lpm_priv->shadow_group_control = PS3_SHADOW_REG_INIT_VALUE;
	lpm_priv->shadow_debug_bus_control = PS3_SHADOW_REG_INIT_VALUE;

	dev_dbg(&lpm_priv->sbd->core, "%s:%u: Lv-1 lpm: id:0x%lx outlet:0x%lx sizeof_tb:0x%lx\n",
		__func__, __LINE__, lpm_priv->lpm_id,
		lpm_priv->outlet_id, lpm_priv->sizeof_tb);
	ret = 0;
	goto unlock;

unlock:
	mutex_unlock(&lpm_priv->mutex);
	return ret;
}
EXPORT_SYMBOL_GPL(ps3_lpm_open);

int ps3_lpm_close(void)
{
	dev_dbg(&lpm_priv->sbd->core, "%s:%u\n", __func__, __LINE__);

	mutex_lock(&lpm_priv->mutex);

	if (lpm_priv->constructed)
		lv1_destruct_lpm(lpm_priv->lpm_id);

	lpm_priv->constructed = 0;
	lpm_priv->lpm_id = 0;

	mutex_unlock(&lpm_priv->mutex);
	return 0;
}
EXPORT_SYMBOL_GPL(ps3_lpm_close);

static int __devinit ps3_lpm_probe(struct ps3_system_bus_device *dev)
{
	dev_dbg(&dev->core, " -> %s:%u\n", __func__, __LINE__);

	if (lpm_priv) {
		dev_info(&dev->core, "%s:%u: called twice\n",
			__func__, __LINE__);
		return -EBUSY;
	}

	lpm_priv = kzalloc(sizeof(*lpm_priv), GFP_KERNEL);

	if (!lpm_priv)
		return -ENOMEM;

	lpm_priv->sbd = dev;
	lpm_priv->pu_id = dev->lpm.pu_id;
	lpm_priv->rights = dev->lpm.rights;
	mutex_init(&lpm_priv->mutex);

	// this should be in ps3_create_lpm()???
	lpm_priv->default_tb_cache = kzalloc(
		PS3_SIZE_OF_PM_DEFAULT_TRACE_BUFFER_CACHE, GFP_KERNEL);

        if (!lpm_priv->default_tb_cache)
		dev_err(&dev->core, "%s:%u: alloc default_tb_cache failed\n",
			__func__, __LINE__);

	dev_info(&dev->core, " <- %s:%u:\n", __func__, __LINE__);

	return 0;
}

static int ps3_lpm_remove(struct ps3_system_bus_device *dev)
{
	dev_dbg(&dev->core, " -> %s:%u:\n", __func__, __LINE__);

	// need to do other cleanups here!!!

	if(lpm_priv) {
		// this should be in ps3_create_lpm()???
		kfree(lpm_priv->default_tb_cache);

		kfree(lpm_priv);
		lpm_priv = NULL;
	}

	dev_info(&dev->core, " <- %s:%u:\n", __func__, __LINE__);
	return 0;
}

static struct ps3_system_bus_driver ps3_lpm_driver = {
	.match_id = PS3_MATCH_ID_LPM,
	.core.name	= "ps3-lpm",
	.core.owner	= THIS_MODULE,
	.probe		= ps3_lpm_probe,
	.remove		= ps3_lpm_remove,
	.shutdown	= ps3_lpm_remove,
};

static int __init ps3_lpm_init(void)
{
	pr_debug("%s:%d:\n", __func__, __LINE__);
	return ps3_system_bus_driver_register(&ps3_lpm_driver);
}

static void __exit ps3_lpm_exit(void)
{
	pr_debug("%s:%d:\n", __func__, __LINE__);
	ps3_system_bus_driver_unregister(&ps3_lpm_driver);
}

module_init(ps3_lpm_init);
module_exit(ps3_lpm_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("PS3 Logical Performance Monitor Driver");
MODULE_AUTHOR("Sony Corporation");
MODULE_ALIAS(PS3_MODULE_ALIAS_LPM);
