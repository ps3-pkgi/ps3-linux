/*
 * PS3 OProfile support
 *
 * This file based on op_model_cell.c, but the spu profiling has not
 * been implemented yet.
 *
 * Copyright (C) 2007 Sony Computer Entertainment Inc.
 * Copyright 2007 Sony Corporation.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307 USA
 */

#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/kthread.h>
#include <linux/oprofile.h>
#include <linux/percpu.h>
#include <linux/smp.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/ptrace.h>
#include <asm/cell-pmu.h>
#include <asm/cputable.h>
#include <asm/oprofile_impl.h>
#include <asm/processor.h>
#include <asm/prom.h>
#include <asm/reg.h>
#include <asm/system.h>
#include <asm/cell-regs.h>
#include <asm/pmc.h>
#include <asm/irq_regs.h>
#include <asm/ps3.h>

#include "../platforms/cell/interrupt.h"
#include "cell/pr_util.h"

#define OP_ERR(f, x...)  pr_info("pmu: " f "\n", ## x)
#define OP_DBG(f, x...)  pr_debug("pmu: " f "\n", ## x)

/*
 * spu_cycle_reset is the number of cycles between samples.
 * This variable is used for SPU profiling and should ONLY be set
 * at the beginning of cell_reg_setup; otherwise, it's read-only.
 */
static unsigned int spu_cycle_reset;

#define NUM_SPUS_PER_NODE    8
#define SPU_CYCLES_EVENT_NUM 2	/*  event number for SPU_CYCLES */

#define PPU_CYCLES_EVENT_NUM 1	/*  event number for CYCLES */
#define PPU_CYCLES_GRP_NUM   1	/* special group number for identifying
				 * PPU_CYCLES event
				 */
#define CBE_COUNT_ALL_CYCLES 0x42800000 /* PPU cycle event specifier */

#define NUM_THREADS 2         /* number of physical threads in
			       * physical processor
			       */
#define NUM_DEBUG_BUS_WORDS 4
#define NUM_INPUT_BUS_WORDS 2

#define MAX_SPU_COUNT 0xFFFFFF	/* maximum 24 bit LFSR value */

struct pmc_cntrl_data {
	unsigned long vcntr;
	unsigned long evnts;
	unsigned long masks;
	unsigned long enabled;
};

struct pm_signal {
	u16 cpu;		/* Processor to modify */
	u16 sub_unit;		/* hw subunit this applies to (if applicable)*/
	short int signal_group; /* Signal Group to Enable/Disable */
	u8 bus_word;		/* Enable/Disable on this Trace/Trigger/Event
				 * Bus Word(s) (bitmask)
				 */
	u8 bit;			/* Trigger/Event bit (if applicable) */
};

struct pm_cntrl {
	u16 enable;
	u16 stop_at_max;
	u16 trace_mode;
	u16 freeze;
	u16 count_mode;
};

static struct {
	u32 group_control;
	u32 debug_bus_control;
	struct pm_cntrl pm_cntrl;
	u32 pm07_cntrl[NR_PHYS_CTRS];
} pm_regs;

#define GET_SUB_UNIT(x) ((x & 0x0000f000) >> 12)
#define GET_BUS_WORD(x) ((x & 0x000000f0) >> 4)
#define GET_BUS_TYPE(x) ((x & 0x00000300) >> 8)
#define GET_POLARITY(x) ((x & 0x00000002) >> 1)
#define GET_COUNT_CYCLES(x) (x & 0x00000001)
#define GET_INPUT_CONTROL(x) ((x & 0x00000004) >> 2)

static DEFINE_PER_CPU(unsigned long[NR_PHYS_CTRS], pmc_values);

static struct pmc_cntrl_data pmc_cntrl[NUM_THREADS][NR_PHYS_CTRS];

/*
 * Interpetation of hdw_thread:
 * 0 - even virtual cpus 0, 2, 4,...
 * 1 - odd virtual cpus 1, 3, 5, ...
 *
 * FIXME: this is strictly wrong, we need to clean this up in a number
 * of places. It works for now. -arnd
 */
static u32 hdw_thread;

static u32 virt_cntr_inter_mask;
static struct timer_list timer_virt_cntr;

/*
 * pm_signal needs to be global since it is initialized in
 * cell_reg_setup at the time when the necessary information
 * is available.
 */
static struct pm_signal pm_signal[NR_PHYS_CTRS];

static u32 reset_value[NR_PHYS_CTRS];
static u32 count_value[NR_PHYS_CTRS];
static int num_counters;
static int oprofile_running;
static DEFINE_SPINLOCK(virt_cntr_lock);

static u32 ctr_enabled;

static unsigned char input_bus[NUM_INPUT_BUS_WORDS];

static u32 ps3_cpu_to_node(int cpu)
{
	return 0;
}

static int pm_activate_signals(u32 node, u32 count)
{
	int i, j;
	struct pm_signal pm_signal_local[NR_PHYS_CTRS];

	/*
	 * There is no debug setup required for the cycles event.
	 * Note that only events in the same group can be used.
	 * Otherwise, there will be conflicts in correctly routing
	 * the signals on the debug bus.  It is the responsiblity
	 * of the OProfile user tool to check the events are in
	 * the same group.
	 */
	i = 0;
	for (j = 0; j < count; j++) {
		if (pm_signal[j].signal_group != PPU_CYCLES_GRP_NUM) {

			/* fw expects physical cpu # */
			pm_signal_local[i].cpu = node;
			pm_signal_local[i].signal_group
			    = pm_signal[j].signal_group;
			pm_signal_local[i].bus_word =
			    pm_signal[j].bus_word;
			pm_signal_local[i].sub_unit =
			    pm_signal[j].sub_unit;
			pm_signal_local[i].bit = pm_signal[j].bit;
			ps3_set_signal(pm_signal[j].signal_group,
				       pm_signal[j].bit,
				       pm_signal[j].sub_unit,
				       pm_signal[j].bus_word);
			i++;
		}
	}
	return 0;
}

/*
 * PM Signal functions
 */
static void set_pm_event(u32 ctr, int event, u32 unit_mask)
{
	struct pm_signal *p;
	u32 signal_bit;
	u32 bus_word, bus_type, count_cycles, polarity, input_control;
	int j, i;

	if (event == PPU_CYCLES_EVENT_NUM) {
		/* Special Event: Count all cpu cycles */
		pm_regs.pm07_cntrl[ctr] = CBE_COUNT_ALL_CYCLES;
		p = &(pm_signal[ctr]);
		p->signal_group = PPU_CYCLES_GRP_NUM;
		p->bus_word = 1;
		p->sub_unit = 0;
		p->bit = 0;
		goto out;
	} else {
		pm_regs.pm07_cntrl[ctr] = 0;
	}

	bus_word = GET_BUS_WORD(unit_mask);
	bus_type = GET_BUS_TYPE(unit_mask);
	count_cycles = GET_COUNT_CYCLES(unit_mask);
	polarity = GET_POLARITY(unit_mask);
	input_control = GET_INPUT_CONTROL(unit_mask);
	signal_bit = (event % 100);

	p = &(pm_signal[ctr]);

	p->signal_group = event / 100;
	p->bus_word = bus_word;
	p->sub_unit = GET_SUB_UNIT(unit_mask);

	/*
	 * This parameter is used to specify the target physical/logical
	 * PPE/SPE object.
	 */
	if (p->signal_group < 42 || 56 < p->signal_group)
		p->sub_unit = 1;

	pm_regs.pm07_cntrl[ctr] = 0;
	pm_regs.pm07_cntrl[ctr] |= PM07_CTR_COUNT_CYCLES(count_cycles);
	pm_regs.pm07_cntrl[ctr] |= PM07_CTR_POLARITY(polarity);
	pm_regs.pm07_cntrl[ctr] |= PM07_CTR_INPUT_CONTROL(input_control);

	/*
	 * Some of the islands signal selection is based on 64 bit words.
	 * The debug bus words are 32 bits, the input words to the performance
	 * counters are defined as 32 bits.  Need to convert the 64 bit island
	 * specification to the appropriate 32 input bit and bus word for the
	 * performance counter event selection.  See the CELL Performance
	 * monitoring signals manual and the Perf cntr hardware descriptions
	 * for the details.
	 */
	if (input_control == 0) {
		if (signal_bit > 31) {
			signal_bit -= 32;
			if (bus_word == 0x3)
				bus_word = 0x2;
			else if (bus_word == 0xc)
				bus_word = 0x8;
		}

		if ((bus_type == 0) && p->signal_group >= 60)
			bus_type = 2;
		if ((bus_type == 0) &&
		    (30 <= p->signal_group && p->signal_group <= 40))
			bus_type = 2;
		if ((bus_type == 1) && p->signal_group >= 50)
			bus_type = 0;

		pm_regs.pm07_cntrl[ctr] |= PM07_CTR_INPUT_MUX(signal_bit);
	} else {
		pm_regs.pm07_cntrl[ctr] = 0;
		p->bit = signal_bit;
	}

	for (i = 0; i < NUM_DEBUG_BUS_WORDS; i++) {
		if (bus_word & (1 << i)) {
			pm_regs.debug_bus_control |=
				(bus_type << (30 - (2 * i)));
			for (j = 0; j < NUM_INPUT_BUS_WORDS; j++) {
				if (input_bus[j] == 0xff) {
					input_bus[j] = i;
					pm_regs.group_control |=
						(i << (30 - (2 * j)));

					break;
				}
			}
		}
	}
	OP_DBG("pm07_ctrl[%d] : 0x%x", ctr, pm_regs.pm07_cntrl[ctr]);
	OP_DBG("group_control : 0x%x", pm_regs.group_control);
	OP_DBG("debug_bus_control : 0x%x", pm_regs.debug_bus_control);
out:
	;
}

static void write_pm_cntrl(int cpu)
{
	/*
	 * Oprofile will use 32 bit counters, set bits 7:10 to 0
	 * pmregs.pm_cntrl is a global
	 */

	u32 val = 0;
	if (pm_regs.pm_cntrl.enable == 1)
		val |= CBE_PM_ENABLE_PERF_MON;

	if (pm_regs.pm_cntrl.stop_at_max == 1)
		val |= CBE_PM_STOP_AT_MAX;

	if (pm_regs.pm_cntrl.trace_mode == 1)
		val |= CBE_PM_TRACE_MODE_SET(pm_regs.pm_cntrl.trace_mode);

	if (pm_regs.pm_cntrl.freeze == 1)
		val |= CBE_PM_FREEZE_ALL_CTRS;

	/*
	 * Routine set_count_mode must be called previously to set
	 * the count mode based on the user selection of user and kernel.
	 */
	val |= CBE_PM_COUNT_MODE_SET(pm_regs.pm_cntrl.count_mode);
	cbe_write_pm(cpu, pm_control, val);
}

static inline void
set_count_mode(u32 kernel, u32 user)
{
	OP_DBG("set_count_mode k:%d u:%d", kernel, user);
	/*
	 * The user must specify user and kernel if they want them. If
	 *  neither is specified, OProfile will count in hypervisor mode.
	 *  pm_regs.pm_cntrl is a global
	 *
	 *  NOTE : PS3 hypervisor rejects ALL_MODES and HYPERVISOR_MODE.
	 *         So, ALL_MODES and HYPERVISOR_MODE are changed to
	 *         PROBLEM_MODE.
	 */
	if (kernel) {
		if (user)
			pm_regs.pm_cntrl.count_mode =
				CBE_COUNT_PROBLEM_MODE;
		else
			pm_regs.pm_cntrl.count_mode =
				CBE_COUNT_SUPERVISOR_MODE;
	} else {
		if (user)
			pm_regs.pm_cntrl.count_mode =
				CBE_COUNT_PROBLEM_MODE;
		else
			pm_regs.pm_cntrl.count_mode =
				CBE_COUNT_PROBLEM_MODE;
	}
}

static inline void enable_ctr(u32 cpu, u32 ctr, u32 *pm07_cntrl)
{

	pm07_cntrl[ctr] |= CBE_PM_CTR_ENABLE;
	cbe_write_pm07_control(cpu, ctr, pm07_cntrl[ctr]);
}

static void add_sample(u32 cpu)
{
	struct pt_regs *regs;
	u64 pc;
	int is_kernel;
	int i;
	u32 value;

	regs = get_irq_regs();
	if (oprofile_running == 1) {
		pc = regs->nip;
		is_kernel = is_kernel_addr(pc);

		for (i = 0; i < num_counters; ++i) {
			value = cbe_read_ctr(cpu, i);
			if (value >= count_value[i] && count_value[i] != 0) {
				OP_DBG("pmu:add_sample ctr:%d"
				       " value:0x%x reset:0x%x count:0x%x",
				       i, value, reset_value[i],
				       count_value[i]);
				oprofile_add_pc(pc, is_kernel, i);
				cbe_write_ctr(cpu, i, reset_value[i]);
			}
		}
	}
}

/*
 * Oprofile is expected to collect data on all CPUs simultaneously.
 * However, there is one set of performance counters per node.	There are
 * two hardware threads or virtual CPUs on each node.  Hence, OProfile must
 * multiplex in time the performance counter collection on the two virtual
 * CPUs.  The multiplexing of the performance counters is done by this
 * virtual counter routine.
 *
 * The pmc_values used below is defined as 'per-cpu' but its use is
 * more akin to 'per-node'.  We need to store two sets of counter
 * values per node -- one for the previous run and one for the next.
 * The per-cpu[NR_PHYS_CTRS] gives us the storage we need.  Each odd/even
 * pair of per-cpu arrays is used for storing the previous and next
 * pmc values for a given node.
 * NOTE: We use the per-cpu variable to improve cache performance.
 *
 * This routine will alternate loading the virtual counters for
 * virtual CPUs
 */
static void cell_virtual_cntr(unsigned long data)
{
	int i, prev_hdw_thread, next_hdw_thread;
	u32 cpu;
	unsigned long flags;

	/*
	 * Make sure that the interrupt_hander and the virt counter are
	 * not both playing with the counters on the same node.
	 */

	spin_lock_irqsave(&virt_cntr_lock, flags);

	prev_hdw_thread = hdw_thread;

	/* switch the cpu handling the interrupts */
	hdw_thread = 1 ^ hdw_thread;
	next_hdw_thread = hdw_thread;

	pm_regs.group_control = 0;
	pm_regs.debug_bus_control = 0;

	for (i = 0; i < NUM_INPUT_BUS_WORDS; i++)
		input_bus[i] = 0xff;

	/*
	 * There are some per thread events.  Must do the
	 * set event, for the thread that is being started
	 */
	for (i = 0; i < num_counters; i++)
		set_pm_event(i,
			pmc_cntrl[next_hdw_thread][i].evnts,
			pmc_cntrl[next_hdw_thread][i].masks);

	/*
	 * The following is done only once per each node, but
	 * we need cpu #, not node #, to pass to the cbe_xxx functions.
	 */
	for_each_online_cpu(cpu) {
		if (ps3_get_hw_thread_id(cpu))
			continue;

		/*
		 * stop counters, save counter values, restore counts
		 * for previous thread
		 */
		cbe_disable_pm(cpu);
		cbe_disable_pm_interrupts(cpu);

		/*
		 * Add sample data at here.
		 * Because PS3 hypervisor does not have
		 * the performance monitor interrupt feature.
		 */
		add_sample(cpu);

		/*
		 * Switch to the other thread. Change the interrupt
		 * and control regs to be scheduled on the CPU
		 * corresponding to the thread to execute.
		 */
		for (i = 0; i < num_counters; i++) {
			if (pmc_cntrl[next_hdw_thread][i].enabled) {
				/*
				 * There are some per thread events.
				 * Must do the set event, enable_cntr
				 * for each cpu.
				 */
				enable_ctr(cpu, i,
					   pm_regs.pm07_cntrl);
			} else {
				cbe_write_pm07_control(cpu, i, 0);
			}
		}

		/* Enable interrupts on the CPU thread that is starting */
		cbe_enable_pm_interrupts(cpu, next_hdw_thread,
					 virt_cntr_inter_mask);
		cbe_enable_pm(cpu);
	}

	spin_unlock_irqrestore(&virt_cntr_lock, flags);

	mod_timer(&timer_virt_cntr, jiffies + HZ / 10);
}

static void start_virt_cntrs(void)
{
	init_timer(&timer_virt_cntr);
	timer_virt_cntr.function = cell_virtual_cntr;
	timer_virt_cntr.data = 0UL;
	timer_virt_cntr.expires = jiffies + HZ / 10;
	add_timer(&timer_virt_cntr);
}

/* This function is called once for all cpus combined */
static int cell_reg_setup(struct op_counter_config *ctr,
			struct op_system_config *sys, int num_ctrs)
{
	int i, j, cpu;
	int ret;

	spu_cycle_reset = 0;

	ret = ps3_lpm_open(PS3_LPM_TB_TYPE_NONE, NULL, 0);
	if (ret) {
		OP_ERR("lpm_open error. %d", ret);
		return -EFAULT;
	}

	if (ctr[0].event == SPU_CYCLES_EVENT_NUM)
		spu_cycle_reset = ctr[0].count;

	num_counters = num_ctrs;

	pm_regs.group_control = 0;
	pm_regs.debug_bus_control = 0;

	/* setup the pm_control register */
	memset(&pm_regs.pm_cntrl, 0, sizeof(struct pm_cntrl));
	pm_regs.pm_cntrl.stop_at_max = 1;
	pm_regs.pm_cntrl.trace_mode = 0;
	pm_regs.pm_cntrl.freeze = 1;

	set_count_mode(sys->enable_kernel, sys->enable_user);

	/* Setup the thread 0 events */
	for (i = 0; i < num_ctrs; ++i) {

		pmc_cntrl[0][i].evnts = ctr[i].event;
		pmc_cntrl[0][i].masks = ctr[i].unit_mask;
		pmc_cntrl[0][i].enabled = ctr[i].enabled;
		pmc_cntrl[0][i].vcntr = i;

		for_each_possible_cpu(j)
			per_cpu(pmc_values, j)[i] = 0;
	}

	/*
	 * Setup the thread 1 events, map the thread 0 event to the
	 * equivalent thread 1 event.
	 */
	for (i = 0; i < num_ctrs; ++i) {
		if ((ctr[i].event >= 2100) && (ctr[i].event <= 2111))
			pmc_cntrl[1][i].evnts = ctr[i].event + 19;
		else if (ctr[i].event == 2203)
			pmc_cntrl[1][i].evnts = ctr[i].event;
		else if ((ctr[i].event >= 2200) && (ctr[i].event <= 2215))
			pmc_cntrl[1][i].evnts = ctr[i].event + 16;
		else
			pmc_cntrl[1][i].evnts = ctr[i].event;

		pmc_cntrl[1][i].masks = ctr[i].unit_mask;
		pmc_cntrl[1][i].enabled = ctr[i].enabled;
		pmc_cntrl[1][i].vcntr = i;
	}

	for (i = 0; i < NUM_INPUT_BUS_WORDS; i++)
		input_bus[i] = 0xff;

	/*
	 * Our counters count up, and "count" refers to
	 * how much before the next interrupt, and we interrupt
	 * on overflow.	 So we calculate the starting value
	 * which will give us "count" until overflow.
	 * Then we set the events on the enabled counters.
	 */
	for (i = 0; i < num_counters; ++i) {
		/* start with virtual counter set 0 */
		if (pmc_cntrl[0][i].enabled) {
			reset_value[i] = 0;
			count_value[i] = ctr[i].count;
			set_pm_event(i,
				     pmc_cntrl[0][i].evnts,
				     pmc_cntrl[0][i].masks);

			/* global, used by cell_cpu_setup */
			ctr_enabled |= (1 << i);
		}
	}

	/* initialize the previous counts for the virtual cntrs */
	for_each_online_cpu(cpu)
		for (i = 0; i < num_counters; ++i)
			per_cpu(pmc_values, cpu)[i] = reset_value[i];

	return 0;
}



/* This function is called once for each cpu */
static int cell_cpu_setup(struct op_counter_config *cntr)
{
	u32 cpu = smp_processor_id();
	u32 num_enabled = 0;
	int i;

	if (spu_cycle_reset)
		return 0;

	/* There is one performance monitor per processor chip (i.e. node),
	 * so we only need to perform this function once per node.
	 */
	if (ps3_get_hw_thread_id(cpu))
		return 0;

	/* Stop all counters */
	cbe_disable_pm(cpu);
	cbe_disable_pm_interrupts(cpu);

	cbe_write_pm(cpu, pm_interval, 0);
	cbe_write_pm(cpu, pm_start_stop, 0);
	cbe_write_pm(cpu, group_control, pm_regs.group_control);
	cbe_write_pm(cpu, debug_bus_control, pm_regs.debug_bus_control);
	write_pm_cntrl(cpu);

	for (i = 0; i < num_counters; ++i) {
		if (ctr_enabled & (1 << i)) {
			pm_signal[num_enabled].cpu = ps3_cpu_to_node(cpu);
			num_enabled++;
		}
	}

	return pm_activate_signals(ps3_cpu_to_node(cpu), num_enabled);
}


static int cell_global_start_spu(struct op_counter_config *ctr)
{
	return -ENOSYS;
}

static int cell_global_start_ppu(struct op_counter_config *ctr)
{
	u32 cpu, i;
	u32 interrupt_mask = 0;

	/* This routine gets called once for the system.
	 * There is one performance monitor per node, so we
	 * only need to perform this function once per node.
	 */
	for_each_online_cpu(cpu) {
		if (ps3_get_hw_thread_id(cpu))
			continue;

		interrupt_mask = 0;

		for (i = 0; i < num_counters; ++i) {
			if (ctr_enabled & (1 << i)) {
				cbe_write_ctr(cpu, i, reset_value[i]);
				enable_ctr(cpu, i, pm_regs.pm07_cntrl);
				interrupt_mask |=
				    CBE_PM_CTR_OVERFLOW_INTR(i);
			} else {
				/* Disable counter */
				cbe_write_pm07_control(cpu, i, 0);
			}
		}

		cbe_get_and_clear_pm_interrupts(cpu);
		cbe_enable_pm_interrupts(cpu, hdw_thread, interrupt_mask);
		cbe_enable_pm(cpu);
	}

	virt_cntr_inter_mask = interrupt_mask;
	oprofile_running = 1;
	/* complete the previous store */
	smp_wmb();

	/*
	 * NOTE: start_virt_cntrs will result in cell_virtual_cntr() being
	 * executed which manipulates the PMU.	We start the "virtual counter"
	 * here so that we do not need to synchronize access to the PMU in
	 * the above for-loop.
	 */
	start_virt_cntrs();

	return 0;
}

static int cell_global_start(struct op_counter_config *ctr)
{
	if (spu_cycle_reset)
		return cell_global_start_spu(ctr);
	else
		return cell_global_start_ppu(ctr);
}

static void cell_global_stop_spu(void)
{
}

static void cell_global_stop_ppu(void)
{
	int cpu;

	/*
	 * This routine will be called once for the system.
	 * There is one performance monitor per node, so we
	 * only need to perform this function once per node.
	 */
	del_timer_sync(&timer_virt_cntr);
	oprofile_running = 0;
	/* complete the previous store */
	smp_wmb();

	for_each_online_cpu(cpu) {
		if (ps3_get_hw_thread_id(cpu))
			continue;

		/* Stop the counters */
		cbe_disable_pm(cpu);

		/* Deactivate the signals */
		ps3_set_signal(0, 0, 0, 0);	/*clear all */

		/* Deactivate interrupts */
		cbe_disable_pm_interrupts(cpu);
	}
}

static void cell_global_stop(void)
{
	if (spu_cycle_reset)
		cell_global_stop_spu();
	else
		cell_global_stop_ppu();
}


/*
 * This function is called from the generic OProfile
 * driver.  When profiling PPUs, we need to do the
 * generic sync start; otherwise, do spu_sync_start.
 */
static int cell_sync_start(void)
{
	OP_ERR("PS3 oprofile support");
	return DO_GENERIC_SYNC;
}

static int cell_sync_stop(void)
{
	int ret;

	ret = ps3_lpm_close();
	if (ret)
		OP_ERR("lpm_close error. %d", ret);

	return 1;
}

struct op_powerpc_model op_model_ps3 = {
	.reg_setup = cell_reg_setup,
	.cpu_setup = cell_cpu_setup,
	.global_start = cell_global_start,
	.global_stop = cell_global_stop,
	.sync_start = cell_sync_start,
	.sync_stop = cell_sync_stop,
};
