/*
 * Cell Broadband Engine Performance Monitor
 *
 * (C) Copyright IBM Corporation 2006
 *
 * Author:
 *   David Erb (djerb@us.ibm.com)
 *   Kevin Corry (kevcorry@us.ibm.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef __ASM_CELL_PMU_H__
#define __ASM_CELL_PMU_H__

/* The Cell PMU has four hardware performance counters, which can be
 * configured as four 32-bit counters or eight 16-bit counters.
 */
#define NR_PHYS_CTRS 4
#define NR_CTRS      (NR_PHYS_CTRS * 2)

/* Macros for the pm_control register. */
#define CBE_PM_16BIT_CTR(ctr)              (1 << (24 - ((ctr) & (NR_PHYS_CTRS - 1))))
#define CBE_PM_ENABLE_PERF_MON             0x80000000
#define CBE_PM_STOP_AT_MAX                 0x40000000
#define CBE_PM_TRACE_MODE_GET(pm_control)  (((pm_control) >> 28) & 0x3)
#define CBE_PM_TRACE_MODE_SET(mode)        (((mode)  & 0x3) << 28)
#define CBE_PM_COUNT_MODE_SET(count)       (((count) & 0x3) << 18)
#define CBE_PM_FREEZE_ALL_CTRS             0x00100000
#define CBE_PM_ENABLE_EXT_TRACE            0x00008000

/* Macros for the trace_address register. */
#define CBE_PM_TRACE_BUF_FULL              0x00000800
#define CBE_PM_TRACE_BUF_EMPTY             0x00000400
#define CBE_PM_TRACE_BUF_DATA_COUNT(ta)    ((ta) & 0x3ff)
#define CBE_PM_TRACE_BUF_MAX_COUNT         0x400

/* Macros for the pm07_control registers. */
#define CBE_PM_CTR_INPUT_MUX(pm07_control) (((pm07_control) >> 26) & 0x3f)
#define CBE_PM_CTR_INPUT_CONTROL           0x02000000
#define CBE_PM_CTR_POLARITY                0x01000000
#define CBE_PM_CTR_COUNT_CYCLES            0x00800000
#define CBE_PM_CTR_ENABLE                  0x00400000
#define PM07_CTR_INPUT_MUX(x)              (((x) & 0x3F) << 26)
#define PM07_CTR_INPUT_CONTROL(x)          (((x) & 1) << 25)
#define PM07_CTR_POLARITY(x)               (((x) & 1) << 24)
#define PM07_CTR_COUNT_CYCLES(x)           (((x) & 1) << 23)
#define PM07_CTR_ENABLE(x)                 (((x) & 1) << 22)

/* Macros for the pm_status register. */
#define CBE_PM_CTR_OVERFLOW_INTR(ctr)      (1 << (31 - ((ctr) & 7)))
#define CBE_PM_OVERFLOW_CTRS(pm_status)    (((pm_status) >> 24) & 0xff)
#define CBE_PM_ALL_OVERFLOW_INTR           0xff000000
#define CBE_PM_INTERVAL_INTR               0x00800000
#define CBE_PM_TRACE_BUFFER_FULL_INTR      0x00400000
#define CBE_PM_TRACE_BUFFER_UNDERFLOW_INTR 0x00200000

enum pm_reg_name {
	group_control,
	debug_bus_control,
	trace_address,
	ext_tr_timer,
	pm_status,
	pm_control,
	pm_interval,
	pm_start_stop,
};

#define CBE_COUNT_SUPERVISOR_MODE       0
#define CBE_COUNT_HYPERVISOR_MODE       1
#define CBE_COUNT_PROBLEM_MODE          2
#define CBE_COUNT_ALL_MODES             3

/**
 * struct cell_pmu_ops - Provides a platfrom independent PMU abstraction.
 * @priv: Void pointer variable for platform driver use.
 */

struct cell_pmu_ops {
	void* priv;
	u32 (*read_phys_ctr)(void* p, u32 cpu, u32 phys_ctr);
	void (*write_phys_ctr)(void* p, u32 cpu, u32 phys_ctr, u32 val);
	u32 (*read_ctr)(void* p, u32 cpu, u32 ctr);
	void (*write_ctr)(void* p, u32 cpu, u32 ctr, u32 val);
	u32 (*read_pm07_control)(void* p, u32 cpu, u32 ctr);
	void (*write_pm07_control)(void* p, u32 cpu, u32 ctr, u32 val);
	u32 (*read_pm)(void* p, u32 cpu, enum pm_reg_name reg);
	void (*write_pm)(void* p, u32 cpu, enum pm_reg_name reg, u32 val);
	u32  (*get_ctr_size)(void* p, u32 cpu, u32 phys_ctr);
	void (*set_ctr_size)(void* p, u32 cpu, u32 phys_ctr, u32 ctr_size);
	void (*enable_pm)(void* p, u32 cpu);
	void (*disable_pm)(void* p, u32 cpu);
	void (*read_trace_buffer)(void* p, u32 cpu, u64 *buf);
	u32  (*get_and_clear_pm_interrupts)(void* p, u32 cpu);
	void (*enable_pm_interrupts)(void* p, u32 cpu, u32 thread, u32 mask);
	void (*disable_pm_interrupts)(void* p, u32 cpu);
	void (*sync_irq)(void* p, int node);
};

extern const struct cell_pmu_ops *cell_pmu_ops;

/**
 * cell_pmu_ops_init - Initialize the cell_pmu_ops pointer.
 * @ops: A platfrom specific instance of struct cell_pmu_ops.
 */

static inline void cell_pmu_ops_init(const struct cell_pmu_ops *ops)
{
	cell_pmu_ops = ops;
}

/* Routines for reading/writing the PMU registers. */

static inline u32 cbe_read_phys_ctr(u32 cpu, u32 phys_ctr)
{
	return cell_pmu_ops->read_phys_ctr(cell_pmu_ops->priv, cpu, phys_ctr);
}

static inline void cbe_write_phys_ctr(u32 cpu, u32 phys_ctr, u32 val)
{
	cell_pmu_ops->write_phys_ctr(cell_pmu_ops->priv, cpu, phys_ctr, val);
}

static inline u32 cbe_read_ctr(u32 cpu, u32 ctr)
{
	return cell_pmu_ops->read_ctr(cell_pmu_ops->priv, cpu, ctr);
}

static inline void cbe_write_ctr(u32 cpu, u32 ctr, u32 val)
{
	cell_pmu_ops->write_ctr(cell_pmu_ops->priv, cpu, ctr, val);
}

static inline u32  cbe_read_pm07_control(u32 cpu, u32 ctr)
{
	return cell_pmu_ops->read_pm07_control(cell_pmu_ops->priv, cpu, ctr);
}

static inline void cbe_write_pm07_control(u32 cpu, u32 ctr, u32 val)
{
	cell_pmu_ops->write_pm07_control(cell_pmu_ops->priv, cpu, ctr, val);
}

static inline u32  cbe_read_pm(u32 cpu, enum pm_reg_name reg)
{
	return cell_pmu_ops->read_pm(cell_pmu_ops->priv, cpu, reg);
}

static inline void cbe_write_pm(u32 cpu, enum pm_reg_name reg, u32 val)
{
	cell_pmu_ops->write_pm(cell_pmu_ops->priv, cpu, reg, val);
}

static inline u32 cbe_get_ctr_size(u32 cpu, u32 phys_ctr)
{
	return cell_pmu_ops->get_ctr_size(cell_pmu_ops->priv, cpu, phys_ctr);
}

static inline void cbe_set_ctr_size(u32 cpu, u32 phys_ctr, u32 ctr_size)
{
	cell_pmu_ops->set_ctr_size(cell_pmu_ops->priv, cpu, phys_ctr, ctr_size);
}

static inline void cbe_enable_pm(u32 cpu)
{
	cell_pmu_ops->enable_pm(cell_pmu_ops->priv, cpu);
}

static inline void cbe_disable_pm(u32 cpu)
{
	return cell_pmu_ops->disable_pm(cell_pmu_ops->priv, cpu);
}

static inline void cbe_read_trace_buffer(u32 cpu, u64 *buf)
{
	cell_pmu_ops->read_trace_buffer(cell_pmu_ops->priv, cpu, buf);
}

static inline void cbe_enable_pm_interrupts(u32 cpu, u32 thread, u32 mask)
{
	cell_pmu_ops->enable_pm_interrupts(cell_pmu_ops->priv, cpu, thread,
		mask);
}

static inline void cbe_disable_pm_interrupts(u32 cpu)
{
	cell_pmu_ops->disable_pm_interrupts(cell_pmu_ops->priv, cpu);
}

static inline u32  cbe_get_and_clear_pm_interrupts(u32 cpu)
{
	return cell_pmu_ops->get_and_clear_pm_interrupts(cell_pmu_ops->priv,
		cpu);
}

static inline void cbe_sync_irq(int node)
{
	cell_pmu_ops->sync_irq(cell_pmu_ops->priv, node);
}

#endif /* __ASM_CELL_PMU_H__ */
