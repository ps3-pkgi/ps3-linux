/*
 * Copyright (c) 2005 David Gibson, IBM Corporation.
 *
 * Based on other versions:
 * Copyright (c) 2005 Hewlett-Packard Development Company, L.P.
 * Contributed by Stephane Eranian <eranian@hpl.hp.com>
 *
 * This file contains powerpc specific definitions for the perfmon
 * interface.
 *
 * This file MUST never be included directly. Use linux/perfmon.h.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of version 2 of the GNU General Public
 * License as published by the Free Software Foundation.
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
#ifndef _ASM_POWERPC_PERFMON_H_
#define _ASM_POWERPC_PERFMON_H_

#ifdef __KERNEL__

#include <asm/pmc.h>

#define HID0_PMC5_6_GR_MODE (1UL << (63 - 40))

/*
 * The Linux port to the POWER architecture has a performance optimization
 * where hardware interrupts are disabled not during a call to
 * spin_lock_irqsave, but rather sets a cpu-specific flag and if an interrupt
 * actually occurs the exception handler wrapper looks at that cpu-specific
 * flag, and if it's set, hardware interrupts are disabled, a second
 * cpu-specific flag is set indicating hardware interrupts have been disabled,
 * and the wrapper returns to the interrupted code, leaving the exceptions
 * pending.  When the call to spin_unlock_irqrestore is made, that second
 * cpu-specific flag is checked, and if it's set, hardware interrupts are
 * restored, at which time any pending exceptions will occur.
 *
 * However, on POWER, this special wrapper is not used because it is
 * desirable for oprofile to be able to profile spin_lock_irqsaved code.
 *
 * To get the best of both worlds, we stick with the original exception
 * wrapper and change perfmon's usage of spin_lock_irqsave and
 * spin_unlock_irqrestore so that we instead call pfm_spin_lock_irqsave and
 * pfm_spin_unlock_irqrestore respectively:
 *
 * With this change, all other architectures have #defines that map
 * pfm_spin_lock_irqsave and pfm_spin_lock_irqrestore back to
 * spin_lock_irqsave and spin_lock_irqrestore.
 *
 * On POWER, the macros are changed such that pfm_spin_lock_irqsave forces the
 * state where it does a hardware lockout of interrupts, and sets the
 * cpu-specific flags as though we've done a "soft disable" and that an
 * exception has already occurred, so that when the [unmodified]
 * spin_unlock_irqrestore is called, it will do the right thing by re-enabling
 * interrupts.
 */
#undef pfm_spin_lock_irqsave
#define pfm_spin_lock_irqsave(l,f) \
	do { \
		local_irq_save(f); \
		hard_irq_disable(); \
		spin_lock(l); \
	} while(0)

enum powerpc_pmu_type {
	PFM_POWERPC_PMU_NONE,
	PFM_POWERPC_PMU_604,
	PFM_POWERPC_PMU_604e,
	PFM_POWERPC_PMU_750,	/* XXX: Minor event set diffs between IBM and Moto. */
	PFM_POWERPC_PMU_7400,
	PFM_POWERPC_PMU_7450,
	PFM_POWERPC_PMU_POWER4,
	PFM_POWERPC_PMU_POWER5,
	PFM_POWERPC_PMU_POWER5p,
	PFM_POWERPC_PMU_POWER6,
	PFM_POWERPC_PMU_CELL,
};

struct pfm_arch_pmu_info {
	enum powerpc_pmu_type pmu_style;

	void (*write_pmc)(unsigned int cnum, u64 value);
	void (*write_pmd)(unsigned int cnum, u64 value);

	u64 (*read_pmd)(unsigned int cnum);

	void (*enable_counters)(struct pfm_context *ctx,
				struct pfm_event_set *set);
	void (*disable_counters)(struct pfm_context *ctx,
				 struct pfm_event_set *set);

	void (*irq_handler)(struct pt_regs *regs, struct pfm_context *ctx);
	void (*get_ovfl_pmds)(struct pfm_context *ctx,
			      struct pfm_event_set *set);

	/* The following routines are optional. */
	void (*restore_pmcs)(struct pfm_event_set *set);
	void (*restore_pmds)(struct pfm_event_set *set);

	int  (*ctxswout_thread)(struct task_struct *task,
				struct pfm_context *ctx,
				struct pfm_event_set *set);
	void (*ctxswin_thread)(struct task_struct *task,
			       struct pfm_context *ctx,
			       struct pfm_event_set *set);
	int  (*load_context)(struct pfm_context *ctx,
			     struct pfm_event_set *set,
			     struct task_struct *task);
	int  (*unload_context)(struct pfm_context *ctx,
			       struct task_struct *task);
	int  (*acquire_pmu)(void);
	void (*release_pmu)(void);
	int  (*create_context)(struct pfm_context *ctx,
			       u32 ctx_flags);
	void (*free_context)(struct pfm_context *ctx);
	void *platform_info;
};

#ifdef CONFIG_PPC32
#define PFM_ARCH_PMD_STK_ARG	6 /* conservative value */
#define PFM_ARCH_PMC_STK_ARG	6 /* conservative value */
#else
#define PFM_ARCH_PMD_STK_ARG	8 /* conservative value */
#define PFM_ARCH_PMC_STK_ARG	8 /* conservative value */
#endif

static inline void pfm_arch_resend_irq(void)
{}

static inline void pfm_arch_serialize(void)
{}

static inline void pfm_arch_write_pmc(struct pfm_context *ctx,
				      unsigned int cnum,
				      u64 value)
{
	struct pfm_arch_pmu_info *arch_info = pfm_pmu_conf->arch_info;

	/*
	 * we only write to the actual register when monitoring is
	 * active (pfm_start was issued)
	 */
	if (ctx && ctx->flags.started == 0)
		return;

	BUG_ON(!arch_info->write_pmc);

	arch_info->write_pmc(cnum, value);
}

static inline void pfm_arch_write_pmd(struct pfm_context *ctx,
				      unsigned int cnum, u64 value)
{
	struct pfm_arch_pmu_info *arch_info = pfm_pmu_conf->arch_info;

	value &= pfm_pmu_conf->ovfl_mask;

	BUG_ON(!arch_info->write_pmd);

	arch_info->write_pmd(cnum, value);
}

static inline u64 pfm_arch_read_pmd(struct pfm_context *ctx, unsigned int cnum)
{
	struct pfm_arch_pmu_info *arch_info = pfm_pmu_conf->arch_info;

	BUG_ON(!arch_info->read_pmd);

	return arch_info->read_pmd(cnum);
}

/*
 * For some CPUs, the upper bits of a counter must be set in order for the
 * overflow interrupt to happen. On overflow, the counter has wrapped around,
 * and the upper bits are cleared. This function may be used to set them back.
 */
static inline void pfm_arch_ovfl_reset_pmd(struct pfm_context *ctx,
					   unsigned int cnum)
{
	u64 val = pfm_arch_read_pmd(ctx, cnum);

	/* This masks out overflow bit 31 */
	pfm_arch_write_pmd(ctx, cnum, val);
}

/*
 * At certain points, perfmon needs to know if monitoring has been
 * explicitely started/stopped by user via pfm_start/pfm_stop. The
 * information is tracked in flags.started. However on certain
 * architectures, it may be possible to start/stop directly from
 * user level with a single assembly instruction bypassing
 * the kernel. This function must be used to determine by
 * an arch-specific mean if monitoring is actually started/stopped.
 */
static inline int pfm_arch_is_active(struct pfm_context *ctx)
{
	return ctx->flags.started;
}

static inline void pfm_arch_ctxswout_sys(struct task_struct *task,
					 struct pfm_context *ctx,
					 struct pfm_event_set *set)
{}

static inline void pfm_arch_ctxswin_sys(struct task_struct *task,
					struct pfm_context *ctx,
					struct pfm_event_set *set)
{}

void pfm_arch_init_percpu(void);
int  pfm_arch_is_monitoring_active(struct pfm_context *ctx);
int  pfm_arch_ctxswout_thread(struct task_struct *task, struct pfm_context *ctx,
			      struct pfm_event_set *set);
void pfm_arch_ctxswin_thread(struct task_struct *task, struct pfm_context *ctx,
			     struct pfm_event_set *set);
void pfm_arch_stop(struct task_struct *task, struct pfm_context *ctx,
			  struct pfm_event_set *set);
void pfm_arch_start(struct task_struct *task, struct pfm_context *ctx,
			   struct pfm_event_set *set);
void pfm_arch_restore_pmds(struct pfm_context *ctx, struct pfm_event_set *set);
void pfm_arch_restore_pmcs(struct pfm_context *ctx, struct pfm_event_set *set);
int  pfm_arch_get_ovfl_pmds(struct pfm_context *ctx,
			    struct pfm_event_set *set);
char *pfm_arch_get_pmu_module_name(void);
/*
 * called from __pfm_interrupt_handler(). ctx is not NULL.
 * ctx is locked. PMU interrupt is masked.
 *
 * must stop all monitoring to ensure handler has consistent view.
 * must collect overflowed PMDs bitmask  into povfls_pmds and
 * npend_ovfls. If no interrupt detected then npend_ovfls
 * must be set to zero.
 */
static inline void pfm_arch_intr_freeze_pmu(struct pfm_context *ctx, struct pfm_event_set *set)
{
	pfm_arch_stop(current, ctx, set);
}

void powerpc_irq_handler(struct pt_regs *regs);

/*
 * unfreeze PMU from pfm_do_interrupt_handler()
 * ctx may be NULL for spurious
 */
static inline void pfm_arch_intr_unfreeze_pmu(struct pfm_context *ctx)
{
	struct pfm_arch_pmu_info *arch_info;

	if (!ctx)
		return;

	PFM_DBG_ovfl("state=%d", ctx->state);

	ctx->flags.started = 1;

	if (ctx->state == PFM_CTX_MASKED)
		return;

	arch_info = pfm_pmu_conf->arch_info;
	BUG_ON(!arch_info->enable_counters);
	arch_info->enable_counters(ctx, ctx->active_set);
}

/*
 * PowerPC does not save the PMDs during pfm_arch_intr_freeze_pmu(), thus
 * this routine needs to do it when switching sets on overflow
 */
static inline void pfm_arch_save_pmds_from_intr(struct pfm_context *ctx,
					   struct pfm_event_set *set)
{
	pfm_save_pmds(ctx, set);
}

/*
 * this function is called from the PMU interrupt handler ONLY.
 * On PPC, the PMU is frozen via arch_stop, masking would be implemented
 * via arch-stop as well. Given that the PMU is already stopped when
 * entering the interrupt handler, we do not need to stop it again, so
 * this function is a nop.
 */
static inline void pfm_arch_mask_monitoring(struct pfm_context *ctx,
					    struct pfm_event_set *set)
{}

/*
 * Simply need to start the context in order to unmask.
 */
static inline void pfm_arch_unmask_monitoring(struct pfm_context *ctx,
					      struct pfm_event_set *set)
{
	pfm_arch_start(current, ctx, set);
}


static inline int pfm_arch_pmu_config_init(struct pfm_pmu_config *cfg)
{
	return 0;
}

static inline void pfm_arch_pmu_config_remove(void)
{}

static inline int pfm_arch_context_create(struct pfm_context *ctx,
					  u32 ctx_flags)
{
	struct pfm_arch_pmu_info *arch_info = pfm_pmu_conf->arch_info;
	int rc = 0;

	if (arch_info->create_context)
		rc = arch_info->create_context(ctx, ctx_flags);

	return rc;
}

static inline void pfm_arch_context_free(struct pfm_context *ctx)
{
	struct pfm_arch_pmu_info *arch_info = pfm_pmu_conf->arch_info;

	if (arch_info->free_context)
		arch_info->free_context(ctx);
}

/* not necessary on PowerPC */
static inline void pfm_cacheflush(void *addr, unsigned int len)
{}

/*
 * function called from pfm_setfl_sane(). Context is locked
 * and interrupts are masked.
 * The value of flags is the value of ctx_flags as passed by
 * user.
 *
 * function must check arch-specific set flags.
 * Return:
 * 	1 when flags are valid
 *      0 on error
 */
static inline int pfm_arch_setfl_sane(struct pfm_context *ctx, u32 flags)
{
	return 0;
}

static inline int pfm_arch_init(void)
{
	return 0;
}

static inline int pfm_arch_load_context(struct pfm_context *ctx,
					struct pfm_event_set *set,
					struct task_struct *task)
{
	struct pfm_arch_pmu_info *arch_info = pfm_pmu_conf->arch_info;
	int rc = 0;

	if (arch_info->load_context)
		rc = arch_info->load_context(ctx, set, task);

	return rc;
}

static inline int pfm_arch_unload_context(struct pfm_context *ctx,
					  struct task_struct *task)
{
	struct pfm_arch_pmu_info *arch_info = pfm_pmu_conf->arch_info;
	int rc = 0;

	if (arch_info->unload_context)
		rc = arch_info->unload_context(ctx, task);

	return rc;
}

/*
 * not applicable to powerpc
 */
static inline int pfm_smpl_buffer_alloc_compat(struct pfm_context *ctx,
					       size_t rsize, struct file *filp)
{
	return -EINVAL;
}

static inline ssize_t pfm_arch_compat_read(struct pfm_context *ctx,
			     char __user *buf,
			     int non_block,
			     size_t size)
{
	return -EINVAL;
}

static inline int pfm_arch_pmu_acquire(void)
{
	struct pfm_arch_pmu_info *arch_info = pfm_pmu_conf->arch_info;
	int rc = 0;

	if (arch_info->acquire_pmu) {
		rc = arch_info->acquire_pmu();
		if (rc)
			return rc;
	}

	return reserve_pmc_hardware(powerpc_irq_handler);
}

static inline void pfm_arch_pmu_release(void)
{
	struct pfm_arch_pmu_info *arch_info = pfm_pmu_conf->arch_info;

	if (arch_info->release_pmu)
		arch_info->release_pmu();

	release_pmc_hardware();
}

static inline void pfm_arch_arm_handle_work(struct task_struct *task)
{}

static inline void pfm_arch_disarm_handle_work(struct task_struct *task)
{}

struct task_struct *pfm_get_task_by_pid(int pid);
void pfm_put_task(struct task_struct *p);

struct pfm_arch_context {
	/* Cell: Most recent value of the pm_status
	 * register read by the interrupt handler.
	 *
	 * Interrupt handler sets last_read_updated if it
	 * just read and updated last_read_pm_status
	 */
	u32 last_read_pm_status;
	u32 last_read_updated;
	u64 powergs_pmc5, powergs_pmc6;
	u64 delta_tb, delta_tb_start;
	u64 delta_purr, delta_purr_start;
};

#define PFM_ARCH_CTX_SIZE sizeof(struct pfm_arch_context)
/*
 * PowerPC does not need extra alignment requirements for the sampling buffer
 */
#define PFM_ARCH_SMPL_ALIGN_SIZE	0


#endif /* __KERNEL__ */
#endif /* _ASM_POWERPC_PERFMON_H_ */
