/*
 * perfmon.c: perfmon2 core functions
 *
 * This file implements the perfmon2 interface which
 * provides access to the hardware performance counters
 * of the host processor.
 *
 *
 * The initial version of perfmon.c was written by
 * Ganesh Venkitachalam, IBM Corp.
 *
 * Then it was modified for perfmon-1.x by Stephane Eranian and
 * David Mosberger, Hewlett Packard Co.
 *
 * Version Perfmon-2.x is a complete rewrite of perfmon-1.x
 * by Stephane Eranian, Hewlett Packard Co.
 *
 * Copyright (c) 1999-2006 Hewlett-Packard Development Company, L.P.
 * Contributed by Stephane Eranian <eranian@hpl.hp.com>
 *                David Mosberger-Tang <davidm@hpl.hp.com>
 *
 * More information about perfmon available at:
 * 	http://www.hpl.hp.com/research/linux/perfmon
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
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/vmalloc.h>
#include <linux/poll.h>
#include <linux/ptrace.h>
#include <linux/perfmon_kern.h>
#include <linux/cpu.h>
#include <linux/random.h>

/*
 * internal variables
 */
static struct kmem_cache *pfm_ctx_cachep;

/*
 * external variables
 */
DEFINE_PER_CPU(struct task_struct *, pmu_owner);
DEFINE_PER_CPU(struct pfm_context  *, pmu_ctx);
DEFINE_PER_CPU(u64, pmu_activation_number);
DEFINE_PER_CPU(struct pfm_stats, pfm_stats);
DEFINE_PER_CPU(struct hrtimer, pfm_hrtimer);

#define PFM_INVALID_ACTIVATION	((u64)~0)

int perfmon_disabled;	/* >0 if perfmon is disabled */

void pfm_context_free(struct pfm_context *ctx)
{
	struct pfm_smpl_fmt *fmt;

	pfm_arch_context_free(ctx);

	fmt = ctx->smpl_fmt;

	pfm_free_sets(ctx);

	if (ctx->smpl_addr) {
		PFM_DBG("freeing sampling buffer @%p size=%zu",
			ctx->smpl_real_addr,
			ctx->smpl_real_size);

		pfm_smpl_buf_space_release(ctx, ctx->smpl_real_size);

		if (fmt->fmt_exit)
			(*fmt->fmt_exit)(ctx->smpl_addr);

		vfree(ctx->smpl_real_addr);
	}

	PFM_DBG("free ctx @%p", ctx);
	kmem_cache_free(pfm_ctx_cachep, ctx);
	/*
	 * decrease refcount on:
	 * 	- PMU description table
	 * 	- sampling format
	 */
	pfm_pmu_conf_put();
	pfm_smpl_fmt_put(fmt);
	pfm_pmu_release();
}

/*
 * only called in for the current task
 */
static int pfm_setup_smpl_fmt(struct pfm_smpl_fmt *fmt, void *fmt_arg,
				struct pfm_context *ctx, u32 ctx_flags,
				int mode, struct file *filp)
{
	size_t size = 0;
	int ret = 0;

	/*
	 * validate parameters
	 */
	if (fmt->fmt_validate) {
		ret = (*fmt->fmt_validate)(ctx_flags, pfm_pmu_conf->regs.num_pmds,
					   fmt_arg);
		PFM_DBG("validate(0x%x,%p)=%d", ctx_flags, fmt_arg, ret);
		if (ret)
			goto error;
	}

	/*
	 * check if buffer format wants to use perfmon
	 * buffer allocation/mapping service
	 */
	size = 0;
	if (fmt->fmt_getsize) {
		ret = (*fmt->fmt_getsize)(ctx_flags, fmt_arg, &size);
		if (ret) {
			PFM_DBG("cannot get size ret=%d", ret);
			goto error;
		}
	}

	if (size) {
		if (mode == PFM_COMPAT)
			ret = pfm_smpl_buffer_alloc_compat(ctx, size, filp);
		else
			ret = pfm_smpl_buffer_alloc(ctx, size);
		if (ret)
			goto error;

	}

	if (fmt->fmt_init) {
		ret = (*fmt->fmt_init)(ctx, ctx->smpl_addr, ctx_flags,
				       pfm_pmu_conf->regs.num_pmds,
				       fmt_arg);
		if (ret)
			goto error_buffer;
	}
	return 0;

error_buffer:
	pfm_smpl_buf_space_release(ctx, ctx->smpl_real_size);
	/*
	 * we do not call fmt_exit, if init has failed
	 */
	vfree(ctx->smpl_real_addr);
error:
	return ret;
}

/*
 * interrupts are masked when entering this function.
 * context must be in MASKED state when calling.
 */
static void pfm_unmask_monitoring(struct pfm_context *ctx,
				  struct pfm_event_set *set)
{
	if (ctx->state != PFM_CTX_MASKED)
		return;

	PFM_DBG_ovfl("unmasking monitoring");

	/*
	 * must be done before calling
	 * pfm_arch_unmask_monitoring()
	 */
	ctx->state = PFM_CTX_LOADED;

	/*
	 * we need to restore the PMDs because they
	 * may have been modified by user while MASKED in
	 * which case the actual registers have no yet
	 * been updated
	 */
	pfm_arch_restore_pmds(ctx, set);

	/*
	 * call arch specific handler
	 */
	pfm_arch_unmask_monitoring(ctx, set);

	/*
	 * clear force reload flag. May have been set
	 * in pfm_write_pmcs or pfm_write_pmds
	 */
	set->priv_flags &= ~PFM_SETFL_PRIV_MOD_BOTH;

	/*
	 * reset set duration timer
	 */
	set->duration_start = sched_clock();

	/*
	 * reload what is leftover of the timeout.
	 * Was saved in pfm_mask_montoring()
	 * Note that hrtimer_rem may be negative (in the past)
	 * or 0 means also in the past, i.e., by the time we maskde
	 *
	 * Note that if hrtimer_rem points to now or in the past,
	 * hrtimer_start could wakeup the softirq thread which could
	 * cause a locking problem. However, this cannot happen because
	 * we set the cb_mode = HRTIMER_CB_IRQSAFE_NO_SOFTIRQ which
	 * prevents the function from trying to wakeup the softirq daemon
	 */
	if (set->flags & PFM_SETFL_TIME_SWITCH)
		hrtimer_start(&__get_cpu_var(pfm_hrtimer), set->hrtimer_rem, HRTIMER_MODE_REL);
}

/*
 * called from pfm_smpl_buffer_alloc_old() (IA64-COMPAT)
 * and pfm_setup_smpl_fmt()
 *
 * interrupts are enabled, context is not locked.
 */
int pfm_smpl_buffer_alloc(struct pfm_context *ctx, size_t rsize)
{
#if PFM_ARCH_SMPL_ALIGN_SIZE > 0
#define PFM_ALIGN_SMPL(a, f) (void *)((((unsigned long)(a))+(f-1)) & ~(f-1))
#else
#define PFM_ALIGN_SMPL(a, f) (a)
#endif
	void *addr, *real_addr;
	size_t size, real_size;
	int ret;

	might_sleep();

	/*
	 * align page boundary
	 */
	size = PAGE_ALIGN(rsize);

	/*
	 * On some arch, it may be necessary to get an alignment greater
	 * than page size to avoid certain cache effects (e.g., MIPS).
	 * This is the reason for PFM_ARCH_SMPL_ALIGN_SIZE.
	 */
	real_size = size + PFM_ARCH_SMPL_ALIGN_SIZE;

	PFM_DBG("buffer req_size=%zu actual_size=%zu before", rsize, size);

	ret = pfm_smpl_buf_space_acquire(ctx, real_size);
	if (ret)
		return ret;

	PFM_DBG("buffer req_size=%zu size=%zu real_size=%zu",
		rsize,
		size,
		real_size);

	/*
	 * vmalloc can sleep. we do not hold
	 * any spinlock and interrupts are enabled
	 */
	real_addr = addr = vmalloc(real_size);
	if (!real_addr) {
		PFM_DBG("cannot allocate sampling buffer");
		goto unres;
	}

	/*
	 * align the useable sampling buffer address to the arch requirement
	 * This is a nop on most architectures
	 */
	addr = PFM_ALIGN_SMPL(real_addr, PFM_ARCH_SMPL_ALIGN_SIZE);

	memset(addr, 0, real_size);

	/*
	 * due to cache aliasing, it may be necessary to flush the pages
	 * on certain architectures (e.g., MIPS)
	 */
	pfm_cacheflush(addr, real_size);

	/*
	 * what needs to be freed
	 */
	ctx->smpl_real_addr = real_addr;
	ctx->smpl_real_size = real_size;

	/*
	 * what is actually available to user
	 */
	ctx->smpl_addr = addr;
	ctx->smpl_size = size;

	PFM_DBG("kernel smpl buffer @ used=%p real=%p", addr, real_addr);

	return 0;
unres:
	PFM_DBG("buffer req_size=%zu actual_size=%zu error", rsize, size);
	pfm_smpl_buf_space_release(ctx, real_size);
	return -ENOMEM;
}

void pfm_reset_pmds(struct pfm_context *ctx,
		    struct pfm_event_set *set,
		    int num_pmds,
		    int reset_mode)
{
	u64 val, mask, new_seed;
	struct pfm_pmd *reg;
	unsigned int i, not_masked;

	not_masked = ctx->state != PFM_CTX_MASKED;

	PFM_DBG_ovfl("%s r_pmds=0x%llx not_masked=%d",
		reset_mode == PFM_PMD_RESET_LONG ? "long" : "short",
		(unsigned long long)set->reset_pmds[0],
		not_masked);

	pfm_stats_inc(reset_pmds_count);

	for (i = 0; num_pmds; i++) {
		if (test_bit(i, cast_ulp(set->reset_pmds))) {
			num_pmds--;

			reg = set->pmds + i;

			val = reset_mode == PFM_PMD_RESET_LONG ? reg->long_reset : reg->short_reset;

			if (reg->flags & PFM_REGFL_RANDOM) {
				mask = reg->mask;
				new_seed = random32();

				/* construct a full 64-bit random value: */
				if ((unlikely(mask >> 32) != 0))
					new_seed |= (u64)random32() << 32;

				/* counter values are negative numbers! */
				val -= (new_seed & mask);
			}

			set->pmds[i].value = val;
			reg->lval = val;

			/*
			 * not all PMD to reset are necessarily
			 * counters
			 */
			if (not_masked)
				pfm_write_pmd(ctx, i, val);

			PFM_DBG_ovfl("set%u pmd%u sval=0x%llx",
					set->id,
					i,
					(unsigned long long)val);
		}
	}

	/*
	 * done with reset
	 */
	bitmap_zero(cast_ulp(set->reset_pmds), i);

	/*
	 * make changes visible
	 */
	if (not_masked)
		pfm_arch_serialize();
}

/*
 * called from pfm_handle_work() and __pfm_restart()
 * for system-wide and per-thread context to resume
 * monitoring after a user level notification.
 *
 * In both cases, the context is locked and interrupts
 * are disabled.
 */
static void pfm_resume_after_ovfl(struct pfm_context *ctx)
{
	struct pfm_smpl_fmt *fmt;
	u32 rst_ctrl;
	struct pfm_event_set *set;
	u64 *reset_pmds;
	void *hdr;
	int state, ret;

	hdr = ctx->smpl_addr;
	fmt = ctx->smpl_fmt;
	state = ctx->state;
	set = ctx->active_set;
	ret = 0;

	if (hdr) {
		rst_ctrl = 0;
		prefetch(hdr);
	} else
		rst_ctrl= PFM_OVFL_CTRL_RESET;

	/*
	 * if using a sampling buffer format and it has a restart callback,
	 * then invoke it. hdr may be NULL, if the format does not use a
	 * perfmon buffer
	 */
	if (fmt && fmt->fmt_restart)
		ret = (*fmt->fmt_restart)(state == PFM_CTX_LOADED, &rst_ctrl, hdr);

	reset_pmds = set->reset_pmds;

	PFM_DBG("fmt_restart=%d reset_count=%d set=%u r_pmds=0x%llx switch=%d ctx_state=%d",
		ret,
		ctx->flags.reset_count,
		set->id,
		(unsigned long long)reset_pmds[0],
		(set->priv_flags & PFM_SETFL_PRIV_SWITCH),
		state);

	if (!ret) {
		/*
		 * switch set if needed
		 */
		if (set->priv_flags & PFM_SETFL_PRIV_SWITCH) {
			set->priv_flags &= ~PFM_SETFL_PRIV_SWITCH;
			pfm_switch_sets(ctx, NULL, PFM_PMD_RESET_LONG, 0);
			set = ctx->active_set;
		} else if (rst_ctrl & PFM_OVFL_CTRL_RESET) {
			int nn;
			nn = bitmap_weight(cast_ulp(set->reset_pmds),
					   pfm_pmu_conf->regs.max_pmd);
			if (nn)
				pfm_reset_pmds(ctx, set, nn, PFM_PMD_RESET_LONG);
		}

		if (!(rst_ctrl & PFM_OVFL_CTRL_MASK))
			pfm_unmask_monitoring(ctx, set);
		else
			PFM_DBG("stopping monitoring?");
		ctx->state = PFM_CTX_LOADED;
	}
}

/*
 * This function is always called after pfm_stop has been issued
 */
static void pfm_flush_pmds(struct task_struct *task, struct pfm_context *ctx)
{
	struct pfm_event_set *set;
	u64 *cnt_pmds;
	u64 ovfl_mask;
	u16 num_ovfls, i, first;

	ovfl_mask = pfm_pmu_conf->ovfl_mask;
	first = pfm_pmu_conf->regs.first_intr_pmd;
	cnt_pmds = pfm_pmu_conf->regs.cnt_pmds;

	/*
	 * save active set
	 * UP:
	 * 	if not current task and due to lazy, state may
	 * 	still be live
	 * for system-wide, guaranteed to run on correct CPU
	 */
	if (__get_cpu_var(pmu_ctx) == ctx) {
		/*
		 * pending overflows have been saved by pfm_stop()
		 */
		pfm_save_pmds(ctx, ctx->active_set);
		pfm_set_pmu_owner(NULL, NULL);
		PFM_DBG("released ownership");
	}

	/*
	 * look for pending interrupts
	 */
	list_for_each_entry(set, &ctx->set_list, list) {

		if (!set->npend_ovfls)
			continue;

		num_ovfls = set->npend_ovfls;
		PFM_DBG("set%u nintrs=%u", set->id, num_ovfls);

		for (i = first; num_ovfls; i++) {
			if (test_bit(i, cast_ulp(set->povfl_pmds))) {
				/* only correct value for counters */
				if(test_bit(i, cast_ulp(cnt_pmds))) {
					set->pmds[i].value += 1 + ovfl_mask;
				}
				num_ovfls--;
			}
			PFM_DBG("pmd%u set=%u val=0x%llx",
				i,
				set->id,
				(unsigned long long)set->pmds[i].value);
		}
		/*
		 * we need to clear to prevent a pfm_getinfo_evtsets() from
		 * returning stale data even after the context is unloaded
		 */
		set->npend_ovfls = 0;
		bitmap_zero(cast_ulp(set->povfl_pmds),
			    pfm_pmu_conf->regs.max_intr_pmd);
	}
}

/*
 * This function is called when we need to perform asynchronous
 * work on a context. This function is called ONLY when about to
 * return to user mode (very much like with signal handling).
 *
 * There are several reasons why we come here:
 *
 *  - per-thread mode, not self-monitoring, to reset the counters
 *    after a pfm_restart()
 *
 *  - we are zombie and we need to cleanup our state
 *
 *  - we need to block after an overflow notification
 *    on a context with the PFM_OVFL_NOTIFY_BLOCK flag
 *
 * This function is never called for a system-wide context.
 *
 * pfm_handle_work() can be called with interrupts enabled
 * (TIF_NEED_RESCHED) or disabled. The down_interruptible
 * call may sleep, therefore we must re-enable interrupts
 * to avoid deadlocks. It is safe to do so because this function
 * is called ONLY when returning to user level, in which case
 * there is no risk of kernel stack overflow due to deep
 * interrupt nesting.
 */
void pfm_handle_work(struct pt_regs *regs)
{
	struct pfm_context *ctx;
	unsigned long flags, dummy_flags;
	int type, ret, info;

#ifdef CONFIG_PPC
	/*
	 * This is just a temporary fix. Obviously we'd like to fix the powerpc
	 * code to make that check before calling __pfm_handle_work() to
	 * prevent the function call overhead, but the call is made from assembly
	 * code, so it will take a little while to figure out how to perform the
	 * check correctly.
	 */
	if (!test_thread_flag(TIF_PERFMON_WORK))
		return;
#endif

	if (!user_mode(regs))
		return;

	clear_thread_flag(TIF_PERFMON_WORK);

	pfm_stats_inc(handle_work_count);

	ctx = current->pfm_context;
	if (ctx == NULL) {
		PFM_DBG("[%d] has no ctx", current->pid);
		return;
	}

	BUG_ON(ctx->flags.system);

	pfm_spin_lock_irqsave(&ctx->lock, flags);

	type = ctx->flags.work_type;
	ctx->flags.work_type = PFM_WORK_NONE;

	PFM_DBG("work_type=%d reset_count=%d",
		type,
		ctx->flags.reset_count);

	switch(type) {
		case PFM_WORK_ZOMBIE:
			goto do_zombie;
		case PFM_WORK_RESET:
			/* simply reset, no blocking */
			goto skip_blocking;
		case PFM_WORK_NONE:
			PFM_DBG("unexpected PFM_WORK_NONE");
			goto nothing_todo;
		case PFM_WORK_BLOCK:
			break;
		default:
			PFM_DBG("unkown type=%d", type);
			goto nothing_todo;
	}

	/*
	 * restore interrupt mask to what it was on entry.
	 * Could be enabled/disabled.
	 */
	pfm_spin_unlock_irqrestore(&ctx->lock, flags);

	/*
	 * force interrupt enable because of down_interruptible()
	 */
	local_irq_enable();

	PFM_DBG("before block sleeping");

	/*
	 * may go through without blocking on SMP systems
	 * if restart has been received already by the time we call down()
	 */
	ret = wait_for_completion_interruptible(&ctx->restart_complete);

	PFM_DBG("after block sleeping ret=%d", ret);

	/*
	 * lock context and mask interrupts again
	 * We save flags into a dummy because we may have
	 * altered interrupts mask compared to entry in this
	 * function.
	 */
	pfm_spin_lock_irqsave(&ctx->lock, dummy_flags);

	if (ctx->state == PFM_CTX_ZOMBIE)
		goto do_zombie;

	/*
	 * in case of interruption of down() we don't restart anything
	 */
	if (ret < 0)
		goto nothing_todo;

skip_blocking:
	/*
	 * iterate over the number of pending resets
	 * There are certain situations where there may be
	 * multiple notifications sent before a pfm_restart().
	 * As such, it may be that multiple pfm_restart() are
	 * issued before the monitored thread gets to
	 * pfm_handle_work(). To avoid losing restarts, pfm_restart()
	 * increments a counter (reset_counts). Here, we take this
	 * into account by potentially calling pfm_resume_after_ovfl()
	 * multiple times. It is up to the sampling format to take the
	 * appropriate actions.
	 */
	while(ctx->flags.reset_count) {
		pfm_resume_after_ovfl(ctx);
		ctx->flags.reset_count--;
	}

nothing_todo:
	/*
	 * restore flags as they were upon entry
	 */
	pfm_spin_unlock_irqrestore(&ctx->lock, flags);
	return;

do_zombie:
	PFM_DBG("context is zombie, bailing out");

	__pfm_unload_context(ctx, &info);

	/*
	 * keep the spinlock check happy
	 */
	spin_unlock(&ctx->lock);

	/*
	 * enable interrupt for vfree()
	 */
	local_irq_enable();

	/*
	 * cancel timer now that context is unlocked
	 */
	if (info & 0x2) {
		ret = hrtimer_cancel(&__get_cpu_var(pfm_hrtimer));
		PFM_DBG("timeout cancel=%d", ret);
	}

	/*
	 * actual context free
	 */
	pfm_context_free(ctx);

	/*
	 * restore interrupts as they were upon entry
	 */
	local_irq_restore(flags);

	/* always true */
	if (info & 0x1)
		pfm_session_release(0, 0);
}

/*
 * called only from exit_thread(): task == current
 * we come here only if current has a context
 * attached (loaded or masked or zombie)
 */
void __pfm_exit_thread(struct task_struct *task)
{
	struct pfm_context *ctx;
	unsigned long flags;
	int free_ok = 0, release_info = 0;
	int ret, need_end;

	ctx  = task->pfm_context;

	BUG_ON(ctx->flags.system);

	pfm_spin_lock_irqsave(&ctx->lock, flags);

	PFM_DBG("state=%d is_self=%d", ctx->state, ctx->flags.is_self);

	/*
	 * __pfm_unload_context() cannot fail
	 * in the context states we are interested in
	 */
	switch (ctx->state) {
	case PFM_CTX_LOADED:
	case PFM_CTX_MASKED:
		/* we need to check before calling
		 * __pfm_unload_context() because it
		 * clears is_self
		 */
		need_end = ctx->flags.is_self == 0;
		__pfm_unload_context(ctx, &release_info);
		if (need_end)
			pfm_end_notify(ctx);
		break;
	case PFM_CTX_ZOMBIE:
		__pfm_unload_context(ctx, &release_info);
		free_ok = 1;
		break;
	default:
		BUG_ON(ctx->state != PFM_CTX_LOADED);
		break;
	}
	pfm_spin_unlock_irqrestore(&ctx->lock, flags);

	/*
	 * cancel timer now that context is unlocked
	 */
	if (release_info & 0x2) {
		ret = hrtimer_cancel(&__get_cpu_var(pfm_hrtimer));
		PFM_DBG("timeout cancel=%d", ret);
	}

	if (release_info & 0x1)
		pfm_session_release(0, 0);

	/*
	 * All memory free operations (especially for vmalloc'ed memory)
	 * MUST be done with interrupts ENABLED.
	 */
	if (free_ok)
		pfm_context_free(ctx);
}

/*
 * called from cpu_init() and pfm_pmu_register()
 */
void __pfm_init_percpu(void *dummy)
{
	struct hrtimer *h;

	h = &__get_cpu_var(pfm_hrtimer);

	pfm_arch_init_percpu();

	/*
	 * initialize per-cpu high res timer
	 */
	hrtimer_init(h, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
#ifdef CONFIG_HIGH_RES_TIMERS
	/*
	 * avoid potential deadlock on the runqueue lock
	 * during context switch when multiplexing. Situation
	 * arises on architectures which run switch_to() with
	 * the runqueue lock held, e.g., x86. On others, e.g.,
	 * IA-64, the problem does not exist.
	 * Setting the callback mode to HRTIMER_CB_IRQSAFE_NO_SOFTIRQ
	 * that the callback routine is only called on hardirq context
	 * not on softirq, thus the context switch will not end up
	 * trying to wakeup the softirqd
	 */
	h->cb_mode = HRTIMER_CB_IRQSAFE_NO_SOFTIRQ;
#endif
	h->function = pfm_handle_switch_timeout;
}

/*
 * global initialization routine, executed only once
 */
int __init pfm_init(void)
{
	PFM_LOG("version %u.%u", PFM_VERSION_MAJ, PFM_VERSION_MIN);

	pfm_ctx_cachep = kmem_cache_create("pfm_context",
				   sizeof(struct pfm_context)+PFM_ARCH_CTX_SIZE,
				   SLAB_HWCACHE_ALIGN, 0, NULL);
	if (!pfm_ctx_cachep) {
		PFM_ERR("cannot initialize context slab");
		goto error_disable;
	}

	if (pfm_init_sets())
		goto error_disable;

	if (pfm_init_fs())
		goto error_disable;

	if (pfm_init_sysfs())
		goto error_disable;

	/* not critical, so no error checking */
	pfm_init_debugfs();

	/*
	 * one time, arch-specific global initialization
	 */
	if (pfm_arch_init())
		goto error_disable;

	if (pfm_init_hotplug())
		goto error_disable;
	return 0;

error_disable:
	PFM_ERR("perfmon is disabled due to initialization error");
	perfmon_disabled = 1;
	return -1;
}

/*
 * must use subsys_initcall() to ensure that the perfmon2 core
 * is initialized before any PMU description module when they are
 * compiled in.
 */
subsys_initcall(pfm_init);

/*
 * function used to start monitoring. When operating in per-thread
 * mode and when not self-monitoring, the monitored thread must be
 * stopped.
 *
 * The pfarg_start argument is optional and may be used to designate
 * the initial event set to activate. Wehn missing, the last active
 * set is used. For the first activation, set0 is used.
 *
 * On some architectures, e.g., IA-64, it may be possible to start monitoring
 * without calling this function under certain conditions (per-thread and self
 * monitoring).
 *
 * the context is locked and interrupts are disabled.
 */
int __pfm_start(struct pfm_context *ctx, struct pfarg_start *start)
{
	struct task_struct *task, *owner_task;
	struct pfm_event_set *new_set, *old_set;
	int is_self;

	task = ctx->task;

	/*
	 * UNLOADED: error
	 * LOADED  : normal start, nop if started unless set is different
	 * MASKED  : nop or change set when unmasking
	 * ZOMBIE  : cannot happen
	 */
	if (ctx->state == PFM_CTX_UNLOADED)
		return -EINVAL;

	old_set = new_set = ctx->active_set;

	/*
	 * always the case for system-wide
	 */
	if (task == NULL)
		task = current;

	is_self = task == current;

	/*
	 * argument is provided?
	 */
	if (start) {
		/*
		 * find the set to load first
		 */
		new_set = pfm_find_set(ctx, start->start_set, 0);
		if (new_set == NULL) {
			PFM_DBG("event set%u does not exist",
				start->start_set);
			return -EINVAL;
		}
	}

	PFM_DBG("cur_set=%u req_set=%u",
		old_set->id,
		new_set->id);

	/*
	 * if we need to change the active set we need
	 * to check if we can access the PMU
	 */
	if (new_set != old_set) {

		owner_task = __get_cpu_var(pmu_owner);
		/*
		 * system-wide: must run on the right CPU
		 * per-thread : must be the owner of the PMU context
		 *
		 * pfm_switch_sets() returns with monitoring stopped
		 */
		if (is_self) {
			pfm_switch_sets(ctx, new_set, PFM_PMD_RESET_LONG, 1);
		} else {
			/*
			 * In a UP kernel, the PMU may contain the state
			 * of the task we want to operate on, yet the task
			 * may be switched out (lazy save). We need to save
			 * current state (old_set), switch active_set and
			 * mark it for reload.
			 */
			if (owner_task == task)
				pfm_save_pmds(ctx, old_set);
			ctx->active_set = new_set;
			new_set->priv_flags |= PFM_SETFL_PRIV_MOD_BOTH;
		}
	}

	/*
	 * always start with full timeout
	 */
	new_set->hrtimer_rem = new_set->hrtimer_exp;

	/*
	 * mark as started
	 * must be done before calling pfm_arch_start()
	 */
	ctx->flags.started = 1;


	pfm_arch_start(task, ctx, new_set);

	/*
	 * we check whether we had a pending ovfl before restarting.
	 * If so we need to regenerate the interrupt to make sure we
	 * keep recorded samples. For non-self monitoring this check
	 * is done in the pfm_ctxswin_thread() routine.
	 *
	 * we check new_set/old_set because pfm_switch_sets() already
	 * takes care of replaying the pending interrupts
	 */
	if (is_self && new_set != old_set && new_set->npend_ovfls) {
		pfm_arch_resend_irq();
		pfm_stats_inc(ovfl_intr_replay_count);
	}

	/*
	 * activate timeout for system-wide, self-montoring
	 * Always start with full timeout
	 * Timeout is at least one tick away, so no risk of
	 * having hrtimer_start() trying to wakeup softirqd
	 * and thus causing troubles. This cannot happen anmyway
	 * because cb_mode = HRTIMER_CB_IRQSAFE_NO_SOFTIRQ
	 */
	if (is_self && new_set->flags & PFM_SETFL_TIME_SWITCH) {
		hrtimer_start(&__get_cpu_var(pfm_hrtimer), new_set->hrtimer_rem, HRTIMER_MODE_REL);
		PFM_DBG("set%u started timeout=%lld",
			new_set->id,
			(unsigned long long)new_set->hrtimer_rem.tv64);
	}

	/*
	 * we restart total duration even if context was
	 * already started. In that case, counts are simply
	 * reset.
	 *
	 * For per-thread, if not self-monitoring, the statement
	 * below will have no effect because thread is stopped.
	 * The field is reset of ctxsw in.
	 */
	new_set->duration_start = sched_clock();

	return 0;
}

/*
 * function used to stop monitoring. When operating in per-thread
 * mode and when not self-monitoring, the monitored thread must be
 * stopped.
 *
 * the context is locked and interrupts are disabled.
 *
 * release_info value upon return:
 * 	- bit 0 : unused
 * 	- bit 1 : when set, must cancel hrtimer
 */
int __pfm_stop(struct pfm_context *ctx, int *release_info)
{
	struct pfm_event_set *set;
	struct task_struct *task;
	u64 now;
	int state;

	*release_info = 0;

	now = sched_clock();
	state = ctx->state;
	set = ctx->active_set;

	/*
	 * context must be attached (zombie cannot happen)
	 */
	if (state == PFM_CTX_UNLOADED)
		return -EINVAL;

	task = ctx->task;

	PFM_DBG("ctx_task=[%d] ctx_state=%d is_system=%d",
		task ? task->pid : -1,
		state,
		ctx->flags.system);

	/*
	 * this happens for system-wide context
	 */
	if (task == NULL)
		task = current;

	/*
	 * compute elapsed time
	 *
	 * unless masked, compute elapsed duration, stop timeout
	 */
	if (task == current && state == PFM_CTX_LOADED) {
		/*
		 * timeout cancel must be deferred until context is
		 * unlocked to avoid race with pfm_handle_switch_timeout()
		 */
		if (set->flags & PFM_SETFL_TIME_SWITCH)
			*release_info |= 0x2;

		set->duration += now - set->duration_start;
	}

	pfm_arch_stop(task, ctx, set);

	ctx->flags.started = 0;
	/*
	 * starting now, in-flight PMU interrupt for this context
	 * are treated as spurious
	 */
	return 0;
}

/*
 * function called from sys_pfm_restart(). It is used when overflow
 * notification is requested. For each notification received, the user
 * must call pfm_restart() to indicate to the kernel that it is done
 * processing the notification.
 *
 * When the caller is doing user level sampling, this function resets
 * the overflowed counters and resumes monitoring which is normally stopped
 * during notification (always the consequence of a counter overflow).
 *
 * When using a sampling format, the format restart() callback is invoked,
 * overflowed PMDS may be reset based upon decision from sampling format.
 *
 * When operating in per-thread mode, and when not self-monitoring, the
 * monitored thread DOES NOT need to be stopped, unlike for many other calls.
 *
 * This means that the effect of the restart may not necessarily be observed
 * right when returning from the call. For instance, counters may not already
 * be reset in the other thread.
 *
 * When operating in system-wide, the caller must be running on the monitored
 * CPU.
 *
 * The context is locked and interrupts are disabled.
 *
 * info value upon return:
 * 	- bit 0: when set, mudt issue complete() on restart semaphore
 */
int __pfm_restart(struct pfm_context *ctx, int *info)
{
	int state;

	state = ctx->state;

	PFM_DBG("state=%d can_restart=%d reset_count=%d",
		state,
		ctx->flags.can_restart,
		ctx->flags.reset_count);

	*info = 0;

	switch (state) {
	case PFM_CTX_MASKED:
		break;
	case PFM_CTX_LOADED:
		if (ctx->smpl_addr && ctx->smpl_fmt->fmt_restart)
			break;
	default:
		PFM_DBG("invalid state=%d", state);
		return -EBUSY;
	}

	/*
	 * first check if allowed to restart, i.e., notifications received
	 */
	if (!ctx->flags.can_restart) {
		PFM_DBG("no restart can_restart=0");
		return -EBUSY;
	}

	pfm_stats_inc(pfm_restart_count);

	/*
	 * at this point, the context is either LOADED or MASKED
	 */
	ctx->flags.can_restart--;

	/*
	 * handle self-monitoring case and system-wide
	 */
	if (ctx->task == current || ctx->flags.system) {
		pfm_resume_after_ovfl(ctx);
		return 0;
	}

	/*
	 * restart another task
	 */

	/*
	 * if blocking, then post the semaphore if PFM_CTX_MASKED, i.e.
	 * the task is blocked or on its way to block. That's the normal
	 * restart path. If the monitoring is not masked, then the task
	 * can be actively monitoring and we cannot directly intervene.
	 * Therefore we use the trap mechanism to catch the task and
	 * force it to reset the buffer/reset PMDs.
	 *
	 * if non-blocking, then we ensure that the task will go into
	 * pfm_handle_work() before returning to user mode.
	 *
	 * We cannot explicitly reset another task, it MUST always
	 * be done by the task itself. This works for system wide because
	 * the tool that is controlling the session is logically doing
	 * "self-monitoring".
	 */
	if (ctx->flags.block && state == PFM_CTX_MASKED) {
		PFM_DBG("unblocking [%d]", ctx->task->pid);
		/*
		 * It is not possible to call complete() with the context locked
		 * otherwise we have a potential deadlock with the PMU context
		 * switch code due to a lock inversion between task_rq_lock()
		 * and the context lock.
		 * Instead we mark whether or not we need to issue the complete
		 * and we invoke the function once the context lock is released
		 * in sys_pfm_restart()
		 */
		*info = 1;
	} else {
		PFM_DBG("[%d] armed exit trap", ctx->task->pid);
		ctx->flags.work_type = PFM_WORK_RESET;
		set_tsk_thread_flag(ctx->task, TIF_PERFMON_WORK);
		pfm_arch_arm_handle_work(ctx->task);
	}
	ctx->flags.reset_count++;
	return 0;
}

/*
 * function used to attach a context to either a CPU or a thread.
 * In per-thread mode, and when not self-monitoring, the thread must be
 * stopped. In system-wide mode, the cpu specified in the pfarg_load.load_tgt
 * argument must be the current CPU.
 *
 * The function must be called with the context locked and interrupts disabled.
 */
int __pfm_load_context(struct pfm_context *ctx, struct pfarg_load *req,
		       struct task_struct *task)
{
	struct pfm_event_set *set;
	struct pfm_context *old;
	int mycpu;
	int ret;

	mycpu = smp_processor_id();

	/*
	 * system-wide: check we are running on the desired CPU
	 */
	if (ctx->flags.system && req->load_pid != mycpu) {
		PFM_DBG("running on wrong CPU: %u vs. %u",
			mycpu, req->load_pid);
		return -EINVAL;
	}

	/*
	 * locate first set to activate
	 */
	set = pfm_find_set(ctx, req->load_set, 0);
	if (set == NULL) {
		PFM_DBG("event set%u does not exist", req->load_set);
		return -EINVAL;
	}

	/*
	 * assess sanity of event sets, initialize set state
	 */
	ret = pfm_prepare_sets(ctx, set);
	if (ret) {
		PFM_DBG("invalid next field pointers in the sets");
		return -EINVAL;
	}

	PFM_DBG("load_pid=%d set=%u set_flags=0x%x",
		req->load_pid,
		set->id,
		set->flags);

	/*
	 * per-thread:
	 *   - task to attach to is checked in sys_pfm_load_context() to avoid
	 *     locking issues. if found, and not self,  task refcount was incremented.
	 */
	if (ctx->flags.system) {
		ctx->cpu = mycpu;
		ctx->task = NULL;
	} else {
		old = cmpxchg(&task->pfm_context, NULL, ctx);
		if (old != NULL) {
			PFM_DBG("load_pid=%d has a context "
				"old=%p new=%p cur=%p",
				req->load_pid,
				old,
				ctx,
				task->pfm_context);
			return -EEXIST;
		}
		ctx->task = task;
		ctx->cpu = -1;
	}

	/*
	 * perform any architecture specific actions
	 */
	ret = pfm_arch_load_context(ctx, set, ctx->task);
	if (ret)
		goto error_noload;

	/*
	 * now reserve the session, before we can proceed with
	 * actually accessing the PMU hardware
	 */
	ret = pfm_session_acquire(ctx->flags.system, ctx->cpu);
	if (ret)
		goto error;

	/*
	 * commit active set
	 */
	ctx->active_set = set;

	ctx->flags.is_self = 0;

	set->runs++;

	if (ctx->flags.system) {
		/*
		 * load PMD from set
		 * load PMC from set
		 */
		pfm_arch_restore_pmds(ctx, set);
		pfm_arch_restore_pmcs(ctx, set);

		/*
		 * set new ownership
		 */
		pfm_set_pmu_owner(ctx->task, ctx);

	} else if (ctx->task != current) {
		/* force a full reload */
		ctx->last_act = PFM_INVALID_ACTIVATION;
		ctx->last_cpu = -1;
		set->priv_flags |= PFM_SETFL_PRIV_MOD_BOTH;

		/*
		 * If cell_spe_follow is true, the task is not marked by
		 * TIF_PERFMON_CTXSW and pfm_ctxsw() is not called
		 * from the task scheduler.
		 * pfm_ctxsw() is called from SPU notifier in perfmon_cell.c
		 *
		 */
		if (!ctx->flags.cell_spe_follow) {
			set_tsk_thread_flag(task, TIF_PERFMON_CTXSW);
			PFM_DBG("[%d] set TIF", task->pid);
		}

		PFM_DBG("context loaded next ctxswin for [%d]", task->pid);
	} else {
#ifndef CONFIG_SMP
		/*
		 * in UP mode, because of lazy save/restore
		 * there may already be valid state on the PMU.
		 * We need to push it out before we can load the
		 * next state
		 */
		struct pfm_context *ctxp;
		ctxp = __get_cpu_var(pmu_ctx);
		if (ctxp)
			pfm_save_prev_context(ctxp);
#endif
		ctx->last_cpu = mycpu;
		__get_cpu_var(pmu_activation_number)++;
		ctx->last_act = __get_cpu_var(pmu_activation_number);

		ctx->flags.is_self = 1;

		/*
		 * If cell_spe_follow is true, the task is not marked by
		 * TIF_PERFMON_CTXSW and pfm_ctxsw() is not called
		 * from the task scheduler.
		 * pfm_ctxsw() is called from SPU notifier in perfmon_cell.c
		 *
		 */
		if (!ctx->flags.cell_spe_follow) {
			set_tsk_thread_flag(task, TIF_PERFMON_CTXSW);
			PFM_DBG("[%d] set TIF", task->pid);
		}

		/*
		 * load PMD from set
		 * load PMC from set
		 */
		pfm_arch_restore_pmds(ctx, set);
		pfm_arch_restore_pmcs(ctx, set);

		/*
		 * set new ownership
		 */
		pfm_set_pmu_owner(ctx->task, ctx);
	}

	ctx->flags.work_type = PFM_WORK_NONE;
	ctx->flags.reset_count = 0;

	/*
	 * reset message queue
	 */
	ctx->msgq_head = ctx->msgq_tail = 0;

	ctx->state = PFM_CTX_LOADED;

	return 0;

error:
	pfm_arch_unload_context(ctx, task);
error_noload:
	/*
	 * detach context
	 */
	if (!ctx->flags.system)
		task->pfm_context = NULL;

	return ret;
}

/*
 * Function used to detach a context from either a CPU or a thread.
 * In the per-thread case and when not self-monitoring, the thread must be
 * stopped. After the call, the context is detached and monitoring is stopped.
 *
 * The function must be called with the context locked and interrupts disabled.
 *
 * release_info value upon return:
 * 	- bit 0: when set, must free context
 * 	- bit 1: when set, must cancel hrtimer
 */
int __pfm_unload_context(struct pfm_context *ctx, int *release_info)
{
	struct task_struct *task;
	struct pfm_event_set *set;
	int ret, is_self;

	PFM_DBG("ctx_state=%d task [%d]", ctx->state, ctx->task ? ctx->task->pid : -1);

	*release_info = 0;

	/*
	 * unload only when necessary
	 */
	if (ctx->state == PFM_CTX_UNLOADED)
		return 0;

	task = ctx->task;
	set = ctx->active_set;
	is_self = ctx->flags.system || task == current;

	/*
	 * stop monitoring
	 */
	ret = __pfm_stop(ctx, release_info);
	if (ret)
		return ret;

	ctx->state = PFM_CTX_UNLOADED;
	ctx->flags.can_restart = 0;

	/*
	 * save PMD registers
	 * release ownership
	 */
	pfm_flush_pmds(task, ctx);

	/*
	 * arch-specific unload operations
	 */
	pfm_arch_unload_context(ctx, task);

	/*
	 * per-thread: disconnect from monitored task
	 */
	if (task) {
		task->pfm_context = NULL;
		ctx->task = NULL;
		clear_tsk_thread_flag(task, TIF_PERFMON_CTXSW);
		clear_tsk_thread_flag(task, TIF_PERFMON_WORK);
		pfm_arch_disarm_handle_work(task);
	}

	ctx->flags.is_self = 0;

	*release_info |= 0x1;

	return 0;
}

static inline int pfm_ctx_flags_sane(u32 ctx_flags)
{
	if (ctx_flags & PFM_FL_SYSTEM_WIDE) {
		if (ctx_flags & PFM_FL_NOTIFY_BLOCK) {
			PFM_DBG("cannot use blocking mode in syswide mode");
			return -EINVAL;
		}
	}
	return 0;
}

/*
 * check for permissions to create a context.
 *
 * A sysadmin may decide to restrict creation of per-thread
 * and/or system-wide context to a group of users using the
 * group id via /sys/kernel/perfmon/task_group  and
 * /sys/kernel/perfmon/sys_group.
 *
 * Once we identify a user level package which can be used
 * to grant/revoke Linux capabilites at login via PAM, we will
 * be able to use capabilities. We would also need to increase
 * the size of cap_t to support more than 32 capabilities (it
 * is currently defined as u32 and 32 capabilities are alrady
 * defined).
 */
static inline int pfm_ctx_permissions(u32 ctx_flags)
{
	if (  (ctx_flags & PFM_FL_SYSTEM_WIDE)
	   && pfm_controls.sys_group != PFM_GROUP_PERM_ANY
	   && !in_group_p(pfm_controls.sys_group)) {
		PFM_DBG("user group not allowed to create a syswide ctx");
		return -EPERM;
	} else if (pfm_controls.task_group != PFM_GROUP_PERM_ANY
		   && !in_group_p(pfm_controls.task_group)) {
		PFM_DBG("user group not allowed to create a task context");
		return -EPERM;
	}
	return 0;
}

/*
 * function used to allocate a new context. A context is allocated along
 * with the default event set. If a sampling format is used, the buffer
 * may be allocated and initialized.
 *
 * The file descriptor identifying the context is allocated and returned
 * to caller.
 *
 * This function operates with no locks and interrupts are enabled.
 * return:
 * 	>=0: the file descriptor to identify the context
 * 	<0 : the error code
 */
int __pfm_create_context(struct pfarg_ctx *req,
			 struct pfm_smpl_fmt *fmt,
			 void *fmt_arg,
			 int mode,
			 struct pfm_context **new_ctx)
{
	struct pfm_context *ctx;
	struct pfm_event_set *set;
	struct file *filp = NULL;
	u32 ctx_flags;
	int fd = 0, ret;

	ctx_flags = req->ctx_flags;

	/* Increase refcount on PMU description */
	ret = pfm_pmu_conf_get(1);
	if (ret < 0)
		goto error_conf;

	ret = pfm_ctx_flags_sane(ctx_flags);
	if (ret < 0)
		goto error_alloc;

	ret = pfm_ctx_permissions(ctx_flags);
	if (ret < 0)
		goto error_alloc;

	/*
	 * we can use GFP_KERNEL and potentially sleep because we do
	 * not hold any lock at this point.
	 */
	might_sleep();
	ret = -ENOMEM;
	ctx = kmem_cache_zalloc(pfm_ctx_cachep, GFP_KERNEL);
	if (!ctx)
		goto error_alloc;

	INIT_LIST_HEAD(&ctx->set_list);
	spin_lock_init(&ctx->lock);
	init_completion(&ctx->restart_complete);
	init_waitqueue_head(&ctx->msgq_wait);

	ret = pfm_pmu_acquire();
	if (ret)
		goto error_file;
	/*
	 * check if PMU is usable
	 */
	if (!(pfm_pmu_conf->regs.num_pmcs && pfm_pmu_conf->regs.num_pmcs)) {
		PFM_DBG("no usable PMU registers");
		ret = -EBUSY;
		goto error_file;
	}

	/*
	 * link to format, must be done first for correct
	 * error handling in pfm_context_free()
	 */
	ctx->smpl_fmt = fmt;

	ret = -ENFILE;
	fd = pfm_alloc_fd(&filp);
	if (fd < 0)
		goto error_file;

	/*
	 * context is unloaded
	 */
	ctx->state = PFM_CTX_UNLOADED;

	/*
	 * initialization of context's flags
	 * must be done before pfm_find_set()
	 */
	ctx->flags.block = (ctx_flags & PFM_FL_NOTIFY_BLOCK) ? 1 : 0;
	ctx->flags.system = (ctx_flags & PFM_FL_SYSTEM_WIDE) ? 1: 0;
	ctx->flags.no_msg = (ctx_flags & PFM_FL_OVFL_NO_MSG) ? 1: 0;
	ctx->flags.ia64_v20_compat = mode == PFM_COMPAT ? 1 : 0;

	/*
	 * initialize arch-specific section
	 * must be done before fmt_init()
	 *
	 * XXX: fix dependency with fmt_init()
	 */
	ret = pfm_arch_context_create(ctx, ctx_flags);
	if (ret)
		goto error_set;

	ret = -ENOMEM;
	/*
	 * create initial set
	 */
	if (pfm_find_set(ctx, 0, 1) == NULL)
		goto error_set;

	set = list_first_entry(&ctx->set_list, struct pfm_event_set, list);

	pfm_init_evtset(set);

	/*
	 * does the user want to sample?
	 */
	if (fmt) {
		ret = pfm_setup_smpl_fmt(fmt, fmt_arg, ctx, ctx_flags,
					 mode, filp);
		if (ret)
			goto error_set;
	}

	filp->private_data = ctx;

	ctx->last_act = PFM_INVALID_ACTIVATION;
	ctx->last_cpu = -1;

	/*
	 * initialize notification message queue
	 */
	ctx->msgq_head = ctx->msgq_tail = 0;

	PFM_DBG("ctx=%p flags=0x%x system=%d notify_block=%d no_msg=%d"
		" use_fmt=%d ctx_fd=%d mode=%d",
		ctx,
		ctx_flags,
		ctx->flags.system,
		ctx->flags.block,
		ctx->flags.no_msg,
		fmt != NULL,
		fd, mode);

	*new_ctx = ctx;

	/*
	 * we defer the fd_install until we are certain the call succeeded
	 * to ensure we do not have to undo its effect. Neither put_filp()
	 * nor put_unused_fd() undoes the effect of fd_install().
	 */
	fd_install(fd, filp);

	return fd;

error_set:
	put_filp(filp);
	put_unused_fd(fd);
error_file:
	/*
	 * calls the right *_put() functions
	 * calls pfm_release_pmu()
	 */
	pfm_context_free(ctx);
	return ret;
error_alloc:
	pfm_pmu_conf_put();
error_conf:
	pfm_smpl_fmt_put(fmt);
	return ret;
}
