/*
 * perfmon_cxtsw.c: perfmon2 context switch code
 *
 * This file implements the perfmon2 interface which
 * provides access to the hardware performance counters
 * of the host processor.
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
 * 	http://perfmon2.sf.net
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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/perfmon.h>

/*
 * used only in UP mode
 */
void pfm_save_prev_context(struct pfm_context *ctxp)
{
	struct pfm_event_set *set;

	/*
	 * in UP per-thread, due to lazy save
	 * there could be a context from another
	 * task. We need to push it first before
	 * installing our new state
	 */
	set = ctxp->active_set;
	pfm_save_pmds(ctxp, set);
	/*
	 * do not clear ownership because we rewrite
	 * right away
	 */
}

void pfm_save_pmds(struct pfm_context *ctx, struct pfm_event_set *set)
{
	u64 val, ovfl_mask;
	u64 *used_pmds, *cnt_pmds;
	u16 i, num;

	ovfl_mask = pfm_pmu_conf->ovfl_mask;
	num = set->nused_pmds;
	cnt_pmds = pfm_pmu_conf->regs.cnt_pmds;
	used_pmds = set->used_pmds;

	/*
	 * save HW PMD, for counters, reconstruct 64-bit value
	 */
	for (i = 0; num; i++) {
		if (test_bit(i, cast_ulp(used_pmds))) {
			val = pfm_read_pmd(ctx, i);
			if (likely(test_bit(i, cast_ulp(cnt_pmds))))
				val = (set->pmds[i].value & ~ovfl_mask) |
					(val & ovfl_mask);
			set->pmds[i].value = val;
			num--;
		}
	}
}

/*
 * interrupts are  disabled (no preemption)
 */
static void __pfm_ctxswin_thread(struct task_struct *task,
				 struct pfm_context *ctx, u64 now)
{
	u64 cur_act;
	struct pfm_event_set *set;
	int reload_pmcs, reload_pmds;
	int mycpu, is_active;

	mycpu = smp_processor_id();

	cur_act = __get_cpu_var(pmu_activation_number);
	/*
	 * we need to lock context because it could be accessed
	 * from another CPU
	 */
	spin_lock(&ctx->lock);

	is_active = pfm_arch_is_active(ctx);

	set = ctx->active_set;

	/*
	 * in case fo zombie, we do not complete ctswin of the
	 * PMU, and we force a call to pfm_handle_work() to finish
	 * cleanup, i.e., free context + smpl_buff. The reason for
	 * deferring to pfm_handle_work() is that it is not possible
	 * to vfree() with interrupts disabled.
	 */
	if (unlikely(ctx->state == PFM_CTX_ZOMBIE)) {
		ctx->flags.work_type = PFM_WORK_ZOMBIE;
		set_tsk_thread_flag(task, TIF_PERFMON_WORK);
		pfm_arch_arm_handle_work(task);
		spin_unlock(&ctx->lock);
		return;
	}

	/*
	 * if we were the last user of the PMU on that CPU,
	 * then nothing to do except restore psr
	 */
	if (ctx->last_cpu == mycpu && ctx->last_act == cur_act) {
		/*
		 * check for forced reload conditions
		 */
		reload_pmcs = set->priv_flags & PFM_SETFL_PRIV_MOD_PMCS;
		reload_pmds = set->priv_flags & PFM_SETFL_PRIV_MOD_PMDS;
	} else {
#ifndef CONFIG_SMP
		struct pfm_context *ctxp;
		ctxp = __get_cpu_var(pmu_ctx);
		if (ctxp)
			pfm_save_prev_context(ctxp);
#endif
		reload_pmcs = 1;
		reload_pmds = 1;
	}
	/* consumed */
	set->priv_flags &= ~PFM_SETFL_PRIV_MOD_BOTH;

	if (reload_pmds)
		pfm_arch_restore_pmds(ctx, set);

	/*
	 * need to check if had in-flight interrupt in
	 * pfm_ctxswout_thread(). If at least one bit set, then we must replay
	 * the interrupt to avoid losing some important performance data.
	 *
	 * npend_ovfls is cleared in interrupt handler
	 */
	if (set->npend_ovfls) {
		pfm_arch_resend_irq();
		pfm_stats_inc(ovfl_intr_replay_count);
	}

	if (reload_pmcs)
		pfm_arch_restore_pmcs(ctx, set);

	/*
	 * record current activation for this context
	 */
	pfm_inc_activation();
	pfm_set_last_cpu(ctx, mycpu);
	pfm_set_activation(ctx);

	/*
	 * establish new ownership.
	 */
	pfm_set_pmu_owner(task, ctx);

	pfm_arch_ctxswin_thread(task, ctx, set);
	/*
	 * set->duration does not count when context in MASKED state.
	 * set->duration_start is reset in unmask_monitoring()
	 */
	set->duration_start = now;

	/*
	 * re-arm switch timeout, if necessary
	 * Timeout is active only if monitoring is active,
	 * i.e., LOADED + started
	 *
	 * We reload the remainder timeout or the full timeout.
	 * Remainder is recorded on context switch out or in
	 * pfm_load_context()
	 */
	if (ctx->state == PFM_CTX_LOADED
	    && (set->flags & PFM_SETFL_TIME_SWITCH) && is_active) {
		struct hrtimer *h;
		h = &__get_cpu_var(pfm_hrtimer);
		PFM_DBG_ovfl("hrtimer=%lld", (long long)set->hrtimer_rem.tv64);
		hrtimer_start(h, set->hrtimer_rem, HRTIMER_MODE_REL);
		/*
		 * timer was not re-armed because it has already expired
		 * timer was not enqueued, we need to switch set now
		 */
		if (!hrtimer_is_queued(h)) {
			int ret;
			ret = pfm_stats_inc(set_switch_exp);
			pfm_switch_sets(ctx, NULL, 1, 0);
			set = ctx->active_set;
			if (ret == HRTIMER_RESTART)
				hrtimer_start(h, set->hrtimer_rem, HRTIMER_MODE_REL);
		}
	}
	spin_unlock(&ctx->lock);
}

/*
 * interrupts are masked, runqueue lock is held.
 *
 * In UP. we simply stop monitoring and leave the state
 * in place, i.e., lazy save
 */
static void __pfm_ctxswout_thread(struct task_struct *task,
				  struct pfm_context *ctx, u64 now)
{
	struct pfm_event_set *set;
	int need_save_pmds, is_active;

	/*
	 * we need to lock context because it could be accessed
	 * from another CPU
	 */
	spin_lock(&ctx->lock);

	is_active = pfm_arch_is_active(ctx);
	set = ctx->active_set;

	/*
	 * stop monitoring and
	 * collect pending overflow information
	 * needed on ctxswin. We cannot afford to lose
	 * a PMU interrupt.
	 */
	need_save_pmds = pfm_arch_ctxswout_thread(task, ctx, set);

	if (ctx->state == PFM_CTX_LOADED) {
		/*
	 	 * accumulate only when set is actively monitoring,
	 	 */
		set->duration += now - set->duration_start;

		/*
		 * record remaining timeout
		 * reload in pfm_ctxsw_in()
		 */
		if (is_active && (set->flags & PFM_SETFL_TIME_SWITCH)) {
			struct hrtimer *h = NULL;
			h = &__get_cpu_var(pfm_hrtimer);
			hrtimer_cancel(h);
			set->hrtimer_rem = hrtimer_get_remaining(h);
			PFM_DBG_ovfl("hrtimer=%lld",
				     (long long)set->hrtimer_rem.tv64);
		}
	}

#ifdef CONFIG_SMP
	/*
	 * in SMP, release ownership of this PMU.
	 * PMU interrupts are masked, so nothing
	 * can happen.
	 */
	pfm_set_pmu_owner(NULL, NULL);

	/*
	 * On some architectures, it is necessary to read the
	 * PMD registers to check for pending overflow in
	 * pfm_arch_ctxswout_thread(). In that case, saving of
	 * the PMDs  may be  done there and not here.
	 */
	if (need_save_pmds)
		pfm_save_pmds(ctx, set);
#endif
	spin_unlock(&ctx->lock);
}

/*
 * no need to lock the context. To operate on a system-wide
 * context, the task has to run on the monitored CPU. In the
 * case of close issued on another CPU, an IPI is sent but
 * this routine runs with interrupts masked, so we are
 * protected
 *
 * On some architectures, such as IA-64, it may be necessary
 * to intervene during the context even in system-wide mode
 * to modify some machine state.
 */
static void __pfm_ctxsw_sys(struct task_struct *prev,
			    struct task_struct *next)
{
	struct pfm_context *ctx;
	struct pfm_event_set *set;

	ctx = __get_cpu_var(pmu_ctx);
	if (!ctx) {
		pr_info("prev=%d tif=%d ctx=%p next=%d tif=%d ctx=%p\n",
			prev->pid,
			test_tsk_thread_flag(prev, TIF_PERFMON_CTXSW),
			prev->pfm_context,
			next->pid,
			test_tsk_thread_flag(next, TIF_PERFMON_CTXSW),
			next->pfm_context);
		BUG_ON(!ctx);
	}

	set = ctx->active_set;

	/*
	 * propagate TIF_PERFMON_CTXSW to ensure that:
	 * - previous task has TIF_PERFMON_CTXSW cleared, in case it is
	 *   scheduled onto another CPU where there is syswide monitoring
	 * - next task has TIF_PERFMON_CTXSW set to ensure it will come back
	 *   here when context switched out
	 */
	clear_tsk_thread_flag(prev, TIF_PERFMON_CTXSW);
	set_tsk_thread_flag(next, TIF_PERFMON_CTXSW);

	/*
	 * nothing to do until actually started
	 * XXX: assumes no mean to start from user level
	 */
	if (!ctx->flags.started)
		return;

	pfm_arch_ctxswout_sys(prev, ctx, set);
	pfm_arch_ctxswin_sys(next, ctx, set);
}

/*
 * come here when either prev or next has TIF_PERFMON_CTXSW flag set
 * Note that this is not because a task has TIF_PERFMON_CTXSW set that
 * it has a context attached, e.g., in system-wide on certain arch.
 */
void pfm_ctxsw(struct task_struct *prev, struct task_struct *next)
{
	struct pfm_context *ctxp, *ctxn;
	u64 now;

	now = sched_clock();

	ctxp = NULL;
	ctxn = NULL;
	if (prev)
		ctxp = prev->pfm_context;
	if (next)
		ctxn = next->pfm_context;

	if (ctxp)
		__pfm_ctxswout_thread(prev, ctxp, now);

	if (ctxn)
		__pfm_ctxswin_thread(next, ctxn, now);

	/*
	 * given that prev and next can never be the same, this
	 * test is checking that ctxp == ctxn == NULL which is
	 * an indication we have an active system-wide session on
	 * this CPU that needs ctxsw intervention. Not all processors
	 * needs this, IA64 is one.
	 */
	if (ctxp == ctxn)
		__pfm_ctxsw_sys(prev, next);

	pfm_stats_inc(ctxsw_count);
	pfm_stats_add(ctxsw_ns, sched_clock() - now);
}
EXPORT_SYMBOL_GPL(pfm_ctxsw);
