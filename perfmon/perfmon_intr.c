/*
 * perfmon_intr.c: perfmon2 interrupt handling
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
#include <linux/perfmon.h>
#include <linux/module.h>

static inline void pfm_mask_monitoring(struct pfm_context *ctx,
				       struct pfm_event_set *set)
{
	u64 now;


	now = sched_clock();

	/*
	 * we save the PMD values such that we can read them while
	 * MASKED without having the thread stopped
	 * because monitoring is stopped
	 *
	 * pfm_save_pmds() could be avoided if we knew
	 * that pfm_arch_intr_freeze() had saved them already
	 */
	pfm_save_pmds(ctx, set);
	pfm_arch_mask_monitoring(ctx, set);
	/*
	 * accumulate the set duration up to this point
	 */
	set->duration += now - set->duration_start;

	ctx->state = PFM_CTX_MASKED;

	/*
	 * need to stop timer and remember remaining time
	 * will be reloaded in pfm_unmask_monitoring
	 * hrtimer is cancelled in the tail of the interrupt
	 * handler once the context is unlocked
	 */
	if (set->flags & PFM_SETFL_TIME_SWITCH) {
		struct hrtimer *h = &__get_cpu_var(pfm_hrtimer);
		hrtimer_cancel(h);
		set->hrtimer_rem = hrtimer_get_remaining(h);
	}
	PFM_DBG_ovfl("can_restart=%u", ctx->flags.can_restart);
}

/*
 * main overflow processing routine.
 *
 * set->num_ovfl_pmds is 0 when returning from this function even though
 * set->ovfl_pmds[] may have bits set. When leaving set->num_ovfl_pmds
 * must never be used to determine if there was a pending overflow.
 */
static void pfm_overflow_handler(struct pfm_context *ctx, struct pfm_event_set *set,
				 unsigned long ip,
				 struct pt_regs *regs)
{
	struct pfm_ovfl_arg *ovfl_arg;
	struct pfm_event_set *set_orig;
	void *hdr;
	u64 old_val, ovfl_mask, new_val, ovfl_thres;
	u64 *ovfl_notify, *ovfl_pmds, *pend_ovfls;
	u64 *smpl_pmds, *reset_pmds, *cnt_pmds;
	u64 now, t0, t1;
	u32 ovfl_ctrl, num_ovfl, num_ovfl_orig;
	u16 i, max_pmd, max_intr, first_intr;
	u8 must_switch, has_64b_ovfl;
	u8 ctx_block, has_notify, has_ovfl_sw;

	now = t0 = sched_clock();

	ovfl_mask = pfm_pmu_conf->ovfl_mask;

	max_pmd = pfm_pmu_conf->regs.max_pmd;
	first_intr = pfm_pmu_conf->regs.first_intr_pmd;
	max_intr = pfm_pmu_conf->regs.max_intr_pmd;
	cnt_pmds = pfm_pmu_conf->regs.cnt_pmds;

	ovfl_pmds = set->ovfl_pmds;
	num_ovfl = num_ovfl_orig = set->npend_ovfls;
	pend_ovfls = set->povfl_pmds;
	has_ovfl_sw = set->flags & PFM_SETFL_OVFL_SWITCH;
	set_orig = set;

	if (unlikely(ctx->state == PFM_CTX_ZOMBIE))
		goto stop_monitoring;

	must_switch = has_64b_ovfl = 0;

	hdr = ctx->smpl_addr;

	PFM_DBG_ovfl("intr_pmds=0x%llx npend=%u ip=%p, blocking=%d "
		     "u_pmds=0x%llx use_fmt=%u",
		     (unsigned long long)pend_ovfls[0],
		     num_ovfl,
		     (void *)ip,
		     ctx->flags.block,
		     (unsigned long long)set->used_pmds[0],
		     ctx->smpl_fmt != NULL);

	/*
	 * initialize temporary bitvectors
	 * we allocate bitvectors in the context
	 * rather than on the stack to minimize stack
	 * space consumption. PMU interrupt is very high
	 * which implies possible deep nesting of interrupt
	 * hence limited kernel stack space.
	 *
	 * This is safe because a context can only be in the
	 * overflow handler once at a time
	 */
	reset_pmds = set->reset_pmds;
	ovfl_notify = ctx->ovfl_ovfl_notify;

	bitmap_zero(cast_ulp(reset_pmds), max_pmd);

	/*
	 * first we update the virtual counters
	 *
	 * we leverage num_ovfl to minimize number of
	 * iterations of the loop.
	 *
	 * The i < max_intr is just a sanity check
	 */
	for (i = first_intr; num_ovfl && i < max_intr ; i++) {
		/*
		 * skip pmd which did not overflow
		 */
		if (!test_bit(i, cast_ulp(pend_ovfls)))
			continue;

		num_ovfl--;

		/*
		 * Update software value for counters ONLY
		 *
		 * Note that the pmd is not necessarily 0 at this point as
		 * qualified events may have happened before the PMU was
		 * frozen. The residual count is not taken into consideration
		 * here but will be with any read of the pmd
		 */
		ovfl_thres = set->pmds[i].ovflsw_thres;

		if (likely(test_bit(i, cast_ulp(cnt_pmds)))) {
			old_val = new_val = set->pmds[i].value;
			new_val += 1 + ovfl_mask;
			set->pmds[i].value = new_val;
		}  else {
			/* for non counter which interrupt, we consider
			 * this equivalent to a 64-bit counter overflow.
			 */
			old_val = 1; new_val = 0;
		}

		/*
		 * check for overflow condition
		 */
		if (likely(old_val > new_val)) {
			has_64b_ovfl = 1;
			if (has_ovfl_sw && ovfl_thres > 0) {
				if (ovfl_thres == 1)
					must_switch = 1;
				set->pmds[i].ovflsw_thres = ovfl_thres - 1;
			}

			/*
			 * what to reset because of this overflow
			 */
			__set_bit(i, cast_ulp(reset_pmds));

			bitmap_or(cast_ulp(reset_pmds),
				  cast_ulp(reset_pmds),
				  cast_ulp(set->pmds[i].reset_pmds),
				  max_pmd);

		} else {
			/*
			 * only keep track of 64-bit overflows or
			 * assimilated
			 */
			__clear_bit(i, cast_ulp(pend_ovfls));

			/*
			 * on some PMU, it may be necessary to re-arm the PMD
			 */
			pfm_arch_ovfl_reset_pmd(ctx, i);
		}

		PFM_DBG_ovfl("ovfl=%s pmd%u new=0x%llx old=0x%llx "
			     "hw_pmd=0x%llx o_pmds=0x%llx must_switch=%u "
			     "o_thres=%llu o_thres_ref=%llu",
			     old_val > new_val ? "64-bit" : "HW",
			     i,
			     (unsigned long long)new_val,
			     (unsigned long long)old_val,
			     (unsigned long long)pfm_read_pmd(ctx, i),
			     (unsigned long long)ovfl_pmds[0],
			     must_switch,
			     (unsigned long long)set->pmds[i].ovflsw_thres,
			     (unsigned long long)set->pmds[i].ovflsw_ref_thres);
	}

	/*
	 * mark the overflows as consumed
	 */
	set->npend_ovfls = 0;

	ctx_block = ctx->flags.block;

	t1 = sched_clock();
	pfm_stats_add(ovfl_intr_p1_ns, t1 - t0);
	t0 = t1;

	/*
	 * there was no 64-bit overflow, nothing else to do
	 */
	if (!has_64b_ovfl)
		return;

	/*
	 * copy pending_ovfls to ovfl_pmd. It is used in
	 * the notification message or getinfo_evtsets().
	 *
	 * pend_ovfls modified to reflect only 64-bit overflows
	 */
	bitmap_copy(cast_ulp(ovfl_pmds),
		    cast_ulp(pend_ovfls),
		    max_intr);

	/*
	 * build ovfl_notify bitmask from ovfl_pmds
	 */
	bitmap_and(cast_ulp(ovfl_notify),
		   cast_ulp(pend_ovfls),
		   cast_ulp(set->ovfl_notify),
		   max_intr);

	has_notify = !bitmap_empty(cast_ulp(ovfl_notify), max_intr);
	/*
	 * must reset for next set of overflows
	 */
	bitmap_zero(cast_ulp(pend_ovfls), max_intr);

	/*
	 * check for format
	 */
	if (likely(ctx->smpl_fmt)) {
		u64 start_cycles, end_cycles;
		u64 *cnt_pmds;
		int j, k, ret = 0;

		ovfl_ctrl = 0;
		num_ovfl = num_ovfl_orig;
		ovfl_arg = &ctx->ovfl_arg;
		cnt_pmds = pfm_pmu_conf->regs.cnt_pmds;

		ovfl_arg->active_set = set->id;

		for (i = first_intr; num_ovfl && !ret; i++) {

			if (!test_bit(i, cast_ulp(ovfl_pmds)))
				continue;

			num_ovfl--;

			ovfl_arg->ovfl_pmd = i;
			ovfl_arg->ovfl_ctrl = 0;

			ovfl_arg->pmd_last_reset = set->pmds[i].lval;
			ovfl_arg->pmd_eventid = set->pmds[i].eventid;

			/*
			 * copy values of pmds of interest.
			 * Sampling format may use them
			 * We do not initialize the unused smpl_pmds_values
			 */
			k = 0;
			smpl_pmds = set->pmds[i].smpl_pmds;
			if (!bitmap_empty(cast_ulp(smpl_pmds), max_pmd)) {

				for (j = 0; j < max_pmd; j++) {

					if (!test_bit(j, cast_ulp(smpl_pmds)))
						continue;

					new_val = pfm_read_pmd(ctx, j);

					/* for counters, build 64-bit value */
					if (test_bit(j, cast_ulp(cnt_pmds))) {
						new_val = (set->pmds[j].value & ~ovfl_mask)
							| (new_val & ovfl_mask);
					}
					ovfl_arg->smpl_pmds_values[k++] = new_val;

					PFM_DBG_ovfl("s_pmd_val[%u]="
						     "pmd%u=0x%llx",
						     k, j,
						     (unsigned long long)new_val);
				}
			}
			ovfl_arg->num_smpl_pmds = k;

			pfm_stats_inc(fmt_handler_calls);

			start_cycles = sched_clock();

			/*
			 * call custom buffer format record (handler) routine
			 */
			ret = (*ctx->smpl_fmt->fmt_handler)(hdr,
							    ovfl_arg,
							    ip,
							    now,
							    regs);

			end_cycles = sched_clock();

			/*
			 * for PFM_OVFL_CTRL_MASK and PFM_OVFL_CTRL_NOTIFY
			 * we take the union
			 *
			 * The reset_pmds mask is constructed automatically
			 * on overflow. When the actual reset takes place
			 * depends on the masking, switch and notification
			 * status. It may be deferred until pfm_restart().
			 */
			ovfl_ctrl |= ovfl_arg->ovfl_ctrl;

			pfm_stats_add(fmt_handler_ns, end_cycles - start_cycles);
		}
		/*
		 * when the format cannot handle the rest of the overflow,
		 * we abort right here
		 */
		if (ret) {
			PFM_DBG_ovfl("handler aborted at PMD%u ret=%d",
				     i, ret);
		}
	} else {
		/*
		 * When no sampling format is used, the default
		 * is:
		 * 	- mask monitoring
		 * 	- notify user if requested
		 *
		 * If notification is not requested, monitoring is masked
		 * and overflowed counters are not reset (saturation).
		 * This mimics the behavior of the default sampling format.
		 */
		ovfl_ctrl = PFM_OVFL_CTRL_NOTIFY;

		if (!must_switch || has_notify)
			ovfl_ctrl |= PFM_OVFL_CTRL_MASK;
	}
	t1 = sched_clock();
	pfm_stats_add(ovfl_intr_p2_ns, t1 - t0);
	t0 = t1;

	PFM_DBG_ovfl("set%u o_notify=0x%llx o_pmds=0x%llx "
		     "r_pmds=0x%llx ovfl_ctrl=0x%x",
		     set->id,
		     (unsigned long long)ovfl_notify[0],
		     (unsigned long long)ovfl_pmds[0],
		     (unsigned long long)reset_pmds[0],
		     ovfl_ctrl);

	/*
	 * we only reset (short reset) when we are not masking. Otherwise
	 * the reset is postponed until restart.
	 */
	if (likely(!(ovfl_ctrl & PFM_OVFL_CTRL_MASK))) {
		if (must_switch) {
			/*
			 * pfm_switch_sets() takes care
			 * of resetting new set if needed
			 */
			pfm_switch_sets_from_intr(ctx);

			/*
			 * update our view of the active set
			 */
			set = ctx->active_set;

			must_switch = 0;
		} else if (ovfl_ctrl & PFM_OVFL_CTRL_RESET) {
			u16 nn;
			t0 = sched_clock();
			nn = bitmap_weight(cast_ulp(reset_pmds), max_pmd);
			if (nn)
				pfm_reset_pmds(ctx, set, nn, PFM_PMD_RESET_SHORT);
		}
		/*
		 * do not block if not masked
		 */
		ctx_block = 0;
	} else {
		pfm_mask_monitoring(ctx, set);
	}
	/*
	 * if we have not switched here, then remember for the
	 * time monitoring is restarted
	 */
	if (must_switch)
		set->priv_flags |= PFM_SETFL_PRIV_SWITCH;

	/*
	 * block only if CTRL_NOTIFY+CTRL_MASK and requested by user
	 *
	 * Defer notification until last operation in the handler
	 * to avoid spinlock contention
	 */
	if (has_notify && (ovfl_ctrl & PFM_OVFL_CTRL_NOTIFY)) {
		int ret;
		if (ctx_block) {
			ctx->flags.work_type = PFM_WORK_BLOCK;
			set_thread_flag(TIF_PERFMON_WORK);
			pfm_arch_arm_handle_work(current);
		}
		/*
		 * Sanity check on the queue.
		 * Should never happen because queue must be sized
		 * appropriatly for format
		 */
		ret = pfm_ovfl_notify_user(ctx, set_orig, ip);
		if (unlikely(ret)) {
			if (ctx->state == PFM_CTX_LOADED)
				pfm_mask_monitoring(ctx, set);
		} else {
			ctx->flags.can_restart++;
			PFM_DBG_ovfl("can_restart=%u", ctx->flags.can_restart);
		}
	}

	t1 = sched_clock();
	pfm_stats_add(ovfl_intr_p3_ns, t1 - t0);

	return;

stop_monitoring:
	/*
	 * Does not happen for a system-wide context nor for a
	 * self-monitored context. We cannot attach to kernel-only
	 * thread, thus it is safe to set TIF bits, i.e., the thread
	 * will eventually leave the kernel or die and either we will
	 * catch the context and clean it up in pfm_handler_work() or
	 * pfm_exit_thread().
	 *
	 * Mask until we get to pfm_handle_work()
	 */
	pfm_mask_monitoring(ctx, set);

	PFM_DBG_ovfl("ctx is zombie, converted to spurious");
	ctx->flags.work_type = PFM_WORK_ZOMBIE;
	set_thread_flag(TIF_PERFMON_WORK);
	pfm_arch_arm_handle_work(current);
}

/*
 * interrupts are masked
 *
 * Context locking necessary to avoid concurrent accesses from other CPUs
 * 	- For per-thread, we must prevent pfm_restart() which works when
 * 	  context is LOADED or MASKED
 */
static void __pfm_interrupt_handler(unsigned long iip, struct pt_regs *regs)
{
	struct task_struct *task;
	struct pfm_context *ctx;
	struct pfm_event_set *set;

	pfm_stats_inc(ovfl_intr_all_count);

	task = __get_cpu_var(pmu_owner);
	ctx = __get_cpu_var(pmu_ctx);

	if (unlikely(ctx == NULL)) {
		PFM_DBG_ovfl("no ctx");
		goto spurious;
	}

	spin_lock(&ctx->lock);

	set = ctx->active_set;

	/*
	 * For SMP per-thread, it is not possible to have
	 * owner != NULL && task != current.
	 *
	 * For UP per-thread, because of lazy save, it
	 * is possible to receive an interrupt in another task
	 * which is not using the PMU. This means
	 * that the interrupt was in-flight at the
	 * time of pfm_ctxswout_thread(). In that
	 * case it will be replayed when the task
	 * is scheduled again. Hence we convert to spurious.
	 *
	 * The basic rule is that an overflow is always
	 * processed in the context of the task that
	 * generated it for all per-thread contexts.
	 *
	 * for system-wide, task is always NULL
	 */
#ifndef CONFIG_SMP
	if (unlikely((task && current->pfm_context != ctx))) {
		PFM_DBG_ovfl("spurious: not owned by current task");
		goto spurious;
	}
#endif
	if (unlikely(!pfm_arch_is_active(ctx))) {
		PFM_DBG_ovfl("spurious: monitoring non active");
		goto spurious;
	}

	/*
	 * freeze PMU and collect overflowed PMD registers
	 * into set->povfl_pmds. Number of overflowed PMDs reported
	 * in set->npend_ovfls
	 */
	pfm_arch_intr_freeze_pmu(ctx, set);
	if (unlikely(!set->npend_ovfls)) {
		PFM_DBG_ovfl("no npend_ovfls");
		goto spurious;
	}

	pfm_stats_inc(ovfl_intr_regular_count);

	pfm_overflow_handler(ctx, set, iip, regs);

	pfm_arch_intr_unfreeze_pmu(ctx);

	spin_unlock(&ctx->lock);

	return;

spurious:
	/* ctx may be NULL */
	pfm_arch_intr_unfreeze_pmu(ctx);
	if (ctx)
		spin_unlock(&ctx->lock);

	pfm_stats_inc(ovfl_intr_spurious_count);
}

void pfm_interrupt_handler(unsigned long iip, struct pt_regs *regs)
{
	u64 start;

	BUG_ON(!irqs_disabled());

	start = sched_clock();

	__pfm_interrupt_handler(iip, regs);

	pfm_stats_add(ovfl_intr_ns, sched_clock() - start);
}
EXPORT_SYMBOL(pfm_interrupt_handler);

