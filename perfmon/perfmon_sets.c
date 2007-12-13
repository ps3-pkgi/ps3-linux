/*
 * perfmon_sets.c: perfmon2 event sets and multiplexing functions
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

static struct kmem_cache	*pfm_set_cachep;

/*
 * reload reference interrupt switch thresholds for PMD which
 * can interrupt
 */
static void pfm_reload_switch_thresholds(struct pfm_event_set *set)
{
	u64 *used_pmds;
	u16 i, max, first;

	used_pmds = set->used_pmds;
	first = pfm_pmu_conf->regs.first_intr_pmd;
	max = pfm_pmu_conf->regs.max_intr_pmd;

	for (i = first; i < max; i++) {
		if (test_bit(i, cast_ulp(used_pmds))) {
			set->pmds[i].ovflsw_thres = set->pmds[i].ovflsw_ref_thres;

			PFM_DBG("set%u pmd%u ovflsw_thres=%llu",
				set->id,
				i,
				(unsigned long long)set->pmds[i].ovflsw_thres);
		}
	}
}

/*
 * connect all sets, reset internal fields
 */
int pfm_prepare_sets(struct pfm_context *ctx, struct pfm_event_set *act_set)
{
	struct pfm_event_set *set;
	u16 max;

	max = pfm_pmu_conf->regs.max_intr_pmd;

	list_for_each_entry(set, &ctx->set_list, list) {
		/*
		 * cleanup bitvectors
		 */
		bitmap_zero(cast_ulp(set->ovfl_pmds), max);
		bitmap_zero(cast_ulp(set->povfl_pmds), max);

		set->npend_ovfls = 0;
		/*
		 * timer is reset when the context is loaded, so any
		 * remainder timeout is cancelled
		 */
		set->hrtimer_rem.tv64 = 0;

		/*
		 * we cannot just use plain clear because of arch-specific flags
		 */
		set->priv_flags &= ~(PFM_SETFL_PRIV_MOD_BOTH|PFM_SETFL_PRIV_SWITCH);
		/*
		 * neither duration nor runs are reset because typically loading/unloading
		 * does not mean counts are reset. To reset, the set must be modified
		 */
	}

	if (act_set->flags & PFM_SETFL_OVFL_SWITCH)
		pfm_reload_switch_thresholds(act_set);

	return 0;
}

/*
 * called by run_hrtimer_softirq()
 */
enum hrtimer_restart pfm_handle_switch_timeout(struct hrtimer *t)
{
	struct pfm_event_set *set;
	struct pfm_context *ctx;
	unsigned long flags;
	enum hrtimer_restart ret = HRTIMER_NORESTART;

	/*
	 * prevent against race with unload
	 */
	ctx  = __get_cpu_var(pmu_ctx);
	if (!ctx)
		return HRTIMER_NORESTART;

	spin_lock_irqsave(&ctx->lock, flags);

	set = ctx->active_set;

	/*
	 * switching occurs only when context is attached
	 */
	switch(ctx->state) {
		case PFM_CTX_LOADED:
			break;
		case PFM_CTX_ZOMBIE:
		case PFM_CTX_UNLOADED:
			goto done;
		case PFM_CTX_MASKED:
			/*
			 * no switching occurs while masked
			 * but timeout is re-armed
			 */
			ret = HRTIMER_RESTART;
			goto done;
	}
	BUG_ON(ctx->flags.system && ctx->cpu != smp_processor_id());

	/*
	 * timer does not run while monitoring is inactive (not started)
	 */
	if (!pfm_arch_is_active(ctx))
		goto done;

	pfm_stats_inc(handle_timeout_count);

	ret  = pfm_switch_sets(ctx, NULL, PFM_PMD_RESET_SHORT, 0);
done:
	spin_unlock_irqrestore(&ctx->lock, flags);
	return ret;
}

/*
 *
 * always operating on the current task
 * interrupts are masked
 *
 * input:
 * 	- new_set: new set to switch to, if NULL follow normal chain
 */
enum hrtimer_restart pfm_switch_sets(struct pfm_context *ctx,
				     struct pfm_event_set *new_set,
				     int reset_mode,
				     int no_restart)
{
	struct pfm_event_set *set;
	u64 now, end;
	u32 new_flags;
	int is_system, is_active, nn;
	enum hrtimer_restart ret = HRTIMER_NORESTART;

	now = sched_clock();
	set = ctx->active_set;
	is_active = pfm_arch_is_active(ctx);

	/*
	 * if no set is explicitly requested,
	 * use the set_switch_next field
	 */
	if (!new_set) {
		/*
		 * we use round-robin unless the user specified
		 * a particular set to go to.
		 */
		new_set = list_first_entry(&set->list, struct pfm_event_set, list);
		if (&new_set->list == &ctx->set_list)
			new_set = list_first_entry(&ctx->set_list, struct pfm_event_set, list);
	}

	PFM_DBG_ovfl("state=%d act=%d cur_set=%u cur_runs=%llu cur_npend=%d next_set=%u "
		  "next_runs=%llu new_npend=%d reset_mode=%d reset_pmds=%llx",
		  ctx->state,
		  is_active,
		  set->id,
		  (unsigned long long)set->runs,
		  set->npend_ovfls,
		  new_set->id,
		  (unsigned long long)new_set->runs,
		  new_set->npend_ovfls,
		  reset_mode,
		  (unsigned long long)new_set->reset_pmds[0]);

	is_system = ctx->flags.system;
	new_flags = new_set->flags;

	/*
	 * nothing more to do
	 */
	if (new_set == set)
		goto skip_same_set;

	if (is_active) {
		pfm_arch_stop(current, ctx, set);
		pfm_save_pmds(ctx, set);
		/*
		 * compute elapsed ns for active set
		 */
		set->duration += now - set->duration_start;
	}

	pfm_arch_restore_pmds(ctx, new_set);
	/*
	 * if masked, we must restore the pmcs such that they
	 * do not capture anything.
	 */
	pfm_arch_restore_pmcs(ctx, new_set);

	if (new_set->npend_ovfls) {
		pfm_arch_resend_irq();
		pfm_stats_inc(ovfl_intr_replay_count);
	}

	new_set->priv_flags &= ~PFM_SETFL_PRIV_MOD_BOTH;

skip_same_set:
	new_set->runs++;
	/*
	 * reset switch threshold
	 */
	if (new_flags & PFM_SETFL_OVFL_SWITCH)
		pfm_reload_switch_thresholds(new_set);

	/*
	 * reset overflowed PMD registers
	 */
	nn = bitmap_weight(cast_ulp(new_set->reset_pmds),
			   pfm_pmu_conf->regs.max_pmd);
	if (nn)
		pfm_reset_pmds(ctx, new_set, nn, reset_mode);
	/*
	 * this is needed when coming from pfm_start()
	 */
	if (no_restart)
		goto skip_restart;

	if (is_active) {
		pfm_arch_start(current, ctx, new_set);
		new_set->duration_start = now;

		/*
		 * install new timeout if necessary
		 */
		if (new_flags & PFM_SETFL_TIME_SWITCH) {
			struct hrtimer *h;
			h = &__get_cpu_var(pfm_hrtimer);
			hrtimer_forward(h, h->base->get_time(), new_set->hrtimer_exp);
			ret = HRTIMER_RESTART;
		}
	}

skip_restart:
	ctx->active_set = new_set;

	end = sched_clock();

	pfm_stats_inc(set_switch_count);
	pfm_stats_add(set_switch_ns, end - now);

	return ret;
}

/*
 * called from __pfm_overflow_handler() to switch event sets.
 * monitoring is stopped, task is current, interrupts are masked.
 * compared to pfm_switch_sets(), this version is simplified because
 * it knows about the call path. There is no need to stop monitoring
 * because it is already frozen by PMU handler.
 */
void pfm_switch_sets_from_intr(struct pfm_context *ctx)
{
	struct pfm_event_set *set, *new_set;
	u64 now, end;
	u32 new_flags;
	int is_system, n;

	now = sched_clock();
	set = ctx->active_set;
	new_set = list_first_entry(&set->list, struct pfm_event_set, list);
	if (&new_set->list == &ctx->set_list)
		new_set = list_first_entry(&ctx->set_list, struct pfm_event_set, list);

	PFM_DBG_ovfl("state=%d cur_set=%u cur_runs=%llu cur_npend=%d next_set=%u "
		  "next_runs=%llu new_npend=%d new_r_pmds=%llx",
		  ctx->state,
		  set->id,
		  (unsigned long long)set->runs,
		  set->npend_ovfls,
		  new_set->id,
		  (unsigned long long)new_set->runs,
		  new_set->npend_ovfls,
		  (unsigned long long)new_set->reset_pmds[0]);

	is_system = ctx->flags.system;
	new_flags = new_set->flags;

	/*
	 * nothing more to do
	 */
	if (new_set == set)
		goto skip_same_set;

	if (set->flags & PFM_SETFL_TIME_SWITCH) {
		hrtimer_cancel(&__get_cpu_var(pfm_hrtimer));
		PFM_DBG_ovfl("cancelled timer for set%u", set->id);
	}

	/*
	 * when called from PMU intr handler, monitoring
	 * is already stopped
	 *
	 * save current PMD registers, we use a special
	 * form for performance reason. On some architectures,
	 * such as x86, the pmds are already saved when entering
	 * the PMU interrupt handler via pfm-arch_intr_freeze()
	 * so we don't need to save them again. On the contrary,
	 * on IA-64, they are not saved by freeze, thus we have to
	 * to it here.
	 */
	pfm_arch_save_pmds_from_intr(ctx, set);

	/*
	 * compute elapsed ns for active set
	 */
	set->duration += now - set->duration_start;

	pfm_arch_restore_pmds(ctx, new_set);

	/*
	 * must not be restored active as we are still executing in the
	 * PMU interrupt handler. activation is deferred to unfreeze PMU
	 */
	pfm_arch_restore_pmcs(ctx, new_set);

	/*
	 * check for pending interrupt on incoming set.
	 * interrupts are masked so handler call deferred
	 */
	if (new_set->npend_ovfls) {
		pfm_arch_resend_irq();
		pfm_stats_inc(ovfl_intr_replay_count);
	}
	/*
	 * no need to restore anything, that is already done
	 */
	new_set->priv_flags &= ~PFM_SETFL_PRIV_MOD_BOTH;
	/*
	 * reset duration counter
	 */
	new_set->duration_start = now;

skip_same_set:
	new_set->runs++;

	/*
	 * reset switch threshold
	 */
	if (new_flags & PFM_SETFL_OVFL_SWITCH)
		pfm_reload_switch_thresholds(new_set);

	/*
	 * reset overflowed PMD registers
	 */
	n = bitmap_weight(cast_ulp(new_set->reset_pmds), pfm_pmu_conf->regs.max_pmd);
	if (n)
		pfm_reset_pmds(ctx, new_set, n, PFM_PMD_RESET_SHORT);

	/*
	 * XXX: isactive?
	 *
	 * Came here following a interrupt which triggered a switch, i.e.,
	 * previous set was using OVFL_SWITCH, thus we just need to arm
	 * check if the next set is using timeout, and if so arm the timer.
	 */
	if (new_flags & PFM_SETFL_TIME_SWITCH) {
		hrtimer_start(&__get_cpu_var(pfm_hrtimer), set->hrtimer_exp, HRTIMER_MODE_REL);
		PFM_DBG("armed new timeout for set%u", new_set->id);
	}

	ctx->active_set = new_set;

	end = sched_clock();

	pfm_stats_inc(set_switch_count);
	pfm_stats_add(set_switch_ns, end - now);
}


static int pfm_setfl_sane(struct pfm_context *ctx, u32 flags)
{
#define PFM_SETFL_BOTH_SWITCH	(PFM_SETFL_OVFL_SWITCH|PFM_SETFL_TIME_SWITCH)
	int ret;

	ret = pfm_arch_setfl_sane(ctx, flags);
	if (ret)
		return ret;

	if ((flags & PFM_SETFL_BOTH_SWITCH) == PFM_SETFL_BOTH_SWITCH) {
		PFM_DBG("both switch ovfl and switch time are set");
		return -EINVAL;
	}
	return 0;
}

/*
 * it is never possible to change the identification of an existing set
 */
static int __pfm_change_evtset(struct pfm_context *ctx,
			       struct pfm_event_set *set,
			       struct pfarg_setdesc *req)
{
	struct timeval tv;
	struct timespec ts;
	ktime_t kt;
	long d, rem, res_ns;
	u32 flags;
	int ret;
	u16 set_id;

	BUG_ON(ctx->state == PFM_CTX_LOADED);

	set_id = req->set_id;
	flags = req->set_flags;

	ret = pfm_setfl_sane(ctx, flags);
	if (ret) {
		PFM_DBG("invalid flags 0x%x set %u", flags, set_id);
		return -EINVAL;
	}

	hrtimer_get_res(CLOCK_MONOTONIC, &ts);
	res_ns = (long)ktime_to_ns(timespec_to_ktime(ts));
	PFM_DBG("clock_res=%ldns", res_ns);

	/*
	 * round-up to multiple of clock resolution
	 * timeout = ((req->set_timeout+res_ns-1)/res_ns)*res_ns;
	 *
	 * u64 division missing on 32-bit arch, so use div_long_long
	 */
	d = div_long_long_rem_signed(req->set_timeout, res_ns, &rem);

	PFM_DBG("set%u flags=0x%x req_timeout=%lluns "
		"HZ=%u TICK_NSEC=%lu eff_timeout=%luns",
		set_id,
		flags,
		(unsigned long long)req->set_timeout,
		HZ, TICK_NSEC,
		d * res_ns);

	/*
	 * Only accept timeout, we can actually achieve.
	 * users can invoke clock_getres(CLOCK_MONOTONIC)
	 * to figure out resolution and adjust timeout
	 */
	if (rem) {
		PFM_DBG("set%u invalid timeout=%llu",
			set_id,
			(unsigned long long)req->set_timeout);
		return -EINVAL;
	}

	tv = ns_to_timeval(req->set_timeout);
	kt = timeval_to_ktime(tv);
	set->hrtimer_exp = kt;

	/*
	 * commit changes
	 */
	set->id = set_id;
	set->flags = flags;
	set->priv_flags = 0;

	/*
	 * activation and duration counters are reset as
	 * most likely major things will change in the set
	 */
	set->runs = 0;
	set->duration = 0;

	return 0;
}

/*
 * this function does not modify the next field
 */
void pfm_init_evtset(struct pfm_event_set *set)
{
	u64 *impl_pmcs;
	u16 i, max_pmc;

	max_pmc = pfm_pmu_conf->regs.max_pmc;
	impl_pmcs =  pfm_pmu_conf->regs.pmcs;

	/*
	 * install default values for all PMC  registers
	 */
	for (i=0; i < max_pmc;  i++) {
		if (test_bit(i, cast_ulp(impl_pmcs))) {
			set->pmcs[i] = pfm_pmu_conf->pmc_desc[i].dfl_val;
			PFM_DBG("set%u pmc%u=0x%llx",
				set->id,
				i,
				(unsigned long long)set->pmcs[i]);
		}
	}

	/*
	 * PMD registers are set to 0 when the event set is allocated,
	 * hence we do not need to explicitly initialize them.
	 *
	 * For virtual PMD registers (i.e., those tied to a SW resource)
	 * their value becomes meaningful once the context is attached.
	 */
}

/*
 * look for an event set using its identification. If the set does not
 * exist:
 * 	- if alloc == 0 then return error
 * 	- if alloc == 1  then allocate set
 *
 * alloc is one ONLY when coming from pfm_create_evtsets() which can only
 * be called when the context is detached, i.e. monitoring is stopped.
 */
struct pfm_event_set *pfm_find_set(struct pfm_context *ctx, u16 set_id, int alloc)
{
	struct pfm_event_set *set = NULL, *prev, *new_set;

	PFM_DBG("looking for set=%u", set_id);

	prev = NULL;
	list_for_each_entry(set, &ctx->set_list, list) {
		if (set->id == set_id)
			return set;
		if (set->id > set_id)
			break;
		prev = set;
	}

	if (!alloc)
		return NULL;

	/*
	 * we are holding the context spinlock and interrupts
	 * are unmasked. We must use GFP_ATOMIC as we cannot
	 * sleep while holding a spin lock.
	 */
	new_set = kmem_cache_zalloc(pfm_set_cachep, GFP_ATOMIC);
	if (!new_set)
		return NULL;

	new_set->id = set_id;

	INIT_LIST_HEAD(&new_set->list);

	if (prev == NULL) {
		list_add(&(new_set->list), &ctx->set_list);
	} else {
		PFM_DBG("add after set=%u", prev->id);
		list_add(&(new_set->list), &prev->list);
	}
	return new_set;
}

/*
 * context is unloaded for this command. Interrupts are enabled
 */
int __pfm_create_evtsets(struct pfm_context *ctx, struct pfarg_setdesc *req,
			int count)
{
	struct pfm_event_set *set;
	u16 set_id;
	int i, ret;

	for (i = 0; i < count; i++, req++) {
		set_id = req->set_id;

		PFM_DBG("set_id=%u", set_id);

		set = pfm_find_set(ctx, set_id, 1);
		if (set == NULL)
			goto error_mem;

		ret = __pfm_change_evtset(ctx, set, req);
		if (ret)
			goto error_params;

		pfm_init_evtset(set);
	}
	return 0;
error_mem:
	PFM_DBG("cannot allocate set %u", set_id);
	return -ENOMEM;
error_params:
	return ret;
}

int __pfm_getinfo_evtsets(struct pfm_context *ctx, struct pfarg_setinfo *req,
				 int count)
{
	struct pfm_event_set *set;
	int i, is_system, is_loaded, ret;
	u16 set_id, max, max_pmc, max_pmd;
	u64 end;

	end = sched_clock();

	is_system = ctx->flags.system;
	is_loaded = ctx->state == PFM_CTX_LOADED;

	max = pfm_pmu_conf->regs.max_intr_pmd;
	max_pmc  = pfm_pmu_conf->regs.max_pmc;
	max_pmd = pfm_pmu_conf->regs.max_pmd;

	ret = -EINVAL;
	for (i = 0; i < count; i++, req++) {

		set_id = req->set_id;

		list_for_each_entry(set, &ctx->set_list, list) {
			if (set->id == set_id)
				goto found;
			if (set->id > set_id)
				goto error;
		}
found:
		/*
		 * compute leftover timeout
		 */
		req->set_flags = set->flags;

		/*
		 * XXX: fixme
		 */
		req->set_timeout = 0;

		req->set_runs = set->runs;
		req->set_act_duration = set->duration;

		/*
		 * adjust for active set if needed
		 */
		if (is_system && is_loaded && ctx->flags.started
		    && set == ctx->active_set)
			req->set_act_duration  += end - set->duration_start;

		/*
		 * copy the list of pmds which last overflowed for
		 * the set
		 */
		bitmap_copy(cast_ulp(req->set_ovfl_pmds),
			    cast_ulp(set->ovfl_pmds),
			    max);

		/*
		 * copy bitmask of available PMU registers
		 */
		bitmap_copy(cast_ulp(req->set_avail_pmcs),
			    cast_ulp(pfm_pmu_conf->regs.pmcs),
			    max_pmc);

		bitmap_copy(cast_ulp(req->set_avail_pmds),
			    cast_ulp(pfm_pmu_conf->regs.pmds),
			    max_pmd);

		PFM_DBG("set%u flags=0x%x eff_usec=%llu runs=%llu "
			"a_pmcs=0x%llx a_pmds=0x%llx",
			set_id,
			set->flags,
			(unsigned long long)req->set_timeout,
			(unsigned long long)set->runs,
			(unsigned long long)pfm_pmu_conf->regs.pmcs[0],
			(unsigned long long)pfm_pmu_conf->regs.pmds[0]);
	}
	ret = 0;
error:
	return ret;
}

/*
 * context is unloaded for this command. Interrupts are enabled
 */
int __pfm_delete_evtsets(struct pfm_context *ctx, void *arg, int count)
{
	struct pfarg_setdesc *req = arg;
	struct pfm_event_set *set;
	u16 set_id;
	int i, ret;

	ret = -EINVAL;
	for (i = 0; i < count; i++, req++) {
		set_id = req->set_id;

		list_for_each_entry(set, &ctx->set_list, list) {
			if (set->id == set_id)
				goto found;
			if (set->id > set_id)
				goto error;
		}
		goto error;
found:
		/*
		 * clear active set if necessary.
		 * will be updated when context is loaded
		 */
		if (set == ctx->active_set)
			ctx->active_set = NULL;

		list_del(&set->list);

		kmem_cache_free(pfm_set_cachep, set);

		PFM_DBG("set%u deleted", set_id);
	}
	ret = 0;
error:
	return ret;
}

/*
 * called from pfm_context_free() to free all sets
 */
void pfm_free_sets(struct pfm_context *ctx)
{
	struct pfm_event_set *set, *tmp;

	list_for_each_entry_safe(set, tmp, &ctx->set_list, list) {
		list_del(&set->list);
		kmem_cache_free(pfm_set_cachep, set);
	}
}

int pfm_sets_init(void)
{

	pfm_set_cachep = kmem_cache_create("pfm_event_set",
					   sizeof(struct pfm_event_set),
					   SLAB_HWCACHE_ALIGN, 0, NULL);
	if (pfm_set_cachep == NULL) {
		PFM_ERR("cannot initialize event set slab");
		return -ENOMEM;
	}
	return 0;
}
