/*
 * This file implements the X86 specific support for the perfmon2 interface
 *
 * Copyright (c) 2005-2007 Hewlett-Packard Development Company, L.P.
 * Contributed by Stephane Eranian <eranian@hpl.hp.com>
 *
 * Copyright (c) 2007 Advanced Micro Devices, Inc.
 * Contributed by Robert Richter <robert.richter@amd.com>
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
#include <linux/interrupt.h>
#include <linux/perfmon.h>
#include <linux/kprobes.h>
#include <linux/kdebug.h>

#include <asm/nmi.h>
#include <asm/apic.h>

DEFINE_PER_CPU(unsigned long, real_iip);
DEFINE_PER_CPU(int, pfm_using_nmi);

struct pfm_ds_area_p4 {
	unsigned long	bts_buf_base;
	unsigned long	bts_index;
	unsigned long	bts_abs_max;
	unsigned long	bts_intr_thres;
	unsigned long	pebs_buf_base;
	unsigned long	pebs_index;
	unsigned long	pebs_abs_max;
	unsigned long	pebs_intr_thres;
	u64		pebs_cnt_reset;
};

struct pfm_ds_area_intel_core {
	u64	bts_buf_base;
	u64	bts_index;
	u64	bts_abs_max;
	u64	bts_intr_thres;
	u64	pebs_buf_base;
	u64	pebs_index;
	u64	pebs_abs_max;
	u64	pebs_intr_thres;
	u64	pebs_cnt_reset;
};


static int (*pfm_has_ovfl)(struct pfm_context *);
static int (*pfm_stop_save)(struct pfm_context *ctx,
			    struct pfm_event_set *set);

static inline int get_smt_id(void)
{
#ifdef CONFIG_SMP
	int cpu = smp_processor_id();
	return (cpu != first_cpu(__get_cpu_var(cpu_sibling_map)));
#else
	return 0;
#endif
}

void __pfm_write_reg_p4(const struct pfm_arch_ext_reg *xreg, u64 val)
{
	u64 pmi;
	int smt_id;

	smt_id = get_smt_id();
	/*
	 * HT is only supported by P4-style PMU
	 *
	 * Adjust for T1 if necessary:
	 *
	 * - move the T0_OS/T0_USR bits into T1 slots
	 * - move the OVF_PMI_T0 bits into T1 slot
	 *
	 * The P4/EM64T T1 is cleared by description table.
	 * User only works with T0.
	 */
	if (smt_id) {
		if (xreg->reg_type & PFM_REGT_ESCR) {

			/* copy T0_USR & T0_OS to T1 */
			val |= ((val & 0xc) >> 2);

			/* clear bits T0_USR & T0_OS */
			val &= ~0xc;

		} else if (xreg->reg_type & PFM_REGT_CCCR) {
			pmi = (val >> 26) & 0x1;
			if (pmi) {
				val &=~(1UL<<26);
				val |= 1UL<<27;
			}
		}
	}
	if (xreg->addrs[smt_id])
		wrmsrl(xreg->addrs[smt_id], val);
}

void __pfm_read_reg_p4(const struct pfm_arch_ext_reg *xreg, u64 *val)
{
	int smt_id;

	smt_id = get_smt_id();

	if (likely(xreg->addrs[smt_id])) {
		rdmsrl(xreg->addrs[smt_id], *val);
		/*
		 * HT is only supported by P4-style PMU
		 *
		 * move the Tx_OS and Tx_USR bits into
		 * T0 slots setting the T1 slots to zero
		 */
		if (xreg->reg_type & PFM_REGT_ESCR) {
			if (smt_id)
				*val |= (((*val) & 0x3) << 2);

			/*
			 * zero out bits that are reserved
			 * (including T1_OS and T1_USR)
			 */
			*val &= PFM_ESCR_RSVD;
		}
	} else {
		*val = 0;
	}
}

/*
 * called from NMI interrupt handler
 */
static void __kprobes __pfm_arch_quiesce_pmu_percpu(void)
{
	struct pfm_arch_pmu_info *arch_info;
	unsigned int i;

	arch_info = pfm_pmu_conf->arch_info;

	/*
	 * quiesce PMU by clearing registers that have enable bits
	 * (start/stop capabilities).
	 */
	for (i = 0; i < arch_info->max_ena; i++)
		if (test_bit(i, cast_ulp(arch_info->enable_mask)))
			pfm_arch_write_pmc(NULL, i, 0);
}

/*
 * Called from pfm_ctxsw(). Task is guaranteed to be current.
 * set cannot be NULL. Context is locked. Interrupts are masked.
 *
 * Caller has already restored all PMD and PMC registers, if
 * necessary (i.e., lazy restore scheme).
 *
 * on X86, there is nothing else to do. Even with PEBS, the
 * DS area is already restore by pfm_arch_restore_pmcs() which
 * is systematically called as the lazy restore scheme does not
 * work for PMCs (stopping is a destructive operation for PMC).
 */
void pfm_arch_ctxswin_thread(struct task_struct *task, struct pfm_context *ctx,
			     struct pfm_event_set *set)
{
	struct pfm_arch_context *ctx_arch;

	ctx_arch = pfm_ctx_arch(ctx);

	if (set->npend_ovfls)
		__get_cpu_var(real_iip) = ctx_arch->saved_real_iip;

	/*
	 * enable RDPMC on this CPU
	 */
	if (ctx_arch->flags.insecure)
		set_in_cr4(X86_CR4_PCE);
}

static int pfm_stop_save_p6(struct pfm_context *ctx,
			    struct pfm_event_set *set)
{
	struct pfm_arch_pmu_info *arch_info = pfm_pmu_conf->arch_info;
	u64 used_mask[PFM_PMC_BV];
	u64 *cnt_pmds;
	u64 val, wmask, ovfl_mask;
	u32 i, count;

	wmask = 1ULL << pfm_pmu_conf->counter_width;

	bitmap_and(cast_ulp(used_mask),
		   cast_ulp(set->used_pmcs),
		   cast_ulp(arch_info->enable_mask),
		   arch_info->max_ena);

	count = bitmap_weight(cast_ulp(used_mask), pfm_pmu_conf->regs.max_pmc);

	/*
	 * stop monitoring
	 * Unfortunately, this is very expensive!
	 * wrmsrl() is serializing.
	 */
	for (i = 0; count; i++) {
		if (test_bit(i, cast_ulp(used_mask))) {
			wrmsrl(pfm_pmu_conf->pmc_desc[i].hw_addr, 0);
			count--;
		}
	}

	/*
	 * if we already having a pending overflow condition, we simply
	 * return to take care of this first.
	 */
	if (set->npend_ovfls)
		return 1;

	ovfl_mask = pfm_pmu_conf->ovfl_mask;
	cnt_pmds = pfm_pmu_conf->regs.cnt_pmds;

	/*
	 * check for pending overflows and save PMDs (combo)
	 * we employ used_pmds because we also need to save
	 * and not just check for pending interrupts.
	 *
	 * Must check for counting PMDs because of virtual PMDs
	 */
	count = set->nused_pmds;
	for (i = 0; count; i++) {
		if (test_bit(i, cast_ulp(set->used_pmds))) {
			val = pfm_arch_read_pmd(ctx, i);
			if (likely(test_bit(i, cast_ulp(cnt_pmds)))) {
				if (!(val & wmask)) {
					__set_bit(i, cast_ulp(set->povfl_pmds));
					set->npend_ovfls++;
				}
				val = (set->pmds[i].value & ~ovfl_mask) | (val & ovfl_mask);
			}
			set->pmds[i].value = val;
			count--;
		}
	}
	/* 0 means: no need to save PMDs at upper level */
	return 0;
}

#define PFM_AMD64_IBSFETCHVAL	(1ULL<<49) /* valid fetch sample */
#define PFM_AMD64_IBSFETCHEN	(1ULL<<48) /* fetch sampling enabled */
#define PFM_AMD64_IBSOPVAL	(1ULL<<18) /* valid execution sample */
#define PFM_AMD64_IBSOPEN	(1ULL<<17) /* execution sampling enabled */

/*
 * Must check for IBS event BEFORE stop_save_p6 because
 * stopping monitoring does destroy IBS state information
 * in IBSFETCHCTL/IBSOPCTL because they are tagged as enable
 * registers.
 */
static int pfm_stop_save_amd64(struct pfm_context *ctx,
			       struct pfm_event_set *set)
{
	struct pfm_arch_pmu_info *arch_info = pfm_pmu_conf->arch_info;
	u64 used_mask[PFM_PMC_BV];
	u64 *cnt_pmds;
	u64 val, wmask, ovfl_mask;
	u32 i, count, use_ibs;

	/*
	 * IBS used if:
	 *   - on family 10h processor with IBS
	 *   - at least one of the IBS PMD registers is used
	 */
	use_ibs = (arch_info->flags & PFM_X86_FL_IBS)
		&& (test_bit(arch_info->ibsfetchctl_pmd, cast_ulp(set->used_pmds))
		    ||test_bit(arch_info->ibsopctl_pmd, cast_ulp(set->used_pmds)));

	wmask = 1ULL << pfm_pmu_conf->counter_width;

	bitmap_and(cast_ulp(used_mask),
		   cast_ulp(set->used_pmcs),
		   cast_ulp(arch_info->enable_mask),
		   arch_info->max_ena);

	count = bitmap_weight(cast_ulp(used_mask), pfm_pmu_conf->regs.max_pmc);

	/*
	 * stop monitoring
	 * Unfortunately, this is very expensive!
	 * wrmsrl() is serializing.
	 *
	 * With IBS, we need to do read-modify-write to preserve the content
	 * for OpsCTL and FetchCTL because they are also used as PMDs and saved
	 * below
	 */
	if (use_ibs) {
		for (i = 0; count; i++) {
			if (test_bit(i, cast_ulp(used_mask))) {
				if (i == arch_info->ibsfetchctl_pmc) {
					rdmsrl(pfm_pmu_conf->pmc_desc[i].hw_addr, val);
					val &= ~PFM_AMD64_IBSFETCHEN;
				} else if (i == arch_info->ibsopctl_pmc) {
					rdmsrl(pfm_pmu_conf->pmc_desc[i].hw_addr, val);
					val &= ~PFM_AMD64_IBSOPEN;
				} else
					val = 0;
				wrmsrl(pfm_pmu_conf->pmc_desc[i].hw_addr, val);
				count--;
			}
		}
	} else {
		for (i = 0; count; i++) {
			if (test_bit(i, cast_ulp(used_mask))) {
				wrmsrl(pfm_pmu_conf->pmc_desc[i].hw_addr, 0);
				count--;
			}
		}
	}

	/*
	 * if we already having a pending overflow condition, we simply
	 * return to take care of this first.
	 */
	if (set->npend_ovfls)
		return 1;

	ovfl_mask = pfm_pmu_conf->ovfl_mask;
	cnt_pmds = pfm_pmu_conf->regs.cnt_pmds;

	/*
	 * check for pending overflows and save PMDs (combo)
	 * we employ used_pmds because we also need to save
	 * and not just check for pending interrupts.
	 *
	 * Must check for counting PMDs because of virtual PMDs and IBS
	 */
	count = set->nused_pmds;
	for (i = 0; count; i++) {
		if (test_bit(i, cast_ulp(set->used_pmds))) {
			val = pfm_arch_read_pmd(ctx, i);
			if (likely(test_bit(i, cast_ulp(cnt_pmds)))) {
				if (!(val & wmask)) {
					__set_bit(i, cast_ulp(set->povfl_pmds));
					set->npend_ovfls++;
				}
				val = (set->pmds[i].value & ~ovfl_mask) | (val & ovfl_mask);
			}
			set->pmds[i].value = val;
			count--;
		}
	}

	/*
	 * check if IBS contains valid data, and mark the corresponding
	 * PMD has overflowed
	 */
	if (use_ibs) {
		i = arch_info->ibsfetchctl_pmd;
		if (set->pmds[i].value & PFM_AMD64_IBSFETCHVAL) {
			__set_bit(i, cast_ulp(set->povfl_pmds));
			set->npend_ovfls++;
		}
		i = arch_info->ibsopctl_pmd;
		if (set->pmds[i].value & PFM_AMD64_IBSOPVAL) {
			__set_bit(i, cast_ulp(set->povfl_pmds));
			set->npend_ovfls++;
		}
	}
	/* 0 means: no need to save PMDs at upper level */
	return 0;
}

static int pfm_stop_save_intel_core(struct pfm_context *ctx,
				    struct pfm_event_set *set)
{
	struct pfm_arch_pmu_info *arch_info = pfm_pmu_conf->arch_info;
	struct pfm_arch_context *ctx_arch;
	struct pfm_ds_area_intel_core *ds = NULL;
	u64 used_mask[PFM_PMC_BV];
	u64 *cnt_mask;
	u64 val, wmask, ovfl_mask;
	u16 count, has_ovfl;
	u16 i, pebs_idx = ~0;

	ctx_arch = pfm_ctx_arch(ctx);

	wmask = 1ULL << pfm_pmu_conf->counter_width;

	/*
	 * used enable pmc bitmask
	 */
	bitmap_and(cast_ulp(used_mask),
			cast_ulp(set->used_pmcs),
			cast_ulp(arch_info->enable_mask),
			arch_info->max_ena);

	count = bitmap_weight(cast_ulp(used_mask), arch_info->max_ena);
	/*
	 * stop monitoring
	 * Unfortunately, this is very expensive!
	 * wrmsrl() is serializing.
	 */
	for (i = 0; count; i++) {
		if (test_bit(i, cast_ulp(used_mask))) {
			wrmsrl(pfm_pmu_conf->pmc_desc[i].hw_addr, 0);
			count--;
		}
	}
	/*
	 * if we already having a pending overflow condition, we simply
	 * return to take care of this first.
	 */
	if (set->npend_ovfls)
		return 1;

	ovfl_mask = pfm_pmu_conf->ovfl_mask;
	cnt_mask = pfm_pmu_conf->regs.cnt_pmds;

	if (ctx_arch->flags.use_pebs) {
		ds = ctx_arch->ds_area;
		pebs_idx = arch_info->pebs_ctr_idx;
		PFM_DBG("ds=%p pebs_idx=0x%llx thres=0x%llx",
			ds,
			(unsigned long long)ds->pebs_index,
			(unsigned long long)ds->pebs_intr_thres);
	}

	/*
	 * Check for pending overflows and save PMDs (combo)
	 * We employ used_pmds and not intr_pmds because we must
	 * also saved on PMD registers.
	 * Must check for counting PMDs because of virtual PMDs
	 *
	 * XXX: should use the ovf_status register instead, yet
	 *      we would have to check if NMI is used and fallback
	 *      to individual pmd inspection.
	 */
	count = set->nused_pmds;

	for (i = 0; count; i++) {
		if (test_bit(i, cast_ulp(set->used_pmds))) {
			val = pfm_arch_read_pmd(ctx, i);
			if (likely(test_bit(i, cast_ulp(cnt_mask)))) {
				if (i == pebs_idx)
					has_ovfl = (ds->pebs_index >= ds->pebs_intr_thres);
				else
					has_ovfl = !(val & wmask);
				if (has_ovfl) {
					__set_bit(i, cast_ulp(set->povfl_pmds));
					set->npend_ovfls++;
				}
				val = (set->pmds[i].value & ~ovfl_mask) | (val & ovfl_mask);
			}
			set->pmds[i].value = val;
			count--;
		}
	}
	/* 0 means: no need to save PMDs at upper level */
	return 0;
}

static int pfm_stop_save_p4(struct pfm_context *ctx,
			    struct pfm_event_set *set)
{
	struct pfm_arch_pmu_info *arch_info = pfm_pmu_conf->arch_info;
	struct pfm_arch_context *ctx_arch;
	struct pfm_arch_ext_reg *xrc, *xrd;
	struct pfm_ds_area_p4 *ds = NULL;
	u64 used_mask[PFM_PMC_BV];
	u16 i, j, count, pebs_idx = ~0;
	u16 max_pmc;
	u64 cccr, ctr1, ctr2, ovfl_mask;

	ctx_arch = pfm_ctx_arch(ctx);
	max_pmc = pfm_pmu_conf->regs.max_pmc;
	xrc = arch_info->pmc_addrs;
	xrd = arch_info->pmd_addrs;
	ovfl_mask = pfm_pmu_conf->ovfl_mask;

	/*
	 * build used enable PMC bitmask
	 * if user did not set any CCCR, then mask is
	 * empty and there is nothing to do because nothing
	 * was started
	 */
	bitmap_and(cast_ulp(used_mask),
		   cast_ulp(set->used_pmcs),
		   cast_ulp(arch_info->enable_mask),
		   arch_info->max_ena);

	count = bitmap_weight(cast_ulp(used_mask), arch_info->max_ena);

	PFM_DBG_ovfl("npend=%u ena_mask=0x%llx u_pmcs=0x%llx count=%u num=%u",
		set->npend_ovfls,
		(unsigned long long)arch_info->enable_mask[0],
		(unsigned long long)set->used_pmcs[0],
		count, arch_info->max_ena);


	/*
	 * ensures we do not destroy pending overflow
	 * information. If pended interrupts are already
	 * known, then we just stop monitoring.
	 */
	if (set->npend_ovfls) {
		/*
		 * clear enable bit
		 * unfortunately, this is very expensive!
		 */
		for (i = 0; count; i++) {
			if (test_bit(i, cast_ulp(used_mask))) {
				__pfm_write_reg_p4(xrc+i, 0);
				count--;
			}
		}
		/* need save PMDs at upper level */
		return 1;
	}

	if (ctx_arch->flags.use_pebs) {
		ds = ctx_arch->ds_area;
		pebs_idx = arch_info->pebs_ctr_idx;
		PFM_DBG("ds=%p pebs_idx=0x%llx thres=0x%llx",
			ds,
			(unsigned long long)ds->pebs_index,
			(unsigned long long)ds->pebs_intr_thres);
	}

	/*
	 * stop monitoring AND collect pending overflow information AND
	 * save pmds.
	 *
	 * We need to access the CCCR twice, once to get overflow info
	 * and a second to stop monitoring (which destroys the OVF flag)
	 * Similarly, we need to read the counter twice to check whether
	 * it did overflow between the CCR read and the CCCR write.
	 */
	for (i = 0; count; i++) {
		if (i != pebs_idx && test_bit(i, cast_ulp(used_mask))) {
			/*
			 * controlled counter
			 */
			j = xrc[i].ctr;

			/* read CCCR (PMC) value */
			__pfm_read_reg_p4(xrc+i, &cccr);

			/* read counter (PMD) controlled by PMC */
			__pfm_read_reg_p4(xrd+j, &ctr1);

			/* clear CCCR value: stop counter but destroy OVF */
			__pfm_write_reg_p4(xrc+i, 0);

			/* read counter controlled by CCCR again */
			__pfm_read_reg_p4(xrd+j, &ctr2);

			/*
			 * there is an overflow if either:
			 * 	- CCCR.ovf is set (and we just cleared it)
			 * 	- ctr2 < ctr1
			 * in that case we set the bit corresponding to the
			 * overflowed PMD  in povfl_pmds.
			 */
			if ((cccr & (1ULL<<31)) || (ctr2 < ctr1)) {
				__set_bit(j, cast_ulp(set->povfl_pmds));
				set->npend_ovfls++;
			}
			ctr2 = (set->pmds[j].value & ~ovfl_mask) | (ctr2 & ovfl_mask);
			set->pmds[j].value = ctr2;
			count--;
		}
	}
	/*
	 * check for PEBS buffer full and set the corresponding PMD overflow
	 */
	if (ctx_arch->flags.use_pebs) {
		PFM_DBG("ds=%p pebs_idx=0x%lx thres=0x%lx", ds, ds->pebs_index, ds->pebs_intr_thres);
		if (ds->pebs_index >= ds->pebs_intr_thres
		    && test_bit(arch_info->pebs_ctr_idx, cast_ulp(set->used_pmds))) {
			__set_bit(arch_info->pebs_ctr_idx, cast_ulp(set->povfl_pmds));
			set->npend_ovfls++;
		}
	}
	/* 0 means: no need to save the PMD at higher level */
	return 0;
}

/*
 * Called from pfm_stop() and idle notifier
 *
 * Interrupts are masked. Context is locked. Set is the active set.
 *
 * For per-thread:
 *   task is not necessarily current. If not current task, then
 *   task is guaranteed stopped and off any cpu. Access to PMU
 *   is not guaranteed.
 *
 * For system-wide:
 * 	task is current
 *
 * must disable active monitoring. ctx cannot be NULL
 */
void pfm_arch_stop(struct task_struct *task, struct pfm_context *ctx,
		   struct pfm_event_set *set)
{
	/*
	 * no need to go through stop_save()
	 * if we are already stopped
	 */
	if (!ctx->flags.started || ctx->state == PFM_CTX_MASKED)
		return;

	if (task == current)
		pfm_stop_save(ctx, set);
}

/*
 * Called from pfm_ctxsw(). Task is guaranteed to be current.
 * Context is locked. Interrupts are masked. Monitoring may be active.
 * PMU access is guaranteed. PMC and PMD registers are live in PMU.
 *
 * Must stop monitoring, save pending overflow information
 *
 * Return:
 * 	non-zero : did not save PMDs (as part of stopping the PMU)
 * 	       0 : saved PMDs (no need to save them in caller)
 */
int pfm_arch_ctxswout_thread(struct task_struct *task, struct pfm_context *ctx,
		             struct pfm_event_set *set)
{
	struct pfm_arch_context *ctx_arch;

	ctx_arch = pfm_ctx_arch(ctx);

	/*
	 * disable lazy restore of PMCS on ctxswin because
	 * we modify some of them.
	 */
	set->priv_flags |= PFM_SETFL_PRIV_MOD_PMCS;

	if (set->npend_ovfls) {
		ctx_arch->saved_real_iip = __get_cpu_var(real_iip);
	}
	/*
	 * disable RDPMC on this CPU
	 */
	if (ctx_arch->flags.insecure)
		clear_in_cr4(X86_CR4_PCE);

	if (ctx->state == PFM_CTX_MASKED)
		return 1;

	return pfm_stop_save(ctx, set);
}

/*
 * called from pfm_start() and idle notifier
 *
 * Interrupts are masked. Context is locked. Set is the active set.
 *
 * For per-thread:
 * 	Task is not necessarily current. If not current task, then task
 * 	is guaranteed stopped and off any cpu. No access to PMU is task
 *	is not current.
 *
 * For system-wide:
 * 	task is always current
 *
 * must enable active monitoring.
 */
void pfm_arch_start(struct task_struct *task, struct pfm_context *ctx,
		    struct pfm_event_set *set)
{
	struct pfm_arch_context *ctx_arch;
	u64 *mask;
	u16 i, num;

	/*
	 * pfm_start issue while context is masked as no effect.
	 * This comes from the fact that on x86, masking and stopping
	 * use the same mechanism, i.e., clearing the enable bits
	 * of the PMC registers.
	 */
	if (ctx->state == PFM_CTX_MASKED)
		return;

	/*
	 * cannot restore PMC if no access to PMU. Will be done
	 * when the thread is switched back in
	 */
	if (task != current)
		return;

	ctx_arch = pfm_ctx_arch(ctx);

	/*
	 * reload DS area pointer.
	 * Must be done before we restore the PMCs
	 * avoid a race condition
	 */
	if (ctx_arch->flags.use_ds)
		wrmsrl(MSR_IA32_DS_AREA, (unsigned long)ctx_arch->ds_area);
	/*
	 * we must actually install all implemented pmcs registers because
	 * until started, we do not write any PMC registers.
	 * Note that registers used  by other subsystems (e.g. NMI) are
	 * removed from pmcs.
	 *
	 * The available registers that are actually not used get their default
	 * value such that counters do not count anything. As such, we can
	 * afford to write all of them but then stop only the one we use.
	 *
	 * XXX: we may be able to optimize this for non-P4 PMU as pmcs are
	 * independent from each others.
	 */
	num = pfm_pmu_conf->regs.num_pmcs;
	mask = pfm_pmu_conf->regs.pmcs;
	for (i = 0; num; i++) {
		if (test_bit(i, cast_ulp(mask))) {
			pfm_arch_write_pmc(ctx, i, set->pmcs[i]);
			num--;
		}
	}
}

/*
 * function called from pfm_switch_sets(), pfm_context_load_thread(),
 * pfm_context_load_sys(), pfm_ctxsw()
 *
 * context is locked. Interrupts are masked. Set cannot be NULL.
 * Access to the PMU is guaranteed.
 *
 * function must restore PMD registers
 */
void pfm_arch_restore_pmds(struct pfm_context *ctx, struct pfm_event_set *set)
{
	u64 *used_pmds;
	u16 i, num;

	used_pmds = set->used_pmds;
	num = set->nused_pmds;

	/*
	 * we can restore only the PMD we use because:
	 * 	- you can only read with pfm_read_pmds() the registers
	 * 	  declared used via pfm_write_pmds(), smpl_pmds, reset_pmds
	 *
	 * 	- if cr4.pce=1, only counters are exposed to user. No
	 * 	  address is ever exposed by counters.
	 */
	for (i = 0; num; i++) {
		if (likely(test_bit(i, cast_ulp(used_pmds)))) {
			pfm_write_pmd(ctx, i, set->pmds[i].value);
			num--;
		}
	}
}

/*
 * function called from pfm_switch_sets(), pfm_context_load_thread(),
 * pfm_context_load_sys(), pfm_ctxsw().
 * Context is locked. Interrupts are masked. set cannot be NULL.
 * Access to the PMU is guaranteed.
 *
 * function must restore all PMC registers from set
 */
void pfm_arch_restore_pmcs(struct pfm_context *ctx, struct pfm_event_set *set)
{
	struct pfm_arch_pmu_info *arch_info = pfm_pmu_conf->arch_info;
	struct pfm_arch_context *ctx_arch;
	u64 *mask;
	u16 i, num;

	ctx_arch = pfm_ctx_arch(ctx);
	/*
	 * we need to restore PMCs only when:
	 * 	- context is not masked
	 * 	- monitoring was activated
	 *
	 * Masking monitoring after an overflow does not change the
	 * value of flags.started
	 */
	if (ctx->state == PFM_CTX_MASKED || !ctx->flags.started)
		return;

	/*
	 * must restore DS pointer before restoring PMCs
	 * as this can potentially reactivate monitoring
	 */
	if (ctx_arch->flags.use_ds)
		wrmsrl(MSR_IA32_DS_AREA, (unsigned long)ctx_arch->ds_area);

	/*
	 * In general, writing MSRs is very expensive, so try to be smart.
	 *
	 * P6-style, Core-style:
	 * 	- pmc are totally independent of each other, there is
	 * 	  possible side-effect from stale pmcs. Therefore we only
	 * 	  restore the registers we use
	 * P4-style:
	 * 	- must restore everything because there are some dependencies
	 * 	(e.g., ESCR and CCCR)
	 */
	if (arch_info->pmu_style == PFM_X86_PMU_P4) {
		num = pfm_pmu_conf->regs.num_pmcs;
		mask = pfm_pmu_conf->regs.pmcs;
	} else {
		num = set->nused_pmcs;
		mask = set->used_pmcs;
	}
	for (i = 0; num; i++) {
		if (test_bit(i, cast_ulp(mask))) {
			pfm_arch_write_pmc(ctx, i, set->pmcs[i]);
			num--;
		}
	}
}

/*
 * invoked only when NMI is used. Called from the LOCAL_PERFMON_VECTOR
 * handler to copy P4 overflow state captured when the NMI triggered.
 * Given that on P4, stopping monitoring destroy the overflow information
 * we save it in pfm_has_ovfl_p4() where monitoring is also stopped.
 *
 * Here we propagate the overflow state to current active set. The
 * freeze_pmu() call we not overwrite this state because npend_ovfls
 * is non-zero.
 */
static void pfm_p4_copy_nmi_state(void)
{
	struct pfm_context *ctx;
	struct pfm_arch_context *ctx_arch;
	struct pfm_event_set *set;

	ctx = __get_cpu_var(pmu_ctx);
	if (!ctx)
		return;

	ctx_arch = pfm_ctx_arch(ctx);
	set = ctx->active_set;

	if (ctx_arch->p4->npend_ovfls) {
		set->npend_ovfls = ctx_arch->p4->npend_ovfls;

		bitmap_copy(cast_ulp(set->povfl_pmds),
			    cast_ulp(ctx_arch->p4->povfl_pmds),
			    pfm_pmu_conf->regs.max_pmd);

		ctx_arch->p4->npend_ovfls = 0;
	}
}

/*
 * The PMU interrupt is handled through an interrupt gate, therefore
 * the CPU automatically clears the EFLAGS.IF, i.e., masking interrupts.
 *
 * The perfmon interrupt handler MUST run with interrupts disabled due
 * to possible race with other, higher priority interrupts, such as timer
 * or IPI function calls.
 *
 * See description in IA-32 architecture manual, Vol 3 section 5.8.1
 */
void smp_pmu_interrupt(struct pt_regs *regs)
{
	struct pfm_arch_pmu_info *arch_info;
	unsigned long iip;
	int using_nmi;

	using_nmi = __get_cpu_var(pfm_using_nmi);

	ack_APIC_irq();

	irq_enter();

	/*
	 * when using NMI, pfm_handle_nmi() gets called
	 * first. It stops monitoring and record the
	 * iip into real_iip, then it repost the interrupt
	 * using the lower priority vector LOCAL_PERFMON_VECTOR
	 *
	 * On P4, due to the difficulty of detecting overflows
	 * and stoppping the PMU, pfm_handle_nmi() needs to
	 * record npend_ovfl and ovfl_pmds in ctx_arch. So
	 * here we simply copy them back to the set.
	 */
	if (using_nmi) {
		arch_info = pfm_pmu_conf->arch_info;
		iip = __get_cpu_var(real_iip);
		if (arch_info->pmu_style == PFM_X86_PMU_P4)
			pfm_p4_copy_nmi_state();
	} else
		iip = instruction_pointer(regs);

	pfm_interrupt_handler(iip, regs);

	/*
	 * On Intel P6, Pentium M, P4, Intel Core:
	 * 	- it is necessary to clear the MASK field for the LVTPC
	 * 	  vector. Otherwise interrupts remain masked. See
	 * 	  section 8.5.1
	 * AMD X86-64:
	 * 	- the documentation does not stipulate the behavior.
	 * 	  To be safe, we also rewrite the vector to clear the
	 * 	  mask field
	 */
	if (!using_nmi && current_cpu_data.x86_vendor == X86_VENDOR_INTEL)
		apic_write(APIC_LVTPC, LOCAL_PERFMON_VECTOR);

	irq_exit();
}

/*
 * detect is counters have overflowed.
 * return:
 * 	0 : no overflow
 * 	1 : at least one overflow
 *
 * used by AMD64 and Intel architectural PMU
 */
static int __kprobes pfm_has_ovfl_p6(struct pfm_context *ctx)
{
	struct pfm_arch_pmu_info *arch_info = pfm_pmu_conf->arch_info;
	struct pfm_arch_ext_reg *xrd;
	u64 *cnt_mask;
	u64 wmask, val;
	u16 i, num;

	cnt_mask = pfm_pmu_conf->regs.cnt_pmds;
	num = pfm_pmu_conf->regs.num_counters;
	wmask = 1ULL << pfm_pmu_conf->counter_width;
	xrd = arch_info->pmd_addrs;

	for (i = 0; num; i++) {
		if (test_bit(i, cast_ulp(cnt_mask))) {
			rdmsrl(xrd[i].addrs[0], val);
			if (!(val & wmask))
				return 1;
			num--;
		}
	}
	return 0;
}

static int __kprobes pfm_has_ovfl_amd64(struct pfm_context *ctx)
{
	struct pfm_arch_pmu_info *arch_info = pfm_pmu_conf->arch_info;
	u64 val;
	/*
	 * Check for IBS events
	 */
	if (arch_info->flags & PFM_X86_FL_IBS) {
		rdmsrl(pfm_pmu_conf->pmc_desc[arch_info->ibsfetchctl_pmc].hw_addr, val);
		if (val & PFM_AMD64_IBSFETCHVAL)
			return 1;
		rdmsrl(pfm_pmu_conf->pmc_desc[arch_info->ibsopctl_pmc].hw_addr, val);
		if (val & PFM_AMD64_IBSOPVAL)
			return 1;
	}
	return pfm_has_ovfl_p6(ctx);
}

/*
 * detect is counters have overflowed.
 * return:
 * 	0 : no overflow
 * 	1 : at least one overflow
 *
 * used by Intel P4
 */
static int __kprobes pfm_has_ovfl_p4(struct pfm_context *ctx)
{	
	struct pfm_arch_ext_reg *xrc, *xrd;
	struct pfm_arch_context *ctx_arch;
	struct pfm_arch_p4_context *p4;
	struct pfm_arch_pmu_info *arch_info = pfm_pmu_conf->arch_info;
	u64 ena_mask[PFM_PMC_BV];
	u64 cccr, ctr1, ctr2;
	int n, i, j;

	ctx_arch = pfm_ctx_arch(ctx);
	xrc = arch_info->pmc_addrs;
	xrd = arch_info->pmd_addrs;
	p4 = ctx_arch->p4;

	bitmap_and(cast_ulp(ena_mask),
			cast_ulp(pfm_pmu_conf->regs.pmcs),
			cast_ulp(arch_info->enable_mask),
			arch_info->max_ena);

	n = bitmap_weight(cast_ulp(ena_mask), arch_info->max_ena);

	for(i=0; n; i++) {
		if (!test_bit(i, cast_ulp(ena_mask)))
			continue;
		/*
		 * controlled counter
		 */
		j = xrc[i].ctr;

		/* read CCCR (PMC) value */
		__pfm_read_reg_p4(xrc+i, &cccr);

		/* read counter (PMD) controlled by PMC */
		__pfm_read_reg_p4(xrd+j, &ctr1);

		/* clear CCCR value: stop counter but destroy OVF */
		__pfm_write_reg_p4(xrc+i, 0);

		/* read counter controlled by CCCR again */
		__pfm_read_reg_p4(xrd+j, &ctr2);

		/*
		 * there is an overflow if either:
		 * 	- CCCR.ovf is set (and we just cleared it)
		 * 	- ctr2 < ctr1
		 * in that case we set the bit corresponding to the
		 * overflowed PMD in povfl_pmds.
		 */
		if ((cccr & (1ULL<<31)) || (ctr2 < ctr1)) {
			__set_bit(j, cast_ulp(ctx_arch->p4->povfl_pmds));
			ctx_arch->p4->npend_ovfls++;
		}
		p4->saved_cccrs[i] = cccr;
		n--;
	}
	/*
	 * if there was no overflow, then it means the NMI was not really
	 * for us, so we have to resume monitoring
	 */
	if (unlikely(!ctx_arch->p4->npend_ovfls)) {
		for(i=0; n; i++) {
			if (!test_bit(i, cast_ulp(ena_mask)))
				continue;
			__pfm_write_reg_p4(xrc+i, ctx_arch->p4->saved_cccrs[i]);
		}
	}
	return 0;
}

/*
 * detect is counters have overflowed.
 * return:
 * 	0 : no overflow
 * 	1 : at least one overflow
 *
 * used by Intel Core-based processors
 */
static int __kprobes pfm_has_ovfl_intel_core(struct pfm_context *ctx)
{
	struct pfm_arch_pmu_info *arch_info = pfm_pmu_conf->arch_info;
	struct pfm_arch_ext_reg *xrd;
	u64 *cnt_mask;
	u64 wmask, val;
	u16 i, num;

	cnt_mask = pfm_pmu_conf->regs.cnt_pmds;
	num = pfm_pmu_conf->regs.num_counters;
	wmask = 1ULL << pfm_pmu_conf->counter_width;
	xrd = arch_info->pmd_addrs;

	for (i = 0; num; i++) {
		if (test_bit(i, cast_ulp(cnt_mask))) {
			rdmsrl(xrd[i].addrs[0], val);
			if (!(val & wmask))
				return 1;
			num--;
		}
	}
	return 0;
}

/*
 * called from notify_die() notifier from an trap handler path. We only
 * care about NMI related callbacks, and ignore everything else.
 *
 * Cannot grab any locks, include the perfmon context lock
 *
 * Must detect if NMI interrupt comes from perfmon, and if so it must
 * stop the PMU and repost a lower-priority interrupt. The perfmon interrupt
 * handler needs to grab the context lock, thus is cannot be run directly
 * from the NMI interrupt call path.
 */
static int __kprobes pfm_handle_nmi(struct notifier_block *nb, unsigned long val,
			      void *data)
{
	struct die_args *args = data;
	struct pfm_context *ctx;

	/*
	 * only NMI related calls
	 */
	if (val != DIE_NMI_IPI)
		return NOTIFY_DONE;

	if (!__get_cpu_var(pfm_using_nmi))
		return NOTIFY_DONE;

	/*
	 * perfmon not active on this processor
	 */
	ctx = __get_cpu_var(pmu_ctx);
	if (ctx == NULL) {
		PFM_DBG_ovfl("ctx NULL");
		return NOTIFY_DONE;
	}

	/*
	 * detect if we have overflows, i.e., NMI interrupt
	 * caused by PMU
	 */
	if (!pfm_has_ovfl(ctx)) {
		PFM_DBG_ovfl("no ovfl");
		return NOTIFY_DONE;
	}

	/*
	 * we stop the PMU to avoid further overflow before this
	 * one is treated by lower priority interrupt handler
	 */
	__pfm_arch_quiesce_pmu_percpu();

	/*
	 * record actual instruction pointer
	 */
	__get_cpu_var(real_iip) = instruction_pointer(args->regs);

	/*
	 * post lower priority interrupt (LOCAL_PERFMON_VECTOR)
	 */
	pfm_arch_resend_irq();

	pfm_stats_get(ovfl_intr_nmi_count)++;

	/*
	 * we need to rewrite the APIC vector on Intel
	 */
	if (current_cpu_data.x86_vendor == X86_VENDOR_INTEL)
		apic_write(APIC_LVTPC, APIC_DM_NMI);

	/*
	 * the notification was for us
	 */
	return NOTIFY_STOP;
}

static struct notifier_block pfm_nmi_nb = {
	.notifier_call = pfm_handle_nmi
};

/*
 * called from pfm_register_pmu_config() after the new
 * config has been validated. The pfm_session_lock
 * is held.
 *
 * return:
 * 	< 0 : if error
 * 	  0 : if success
 */
int pfm_arch_pmu_config_init(struct pfm_pmu_config *cfg)
{
	struct pfm_arch_pmu_info *arch_info = cfg->arch_info;

	/*
	 * adust stop routine based on PMU model
	 *
	 * P6  : P6, Pentium M, Intel architectural perfmon
	 * P4  : Xeon, EM64T, P4
	 * Core: Core 2,
	 * AMD64: AMD64 (K8, family 10h)
	 */
	switch(arch_info->pmu_style) {
	case PFM_X86_PMU_P4:
		pfm_stop_save = pfm_stop_save_p4;
		pfm_has_ovfl  = pfm_has_ovfl_p4;
		break;
	case PFM_X86_PMU_P6:
		pfm_stop_save = pfm_stop_save_p6;
		pfm_has_ovfl  = pfm_has_ovfl_p6;
		break;
	case PFM_X86_PMU_CORE:
		pfm_stop_save = pfm_stop_save_intel_core;
		pfm_has_ovfl  = pfm_has_ovfl_intel_core;
		break;
	case PFM_X86_PMU_AMD64:
		pfm_stop_save = pfm_stop_save_amd64;
		pfm_has_ovfl  = pfm_has_ovfl_amd64;
		break;
	default:
		PFM_INFO("unknown pmu_style=%d", arch_info->pmu_style);
		return -EINVAL;
	}
	return 0;
}

void pfm_arch_pmu_config_remove(void)
{
}

char *pfm_arch_get_pmu_module_name(void)
{
	switch(current_cpu_data.x86) {
	case 6:
		switch(current_cpu_data.x86_model) {
		case 3: /* Pentium II */
		case 7 ... 11:
		case 13:
			return "perfmon_p6";
		case 15: /* Merom */
		case 23: /* Penryn */
			return "perfmon_intel_core";
		default:
			goto try_arch;
		}
	case 15:
	case 16:
		/* All Opteron processors */
		if (current_cpu_data.x86_vendor == X86_VENDOR_AMD)
			return "perfmon_amd64";

		switch(current_cpu_data.x86_model) {
		case 0 ... 6:
			return "perfmon_p4";
		}
		/* FALL THROUGH */
	default:
try_arch:
		if (boot_cpu_has(X86_FEATURE_ARCH_PERFMON))
			return "perfmon_intel_arch";
		return NULL;
	}
	return NULL;
}

void pfm_arch_resend_irq(void)
{
	unsigned long val, dest;
	/*
	 * we cannot use hw_resend_irq() because it goes to
	 * the I/O APIC. We need to go to the Local APIC.
	 *
	 * The "int vec" is not the right solution either
	 * because it triggers a software intr. We need
	 * to regenerate the interrupt and have it pended
	 * until we unmask interrupts.
	 *
	 * Instead we send ourself an IPI on the perfmon
	 * vector.
	 */
	val  = APIC_DEST_SELF|APIC_INT_ASSERT|
	       APIC_DM_FIXED|LOCAL_PERFMON_VECTOR;

	dest = apic_read(APIC_ID);
	apic_write(APIC_ICR2, dest);
	apic_write(APIC_ICR, val);
}

DEFINE_PER_CPU(unsigned long, saved_lvtpc);

static void pfm_arch_pmu_acquire_percpu(void *data)
{

	unsigned int tmp, vec;
	unsigned long flags = (unsigned long)data;
	unsigned long lvtpc;

	/*
	 * we only reprogram the LVTPC vector if we have detected
	 * no sharing, otherwise it means the APIC is already program
	 * and we use whatever vector (likely NMI) was used
	 */
	if (!(flags & PFM_X86_FL_SHARING)) {
		vec = flags & PFM_X86_FL_USE_NMI ? APIC_DM_NMI : LOCAL_PERFMON_VECTOR;
		tmp = apic_read(APIC_LVTERR);
		apic_write(APIC_LVTERR, tmp | APIC_LVT_MASKED);
		apic_write(APIC_LVTPC, vec);
		apic_write(APIC_LVTERR, tmp);
		PFM_DBG("written LVTPC=0x%x", vec);
	}
	lvtpc = (unsigned long)apic_read(APIC_LVTPC);
	__get_cpu_var(pfm_using_nmi) = lvtpc == APIC_DM_NMI;
	PFM_DBG("LTVPC=0x%lx using_nmi=%d", lvtpc, __get_cpu_var(pfm_using_nmi));
}

/*
 * called from pfm_pmu_acquire() with
 * pfm_pmu_conf.regs copied from pfm_pmu_conf.full_regs
 * needs to adjust regs to match current PMU availabilityy
 *
 * Caller does recalculate all max/num/first limits on the
 * pfm_pmu_conf.regs structure.
 *
 * interrupts are not masked
 *
 *
 * XXX: until reserve_*_nmi() get fixed by Bjorn to work
 * correctly whenever the NMI watchdog is not used. We skip
 * the allocation. Yet we do the percpu initialization.
 */
int pfm_arch_pmu_acquire(void)
{
	struct pfm_arch_pmu_info *arch_info;
	struct pfm_regmap_desc *d;
	struct pfm_arch_ext_reg *pc;
	u16 i, n, ena = 0, nlost;

	arch_info = pfm_pmu_conf->arch_info;
	pc = arch_info->pmc_addrs;

	bitmap_zero(cast_ulp(arch_info->enable_mask), PFM_MAX_PMCS);
	arch_info->flags &= ~PFM_X86_FL_SHARING;

	d = pfm_pmu_conf->pmc_desc;
	n = pfm_pmu_conf->regs.num_pmcs;
	nlost = 0;
	for(i=0; n; i++, d++) {
		/*
		 * skip not implemented registers (including those
		 * already removed by the module)
		 */
		if (!(d->type & PFM_REG_I))
			continue;

		n--;

		if (d->type & PFM_REG_V)
			continue;

		/*
		 * reserve register with lower-level allocator
		 */
		if (!reserve_evntsel_nmi(d->hw_addr)) {
			PFM_DBG("pmc%d (%s) in use elsewhere, disabling", i, d->desc);
			__clear_bit(i, cast_ulp(pfm_pmu_conf->regs.pmcs));
			nlost++;
			continue;
		}

		if (!(pc[i].reg_type & PFM_REGT_EN))
			continue;
		__set_bit(i, cast_ulp(arch_info->enable_mask));
		ena++;
		arch_info->max_ena = i + 1;
	}

	PFM_DBG("%u PMCs with enable capability", ena);

	if (!ena) {
		PFM_INFO("no registers with start/stop capability,"
			 "try rebooting with nmi_watchdog=0, or check that Oprofile is not running");
		goto undo;
	}
	PFM_DBG("nlost=%d info_flags=0x%x\n", nlost, arch_info->flags);
	/*
	 * some PMU models (e.g., P6) do not support sharing
	 * so check if we found less than the expected number of PMC registers
	 */
	if (nlost) {
		if (arch_info->flags & PFM_X86_FL_NO_SHARING) {
			PFM_INFO("PMU already used by another subsystem, "
				 "PMU does not support sharing, "
				 "try disabling Oprofile or "
				 "reboot with nmi_watchdog=0");
			goto undo;
		}
		arch_info->flags |= PFM_X86_FL_SHARING;
	}

	d = pfm_pmu_conf->pmd_desc;
	n = pfm_pmu_conf->regs.num_pmds;
	for(i=0; n; i++, d++) {
		if (!(d->type & PFM_REG_I))
			continue;
		n--;

		if (d->type & PFM_REG_V)
			continue;

		if (!reserve_perfctr_nmi(d->hw_addr)) {
			PFM_DBG("pmd%d (%s) in use elsewhere, disabling", i, d->desc);
			__clear_bit(i, cast_ulp(pfm_pmu_conf->regs.pmds));
			__clear_bit(i, cast_ulp(pfm_pmu_conf->regs.cnt_pmds));
			__clear_bit(i, cast_ulp(pfm_pmu_conf->regs.rw_pmds));
		}
	}
	/*
	 * program APIC on each CPU
	 */
	on_each_cpu(pfm_arch_pmu_acquire_percpu,
		    (void *)(unsigned long)arch_info->flags , 0, 1);

	return 0;
undo:
	/*
	 * must undo reservation in case of error
	 */
	n = pfm_pmu_conf->regs.max_pmc;
	d = pfm_pmu_conf->pmc_desc;
	for(i=0; i < n; i++, d++) {
		if (!test_bit(i, cast_ulp(pfm_pmu_conf->regs.pmcs)))
			continue;
		release_evntsel_nmi(d->hw_addr);
	}
	return -EBUSY;
}

static void pfm_arch_pmu_release_percpu(void *data)
{
	__get_cpu_var(pfm_using_nmi) = 0;
}

/*
 * called from pfm_pmu_release()
 * interrupts are not masked
 */
void pfm_arch_pmu_release(void)
{
	struct pfm_regmap_desc *d;
	u16 i, n;

	d = pfm_pmu_conf->pmc_desc;
	n = pfm_pmu_conf->regs.num_pmcs;
	for(i=0; n; i++, d++) {
		if (!test_bit(i, cast_ulp(pfm_pmu_conf->regs.pmcs)))
			continue;
		release_evntsel_nmi(d->hw_addr);
		n--;
		PFM_DBG("pmc%u released", i);
	}
	d = pfm_pmu_conf->pmd_desc;
	n = pfm_pmu_conf->regs.num_pmds;
	for(i=0; n; i++, d++) {
		if (!test_bit(i, cast_ulp(pfm_pmu_conf->regs.pmds)))
			continue;
		release_perfctr_nmi(d->hw_addr);
		n--;
		PFM_DBG("pmd%u released", i);
	}
	on_each_cpu(pfm_arch_pmu_release_percpu, NULL , 0, 1);
}

int pfm_arch_init(void)
{
	/*
	 * we need to register our NMI handler when the kernels boots
	 * to avoid a deadlock condition with the NMI watchdog or Oprofile
	 * if we were to try and register/unregister on-demand.
	 */
	register_die_notifier(&pfm_nmi_nb);
	return 0;
}
