/*
 * Copyright (c) 2005-2006 Hewlett-Packard Development Company, L.P.
 * Contributed by Stephane Eranian <eranian@hpl.hp.com>
 *
 * Copyright (c) 2007 Advanced Micro Devices, Inc.
 * Contributed by Robert Richter <robert.richter@amd.com>
 *
 * This file contains X86 Processor Family specific definitions
 * for the perfmon interface. This covers P6, Pentium M, P4/Xeon
 * (32-bit and 64-bit, i.e., EM64T) and AMD X86-64.
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
#ifndef _ASM_X86_PERFMON_H_
#define _ASM_X86_PERFMON_H_

#ifdef __KERNEL__

#ifdef CONFIG_4KSTACKS
#define PFM_ARCH_PMD_STK_ARG	2
#define PFM_ARCH_PMC_STK_ARG	2
#else
#define PFM_ARCH_PMD_STK_ARG	4 /* about 700 bytes of stack space */
#define PFM_ARCH_PMC_STK_ARG	4 /* about 200 bytes of stack space */
#endif

/*
 * For P4:
 * - bits 31 - 63 reserved
 * - T1_OS and T1_USR bits are reserved - set depending on logical proc
 *      user mode application should use T0_OS and T0_USR to indicate
 * RSVD: reserved bits must be 1
 */
#define PFM_ESCR_RSVD  ~0x000000007ffffffcULL

/*
 * bitmask for reg_type
 */
#define PFM_REGT_NA		0x0000	/* not available */
#define PFM_REGT_EN		0x0001	/* has enable bit (cleared on ctxsw) */
#define PFM_REGT_ESCR		0x0002	/* P4: ESCR */
#define PFM_REGT_CCCR		0x0004	/* P4: CCCR */
#define PFM_REGT_PEBS		0x0010	/* PEBS related */
#define PFM_REGT_NOHT		0x0020	/* unavailable with HT */
#define PFM_REGT_CTR		0x0040	/* counter */
#define PFM_REGT_OTH		0x0080	/* other type of register */
#define PFM_REGT_IBS		0x0100	/* IBS register set */
#define PFM_REGT_IBS_EXT	0x0200	/* IBS extended register set */

/*
 * This design and the partitioning of resources for SMT (hyper threads)
 * is very static and limited due to limitations in the number of ESCRs
 * and CCCRs per group.
 */
#define MAX_SMT_ID 1

/*
 * For extended register information in addition to address that is used
 * at runtime to figure out the mapping of reg addresses to logical procs
 * and association of registers to hardware specific features
 */
struct pfm_arch_ext_reg {
	/*
	 * one each for the logical CPUs.  Index 0 corresponds to T0 and
	 * index 1 corresponds to T1.  Index 1 can be zero if no T1
	 * complement reg exists.
	 */
	unsigned long addrs[MAX_SMT_ID+1];
	unsigned int ctr;	/* for CCCR/PERFEVTSEL, associated counter */
	unsigned int reg_type;
};

typedef int (*pfm_check_session_t)(struct pfm_context *ctx);

struct pfm_arch_pmu_info {
	struct pfm_arch_ext_reg pmc_addrs[PFM_MAX_PMCS];
	struct pfm_arch_ext_reg pmd_addrs[PFM_MAX_PMDS];
	u64 enable_mask[PFM_PMC_BV]; /* PMC registers with enable bit */

	u16 max_ena;		/* highest enable bit + 1 */
	u16 flags;		/* PMU feature flags */
	u16 pebs_ctr_idx;	/* index of PEBS counter for overflow */
	u16 reserved;		/* for future use */

	/*
	 * optional callbacks invoked by pfm_arch_*load_context()
	 */
	int (*load_context)(struct pfm_context *ctx);
	int (*unload_context)(struct pfm_context *ctx);

	u16 ibsfetchctl_pmc;	/* AMD: index of IBSFETCHCTL PMC register */
	u16 ibsfetchctl_pmd;	/* AMD: index of IBSFETCHCTL PMD register */
	u16 ibsopctl_pmc;	/* AMD: index of IBSOPCTL PMC register */
	u16 ibsopctl_pmd;	/* AMD: index of IBSOPCTL PMD register */
	u8  ibs_eilvt_off;	/* AMD: extended interrupt LVT offset */
	u8  pmu_style;		/* type of PMU: P4, P6, CORE, AMD64 */
};

/*
 * X86 PMU style
 */
#define PFM_X86_PMU_P4		1 /* Intel P4/Xeon/EM64T processor PMU */
#define PFM_X86_PMU_P6		2 /* Intel P6/Pentium M */
#define PFM_X86_PMU_CORE	3 /* Intel Core PMU */
#define PFM_X86_PMU_AMD64	4 /* AMD64 PMU (K8, family 10h) */

/*
 * PMU feature flags
 */
#define PFM_X86_FL_PMU_DS	0x01	/* Intel: support for Data Save Area (DS) */
#define PFM_X86_FL_PMU_PEBS	0x02	/* Intel: support PEBS (implies DS) */
#define PFM_X86_FL_USE_NMI	0x04	/* user asking for NMI */
#define PFM_X86_FL_NO_SHARING	0x08	/* no sharing with other subsystems */
#define PFM_X86_FL_SHARING	0x10	/* PMU is being shared */
#define PFM_X86_FL_IBS		0x20	/* AMD: PMU has IBS support */
#define PFM_X86_FL_IBS_EXT	0x40	/* AMD: PMU has IBSext support */
#define PFM_X86_FL_USE_EI	0x80	/* AMD: PMU uses extended interrupts */

/*
 * architecture specific context extension.
 * located at: (struct pfm_arch_context *)(ctx+1)
 */

struct pfm_arch_p4_context {
	u32	npend_ovfls;	/* P4 NMI #pending ovfls */
	u32	reserved;
	u64	povfl_pmds[PFM_PMD_BV]; /* P4 NMI overflowed counters */
	u64	saved_cccrs[PFM_MAX_PMCS];
};

struct pfm_x86_context_flags {
	unsigned int insecure:1;  /* insecure monitoring for non-self session */
	unsigned int use_pebs:1;  /* PEBS used */
	unsigned int use_ds:1;    /* DS used */
	unsigned int reserved:29; /* for future use */
};

struct pfm_arch_context {
	u64				saved_real_iip;	/* instr pointer of last NMI intr (ctxsw) */
	struct pfm_x86_context_flags	flags;		/* arch-specific flags */
	void				*ds_area;	/* address of DS management area */
	struct pfm_arch_p4_context	*p4;		/* P4 specific state */
};

void __pfm_read_reg_p4(const struct pfm_arch_ext_reg *xreg, u64 *val);
void __pfm_write_reg_p4(const struct pfm_arch_ext_reg *xreg, u64 val);


extern int  pfm_arch_init(void);
extern void pfm_arch_resend_irq(void);

static inline void pfm_arch_serialize(void)
{}

/*
 * on x86, the PMDs are already saved by pfm_arch_freeze_pmu()
 * when entering the PMU interrupt handler, thus, we do not need
 * to save them again in pfm_switch_sets_from_intr()
 */
static inline void pfm_arch_save_pmds_from_intr(struct pfm_context *ctx,
						struct pfm_event_set *set)
{}

/*
 * in certain situations, ctx may be NULL
 */
static inline void pfm_arch_write_pmc(struct pfm_context *ctx, unsigned int cnum, u64 value)
{
	struct pfm_arch_pmu_info *arch_info = pfm_pmu_conf->arch_info;
	/*
	 * we only write to the actual register when monitoring is
	 * active (pfm_start was issued)
	 */
	if (ctx && ctx->flags.started == 0)
		return;

	PFM_DBG_ovfl("pfm_arch_write_pmc(0x%lx, 0x%Lx)",
		     pfm_pmu_conf->pmc_desc[cnum].hw_addr,
		     (unsigned long long) value);

	if (arch_info->pmu_style == PFM_X86_PMU_P4)
		__pfm_write_reg_p4(&arch_info->pmc_addrs[cnum], value);
	else
		wrmsrl(pfm_pmu_conf->pmc_desc[cnum].hw_addr, value);
}

static inline void pfm_arch_write_pmd(struct pfm_context *ctx, unsigned int cnum, u64 value)
{
	struct pfm_arch_pmu_info *arch_info = pfm_pmu_conf->arch_info;

	/*
	 * to make sure the counter overflows, we set the bits from
	 * bit 31 till the width of the counters.
	 * We clear any other unimplemented bits.
	 */
	if (pfm_pmu_conf->pmd_desc[cnum].type & PFM_REG_C64)
		value = (value | ~pfm_pmu_conf->ovfl_mask)
		      & ~pfm_pmu_conf->pmd_desc[cnum].rsvd_msk;

	PFM_DBG_ovfl("pfm_arch_write_pmd(0x%lx, 0x%Lx)",
		     pfm_pmu_conf->pmd_desc[cnum].hw_addr,
		     (unsigned long long) value);

	if (arch_info->pmu_style == PFM_X86_PMU_P4)
		__pfm_write_reg_p4(&arch_info->pmd_addrs[cnum], value);
	else
		wrmsrl(pfm_pmu_conf->pmd_desc[cnum].hw_addr, value);
}

static inline u64 pfm_arch_read_pmd(struct pfm_context *ctx, unsigned int cnum)
{
	struct pfm_arch_pmu_info *arch_info = pfm_pmu_conf->arch_info;
	u64 tmp;
	if (arch_info->pmu_style == PFM_X86_PMU_P4)
		__pfm_read_reg_p4(&arch_info->pmd_addrs[cnum], &tmp);
	else
		rdmsrl(pfm_pmu_conf->pmd_desc[cnum].hw_addr, tmp);

	PFM_DBG_ovfl("pfm_arch_read_pmd(0x%lx) = 0x%Lx",
		     pfm_pmu_conf->pmd_desc[cnum].hw_addr,
		     (unsigned long long) tmp);
	return tmp;
}

static inline u64 pfm_arch_read_pmc(struct pfm_context *ctx, unsigned int cnum)
{
	struct pfm_arch_pmu_info *arch_info = pfm_pmu_conf->arch_info;
	u64 tmp;
	if (arch_info->pmu_style == PFM_X86_PMU_P4)
		__pfm_read_reg_p4(&arch_info->pmc_addrs[cnum], &tmp);
	else
		rdmsrl(pfm_pmu_conf->pmc_desc[cnum].hw_addr, tmp);

	PFM_DBG_ovfl("pfm_arch_read_pmc(0x%lx) = 0x%016Lx",
		     pfm_pmu_conf->pmc_desc[cnum].hw_addr,
		     (unsigned long long) tmp);
	return tmp;
}

/*
 * At certain points, perfmon needs to know if monitoring has been
 * explicitely started/stopped by user via pfm_start/pfm_stop. The
 * information is tracked in flags.started. However on certain
 * architectures, it may be possible to start/stop directly from
 * user level with a single assembly instruction bypassing
 * the kernel. This function is used to determine by
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

static inline void pfm_arch_init_percpu(void)
{}

/* not necessary on IA-64 */
static inline void pfm_cacheflush(void *addr, unsigned int len)
{}

int  pfm_arch_ctxswout_thread(struct task_struct *task,
			      struct pfm_context *ctx,
			      struct pfm_event_set *set);

void pfm_arch_ctxswin_thread(struct task_struct *task,
			     struct pfm_context *ctx,
			     struct pfm_event_set *set);

void pfm_arch_stop(struct task_struct *task,
		   struct pfm_context *ctx, struct pfm_event_set *set);
void pfm_arch_start(struct task_struct *task,
		    struct pfm_context *ctx, struct pfm_event_set *set);

void pfm_arch_restore_pmds(struct pfm_context *ctx, struct pfm_event_set *set);
void pfm_arch_restore_pmcs(struct pfm_context *ctx, struct pfm_event_set *set);
int  pfm_arch_pmu_config_init(struct pfm_pmu_config *cfg);
void pfm_arch_pmu_config_remove(void);
char *pfm_arch_get_pmu_module_name(void);

static inline int pfm_arch_unload_context(struct pfm_context *ctx,
					  struct task_struct *task)
{
	struct pfm_arch_pmu_info *arch_info;
	struct pfm_arch_context *ctx_arch;
	int ret = 0;

	ctx_arch = pfm_ctx_arch(ctx);

	arch_info = pfm_pmu_conf->arch_info;
	if (arch_info->unload_context) {
		ret = arch_info->unload_context(ctx);
	}

	if (ctx_arch->flags.insecure) {
		PFM_DBG("clear cr4.pce");
		clear_in_cr4(X86_CR4_PCE);
	}

	return ret;
}

static inline int pfm_arch_load_context(struct pfm_context *ctx,
					struct pfm_event_set *set,
					struct task_struct *task)
{
	struct pfm_arch_pmu_info *arch_info;
	struct pfm_arch_context *ctx_arch;
	int ret = 0;

	ctx_arch = pfm_ctx_arch(ctx);

	/*
	 * RDPMC is automatically authorized in system-wide and
	 * also in self-monitoring per-thread context.
	 * It may be authorized in other situations if the
	 * PFM_X86_FL_INSECURE flags was set
	 */
	if (ctx->flags.system || task == current) {
		PFM_DBG("set cr4.pce");
		set_in_cr4(X86_CR4_PCE);
		ctx_arch->flags.insecure = 1;
	}

	arch_info = pfm_pmu_conf->arch_info;
	if (arch_info->load_context) {
		ret = arch_info->load_context(ctx);
	}
	return ret;
}

/*
 * this function is called from the PMU interrupt handler ONLY.
 * On x86, the PMU is frozen via arch_stop, masking would be implemented
 * via arch-stop as well. Given that the PMU is already stopped when
 * entering the interrupt handler, we do not need to stop it again, so
 * this function is a nop.
 */
static inline void pfm_arch_mask_monitoring(struct pfm_context *ctx,
					    struct pfm_event_set *set)
{}

/*
 * on x86 masking/unmasking uses the start/stop mechanism, so we simply
 * need to start here.
 */
static inline void pfm_arch_unmask_monitoring(struct pfm_context *ctx,
					      struct pfm_event_set *set)
{
	pfm_arch_start(current, ctx, set);
}

/*
 * called from __pfm_interrupt_handler(). ctx is not NULL.
 * ctx is locked. interrupts are masked
 *
 * The following actions must take place:
 *  - stop all monitoring to ensure handler has consistent view.
 *  - collect overflowed PMDs bitmask into povfls_pmds and
 *    npend_ovfls. If no interrupt detected then npend_ovfls
 *    must be set to zero.
 */
static inline void pfm_arch_intr_freeze_pmu(struct pfm_context *ctx,
					    struct pfm_event_set *set)
{
	/*
	 * on X86, freezing is equivalent to stopping
	 */
	pfm_arch_stop(current, ctx, set);

	/*
	 * we mark monitoring as stopped to avoid
	 * certain side effects especially in
	 * pfm_switch_sets_from_intr() and
	 * pfm_arch_restore_pmcs()
	 */
	ctx->flags.started = 0;
}

/*
 * unfreeze PMU from pfm_do_interrupt_handler().
 * ctx may be NULL for spurious interrupts.
 * interrupts are masked.
 */
static inline void pfm_arch_intr_unfreeze_pmu(struct pfm_context *ctx)
{
	if (ctx == NULL)
		return;

	PFM_DBG_ovfl("state=%d", ctx->state);

	/*
	 * restore flags.started which is cleared in
	 * pfm_arch_intr_freeze_pmu()
	 */
	ctx->flags.started = 1;

	if (ctx->state == PFM_CTX_MASKED)
		return;

	pfm_arch_restore_pmcs(ctx, ctx->active_set);
}


/*
 * function called from pfm_setfl_sane(). Context is locked
 * and interrupts are masked.
 * The value of flags is the value of ctx_flags as passed by
 * user.
 *
 * function must check arch-specific set flags.
 * Return:
 *      1 when flags are valid
 *      0 on error
 */
static inline int pfm_arch_setfl_sane(struct pfm_context *ctx, u32 flags)
{
	return 0;
}

int pfm_arch_pmu_acquire(void);
void pfm_arch_pmu_release(void);

/*
 * For some CPUs, the upper bits of a counter must be set in order for the
 * overflow interrupt to happen. On overflow, the counter has wrapped around,
 * and the upper bits are cleared. This function may be used to set them back.
 *
 * x86: The current version loses whatever is remaining in the counter,
 * which is usually has a small count. In order not to loose this count,
 * we do a read-modify-write to set the upper bits while preserving the
 * low-order bits. This is slow but works.
 */
static inline void pfm_arch_ovfl_reset_pmd(struct pfm_context *ctx, unsigned int cnum)
{
	u64 val;
	val = pfm_arch_read_pmd(ctx, cnum);
	pfm_arch_write_pmd(ctx, cnum, val);
}

/*
 * not used for i386/x86_64
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

static inline int pfm_arch_context_create(struct pfm_context *ctx, u32 ctx_flags)
{
	struct pfm_arch_pmu_info *arch_info = pfm_pmu_conf->arch_info;
	struct pfm_arch_context *ctx_arch;

	if (arch_info->pmu_style != PFM_X86_PMU_P4)
		return 0;

	ctx_arch = pfm_ctx_arch(ctx);

	ctx_arch->p4 = kzalloc(sizeof(*(ctx_arch->p4)), GFP_KERNEL);
	if (!ctx_arch->p4)
		return -ENOMEM;

	return 0;
}

static inline void pfm_arch_context_free(struct pfm_context *ctx)
{
	struct pfm_arch_context *ctx_arch;

	ctx_arch = pfm_ctx_arch(ctx);

	/*
	 * we do not check if P4, because it would be NULL and
	 * kfree can deal with NULL
	 */
	kfree(ctx_arch->p4);
}

static inline void pfm_arch_arm_handle_work(struct task_struct *task)
{
}

static inline void pfm_arch_disarm_handle_work(struct task_struct *task)
{
}
#define PFM_ARCH_CTX_SIZE	(sizeof(struct pfm_arch_context))
/*
 * i386/x86_64 do not need extra alignment requirements for the sampling buffer
 */
#define PFM_ARCH_SMPL_ALIGN_SIZE	0

asmlinkage void  pmu_interrupt(void);

#endif /* __KERNEL__ */

#endif /* _ASM_X86_PERFMON_H_ */
