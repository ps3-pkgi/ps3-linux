/*
 * This file contains the Intel Core PMU registers description tables.
 * Intel Core-based processors support architectural perfmon v2 + PEBS
 *
 * Copyright (c) 2006-2007 Hewlett-Packard Development Company, L.P.
 * Contributed by Stephane Eranian <eranian@hpl.hp.com>
 */
#include <linux/module.h>
#include <linux/perfmon.h>
#include <asm/nmi.h>

MODULE_AUTHOR("Stephane Eranian <eranian@hpl.hp.com>");
MODULE_DESCRIPTION("Intel Core");
MODULE_LICENSE("GPL");

static int force_nmi;
MODULE_PARM_DESC(force_nmi, "bool: force use of NMI for PMU interrupt");
module_param(force_nmi, bool, 0600);

/*
 * - upper 32 bits are reserved
 * - INT: APIC enable bit is reserved (forced to 1)
 * - bit 21 is reserved
 *
 *   RSVD: reserved bits must be 1
 */
#define PFM_CORE_PMC_RSVD ((~((1ULL<<32)-1)) \
			| (1ULL<<20)   \
			| (1ULL<<21))

/*
 * force Local APIC interrupt on overflow
 * disable with NO_EMUL64
 */
#define PFM_CORE_PMC_VAL	(1ULL<<20)
#define PFM_CORE_NO64		(1ULL<<20)

#define PFM_CORE_NA { .reg_type = PFM_REGT_NA}

#define PFM_CORE_CA(m, c, t) \
	{ \
	  .addrs[0] = m, \
	  .ctr = c, \
	  .reg_type = t \
	}
/*
 * physical addresses of MSR for evntsel and perfctr registers
 *
 * IMPORTANT:
 * 	The mapping  was chosen to be compatible with the Intel
 * 	architectural perfmon, so that applications which only
 * 	know about the architectural perfmon can work on Core
 * 	without any changes.
 *
 * 	We do not expose the GLOBAL_* registers because:
 * 	- would be incompatible with architectural perfmon v1
 * 	  (unless default means, measures everything for GLOBAL_CTRL)
 *	- would cause a conflict when NMI watchdog is enabled.
 *
 */
struct pfm_arch_pmu_info pfm_core_pmu_info={
	.pmc_addrs = {
/* pmc0  */	PFM_CORE_CA(MSR_P6_EVNTSEL0, 0, PFM_REGT_EN),
/* pmc1  */	PFM_CORE_CA(MSR_P6_EVNTSEL1, 1, PFM_REGT_EN),
/* pmc2  */	PFM_CORE_NA, PFM_CORE_NA,
/* pmc4  */	PFM_CORE_NA, PFM_CORE_NA, PFM_CORE_NA, PFM_CORE_NA,
/* pmc8  */	PFM_CORE_NA, PFM_CORE_NA, PFM_CORE_NA, PFM_CORE_NA,
/* pmc12 */	PFM_CORE_NA, PFM_CORE_NA, PFM_CORE_NA, PFM_CORE_NA,
/* pmc16 */	PFM_CORE_CA(MSR_CORE_PERF_FIXED_CTR_CTRL, 0, PFM_REGT_EN),
/* pmc17 */	PFM_CORE_CA(MSR_IA32_PEBS_ENABLE, 0, PFM_REGT_EN)
	},
	.pmd_addrs = {
/* pmd0  */	PFM_CORE_CA(MSR_P6_PERFCTR0, 0, PFM_REGT_CTR),
/* pmd1  */	PFM_CORE_CA(MSR_P6_PERFCTR1, 0, PFM_REGT_CTR),
/* pmd2  */	PFM_CORE_NA, PFM_CORE_NA,
/* pmd4  */	PFM_CORE_NA, PFM_CORE_NA, PFM_CORE_NA, PFM_CORE_NA,
/* pmd8  */	PFM_CORE_NA, PFM_CORE_NA, PFM_CORE_NA, PFM_CORE_NA,
/* pmd12 */	PFM_CORE_NA, PFM_CORE_NA, PFM_CORE_NA, PFM_CORE_NA,
/* pmd16 */	PFM_CORE_CA(MSR_CORE_PERF_FIXED_CTR0, 0, PFM_REGT_CTR),
/* pmd17 */	PFM_CORE_CA(MSR_CORE_PERF_FIXED_CTR1, 0, PFM_REGT_CTR),
/* pmd18 */	PFM_CORE_CA(MSR_CORE_PERF_FIXED_CTR2, 0, PFM_REGT_CTR)
	},
	.pebs_ctr_idx = 0, /* IA32_PMC0 */
	.pmu_style = PFM_X86_PMU_CORE
};

static struct pfm_regmap_desc pfm_core_pmc_desc[]={
/* pmc0  */ {
	      .type = PFM_REG_I64,
	      .desc = "PERFEVTSEL0",
	      .dfl_val = PFM_CORE_PMC_VAL,
	      .rsvd_msk = PFM_CORE_PMC_RSVD,
	      .no_emul64_msk = PFM_CORE_NO64,
	      .hw_addr = MSR_P6_EVNTSEL0
	    },
/* pmc1  */ {
	      .type = PFM_REG_I64,
	      .desc = "PERFEVTSEL1",
	      .dfl_val = PFM_CORE_PMC_VAL,
	      .rsvd_msk = PFM_CORE_PMC_RSVD,
	      .no_emul64_msk = PFM_CORE_NO64,
	      .hw_addr = MSR_P6_EVNTSEL1
	    },
/* pmc2  */ PMX_NA, PMX_NA,
/* pmc4  */ PMX_NA, PMX_NA, PMX_NA, PMX_NA,
/* pmc8  */ PMX_NA, PMX_NA, PMX_NA, PMX_NA,
/* pmc12 */ PMX_NA, PMX_NA, PMX_NA, PMX_NA,
/* pmc16 */ { .type = PFM_REG_I,
	      .desc = "FIXED_CTRL",
	      .dfl_val = 0x888ULL,
	      .rsvd_msk = 0xfffffffffffffcccULL,
	      .no_emul64_msk = 0,
	      .hw_addr = MSR_CORE_PERF_FIXED_CTR_CTRL
	    },
/* pmc17  */ { .type = PFM_REG_W,
	      .desc = "PEBS_ENABLE",
	      .dfl_val = 0,
	      .rsvd_msk = 0xfffffffffffffffeULL,
	      .no_emul64_msk = 0,
	      .hw_addr = MSR_IA32_PEBS_ENABLE
	    }
};

#define PFM_CORE_D(n) PMD_D(PFM_REG_C, "PMC"#n, MSR_P6_PERFCTR0+n)
#define PFM_CORE_FD(n) PMD_D(PFM_REG_C, "FIXED_CTR"#n, MSR_CORE_PERF_FIXED_CTR0+n)

static struct pfm_regmap_desc pfm_core_pmd_desc[]={
/* pmd0  */ PFM_CORE_D(0),
/* pmd1  */ PFM_CORE_D(1),
/* pmd2  */ PMX_NA, PMX_NA,
/* pmd4  */ PMX_NA, PMX_NA, PMX_NA, PMX_NA,
/* pmd8  */ PMX_NA, PMX_NA, PMX_NA, PMX_NA,
/* pmd12 */ PMX_NA, PMX_NA, PMX_NA, PMX_NA,
/* pmd16 */ PFM_CORE_FD(0),
/* pmd17 */ PFM_CORE_FD(1),
/* pmd18 */ PFM_CORE_FD(2)
};
#define PFM_CORE_NUM_PMCS	ARRAY_SIZE(pfm_core_pmc_desc)
#define PFM_CORE_NUM_PMDS	ARRAY_SIZE(pfm_core_pmd_desc)

static struct pfm_pmu_config pfm_core_pmu_conf;

static int pfm_core_probe_pmu(void)
{
	unsigned int i;

	/*
	 * Check for Intel Core processor explicitely
	 * Checking for cpu_has_perfmon is not enough as this
	 * matches intel Core Duo/Core Solo but none supports
	 * PEBS.
	 *
	 * Intel Core = arch perfmon v2 + PEBS
	 */
	if (current_cpu_data.x86 != 6)
		return -1;

	switch(current_cpu_data.x86_model) {
		case 15: /* Merom */
			break;
		case 23: /* Penryn */
			break;
		default:
			return -1;
	}

	if (!cpu_has_apic) {
		PFM_INFO("no Local APIC, unsupported");
		return -1;
	}

	PFM_INFO("nmi_watchdog=%d nmi_active=%d force_nmi=%d",
		nmi_watchdog, atomic_read(&nmi_active), force_nmi);

	/*
	 * Intel Core processors implement DS and PEBS, no need to check
	 */
	if (cpu_has_pebs) {
		pfm_core_pmu_info.flags |= PFM_X86_FL_PMU_DS|PFM_X86_FL_PMU_PEBS;
		PFM_INFO("PEBS supported, enabled");
	}

	/*
	 * Core 2 have 40-bit counters (generic, fixed)
	 */
	for(i=0; i < PFM_CORE_NUM_PMDS; i++)
		pfm_core_pmd_desc[i].rsvd_msk = ~((1ULL<<40)-1);

	if (force_nmi)
		pfm_core_pmu_info.flags |= PFM_X86_FL_USE_NMI;

	return 0;
}

static int pfm_core_pmc17_check(struct pfm_context *ctx,
			     struct pfm_event_set *set,
			     struct pfarg_pmc *req)
{
	struct pfm_arch_context *ctx_arch;
	ctx_arch = pfm_ctx_arch(ctx);

	/*
	 * if user activates PEBS_ENABLE, then we need to have a valid
	 * DS Area setup. This only happens when the PEBS sampling format is used
	 * in which case PFM_X86_USE_PEBS is set. We must reject all other requests.
	 * Otherwise we may pickup stale MSR_IA32_DS_AREA values. It appears
	 * that a value of 0 for this MSR does crash the system with PEBS_ENABLE=1.
	 */
	if (!ctx_arch->flags.use_pebs && req->reg_value) {
		PFM_DBG("pmc17 (PEBS_ENABLE) can only be used with PEBS sampling format");
		return -EINVAL;
	}
	return 0;
}

/*
 * Counters may have model-specific width which can be probed using
 * the CPUID.0xa leaf. Yet, the documentation says: "
 * In the initial implementation, only the read bit width is reported
 * by CPUID, write operations are limited to the low 32 bits.
 * Bits [w-32] are sign extensions of bit 31. As such the effective width
 * of a counter is 31 bits only.
 */
static struct pfm_pmu_config pfm_core_pmu_conf={
	.pmu_name = "Intel Core",
	.pmd_desc = pfm_core_pmd_desc,
	.counter_width = 31,
	.num_pmc_entries = PFM_CORE_NUM_PMCS,
	.num_pmd_entries = PFM_CORE_NUM_PMDS,
	.pmc_desc = pfm_core_pmc_desc,
	.probe_pmu = pfm_core_probe_pmu,
	.version = "1.2",
	.flags = PFM_PMU_BUILTIN_FLAG,
	.owner = THIS_MODULE,
	.arch_info = &pfm_core_pmu_info,
	.pmc_write_check = pfm_core_pmc17_check
};

static int __init pfm_core_pmu_init_module(void)
{
	return pfm_pmu_register(&pfm_core_pmu_conf);
}

static void __exit pfm_core_pmu_cleanup_module(void)
{
	pfm_pmu_unregister(&pfm_core_pmu_conf);
}

module_init(pfm_core_pmu_init_module);
module_exit(pfm_core_pmu_cleanup_module);
