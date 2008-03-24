/*
 * This file contains the Intel architectural perfmon v1 or v2
 * description tables.
 *
 * Architectural perfmon was introduced with Intel Core Solo/Duo
 * processors.
 *
 * Copyright (c) 2006-2007 Hewlett-Packard Development Company, L.P.
 * Contributed by Stephane Eranian <eranian@hpl.hp.com>
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
#include <linux/perfmon_kern.h>
#include <asm/msr.h>
#include <asm/apic.h>
#include <asm/nmi.h>

MODULE_AUTHOR("Stephane Eranian <eranian@hpl.hp.com>");
MODULE_DESCRIPTION("Intel architectural perfmon v1");
MODULE_LICENSE("GPL");

static int force, force_nmi;
MODULE_PARM_DESC(force, "bool: force module to load succesfully");
MODULE_PARM_DESC(force_nmi, "bool: force use of NMI for PMU interrupt");
module_param(force, bool, 0600);
module_param(force_nmi, bool, 0600);

/*
 * - upper 32 bits are reserved
 * - INT: APIC enable bit is reserved (forced to 1)
 * - bit 21 is reserved
 *
 * RSVD: reserved bits are 1
 */
#define PFM_IA_PMC_RSVD	((~((1ULL<<32)-1)) \
			| (1ULL<<20) \
			| (1ULL<<21))

/*
 * force Local APIC interrupt on overflow
 * disable with NO_EMUL64
 */
#define PFM_IA_PMC_VAL	(1ULL<<20)
#define PFM_IA_NO64	(1ULL<<20)

/*
 * architectuture specifies that:
 * IA32_PMCx MSR        : starts at 0x0c1 & occupy a contiguous block of MSR
 * IA32_PERFEVTSELx MSR : starts at 0x186 & occupy a contiguous block of MSR
 * MSR_GEN_FIXED_CTR0   : starts at 0x309 & occupy a contiguous block of MSR
 */
#define MSR_GEN_SEL_BASE	MSR_P6_EVNTSEL0
#define MSR_GEN_PMC_BASE	MSR_P6_PERFCTR0
#define MSR_GEN_FIXED_PMC_BASE	MSR_CORE_PERF_FIXED_CTR0

#define PFM_IA_SEL(n)	{ 			\
	.addrs[0] = MSR_GEN_SEL_BASE+(n),	\
	.ctr = n,				\
	.reg_type = PFM_REGT_EN}

#define PFM_IA_CTR(n) {				\
	.addrs[0] = MSR_GEN_PMC_BASE+(n),	\
	.ctr = n,				\
	.reg_type = PFM_REGT_CTR}

#define PFM_IA_FCTR(n) {			\
	.addrs[0] = MSR_GEN_FIXED_PMC_BASE+(n),	\
	.ctr = n,				\
	.reg_type = PFM_REGT_CTR}

/*
 * layout of EAX for CPUID.0xa leaf function
 */
struct pmu_eax {
        unsigned int version:8;		/* architectural perfmon version */
        unsigned int num_cnt:8; 	/* number of generic counters */
        unsigned int cnt_width:8;	/* width of generic counters */
        unsigned int ebx_length:8;	/* number of architected events */
};

/*
 * layout of EDX for CPUID.0xa leaf function when perfmon v2 is detected
 */
struct pmu_edx {
        unsigned int num_cnt:5;		/* number of fixed counters */
        unsigned int cnt_width:8;	/* width of fixed counters */
        unsigned int reserved:19;
};


/*
 * physical addresses of MSR controlling the perfevtsel and counter registers
 */
struct pfm_arch_pmu_info pfm_intel_arch_pmu_info={
	.pmc_addrs = {
/* pmc0  */	PFM_IA_SEL(0) ,  PFM_IA_SEL(1),  PFM_IA_SEL(2),  PFM_IA_SEL(3),
/* pmc4  */	PFM_IA_SEL(4) ,  PFM_IA_SEL(5),  PFM_IA_SEL(6),  PFM_IA_SEL(7),
/* pmc8  */	PFM_IA_SEL(8) ,  PFM_IA_SEL(9), PFM_IA_SEL(10), PFM_IA_SEL(11),
/* pmc12 */	PFM_IA_SEL(12), PFM_IA_SEL(13), PFM_IA_SEL(14), PFM_IA_SEL(15),

/* pmc16 */	{
			.addrs[0] = MSR_CORE_PERF_FIXED_CTR_CTRL,
			.reg_type = PFM_REGT_EN
		}
	},

	.pmd_addrs = {
/* pmd0  */	PFM_IA_CTR(0) ,  PFM_IA_CTR(1),  PFM_IA_CTR(2),  PFM_IA_CTR(3),
/* pmd4  */	PFM_IA_CTR(4) ,  PFM_IA_CTR(5),  PFM_IA_CTR(6),  PFM_IA_CTR(7),
/* pmd8  */	PFM_IA_CTR(8) ,  PFM_IA_CTR(9), PFM_IA_CTR(10), PFM_IA_CTR(11),
/* pmd12 */	PFM_IA_CTR(12), PFM_IA_CTR(13), PFM_IA_CTR(14), PFM_IA_CTR(15),

/* pmd16 */	PFM_IA_FCTR(0), PFM_IA_FCTR(1), PFM_IA_FCTR(2), PFM_IA_FCTR(3),
/* pmd20 */	PFM_IA_FCTR(4), PFM_IA_FCTR(5), PFM_IA_FCTR(6), PFM_IA_FCTR(7),
/* pmd24 */	PFM_IA_FCTR(8), PFM_IA_FCTR(9), PFM_IA_FCTR(10), PFM_IA_FCTR(11),
/* pmd28 */	PFM_IA_FCTR(12), PFM_IA_FCTR(13), PFM_IA_FCTR(14), PFM_IA_FCTR(15)
	},
	.pmu_style = PFM_X86_PMU_P6
};

#define PFM_IA_C(n) {                   \
	.type = PFM_REG_I64,            \
	.desc = "PERFEVTSEL"#n,         \
	.dfl_val = PFM_IA_PMC_VAL,      \
	.rsvd_msk = PFM_IA_PMC_RSVD,    \
	.no_emul64_msk = PFM_IA_NO64,   \
	.hw_addr = MSR_GEN_SEL_BASE+(n) \
	}

#define PFM_IA_D(n) PMD_D(PFM_REG_C, "PMC"#n, MSR_P6_PERFCTR0+n)
#define PFM_IA_FD(n) PMD_D(PFM_REG_C, "FIXED_CTR"#n, MSR_CORE_PERF_FIXED_CTR0+n)

static struct pfm_regmap_desc pfm_intel_arch_pmc_desc[]={
/* pmc0  */ PFM_IA_C(0),  PFM_IA_C(1),   PFM_IA_C(2),  PFM_IA_C(3),
/* pmc4  */ PFM_IA_C(4),  PFM_IA_C(5),   PFM_IA_C(6),  PFM_IA_C(7),
/* pmc8  */ PFM_IA_C(8),  PFM_IA_C(9),  PFM_IA_C(10), PFM_IA_C(11),
/* pmc12 */ PFM_IA_C(12), PFM_IA_C(13), PFM_IA_C(14), PFM_IA_C(15),

/* pmc16 */ { .type = PFM_REG_I,
	      .desc = "FIXED_CTRL",
	      .dfl_val = 0x8888888888888888ULL,
	      .rsvd_msk = 0xccccccccccccccccULL,
	      .no_emul64_msk = 0,
	      .hw_addr = MSR_CORE_PERF_FIXED_CTR_CTRL
	    },
};
#define PFM_IA_MAX_PMCS	ARRAY_SIZE(pfm_intel_arch_pmc_desc)

static struct pfm_regmap_desc pfm_intel_arch_pmd_desc[]={
/* pmd0  */  PFM_IA_D(0),  PFM_IA_D(1),  PFM_IA_D(2),  PFM_IA_D(3),
/* pmd4  */  PFM_IA_D(4),  PFM_IA_D(5),  PFM_IA_D(6),  PFM_IA_D(7),
/* pmd8  */  PFM_IA_D(8),  PFM_IA_D(9), PFM_IA_D(10), PFM_IA_D(11),
/* pmd12 */ PFM_IA_D(12), PFM_IA_D(13), PFM_IA_D(14), PFM_IA_D(15),

/* pmd16 */ PFM_IA_FD(0), PFM_IA_FD(1), PFM_IA_FD(2), PFM_IA_FD(3),
/* pmd20 */ PFM_IA_FD(4), PFM_IA_FD(5), PFM_IA_FD(6), PFM_IA_FD(7),
/* pmd24 */ PFM_IA_FD(8), PFM_IA_FD(9), PFM_IA_FD(10), PFM_IA_FD(11),
/* pmd28 */ PFM_IA_FD(16), PFM_IA_FD(17), PFM_IA_FD(18), PFM_IA_FD(19)
};
#define PFM_IA_MAX_PMDS	ARRAY_SIZE(pfm_intel_arch_pmd_desc)

#define PFM_IA_MAX_CNT		16 /* maximum # of generic counters in mapping table */
#define PFM_IA_MAX_FCNT		16 /* maximum # of fixed counters in mapping table */
#define PFM_IA_FCNT_BASE	16 /* base index of fixed counters PMD */

static struct pfm_pmu_config pfm_intel_arch_pmu_conf;

static int pfm_intel_arch_check_errata(void)
{
	/*
	 * Core Duo errata AE49 (no fix). Both counters share a single
	 * enable bit in PERFEVTSEL0
	 */
	if (current_cpu_data.x86 == 6 && current_cpu_data.x86_model == 14) {
		pfm_intel_arch_pmu_info.flags |= PFM_X86_FL_NO_SHARING;
	}
	return 0;
}

static int pfm_intel_arch_probe_pmu(void)
{
	union {
		unsigned int val;
		struct pmu_eax eax;
		struct pmu_edx edx;
	} eax, edx;
	unsigned int ebx, ecx;
	unsigned int num_cnt, i;
	u64 dfl, rsvd;

	edx.val = 0;

	if (!cpu_has_arch_perfmon && force == 0) {
		PFM_INFO("no support for Intel architectural PMU");
		return -1;
	}

	if (!cpu_has_apic) {
		PFM_INFO("no Local APIC, try rebooting with lapic option");
		return -1;
	}

	if (pfm_intel_arch_check_errata())
		return -1;

	if (force == 0) {
		/* cpuid() call protected by cpu_has_arch_perfmon */
		cpuid(0xa, &eax.val, &ebx, &ecx, &edx.val);
	} else {
		/* lowest common denominator */
		eax.eax.version = 1;
		eax.eax.num_cnt = 2;
		eax.eax.cnt_width = 40;
		edx.val = 0;
	}
	/*
	 * reject processors supported by perfmon_intel_core
	 *
	 * We need to do this explicitely to avoid depending
	 * on the link order in case, the modules are compiled as
	 * builtin.
	 *
	 * non Intel processors are rejected by cpu_has_arch_perfmon
	 */
	if (current_cpu_data.x86 == 6) {
		switch(current_cpu_data.x86_model) {
			case 15: /* Merom: use perfmon_intel_core  */
			case 23: /* Penryn: use perfmon_intel_core */
				return -1;
			default:
				break;
		}
	}

	/*
	 * some 6/15 models have buggy BIOS
	 */
	if (eax.eax.version == 0
	    && current_cpu_data.x86 == 6 && current_cpu_data.x86_model == 15) {
		PFM_INFO("buggy v2 BIOS, adjusting for 2 generic counters");
		eax.eax.version = 2;
		eax.eax.num_cnt = 2;
		eax.eax.cnt_width = 40;
	}

	/*
	 * some v2 BIOSes are incomplete
	 */
	if (eax.eax.version == 2 && !edx.edx.num_cnt) {
		PFM_INFO("buggy v2 BIOS, adjusting for 3 fixed counters");
		edx.edx.num_cnt = 3;
		edx.edx.cnt_width = 40;
	}

	/*
	 * no fixed counters on earlier versions
	 */
	if (eax.eax.version < 2)
		edx.val = 0;

	PFM_INFO("detected architecural perfmon v%d", eax.eax.version);
	PFM_INFO("num_gen=%d width=%d num_fixed=%d width=%d",
		  eax.eax.num_cnt,
		  eax.eax.cnt_width,
		  edx.edx.num_cnt,
		  edx.edx.cnt_width);

	/* number of generic counters */
	num_cnt = eax.eax.num_cnt;

	if (num_cnt >= PFM_IA_MAX_CNT) {
		printk(KERN_INFO "perfmon: Limiting number of generic counters to %zu,"
				 "HW supports %u", PFM_IA_MAX_PMCS, num_cnt);
		num_cnt = PFM_IA_MAX_CNT;

	}

	/*
	 * adjust rsvd_msk for generic counters based on actual width
	 */
	for(i=0; i < num_cnt; i++)
		pfm_intel_arch_pmd_desc[i].rsvd_msk = ~((1ULL<<eax.eax.cnt_width)-1);

	/*
	 * mark unused generic counters as not available
	 */
	for(i=num_cnt; i < PFM_IA_MAX_CNT; i++) {
		pfm_intel_arch_pmd_desc[i].type = PFM_REG_NA;
		pfm_intel_arch_pmc_desc[i].type = PFM_REG_NA;
	}

	/*
	 * now process fixed counters (if any)
	 */
	num_cnt = edx.edx.num_cnt;

	/*
	 * adjust rsvd_msk for fixed counters based on actual width
	 */
	for(i=0; i < num_cnt; i++)
		pfm_intel_arch_pmd_desc[PFM_IA_FCNT_BASE+i].rsvd_msk = ~((1ULL<<edx.edx.cnt_width)-1);

	/*
	 * mark unused fixed counters as
	 * unavailable.
	 * update the rsvd_msk, dfl_val for
	 * FIXED_CTRL:
	 * 	rsvd_msk: set all 4 bits
	 *	dfl_val : clear all 4 bits
	 */
	dfl = pfm_intel_arch_pmc_desc[16].dfl_val;
	rsvd = pfm_intel_arch_pmc_desc[16].rsvd_msk;

	for(i=num_cnt; i < PFM_IA_MAX_FCNT; i++) {
		pfm_intel_arch_pmd_desc[PFM_IA_FCNT_BASE+i].type = PFM_REG_NA;
		rsvd |= 0xfULL << (i<<2);
		dfl &= ~(0xfULL << (i<<2));
	}

	/*
	 * FIXED_CTR_CTRL unavailable when no fixed counters are defined
	 */
	if (!num_cnt) {
		pfm_intel_arch_pmc_desc[16].type = PFM_REG_NA;
	} else {
		pfm_intel_arch_pmc_desc[16].rsvd_msk = rsvd;
		pfm_intel_arch_pmc_desc[16].dfl_val = dfl;
	}

	/*
	 * Maximum number of entries in both tables (some maybe NA)
	 */
	pfm_intel_arch_pmu_conf.num_pmc_entries = PFM_IA_MAX_PMCS;
	pfm_intel_arch_pmu_conf.num_pmd_entries = PFM_IA_MAX_PMDS;

	if (force_nmi)
		pfm_intel_arch_pmu_info.flags |= PFM_X86_FL_USE_NMI;


	return 0;
}

/*
 * Counters may have model-specific width. Yet the documentation says
 * that only the lower 32 bits can be written to due to the specification
 * of wrmsr. bits [32-(w-1)] are sign extensions of bit 31. Bits [w-63] must
 * not be set (see rsvd_msk for PMDs). As such the effective width of a
 * counter is 31 bits only regardless of what CPUID.0xa returns.
 *
 * See IA-32 Intel Architecture Software developer manual Vol 3B chapter 18
 */
static struct pfm_pmu_config pfm_intel_arch_pmu_conf={
	.pmu_name = "Intel architectural",
	.pmd_desc = pfm_intel_arch_pmd_desc,
	.counter_width   = 31,
	.pmc_desc = pfm_intel_arch_pmc_desc,
	.probe_pmu = pfm_intel_arch_probe_pmu,
	.version = "1.0",
	.flags = PFM_PMU_BUILTIN_FLAG,
	.owner = THIS_MODULE,
	.arch_info = &pfm_intel_arch_pmu_info
};

static int __init pfm_intel_arch_pmu_init_module(void)
{
	return pfm_pmu_register(&pfm_intel_arch_pmu_conf);
}

static void __exit pfm_intel_arch_pmu_cleanup_module(void)
{
	pfm_pmu_unregister(&pfm_intel_arch_pmu_conf);
}

module_init(pfm_intel_arch_pmu_init_module);
module_exit(pfm_intel_arch_pmu_cleanup_module);
