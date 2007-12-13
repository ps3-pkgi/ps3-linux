/*
 * This file contains the POWER5 PMU register description tables
 * and pmc checker used by perfmon.c.
 *
 * Copyright (c) 2005 David Gibson, IBM Corporation.
 *
 * Based on perfmon_p6.c:
 * Copyright (c) 2005-2006 Hewlett-Packard Development Company, L.P.
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
#include <linux/perfmon.h>

MODULE_AUTHOR("David Gibson <dwg@au1.ibm.com>");
MODULE_DESCRIPTION("POWER5 PMU description table");
MODULE_LICENSE("GPL");

static struct pfm_regmap_desc pfm_power5_pmc_desc[]={
/* mmcr0 */ PMC_D(PFM_REG_I, "MMCR0", MMCR0_FC, 0, 0, SPRN_MMCR0),
/* mmcr1 */ PMC_D(PFM_REG_I, "MMCR1", 0, 0, 0, SPRN_MMCR1),
/* mmcra */ PMC_D(PFM_REG_I, "MMCRA", 0, 0, 0, SPRN_MMCRA)
};
#define PFM_PM_NUM_PMCS	ARRAY_SIZE(pfm_power5_pmc_desc)

/* The TB and PURR registers are read-only. Also, note that the TB register
 * actually consists of both the 32-bit SPRN_TBRU and SPRN_TBRL registers.
 * For Perfmon2's purposes, we'll treat it as a single 64-bit register.
 */
static struct pfm_regmap_desc pfm_power5_pmd_desc[]={
/* tb    */ PMD_D((PFM_REG_I|PFM_REG_RO), "TB", SPRN_TBRL),
/* pmd1  */ PMD_D(PFM_REG_C, "PMC1", SPRN_PMC1),
/* pmd2  */ PMD_D(PFM_REG_C, "PMC2", SPRN_PMC2),
/* pmd3  */ PMD_D(PFM_REG_C, "PMC3", SPRN_PMC3),
/* pmd4  */ PMD_D(PFM_REG_C, "PMC4", SPRN_PMC4),
/* pmd5  */ PMD_D(PFM_REG_C, "PMC5", SPRN_PMC5),
/* pmd6  */ PMD_D(PFM_REG_C, "PMC6", SPRN_PMC6),
/* purr  */ PMD_D((PFM_REG_I|PFM_REG_RO), "PURR", SPRN_PURR),
};
#define PFM_PM_NUM_PMDS	ARRAY_SIZE(pfm_power5_pmd_desc)

static int pfm_power5_probe_pmu(void)
{
	unsigned long pvr = mfspr(SPRN_PVR);

	if (PVR_VER(pvr) != PV_POWER5)
		return -1;

	return 0;
}

static void pfm_power5_write_pmc(unsigned int cnum, u64 value)
{
	switch (pfm_pmu_conf->pmc_desc[cnum].hw_addr) {
	case SPRN_MMCR0:
		mtspr(SPRN_MMCR0, value);
		break;
	case SPRN_MMCR1:
		mtspr(SPRN_MMCR1, value);
		break;
	case SPRN_MMCRA:
		mtspr(SPRN_MMCRA, value);
		break;
	default:
		BUG();
	}
}

static void pfm_power5_write_pmd(unsigned int cnum, u64 value)
{
	switch (pfm_pmu_conf->pmd_desc[cnum].hw_addr) {
	case SPRN_PMC1:
		mtspr(SPRN_PMC1, value);
		break;
	case SPRN_PMC2:
		mtspr(SPRN_PMC2, value);
		break;
	case SPRN_PMC3:
		mtspr(SPRN_PMC3, value);
		break;
	case SPRN_PMC4:
		mtspr(SPRN_PMC4, value);
		break;
	case SPRN_PMC5:
		mtspr(SPRN_PMC5, value);
		break;
	case SPRN_PMC6:
		mtspr(SPRN_PMC6, value);
		break;
	case SPRN_PMC7:
		mtspr(SPRN_PMC7, value);
		break;
	case SPRN_PMC8:
		mtspr(SPRN_PMC8, value);
		break;
	case SPRN_TBRL:
	case SPRN_PURR:
		/* Ignore writes to read-only registers. */
		break;
	default:
		BUG();
	}
}

static u64 pfm_power5_read_pmd(unsigned int cnum)
{
	switch (pfm_pmu_conf->pmd_desc[cnum].hw_addr) {
	case SPRN_PMC1:
		return mfspr(SPRN_PMC1);
	case SPRN_PMC2:
		return mfspr(SPRN_PMC2);
	case SPRN_PMC3:
		return mfspr(SPRN_PMC3);
	case SPRN_PMC4:
		return mfspr(SPRN_PMC4);
	case SPRN_PMC5:
		return mfspr(SPRN_PMC5);
	case SPRN_PMC6:
		return mfspr(SPRN_PMC6);
	case SPRN_PMC7:
		return mfspr(SPRN_PMC7);
	case SPRN_PMC8:
		return mfspr(SPRN_PMC8);
	case SPRN_TBRL:
		return ((u64)mfspr(SPRN_TBRU) << 32) | mfspr(SPRN_TBRL);
	case SPRN_PURR:
		if (cpu_has_feature(CPU_FTR_PURR))
			return mfspr(SPRN_PURR);
		else
			return 0;
	default:
		BUG();
	}
}

/**
 * pfm_power5_enable_counters
 *
 * Just need to load the current values into the control registers.
 **/
static void pfm_power5_enable_counters(struct pfm_context *ctx,
				       struct pfm_event_set *set)
{
	unsigned int i, max_pmc;

	max_pmc = pfm_pmu_conf->regs.max_pmc;

	for (i = 0; i < max_pmc; i++)
		if (test_bit(i, set->used_pmcs))
			pfm_power5_write_pmc(i, set->pmcs[i]);
}

/**
 * pfm_power5_disable_counters
 *
 * Just need to zero all the control registers.
 **/
static void pfm_power5_disable_counters(struct pfm_context *ctx,
					struct pfm_event_set *set)
{
	unsigned int i, max;

	max = pfm_pmu_conf->regs.max_pmc;

	for (i = 0; i < max; i++)
		if (test_bit(i, set->used_pmcs))
			pfm_power5_write_pmc(i, 0);
}

/**
 * pfm_power5_get_ovfl_pmds
 *
 * Determine which counters in this set have overflowed and fill in the
 * set->povfl_pmds mask and set->npend_ovfls count.
 **/
static void pfm_power5_get_ovfl_pmds(struct pfm_context *ctx,
				     struct pfm_event_set *set)
{
	unsigned int i;
	unsigned int max = pfm_pmu_conf->regs.max_intr_pmd;
	u64 *used_pmds = set->used_pmds;
	u64 *intr_pmds = pfm_pmu_conf->regs.intr_pmds;
	u64 width_mask = 1 << pfm_pmu_conf->counter_width;
	u64 new_val, mask[PFM_PMD_BV];

	bitmap_and(cast_ulp(mask), cast_ulp(intr_pmds),
		   cast_ulp(used_pmds), max);

	for (i = 0; i < max; i++) {
		if (test_bit(i, mask)) {
			new_val = pfm_power5_read_pmd(i);
			if (new_val & width_mask) {
				set_bit(i, set->povfl_pmds);
				set->npend_ovfls++;
			}
		}
	}
}

static void pfm_power5_irq_handler(struct pt_regs *regs,
				   struct pfm_context *ctx)
{
	u32 mmcr0;
	u64 mmcra;

	/* Disable the counters (set the freeze bit) to not polute
	 * the counts.
	 */
	mmcr0 = mfspr(SPRN_MMCR0);
	mtspr(SPRN_MMCR0, (mmcr0 | MMCR0_FC));
	mmcra = mfspr(SPRN_MMCRA);

	/* Set the PMM bit (see comment below). */
	mtmsrd(mfmsr() | MSR_PMM);

	pfm_interrupt_handler(instruction_pointer(regs), regs);

	mmcr0 = mfspr(SPRN_MMCR0);
	/* Reset the perfmon trigger. */
	mmcr0 |= MMCR0_PMXE;

	/*
	 * We must clear the PMAO bit on some (GQ) chips. Just do it
	 * all the time.
	 */
	mmcr0 &= ~MMCR0_PMAO;

	/* Clear the appropriate bits in the MMCRA. */
	mmcra &= POWER6_MMCRA_THRM | POWER6_MMCRA_OTHER;
	mtspr(SPRN_MMCRA, mmcra);

	/*
	 * Now clear the freeze bit, counting will not start until we
	 * rfid from this exception, because only at that point will
	 * the PMM bit be cleared.
	 */
	mmcr0 &= ~MMCR0_FC;
	mtspr(SPRN_MMCR0, mmcr0);
}

struct pfm_arch_pmu_info pfm_power5_pmu_info = {
	.pmu_style        = PFM_POWERPC_PMU_POWER5,
	.write_pmc        = pfm_power5_write_pmc,
	.write_pmd        = pfm_power5_write_pmd,
	.read_pmd         = pfm_power5_read_pmd,
	.irq_handler      = pfm_power5_irq_handler,
	.get_ovfl_pmds    = pfm_power5_get_ovfl_pmds,
	.enable_counters  = pfm_power5_enable_counters,
	.disable_counters = pfm_power5_disable_counters,
};

/*
 * impl_pmcs, impl_pmds are computed at runtime to minimize errors!
 */
static struct pfm_pmu_config pfm_power5_pmu_conf = {
	.pmu_name = "POWER5",
	.counter_width = 31,
	.pmd_desc = pfm_power5_pmd_desc,
	.pmc_desc = pfm_power5_pmc_desc,
	.num_pmc_entries = PFM_PM_NUM_PMCS,
	.num_pmd_entries = PFM_PM_NUM_PMDS,
	.probe_pmu  = pfm_power5_probe_pmu,
	.arch_info = &pfm_power5_pmu_info,
	.flags = PFM_PMU_BUILTIN_FLAG,
	.owner = THIS_MODULE
};

static int __init pfm_power5_pmu_init_module(void)
{
	return pfm_pmu_register(&pfm_power5_pmu_conf);
}

static void __exit pfm_power5_pmu_cleanup_module(void)
{
	pfm_pmu_unregister(&pfm_power5_pmu_conf);
}

module_init(pfm_power5_pmu_init_module);
module_exit(pfm_power5_pmu_cleanup_module);
