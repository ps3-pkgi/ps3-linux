/*
 * Copyright (c) 2006 Hewlett-Packard Development Company, L.P.
 * Contributed by Stephane Eranian <eranian@hpl.hp.com>
 *
 * Interface for PMU description modules
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
#ifndef __PERFMON_PMU_H__
#define __PERFMON_PMU_H__ 1

/*
 * generic information about a PMC or PMD register
 */
struct pfm_regmap_desc {
	u16  type;		/* role of the register */
	u16  reserved1;		/* for future use */
	u32  reserved2;		/* for future use */
	u64  dfl_val;		/* power-on default value (quiescent) */
	u64  rsvd_msk;		/* reserved bits: 1 means reserved */
	u64  no_emul64_msk;	/* bits to clear for PFM_REGFL_NO_EMUL64 */
	unsigned long hw_addr;	/* HW register address or index */
	struct kobject	kobj;	/* for internal use only */
	char *desc;		/* HW register description string */
};
#define to_reg(n) container_of(n, struct pfm_regmap_desc, kobj)

/*
 * pfm_reg_desc helper macros
 */
#define PMC_D(t,d,v,r,n, h)   \
	{ .type = t,          \
	  .desc = d,          \
	  .dfl_val = v,       \
	  .rsvd_msk = r,      \
	  .no_emul64_msk = n, \
	  .hw_addr = h	      \
	}

#define PMD_D(t,d, h)         \
	{ .type = t,          \
	  .desc = d,          \
	  .rsvd_msk = 0,      \
	  .no_emul64_msk = 0, \
	  .hw_addr = h	      \
	}

#define PMX_NA \
	{ .type = PFM_REG_NA }

/*
 * type of a PMU register (16-bit bitmask) for use with pfm_reg_desc.type
 */
#define PFM_REG_NA	0x00  /* not avail. (not impl.,no access) must be 0 */
#define PFM_REG_I	0x01  /* PMC/PMD: implemented */
#define PFM_REG_WC	0x02  /* PMC: has write_checker */
#define PFM_REG_C64	0x04  /* PMD: 64-bit virtualization */
#define PFM_REG_RO	0x08  /* PMD: read-only (writes ignored) */
#define PFM_REG_V	0x10  /* PMD: virtual reg (provided by PMU description) */
#define PFM_REG_INTR	0x20  /* PMD: register can generate interrupt */
#define PFM_REG_NO64	0x100 /* PMC: supports PFM_REGFL_NO_EMUL64 */

/*
 * define some shortcuts for common types
 */
#define PFM_REG_W	(PFM_REG_WC|PFM_REG_I)
#define PFM_REG_W64	(PFM_REG_WC|PFM_REG_NO64|PFM_REG_I)
#define PFM_REG_C	(PFM_REG_C64|PFM_REG_INTR|PFM_REG_I)
#define PFM_REG_I64	(PFM_REG_NO64|PFM_REG_I)
#define PFM_REG_IRO	(PFM_REG_I|PFM_REG_RO)

typedef int (*pfm_pmc_check_t)(struct pfm_context *ctx,
			       struct pfm_event_set *set,
			       struct pfarg_pmc *req);

typedef int (*pfm_pmd_check_t)(struct pfm_context *ctx,
			       struct pfm_event_set *set,
			       struct pfarg_pmd *req);


typedef u64 (*pfm_pmd_sread_t)(struct pfm_context *ctx, unsigned int cnum);
typedef void (*pfm_pmd_swrite_t)(struct pfm_context *ctx, unsigned int cnum, u64 val);

/*
 * registers description
 */
struct pfm_regdesc {
	u64 pmcs[PFM_PMC_BV];		/* available PMC */
	u64 pmds[PFM_PMD_BV];		/* available PMD */
	u64 rw_pmds[PFM_PMD_BV];	/* available RW PMD */
	u64 intr_pmds[PFM_PMD_BV];	/* PMD generating intr */
	u64 cnt_pmds[PFM_PMD_BV];	/* PMD counters */
	u16 max_pmc;			/* highest+1 avail PMC */
	u16 max_pmd;			/* highest+1 avail PMD */
	u16 max_rw_pmd;			/* highest+1 avail RW PMD */
	u16 first_intr_pmd;		/* first intr PMD */
	u16 max_intr_pmd;		/* highest+1 intr PMD */
	u16 num_rw_pmd;			/* number of avail RW PMD */
	u16 num_pmcs;			/* number of logical PMCS */
	u16 num_pmds;			/* number of logical PMDS */
	u16 num_counters;		/* number of counting PMD */
};

/*
 * structure used by pmu description modules
 *
 * probe_pmu() routine return value:
 * 	- 1 means recognized PMU
 * 	- 0 means not recognized PMU
 */
struct pfm_pmu_config {
	char *pmu_name;				/* PMU family name */
	char *version;				/* config module version number */

	int counter_width;			/* width of hardware counter */

	struct pfm_regmap_desc	*pmc_desc;	/* PMC register descriptions */
	struct pfm_regmap_desc	*pmd_desc;	/* PMD register descriptions */

	pfm_pmc_check_t		pmc_write_check;/* PMC write checker callback (optional) */
	pfm_pmd_check_t		pmd_write_check;/* PMD write checker callback (optional) */
	pfm_pmd_check_t		pmd_read_check;	/* PMD read checker callback  (optional) */

	pfm_pmd_sread_t		pmd_sread;	/* PMD model specific read (optional) */
	pfm_pmd_swrite_t	pmd_swrite;	/* PMD model specific write (optional) */

	int             	(*probe_pmu)(void);/* probe PMU routine */

	u16			num_pmc_entries;/* number of entries in pmc_desc */
	u16			num_pmd_entries;/* number of entries in pmd_desc */

	void			*arch_info;	/* arch-specific information */
	u32			flags;		/* set of flags */

	struct module		*owner;		/* pointer to module struct */

	/*
	 * fields computed internally, do not set in module
	 */
	struct pfm_regdesc	regs;		/* registers currently available */
	struct pfm_regdesc	full_regs;	/* registers presented by module */

	u64			ovfl_mask;	/* overflow mask */
	struct kobject		kobj;		/* for internal use only */
};
#define to_pmu(n) container_of(n, struct pfm_pmu_config, kobj)

/*
 * pfm_pmu_config flags
 */
#define PFM_PMUFL_IS_BUILTIN	0x1	/* pmu config is compiled in */

/*
 * we need to know whether the PMU description is builtin or compiled
 * as a module
 */
#ifdef MODULE
#define PFM_PMU_BUILTIN_FLAG	0	/* not built as a module */
#else
#define PFM_PMU_BUILTIN_FLAG	PFM_PMUFL_IS_BUILTIN /* built as a module */
#endif

int pfm_pmu_register(struct pfm_pmu_config *cfg);
void pfm_pmu_unregister(struct pfm_pmu_config *cfg);

#endif /* __PERFMON_PMU_H__ */
