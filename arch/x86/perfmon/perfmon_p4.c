/*
 * This file contains the P4/Xeon PMU register description tables
 * for both 32 and 64 bit modes.
 *
 * Copyright (c) 2005 Intel Corporation
 * Contributed by Bryan Wilkerson <bryan.p.wilkerson@intel.com>
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
#include <asm/msr.h>
#include <asm/apic.h>
#include <asm/nmi.h>

MODULE_AUTHOR("Bryan Wilkerson <bryan.p.wilkerson@intel.com>");
MODULE_DESCRIPTION("P4/Xeon/EM64T PMU description table");
MODULE_LICENSE("GPL");

static int force;
MODULE_PARM_DESC(force, "bool: force module to load succesfully");
module_param(force, bool, 0600);

static int force_nmi;
MODULE_PARM_DESC(force_nmi, "bool: force use of NMI for PMU interrupt");
module_param(force_nmi, bool, 0600);

/*
 * CCCR default value:
 * 	- OVF_PMI_T0=1 (bit 26)
 * 	- OVF_PMI_T1=0 (bit 27) (set if necessary in pfm_write_reg())
 * 	- all other bits are zero
 *
 * OVF_PMI is forced to zero if PFM_REGFL_NO_EMUL64 is set on CCCR
 */
#define PFM_CCCR_DFL	(1ULL<<26) | (3ULL<<16)

/*
 * CCCR reserved fields:
 * 	- bits 0-11, 25-29, 31-63
 * 	- OVF_PMI (26-27), override with REGFL_NO_EMUL64
 *
 * RSVD: reserved bits must be 1
 */
#define PFM_CCCR_RSVD     ~((0xfull<<12)  \
			| (0x7full<<18) \
			| (0x1ull<<30))

#define PFM_P4_NO64	(3ULL<<26) /* use 3 even in non HT mode */

/*
 * With HyperThreading enabled:
 *
 *  The ESCRs and CCCRs are divided in half with the top half
 *  belonging to logical processor 0 and the bottom half going to
 *  logical processor 1. Thus only half of the PMU resources are
 *  accessible to applications.
 *
 *  PEBS is not available due to the fact that:
 *  	- MSR_PEBS_MATRIX_VERT is shared between the threads
 *      - IA32_PEBS_ENABLE is shared between the threads
 *
 * With HyperThreading disabled:
 *
 * The full set of PMU resources is exposed to applications.
 *
 * The mapping is chosen such that PMCxx -> MSR is the same
 * in HT and non HT mode, if register is present in HT mode.
 *
 */
#define PFM_REGT_NHTESCR (PFM_REGT_ESCR|PFM_REGT_NOHT)
#define PFM_REGT_NHTCCCR (PFM_REGT_CCCR|PFM_REGT_NOHT|PFM_REGT_EN)
#define PFM_REGT_NHTPEBS (PFM_REGT_PEBS|PFM_REGT_NOHT|PFM_REGT_EN)
#define PFM_REGT_NHTCTR  (PFM_REGT_CTR|PFM_REGT_NOHT)
#define PFM_REGT_ENAC    (PFM_REGT_CCCR|PFM_REGT_EN)

static struct pfm_arch_pmu_info pfm_p4_pmu_info={
 .pmc_addrs = {
	/*pmc 0 */    {{MSR_P4_BPU_ESCR0, MSR_P4_BPU_ESCR1}, 0, PFM_REGT_ESCR}, /*   BPU_ESCR0,1 */
	/*pmc 1 */    {{MSR_P4_IS_ESCR0, MSR_P4_IS_ESCR1}, 0, PFM_REGT_ESCR}, /*    IS_ESCR0,1 */
	/*pmc 2 */    {{MSR_P4_MOB_ESCR0, MSR_P4_MOB_ESCR1}, 0, PFM_REGT_ESCR}, /*   MOB_ESCR0,1 */
	/*pmc 3 */    {{MSR_P4_ITLB_ESCR0, MSR_P4_ITLB_ESCR1}, 0, PFM_REGT_ESCR}, /*  ITLB_ESCR0,1 */
	/*pmc 4 */    {{MSR_P4_PMH_ESCR0, MSR_P4_PMH_ESCR1}, 0, PFM_REGT_ESCR}, /*   PMH_ESCR0,1 */
	/*pmc 5 */    {{MSR_P4_IX_ESCR0, MSR_P4_IX_ESCR1}, 0, PFM_REGT_ESCR}, /*    IX_ESCR0,1 */
	/*pmc 6 */    {{MSR_P4_FSB_ESCR0, MSR_P4_FSB_ESCR1}, 0, PFM_REGT_ESCR}, /*   FSB_ESCR0,1 */
	/*pmc 7 */    {{MSR_P4_BSU_ESCR0, MSR_P4_BSU_ESCR1}, 0, PFM_REGT_ESCR}, /*   BSU_ESCR0,1 */
	/*pmc 8 */    {{MSR_P4_MS_ESCR0, MSR_P4_MS_ESCR1}, 0, PFM_REGT_ESCR}, /*    MS_ESCR0,1 */
	/*pmc 9 */    {{MSR_P4_TC_ESCR0, MSR_P4_TC_ESCR1}, 0, PFM_REGT_ESCR}, /*    TC_ESCR0,1 */
	/*pmc 10*/    {{MSR_P4_TBPU_ESCR0, MSR_P4_TBPU_ESCR1}, 0, PFM_REGT_ESCR}, /*  TBPU_ESCR0,1 */
	/*pmc 11*/    {{MSR_P4_FLAME_ESCR0, MSR_P4_FLAME_ESCR1}, 0, PFM_REGT_ESCR}, /* FLAME_ESCR0,1 */
	/*pmc 12*/    {{MSR_P4_FIRM_ESCR0, MSR_P4_FIRM_ESCR1}, 0, PFM_REGT_ESCR}, /*  FIRM_ESCR0,1 */
	/*pmc 13*/    {{MSR_P4_SAAT_ESCR0, MSR_P4_SAAT_ESCR1}, 0, PFM_REGT_ESCR}, /*  SAAT_ESCR0,1 */
	/*pmc 14*/    {{MSR_P4_U2L_ESCR0, MSR_P4_U2L_ESCR1}, 0, PFM_REGT_ESCR}, /*   U2L_ESCR0,1 */
	/*pmc 15*/    {{MSR_P4_DAC_ESCR0, MSR_P4_DAC_ESCR1}, 0, PFM_REGT_ESCR}, /*   DAC_ESCR0,1 */
	/*pmc 16*/    {{MSR_P4_IQ_ESCR0, MSR_P4_IQ_ESCR1}, 0, PFM_REGT_ESCR}, /*    IQ_ESCR0,1 (only model 1 and 2) */
	/*pmc 17*/    {{MSR_P4_ALF_ESCR0, MSR_P4_ALF_ESCR1}, 0, PFM_REGT_ESCR}, /*   ALF_ESCR0,1 */
	/*pmc 18*/    {{MSR_P4_RAT_ESCR0, MSR_P4_RAT_ESCR1}, 0, PFM_REGT_ESCR}, /*   RAT_ESCR0,1 */
	/*pmc 19*/    {{MSR_P4_SSU_ESCR0, 0}, 0, PFM_REGT_ESCR}, /*   SSU_ESCR0   */
	/*pmc 20*/    {{MSR_P4_CRU_ESCR0, MSR_P4_CRU_ESCR1}, 0, PFM_REGT_ESCR}, /*   CRU_ESCR0,1 */
	/*pmc 21*/    {{MSR_P4_CRU_ESCR2, MSR_P4_CRU_ESCR3}, 0, PFM_REGT_ESCR}, /*   CRU_ESCR2,3 */
	/*pmc 22*/    {{MSR_P4_CRU_ESCR4, MSR_P4_CRU_ESCR5}, 0, PFM_REGT_ESCR}, /*   CRU_ESCR4,5 */

	/*pmc 23*/    {{MSR_P4_BPU_CCCR0, MSR_P4_BPU_CCCR2}, 0, PFM_REGT_ENAC}, /*   BPU_CCCR0,2 */
	/*pmc 24*/    {{MSR_P4_BPU_CCCR1, MSR_P4_BPU_CCCR3}, 1, PFM_REGT_ENAC}, /*   BPU_CCCR1,3 */
	/*pmc 25*/    {{MSR_P4_MS_CCCR0, MSR_P4_MS_CCCR2}, 2, PFM_REGT_ENAC}, /*    MS_CCCR0,2 */
	/*pmc 26*/    {{MSR_P4_MS_CCCR1, MSR_P4_MS_CCCR3}, 3, PFM_REGT_ENAC}, /*    MS_CCCR1,3 */
	/*pmc 27*/    {{MSR_P4_FLAME_CCCR0, MSR_P4_FLAME_CCCR2}, 4, PFM_REGT_ENAC}, /* FLAME_CCCR0,2 */
	/*pmc 28*/    {{MSR_P4_FLAME_CCCR1, MSR_P4_FLAME_CCCR3}, 5, PFM_REGT_ENAC}, /* FLAME_CCCR1,3 */
	/*pmc 29*/    {{MSR_P4_IQ_CCCR0, MSR_P4_IQ_CCCR2}, 6, PFM_REGT_ENAC}, /*    IQ_CCCR0,2 */
	/*pmc 30*/    {{MSR_P4_IQ_CCCR1, MSR_P4_IQ_CCCR3}, 7, PFM_REGT_ENAC}, /*    IQ_CCCR1,3 */
	/*pmc 31*/    {{MSR_P4_IQ_CCCR4, MSR_P4_IQ_CCCR5}, 8, PFM_REGT_ENAC}, /*    IQ_CCCR4,5 */
	/* non HT extensions */
	/*pmc 32*/    {{MSR_P4_BPU_ESCR1,     0}, 0, PFM_REGT_NHTESCR}, /*   BPU_ESCR1   */
	/*pmc 33*/    {{MSR_P4_IS_ESCR1,     0}, 0, PFM_REGT_NHTESCR}, /*    IS_ESCR1   */
	/*pmc 34*/    {{MSR_P4_MOB_ESCR1,     0}, 0, PFM_REGT_NHTESCR}, /*   MOB_ESCR1   */
	/*pmc 35*/    {{MSR_P4_ITLB_ESCR1,     0}, 0, PFM_REGT_NHTESCR}, /*  ITLB_ESCR1   */
	/*pmc 36*/    {{MSR_P4_PMH_ESCR1,     0}, 0, PFM_REGT_NHTESCR}, /*   PMH_ESCR1   */
	/*pmc 37*/    {{MSR_P4_IX_ESCR1,     0}, 0, PFM_REGT_NHTESCR}, /*    IX_ESCR1   */
	/*pmc 38*/    {{MSR_P4_FSB_ESCR1,     0}, 0, PFM_REGT_NHTESCR}, /*   FSB_ESCR1   */
	/*pmc 39*/    {{MSR_P4_BSU_ESCR1,     0}, 0, PFM_REGT_NHTESCR}, /*   BSU_ESCR1   */
	/*pmc 40*/    {{MSR_P4_MS_ESCR1,     0}, 0, PFM_REGT_NHTESCR}, /*    MS_ESCR1   */
	/*pmc 41*/    {{MSR_P4_TC_ESCR1,     0}, 0, PFM_REGT_NHTESCR}, /*    TC_ESCR1   */
	/*pmc 42*/    {{MSR_P4_TBPU_ESCR1,     0}, 0, PFM_REGT_NHTESCR}, /*  TBPU_ESCR1   */
	/*pmc 43*/    {{MSR_P4_FLAME_ESCR1,     0}, 0, PFM_REGT_NHTESCR}, /* FLAME_ESCR1   */
	/*pmc 44*/    {{MSR_P4_FIRM_ESCR1,     0}, 0, PFM_REGT_NHTESCR}, /*  FIRM_ESCR1   */
	/*pmc 45*/    {{MSR_P4_SAAT_ESCR1,     0}, 0, PFM_REGT_NHTESCR}, /*  SAAT_ESCR1   */
	/*pmc 46*/    {{MSR_P4_U2L_ESCR1,     0}, 0, PFM_REGT_NHTESCR}, /*   U2L_ESCR1   */
	/*pmc 47*/    {{MSR_P4_DAC_ESCR1,     0}, 0, PFM_REGT_NHTESCR}, /*   DAC_ESCR1   */
	/*pmc 48*/    {{MSR_P4_IQ_ESCR1,     0}, 0, PFM_REGT_NHTESCR}, /*    IQ_ESCR1   (only model 1 and 2) */
	/*pmc 49*/    {{MSR_P4_ALF_ESCR1,     0}, 0, PFM_REGT_NHTESCR}, /*   ALF_ESCR1   */
	/*pmc 50*/    {{MSR_P4_RAT_ESCR1,     0}, 0, PFM_REGT_NHTESCR}, /*   RAT_ESCR1   */
	/*pmc 51*/    {{MSR_P4_CRU_ESCR1,     0}, 0, PFM_REGT_NHTESCR}, /*   CRU_ESCR1   */
	/*pmc 52*/    {{MSR_P4_CRU_ESCR3,     0}, 0, PFM_REGT_NHTESCR}, /*   CRU_ESCR3   */
	/*pmc 53*/    {{MSR_P4_CRU_ESCR5,     0}, 0, PFM_REGT_NHTESCR}, /*   CRU_ESCR5   */
	/*pmc 54*/    {{MSR_P4_BPU_CCCR1,     0}, 9, PFM_REGT_NHTCCCR}, /*   BPU_CCCR1   */
	/*pmc 55*/    {{MSR_P4_BPU_CCCR3,     0},10, PFM_REGT_NHTCCCR}, /*   BPU_CCCR3   */
	/*pmc 56*/    {{MSR_P4_MS_CCCR1,     0},11, PFM_REGT_NHTCCCR}, /*    MS_CCCR1   */
	/*pmc 57*/    {{MSR_P4_MS_CCCR3,     0},12, PFM_REGT_NHTCCCR}, /*    MS_CCCR3   */
	/*pmc 58*/    {{MSR_P4_FLAME_CCCR1,     0},13, PFM_REGT_NHTCCCR}, /* FLAME_CCCR1   */
	/*pmc 59*/    {{MSR_P4_FLAME_CCCR3,     0},14, PFM_REGT_NHTCCCR}, /* FLAME_CCCR3   */
	/*pmc 60*/    {{MSR_P4_IQ_CCCR2,     0},15, PFM_REGT_NHTCCCR}, /*    IQ_CCCR2   */
	/*pmc 61*/    {{MSR_P4_IQ_CCCR3,     0},16, PFM_REGT_NHTCCCR}, /*    IQ_CCCR3   */
	/*pmc 62*/    {{MSR_P4_IQ_CCCR5,     0},17, PFM_REGT_NHTCCCR}, /*    IQ_CCCR5   */
	/*pmc 63*/    {{0x3f2,     0}, 0, PFM_REGT_NHTPEBS},/* PEBS_MATRIX_VERT */
	/*pmc 64*/    {{0x3f1,     0}, 0, PFM_REGT_NHTPEBS} /* PEBS_ENABLE   */
},

.pmd_addrs = {
	/*pmd 0 */    {{MSR_P4_BPU_PERFCTR0, MSR_P4_BPU_PERFCTR2}, 0, PFM_REGT_CTR},  /*   BPU_CTR0,2  */
	/*pmd 1 */    {{MSR_P4_BPU_PERFCTR1, MSR_P4_BPU_PERFCTR3}, 0, PFM_REGT_CTR},  /*   BPU_CTR1,3  */
	/*pmd 2 */    {{MSR_P4_MS_PERFCTR0, MSR_P4_MS_PERFCTR2}, 0, PFM_REGT_CTR},  /*    MS_CTR0,2  */
	/*pmd 3 */    {{MSR_P4_MS_PERFCTR1, MSR_P4_MS_PERFCTR3}, 0, PFM_REGT_CTR},  /*    MS_CTR1,3  */
	/*pmd 4 */    {{MSR_P4_FLAME_PERFCTR0, MSR_P4_FLAME_PERFCTR2}, 0, PFM_REGT_CTR},  /* FLAME_CTR0,2  */
	/*pmd 5 */    {{MSR_P4_FLAME_PERFCTR1, MSR_P4_FLAME_PERFCTR3}, 0, PFM_REGT_CTR},  /* FLAME_CTR1,3  */
	/*pmd 6 */    {{MSR_P4_IQ_PERFCTR0, MSR_P4_IQ_PERFCTR2}, 0, PFM_REGT_CTR},  /*    IQ_CTR0,2  */
	/*pmd 7 */    {{MSR_P4_IQ_PERFCTR1, MSR_P4_IQ_PERFCTR3}, 0, PFM_REGT_CTR},  /*    IQ_CTR1,3  */
	/*pmd 8 */    {{MSR_P4_IQ_PERFCTR4, MSR_P4_IQ_PERFCTR5}, 0, PFM_REGT_CTR},  /*    IQ_CTR4,5  */
	/*
	 * non HT extensions
	 */
	/*pmd 9 */    {{MSR_P4_BPU_PERFCTR2,     0}, 0, PFM_REGT_NHTCTR},  /*   BPU_CTR2    */
	/*pmd 10*/    {{MSR_P4_BPU_PERFCTR3,     0}, 0, PFM_REGT_NHTCTR},  /*   BPU_CTR3    */
	/*pmd 11*/    {{MSR_P4_MS_PERFCTR2,     0}, 0, PFM_REGT_NHTCTR},  /*    MS_CTR2    */
	/*pmd 12*/    {{MSR_P4_MS_PERFCTR3,     0}, 0, PFM_REGT_NHTCTR},  /*    MS_CTR3    */
	/*pmd 13*/    {{MSR_P4_FLAME_PERFCTR2,     0}, 0, PFM_REGT_NHTCTR},  /* FLAME_CTR2    */
	/*pmd 14*/    {{MSR_P4_FLAME_PERFCTR3,     0}, 0, PFM_REGT_NHTCTR},  /* FLAME_CTR3    */
	/*pmd 15*/    {{MSR_P4_IQ_PERFCTR2,     0}, 0, PFM_REGT_NHTCTR},  /*    IQ_CTR2    */
	/*pmd 16*/    {{MSR_P4_IQ_PERFCTR3,     0}, 0, PFM_REGT_NHTCTR},  /*    IQ_CTR3    */
	/*pmd 17*/    {{MSR_P4_IQ_PERFCTR5,     0}, 0, PFM_REGT_NHTCTR},  /*    IQ_CTR5    */
},
.pebs_ctr_idx = 8, /* thread0: IQ_CTR4, thread1: IQ_CTR5 */
.pmu_style = PFM_X86_PMU_P4
};

static struct pfm_regmap_desc pfm_p4_pmc_desc[]={
/* pmc0  */ PMC_D(PFM_REG_I, "BPU_ESCR0"  , 0x0, PFM_ESCR_RSVD, 0, MSR_P4_BPU_ESCR0),
/* pmc1  */ PMC_D(PFM_REG_I, "IS_ESCR0"   , 0x0, PFM_ESCR_RSVD, 0, MSR_P4_IQ_ESCR0),
/* pmc2  */ PMC_D(PFM_REG_I, "MOB_ESCR0"  , 0x0, PFM_ESCR_RSVD, 0, MSR_P4_MOB_ESCR0),
/* pmc3  */ PMC_D(PFM_REG_I, "ITLB_ESCR0" , 0x0, PFM_ESCR_RSVD, 0, MSR_P4_ITLB_ESCR0),
/* pmc4  */ PMC_D(PFM_REG_I, "PMH_ESCR0"  , 0x0, PFM_ESCR_RSVD, 0, MSR_P4_PMH_ESCR0),
/* pmc5  */ PMC_D(PFM_REG_I, "IX_ESCR0"   , 0x0, PFM_ESCR_RSVD, 0, MSR_P4_IX_ESCR0),
/* pmc6  */ PMC_D(PFM_REG_I, "FSB_ESCR0"  , 0x0, PFM_ESCR_RSVD, 0, MSR_P4_FSB_ESCR0),
/* pmc7  */ PMC_D(PFM_REG_I, "BSU_ESCR0"  , 0x0, PFM_ESCR_RSVD, 0, MSR_P4_BSU_ESCR0),
/* pmc8  */ PMC_D(PFM_REG_I, "MS_ESCR0"   , 0x0, PFM_ESCR_RSVD, 0, MSR_P4_MS_ESCR0),
/* pmc9  */ PMC_D(PFM_REG_I, "TC_ESCR0"   , 0x0, PFM_ESCR_RSVD, 0, MSR_P4_TC_ESCR0),
/* pmc10 */ PMC_D(PFM_REG_I, "TBPU_ESCR0" , 0x0, PFM_ESCR_RSVD, 0, MSR_P4_TBPU_ESCR0),
/* pmc11 */ PMC_D(PFM_REG_I, "FLAME_ESCR0", 0x0, PFM_ESCR_RSVD, 0, MSR_P4_FLAME_ESCR0),
/* pmc12 */ PMC_D(PFM_REG_I, "FIRM_ESCR0" , 0x0, PFM_ESCR_RSVD, 0, MSR_P4_FIRM_ESCR0),
/* pmc13 */ PMC_D(PFM_REG_I, "SAAT_ESCR0" , 0x0, PFM_ESCR_RSVD, 0, MSR_P4_SAAT_ESCR0),
/* pmc14 */ PMC_D(PFM_REG_I, "U2L_ESCR0"  , 0x0, PFM_ESCR_RSVD, 0, MSR_P4_U2L_ESCR0),
/* pmc15 */ PMC_D(PFM_REG_I, "DAC_ESCR0"  , 0x0, PFM_ESCR_RSVD, 0, MSR_P4_DAC_ESCR0),
/* pmc16 */ PMC_D(PFM_REG_I, "IQ_ESCR0"   , 0x0, PFM_ESCR_RSVD, 0, MSR_P4_IQ_ESCR0), /* only model 1 and 2*/
/* pmc17 */ PMC_D(PFM_REG_I, "ALF_ESCR0"  , 0x0, PFM_ESCR_RSVD, 0, MSR_P4_ALF_ESCR0),
/* pmc18 */ PMC_D(PFM_REG_I, "RAT_ESCR0"  , 0x0, PFM_ESCR_RSVD, 0, MSR_P4_RAT_ESCR0),
/* pmc19 */ PMC_D(PFM_REG_I, "SSU_ESCR0"  , 0x0, PFM_ESCR_RSVD, 0, MSR_P4_SSU_ESCR0),
/* pmc20 */ PMC_D(PFM_REG_I, "CRU_ESCR0"  , 0x0, PFM_ESCR_RSVD, 0, MSR_P4_CRU_ESCR0),
/* pmc21 */ PMC_D(PFM_REG_I, "CRU_ESCR2"  , 0x0, PFM_ESCR_RSVD, 0, MSR_P4_CRU_ESCR2),
/* pmc22 */ PMC_D(PFM_REG_I, "CRU_ESCR4"  , 0x0, PFM_ESCR_RSVD, 0, MSR_P4_CRU_ESCR4),
/* pmc23 */ PMC_D(PFM_REG_I64, "BPU_CCCR0"  , PFM_CCCR_DFL, PFM_CCCR_RSVD, PFM_P4_NO64, MSR_P4_BPU_CCCR0),
/* pmc24 */ PMC_D(PFM_REG_I64, "BPU_CCCR1"  , PFM_CCCR_DFL, PFM_CCCR_RSVD, PFM_P4_NO64, MSR_P4_BPU_CCCR1),
/* pmc25 */ PMC_D(PFM_REG_I64, "MS_CCCR0"   , PFM_CCCR_DFL, PFM_CCCR_RSVD, PFM_P4_NO64, MSR_P4_MS_CCCR0),
/* pmc26 */ PMC_D(PFM_REG_I64, "MS_CCCR1"   , PFM_CCCR_DFL, PFM_CCCR_RSVD, PFM_P4_NO64, MSR_P4_MS_CCCR1),
/* pmc27 */ PMC_D(PFM_REG_I64, "FLAME_CCCR0", PFM_CCCR_DFL, PFM_CCCR_RSVD, PFM_P4_NO64, MSR_P4_FLAME_CCCR0),
/* pmc28 */ PMC_D(PFM_REG_I64, "FLAME_CCCR1", PFM_CCCR_DFL, PFM_CCCR_RSVD, PFM_P4_NO64, MSR_P4_FLAME_CCCR1),
/* pmc29 */ PMC_D(PFM_REG_I64, "IQ_CCCR0"   , PFM_CCCR_DFL, PFM_CCCR_RSVD, PFM_P4_NO64, MSR_P4_IQ_CCCR0),
/* pmc30 */ PMC_D(PFM_REG_I64, "IQ_CCCR1"   , PFM_CCCR_DFL, PFM_CCCR_RSVD, PFM_P4_NO64, MSR_P4_IQ_CCCR1),
/* pmc31 */ PMC_D(PFM_REG_I64, "IQ_CCCR4"   , PFM_CCCR_DFL, PFM_CCCR_RSVD, PFM_P4_NO64, MSR_P4_IQ_CCCR4),
		/* No HT extension */
/* pmc32 */ PMC_D(PFM_REG_I, "BPU_ESCR1"  , 0x0, PFM_ESCR_RSVD, 0, MSR_P4_BPU_ESCR1),
/* pmc33 */ PMC_D(PFM_REG_I, "IS_ESCR1"   , 0x0, PFM_ESCR_RSVD, 0, MSR_P4_IS_ESCR1),
/* pmc34 */ PMC_D(PFM_REG_I, "MOB_ESCR1"  , 0x0, PFM_ESCR_RSVD, 0, MSR_P4_MOB_ESCR1),
/* pmc35 */ PMC_D(PFM_REG_I, "ITLB_ESCR1" , 0x0, PFM_ESCR_RSVD, 0, MSR_P4_ITLB_ESCR1),
/* pmc36 */ PMC_D(PFM_REG_I, "PMH_ESCR1"  , 0x0, PFM_ESCR_RSVD, 0, MSR_P4_PMH_ESCR1),
/* pmc37 */ PMC_D(PFM_REG_I, "IX_ESCR1"   , 0x0, PFM_ESCR_RSVD, 0, MSR_P4_IX_ESCR1),
/* pmc38 */ PMC_D(PFM_REG_I, "FSB_ESCR1"  , 0x0, PFM_ESCR_RSVD, 0, MSR_P4_FSB_ESCR1),
/* pmc39 */ PMC_D(PFM_REG_I, "BSU_ESCR1"  , 0x0, PFM_ESCR_RSVD, 0, MSR_P4_BSU_ESCR1),
/* pmc40 */ PMC_D(PFM_REG_I, "MS_ESCR1"   , 0x0, PFM_ESCR_RSVD, 0, MSR_P4_MS_ESCR1),
/* pmc41 */ PMC_D(PFM_REG_I, "TC_ESCR1"   , 0x0, PFM_ESCR_RSVD, 0, MSR_P4_TC_ESCR1),
/* pmc42 */ PMC_D(PFM_REG_I, "TBPU_ESCR1" , 0x0, PFM_ESCR_RSVD, 0, MSR_P4_TBPU_ESCR1),
/* pmc43 */ PMC_D(PFM_REG_I, "FLAME_ESCR1", 0x0, PFM_ESCR_RSVD, 0, MSR_P4_FLAME_ESCR1),
/* pmc44 */ PMC_D(PFM_REG_I, "FIRM_ESCR1" , 0x0, PFM_ESCR_RSVD, 0, MSR_P4_FIRM_ESCR1),
/* pmc45 */ PMC_D(PFM_REG_I, "SAAT_ESCR1" , 0x0, PFM_ESCR_RSVD, 0, MSR_P4_SAAT_ESCR1),
/* pmc46 */ PMC_D(PFM_REG_I, "U2L_ESCR1"  , 0x0, PFM_ESCR_RSVD, 0, MSR_P4_U2L_ESCR1),
/* pmc47 */ PMC_D(PFM_REG_I, "DAC_ESCR1"  , 0x0, PFM_ESCR_RSVD, 0, MSR_P4_DAC_ESCR1),
/* pmc48 */ PMC_D(PFM_REG_I, "IQ_ESCR1"   , 0x0, PFM_ESCR_RSVD, 0, MSR_P4_IQ_ESCR1), /* only model 1 and 2 */
/* pmc49 */ PMC_D(PFM_REG_I, "ALF_ESCR1"  , 0x0, PFM_ESCR_RSVD, 0, MSR_P4_ALF_ESCR1),
/* pmc50 */ PMC_D(PFM_REG_I, "RAT_ESCR1"  , 0x0, PFM_ESCR_RSVD, 0, MSR_P4_RAT_ESCR1),
/* pmc51 */ PMC_D(PFM_REG_I, "CRU_ESCR1"  , 0x0, PFM_ESCR_RSVD, 0, MSR_P4_CRU_ESCR1),
/* pmc52 */ PMC_D(PFM_REG_I, "CRU_ESCR3"  , 0x0, PFM_ESCR_RSVD, 0, MSR_P4_CRU_ESCR3),
/* pmc53 */ PMC_D(PFM_REG_I, "CRU_ESCR5"  , 0x0, PFM_ESCR_RSVD, 0, MSR_P4_CRU_ESCR5),
/* pmc54 */ PMC_D(PFM_REG_I64, "BPU_CCCR2"  , PFM_CCCR_DFL, PFM_CCCR_RSVD, PFM_P4_NO64, MSR_P4_BPU_CCCR2),
/* pmc55 */ PMC_D(PFM_REG_I64, "BPU_CCCR3"  , PFM_CCCR_DFL, PFM_CCCR_RSVD, PFM_P4_NO64, MSR_P4_BPU_CCCR3),
/* pmc56 */ PMC_D(PFM_REG_I64, "MS_CCCR2"   , PFM_CCCR_DFL, PFM_CCCR_RSVD, PFM_P4_NO64, MSR_P4_MS_CCCR2),
/* pmc57 */ PMC_D(PFM_REG_I64, "MS_CCCR3"   , PFM_CCCR_DFL, PFM_CCCR_RSVD, PFM_P4_NO64, MSR_P4_MS_CCCR3),
/* pmc58 */ PMC_D(PFM_REG_I64, "FLAME_CCCR2", PFM_CCCR_DFL, PFM_CCCR_RSVD, PFM_P4_NO64, MSR_P4_FLAME_CCCR2),
/* pmc59 */ PMC_D(PFM_REG_I64, "FLAME_CCCR3", PFM_CCCR_DFL, PFM_CCCR_RSVD, PFM_P4_NO64, MSR_P4_FLAME_CCCR3),
/* pmc60 */ PMC_D(PFM_REG_I64, "IQ_CCCR2"   , PFM_CCCR_DFL, PFM_CCCR_RSVD, PFM_P4_NO64, MSR_P4_IQ_CCCR2),
/* pmc61 */ PMC_D(PFM_REG_I64, "IQ_CCCR3"   , PFM_CCCR_DFL, PFM_CCCR_RSVD, PFM_P4_NO64, MSR_P4_IQ_CCCR3),
/* pmc62 */ PMC_D(PFM_REG_I64, "IQ_CCCR5"   , PFM_CCCR_DFL, PFM_CCCR_RSVD, PFM_P4_NO64, MSR_P4_IQ_CCCR5),
/* pmc63 */ PMC_D(PFM_REG_I, "PEBS_MATRIX_VERT", 0, 0xffffffffffffffecULL, 0, 0x3f2),
/* pmc64 */ PMC_D(PFM_REG_I, "PEBS_ENABLE", 0, 0xfffffffff8ffe000ULL, 0, 0x3f1)
};
#define PFM_P4_NUM_PMCS ARRAY_SIZE(pfm_p4_pmc_desc)

/*
 * See section 15.10.6.6 for details about the IQ block
 */
static struct pfm_regmap_desc pfm_p4_pmd_desc[]={
/* pmd0  */ PMD_D(PFM_REG_C, "BPU_CTR0", MSR_P4_BPU_PERFCTR0),
/* pmd1  */ PMD_D(PFM_REG_C, "BPU_CTR1", MSR_P4_BPU_PERFCTR1),
/* pmd2  */ PMD_D(PFM_REG_C, "MS_CTR0", MSR_P4_MS_PERFCTR0),
/* pmd3  */ PMD_D(PFM_REG_C, "MS_CTR1", MSR_P4_MS_PERFCTR1),
/* pmd4  */ PMD_D(PFM_REG_C, "FLAME_CTR0", MSR_P4_FLAME_PERFCTR0),
/* pmd5  */ PMD_D(PFM_REG_C, "FLAME_CTR1", MSR_P4_FLAME_PERFCTR1),
/* pmd6  */ PMD_D(PFM_REG_C, "IQ_CTR0", MSR_P4_IQ_PERFCTR0),
/* pmd7  */ PMD_D(PFM_REG_C, "IQ_CTR1", MSR_P4_IQ_PERFCTR1),
/* pmd8  */ PMD_D(PFM_REG_C, "IQ_CTR4", MSR_P4_IQ_PERFCTR4),
		/* no HT extension */
/* pmd9  */ PMD_D(PFM_REG_C, "BPU_CTR2", MSR_P4_BPU_PERFCTR2),
/* pmd10 */ PMD_D(PFM_REG_C, "BPU_CTR3", MSR_P4_BPU_PERFCTR3),
/* pmd11 */ PMD_D(PFM_REG_C, "MS_CTR2", MSR_P4_MS_PERFCTR2),
/* pmd12 */ PMD_D(PFM_REG_C, "MS_CTR3", MSR_P4_MS_PERFCTR3),
/* pmd13 */ PMD_D(PFM_REG_C, "FLAME_CTR2", MSR_P4_FLAME_PERFCTR2),
/* pmd14 */ PMD_D(PFM_REG_C, "FLAME_CTR3", MSR_P4_FLAME_PERFCTR3),
/* pmd15 */ PMD_D(PFM_REG_C, "IQ_CTR2", MSR_P4_IQ_PERFCTR2),
/* pmd16 */ PMD_D(PFM_REG_C, "IQ_CTR3", MSR_P4_IQ_PERFCTR3),
/* pmd17 */ PMD_D(PFM_REG_C, "IQ_CTR5", MSR_P4_IQ_PERFCTR5)
};
#define PFM_P4_NUM_PMDS ARRAY_SIZE(pfm_p4_pmd_desc)

/*
 * Due to hotplug CPU support, threads may not necessarily
 * be activated at the time the module is inserted. We need
 * to check whether  they could be activated by looking at
 * the present CPU (present != online).
 */
static int pfm_p4_probe_pmu(void)
{
	unsigned int i;
	int ht_enabled;

	/*
	 * only works on Intel processors
	 */
	if (current_cpu_data.x86_vendor != X86_VENDOR_INTEL) {
		PFM_INFO("not running on Intel processor");
		return -1;
	}

	if (current_cpu_data.x86 != 15) {
		PFM_INFO("unsupported family=%d", current_cpu_data.x86);
		return -1;
	}

	switch(current_cpu_data.x86_model) {
		case 0 ... 2:
			break;
		case 3 ... 6:
			/*
			 * IQ_ESCR0, IQ_ESCR1 only present on model 1, 2
			 */
			pfm_p4_pmc_desc[16].type = PFM_REG_NA;
			pfm_p4_pmc_desc[48].type = PFM_REG_NA;
			break;
		default:
			/*
			 * do not know if they all work the same, so reject
			 * for now
			 */
			if (!force) {
				PFM_INFO("unsupported model %d",
					 current_cpu_data.x86_model);
				return -1;
			}
	}

	/*
	 * check for local APIC (required)
	 */
	if (!cpu_has_apic) {
		PFM_INFO("no local APIC, unsupported");
		return -1;
	}
#ifdef CONFIG_SMP
	ht_enabled = (cpus_weight(__get_cpu_var(cpu_core_map))
		   / current_cpu_data.x86_max_cores) > 1;
#else
	ht_enabled = 0;
#endif
	if (cpu_has_ht) {

		PFM_INFO("HyperThreading supported, status %s",
			 ht_enabled ? "on": "off");
		/*
		 * disable registers not supporting HT
		 */
		if (ht_enabled) {
			PFM_INFO("disabling half the registers for HT");
			for (i = 0; i < PFM_P4_NUM_PMCS; i++) {
				if (pfm_p4_pmu_info.pmc_addrs[(i)].reg_type &
				    PFM_REGT_NOHT)
					pfm_p4_pmc_desc[i].type = PFM_REG_NA;
			}
			for (i = 0; i < PFM_P4_NUM_PMDS; i++) {
				if (pfm_p4_pmu_info.pmd_addrs[(i)].reg_type &
				    PFM_REGT_NOHT)
					pfm_p4_pmd_desc[i].type = PFM_REG_NA;
			}
		}
	}

	if (cpu_has_ds) {
		PFM_INFO("Data Save Area (DS) supported");

		pfm_p4_pmu_info.flags = PFM_X86_FL_PMU_DS;

		if (cpu_has_pebs) {
			/*
			 * PEBS does not work with HyperThreading enabled
			 */
	                if (ht_enabled) {
				PFM_INFO("PEBS supported, status off (because of HT)");
			} else {
				pfm_p4_pmu_info.flags |= PFM_X86_FL_PMU_PEBS;
				PFM_INFO("PEBS supported, status on");
			}
		}
	}
	if (force_nmi)
		pfm_p4_pmu_info.flags |= PFM_X86_FL_USE_NMI;
	return 0;
}

static struct pfm_pmu_config pfm_p4_pmu_conf={
	.pmu_name = "Intel P4",
	.counter_width = 40,
	.pmd_desc = pfm_p4_pmd_desc,
	.pmc_desc = pfm_p4_pmc_desc,
	.num_pmc_entries = PFM_P4_NUM_PMCS,
	.num_pmd_entries = PFM_P4_NUM_PMDS,
	.probe_pmu = pfm_p4_probe_pmu,
	.version = "1.0",
	.flags = PFM_PMU_BUILTIN_FLAG,
	.owner = THIS_MODULE,
	.arch_info = &pfm_p4_pmu_info
};

static int __init pfm_p4_pmu_init_module(void)
{
	return pfm_pmu_register(&pfm_p4_pmu_conf);
}

static void __exit pfm_p4_pmu_cleanup_module(void)
{
	pfm_pmu_unregister(&pfm_p4_pmu_conf);
}

module_init(pfm_p4_pmu_init_module);
module_exit(pfm_p4_pmu_cleanup_module);
