/*
 * This file contains the PMU description for the Athlon64 and Opteron64
 * processors. It supports 32 and 64-bit modes.
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
#include <linux/module.h>
#include <linux/vmalloc.h>
#include <linux/topology.h>
#include <linux/pci.h>
#include <linux/perfmon_kern.h>

#include <asm/apic.h>

MODULE_AUTHOR("Stephane Eranian <eranian@hpl.hp.com>");
MODULE_AUTHOR("Robert Richter <robert.richter@amd.com>");
MODULE_DESCRIPTION("AMD64 PMU description table");
MODULE_LICENSE("GPL");

static int force_nmi;
MODULE_PARM_DESC(force_nmi, "bool: force use of NMI for PMU interrupt");
module_param(force_nmi, bool, 0600);

static struct pfm_arch_pmu_info pfm_amd64_pmu_info = {
	.pmc_addrs = {
/* pmc0  */	{{MSR_K7_EVNTSEL0, 0}, 0, PFM_REGT_EN},
/* pmc1  */	{{MSR_K7_EVNTSEL1, 0}, 1, PFM_REGT_EN},
/* pmc2  */	{{MSR_K7_EVNTSEL2, 0}, 2, PFM_REGT_EN},
/* pmc3  */	{{MSR_K7_EVNTSEL3, 0}, 3, PFM_REGT_EN},
/* pmc4  */	{{MSR_AMD64_IBSFETCHCTL, 0}, 0, PFM_REGT_EN|PFM_REGT_IBS},
/* pmc5  */	{{MSR_AMD64_IBSOPCTL, 0}, 0, PFM_REGT_EN|PFM_REGT_IBS},
	},
	.pmd_addrs = {
/* pmd0  */	{{MSR_K7_PERFCTR0, 0}, 0, PFM_REGT_CTR},
/* pmd1  */	{{MSR_K7_PERFCTR1, 0}, 0, PFM_REGT_CTR},
/* pmd2  */	{{MSR_K7_PERFCTR2, 0}, 0, PFM_REGT_CTR},
/* pmd3  */	{{MSR_K7_PERFCTR3, 0}, 0, PFM_REGT_CTR},
/* pmd4  */	{{MSR_AMD64_IBSFETCHCTL, 0}, 0, PFM_REGT_IBS},
/* pmd5  */	{{MSR_AMD64_IBSFETCHLINAD, 0}, 0, PFM_REGT_IBS},
/* pmd6  */	{{MSR_AMD64_IBSFETCHPHYSAD, 0}, 0, PFM_REGT_IBS},
/* pmd7  */	{{MSR_AMD64_IBSOPCTL, 0}, 0, PFM_REGT_IBS},
/* pmd8  */	{{MSR_AMD64_IBSOPRIP, 0}, 0, PFM_REGT_IBS},
/* pmd9  */	{{MSR_AMD64_IBSOPDATA, 0}, 0, PFM_REGT_IBS},
/* pmd10 */	{{MSR_AMD64_IBSOPDATA2, 0}, 0, PFM_REGT_IBS},
/* pmd11 */	{{MSR_AMD64_IBSOPDATA3, 0}, 0, PFM_REGT_IBS_EXT},
/* pmd12 */	{{MSR_AMD64_IBSDCLINAD, 0}, 0, PFM_REGT_IBS_EXT},
/* pmd13 */	{{MSR_AMD64_IBSDCPHYSAD, 0}, 0, PFM_REGT_IBS_EXT},
	},
	.ibsfetchctl_pmc = 4,
	.ibsfetchctl_pmd = 4,
	.ibsopctl_pmc = 5,
	.ibsopctl_pmd = 7,
	.pmu_style = PFM_X86_PMU_AMD64,
};

/*
 * force Local APIC interrupt on overflow
 */
#define PFM_K8_VAL	(1ULL<<20)
#define PFM_K8_NO64	(1ULL<<20)

/*
 * for performance counter control registers:
 *
 * reserved bits must be zero
 *
 * for family 15:
 * - upper 32 bits are reserved
 *
 * for family 16:
 * - bits 36-39 are reserved
 * - bits 42-63 are reserved
 */
#define PFM_K8_RSVD ((~((1ULL<<32)-1)) | (1ULL<<20) | (1ULL<<21))
#define PFM_16_RSVD ((0x3fffffULL<<42) | (0xfULL<<36) | (1ULL<<20) | (1ULL<<21))

/*
 * for IBS registers:
 * 	IBSFETCHCTL: all bits are reserved except bits 57, 48, 15:0
 * 	IBSOPSCTL  : all bits are reserved except bits 17, 15:0
 */
#define PFM_AMD64_IBSFETCHCTL_RSVD	(~((1ULL<<48)|(1ULL<<57)|0xffffULL))
#define PFM_AMD64_IBSOPCTL_RSVD		(~((1ULL<<17)|0xffffULL))

#define IBSCTL				0x1cc
#define IBSCTL_LVTOFFSETVAL		(1 << 8)

#define ENABLE_CF8_EXT_CFG		(1ULL << 46)

static struct pfm_regmap_desc pfm_amd64_pmc_desc[] = {
/* pmc0  */ PMC_D(PFM_REG_I64, "PERFSEL0", PFM_K8_VAL, PFM_K8_RSVD, PFM_K8_NO64, MSR_K7_EVNTSEL0),
/* pmc1  */ PMC_D(PFM_REG_I64, "PERFSEL1", PFM_K8_VAL, PFM_K8_RSVD, PFM_K8_NO64, MSR_K7_EVNTSEL1),
/* pmc2  */ PMC_D(PFM_REG_I64, "PERFSEL2", PFM_K8_VAL, PFM_K8_RSVD, PFM_K8_NO64, MSR_K7_EVNTSEL2),
/* pmc3  */ PMC_D(PFM_REG_I64, "PERFSEL3", PFM_K8_VAL, PFM_K8_RSVD, PFM_K8_NO64, MSR_K7_EVNTSEL3),
/* pmc4  */ PMC_D(PFM_REG_I,   "IBSFETCHCTL", 0, PFM_AMD64_IBSFETCHCTL_RSVD, 0, MSR_AMD64_IBSFETCHCTL),
/* pmc5  */ PMC_D(PFM_REG_I,   "IBSOPCTL",    0, PFM_AMD64_IBSOPCTL_RSVD,    0, MSR_AMD64_IBSOPCTL),
};
#define PFM_AMD_NUM_PMCS ARRAY_SIZE(pfm_amd64_pmc_desc)

#define PFM_REG_IBS (PFM_REG_I|PFM_REG_INTR)
static struct pfm_regmap_desc pfm_amd64_pmd_desc[] = {
/* pmd0  */ PMD_D(PFM_REG_C,   "PERFCTR0",	MSR_K7_PERFCTR0),
/* pmd1  */ PMD_D(PFM_REG_C,   "PERFCTR1",	MSR_K7_PERFCTR1),
/* pmd2  */ PMD_D(PFM_REG_C,   "PERFCTR2",	MSR_K7_PERFCTR2),
/* pmd3  */ PMD_D(PFM_REG_C,   "PERFCTR3",	MSR_K7_PERFCTR3),
/* pmd4  */ PMD_D(PFM_REG_IBS, "IBSFETCHCTL",	MSR_AMD64_IBSFETCHCTL),
/* pmd5  */ PMD_D(PFM_REG_IRO, "IBSFETCHLINAD",	MSR_AMD64_IBSFETCHLINAD),
/* pmd6  */ PMD_D(PFM_REG_IRO, "IBSFETCHPHYSAD", MSR_AMD64_IBSFETCHPHYSAD),
/* pmd7  */ PMD_D(PFM_REG_IBS, "IBSOPCTL",	MSR_AMD64_IBSOPCTL),
/* pmd8  */ PMD_D(PFM_REG_IRO, "IBSOPRIP",	MSR_AMD64_IBSOPRIP),
/* pmd9  */ PMD_D(PFM_REG_IRO, "IBSOPDATA",	MSR_AMD64_IBSOPDATA),
/* pmd10 */ PMD_D(PFM_REG_IRO, "IBSOPDATA2",	MSR_AMD64_IBSOPDATA2),
/* pmd11 */ PMD_D(PFM_REG_IRO, "IBSOPDATA3",	MSR_AMD64_IBSOPDATA3),
/* pmd12 */ PMD_D(PFM_REG_IRO, "IBSDCLINAD",	MSR_AMD64_IBSDCLINAD),
/* pmd13 */ PMD_D(PFM_REG_IRO, "IBSDCPHYSAD",	MSR_AMD64_IBSDCPHYSAD),
};
#define PFM_AMD_NUM_PMDS ARRAY_SIZE(pfm_amd64_pmd_desc)

static struct pfm_context **pfm_nb_sys_owners;
static struct pfm_context *pfm_nb_task_owner;

static struct pfm_pmu_config pfm_amd64_pmu_conf;

/* Functions for accessing extended PCI config space. Can be removed
   when Kernel API exists. */
extern spinlock_t pci_config_lock;

#define PCI_CONF1_ADDRESS(bus, devfn, reg) \
	(0x80000000 | ((reg & 0xF00) << 16) | ((bus & 0xFF) << 16) \
	| (devfn << 8) | (reg & 0xFC))

#define is_ibs(x) (pfm_amd64_pmu_info.pmc_addrs[x].reg_type & PFM_REGT_IBS)

static int pci_read(unsigned int seg, unsigned int bus,
		    unsigned int devfn, int reg, int len, u32 *value)
{
	unsigned long flags;

	if ((bus > 255) || (devfn > 255) || (reg > 4095)) {
		*value = -1;
		return -EINVAL;
	}

	spin_lock_irqsave(&pci_config_lock, flags);

	outl(PCI_CONF1_ADDRESS(bus, devfn, reg), 0xCF8);

	switch (len) {
	case 1:
		*value = inb(0xCFC + (reg & 3));
		break;
	case 2:
		*value = inw(0xCFC + (reg & 2));
		break;
	case 4:
		*value = inl(0xCFC);
		break;
	}

	spin_unlock_irqrestore(&pci_config_lock, flags);

	return 0;
}

static int pci_write(unsigned int seg, unsigned int bus,
		     unsigned int devfn, int reg, int len, u32 value)
{
	unsigned long flags;

	if ((bus > 255) || (devfn > 255) || (reg > 4095))
		return -EINVAL;

	spin_lock_irqsave(&pci_config_lock, flags);

	outl(PCI_CONF1_ADDRESS(bus, devfn, reg), 0xCF8);

	switch (len) {
	case 1:
		outb((u8)value, 0xCFC + (reg & 3));
		break;
	case 2:
		outw((u16)value, 0xCFC + (reg & 2));
		break;
	case 4:
		outl((u32)value, 0xCFC);
		break;
	}

	spin_unlock_irqrestore(&pci_config_lock, flags);

	return 0;
}

static inline int
pci_read_ext_config_dword(struct pci_dev *dev, int where, u32 *val)
{
	return pci_read(0, dev->bus->number, dev->devfn, where, 4, val);
}

static inline int
pci_write_ext_config_dword(struct pci_dev *dev, int where, u32 val)
{
	return pci_write(0, dev->bus->number, dev->devfn, where, 4, val);
}

static void pfm_amd64_enable_pci_ecs_per_cpu(void)
{
	u64 reg;
	/* enable PCI extended config space */
	rdmsrl(MSR_AMD64_NB_CFG, reg);
	if (reg & ENABLE_CF8_EXT_CFG)
		return;
	reg |= ENABLE_CF8_EXT_CFG;
	wrmsrl(MSR_AMD64_NB_CFG, reg);
}

static void pfm_amd64_setup_eilvt_per_cpu(void *info)
{
	u8 lvt_off;

	pfm_amd64_enable_pci_ecs_per_cpu();

	/* program the IBS vector to the perfmon vector */
	lvt_off =  setup_APIC_eilvt_ibs(LOCAL_PERFMON_VECTOR,
					APIC_EILVT_MSG_FIX, 0);
	PFM_DBG("APIC_EILVT%d set to 0x%x", lvt_off, LOCAL_PERFMON_VECTOR);
	pfm_amd64_pmu_info.ibs_eilvt_off = lvt_off;
}

static int pfm_amd64_setup_eilvt(void)
{
	struct pci_dev *cpu_cfg;
	int nodes;
	u32 value = 0;

	/* per CPU setup */
	on_each_cpu(pfm_amd64_setup_eilvt_per_cpu, NULL, 0, 1);

	nodes = 0;
	cpu_cfg = NULL;
	do {
		cpu_cfg = pci_get_device(PCI_VENDOR_ID_AMD,
					 PCI_DEVICE_ID_AMD_10H_NB_MISC,
					 cpu_cfg);
		if (!cpu_cfg)
			break;
		++nodes;
		pci_write_ext_config_dword(cpu_cfg, IBSCTL,
					   pfm_amd64_pmu_info.ibs_eilvt_off
					   | IBSCTL_LVTOFFSETVAL);
		pci_read_ext_config_dword(cpu_cfg, IBSCTL, &value);
		if (value != (pfm_amd64_pmu_info.ibs_eilvt_off
			      | IBSCTL_LVTOFFSETVAL)) {
			PFM_DBG("Failed to setup IBS LVT offset, "
				"IBSCTL = 0x%08x", value);
			return 1;
		}
	} while (1);

	if (!nodes) {
		PFM_DBG("No CPU node configured for IBS");
		return 1;
	}

#ifdef CONFIG_X86_64
	/* Sanity check */
	/* Works only for 64bit with proper numa implementation. */
	if (nodes != num_possible_nodes()) {
		PFM_DBG("Failed to setup CPU node(s) for IBS, "
			"found: %d, expected %d",
			nodes, num_possible_nodes());
		return 1;
	}
#endif
	return 0;
}

/*
 * There can only be one user per socket for the Northbridge (NB) events,
 * so we enforce mutual exclusion as follows:
 * 	- per-thread : only one context machine-wide can use NB events
 * 	- system-wide: only one context per processor socket
 *
 * Exclusion is enforced at:
 * 	- pfm_load_context()
 * 	- pfm_write_pmcs() for attached contexts
 *
 * Exclusion is released at:
 * 	- pfm_unload_context() or any calls that implicitely uses it
 *
 * return:
 * 	0  : successfully acquire NB access
 * 	< 0:  errno, failed to acquire NB access
 */
static int pfm_amd64_acquire_nb(struct pfm_context *ctx)
{
	struct pfm_context **entry, *old;
	int proc_id;

#ifdef CONFIG_SMP
	proc_id = cpu_data(smp_processor_id()).phys_proc_id;
#else
	proc_id = 0;
#endif

	if (ctx->flags.system)
		entry = &pfm_nb_sys_owners[proc_id];
	else
		entry = &pfm_nb_task_owner;

	old = cmpxchg(entry, NULL, ctx);
	if (!old) {
		if (ctx->flags.system)
			PFM_DBG("acquired Northbridge event access on socket %u", proc_id);
		else
			PFM_DBG("acquired Northbridge event access globally");
	} else if (old != ctx) {
		if (ctx->flags.system)
			PFM_DBG("NorthBridge event conflict on socket %u", proc_id);
		else
			PFM_DBG("global NorthBridge event conflict");
		return -EBUSY;
	}
	return 0;
}

/*
 * invoked from pfm_write_pmcs() when pfm_nb_sys_owners is not NULL,i.e.,
 * when we have detected a multi-core processor.
 *
 * context is locked, interrupts are masked
 */
static int pfm_amd64_pmc_write_check(struct pfm_context *ctx,
			     struct pfm_event_set *set,
			     struct pfarg_pmc *req)
{
	unsigned int event;

	/*
	 * delay checking NB event until we load the context
	 */
	if (ctx->state == PFM_CTX_UNLOADED)
		return 0;

	/*
	 * check event is NB event
	 */
	event = (unsigned int)(req->reg_value & 0xff);
	if (event < 0xee)
		return 0;

	return pfm_amd64_acquire_nb(ctx);
}

/*
 * invoked on pfm_load_context().
 * context is locked, interrupts are masked
 */
static int pfm_amd64_load_context(struct pfm_context *ctx)
{
	struct pfm_event_set *set;
	unsigned int i, n;

	/*
	 * scan all sets for NB events
	 */
	list_for_each_entry(set, &ctx->set_list, list) {
		n = set->nused_pmcs;
		for (i = 0; n; i++) {
			if (!test_bit(i, cast_ulp(set->used_pmcs)))
				continue;

			if (!is_ibs(i) && (set->pmcs[i] & 0xff) >= 0xee)
				goto found;
			n--;
		}
	}
	return 0;
found:
	return pfm_amd64_acquire_nb(ctx);
}

/*
 * invoked on pfm_unload_context()
 */
static int pfm_amd64_unload_context(struct pfm_context *ctx)
{
	struct pfm_context **entry, *old;
	int proc_id;

#ifdef CONFIG_SMP
	proc_id = cpu_data(smp_processor_id()).phys_proc_id;
#else
	proc_id = 0;
#endif

	/*
	 * unload always happens on the monitored CPU in system-wide
	 */
	if (ctx->flags.system)
		entry = &pfm_nb_sys_owners[proc_id];
	else
		entry = &pfm_nb_task_owner;

	old = cmpxchg(entry, ctx, NULL);
	if (old == ctx) {
		if (ctx->flags.system)
			PFM_DBG("released NorthBridge on socket %u", proc_id);
		else
			PFM_DBG("released NorthBridge events globally");
	}
	return 0;
}

/*
 * detect if we need to activate NorthBridge event access control
 */
static int pfm_amd64_setup_nb_event_control(void)
{
	unsigned int c, n = 0;
	unsigned int max_phys = 0;

#ifdef CONFIG_SMP
	for_each_possible_cpu(c) {
		if (cpu_data(c).phys_proc_id > max_phys)
			max_phys = cpu_data(c).phys_proc_id;
	}
#else
	max_phys = 0;
#endif
	if (max_phys > 255) {
		PFM_INFO("socket id %d is too big to handle", max_phys);
		return -ENOMEM;
	}

	n = max_phys + 1;
	if (n < 2)
		return 0;

	pfm_nb_sys_owners = vmalloc(n * sizeof(*pfm_nb_sys_owners));
	if (!pfm_nb_sys_owners)
		return -ENOMEM;

	memset(pfm_nb_sys_owners, 0, n * sizeof(*pfm_nb_sys_owners));
	pfm_nb_task_owner = NULL;

	/*
	 * activate write-checker for PMC registers
	 */
	for (c = 0; c < PFM_AMD_NUM_PMCS; c++) {
		if (!is_ibs(c))
			pfm_amd64_pmc_desc[c].type |= PFM_REG_WC;
	}

	pfm_amd64_pmu_info.load_context = pfm_amd64_load_context;
	pfm_amd64_pmu_info.unload_context = pfm_amd64_unload_context;

	pfm_amd64_pmu_conf.pmc_write_check = pfm_amd64_pmc_write_check;

	PFM_INFO("NorthBridge event access control enabled");

	return 0;
}

/*
 * disable registers which are not available on
 * the host (applies to IBS registers)
 */
static void pfm_amd64_check_registers(void)
{
	struct pfm_arch_ext_reg *ext_reg;
	u16 i, has_ibs, has_ibsext;

	has_ibs = pfm_amd64_pmu_info.flags & PFM_X86_FL_IBS;
	has_ibsext = pfm_amd64_pmu_info.flags & PFM_X86_FL_IBS_EXT;

	PFM_DBG("has_ibs=%d has_ibs_ext=%d", has_ibs, has_ibsext);

	/*
	 * Scan PMC registers
	 */
	ext_reg = pfm_amd64_pmu_info.pmc_addrs;
	for (i = 0; i < PFM_AMD_NUM_PMCS;  i++, ext_reg++) {
		if (!has_ibs && ext_reg->reg_type & PFM_REGT_IBS) {
			ext_reg->reg_type = PFM_REGT_NA;
			pfm_amd64_pmc_desc[i].type = PFM_REG_NA;
			PFM_DBG("pmc%u not available", i);
		}
		if (!has_ibsext && ext_reg->reg_type & PFM_REGT_IBS_EXT) {
			ext_reg->reg_type = PFM_REGT_NA;
			pfm_amd64_pmc_desc[i].type = PFM_REG_NA;
			PFM_DBG("pmc%u not available", i);
		}
	}

	/*
	 * Scan PMD registers
	 */
	ext_reg = pfm_amd64_pmu_info.pmd_addrs;
	for (i = 0; i < PFM_AMD_NUM_PMDS;  i++, ext_reg++) {
		if (!has_ibs && ext_reg->reg_type & PFM_REGT_IBS) {
			ext_reg->reg_type = PFM_REGT_NA;
			pfm_amd64_pmd_desc[i].type = PFM_REG_NA;
			PFM_DBG("pmd%u not available", i);
		}
		if (!has_ibsext && ext_reg->reg_type & PFM_REGT_IBS_EXT) {
			ext_reg->reg_type = PFM_REGT_NA;
			pfm_amd64_pmd_desc[i].type = PFM_REG_NA;
			PFM_DBG("pmd%u not available", i);
		}

		/*
		 * adjust reserved mask for counters
		 */
		if (ext_reg->reg_type & PFM_REGT_CTR)
			pfm_amd64_pmd_desc[i].rsvd_msk = ~((1ULL<<48)-1);
	}
	/*
	 * adjust reserved bit fields for family 16
	 */
	if (current_cpu_data.x86 == 16) {
		for(i=0; i < PFM_AMD_NUM_PMCS; i++)
			if (pfm_amd64_pmc_desc[i].rsvd_msk == PFM_K8_RSVD)
				pfm_amd64_pmc_desc[i].rsvd_msk = PFM_16_RSVD;
	}
}

static int pfm_amd64_probe_pmu(void)
{
	u64 val = 0;
	if (current_cpu_data.x86_vendor != X86_VENDOR_AMD) {
		PFM_INFO("not an AMD processor");
		return -1;
	}

	switch (current_cpu_data.x86) {
	case 16:
		if (current_cpu_data.x86_model >= 2) {
			/* Family 10h, RevB and later */
			pfm_amd64_pmu_info.flags |= PFM_X86_FL_IBS_EXT
				| PFM_X86_FL_USE_EI;
		}
		pfm_amd64_pmu_info.flags |= PFM_X86_FL_IBS;
		rdmsrl(MSR_AMD64_IBSCTL, val);
	case 15:
	case  6:
		PFM_INFO("found family=%d VAL=0x%llx", current_cpu_data.x86, (unsigned long long)val);
		break;
	default:
		PFM_INFO("unsupported family=%d", current_cpu_data.x86);
		return -1;
	}

	/*
	 * check for local APIC (required)
	 */
	if (!cpu_has_apic) {
		PFM_INFO("no local APIC, unsupported");
		return -1;
	}

	if (current_cpu_data.x86_max_cores > 1
	    && pfm_amd64_setup_nb_event_control())
		return -1;

	if (force_nmi)
		pfm_amd64_pmu_info.flags |= PFM_X86_FL_USE_NMI;

	/* Setup extended interrupt */
	if (pfm_amd64_pmu_info.flags & PFM_X86_FL_USE_EI) {
		if (pfm_amd64_setup_eilvt()) {
			PFM_INFO("Failed to initialize extended interrupts "
				 "for IBS");
			pfm_amd64_pmu_info.flags &= ~(PFM_X86_FL_IBS
					      | PFM_X86_FL_IBS_EXT
					      | PFM_X86_FL_USE_EI);
			PFM_INFO("Unable to use IBS");
		}
	}

	if (pfm_amd64_pmu_info.flags & PFM_X86_FL_IBS)
		PFM_INFO("IBS supported");

	if (pfm_amd64_pmu_info.flags & PFM_X86_FL_IBS_EXT)
		PFM_INFO("IBS extended registers supported");

	if (pfm_amd64_pmu_info.flags & PFM_X86_FL_USE_EI)
		PFM_INFO("Using extended interrupts for IBS");
	else if (pfm_amd64_pmu_info.flags & (PFM_X86_FL_IBS|PFM_X86_FL_IBS_EXT))
		PFM_INFO("Using performance counter interrupts for IBS");

	pfm_amd64_check_registers();

	return 0;
}

static struct pfm_pmu_config pfm_amd64_pmu_conf = {
	.pmu_name = "AMD64",
	.counter_width = 47,
	.pmd_desc = pfm_amd64_pmd_desc,
	.pmc_desc = pfm_amd64_pmc_desc,
	.num_pmc_entries = PFM_AMD_NUM_PMCS,
	.num_pmd_entries = PFM_AMD_NUM_PMDS,
	.probe_pmu = pfm_amd64_probe_pmu,
	.version = "1.2",
	.arch_info = &pfm_amd64_pmu_info,
	.flags = PFM_PMU_BUILTIN_FLAG,
	.owner = THIS_MODULE,
};

static int __init pfm_amd64_pmu_init_module(void)
{
	return pfm_pmu_register(&pfm_amd64_pmu_conf);
}

static void __exit pfm_amd64_pmu_cleanup_module(void)
{
	if (pfm_nb_sys_owners)
		vfree(pfm_nb_sys_owners);

	pfm_pmu_unregister(&pfm_amd64_pmu_conf);
}

module_init(pfm_amd64_pmu_init_module);
module_exit(pfm_amd64_pmu_cleanup_module);
