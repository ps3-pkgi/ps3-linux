/*
 * perfmon_pmu.c: perfmon2 PMU configuration management
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
#include <linux/module.h>
#include <linux/perfmon.h>

#ifndef CONFIG_MODULE_UNLOAD
#define module_refcount(n)	1
#endif

static __cacheline_aligned_in_smp int request_mod_in_progress;
static __cacheline_aligned_in_smp DEFINE_SPINLOCK(pfm_pmu_conf_lock);

static __cacheline_aligned_in_smp DEFINE_SPINLOCK(pfm_pmu_acq_lock);
static u32 pfm_pmu_acquired;

/*
 * perfmon core must acces PMU information ONLY through pfm_pmu_conf
 * if pfm_pmu_conf is NULL, then no description is registered
 */
struct pfm_pmu_config	*pfm_pmu_conf;
EXPORT_SYMBOL(pfm_pmu_conf);

static inline int pmu_is_module(struct pfm_pmu_config *c)
{
	return !(c->flags & PFM_PMUFL_IS_BUILTIN);
}

/*
 * compute the following:
 * 	- max_pmc, num_pmcs max_pmd, num_pmds
 * 	- first_intr_pmd, max_rw_pmd
 * based on existing regdesc->pmds and regdesc->pmcs
 */
static void pfm_pmu_regdesc_calc_limits(struct pfm_regdesc *d)
{
	u16 n, n2, n_counters, i;
	int max1, max2, max3, first_intr, first_i;

	n = 0;
	max1 = max2 = -1;
	for (i = 0; i < pfm_pmu_conf->num_pmc_entries;  i++) {
		if (!test_bit(i, cast_ulp(d->pmcs)))
			continue;
		max1 = i;
		n++;
	}
	d->max_pmc = max1 + 1;
	d->num_pmcs = n;

	n = n_counters = n2 = 0;
	max1 = max2 = max3 = first_intr = first_i = -1;
	for (i = 0; i < pfm_pmu_conf->num_pmd_entries;  i++) {
		if (!test_bit(i, cast_ulp(d->pmds)))
			continue;

		if (first_i == -1)
			first_i = i;

		max1 = i;
		n++;

		/*
		 * read-write registers
		 */
		if (!(pfm_pmu_conf->pmd_desc[i].type & PFM_REG_RO)) {
			max3 = i;
			n2++;
		}

		/*
		 * counters registers
		 */
		if (pfm_pmu_conf->pmd_desc[i].type & PFM_REG_C64)
			n_counters++;

		if (pfm_pmu_conf->pmd_desc[i].type & PFM_REG_INTR) {
			max2 = i;
			if (first_intr == -1)
				first_intr = i;
		}
	}
	d->max_pmd = max1 + 1;
	d->first_intr_pmd = first_intr == -1 ?  first_i : first_intr;

	d->max_intr_pmd  = max2 + 1;

	d->num_counters = n_counters;
	d->num_pmds = n;
	d->max_rw_pmd = max3 + 1;
	d->num_rw_pmd = n2;
}

static int pfm_regdesc_init(struct pfm_regdesc *d, struct pfm_pmu_config *cfg)
{
	u16 n, n2, n_counters, i;
	int max1, max2, max3, first_intr, first_i;

	memset(d, 0 , sizeof(*d));
	/*
	 * compute the number of implemented PMC from the
	 * description table
	 */
	n = 0;
	max1 = max2 = -1;
	for (i = 0; i < cfg->num_pmc_entries;  i++) {
		if (!(cfg->pmc_desc[i].type & PFM_REG_I))
			continue;

		__set_bit(i, cast_ulp(d->pmcs));

		max1 = i;
		n++;
	}

	if (!n) {
		PFM_INFO("%s PMU description has no PMC registers",
			 cfg->pmu_name);
		return -EINVAL;
	}

	d->max_pmc = max1 + 1;
	d->num_pmcs = n;

	n = n_counters = n2 = 0;
	max1 = max2 = max3 = first_intr = first_i = -1;
	for (i = 0; i < cfg->num_pmd_entries;  i++) {
		if (!(cfg->pmd_desc[i].type & PFM_REG_I))
			continue;

		if (first_i == -1)
			first_i = i;

		__set_bit(i, cast_ulp(d->pmds));
		max1 = i;
		n++;

		/*
		 * read-write registers
		 */
		if (!(cfg->pmd_desc[i].type & PFM_REG_RO)) {
			__set_bit(i, cast_ulp(d->rw_pmds));
			max3 = i;
			n2++;
		}

		/*
		 * counters registers
		 */
		if (cfg->pmd_desc[i].type & PFM_REG_C64) {
			__set_bit(i, cast_ulp(d->cnt_pmds));
			n_counters++;
		}

		/*
		 * PMD with intr capabilities
		 */
		if (cfg->pmd_desc[i].type & PFM_REG_INTR) {
			__set_bit(i, cast_ulp(d->intr_pmds));
			max2 = i;
			if (first_intr == -1)
				first_intr = i;
		}
	}

	if (!n) {
		PFM_INFO("%s PMU description has no PMD registers",
			 cfg->pmu_name);
		return -EINVAL;
	}

	d->max_pmd = max1 + 1;
	d->first_intr_pmd = first_intr == -1 ?  first_i : first_intr;

	d->max_intr_pmd  = max2 + 1;

	d->num_counters = n_counters;
	d->num_pmds = n;
	d->max_rw_pmd = max3 + 1;
	d->num_rw_pmd = n2;

	return 0;
}

/*
 * initialize PMU configuration from PMU config descriptor
 */
static int pfm_pmu_config_init(struct pfm_pmu_config *cfg)
{
	int ret;

	/*
	 * we build the register description using the full mapping
	 * table as defined by the module. On first use, we update
	 * the current description (regs) based on local constraints.
	 */
	ret = pfm_regdesc_init(&cfg->full_regs, cfg);
	if (ret)
		return ret;

	if (!cfg->version)
		cfg->version = "0.0";

	pfm_pmu_conf = cfg;
	pfm_pmu_conf->ovfl_mask = (1ULL << cfg->counter_width) -1;

	PFM_INFO("%s PMU detected, %u PMCs, %u PMDs, %u counters (%u bits)",
		 pfm_pmu_conf->pmu_name,
		 pfm_pmu_conf->full_regs.num_pmcs,
		 pfm_pmu_conf->full_regs.num_pmds,
		 pfm_pmu_conf->full_regs.num_counters,
		 pfm_pmu_conf->counter_width);

	return 0;
}

int pfm_pmu_register(struct pfm_pmu_config *cfg)
{
	u16 i, nspec, nspec_ro, num_pmcs, num_pmds, num_wc = 0;
	int type, ret = -EBUSY;

	if (perfmon_disabled) {
		PFM_INFO("perfmon disabled, cannot add PMU description");
		return -ENOSYS;
	}

	nspec = nspec_ro = num_pmds = num_pmcs = 0;

	/* some sanity checks */
	if (cfg == NULL || cfg->pmu_name == NULL) {
		PFM_INFO("PMU config descriptor is invalid");
		return -EINVAL;
	}

	/* must have a probe */
	if (cfg->probe_pmu == NULL) {
		PFM_INFO("PMU config has no probe routine");
		return -EINVAL;
	}

	/*
	 * execute probe routine before anything else as it
	 * may update configuration tables
	 */
	if ((*cfg->probe_pmu)() == -1) {
		PFM_INFO("%s PMU detection failed", cfg->pmu_name);
		return -EINVAL;
	}

	if (!(cfg->flags & PFM_PMUFL_IS_BUILTIN) && cfg->owner == NULL) {
		PFM_INFO("PMU config %s is missing owner", cfg->pmu_name);
		return -EINVAL;
	}

	if (!cfg->num_pmd_entries) {
		PFM_INFO("%s needs to define num_pmd_entries", cfg->pmu_name);
		return -EINVAL;
	}

	if (!cfg->num_pmc_entries) {
		PFM_INFO("%s needs to define num_pmc_entries", cfg->pmu_name);
		return -EINVAL;
	}

	if (!cfg->counter_width) {
		PFM_INFO("PMU config %s, zero width counters", cfg->pmu_name);
		return -EINVAL;
	}

	/*
	 * REG_RO, REG_V not supported on PMC registers
	 */
	for (i = 0; i < cfg->num_pmc_entries;  i++) {

		type = cfg->pmc_desc[i].type;

		if (type & PFM_REG_I)
			num_pmcs++;

		if (type & PFM_REG_WC)
			num_wc++;

		if (type & PFM_REG_V) {
			PFM_INFO("PFM_REG_V is not supported on "
				 "PMCs (PMC%d)", i);
			return -EINVAL;
		}
		if (type & PFM_REG_RO) {
			PFM_INFO("PFM_REG_RO meaningless on "
				 "PMCs (PMC%u)", i);
			return -EINVAL;
		}
	}

	if (num_wc && cfg->pmc_write_check == NULL) {
		PFM_INFO("some PMCs have write-checker but no callback provided\n");
		return -EINVAL;
	}

	/*
	 * check virtual PMD registers
	 */
	num_wc= 0;
	for (i = 0; i < cfg->num_pmd_entries;  i++) {

		type = cfg->pmd_desc[i].type;

		if (type & PFM_REG_I)
			num_pmds++;

		if (type & PFM_REG_V) {
			nspec++;
			if (type & PFM_REG_RO)
				nspec_ro++;
		}

		if (type & PFM_REG_WC)
			num_wc++;
	}

	if (num_wc && cfg->pmd_write_check == NULL) {
		PFM_INFO("PMD have write-checker but no callback provided\n");
		return -EINVAL;
	}

	if (nspec && cfg->pmd_sread == NULL) {
		PFM_INFO("PMU config is missing pmd_sread()");
		return -EINVAL;
	}

	nspec = nspec - nspec_ro;
	if (nspec && cfg->pmd_swrite == NULL) {
		PFM_INFO("PMU config is missing pmd_swrite()");
		return -EINVAL;
	}

	if (num_pmcs >= PFM_MAX_PMCS) {
		PFM_INFO("%s PMCS registers exceed name space [0-%u]",
			 cfg->pmu_name,
			 PFM_MAX_PMCS);
		return -EINVAL;
	}
	if (num_pmds >= PFM_MAX_PMDS) {
		PFM_INFO("%s PMDS registers exceed name space [0-%u]",
			 cfg->pmu_name,
			 PFM_MAX_PMDS);
		return -EINVAL;
	}
	spin_lock(&pfm_pmu_conf_lock);

	if (pfm_pmu_conf)
		goto unlock;

	ret = pfm_pmu_config_init(cfg);
	if (ret)
		goto unlock;

	ret = pfm_arch_pmu_config_init(pfm_pmu_conf);
	if (ret)
		goto unlock;

	ret = pfm_sysfs_add_pmu(pfm_pmu_conf);
	if (ret) {
		pfm_arch_pmu_config_remove();
		pfm_pmu_conf = NULL;
	}

unlock:
	spin_unlock(&pfm_pmu_conf_lock);

	if (ret) {
		PFM_INFO("register %s PMU error %d", cfg->pmu_name, ret);
	} else {
		PFM_INFO("%s PMU installed", cfg->pmu_name);
		/*
		 * (re)initialize PMU on each PMU now that we have a description
		 */
		on_each_cpu(__pfm_init_percpu, cfg, 0, 0);
	}
	return ret;
}
EXPORT_SYMBOL(pfm_pmu_register);

/*
 * remove PMU description. Caller must pass address of current
 * configuration. This is mostly for sanity checking as only
 * one config can exist at any time.
 *
 * We are using the module refcount mechanism to protect against
 * removal while the configuration is being used. As long as there is
 * one context, a PMU configuration cannot be removed. The protection is
 * managed in module logic.
 */
void pfm_pmu_unregister(struct pfm_pmu_config *cfg)
{
	if (!(cfg ||pfm_pmu_conf))
		return;

	spin_lock(&pfm_pmu_conf_lock);

	BUG_ON(module_refcount(pfm_pmu_conf->owner));

	if (cfg->owner == pfm_pmu_conf->owner) {
		pfm_arch_pmu_config_remove();
		pfm_sysfs_remove_pmu(pfm_pmu_conf);
		pfm_pmu_conf = NULL;
	}

	spin_unlock(&pfm_pmu_conf_lock);
}
EXPORT_SYMBOL(pfm_pmu_unregister);

static int pfm_pmu_request_module(void)
{
	char *mod_name;
	int ret;

	mod_name = pfm_arch_get_pmu_module_name();
	if (mod_name == NULL)
		return -ENOSYS;

	ret = request_module(mod_name);

	PFM_DBG("mod=%s ret=%d\n", mod_name, ret);
	return ret;
}

/*
 * autoload:
 * 	0     : do not try to autoload the PMU description module
 * 	not 0 : try to autoload the PMU description module
 */
int pfm_pmu_conf_get(int autoload)
{
	int ret;

	spin_lock(&pfm_pmu_conf_lock);

	if (request_mod_in_progress) {
		ret = -ENOSYS;
		goto skip;
	}

	if (autoload && pfm_pmu_conf == NULL) {

		request_mod_in_progress = 1;

		spin_unlock(&pfm_pmu_conf_lock);

		pfm_pmu_request_module();

		spin_lock(&pfm_pmu_conf_lock);

		request_mod_in_progress = 0;

		/*
		 * request_module() may succeed but the module
		 * may not have registered properly so we need
		 * to check
		 */
	}

	ret = pfm_pmu_conf == NULL ? -ENOSYS : 0;
	if (!ret && pmu_is_module(pfm_pmu_conf)
	    && !try_module_get(pfm_pmu_conf->owner))
		ret = -ENOSYS;
skip:
	spin_unlock(&pfm_pmu_conf_lock);

	return ret;
}

void pfm_pmu_conf_put(void)
{
	if (pfm_pmu_conf == NULL || !pmu_is_module(pfm_pmu_conf))
		return;

	spin_lock(&pfm_pmu_conf_lock);
	module_put(pfm_pmu_conf->owner);
	spin_unlock(&pfm_pmu_conf_lock);
}


/*
 * acquire PMU resource from lower-level PMU register allocator
 * (currently perfctr-watchdog.c)
 *
 * acquisition is done when the first context is created (and not
 * when it is loaded). We grab all that is defined in the description
 * module and then we make adjustments at the arch-specific level.
 *
 * The PMU resource is released when the last perfmon context is
 * destroyed.
 *
 * interrupts are not masked
 */
int pfm_pmu_acquire(void)
{
	int ret = 0;

	spin_lock(&pfm_pmu_acq_lock);

	PFM_DBG("pmu_acquired=%d", pfm_pmu_acquired);

	pfm_pmu_acquired++;

	if (pfm_pmu_acquired == 1) {
		/*
		 * copy full description and then check if arch-specific
		 * layer needs some adjustments
		 */
		pfm_pmu_conf->regs = pfm_pmu_conf->full_regs;

		ret = pfm_arch_pmu_acquire();
		if (ret) {
			pfm_pmu_acquired--;
		} else {
			/*
			 * calculate new limits (num, max)
			 */
			pfm_pmu_regdesc_calc_limits(&pfm_pmu_conf->regs);

			/* available PMU ressources */
			PFM_DBG("PMU acquired: %u PMCs, %u PMDs, %u counters",
				pfm_pmu_conf->regs.num_pmcs,
				pfm_pmu_conf->regs.num_pmds,
				pfm_pmu_conf->regs.num_counters);
		}
	}
	spin_unlock(&pfm_pmu_acq_lock);

	return ret;
}

/*
 * release the PMU resource
 *
 * actual release happens when last context is destroyed
 *
 * interrupts are not masked
 */
void pfm_pmu_release(void)
{
	BUG_ON(irqs_disabled());

	/*
	 * we need to use a spinlock because release takes some time
	 * and we may have a race with pfm_pmu_acquire()
	 */
	spin_lock(&pfm_pmu_acq_lock);

	PFM_DBG("pmu_acquired=%d", pfm_pmu_acquired);

	/*
	 * we decouple test and decrement because if we had errors
	 * in pfm_pmu_acquire(), we still come here on pfm_context_free()
	 * but with pfm_pmu_acquire=0
	 */
	if (pfm_pmu_acquired > 0 && --pfm_pmu_acquired == 0) {
		pfm_arch_pmu_release();
		memset(&pfm_pmu_conf->regs, 0, sizeof(pfm_pmu_conf->regs));
		PFM_DBG("PMU released");
	}
	spin_unlock(&pfm_pmu_acq_lock);
}
