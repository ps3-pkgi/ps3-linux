/*
 * perfmon_sysfs.c: perfmon2 sysfs interface
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
#include <linux/module.h> /* for EXPORT_SYMBOL */
#include <linux/perfmon_kern.h>


extern void pfm_reset_stats(int cpu);

struct pfm_attribute {
	struct attribute attr;
	ssize_t (*show)(void *, char *);
	ssize_t (*store)(void *, const char *, size_t);
};
#define to_attr(n) container_of(n, struct pfm_attribute, attr);

#define PFM_RO_ATTR(_name) \
struct pfm_attribute attr_##_name = __ATTR_RO(_name)

#define PFM_RW_ATTR(_name,_mode,_show,_store) 			\
struct pfm_attribute attr_##_name = __ATTR(_name,_mode,_show,_store);

int pfm_sysfs_add_pmu(struct pfm_pmu_config *pmu);

struct pfm_controls pfm_controls = {
	.sys_group = PFM_GROUP_PERM_ANY,
	.task_group = PFM_GROUP_PERM_ANY,
	.arg_mem_max = PAGE_SIZE,
	.smpl_buffer_mem_max = ~0,
};
EXPORT_SYMBOL(pfm_controls);

DECLARE_PER_CPU(struct pfm_stats, pfm_stats);

static struct kobject pfm_kernel_kobj, pfm_kernel_fmt_kobj;

static ssize_t pfm_fmt_attr_show(struct kobject *kobj,
		struct attribute *attr, char *buf)
{
	struct pfm_smpl_fmt *fmt = to_smpl_fmt(kobj);
	struct pfm_attribute *attribute = to_attr(attr);
	return attribute->show ? attribute->show(fmt, buf) : -EIO;
}

static ssize_t pfm_pmu_attr_show(struct kobject *kobj,
		struct attribute *attr, char *buf)
{
	struct pfm_pmu_config *pmu= to_pmu(kobj);
	struct pfm_attribute *attribute = to_attr(attr);
	return attribute->show ? attribute->show(pmu, buf) : -EIO;
}

static ssize_t pfm_regs_attr_show(struct kobject *kobj,
		struct attribute *attr, char *buf)
{
	struct pfm_regmap_desc *reg = to_reg(kobj);
	struct pfm_attribute *attribute = to_attr(attr);
	return attribute->show ? attribute->show(reg, buf) : -EIO;
}


static struct sysfs_ops pfm_fmt_sysfs_ops = {
	.show = pfm_fmt_attr_show
};

static struct sysfs_ops pfm_pmu_sysfs_ops = {
	.show = pfm_pmu_attr_show
};

static struct sysfs_ops pfm_regs_sysfs_ops = {
	.show  = pfm_regs_attr_show
};

static struct kobj_type pfm_fmt_ktype = {
	.sysfs_ops = &pfm_fmt_sysfs_ops,
};

static struct kobj_type pfm_pmu_ktype = {
	.sysfs_ops = &pfm_pmu_sysfs_ops,
};

static struct kobj_type pfm_regs_ktype = {
	.sysfs_ops = &pfm_regs_sysfs_ops,
};

static ssize_t version_show(void *info, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u.%u\n",  PFM_VERSION_MAJ, PFM_VERSION_MIN);
}

static ssize_t task_sessions_count_show(void *info, char *buf)
{
	return pfm_sysfs_res_show(buf, PAGE_SIZE, 0);
}

static ssize_t sys_sessions_count_show(void *info, char *buf)
{
	return pfm_sysfs_res_show(buf, PAGE_SIZE, 1);
}

static ssize_t smpl_buffer_mem_cur_show(void *info, char *buf)
{
	return pfm_sysfs_res_show(buf, PAGE_SIZE, 2);
}

static ssize_t debug_show(void *info, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", pfm_controls.debug);
}

static ssize_t debug_store(void *info, const char *buf, size_t sz)
{
	int d, i;

	if (sscanf(buf,"%d", &d) != 1)
		return -EINVAL;

	pfm_controls.debug = d;

	if (d == 0) {
		for_each_online_cpu(i) {
			pfm_reset_stats(i);
		}
	}
	return sz;
}

static ssize_t debug_ovfl_show(void *info, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", pfm_controls.debug_ovfl);
}

static ssize_t debug_ovfl_store(void *info, const char *buf, size_t sz)
{
	int d;

	if (sscanf(buf,"%d", &d) != 1)
		return -EINVAL;

	pfm_controls.debug_ovfl = d;

	return strnlen(buf, PAGE_SIZE);
}

static ssize_t reset_stats_show(void *info, char *buf)
{
	buf[0]='0';
	buf[1]='\0';
	return strnlen(buf, PAGE_SIZE);
}

static ssize_t reset_stats_store(void *info, const char *buf, size_t count)
{
	int i;

	for_each_online_cpu(i) {
		pfm_reset_stats(i);
	}
	return count;
}

static ssize_t sys_group_show(void *info, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", pfm_controls.sys_group);
}

static ssize_t sys_group_store(void *info, const char *buf, size_t sz)
{
	int d;

	if (sscanf(buf,"%d", &d) != 1)
		return -EINVAL;

	pfm_controls.sys_group = d;

	return strnlen(buf, PAGE_SIZE);
}

static ssize_t task_group_show(void *info, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", pfm_controls.task_group);
}

static ssize_t task_group_store(void *info, const char *buf, size_t sz)
{
	int d;

	if (sscanf(buf,"%d", &d) != 1)
		return -EINVAL;

	pfm_controls.task_group = d;

	return strnlen(buf, PAGE_SIZE);
}

static ssize_t buf_size_show(void *info, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%zu\n", pfm_controls.smpl_buffer_mem_max);
}

static ssize_t buf_size_store(void *info, const char *buf, size_t sz)
{
	size_t d;

	if (sscanf(buf,"%zu", &d) != 1)
		return -EINVAL;
	/*
	 * we impose a page as the minimum
	 */
	if (d < PAGE_SIZE)
		return -EINVAL;

	pfm_controls.smpl_buffer_mem_max = d;

	return strnlen(buf, PAGE_SIZE);
}

static ssize_t arg_size_show(void *info, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%zu\n", pfm_controls.arg_mem_max);
}

static ssize_t arg_size_store(void *info, const char *buf, size_t sz)
{
	size_t d;

	if (sscanf(buf,"%zu", &d) != 1)
		return -EINVAL;

	/*
	 * we impose a page as the minimum.
	 *
	 * This limit may be smaller than the stack buffer
	 * available and that is fine.
	 */
	if (d < PAGE_SIZE)
		return -EINVAL;

	pfm_controls.arg_mem_max = d;

	return strnlen(buf, PAGE_SIZE);
}

static int __init enable_debug(char *str)
{
	pfm_controls.debug = 1;
	PFM_INFO("debug output enabled\n");
	return 1;
}
__setup("perfmon_debug", enable_debug);

/*
 * /sys/kernel/perfmon attributes
 */
static PFM_RO_ATTR(version);
static PFM_RO_ATTR(task_sessions_count);
static PFM_RO_ATTR(sys_sessions_count);
static PFM_RO_ATTR(smpl_buffer_mem_cur);

static PFM_RW_ATTR(debug, 0644, debug_show, debug_store);
static PFM_RW_ATTR(debug_ovfl, 0644, debug_ovfl_show, debug_ovfl_store);
static PFM_RW_ATTR(reset_stats, 0644, reset_stats_show, reset_stats_store);
static PFM_RW_ATTR(sys_group, 0644, sys_group_show, sys_group_store);
static PFM_RW_ATTR(task_group, 0644, task_group_show, task_group_store);
static PFM_RW_ATTR(smpl_buffer_mem_max, 0644, buf_size_show, buf_size_store);
static PFM_RW_ATTR(arg_mem_max, 0644, arg_size_show, arg_size_store);

static struct attribute *pfm_kernel_attrs[] = {
	&attr_version.attr,
	&attr_task_sessions_count.attr,
	&attr_sys_sessions_count.attr,
	&attr_smpl_buffer_mem_cur.attr,
	&attr_debug.attr,
	&attr_debug_ovfl.attr,
	&attr_reset_stats.attr,
	&attr_sys_group.attr,
	&attr_task_group.attr,
	&attr_smpl_buffer_mem_max.attr,
	&attr_arg_mem_max.attr,
	NULL
};

static ssize_t pfm_kernel_show(struct kobject * kobj,
			       struct attribute * attr ,char * buf)
{
	int ret;
	struct pfm_attribute *fattr = to_attr(attr);

	if (fattr->show)
		ret = fattr->show(NULL, buf);
	else
		ret = -EIO;

	return ret;
}

static ssize_t pfm_kernel_store(struct kobject * kobj,
			        struct attribute * attr,
			        const char * buf, size_t count)
{
	int ret;
	struct pfm_attribute *fattr = to_attr(attr);

	if (fattr->store)
		ret = fattr->store(NULL, buf, count);
	else
		ret = -EIO;
	return ret;
}

static struct sysfs_ops pfm_kernel_sysfs_ops = {
	.show	= pfm_kernel_show,
	.store	= pfm_kernel_store,
};

static struct kobj_type ktype_kernel = {
	.sysfs_ops	= &pfm_kernel_sysfs_ops,
	.default_attrs	= pfm_kernel_attrs,
};

int __init pfm_init_sysfs(void)
{
	int ret;

	kobject_init(&pfm_kernel_fmt_kobj, &pfm_fmt_ktype);

	ret = kobject_init_and_add(&pfm_kernel_kobj, &ktype_kernel,
				   kobject_get(kernel_kobj), "perfmon");
	if (ret) {
		PFM_ERR("cannot add kernel object: %d", ret);
		return ret;
	}

	ret = kobject_add(&pfm_kernel_fmt_kobj, &pfm_kernel_kobj, "%s", "formats");
	if (ret) {
		PFM_ERR("cannot add fmt object: %d", ret);
		goto error_fmt;
	}
	if (pfm_pmu_conf)
		pfm_sysfs_add_pmu(pfm_pmu_conf);

	pfm_sysfs_builtin_fmt_add();

	return 0;

error_fmt:
	kobject_del(&pfm_kernel_kobj);
	return ret;
}

/*
 * per-cpu perfmon stats attributes
 */
#define PFM_DECL_STATS_ATTR(name) \
static ssize_t name##_show(void *info, char *buf) \
{ \
	struct pfm_stats *st = info;\
	return snprintf(buf, PAGE_SIZE, "%llu\n", \
			(unsigned long long)st->name); \
} \
static PFM_RO_ATTR(name)

/*
 * per-reg attributes
 */
static ssize_t name_show(void *info, char *buf)
{
	struct pfm_regmap_desc *reg = info;
	return snprintf(buf, PAGE_SIZE, "%s\n", reg->desc);
}
static PFM_RO_ATTR(name);

static ssize_t dfl_val_show(void *info, char *buf)
{
	struct pfm_regmap_desc *reg = info;
	return snprintf(buf, PAGE_SIZE, "0x%llx\n",
			(unsigned long long)reg->dfl_val);
}
static PFM_RO_ATTR(dfl_val);

static ssize_t rsvd_msk_show(void *info, char *buf)
{
	struct pfm_regmap_desc *reg = info;
	return snprintf(buf, PAGE_SIZE, "0x%llx\n",
			(unsigned long long)reg->rsvd_msk);
}
static PFM_RO_ATTR(rsvd_msk);

static ssize_t width_show(void *info, char *buf)
{
	struct pfm_regmap_desc *reg = info;
	int w;

	w = (reg->type & PFM_REG_C64) ? pfm_pmu_conf->counter_width : 64;

	return snprintf(buf, PAGE_SIZE, "%d\n", w);
}
static PFM_RO_ATTR(width);


static ssize_t addr_show(void *info, char *buf)
{
	struct pfm_regmap_desc *reg = info;
	return snprintf(buf, PAGE_SIZE, "0x%lx\n", reg->hw_addr);
}
static PFM_RO_ATTR(addr);

static ssize_t fmt_version_show(void *data, char *buf)
{
	struct pfm_smpl_fmt *fmt = data;

	return snprintf(buf, PAGE_SIZE, "%u.%u",
		fmt->fmt_version >>16 & 0xffff,
		fmt->fmt_version & 0xffff);
}

/*
 * do not use predefined macros because of name conflict
 * with /sys/kernel/perfmon/version
 */
struct pfm_attribute attr_fmt_version = {
	.attr	= { .name = "version", .mode = 0444 },
	.show	= fmt_version_show,
};

static struct attribute *pfm_fmt_attrs[] = {
	&attr_fmt_version.attr,
	NULL
};

static struct attribute_group pfm_fmt_attr_group = {
	.attrs = pfm_fmt_attrs,
};


/*
 * when a sampling format module is inserted, we populate
 * sysfs with some information
 */
int pfm_sysfs_add_fmt(struct pfm_smpl_fmt *fmt)
{
	int ret;

	kobject_init(&fmt->kobj, &pfm_fmt_ktype);

	ret = kobject_add(&fmt->kobj, &pfm_kernel_fmt_kobj, "%s", fmt->fmt_name);
	if (ret)
		return ret;

	ret = sysfs_create_group(&fmt->kobj, &pfm_fmt_attr_group);
	if (ret)
		kobject_del(&fmt->kobj);

	return ret;
}

/*
 * when a sampling format module is removed, its information
 * must also be removed from sysfs
 */
void pfm_sysfs_remove_fmt(struct pfm_smpl_fmt *fmt)
{
	sysfs_remove_group(&fmt->kobj, &pfm_fmt_attr_group);
	kobject_del(&fmt->kobj);
}

static struct attribute *pfm_reg_attrs[] = {
	&attr_name.attr,
	&attr_dfl_val.attr,
	&attr_rsvd_msk.attr,
	&attr_width.attr,
	&attr_addr.attr,
	NULL
};

static struct attribute_group pfm_reg_attr_group = {
	.attrs = pfm_reg_attrs,
};

static ssize_t model_show(void *info, char *buf)
{
	struct pfm_pmu_config *p = info;
	return snprintf(buf, PAGE_SIZE, "%s\n", p->pmu_name);
}
static PFM_RO_ATTR(model);

static struct attribute *pfm_pmu_desc_attrs[] = {
	&attr_model.attr,
	NULL
};

static struct attribute_group pfm_pmu_desc_attr_group = {
	.attrs = pfm_pmu_desc_attrs,
};

static int pfm_sysfs_add_pmu_regs(struct pfm_pmu_config *pmu)
{
	struct pfm_regmap_desc *reg;
	unsigned int i, k;
	int ret;

	reg = pmu->pmc_desc;
	for(i=0; i < pmu->num_pmc_entries; i++, reg++) {

		if (!(reg->type & PFM_REG_I))
			continue;

		kobject_init(&reg->kobj, &pfm_regs_ktype);

		ret = kobject_add(&reg->kobj, &pmu->kobj, "pmc%u", i);
		if (ret)
			goto undo_pmcs;

		ret = sysfs_create_group(&reg->kobj, &pfm_reg_attr_group);
		if (ret) {
			kobject_del(&reg->kobj);
			goto undo_pmcs;
		}
	}

	reg = pmu->pmd_desc;
	for(i=0; i < pmu->num_pmd_entries; i++, reg++) {

		if (!(reg->type & PFM_REG_I))
			continue;

		kobject_init(&reg->kobj, &pfm_regs_ktype);

		ret = kobject_add(&reg->kobj, &pmu->kobj, "pmd%u", i);
		if (ret)
			goto undo_pmds;

		ret = sysfs_create_group(&reg->kobj, &pfm_reg_attr_group);
		if (ret) {
			kobject_del(&reg->kobj);
			goto undo_pmds;
		}
	}
	return 0;
undo_pmds:
	reg = pmu->pmd_desc;
	for(k = 0; k < i; k++, reg++) {
		if (!(reg->type & PFM_REG_I))
			continue;
		sysfs_remove_group(&reg->kobj, &pfm_reg_attr_group);
		kobject_del(&reg->kobj);
	}
	i = pmu->num_pmc_entries;
	/* fall through */
undo_pmcs:
	reg = pmu->pmc_desc;
	for(k=0; k < i; k++, reg++) {
		if (!(reg->type & PFM_REG_I))
			continue;
		sysfs_remove_group(&reg->kobj, &pfm_reg_attr_group);
		kobject_del(&reg->kobj);
	}
	return ret;
}

static int pfm_sysfs_del_pmu_regs(struct pfm_pmu_config *pmu)
{
	struct pfm_regmap_desc *reg;
	unsigned int i;

	reg = pmu->pmc_desc;
	for(i=0; i < pmu->regs.max_pmc; i++, reg++) {

		if (!(reg->type & PFM_REG_I))
			continue;

		sysfs_remove_group(&reg->kobj, &pfm_reg_attr_group);
		kobject_del(&reg->kobj);
	}

	reg = pmu->pmd_desc;
	for(i=0; i < pmu->regs.max_pmd; i++, reg++) {

		if (!(reg->type & PFM_REG_I))
			continue;

		sysfs_remove_group(&reg->kobj, &pfm_reg_attr_group);
		kobject_del(&reg->kobj);
	}
	return 0;
}

/*
 * when a PMU description module is inserted, we create
 * a pmu_desc subdir in sysfs and we populate it with
 * PMU specific information, such as register mappings
 */
int pfm_sysfs_add_pmu(struct pfm_pmu_config *pmu)
{
	int ret;

	kobject_init(&pmu->kobj, &pfm_pmu_ktype);

	ret = kobject_add(&pmu->kobj, &pfm_kernel_kobj, "%s", "pmu_desc");
	if (ret)
		return ret;

	ret = sysfs_create_group(&pmu->kobj, &pfm_pmu_desc_attr_group);
	if (ret)
		kobject_del(&pmu->kobj);

	ret = pfm_sysfs_add_pmu_regs(pmu);
	if (ret) {
		sysfs_remove_group(&pmu->kobj, &pfm_pmu_desc_attr_group);
		kobject_del(&pmu->kobj);
	}
	return ret;
}

/*
 * when a PMU description module is removed, we also remove
 * all its information from sysfs, i.e., the pmu_desc subdir
 * disappears
 */
int pfm_sysfs_remove_pmu(struct pfm_pmu_config *pmu)
{
	pfm_sysfs_del_pmu_regs(pmu);
	sysfs_remove_group(&pmu->kobj, &pfm_pmu_desc_attr_group);
	kobject_del(&pmu->kobj);

	return 0;
}
