/*
 * perfmon_res.c:  perfmon2 resource allocations
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
#include <linux/spinlock.h>
#include <linux/perfmon.h>
#include <linux/module.h>

/*
 * global information about all sessions
 * mostly used to synchronize between system wide and per-process
 */
struct pfm_sessions {
	u32		pfs_task_sessions;/* #num loaded per-thread sessions */
	size_t		pfs_smpl_buffer_mem_cur; /* current smpl buf mem usage */
	cpumask_t	pfs_sys_cpumask;  /* bitmask of used cpus (system-wide) */
};

static struct pfm_sessions pfm_sessions;

static __cacheline_aligned_in_smp DEFINE_SPINLOCK(pfm_sessions_lock);

/*
 * sampling buffer allocated by perfmon must be
 * checked against max usage thresholds for security
 * reasons.
 *
 * The first level check is against the system wide limit
 * as indicated by the system administrator in /proc/sys/kernel/perfmon
 *
 * The second level check is on a per-process basis using
 * RLIMIT_MEMLOCK limit.
 *
 * Operating on the current task only.
 */
int pfm_reserve_buf_space(size_t size)
{
	struct mm_struct *mm;
	unsigned long locked;
	unsigned long buf_mem, buf_mem_max;
	unsigned long flags;

	pfm_spin_lock_irqsave(&pfm_sessions_lock, flags);

	/*
	 * check against global buffer limit
	 */
	buf_mem_max = pfm_controls.smpl_buffer_mem_max;
	buf_mem = pfm_sessions.pfs_smpl_buffer_mem_cur + size;

	if (buf_mem <= buf_mem_max) {
		pfm_sessions.pfs_smpl_buffer_mem_cur = buf_mem;

		PFM_DBG("buf_mem_max=%lu current_buf_mem=%lu",
			buf_mem_max,
			buf_mem);
	}
	pfm_spin_unlock_irqrestore(&pfm_sessions_lock, flags);

	if (buf_mem > buf_mem_max) {
		PFM_DBG("smpl buffer memory threshold reached");
		return -ENOMEM;
	}

	/*
	 * check against RLIMIT_MEMLOCK
	 */
	mm = get_task_mm(current);

	down_write(&mm->mmap_sem);

	locked  = mm->locked_vm << PAGE_SHIFT;
	locked += size;

	if (locked > current->signal->rlim[RLIMIT_MEMLOCK].rlim_cur) {

		PFM_DBG("RLIMIT_MEMLOCK reached ask_locked=%lu rlim_cur=%lu",
			locked,
			current->signal->rlim[RLIMIT_MEMLOCK].rlim_cur);

		up_write(&mm->mmap_sem);
		mmput(mm);
		goto unres;
	}

	mm->locked_vm = locked >> PAGE_SHIFT;

	up_write(&mm->mmap_sem);

	mmput(mm);

	return 0;

unres:
	/*
	 * remove global buffer memory allocation
	 */
	pfm_spin_lock_irqsave(&pfm_sessions_lock, flags);

	pfm_sessions.pfs_smpl_buffer_mem_cur -= size;

	pfm_spin_unlock_irqrestore(&pfm_sessions_lock, flags);

	return -ENOMEM;
}
/*
 *There exist multiple paths leading to this function. We need to
 * be very careful withlokcing on the mmap_sem as it may already be
 * held by the time we come here.
 * The following paths exist:
 *
 * exit path:
 * sys_exit_group
 *    do_group_exit
 *     do_exit
 *      exit_mm
 *       mmput
 *        exit_mmap
 *         remove_vma
 *          fput
 *           __fput
 *            pfm_close
 *             __pfm_close
 *              pfm_context_free
 * 	         pfm_release_buf_space
 * munmap path:
 * sys_munmap
 *  do_munmap
 *   remove_vma
 *    fput
 *     __fput
 *      pfm_close
 *       __pfm_close
 *        pfm_context_free
 *         pfm_release_buf_space
 *
 * close path:
 * sys_close
 *  filp_close
 *   fput
 *    __fput
 *     pfm_close
 *      __pfm_close
 *       pfm_context_free
 *        pfm_release_buf_space
 *
 * The issue is that on the munmap() path, the mmap_sem is already held
 * in write-mode by the time we come here. To avoid the deadlock, we need
 * to know where we are coming from and skip down_write(). If is fairly
 * difficult to know this because of the lack of good hooks and
 * the fact that, there may not have been any mmap() of the sampling buffer
 * (i.e. create_context() followed by close() or exit()).
 *
 * We use a set flag ctx->flags.mmap_nlock which is toggle in the vm_ops
 * callback in remove_vma() which is called systematically for the call, so
 * on all but the pure close() path. The exit path does not already hold
 * the lock but this is exit so there is no task->mm by the time we come here.
 *
 * The mmap_nlock is set only when unmapping and this is the LAST reference
 * to the file (i.e., close() followed by munmap()).
 */
void pfm_release_buf_space(struct pfm_context *ctx, size_t size)
{
	unsigned long flags;
	struct mm_struct *mm;

	mm = get_task_mm(current);
	if (mm) {
		if (ctx->flags.mmap_nlock == 0) {
			PFM_DBG("doing down_write");
			down_write(&mm->mmap_sem);
		}

		mm->locked_vm -= size >> PAGE_SHIFT;

		PFM_DBG("locked_vm=%lu size=%zu", mm->locked_vm, size);

		if (ctx->flags.mmap_nlock == 0)
			up_write(&mm->mmap_sem);

		mmput(mm);
	}

	pfm_spin_lock_irqsave(&pfm_sessions_lock, flags);

	pfm_sessions.pfs_smpl_buffer_mem_cur -= size;

	pfm_spin_unlock_irqrestore(&pfm_sessions_lock, flags);
}

int pfm_reserve_session(int is_system, u32 cpu)
{
	unsigned long flags;
	u32 nsys_cpus;
	int ret = 0;

	/*
	 * validy checks on cpu_mask have been done upstream
	 */
	pfm_spin_lock_irqsave(&pfm_sessions_lock, flags);

	nsys_cpus = cpus_weight(pfm_sessions.pfs_sys_cpumask);

	PFM_DBG("in  sys=%u task=%u is_sys=%d cpu=%u",
		nsys_cpus,
		pfm_sessions.pfs_task_sessions,
		is_system,
		cpu);

	if (is_system) {
		/*
		 * cannot mix system wide and per-task sessions
		 */
		if (pfm_sessions.pfs_task_sessions > 0) {
			PFM_DBG("%u conflicting task_sessions",
				pfm_sessions.pfs_task_sessions);
			ret = -EBUSY;
			goto abort;
		}

		if (cpu_isset(cpu, pfm_sessions.pfs_sys_cpumask)) {
			PFM_DBG("conflicting session on CPU%u", cpu);
			ret = -EBUSY;
			goto abort;
		}

		PFM_DBG("reserved session on CPU%u", cpu);

		cpu_set(cpu, pfm_sessions.pfs_sys_cpumask);
		nsys_cpus++;
	} else {
		if (nsys_cpus) {
			ret = -EBUSY;
			goto abort;
		}
		pfm_sessions.pfs_task_sessions++;
	}

	PFM_DBG("out sys=%u task=%u is_sys=%d cpu=%u",
		nsys_cpus,
		pfm_sessions.pfs_task_sessions,
		is_system,
		cpu);

abort:
	pfm_spin_unlock_irqrestore(&pfm_sessions_lock, flags);

	return ret;
}

/*
 * called from __pfm_unload_context()
 */
int pfm_release_session(int is_system, u32 cpu)
{
	unsigned long flags;

	pfm_spin_lock_irqsave(&pfm_sessions_lock, flags);

	PFM_DBG("in sys_sessions=%u task_sessions=%u syswide=%d cpu=%u",
		cpus_weight(pfm_sessions.pfs_sys_cpumask),
		pfm_sessions.pfs_task_sessions,
		is_system, cpu);

	if (is_system)
		cpu_clear(cpu, pfm_sessions.pfs_sys_cpumask);
	else
		pfm_sessions.pfs_task_sessions--;

	PFM_DBG("out sys_sessions=%u task_sessions=%u syswide=%d cpu=%u",
		cpus_weight(pfm_sessions.pfs_sys_cpumask),
		pfm_sessions.pfs_task_sessions,
		is_system, cpu);

	pfm_spin_unlock_irqrestore(&pfm_sessions_lock, flags);
	return 0;
}

int pfm_reserve_allcpus(void)
{
	unsigned long flags;
	u32 nsys_cpus, cpu;

	pfm_spin_lock_irqsave(&pfm_sessions_lock, flags);

	nsys_cpus = cpus_weight(pfm_sessions.pfs_sys_cpumask);

	PFM_DBG("in  sys=%u task=%u",
		nsys_cpus,
		pfm_sessions.pfs_task_sessions);

	if (nsys_cpus) {
		PFM_DBG("already some system-wide sessions");
		goto abort;
	}

	/*
	 * cannot mix system wide and per-task sessions
	 */
	if (pfm_sessions.pfs_task_sessions) {
		PFM_DBG("%u conflicting task_sessions",
		  	pfm_sessions.pfs_task_sessions);
		goto abort;
	}

	for_each_online_cpu(cpu) {
		cpu_set(cpu, pfm_sessions.pfs_sys_cpumask);
		nsys_cpus++;
	}

	PFM_DBG("out sys=%u task=%u",
		nsys_cpus,
		pfm_sessions.pfs_task_sessions);

	pfm_spin_unlock_irqrestore(&pfm_sessions_lock, flags);

	return 0;

abort:
	pfm_spin_unlock_irqrestore(&pfm_sessions_lock, flags);

	return -EBUSY;
}
EXPORT_SYMBOL(pfm_reserve_allcpus);

int pfm_release_allcpus(void)
{
	unsigned long flags;
	u32 nsys_cpus, cpu;

	pfm_spin_lock_irqsave(&pfm_sessions_lock, flags);

	nsys_cpus = cpus_weight(pfm_sessions.pfs_sys_cpumask);

	PFM_DBG("in  sys=%u task=%u",
		nsys_cpus,
		pfm_sessions.pfs_task_sessions);

	/*
	 * XXX: could use __cpus_clear() with nbits
	 */
	for_each_online_cpu(cpu) {
		cpu_clear(cpu, pfm_sessions.pfs_sys_cpumask);
		nsys_cpus--;
	}

	PFM_DBG("out sys=%u task=%u",
		nsys_cpus,
		pfm_sessions.pfs_task_sessions);

	pfm_spin_unlock_irqrestore(&pfm_sessions_lock, flags);

	return 0;
}
EXPORT_SYMBOL(pfm_release_allcpus);

/*
 * called from perfmon_sysfs.c:
 *  what=0 : pfs_task_sessions
 *  what=1 : cpus_weight(pfs_sys_cpumask)
 *  what=2 : smpl_buffer_mem_cur
 *  what=3 : pmu model name
 *
 * return number of bytes written into buf (up to sz)
 */
ssize_t pfm_sysfs_session_show(char *buf, size_t sz, int what)
{
	unsigned long flags;

	pfm_spin_lock_irqsave(&pfm_sessions_lock, flags);

	switch (what) {
	case 0: snprintf(buf, sz, "%u\n", pfm_sessions.pfs_task_sessions);
		break;
	case 1: snprintf(buf, sz, "%d\n", cpus_weight(pfm_sessions.pfs_sys_cpumask));
		break;
	case 2: snprintf(buf, sz, "%zu\n", pfm_sessions.pfs_smpl_buffer_mem_cur);
		break;
	case 3:
		snprintf(buf, sz, "%s\n",
			pfm_pmu_conf ?	pfm_pmu_conf->pmu_name
				     :	"unknown\n");
	}
	pfm_spin_unlock_irqrestore(&pfm_sessions_lock, flags);
	return strlen(buf);
}
