/*
 * Copyright (c) 2001-2006 Hewlett-Packard Development Company, L.P.
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

#ifndef __LINUX_PERFMON_KERN_H__
#define __LINUX_PERFMON_KERN_H__

#ifdef __KERNEL__

#ifdef CONFIG_PERFMON

#include <linux/file.h>
#include <linux/seq_file.h>
#include <linux/interrupt.h>
#include <linux/kobject.h>
#include <linux/perfmon.h>

/*
 * system adminstrator configuration controls available via
 * the /sys/kerne/perfmon interface
 */
struct pfm_controls {
	int	debug;		/* debugging via syslog */
	int	debug_ovfl;	/* overflow handling debugging */
	gid_t	sys_group;	/* gid to create a syswide context */
	gid_t	task_group;	/* gid to create a per-task context */
	size_t	arg_mem_max;	/* maximum vector argument size */
	size_t	smpl_buffer_mem_max; /* max buf mem, -1 for infinity */
};

/*
 * software PMD 
 */
struct pfm_pmd {
	u64 value;			/* 64-bit value */
	u64 lval;			/* last reset value */
	u64 ovflsw_thres;		/* #ovfls left before switch */
	u64 long_reset;			/* long reset value on overflow */
	u64 short_reset;		/* short reset value on overflow */
	u64 reset_pmds[PFM_PMD_BV];	/* pmds to reset on overflow */
	u64 smpl_pmds[PFM_PMD_BV];	/* pmds to record on overflow */
	u64 mask;			/* range mask for random value */
	u64 ovflsw_ref_thres;		/* #ovfls before next set */
	u64 eventid;			/* opaque event identifier */
	u32 flags;			/* notify/do not notify */
};

/*
 * event_set: encapsulates the full PMU state
 */
struct pfm_event_set {
	struct list_head list;		/* ordered chain of sets */
	u16 id;				/* set identification */
	u16 nused_pmds;			/* max number of used PMDs */
	u16 nused_pmcs;			/* max number of used PMCs */
	u16 pad1;			/* paddding */
	u32 flags;			/* public flags */
	u32 priv_flags;			/* private flags (see below) */
	u64 runs;			/* # of activations */
	u32 npend_ovfls;		/* number of pending PMD overflow */
	u32 pad2;			/* padding */
	u64 used_pmds[PFM_PMD_BV];	/* used PMDs */
	u64 povfl_pmds[PFM_PMD_BV];	/* pending overflowed PMDs */
	u64 ovfl_pmds[PFM_PMD_BV];	/* last overflowed PMDs */
	u64 reset_pmds[PFM_PMD_BV];	/* union of PMDs to reset */
	u64 ovfl_notify[PFM_PMD_BV];	/* notify on overflow */
	u64 used_pmcs[PFM_PMC_BV];	/* used PMCs */
	u64 pmcs[PFM_MAX_PMCS];		/* PMC values */

	struct pfm_pmd pmds[PFM_MAX_PMDS];

	ktime_t hrtimer_exp;		/* switch timeout reference */
	ktime_t hrtimer_rem;		/* per-thread remainder timeout */

	u64 duration_start;		/* start time in ns */
	u64 duration;			/* total active ns */
};

/*
 * common private event set flags (priv_flags)
 *
 * upper 16 bits: for arch-specific use
 * lower 16 bits: for common use
 */
#define PFM_SETFL_PRIV_MOD_PMDS 0x1 /* PMD register(s) modified */
#define PFM_SETFL_PRIV_MOD_PMCS 0x2 /* PMC register(s) modified */
#define PFM_SETFL_PRIV_SWITCH	0x4 /* must switch set on restart */
#define PFM_SETFL_PRIV_MOD_BOTH	(PFM_SETFL_PRIV_MOD_PMDS | PFM_SETFL_PRIV_MOD_PMCS)

/*
 * context flags
 */
struct pfm_context_flags {
	unsigned int block:1;		/* task blocks on user notifications */
	unsigned int system:1;		/* do system wide monitoring */
	unsigned int no_msg:1;		/* no message sent on overflow */
	unsigned int switch_ovfl:1;	/* switch set on counter ovfl */
	unsigned int switch_time:1;	/* switch set on timeout */
	unsigned int started:1;		/* pfm_start() issued */
	unsigned int work_type:2;	/* type of work for pfm_handle_work */
	unsigned int mmap_nlock:1;	/* no lock in pfm_release_buf_space */
	unsigned int ia64_v20_compat:1;	/* context is IA-64 v2.0 mode */
	unsigned int can_restart:8;	/* allowed to issue a PFM_RESTART */
	unsigned int reset_count:8;	/* number of pending resets */
	unsigned int is_self:1;		/* per-thread and self-montoring */
	unsigned int cell_spe_follow:1;	/* cell spe ctx follow */
	unsigned int reserved:4;	/* for future use */
};

/*
 * values for work_type (TIF_PERFMON_WORK must be set)
 */
#define PFM_WORK_NONE	0	/* nothing to do */
#define PFM_WORK_RESET	1	/* reset overflowed counters */
#define PFM_WORK_BLOCK	2	/* block current thread */
#define PFM_WORK_ZOMBIE	3	/* cleanup zombie context */

/*
 * overflow description argument passed to sampling format
 */
struct pfm_ovfl_arg {
	u16 ovfl_pmd;		/* index of overflowed PMD  */
	u16 active_set;		/* set active at the time of the overflow */
	u32 ovfl_ctrl;		/* control flags */
	u64 pmd_last_reset;	/* last reset value of overflowed PMD */
	u64 smpl_pmds_values[PFM_MAX_PMDS]; /* values of other PMDs */
	u64 pmd_eventid;	/* eventid associated with PMD */
	u16 num_smpl_pmds;	/* number of PMDS in smpl_pmd_values */
};
/*
 * depth of message queue
 *
 * Depth cannot be bigger than 255 (see reset_count)
 */
#define PFM_MSGS_ORDER		3 /* log2(number of messages) */
#define PFM_MSGS_COUNT		(1<<PFM_MSGS_ORDER) /* number of messages */
#define PFM_MSGQ_MASK		(PFM_MSGS_COUNT-1)

/*
 * perfmon context state
 */
#define PFM_CTX_UNLOADED	1 /* context is not loaded onto any task */
#define PFM_CTX_LOADED		2 /* context is loaded onto a task */
#define PFM_CTX_MASKED		3 /* context is loaded, monitoring is masked */
#define PFM_CTX_ZOMBIE		4 /* context lost owner but is still attached */


/*
 * context: contains all the state of a session
 */
struct pfm_context {
	spinlock_t		lock;		/* context protection */

	struct pfm_context_flags flags;
	u32			state;		/* current state */
	struct task_struct 	*task;		/* attached task */

	struct completion       restart_complete;/* block on notification */
	u64 			last_act;	/* last activation */
	u32			last_cpu;   	/* last CPU used (SMP only) */
	u32			cpu;		/* cpu bound to context */

	struct pfm_smpl_fmt	*smpl_fmt;	/* sampling format callbacks */
	void			*smpl_addr;	/* user smpl buffer base */
	size_t			smpl_size;	/* user smpl buffer size */
	void			*smpl_real_addr;/* actual smpl buffer base */
	size_t			smpl_real_size; /* actual smpl buffer size */

	wait_queue_head_t 	msgq_wait;	/* pfm_read() wait queue */

	union pfarg_msg		msgq[PFM_MSGS_COUNT];
	int			msgq_head;
	int			msgq_tail;

	struct fasync_struct	*async_queue;	/* async notification */

	struct pfm_event_set	*active_set;	/* active set */
	struct list_head	set_list;	/* ordered list of sets */

	/*
	 * save stack space by allocating temporary variables for
	 * pfm_overflow_handler() in pfm_context
	 */
	struct pfm_ovfl_arg 	ovfl_arg;
	u64			ovfl_ovfl_notify[PFM_PMD_BV];
};

/*
 * type of PMD reset for pfm_reset_pmds() or pfm_switch_sets*()
 */
#define PFM_PMD_RESET_SHORT	1	/* use short reset value */
#define PFM_PMD_RESET_LONG	2	/* use long reset value  */

/*
 * check_mask bitmask values for pfm_check_task_state()
 */
#define PFM_CMD_STOPPED		0x01	/* command needs thread stopped */
#define PFM_CMD_UNLOADED	0x02	/* command needs ctx unloaded */
#define PFM_CMD_UNLOAD		0x04	/* command is unload */

DECLARE_PER_CPU(struct task_struct *, pmu_owner);
DECLARE_PER_CPU(struct pfm_context *, pmu_ctx);
DECLARE_PER_CPU(u64, pmu_activation_number);
DECLARE_PER_CPU(struct pfm_stats, pfm_stats);
DECLARE_PER_CPU(struct hrtimer, pfm_hrtimer);

/*
 * logging
 */
#define PFM_ERR(f, x...)  printk(KERN_ERR     "perfmon: " f "\n", ## x)
#define PFM_WARN(f, x...) printk(KERN_WARNING "perfmon: " f "\n", ## x)
#define PFM_LOG(f, x...)  printk(KERN_NOTICE  "perfmon: " f "\n", ## x)
#define PFM_INFO(f, x...) printk(KERN_INFO    "perfmon: " f "\n", ## x)

/*
 * debugging
 *
 * Printk rate limiting is enforced to avoid getting flooded with too many
 * error messages on the console (which could render the machine unresponsive).
 * To get full debug output (turn off ratelimit):
 * 	$ echo 0 >/proc/sys/kernel/printk_ratelimit
 */
#ifdef CONFIG_PERFMON_DEBUG
#define PFM_DBG(f, x...) \
	do { \
		if (unlikely(pfm_controls.debug >0 && printk_ratelimit())) { \
			printk("perfmon: %s.%d: CPU%d [%d]: " f "\n", \
			       __FUNCTION__, __LINE__, \
			       smp_processor_id(), current->pid , ## x); \
		} \
	} while (0)

#define PFM_DBG_ovfl(f, x...) \
	do { \
		if (unlikely(pfm_controls.debug_ovfl >0 && printk_ratelimit())) { \
			printk("perfmon: %s.%d: CPU%d [%d]: " f "\n", \
			       __FUNCTION__, __LINE__, \
			       smp_processor_id(), current->pid , ## x); \
		} \
	} while (0)
#else
#define PFM_DBG(f, x...)	do {} while(0)
#define PFM_DBG_ovfl(f, x...)	do {} while(0)
#endif

extern struct pfm_pmu_config  *pfm_pmu_conf;
extern struct pfm_controls pfm_controls;
extern int perfmon_disabled;

static inline struct pfm_arch_context *pfm_ctx_arch(struct pfm_context *c)
{
	return (struct pfm_arch_context *)(c+1);
}

static inline void pfm_set_pmu_owner(struct task_struct *task,
				     struct pfm_context *ctx)
{
	__get_cpu_var(pmu_owner) = task;
	__get_cpu_var(pmu_ctx) = ctx;
}

static inline int pfm_msgq_is_empty(struct pfm_context *ctx)
{
	return ctx->msgq_head == ctx->msgq_tail;
}

void pfm_get_next_msg(struct pfm_context *ctx, union pfarg_msg *m);
int pfm_end_notify(struct pfm_context *ctx);
int pfm_ovfl_notify(struct pfm_context *ctx, struct pfm_event_set *set,
		    unsigned long ip);

int  pfm_get_args(void __user *ureq, size_t sz, size_t lsz, void *laddr,
		  void **req, void **to_free);

int pfm_get_task(struct pfm_context *ctx, pid_t pid, struct task_struct **task);
int pfm_get_smpl_arg(char __user *fmt_uname, void __user *uaddr, size_t usize, void **arg,
		     struct pfm_smpl_fmt **fmt);

int pfm_alloc_fd(struct file **cfile);

int __pfm_write_pmcs(struct pfm_context *ctx, struct pfarg_pmc *req, int count);
int __pfm_write_pmds(struct pfm_context *ctx, struct pfarg_pmd *req, int count,
		     int compat);
int __pfm_read_pmds(struct pfm_context *ctx, struct pfarg_pmd *req, int count);
int __pfm_load_context(struct pfm_context *ctx, struct pfarg_load *req,
		       struct task_struct *task);
int __pfm_unload_context(struct pfm_context *ctx, int *can_release);
int __pfm_stop(struct pfm_context *ctx, int *release_info);
int  __pfm_restart(struct pfm_context *ctx, int *unblock);
int __pfm_start(struct pfm_context *ctx, struct pfarg_start *start);
int __pfm_delete_evtsets(struct pfm_context *ctx, void *arg, int count);
int __pfm_getinfo_evtsets(struct pfm_context *ctx, struct pfarg_setinfo *req,
			  int count);
int __pfm_create_evtsets(struct pfm_context *ctx, struct pfarg_setdesc *req,
			int count);

int __pfm_create_context(struct pfarg_ctx *req,
			 struct pfm_smpl_fmt *fmt,
			 void *fmt_arg,
			 int mode,
			 struct pfm_context **new_ctx);

int pfm_check_task_state(struct pfm_context *ctx, int check_mask,
			 unsigned long *flags);

struct pfm_event_set *pfm_find_set(struct pfm_context *ctx, u16 set_id,
				   int alloc);

struct pfm_context *pfm_get_ctx(int fd);

void pfm_context_free(struct pfm_context *ctx);
struct pfm_context *pfm_context_alloc(void);
int pfm_pmu_conf_get(int autoload);
void pfm_pmu_conf_put(void);

int pfm_pmu_acquire(void);
void pfm_pmu_release(void);

int pfm_session_acquire(int is_system, u32 cpu);
void pfm_session_release(int is_system, u32 cpu);
int pfm_session_allcpus_acquire(void);
void pfm_session_allcpus_release(void);

int pfm_smpl_buffer_alloc(struct pfm_context *ctx, size_t rsize);
int pfm_smpl_buf_space_acquire(struct pfm_context *ctx, size_t size);
void pfm_smpl_buf_space_release(struct pfm_context *ctx, size_t size);

struct pfm_smpl_fmt *pfm_smpl_fmt_get(char *name);
void pfm_smpl_fmt_put(struct pfm_smpl_fmt *fmt);

int  pfm_init_sysfs(void);
int  pfm_init_debugfs(void);
ssize_t pfm_sysfs_res_show(char *buf, size_t sz, int what);

int pfm_debugfs_add_cpu(int mycpu);
void pfm_debugfs_del_cpu(int mycpu);

void pfm_interrupt_handler(unsigned long iip, struct pt_regs *regs);
void pfm_save_prev_context(struct pfm_context *ctxp);

void pfm_reset_pmds(struct pfm_context *ctx, struct pfm_event_set *set,
		    int num_pmds,
		    int reset_mode);

int pfm_prepare_sets(struct pfm_context *ctx, struct pfm_event_set *act_set);
int pfm_init_sets(void);

void pfm_free_sets(struct pfm_context *ctx);
void pfm_init_evtset(struct pfm_event_set *set);
void pfm_switch_sets_from_intr(struct pfm_context *ctx);
enum hrtimer_restart pfm_handle_switch_timeout(struct hrtimer *t);

enum hrtimer_restart pfm_switch_sets(struct pfm_context *ctx,
		    struct pfm_event_set *new_set,
		    int reset_mode,
		    int no_restart);

void pfm_save_pmds(struct pfm_context *ctx, struct pfm_event_set *set);

int pfm_init_fs(void);

int pfm_init_hotplug(void);
void pfm_cpu_disable(void);

/*
 * Allow arches to override the implementation for pfm_spin_lock_irqsave
 * and pfm_spin_unlock_irqrestore.
 */
#define pfm_spin_lock_irqsave(l, f) spin_lock_irqsave(l, f)
#define pfm_spin_unlock_irqrestore(l, f) spin_unlock_irqrestore(l, f)

#include <linux/perfmon_pmu.h>
#include <linux/perfmon_fmt.h>

extern const struct file_operations pfm_file_ops;
/*
 * upper limit for count in calls that take vector arguments. This is used
 * to prevent for multiplication overflow when we compute actual storage size
 */
#define PFM_MAX_ARG_COUNT(m) (INT_MAX/sizeof(*(m)))

#define cast_ulp(_x) ((unsigned long *)_x)

#define PFM_NORMAL      0
#define PFM_COMPAT      1

void __pfm_exit_thread(struct task_struct *task);
void __pfm_copy_thread(struct task_struct *task);
void pfm_ctxsw(struct task_struct *prev, struct task_struct *next);
void pfm_handle_work(struct pt_regs *regs);
void __pfm_init_percpu (void *dummy);

static inline void pfm_exit_thread(struct task_struct *task)
{
	if (task->pfm_context)
		__pfm_exit_thread(task);
}

static inline void pfm_copy_thread(struct task_struct *task)
{
	/*
	 * context or perfmon TIF state  is NEVER inherited
	 * in child task. Holds for per-thread and system-wide
	 */
	task->pfm_context = NULL;
	clear_tsk_thread_flag(task, TIF_PERFMON_CTXSW);
	clear_tsk_thread_flag(task, TIF_PERFMON_WORK);
}

static inline void pfm_init_percpu(void)
{
	__pfm_init_percpu(NULL);
}

/*
 * pfm statistics are available via debugfs
 * and perfmon subdir.
 *
 * When adding new stats, make sure you also
 * update the name table in perfmon_debugfs.c
 */
enum pfm_stats_names {
	PFM_ST_ovfl_intr_all_count = 0,
	PFM_ST_ovfl_intr_ns,
	PFM_ST_ovfl_intr_p1_ns,
	PFM_ST_ovfl_intr_p2_ns,
	PFM_ST_ovfl_intr_p3_ns,
	PFM_ST_ovfl_intr_spurious_count,
	PFM_ST_ovfl_intr_replay_count,
	PFM_ST_ovfl_intr_regular_count,
	PFM_ST_handle_work_count,
	PFM_ST_ovfl_notify_count,
	PFM_ST_reset_pmds_count,
	PFM_ST_pfm_restart_count,
	PFM_ST_fmt_handler_calls,
	PFM_ST_fmt_handler_ns,
	PFM_ST_set_switch_count,
	PFM_ST_set_switch_ns,
	PFM_ST_set_switch_exp,
	PFM_ST_ctxsw_count,
	PFM_ST_ctxsw_ns,
	PFM_ST_handle_timeout_count,
	PFM_ST_ovfl_intr_nmi_count,
	PFM_ST_LAST	/* last entry marked */
};
#define PFM_NUM_STATS PFM_ST_LAST

struct pfm_stats {
	u64 v[PFM_NUM_STATS];
	struct dentry *dirs[PFM_NUM_STATS];
	struct dentry *cpu_dir;
	char cpu_name[8];
};

#define pfm_stats_get(x)  __get_cpu_var(pfm_stats).v[PFM_ST_##x]
#define pfm_stats_inc(x)  __get_cpu_var(pfm_stats).v[PFM_ST_##x]++
#define pfm_stats_add(x,y)  __get_cpu_var(pfm_stats).v[PFM_ST_##x] += (y)

/*
 * include arch-specific kernel level definitions
 */
#include <asm/perfmon_kern.h>

/*
 * read a single PMD register.
 *
 * virtual PMD registers have special handler.
 * Depends on definitions in asm/perfmon_kern.h
 */
static inline u64 pfm_read_pmd(struct pfm_context *ctx, unsigned int cnum)
{
	if (unlikely(pfm_pmu_conf->pmd_desc[cnum].type & PFM_REG_V))
		return pfm_pmu_conf->pmd_sread(ctx, cnum);

	return pfm_arch_read_pmd(ctx, cnum);
}
/*
 * write a single PMD register.
 *
 * virtual PMD registers have special handler.
 * Depends on definitions in asm/perfmon_kern.h
 */
static inline void pfm_write_pmd(struct pfm_context *ctx, unsigned int cnum, u64 value)
{
	/*
	 * PMD writes are ignored for read-only registers
	 */
	if (pfm_pmu_conf->pmd_desc[cnum].type & PFM_REG_RO)
		return;

	if (pfm_pmu_conf->pmd_desc[cnum].type & PFM_REG_V) {
		pfm_pmu_conf->pmd_swrite(ctx, cnum, value);
		return;
	}
	/*
	 * clear unimplemented bits
	 */
	value &= ~pfm_pmu_conf->pmd_desc[cnum].rsvd_msk;

	pfm_arch_write_pmd(ctx, cnum, value);
}

/*
 * max vector argument elements for local storage (no kmalloc/kfree)
 * The PFM_ARCH_PM*_ARG should be defined in perfmon_kern.h.
 * If not, default (conservative) values are used
 */
#ifndef PFM_ARCH_PMC_STK_ARG
#define PFM_ARCH_PMC_STK_ARG	1
#endif

#ifndef PFM_ARCH_PMD_STK_ARG
#define PFM_ARCH_PMD_STK_ARG	1
#endif

#define PFM_PMC_STK_ARG	PFM_ARCH_PMC_STK_ARG
#define PFM_PMD_STK_ARG	PFM_ARCH_PMD_STK_ARG


#else /* !CONFIG_PERFMON */


/*
 * perfmon hooks are nops when CONFIG_PERFMON is undefined
 */
#define tsks_have_perfmon(p, n)	(0)
#define pfm_cpu_disable()		do { } while (0)
#define pfm_init_percpu()		do { } while (0)
#define pfm_exit_thread(_t)  		do { } while (0)
#define pfm_handle_work(_t)    		do { } while (0)
#define pfm_copy_thread(_t)		do { } while (0)
#define pfm_ctxsw(_p, _t)     		do { } while (0)
#define	pfm_release_allcpus()		do { } while (0)
#define	pfm_reserve_allcpus()		(0)
/*
 * include arch-specific kernel level definitions
 */
#include <asm/perfmon_kern.h>

#endif /* CONFIG_PERFMON */

#endif /* __KERNEL__*/
#endif /* __LINUX_PERFMON_KERN_H__ */
