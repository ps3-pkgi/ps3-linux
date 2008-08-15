/*
 *  Copyright 2007 Sony Corp.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef _ASM_POWERPC_EMULATED_OPS_H
#define _ASM_POWERPC_EMULATED_OPS_H

#ifdef CONFIG_SYSCTL

#ifdef CONFIG_PPC64

#include <linux/percpu.h>

#include <asm/atomic.h>

DECLARE_PER_CPU(atomic_long_t, emulated_dcba);
DECLARE_PER_CPU(atomic_long_t, emulated_dcbz);
DECLARE_PER_CPU(atomic_long_t, emulated_fp_pair);
DECLARE_PER_CPU(atomic_long_t, emulated_mcrxr);
DECLARE_PER_CPU(atomic_long_t, emulated_mfpvr);
DECLARE_PER_CPU(atomic_long_t, emulated_multiple);
DECLARE_PER_CPU(atomic_long_t, emulated_popcntb);
DECLARE_PER_CPU(atomic_long_t, emulated_spe);
DECLARE_PER_CPU(atomic_long_t, emulated_string);
#ifdef CONFIG_MATH_EMULATION
DECLARE_PER_CPU(atomic_long_t, emulated_math);
#elif defined(CONFIG_8XX_MINIMAL_FPEMU)
DECLARE_PER_CPU(atomic_long_t, emulated_8xx);
#endif

#define WARN_EMULATE_INC(type)						\
	do {								\
		atomic_long_inc(&per_cpu(emulated_ ## type,		\
					 raw_smp_processor_id()));	\
	} while (0)

#else /* !CONFIG_PPC64 */

#define WARN_EMULATE_INC(type)	do { } while (0)

#endif /* !CONFIG_PPC64 */

extern int sysctl_warn_emulated;
extern void do_warn_emulate(const char *type);

#define WARN_EMULATE(type)						\
	do {								\
		WARN_EMULATE_INC(type);					\
		if (sysctl_warn_emulated)				\
			do_warn_emulate(#type);				\
	} while (0)

#else /* !CONFIG_SYSCTL */

#define WARN_EMULATE(type)	do { } while (0)

#endif /* !CONFIG_SYSCTL */

#endif /* _ASM_POWERPC_EMULATED_OPS_H */
