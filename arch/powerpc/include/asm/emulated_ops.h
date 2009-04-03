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

#include <linux/percpu.h>

#include <asm/atomic.h>

#ifdef CONFIG_ALTIVEC
DECLARE_PER_CPU(atomic_long_t, emulated_altivec);
#endif
DECLARE_PER_CPU(atomic_long_t, emulated_dcba);
DECLARE_PER_CPU(atomic_long_t, emulated_dcbz);
DECLARE_PER_CPU(atomic_long_t, emulated_fp_pair);
DECLARE_PER_CPU(atomic_long_t, emulated_isel);
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
#ifdef CONFIG_VSX
DECLARE_PER_CPU(atomic_long_t, emulated_vsx);
#endif

extern void warn_emulated_print(const char *type);

#ifdef CONFIG_SYSCTL
extern int sysctl_warn_emulated;
#else
#define sysctl_warn_emulated		0
#endif

#define WARN_EMULATED(type)						\
	do {								\
		atomic_long_inc(&per_cpu(emulated_ ## type,		\
					 raw_smp_processor_id()));	\
		if (sysctl_warn_emulated)				\
			warn_emulated_print(#type);			\
	} while (0)

#endif /* _ASM_POWERPC_EMULATED_OPS_H */
