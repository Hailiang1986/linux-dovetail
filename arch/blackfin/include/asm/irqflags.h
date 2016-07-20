/*
 * interface to Blackfin CEC
 *
 * Copyright 2009 Analog Devices Inc.
 * Licensed under the GPL-2 or later.
 */

#ifndef __ASM_BFIN_IRQFLAGS_H__
#define __ASM_BFIN_IRQFLAGS_H__

#include <mach/blackfin.h>

#ifdef CONFIG_SMP
# include <asm/pda.h>
# include <asm/processor.h>
# define bfin_irq_flags cpu_pda[blackfin_core_id()].imask
#else
extern unsigned long bfin_irq_flags;
#endif

static inline notrace void bfin_sti(unsigned long flags)
{
	asm volatile("sti %0;" : : "d" (flags));
}

static inline notrace unsigned long bfin_cli(void)
{
	unsigned long flags;
	asm volatile("cli %0;" : "=d" (flags));
	return flags;
}

#ifdef CONFIG_DEBUG_HWERR
# define bfin_no_irqs 0x3f
#else
# define bfin_no_irqs 0x1f
#endif

/*****************************************************************************/
/*
 * Hard, untraced CPU interrupt flag manipulation and access.
 */
static inline notrace void __hard_local_irq_disable(void)
{
	bfin_cli();
}

static inline notrace void __hard_local_irq_enable(void)
{
	bfin_sti(bfin_irq_flags);
}

static inline notrace unsigned long hard_local_save_flags(void)
{
	return bfin_read_IMASK();
}

static inline notrace unsigned long __hard_local_irq_save(void)
{
	unsigned long flags;
	flags = bfin_cli();
#ifdef CONFIG_DEBUG_HWERR
	bfin_sti(0x3f);
#endif
	return flags;
}

static inline notrace int hard_irqs_disabled_flags(unsigned long flags)
{
#ifdef CONFIG_BF60x
	return (flags & IMASK_IVG11) == 0;
#else
	return (flags & ~0x3f) == 0;
#endif
}

static inline notrace int hard_irqs_disabled(void)
{
	unsigned long flags = hard_local_save_flags();
	return hard_irqs_disabled_flags(flags);
}

static inline notrace void __hard_local_irq_restore(unsigned long flags)
{
	if (!hard_irqs_disabled_flags(flags))
		__hard_local_irq_enable();
}

/*****************************************************************************/
/*
 * Interrupt pipe handling.
 */
/*
 * Direct interface to linux/irqflags.h.
 */
#define arch_local_save_flags()		hard_local_save_flags()
#define arch_local_irq_save()		__hard_local_irq_save()
#define arch_local_irq_restore(flags)	__hard_local_irq_restore(flags)
#define arch_local_irq_enable()		__hard_local_irq_enable()
#define arch_local_irq_disable()	__hard_local_irq_disable()
#define arch_irqs_disabled_flags(flags)	hard_irqs_disabled_flags(flags)
#define arch_irqs_disabled()		hard_irqs_disabled()

/*
 * Interface to various arch routines that may be traced.
 */
#define hard_local_irq_save()		__hard_local_irq_save()
#define hard_local_irq_restore(flags)	__hard_local_irq_restore(flags)
#define hard_local_irq_enable()		__hard_local_irq_enable()
#define hard_local_irq_disable()	__hard_local_irq_disable()
#define hard_local_irq_save_cond()		hard_local_save_flags()
#define hard_local_irq_restore_cond(flags)	do { (void)(flags); } while (0)

#ifdef CONFIG_SMP
#define hard_local_irq_save_smp()		hard_local_irq_save()
#define hard_local_irq_restore_smp(flags)	hard_local_irq_restore(flags)
#else
#define hard_local_irq_save_smp()		hard_local_save_flags()
#define hard_local_irq_restore_smp(flags)	do { (void)(flags); } while (0)
#endif

#endif
