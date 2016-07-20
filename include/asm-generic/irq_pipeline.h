/*
 * include/asm-generic/irq_pipeline.h
 *
 * Copyright (C) 2002-2016 Philippe Gerum.
 */
#ifndef __ASM_GENERIC_IRQ_PIPELINE_H
#define __ASM_GENERIC_IRQ_PIPELINE_H

#include <linux/kconfig.h>
#include <linux/types.h>

#ifdef CONFIG_IRQ_PIPELINE

#include <asm/bitsperlong.h>

unsigned long root_irq_save(void);
void root_irq_restore(unsigned long x);
void __root_irq_enable(void);
void root_irq_enable(void);
void root_irq_disable(void);
unsigned long root_irqs_disabled(void);

#define hard_cond_local_irq_enable()		hard_local_irq_enable()
#define hard_cond_local_irq_disable()		hard_local_irq_disable()
#define hard_cond_local_irq_save()		hard_local_irq_save()
#define hard_cond_local_irq_restore(__flags)	hard_local_irq_restore(__flags)

void __root_irq_restore_nosync(unsigned long flags);
void root_irq_restore_nosync(unsigned long flags);

#define hard_local_irq_save()			native_irq_save()
#define hard_local_irq_restore(__flags)		native_irq_restore(__flags)
#define hard_local_irq_enable()			native_irq_enable()
#define hard_local_irq_disable()		native_irq_disable()
#define hard_local_save_flags()			native_save_flags()

#define hard_irqs_disabled()			native_irqs_disabled()
#define hard_irqs_disabled_flags(__flags)	native_irqs_disabled_flags(__flags)

void irq_pipeline_nmi_enter(void);
void irq_pipeline_nmi_exit(void);

#else /* !CONFIG_IRQ_PIPELINE */

#define hard_local_save_flags()			({ unsigned long __flags; \
						local_save_flags(__flags); __flags; })
#define hard_local_irq_enable()			local_irq_enable()
#define hard_local_irq_disable()		local_irq_disable()
#define hard_local_irq_save()			({ unsigned long __flags; \
						local_irq_save(__flags); __flags; })
#define hard_local_irq_restore(__flags)		local_irq_restore(__flags)

#define hard_cond_local_irq_enable()		do { } while(0)
#define hard_cond_local_irq_disable()		do { } while(0)
#define hard_cond_local_irq_save()		0
#define hard_cond_local_irq_restore(__flags)	do { (void)(__flags); } while(0)

#define hard_irqs_disabled()			irqs_disabled()
#define hard_irqs_disabled_flags(__flags)	raw_irqs_disabled_flags(__flags)

static inline void irq_pipeline_nmi_enter(void) { }
static inline void irq_pipeline_nmi_exit(void) { }

#endif /* !CONFIG_IRQ_PIPELINE */

#if defined(CONFIG_SMP) && defined(CONFIG_IRQ_PIPELINE)
#define hard_smp_local_irq_save()		hard_local_irq_save()
#define hard_smp_local_irq_restore(__flags)	hard_local_irq_restore(__flags)
#else /* !CONFIG_SMP */
#define hard_smp_local_irq_save()		0
#define hard_smp_local_irq_restore(__flags)	do { (void)(__flags); } while(0)
#endif /* CONFIG_SMP */

#ifdef CONFIG_DEBUG_IRQ_PIPELINE
void check_root_stage(void);
#define check_hard_irqs_disabled()		WARN_ON_ONCE(!hard_irqs_disabled())
#else
static inline void check_root_stage(void) { }
static inline void check_hard_irqs_disabled(void) { }
#endif

static inline bool irqs_pipelined(void)
{
	return IS_ENABLED(CONFIG_IRQ_PIPELINE);
}

static inline bool irq_pipeline_debug(void)
{
	return IS_ENABLED(CONFIG_DEBUG_IRQ_PIPELINE);
}

#endif /* __ASM_GENERIC_IRQ_PIPELINE_H */
