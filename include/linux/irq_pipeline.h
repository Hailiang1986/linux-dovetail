/*
 * include/linux/irq_pipeline.h
 *
 * Copyright (C) 2002-2017 Philippe Gerum
 *               2006-2012 Gilles Chanteperdrix
 *               2007 Jan Kiszka.
 */
#ifndef _LINUX_IRQ_PIPELINE_H
#define _LINUX_IRQ_PIPELINE_H

struct cpuidle_device;
struct cpuidle_state;

#ifdef CONFIG_IRQ_PIPELINE

#include <linux/compiler.h>
#include <linux/irqdomain.h>
#include <linux/percpu.h>
#include <linux/interrupt.h>
#include <linux/irqstage.h>
#include <linux/thread_info.h>
#include <linux/irqreturn.h>
#include <linux/stop_machine.h>
#include <asm/irqflags.h>

void irq_pipeline_init_early(void);

void irq_pipeline_init(void);

void irq_pipeline_init_late(void);

void arch_irq_pipeline_init(void);

void __irq_pipeline_enter(unsigned int irq, struct pt_regs *regs);

void irq_pipeline_enter(unsigned int irq, struct pt_regs *regs);

void irq_pipeline_enter_nosync(unsigned int irq);

unsigned long irq_pipeline_lock_many(const struct cpumask *mask,
				     cpu_stop_fn_t fn,
				     void *data);
static inline
unsigned long irq_pipeline_lock(cpu_stop_fn_t fn,
				void *data)
{
	return irq_pipeline_lock_many(cpu_online_mask, fn, data);
}

void irq_pipeline_unlock(unsigned long flags);

void irq_pipeline_inject(unsigned int irq);

void irq_pipeline_clear(unsigned int irq);

void do_IRQ_pipelined(struct irq_desc *desc);

void irq_push_stage(struct irq_stage *stage,
		    const char *name);

void irq_pop_stage(struct irq_stage *stage);

void arch_irq_push_stage(struct irq_stage *stage);

#ifdef CONFIG_SMP
void irq_pipeline_send_remote(unsigned int ipi,
			      const struct cpumask *cpumask);
#endif	/* CONFIG_SMP */

void irq_pipeline_oops(void);

bool irq_pipeline_steal_tick(void);

bool __irq_cpuidle_enter(void);

bool irq_cpuidle_enter(struct cpuidle_device *dev,
		       struct cpuidle_state *state);

void irq_cpuidle_exit(void);

extern bool irq_pipeline_active;
	
static inline bool irq_critical_context(void)
{
	return on_head_stage() ||
		(hard_irqs_disabled() && irq_pipeline_active);
}

extern struct irq_domain *synthetic_irq_domain;

#else /* !CONFIG_IRQ_PIPELINE */

static inline
void irq_pipeline_init_early(void) { }

static inline
void irq_pipeline_init(void) { }

static inline
void irq_pipeline_init_late(void) { }

static inline
void irq_pipeline_enter(unsigned int irq, struct pt_regs *regs) { }

static inline
void irq_pipeline_enter_nosync(unsigned int irq) { }

static inline
void irq_pipeline_clear(unsigned int irq) { }

static inline
void irq_pipeline_oops(void) { }

static inline bool __irq_cpuidle_enter(void)
{
	return true;
}

static inline bool irq_cpuidle_enter(struct cpuidle_device *dev,
				     struct cpuidle_state *state)
{
	return __irq_cpuidle_enter();
}

static inline void irq_cpuidle_exit(void)
{ }

static inline bool irq_critical_context(void)
{
	return false;
}

#endif /* !CONFIG_IRQ_PIPELINE */

#endif /* _LINUX_IRQ_PIPELINE_H */
