/*
 * SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (C) 2002 Philippe Gerum  <rpm@xenomai.org>.
 *               2006 Gilles Chanteperdrix.
 *               2007 Jan Kiszka.
 */
#ifndef _LINUX_IRQ_PIPELINE_H
#define _LINUX_IRQ_PIPELINE_H

struct cpuidle_device;
struct cpuidle_state;
struct irq_desc;

#ifdef CONFIG_IRQ_PIPELINE

#include <linux/compiler.h>
#include <linux/irqdomain.h>
#include <linux/percpu.h>
#include <linux/interrupt.h>
#include <linux/irqstage.h>
#include <linux/thread_info.h>
#include <asm/irqflags.h>

void irq_pipeline_init_early(void);

void irq_pipeline_init(void);

void arch_irq_pipeline_init(void);

int irq_inject_pipeline(unsigned int irq);

int generic_pipeline_irq(unsigned int irq,
			 struct pt_regs *regs);

void synchronize_pipeline(void);

static __always_inline void synchronize_pipeline_on_irq(void)
{
	/*
	 * Optimize if we preempted the high priority oob stage: we
	 * don't need to synchronize the pipeline unless there is a
	 * pending interrupt for it.
	 */
	if (running_inband() ||
	    stage_irqs_pending(this_oob_staged()))
		synchronize_pipeline();
}

void enter_oob_irq(void);

static __always_inline void enter_irq_pipeline(struct pt_regs *regs)
{
	enter_oob_irq();
}

void exit_oob_irq(void);

void dovetail_call_mayday(struct thread_info *ti,
			  struct pt_regs *regs);

static __always_inline bool leave_irq_pipeline(struct pt_regs *regs)
{
	exit_oob_irq();

	/*
	 * We have to synchronize the logs because interrupts might
	 * have been logged while we were busy handling an OOB event
	 * coming from the hardware:
	 *
	 * - as a result of calling an OOB handler which in turned
	 * posted them.
	 *
	 * - because we posted them directly for scheduling the
	 * interrupt to happen from the inband stage.
	 *
	 * This also means that hardware-originated OOB events have
	 * higher precedence when received than software-originated
	 * ones, which are synced once all IRQ flow handlers involved
	 * in the interrupt have run.
	 */
	synchronize_pipeline_on_irq();

#ifdef CONFIG_DOVETAIL
	/*
	 * Sending MAYDAY is in essence a rare case, so prefer test
	 * then maybe clear over test_and_clear.
	 */
	if (user_mode(regs) && test_thread_flag(TIF_MAYDAY))
		dovetail_call_mayday(current_thread_info(), regs);
#endif

	return running_inband() && !irqs_disabled();
}

bool handle_oob_irq(struct irq_desc *desc);

void arch_do_IRQ_pipelined(struct irq_desc *desc);

void irq_pipeline_clear(struct irq_desc *desc);

#ifdef CONFIG_SMP
void irq_pipeline_send_remote(unsigned int ipi,
			      const struct cpumask *cpumask);
#endif	/* CONFIG_SMP */

void irq_pipeline_oops(void);

bool irq_pipeline_steal_tick(void);

bool irq_cpuidle_enter(struct cpuidle_device *dev,
		       struct cpuidle_state *state);

int run_oob_call(int (*fn)(void *arg), void *arg);

extern bool irq_pipeline_active;

static inline bool inband_unsafe(void)
{
	return running_oob() ||
		(hard_irqs_disabled() && irq_pipeline_active);
}

static inline bool inband_irq_pending(void)
{
	check_hard_irqs_disabled();

	return stage_irqs_pending(this_inband_staged());
}

extern struct irq_domain *synthetic_irq_domain;

#else /* !CONFIG_IRQ_PIPELINE */

#include <linux/irqstage.h>
#include <asm/irq_pipeline.h>

static inline
void irq_pipeline_init_early(void) { }

static inline
void irq_pipeline_init(void) { }

static inline
void irq_pipeline_clear(struct irq_desc *desc) { }

static inline
void irq_pipeline_oops(void) { }

static inline
int generic_pipeline_irq(unsigned int irq, struct pt_regs *regs)
{
	return 0;
}

static inline bool handle_oob_irq(struct irq_desc *desc)
{
	return false;
}

static inline bool irq_cpuidle_enter(struct cpuidle_device *dev,
				     struct cpuidle_state *state)
{
	return true;
}

static inline bool inband_unsafe(void)
{
	return false;
}

static inline bool inband_irq_pending(void)
{
	return false;
}

#endif /* !CONFIG_IRQ_PIPELINE */

#endif /* _LINUX_IRQ_PIPELINE_H */
