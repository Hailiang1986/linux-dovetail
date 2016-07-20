/*
 *   include/linux/irqstage.h
 *
 *   Copyright (C) 2007-2016 Philippe Gerum.
 */
#ifndef _LINUX_IRQSTAGE_H
#define _LINUX_IRQSTAGE_H

#ifdef CONFIG_IRQ_PIPELINE

#include <linux/percpu.h>
#include <linux/bitops.h>
#include <linux/preempt.h>
#include <asm/irq_pipeline.h>

struct task_struct;
struct hypervisor_stall;

struct irq_stage {
	int index;
	const char *name;
};

extern struct irq_stage root_irq_stage;

extern struct irq_stage *head_irq_stage;

/* Interrupts (virtually) disabled. */
#define STAGE_STALL_BIT		0
/* Stall state upon NMI entry (root stage only). */
#define STAGE_STALL_NMI_BIT	1

struct irq_event_map;

struct irq_log {
	unsigned long himap;
	struct irq_event_map *map;
};

/* Per-CPU, per-stage information. */
struct irq_stage_data {
	unsigned long status;
	struct irq_log log;
	struct irq_stage *stage;
};

/* Per-CPU pipeline descriptor. */
struct irq_pipeline_data {
	struct irq_stage_data stages[2];
	struct irq_stage_data *curr;
	struct pt_regs tick_regs;
};

DECLARE_PER_CPU(struct irq_pipeline_data, irq_pipeline);

struct irq_stage_data *irq_stage_context(struct irq_stage *stage, int cpu);

struct irq_stage_data *irq_stage_this_context(struct irq_stage *stage);

/**
 * irq_root_this_context - return the address of the pipeline context
 * data for the root stage on the current CPU. CPU migration must be
 * disabled.
 *
 * NOTE: this accessor is recommended when the stage we refer to is
 * known at build time to be the root one.
 */
static inline struct irq_stage_data *irq_root_this_context(void)
{
	return raw_cpu_ptr(&irq_pipeline.stages[0]);
}

/**
 * irq_head_this_context - return the address of the pipeline context
 * data for the registered head stage on the current CPU. CPU
 * migration must be disabled.
 *
 * NOTE: this accessor is recommended when the stage we refer to is
 * known at build time to be the registered head stage. This address
 * is always different from the context data of the root stage, even
 * in absence of registered head stage.
 */
static inline struct irq_stage_data *irq_head_this_context(void)
{
	return raw_cpu_ptr(&irq_pipeline.stages[1]);
}

/**
 * irq_get_current_context() - return the address of the pipeline
 * context data of the stage running on the current CPU. CPU migration
 * must be disabled.
 */
static inline struct irq_stage_data *irq_get_current_context(void)
{
	return raw_cpu_read(irq_pipeline.curr);
}

#define irq_current_context irq_get_current_context()

static inline
void __irq_set_current_context(struct irq_stage_data *pd)
{
	struct irq_pipeline_data *p = raw_cpu_ptr(&irq_pipeline);
	p->curr = pd;
}

/**
 * irq_set_*_context() - switch the current CPU to the specified stage
 * context. CPU migration must be disabled.
 *
 * NOTE: calling these routines is the only sane and safe way to
 * change the current stage for the current CPU. Don't bypass,
 * ever. Really.
 */
static inline
void irq_set_head_context(struct irq_stage_data *pd)
{
	__irq_set_current_context(pd);
	if (!(preempt_count() & STAGE_MASK))
		preempt_count_add(STAGE_OFFSET);
}

static inline
void irq_set_root_context(struct irq_stage_data *pd)
{
	__irq_set_current_context(pd);
	if (preempt_count() & STAGE_MASK)
		preempt_count_sub(STAGE_OFFSET);
}

static inline
void irq_set_current_context(struct irq_stage_data *pd)
{
	if (pd->stage == &root_irq_stage)
		irq_set_root_context(pd);
	else
		irq_set_head_context(pd);
}

static inline struct irq_stage *__get_current_irq_stage(void)
{
	return irq_get_current_context()->stage;
}

#define __current_irq_stage	__get_current_irq_stage()

/**
 * __get_current_irq_stage() - return the address of the pipeline
 * stage running on the current CPU.
 */
static inline struct irq_stage *get_current_irq_stage(void)
{
	struct irq_stage *stage;
	unsigned long flags;

	flags = hard_smp_local_irq_save();
	stage = __get_current_irq_stage();
	hard_smp_local_irq_restore(flags);

	return stage;
}

#define current_irq_stage	get_current_irq_stage()

static inline bool on_root_stage(void)
{
	return stage_level() == 0;
}

/*
 * Unlike testing for the leading stage context, being on the head
 * stage really means running over a context distinct from the root
 * one. So on_head_stage() will always return false whenever no head
 * stage is registered at the time of the call.
 */
static inline bool on_head_stage(void)
{
	return !on_root_stage();
}

#define irq_root_status		(irq_root_this_context()->status)
#define irq_head_status		(irq_head_this_context()->status)

static inline bool head_stage_present(void)
{
	return head_irq_stage != &root_irq_stage;
}

/**
 * irq_staged_waiting() - Whether we have interrupts pending
 * (i.e. logged) for the given stage context (which must belong to the
 * current CPU). Hard IRQs must be disabled.
 */
static inline int irq_staged_waiting(struct irq_stage_data *pd)
{
	return pd->log.himap != 0;
}

void __irq_stage_sync_current(void);

void irq_stage_sync_current(void);

void __irq_pipeline_sync(struct irq_stage *top);

void irq_stage_post_event(struct irq_stage *stage,
			  unsigned int irq);

static inline void irq_stage_post_head(unsigned int irq)
{
	irq_stage_post_event(head_irq_stage, irq);
}

static inline void irq_stage_post_root(unsigned int irq)
{
	irq_stage_post_event(&root_irq_stage, irq);
}

static inline void head_irq_disable(void)
{
	hard_local_irq_disable();
	__set_bit(STAGE_STALL_BIT, &irq_head_status);
}

static inline unsigned long head_irq_save(void)
{
	hard_local_irq_disable();
	return __test_and_set_bit(STAGE_STALL_BIT, &irq_head_status);
}

static inline unsigned long head_irqs_disabled(void)
{
	unsigned long flags, ret;

	flags = hard_smp_local_irq_save();
	ret = test_bit(STAGE_STALL_BIT, &irq_head_status);
	hard_smp_local_irq_restore(flags);

	return ret;
}

void head_irq_enable(void);

void __head_irq_restore(unsigned long x);

static inline void head_irq_restore(unsigned long x)
{
	if ((x ^ test_bit(STAGE_STALL_BIT, &irq_head_status)) & 1)
		__head_irq_restore(x);
}

bool irq_stage_disabled(void);

unsigned long irq_stage_test_and_disable(int *irqsoff);

static inline unsigned long irq_stage_disable(void)
{
	return irq_stage_test_and_disable(NULL);
}

void irq_stage_restore(unsigned long flags);

#define irq_stage_save_flags(__flags)				\
	do {							\
		__flags = hard_local_save_flags();		\
		__flags = arch_irqs_merge_flags((__flags), 	\
					irqs_disabled());	\
	} while (0)

#else /* !CONFIG_IRQ_PIPELINE */

static inline bool on_root_stage(void)
{
	return true;
}

static inline bool on_head_stage(void)
{
	return false;
}

static inline bool head_stage_present(void)
{
	return false;
}

static inline bool irq_stage_disabled(void)
{
	return irqs_disabled();
}

#define irq_stage_test_and_disable(__irqsoff)			\
	({							\
		unsigned long __flags;				\
		raw_local_irq_save(__flags);			\
		*(__irqsoff) = irqs_disabled_flags(__flags);	\
		__flags;					\
	})

#define irq_stage_disable()					\
	({							\
		unsigned long __flags;				\
		raw_local_irq_save(__flags);			\
		__flags;					\
	})

#define irq_stage_restore(__flags)	raw_local_irq_restore(__flags)

#define irq_stage_save_flags(__flags)	raw_local_save_flags(__flags)

#endif /* !CONFIG_IRQ_PIPELINE */

#endif	/* !_LINUX_IRQSTAGE_H */
