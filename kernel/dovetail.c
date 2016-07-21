/* -*- linux-c -*-
 * kernel/dovetail.c
 *
 * Copyright (C) 2002-2018 Philippe Gerum.
 *
 * Dual kernel interface.
 */
#include <linux/timekeeper_internal.h>
#include <linux/sched/signal.h>
#include <linux/irq_pipeline.h>
#include <linux/dovetail.h>
#include <asm/asm-offsets.h>
#include <asm/unistd.h>
#include <asm/syscall.h>

static bool dovetail_enabled;

void __weak arch_dovetail_init_task(struct task_struct *p)
{
}

void dovetail_init_task(struct task_struct *p)
{
	struct thread_info *ti = task_thread_info(p);

	clear_ti_local_flags(ti, _TLF_DOVETAIL|_TLF_HEAD);
	arch_dovetail_init_task(p);
}

void __weak arch_dovetail_enable(int flags)
{
}

void dovetail_enable(int flags)	/* flags unused yet. */
{
	check_root_stage();
	arch_dovetail_enable(flags);
	set_thread_local_flags(_TLF_DOVETAIL);
}
EXPORT_SYMBOL_GPL(dovetail_enable);

void dovetail_disable(void)
{
	clear_thread_local_flags(_TLF_DOVETAIL);
	clear_thread_flag(TIF_MAYDAY);
}
EXPORT_SYMBOL_GPL(dovetail_disable);

void __weak dovetail_fastcall_hook(struct pt_regs *regs)
{
}

int __weak dovetail_syscall_hook(struct irq_stage *stage,
				 struct pt_regs *regs)
{
	return 0;
}

void __weak dovetail_mayday_hook(struct pt_regs *regs)
{
}

static inline
void call_mayday(struct thread_info *ti, struct pt_regs *regs)
{
	clear_ti_thread_flag(ti, TIF_MAYDAY);
	dovetail_mayday_hook(regs);
}

void dovetail_call_mayday(struct thread_info *ti, struct pt_regs *regs)
{
	unsigned long flags;

	flags = hard_local_irq_save();
	call_mayday(ti, regs);
	hard_local_irq_restore(flags);
}

int dovetail_pipeline_syscall(struct thread_info *ti, struct pt_regs *regs)
{
	struct irq_stage_data *p, *this_context;
	struct irq_stage *caller_stage, *stage;
	unsigned long flags;
	int ret = 0;

	/*
	 * We should definitely not pipeline a syscall through the
	 * slow path with IRQs off.
	 */
	WARN_ON_ONCE(dovetail_debug() && hard_irqs_disabled());

	if (!dovetail_enabled)
		return 0;

	flags = hard_local_irq_save();
	caller_stage = __current_irq_stage;
	this_context = irq_get_current_context();
	stage = head_irq_stage;
next:
	p = irq_stage_this_context(stage);
	irq_set_current_context(p);
	__set_bit(STAGE_CALLOUT_BIT, &p->status);
	hard_local_irq_restore(flags);
	ret = dovetail_syscall_hook(caller_stage, regs);
	flags = hard_local_irq_save();
	__clear_bit(STAGE_CALLOUT_BIT, &p->status);
	if (__current_irq_stage != stage)
		/* Account for stage migration. */
		this_context = irq_get_current_context();
	else
		irq_set_current_context(this_context);

	if (this_context->stage == &root_irq_stage) {
		if (stage != &root_irq_stage && ret == 0) {
			stage = &root_irq_stage;
			goto next;
		}
		/*
		 * Careful: we may have migrated from head->root, so p
		 * would be irq_stage_this_context(head).
		 */
		p = irq_root_this_context();
		if (irq_staged_waiting(p))
			irq_stage_sync_current();
 	} else if (test_ti_thread_flag(ti, TIF_MAYDAY))
		call_mayday(ti, regs);

	hard_local_irq_restore(flags);

	return ret;
}

void dovetail_root_sync(void)
{
	struct irq_stage_data *p;
	unsigned long flags;

	flags = hard_local_irq_save();

	p = irq_root_this_context();
	if (irq_staged_waiting(p))
		irq_stage_sync_current();

	hard_local_irq_restore(flags);
}

int dovetail_handle_syscall(struct thread_info *ti,
			    unsigned long nr, struct pt_regs *regs)
{
	unsigned long local_flags = READ_ONCE(ti_local_flags(ti));
	int ret;

	/*
	 * If the syscall # is out of bounds and the current IRQ stage
	 * is not the root one, this has to be a non-native system
	 * call handled by some co-kernel on the head stage. Hand it
	 * over to the head stage via the fast syscall handler.
	 *
	 * Otherwise, if the system call is out of bounds or the
	 * current thread is shared with a co-kernel (aka
	 * "dovetailed"), hand the syscall over to the latter through
	 * the pipeline stages. This allows:
	 *
	 * - the co-kernel to receive the initial - foreign - syscall
	 * a thread should send for enabling dovetailing.
	 *
	 * - the co-kernel to manipulate the current execution stage
	 * for handling the request, which includes switching the
	 * current thread back to the root stage if the syscall is a
	 * native one, or promoting it to the head stage if handling
	 * the foreign syscall requires this.
	 *
	 * Native syscalls from regular (non-dovetailed) threads are
	 * ignored by this routine, and flow down to the regular
	 * system call handler.
	 */

	if (nr >= NR_syscalls && (local_flags & _TLF_HEAD)) {
		dovetail_fastcall_hook(regs);
		local_flags = READ_ONCE(ti_local_flags(ti));
		if (local_flags & _TLF_HEAD) {
			if (test_ti_thread_flag(ti, TIF_MAYDAY))
				call_mayday(ti, regs);
			return 1; /* don't pass down, no tail work. */
		} else {
			dovetail_root_sync();
			return -1; /* don't pass down, do tail work. */
		}
	}

	if ((local_flags & _TLF_DOVETAIL) || nr >= NR_syscalls) {
		ret = dovetail_pipeline_syscall(ti, regs);
		local_flags = READ_ONCE(ti_local_flags(ti));
		if (local_flags & _TLF_HEAD)
			return 1; /* don't pass down, no tail work. */
		if (ret)
			return -1; /* don't pass down, do tail work. */
	}

	return 0; /* pass syscall down to the host. */
}

void __weak dovetail_trap_hook(unsigned int trapnr, struct pt_regs *regs)
{
}

void dovetail_handle_trap(unsigned int exception, struct pt_regs *regs)
{
	struct irq_stage_data *p;
	unsigned long flags;

	/*
	 * We send a notification about all traps raised over a
	 * registered head stage only.
	 */
	if (on_root_stage())
		return;

	flags = hard_local_irq_save();
	p = irq_head_this_context();

	if (likely(dovetail_enabled)) {
		__set_bit(STAGE_CALLOUT_BIT, &p->status);
		hard_local_irq_restore(flags);
		dovetail_trap_hook(exception, regs);
		flags = hard_local_irq_save();
		__clear_bit(STAGE_CALLOUT_BIT, &p->status);
	}

	hard_local_irq_restore(flags);
}

void __weak dovetail_kevent_hook(int kevent, void *data)
{
}

void dovetail_handle_kevent(int kevent, void *data)
{
	struct irq_stage_data *p;
	unsigned long flags;

	check_root_stage();

	flags = hard_local_irq_save();

	p = irq_root_this_context();
	if (likely(dovetail_enabled)) {
		__set_bit(STAGE_CALLOUT_BIT, &p->status);
		hard_local_irq_restore(flags);
		dovetail_kevent_hook(kevent, data);
		flags = hard_local_irq_save();
		__clear_bit(STAGE_CALLOUT_BIT, &p->status);
	}

	hard_local_irq_restore(flags);
}

#ifdef CONFIG_DOVETAIL_TRACK_VM_GUEST
void dovetail_hypervisor_stall(void)
{
	struct hypervisor_stall *nfy;
	struct irq_pipeline_data *p;

	check_hard_irqs_disabled();
	p = raw_cpu_ptr(&irq_pipeline);
	nfy = p->vm_notifier;
	if (unlikely(nfy))
		nfy->handler(nfy);
}
EXPORT_SYMBOL_GPL(dovetail_hypervisor_stall);
#endif

int dovetail_start(void)
{
	check_root_stage();

	if (!head_stage_present())
		return -ENODEV;
	else if (dovetail_enabled)
		return -EBUSY;

	dovetail_enabled = true;
	smp_wmb();

	return 0;
}
EXPORT_SYMBOL_GPL(dovetail_start);

static void sync_event_notifier(struct irq_stage *stage)
{
	struct irq_stage_data *p;
	int cpu;

	/*
	 * Wait for any ongoing callout to finish, since our caller
	 * might subsequently unmap the stage code.
	 */
	for_each_online_cpu(cpu) {
		p = irq_stage_context(stage, cpu);
		while (test_bit(STAGE_CALLOUT_BIT, &p->status))
			schedule_timeout_interruptible(HZ / 50);
	}
}

void dovetail_stop(void)
{
	check_root_stage();

	dovetail_enabled = false;
	smp_wmb();
	sync_event_notifier(head_irq_stage);
	sync_event_notifier(&root_irq_stage);
}
EXPORT_SYMBOL_GPL(dovetail_stop);
