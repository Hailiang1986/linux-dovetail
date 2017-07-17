/*
 * include/linux/dovetail.h
 *
 * Copyright (C) 2016 Philippe Gerum.
 */
#ifndef _LINUX_DOVETAIL_H
#define _LINUX_DOVETAIL_H

#ifdef CONFIG_DOVETAIL

#include <linux/sched.h>
#include <linux/thread_info.h>
#include <asm/dovetail.h>

struct pt_regs;
struct task_struct;

#define KEVENT_TASK_SCHEDULE	0
#define KEVENT_TASK_SIGWAKE	1
#define KEVENT_TASK_SETAFFINITY	2
#define KEVENT_TASK_EXIT	3
#define KEVENT_PROCESS_CLEANUP	4

struct dovetail_migration_data {
	struct task_struct *task;
	int dest_cpu;
};

struct hypervisor_stall {
	void (*handler)(struct hypervisor_stall *nfy);
};

int dovetail_start(void);

void dovetail_stop(void);

void dovetail_init_task(struct task_struct *p);

void arch_dovetail_init_task(struct task_struct *p);

int dovetail_handle_syscall(struct thread_info *ti,
			    unsigned long syscall, struct pt_regs *regs);

void dovetail_handle_trap(unsigned int trapnr,
			  struct pt_regs *regs);

void dovetail_handle_kevent(int event, void *data);


void dovetail_clock_set(void);

#ifdef CONFIG_DOVETAIL_TRACK_VM_GUEST
void dovetail_hypervisor_stall(void);
#else
static inline void dovetail_hypervisor_stall(void) { }
#endif

static inline void dovetail_signal_task(struct task_struct *p)
{
	if (test_ti_local_flags(task_thread_info(p), _TLF_DOVETAIL))
		dovetail_handle_kevent(KEVENT_TASK_SIGWAKE, p);
}

static inline void dovetail_change_task_affinity(struct task_struct *p, int cpu)
{
	if (test_ti_local_flags(task_thread_info(p), _TLF_DOVETAIL)) {
		struct dovetail_migration_data d = {
			.task = p,
			.dest_cpu = cpu,
		};
		dovetail_handle_kevent(KEVENT_TASK_SETAFFINITY, &d);
	}
}

static inline void dovetail_task_exit(void)
{
	if (test_thread_local_flags(_TLF_DOVETAIL))
		dovetail_handle_kevent(KEVENT_TASK_EXIT, NULL);
}

static inline
void dovetail_context_switch(struct task_struct *next)
{
	struct task_struct *prev = current;

	if (test_ti_local_flags(task_thread_info(next), _TLF_DOVETAIL) ||
	    test_ti_local_flags(task_thread_info(prev), _TLF_DOVETAIL)) {
		__this_cpu_write(irq_pipeline.rqlock_owner, prev);
		dovetail_handle_kevent(KEVENT_TASK_SCHEDULE, next);
	}
}

static inline void dovetail_mm_cleanup(struct mm_struct *mm)
{
	/*
	 * Notify regardless of _TLF_DOVETAIL: current may have
	 * resources to clean up although it might not be interested
	 * in other kernel events.
	 */
	dovetail_handle_kevent(KEVENT_PROCESS_CLEANUP, mm);
}

/* Hypervisor-side calls, hw IRQs off. */
static inline void dovetail_enter_vm_guest(struct hypervisor_stall *nfy)
{
	struct irq_pipeline_data *p = raw_cpu_ptr(&irq_pipeline);
	p->vm_notifier = nfy;
	barrier();
}

static inline void dovetail_exit_vm_guest(void)
{
	struct irq_pipeline_data *p = raw_cpu_ptr(&irq_pipeline);
	p->vm_notifier = NULL;
	barrier();
}

static inline void dovetail_clear_callouts(struct irq_stage_data *p)
{
	__clear_bit(STAGE_CALLOUT_BIT, &p->status);
}

static inline void dovetail_prepare_switch(struct task_struct *next)
{
	dovetail_context_switch(next);
	hard_local_irq_disable();
}

static inline void dovetail_leave_head(void)
{
	clear_thread_local_flags(_TLF_HEAD);
}

static inline void dovetail_leave_root(void)
{
	dovetail_hypervisor_stall();
}

int dovetail_context_switch_tail(void);

void dovetail_enable(int flags);

void dovetail_disable(void);

void dovetail_complete_domain_migration(void);

int dovetail_enter_head(void);

void dovetail_enter_root(void);

void dovetail_root_sync(void);

static inline
struct dovetail_state *dovetail_current_state(void)
{
	return &current_thread_info()->dovetail_state;
}

static inline
struct dovetail_state *dovetail_task_state(struct task_struct *p)
{
	return &task_thread_info(p)->dovetail_state;
}

static inline void dovetail_send_mayday(struct task_struct *castaway)
{
	struct thread_info *ti = task_thread_info(castaway);

	if (test_ti_local_flags(ti, _TLF_DOVETAIL))
		set_ti_thread_flag(ti, TIF_MAYDAY);
}

#define dovetail_switch_mm_enter(flags)		\
  do {						\
    	(flags) = hard_cond_local_irq_save();	\
	barrier();				\
  } while (0)					\

#define dovetail_switch_mm_exit(flags)	\
  do {						\
	barrier();				\
    	hard_cond_local_irq_restore(flags);	\
  } while (0)					\

#else	/* !CONFIG_DOVETAIL */

static inline
void dovetail_init_task(struct task_struct *p) { }

struct irq_stage_data;

#define dovetail_handle_trap(__trapnr, __regs)	 do { } while (0)

static inline
int dovetail_handle_syscall(struct thread_info *ti,
			    unsigned long syscall, struct pt_regs *regs)
{
	return 0;
}

static inline void dovetail_signal_task(struct task_struct *p) { }

static inline
void dovetail_change_task_affinity(struct task_struct *p, int cpu) { }

static inline void dovetail_task_exit(void) { }

static inline void dovetail_mm_cleanup(struct mm_struct *mm) { }

#define dovetail_enter_vm_guest(__nfy) do { } while (0)

#define dovetail_exit_vm_guest(__nfy) do { } while (0)

static inline void dovetail_complete_domain_migration(void) { }

static inline void dovetail_prepare_switch(struct task_struct *next) { }

static inline int dovetail_context_switch_tail(void)
{
	return 0;
}

static inline void dovetail_clear_callouts(struct irq_stage_data *p) { }

#define dovetail_switch_mm_enter(flags)		\
  do { (void)(flags); } while (0)

#define dovetail_switch_mm_exit(flags)	\
  do { (void)(flags); } while (0)

static inline void dovetail_clock_set(void) { }

#endif	/* !CONFIG_DOVETAIL */

static inline bool dovetailing(void)
{
	return IS_ENABLED(CONFIG_DOVETAIL);
}

static inline bool dovetail_debug(void)
{
	return IS_ENABLED(CONFIG_DEBUG_DOVETAIL);
}

#endif /* _LINUX_DOVETAIL_H */
