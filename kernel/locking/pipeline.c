/* -*- linux-c -*-
 * kernel/locking/pipeline.c
 *
 * Copyright (C) 2016 Philippe Gerum.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, Inc., 675 Mass Ave, Cambridge MA 02139,
 * USA; either version 2 of the License, or (at your option) any later
 * version.
 */
#include <linux/linkage.h>
#include <linux/preempt.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/irq_pipeline.h>
#include <linux/kconfig.h>

/*
 * A mutable spinlock behaves in different ways depending on the
 * current interrupt stage on entry.
 *
 * Such spinlock always leaves hard IRQs disabled once locked. In
 * addition, it stalls the root stage when protecting a critical
 * section there, disabling preemption like regular spinlocks do as
 * well. This combination preserves the regular locking logic when
 * called from the root stage, while fully disabling preemption by
 * other interrupt stages.
 *
 * When taken from the pipeline entry context, a mutable lock behaves
 * like a hard spinlock, assuming that hard IRQs are already disabled.
 *
 * The irq descriptor lock (struct irq_desc) is a typical example of
 * such lock, which properly serializes accesses regardless of the
 * calling context.
 */
void __mutable_spin_lock(struct raw_spinlock *rlock)
{
	struct mutable_spinlock *lock;
	unsigned long __flags;

	if (on_root_stage())
		preempt_disable();

	__flags = hard_local_irq_save();
	hard_lock_acquire(rlock, 0, _RET_IP_);
	LOCK_CONTENDED(rlock, do_raw_spin_trylock, do_raw_spin_lock);
	lock = container_of(rlock, struct mutable_spinlock, rlock);
	lock->hwflags = __flags;
}
EXPORT_SYMBOL(__mutable_spin_lock);

void __mutable_spin_unlock(struct raw_spinlock *rlock)
{
	struct mutable_spinlock *lock;
	unsigned long __flags;

	hard_lock_release(rlock, _RET_IP_);
	lock = container_of(rlock, struct mutable_spinlock, rlock);
	__flags = lock->hwflags;
	do_raw_spin_unlock(rlock);
	hard_local_irq_restore(__flags);

	if (on_root_stage())
		preempt_enable();
}
EXPORT_SYMBOL(__mutable_spin_unlock);

void __mutable_spin_lock_irq(struct raw_spinlock *rlock)
{
	struct mutable_spinlock *lock;
	unsigned long __flags;

	__flags = hard_local_irq_save();

	if (on_root_stage()) {
		__set_bit(STAGE_STALL_BIT, &irq_root_status);
		if (!hard_irqs_disabled_flags(__flags))
			trace_hardirqs_off();
		preempt_disable();
	}

	hard_lock_acquire(rlock, 0, _RET_IP_);
	LOCK_CONTENDED(rlock, do_raw_spin_trylock, do_raw_spin_lock);
	lock = container_of(rlock, struct mutable_spinlock, rlock);
	lock->hwflags = __flags;
}
EXPORT_SYMBOL(__mutable_spin_lock_irq);

void __mutable_spin_unlock_irq(struct raw_spinlock *rlock)
{
	struct mutable_spinlock *lock;
	unsigned long __flags;

	hard_lock_release(rlock, _RET_IP_);
	lock = container_of(rlock, struct mutable_spinlock, rlock);
	__flags = lock->hwflags;
	do_raw_spin_unlock(rlock);

	if (on_root_stage()) {
		if (!hard_irqs_disabled_flags(__flags))
			trace_hardirqs_on();
		__clear_bit(STAGE_STALL_BIT, &irq_root_status);
		hard_local_irq_restore(__flags);
		preempt_enable();
		return;
	}

	hard_local_irq_restore(__flags);
}
EXPORT_SYMBOL(__mutable_spin_unlock_irq);

unsigned long __mutable_spin_lock_irqsave(struct raw_spinlock *rlock)
{
	struct mutable_spinlock *lock;
	unsigned long __flags, flags;

	__flags = flags = hard_local_irq_save();

	if (on_root_stage()) {
		flags = __test_and_set_bit(STAGE_STALL_BIT, &irq_root_status);
		if (!hard_irqs_disabled_flags(__flags))
			trace_hardirqs_off();
		preempt_disable();
	}
	
	hard_lock_acquire(rlock, 0, _RET_IP_);
	LOCK_CONTENDED(rlock, do_raw_spin_trylock, do_raw_spin_lock);
	lock = container_of(rlock, struct mutable_spinlock, rlock);
	lock->hwflags = __flags;

	return flags;
}
EXPORT_SYMBOL(__mutable_spin_lock_irqsave);

void __mutable_spin_unlock_irqrestore(struct raw_spinlock *rlock,
				      unsigned long flags)
{
	struct mutable_spinlock *lock;
	unsigned long __flags;

	hard_lock_release(rlock, _RET_IP_);
	lock = container_of(rlock, struct mutable_spinlock, rlock);
	__flags = lock->hwflags;
	do_raw_spin_unlock(rlock);

	if (on_root_stage()) {
		if (!flags) {
			if (!hard_irqs_disabled_flags(__flags))
				trace_hardirqs_on();
			__clear_bit(STAGE_STALL_BIT, &irq_root_status);
		}
		hard_local_irq_restore(__flags);
		preempt_enable();
		return;
	}

	hard_local_irq_restore(__flags);
}
EXPORT_SYMBOL(__mutable_spin_unlock_irqrestore);

int __mutable_spin_trylock(struct raw_spinlock *rlock)
{
	struct mutable_spinlock *lock;
	unsigned long __flags;

	if (on_root_stage())
		preempt_disable();
	
	lock = container_of(rlock, struct mutable_spinlock, rlock);
	__flags = hard_local_irq_save();

	if (do_raw_spin_trylock(rlock)) {
		lock->hwflags = __flags;
		hard_lock_acquire(rlock, 1, _RET_IP_);
		return 1;
	}

	hard_local_irq_restore(__flags);

	if (on_root_stage())
		preempt_enable();

	return 0;
}
EXPORT_SYMBOL(__mutable_spin_trylock);

int __mutable_spin_trylock_irqsave(struct raw_spinlock *rlock,
				   unsigned long *flags)
{
	struct mutable_spinlock *lock;
	unsigned long __flags;
	bool on_root;

	on_root = on_root_stage();

	__flags = *flags = hard_local_irq_save();

	lock = container_of(rlock, struct mutable_spinlock, rlock);
	if (on_root) {
		*flags = __test_and_set_bit(STAGE_STALL_BIT, &irq_root_status);
		if (!hard_irqs_disabled_flags(__flags))
			trace_hardirqs_off();
		preempt_disable();
	}
	
	if (do_raw_spin_trylock(rlock)) {
		hard_lock_acquire(rlock, 1, _RET_IP_);
		lock->hwflags = __flags;
		return 1;
	}

	if (on_root && !*flags) {
		trace_hardirqs_on();
		__clear_bit(STAGE_STALL_BIT, &irq_root_status);
	}
	
	hard_local_irq_restore(__flags);

	if (on_root)
		preempt_enable();

	return 0;
}
EXPORT_SYMBOL(__mutable_spin_trylock_irqsave);
