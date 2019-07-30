/*
 * SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (C) 2019 Philippe Gerum  <rpm@xenomai.org>.
 */
#include <linux/kernel.h>
#include <linux/smp.h>
#include <linux/irq.h>
#include <linux/irq_pipeline.h>
#include <asm/irqdomain.h>
#include <asm/apic.h>
#include <asm/traps.h>
#include <asm/irq_work.h>
#include <asm/mshyperv.h>

static struct irq_domain *sipic_domain;

static void sipic_irq_noop(struct irq_data *data) { }

static unsigned int sipic_irq_noop_ret(struct irq_data *data)
{
	return 0;
}

static struct irq_chip sipic_chip = {
	.name		= "SIPIC",
	.irq_startup	= sipic_irq_noop_ret,
	.irq_shutdown	= sipic_irq_noop,
	.irq_enable	= sipic_irq_noop,
	.irq_disable	= sipic_irq_noop,
	.flags		= IRQCHIP_PIPELINE_SAFE | IRQCHIP_SKIP_SET_WAKE,
};

void handle_apic_irq(struct irq_desc *desc)
{
	unsigned int irq = irq_desc_get_irq(desc);
	struct pt_regs *regs = get_irq_regs(); /* from generic_pipeline_irq() */

	if (WARN_ON_ONCE(irq_pipeline_debug() && !on_pipeline_entry()))
		return;

	switch (apicm_irq_vector(irq)) {
	case SPURIOUS_APIC_VECTOR:
	case ERROR_APIC_VECTOR:
		/*
		 * No ack for error events, which should never
		 * happen. If they do, the situation is messy, leave
		 * the decision to acknowledge or not to the in-band
		 * handler.
		 */
		break;
	case THERMAL_APIC_VECTOR:
		/*
		 * MCE events are non-maskable, their in-band handlers
		 * have to be OOB-compatible by construction, so we
		 * can run them immediately.
		 */
		smp_thermal_interrupt(regs);
		__ack_APIC_irq();
		return;
	case THRESHOLD_APIC_VECTOR:
		smp_threshold_interrupt(regs);
		__ack_APIC_irq();
		return;
	default:
		__ack_APIC_irq();
	}

	handle_oob_irq(desc);
}

void irq_pipeline_send_remote(unsigned int ipi,
			      const struct cpumask *cpumask)
{
	apic->send_IPI_mask_allbutself(cpumask,	apicm_irq_vector(ipi));
}
EXPORT_SYMBOL_GPL(irq_pipeline_send_remote);

void uv_bau_message_interrupt(struct pt_regs *regs);

static void do_apic_irq(unsigned int irq, struct pt_regs *regs)
{
	int vector = apicm_irq_vector(irq);

	switch (vector) {
	case SPURIOUS_APIC_VECTOR:
		smp_spurious_interrupt(regs);
		break;
	case ERROR_APIC_VECTOR:
		smp_error_interrupt(regs);
		break;
#ifdef CONFIG_SMP
	case RESCHEDULE_VECTOR:
		smp_reschedule_interrupt(regs);
		break;
	case CALL_FUNCTION_VECTOR:
		smp_call_function_interrupt(regs);
		break;
	case CALL_FUNCTION_SINGLE_VECTOR:
		smp_call_function_single_interrupt(regs);
		break;
	case REBOOT_VECTOR:
		smp_reboot_interrupt();
		break;
#endif
	case X86_PLATFORM_IPI_VECTOR:
		smp_x86_platform_ipi(regs);
		break;
	case IRQ_WORK_VECTOR:
		smp_irq_work_interrupt(regs);
		break;
#ifdef CONFIG_X86_UV
	case UV_BAU_MESSAGE:
		uv_bau_message_interrupt(regs);
		break;
#endif
#ifdef CONFIG_X86_MCE_AMD
	case DEFERRED_ERROR_VECTOR:
		smp_deferred_error_interrupt(regs);
		break;
#endif
#ifdef CONFIG_HAVE_KVM
	case POSTED_INTR_VECTOR:
		smp_kvm_posted_intr_ipi(regs);
		break;
	case POSTED_INTR_WAKEUP_VECTOR:
		smp_kvm_posted_intr_wakeup_ipi(regs);
		break;
	case POSTED_INTR_NESTED_VECTOR:
		smp_kvm_posted_intr_nested_ipi(regs);
		break;
#endif
#ifdef CONFIG_HYPERV
	case HYPERVISOR_CALLBACK_VECTOR:
		hyperv_vector_handler(regs);
		break;
	case HYPERV_REENLIGHTENMENT_VECTOR:
		hyperv_reenlightenment_intr(regs);
		break;
	case HYPERV_STIMER0_VECTOR:
		hv_stimer0_vector_handler(regs);
		break;
#endif
	case LOCAL_TIMER_VECTOR:
		smp_apic_timer_interrupt(regs);
		break;
	case THERMAL_APIC_VECTOR:
	case THRESHOLD_APIC_VECTOR:
		/*
		 * MCE have been dealt with immediatly on entry to the
		 * pipeline (see handle_apic_irq()).
		 */
		break;
	default:
		printk_once(KERN_ERR "irq_pipeline: unexpected event"
			" on vector #%.2x (irq=%u)", vector, irq);
	}
}

void arch_do_IRQ_pipelined(struct irq_desc *desc)
{
	struct pt_regs *regs = raw_cpu_ptr(&irq_pipeline.tick_regs);
	struct pt_regs *old_regs = set_irq_regs(regs);
	unsigned int irq = irq_desc_get_irq(desc);

	if (desc->irq_data.domain == sipic_domain) {
		do_apic_irq(irq, regs);
		return;
	}

	entering_irq();
	generic_handle_irq_desc(desc);
	exiting_irq();

	set_irq_regs(old_regs);
}

__visible unsigned int __irq_entry handle_arch_irq_pipelined(struct pt_regs *regs)
{
	unsigned int irq, vector = ~regs->orig_ax;
	struct irq_desc *desc;

	if (vector >= FIRST_SYSTEM_VECTOR)
		irq = apicm_vector_irq(vector);
	else {
		desc = __this_cpu_read(vector_irq[vector]);
		if (unlikely(desc == NULL)) {
			pr_err("IRQ pipeline: unhandled vector %#.2x\n", vector);
			return 0;
		}
		irq = irq_desc_get_irq(desc);
	}

	generic_pipeline_irq(irq, regs);

	return leave_irq_pipeline(regs);
}

static int sipic_irq_map(struct irq_domain *d, unsigned int irq,
			irq_hw_number_t hwirq)
{
	irq_set_percpu_devid(irq);
	irq_set_chip_and_handler(irq, &sipic_chip, handle_apic_irq);

	return 0;
}

static struct irq_domain_ops sipic_domain_ops = {
	.map	= sipic_irq_map,
};

static void create_x86_apic_domain(void)
{
	sipic_domain = irq_domain_add_simple(NULL, NR_APIC_VECTORS,
					     FIRST_SYSTEM_IRQ,
					     &sipic_domain_ops, NULL);
}

#ifdef CONFIG_SMP

void handle_irq_move_cleanup(struct irq_desc *desc)
{
	if (on_pipeline_entry()) {
		/* First there on receipt from hardware. */
		__ack_APIC_irq();
		handle_oob_irq(desc);
	} else /* Next there on inband delivery. */
		smp_irq_move_cleanup_interrupt();
}

static void smp_setup(void)
{
	int irq;

	/*
	 * The IRQ cleanup event must be pipelined to the inband
	 * stage, so we need a valid IRQ descriptor for it. Since we
	 * still are in the early boot stage on CPU0, we ask for a 1:1
	 * mapping between the vector number and IRQ number, to make
	 * things easier for us later on.
	 */
	irq = irq_alloc_desc_at(IRQ_MOVE_CLEANUP_VECTOR, 0);
	WARN_ON(IRQ_MOVE_CLEANUP_VECTOR != irq);
	/*
	 * Set up the vector_irq[] mapping array for the boot CPU,
	 * other CPUs will copy this entry when their APIC is going
	 * online (see lapic_online()).
	 */
	per_cpu(vector_irq, 0)[irq] = irq_to_desc(irq);

	irq_set_chip_and_handler(irq, &dummy_irq_chip,
				handle_irq_move_cleanup);
}

#else

static void smp_setup(void) { }

#endif

void __init arch_irq_pipeline_init(void)
{
	/*
	 * Create an IRQ domain for mapping APIC system interrupts
	 * (in-band and out-of-band), with fixed sirq numbers starting
	 * from FIRST_SYSTEM_IRQ. Upon receipt of a system interrupt,
	 * the corresponding sirq is injected into the pipeline.
	 */
	create_x86_apic_domain();

	smp_setup();
}
