#ifndef _ASM_CLOCKSOURCE_H
#define _ASM_CLOCKSOURCE_H

enum vdso_arch_clockmode {
	/* vdso clocksource not usable */
	VDSO_CLOCKMODE_NONE,
	/* vdso clocksource usable */
	VDSO_CLOCKMODE_ARCHTIMER,
	VDSO_CLOCKMODE_ARCHTIMER_NOCOMPAT = VDSO_CLOCKMODE_ARCHTIMER,
};

enum arch_clock_uaccess_type {
	ARM_CLOCK_NONE = 0,
	ARM_CLOCK_ARCH_TIMER,

	ARM_CLOCK_USER_MMIO_BASE, /* Must remain last */
};

struct arch_clocksource_data {
	/* Usable for direct VDSO access? */
	enum vdso_arch_clockmode clock_mode;
	enum arch_clock_uaccess_type clock_type;
};

#endif
