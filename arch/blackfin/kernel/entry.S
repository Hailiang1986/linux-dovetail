/*
 * Copyright 2004-2009 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later
 */

#include <linux/linkage.h>
#include <asm/thread_info.h>
#include <asm/errno.h>
#include <asm/blackfin.h>
#include <asm/asm-offsets.h>

#include <asm/context.S>

#ifdef CONFIG_EXCPT_IRQ_SYSC_L1
.section .l1.text
#else
.text
#endif

ENTRY(_ret_from_fork)
	SP += -12;
	pseudo_long_call _schedule_tail, p5;
	SP += 12;
	p1 = [sp++];
	r0 = [sp++];
	cc = p1 == 0;
	if cc jump .Lfork;
	sp += -12;
	call (p1);
	sp += 12;
.Lfork:
	RESTORE_CONTEXT
	rti;
ENDPROC(_ret_from_fork)
