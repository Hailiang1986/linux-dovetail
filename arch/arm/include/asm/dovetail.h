/*
 * arch/arm/include/asm/dovetail.h
 *
 * Copyright (C) 2016 Philippe Gerum.
 */
#ifndef _ASM_ARM_DOVETAIL_H
#define _ASM_ARM_DOVETAIL_H

/* ARM traps */
#define ARM_TRAP_ACCESS		0	/* Data or instruction access exception */
#define ARM_TRAP_SECTION	1	/* Section fault */
#define ARM_TRAP_DABT		2	/* Generic data abort */
#define ARM_TRAP_UNKNOWN	3	/* Unknown exception */
#define ARM_TRAP_BREAK		4	/* Instruction breakpoint */
#define ARM_TRAP_FPU		5	/* Floating point exception */
#define ARM_TRAP_VFP		6	/* VFP floating point exception */
#define ARM_TRAP_UNDEFINSTR	7	/* Undefined instruction */
#define ARM_TRAP_ALIGNMENT	8	/* Unaligned access exception */
#define ARM_TRAP_MAYDAY		9	/* Internal recovery trap */

#endif /* _ASM_ARM_DOVETAIL_H */
