.cpu cortex-m4
.syntax unified
.thumb

.section .text

.thumb_func
.global enter_user_mode
enter_user_mode:
	bkpt
	LDR r12, =__psp_stack_top
	MSR PSP, r12
	b _crt0
  bkpt
