#include </tmp/349_arg.h>
#include <stdlib.h>
#include <stdio.h>

extern int main( int argc, char const *argv[] );

/** NOTE : An rtos with memory protection would jump to user mode via exception
 * return. That way the jump to user code and switch to user mode happens
 * atomically, that way you dont get a mpu fault. For simplicity, here we jump
 * to user code first and then switch to user mode. THIS IS HACKY AND
 * VVVVVBAD.
 */

void __attribute__ ( ( noinline ) ) crt0() {
	int ret_val = main( user_argc, user_argv );
	// have to spin to ensure uart buffer is flushed
	exit( ret_val );
}

void _crt0() {
  __asm volatile( "mov r0, #3" );
  __asm volatile( "msr control, r0" );
  __asm volatile( "isb" );
  // cannot inline crt0 becasue if you return here, you will be using the
  // old stack
  crt0();
}
