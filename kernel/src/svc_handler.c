/**
 * @file 
 *
 * @brief      
 *
 * @date       
 *
 * @author     
 */

#include <stdint.h>
#include <debug.h>
#include <svc_num.h>
#include <syscall.h>
#include <printk.h>
/** @brief ANGLE 90 at 1.6 ms */
#define PULSE_ANGLE90	160
/** @brief ANGLE 180 at 2.4 ms */
#define PULSE_ANGLE180	240
#define UNUSED __attribute__((unused))

typedef struct{	
	uint32_t r0;
	uint32_t r1;
	uint32_t r2;
	uint32_t r3;
	uint32_t r12;
	uint32_t lr;
	uint32_t pc; 
	uint32_t xPSR; 	
}stack_frame_t;

void svc_c_handler(uint8_t svcnum, int* PSP) {
	int svc_number = svcnum;
	stack_frame_t* s = (stack_frame_t*) PSP;
	int ret_val;
	switch ( svc_number ) {
		case SVC_SBRK: 
			ret_val = (int)sys_sbrk(*PSP);
			break;
		case SVC_WRITE: 
			ret_val = sys_write(*PSP, (char*)*(PSP+1), *(PSP+2));
			break;
		case SVC_READ:
			ret_val = sys_read(*PSP, (char*)*(PSP+1), *(PSP+2));
			break;
		case SVC_EXIT: 
			sys_exit(*PSP);
			break;
		case SVC_SERVO_ENABLE:
			ret_val = sys_servo_enable(*PSP, *(PSP+1));
			break;
		case SVC_SERVO_SET: 
			ret_val = sys_servo_set(*PSP, *(PSP+1));
			break;
		case SVC_FSTAT:
			ret_val = (int)sys_fstat(*PSP, (void *)*(PSP+1));
			break;
		default:
		printk( "Not implemented, svc num %d\n", svc_number );
	}
	// place the value in r0 and return
	s->r0 = ret_val;
	return;
}
