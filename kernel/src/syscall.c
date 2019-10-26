/**
 * @file 
 *
 * @brief      
 *
 * @date       
 *
 * @author     
 */

#include <unistd.h>
#include <syscall.h>
#include <printk.h>
#include <led_driver.h>
#include <uart.h>
#include <i2c.h>
#include <arm.h>

char* current_sbrk = 0; 
extern char __heap_low;
extern char __heap_top;

#define UNUSED __attribute__((unused))

void *sys_sbrk(int incr){
	if(current_sbrk == 0){
		current_sbrk = &__heap_low;
	}
	char *result_block = current_sbrk; 
	if(current_sbrk + incr > &__heap_top){
		return (void *) -1;
	}
	current_sbrk += incr; 
		
	return (void *)result_block;
}

int sys_write(int file, char *ptr, int len){
	int count = 0; 
	int write = 0; 
	if(file == 1){
		while(count < len){
			if(uart_put_byte(ptr[count]) == -1){
				wait_for_interrupt();
				continue;
			}
			count++;
			write = 1;
		}
	}
	if(write!=0) return count; 
	else return -1;
}


int sys_read(int file, char *ptr, int len){
	int count = 0; 
	char c; 
	if(file == 0){
		while(count < len){
			if(uart_get_byte(&c) == -1){
				wait_for_interrupt();
				continue;
			}
			if(c == 4){
				break;
			}
			if(c == 8 || c == 127) {
				if(count > 0) {
					count--; 
					uart_put_byte('\b');
					uart_put_byte(' ');
					uart_put_byte('\b');
				}
			}else if(c == 10 || c == 13){
				if (c == 13) {
					uart_get_byte(&c);
				}
				*ptr++ = c;
				uart_put_byte('\n');
				count++;
				return count; 
			} else {
				*ptr++ = c;
				count++;
				uart_put_byte(c);
			}
		}
		return count;
	}
	return -1;
}

void *sys_fstat(UNUSED int file, UNUSED void *p){
	return (void *)0;
}

void sys_exit(int status){
	disable_interrupts();
	printk("Exit Status: %d\n", status);
	led_set_display(status);
	uart_flush();
	while(1){
		wait_for_interrupt();
	}
	
}
