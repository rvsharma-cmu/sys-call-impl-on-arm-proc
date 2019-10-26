/**
 * @file 
 *
 * @brief      
 *
 * @date       
 *
 * @author     
 */

#ifndef _SYSCALLS_H_
#define _SYSCALLS_H_

void *sys_sbrk(int incr);

void *sys_fstat(int file, void *p);

int sys_write(int file, char *ptr, int len);

int sys_read(int file, char *ptr, int len);

void sys_exit(int status);

int sys_servo_enable(uint8_t channel, uint8_t enabled);

int sys_servo_set(uint8_t channel, uint32_t angle);
void sys_servo_init();
void position();
void sys_servo_position();

#endif /* _SYSCALLS_H_ */
