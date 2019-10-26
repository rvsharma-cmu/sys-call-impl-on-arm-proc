/**
 * @file kernel.c
 *
 * @brief      Kernel entry point - to test systick using int based uart
 *
 * @date       10-Oct-19
 *
 * @author     Aanand Nayyar (aanandn)
 */

#include <arm.h>

#include <kernel.h>

#include <timer.h>

// #include <uart_polling.h>

#include <uart.h>

#include <printk.h>
#include <i2c.h>
#include <led_driver.h>
#include <syscall.h>

/* comment next line for disabling Kernel debug */
#define DEBUG_K    1

#ifdef DEBUG_K
#define dbg_printk(...) printk(__VA_ARGS__)
#else
#define dbg_printk(...)
#endif


int kernel_main( void ) {
    init_349(); // DO NOT REMOVE THIS LINE
    
//  uart_polling_init(115200);
	uint32_t clock_frequency = 10000;
    uart_init(115200);
    // led and i2c init
    i2c_master_init(I2C_CCR_11_0);
    led_driver_init();
    sys_servo_init(clock_frequency);
	enable_interrupts();
    timer_start(clock_frequency);
    printk("%s: UART and sysTICK init done\n",__func__);
	printk("Entering user mode\n");
	enter_user_mode(); 
    while (1)   {
    }

    return 0;
}
