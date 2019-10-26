/**
 * @file	syscall_servo.c
 *
 * @brief	Manages syscalls for the servo
 *
 * @date	10/18/2019
 *
 * @author	rvsharma   
 */

#include <unistd.h>
#include <syscall.h>
#include <gpio.h>
#include <printk.h>

#define UNUSED __attribute__((unused))
/** @brief Channel 1 for Servo */
#define CHN_1 1
/** @brief Channel 2 for Servo */
#define CHN_2 2
/** @brief 20ms pulse duration. So as to send pulse every 20 ms */
#define PULSE_FULL 200

/** @brief This is set to 1 if Servo Channel 1 is enabled. */ 
uint8_t servo1_enabled = 0;
/** @brief This is set to 1 if Servo Channel 2 is enabled. */ 
uint8_t servo2_enabled = 0;
/** @brief current angle for servo 1 */ 
uint32_t servo1_angle = 6;
/** @brief current angle for servo 2 */ 
uint32_t servo2_angle = 6;
/** @brief sys tick timer value */ 
uint32_t sys_tick = 0;
/** @brief indicator to indicate whether pin is high or low */ 
uint8_t servo2_high_low = 0;
/** @brief indicator to indicate whether pin is high or low */ 
uint8_t servo1_high_low = 0;
/** @brief Pulse difference between 0 and 180 degrees */
float pulse_difference = 1.8;
/** @brief angle difference between lowest and highest position of servo */
uint32_t angle_difference = 180;
/** @brief clock frequency set when initializing servo */
uint32_t clk_frequency;
/** pulse offset for SG90 */
float PULSE_OFFSET = 0.6;

/**
 * @brief calculate the counter required for the 
 * corresponding angle in the input 
 * @param angle Angle value in degrees 
 * @return pulse length for the angle to set in the 
 * sys_servo_set function
 */
int calculate_pulse(int angle){
	float resolution = pulse_difference/angle_difference;
	float step = ((float)1000)/clk_frequency;
	return ((resolution * angle) + PULSE_OFFSET)/step;
}

/**
 * @brief enable or disable the servo channel. 
 * This method should enable the periodic pulse if enabled is 1 and 
 * set the channel output to low if enabled is 0
 * @param channel This is the servo channel: 1, 2
 * @param enabled 1 for channel enabled and 0 for disabled
 * @return 0 if successful and -1 if error in the function
 */
int sys_servo_enable(uint8_t channel, uint8_t enabled){

	if(channel == CHN_1){
		servo1_enabled = enabled;
	} else if(channel == CHN_2){
		servo2_enabled = enabled;
	} else {
		return -1;
	}
	return 0;
}

/**
 * @brief servo init function. This function will set the GPIO pins to 
 * corresponding configurations 
 */ 
void sys_servo_init(uint32_t clck_frq){
	// servo pin 1
	gpio_init(GPIO_A, 8, MODE_GP_OUTPUT, OUTPUT_PUSH_PULL, OUTPUT_SPEED_LOW, PUPD_NONE, ALT0);
	// servo pin 2
	gpio_init(GPIO_A, 9, MODE_GP_OUTPUT, OUTPUT_PUSH_PULL, OUTPUT_SPEED_LOW, PUPD_NONE, ALT0);
	clk_frequency = clck_frq;
}

/** 
 * Trigger that is used for setting the servo position 
 * This method will be called each time there is  a 
 * systick interrupt in the system
 */
void sys_servo_position() {
	if(sys_tick == 0){
		if(servo1_enabled == 1){
			gpio_set(GPIO_A, 8);
			servo1_high_low = 1;
		}
		if(servo2_enabled == 1){
			gpio_set(GPIO_A, 9);
			servo2_high_low = 1;
		}
	} else {
		if(servo1_high_low == 1 && sys_tick == servo1_angle){
			gpio_clr(GPIO_A, 8);
			servo1_high_low = 0; 
		}
		if(servo2_high_low == 1 && sys_tick == servo2_angle){
			gpio_clr(GPIO_A, 9);
			servo2_high_low = 0; 
		}
	}
	sys_tick++;
	sys_tick %= PULSE_FULL;
}
	
/**
 * @brief Function for setting the angle for the servo 
 * This method will take the arguments on the channel that is 
 * enabled and set the angle accordingly 
 * @param channel The channel in which to set the angle 
 * @param angle The angle to set the servo too 
 * @return -1 if failed to set the angle or incorrect input given 
 * and returns 0 if successful. 
 */
int sys_servo_set(uint8_t channel, uint32_t angle){
	angle = calculate_pulse(angle);
	if(channel == CHN_1){
		servo1_angle = angle;	
	} else if(channel == CHN_2) {
		servo2_angle = angle;
	} else {
		return -1;
	}
	return 0;
}
