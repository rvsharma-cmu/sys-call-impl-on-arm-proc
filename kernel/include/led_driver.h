
#ifndef _LED_DRIVER_H_
#define _LED_DRIVER_H_

#include <unistd.h>

void led_driver_init();

void led_set_display(uint32_t input);

void led_brightness(uint8_t br);

void led_blink_off();

void led_blink_on();

#endif /* _LED_DRIVER_H_ */
