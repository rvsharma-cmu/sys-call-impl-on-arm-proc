/**
 * @file led_driver.c
 *
 * @author aanandn, rvsharma
 *
 * Core logic for led_driver:
 *
 * core logic to drive four 7 segment displays of the LED chip
 * on the PCB. Uses switch based table for conversion
 *
 */

#include <i2c.h>
#include <led_driver.h>
#include <unistd.h>
#include <printk.h>

/* comment next line for disabling Led driver debug */
//#define DEBUG_LEDDR    1

#ifdef DEBUG_LEDDR
#define dbg_printk(...) printk(__VA_ARGS__)
#else
#define dbg_printk(...)
#endif

#define LED_SLAVE_ADDR      0xE0    /* slave address */

#define LED_OSCILLATOR_ON   0x21    /* HTK cmd essentially turns on HTK chip */

#define LED_DISPLAY_ON      0x81    /* cmd - display on */
#define LED_DISPLAY_OFF     0x80    /* cmd - display off */
#define LED_MAX_BRIGHTNESS  0xEF    /* cmd - brightness max */

#define BLINK_1HZ       (0x02 << 1) /* blink rate 1HZ value */

#define LED_COM4            0x08
#define LED_COM3            0x06
#define LED_COM1            0x02
#define LED_COM0            0x00

#define NUM_SEVEN_SEG       4
#define EXTRACT_2_BYTE      0xFFFF

#define LED_KEY_DATA_

uint8_t hex_to_seven_segment(uint8_t hex);

/** @brief write buffer to display RAM
 *
 * @param disp_buffer is the LED display buffer
 */
void write_display(uint8_t disp_buffer[]) {
    uint8_t curr_byte, next_byte;

    i2c_master_start();
    i2c_master_send_slave_addr(LED_SLAVE_ADDR);

    for (uint8_t i = 0; i < 8; i++) {
        curr_byte = disp_buffer[i];
        next_byte = 0;
        i2c_master_send_data(&curr_byte, 1);
        i2c_master_send_data(&next_byte, 1);
    }
    i2c_master_stop();
}

void led_send_data(uint8_t *buf, uint16_t len) {
    i2c_master_start();

    i2c_master_write(buf, len, LED_SLAVE_ADDR);

    i2c_master_stop();

}

/** @brief LED initialization: Oscillator enabling, Turning on the display
 * Setting max brightness; and Clearing the RAM
 */
void led_driver_init() {
    uint8_t cmd, disp_buffer[8] = {0};

    dbg_printk("%s: Turning the oscilator on\n", __func__);
    cmd = LED_OSCILLATOR_ON;
    led_send_data(&cmd, 1);

    // send the display on command
    dbg_printk("%s: turning the display on\n", __func__);
    cmd = LED_DISPLAY_ON;
    led_send_data(&cmd, 1);

    //full brightness
    dbg_printk("%s: turning on max brightness\n", __func__);
    cmd = LED_MAX_BRIGHTNESS;
    led_send_data(&cmd, 1);

    write_display(disp_buffer); // zero out display
}

/** @brief sends each hex digit to the LED seven segment digits
 * @param led_addr Address of the LED driver
 * @param input half byte to be sent
 */
void led_set_each_byte(uint8_t led_addr, uint8_t input) {

    i2c_master_start();

    dbg_printk("%s: sending the slave address as input\n", __func__);
    i2c_master_send_slave_addr(LED_SLAVE_ADDR);

    dbg_printk("%s: sending the command register address as input\n", __func__);
    i2c_master_send_data(&led_addr, 1);

    dbg_printk("%s: sending the data register as input\n", __func__);
    i2c_master_send_data(&input, 1);

    i2c_master_stop();
}

/** @brief Set the 4 seven segment LEDs to the given 16 bit input
 *
 * @param input Maximum 4 hex digits can be displayed together (4 * 4bits)
 */
void led_set_display(uint32_t input) {
    uint8_t address_arr[NUM_SEVEN_SEG] = {LED_COM4, LED_COM3, LED_COM1,
                                          LED_COM0};

    input &= EXTRACT_2_BYTE;
    for (uint32_t i = 0; i < NUM_SEVEN_SEG; i++) {
        uint8_t seven_seg_val = hex_to_seven_segment((input >> (4 * i)) & 0x0F);
        led_set_each_byte(address_arr[i], seven_seg_val);
    }
}

uint8_t hex_to_seven_segment(uint8_t hex) {
    uint8_t result;

    switch (hex) {
        case 0x0:
            result = 0b00111111;
            break;

        case 0x1:
            result = 0b00000110;
            break;

        case 0x2:
            result = 0b01011011;
            break;

        case 0x3:
            result = 0b01001111;
            break;

        case 0x4:
            result = 0b01100110;
            break;

        case 0x5:
            result = 0b01101101;
            break;

        case 0x6:
            result = 0b01111101;
            break;

        case 0x7:
            result = 0b00000111;
            break;

        case 0x8:
            result = 0b01111111;
            break;

        case 0x9:
            result = 0b01101111;
            break;

        case 0xA:
            result = 0b01110111;
            break;

        case 0xB:
            result = 0b01111100;
            break;

        case 0xC:
            result = 0b00111001;
            break;

        case 0xD:
            result = 0b01011110;
            break;

        case 0xE:
            result = 0b01111001;
            break;
        case 0xF:
            result = 0b01110001;
            break;

        default:
            result = 0b10000000; // dot when error
    }
    return result;
}