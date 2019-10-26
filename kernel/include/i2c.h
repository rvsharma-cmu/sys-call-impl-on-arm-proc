#ifndef _I2C_H_
#define _I2C_H_

#include <stdint.h>

/** @brief Value to set in the CCR for achieving 100kHz frequency. To be used 
 *  with I2C init clk argument */ 
#define I2C_CCR_11_0 0x80

void i2c_master_init(uint16_t clk);

void i2c_master_start();

void i2c_master_stop();

int i2c_master_write(uint8_t *buf, uint16_t len, uint8_t slave_addr);

int i2c_master_read(uint8_t *buf, uint16_t len, uint8_t slave_addr);

void i2c_master_send_slave_addr(uint8_t slave_addr);

int i2c_master_send_data(uint8_t *buf, uint16_t len);

#endif /* _I2C_H_ */
