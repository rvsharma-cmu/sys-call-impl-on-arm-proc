/**
 * @file i2c.c
 *
 * @author aanandn, rvsharma
 *
 * Core logic for i2c:
 *
 * logic for initializing the I2C module
 * Method for sending a start and stop condition on the SDA bus
 * Method for sending a slave address and then sending the data bits on the
 * SDA bus
 *
 */

#include <gpio.h>
#include <i2c.h>
#include <unistd.h>
#include <rcc.h>
#include <printk.h>

/* comment next line for disabling I2C debug */
// #define DEBUG_I2C    1

#ifdef DEBUG_I2C
#define dbg_printk(...) printk(__VA_ARGS__)
#else
#define dbg_printk(...)
#endif

/*
 * Struct for the I2C registers
 */
struct i2c_reg_map {
    volatile uint32_t CR1;      /** @brief Control Register 1 */
    volatile uint32_t CR2;      /** @brief Control Register 2 */
    volatile uint32_t OAR1;     /** @brief Own Address Register 1 */
    volatile uint32_t OAR2;     /** @brief Own Address Register 2 */
    volatile uint32_t DR;       /** @brief Data register */
    volatile uint32_t SR1;      /** @brief Status Register 1 */
    volatile uint32_t SR2;      /** @brief Status Register 2 */
    volatile uint32_t CCR;      /** @brief Clock control register */
    volatile uint32_t TRISE;    /** @brief TRISE register */
    volatile uint32_t FLTR;     /** @brief Filter register */
};

/** @brief Base address for I2C1 registers in STM32*/
#define I2C1_BASE (struct i2c_reg_map *) 0x40005400

/** @brief Setting the 7 bit addressing mode */
#define I2C_OAR1_ADDMODE 0x7FFF

/** @brief Should always be kept at 1 by software */
#define I2C_OAR1_BIT14 (1<<14)

/** @brief for setting RCC I2C1 APB enable in RCC */
#define I2C_RCC_APB1ENR (1<<21)

/** @brief Value of I2C_TRISE[5:0] Value = 0d17 = 0x11 */
#define I2C_TRISE_5_0   0x11

/*
 *  CR1/CR2 registers macros
 */
/** @brief Bit for enabling the I2C peripheral */
#define I2C_CR1_PEN     0x1

/** @brief Set the value of Start bit in I2C_CR1 */
#define I2C_CR1_STARTC  (1<<8)

/** @brief Set the value of Stop bit in I2C_CR1 */
#define I2C_CR1_STOPC   (1<<9)

/** @brief I2C_CR1_ACK, the bit for enabling
  * the generation of Acknowledgement. */
#define I2C_CR1_ACK     (1<<10)

/** @brief The Peripheral Clock Frequency - 16MHz, to be set in CR2 bit [5:0] */
#define I2C_CR2_PERCF_5_0 0b010000

/*
 *  SR1/SR2 registers macros
 */
/** @brief Start Bit; bit 0 in SR1 register which is set once
 * start condition is generated. Cleared by reading SR1
 * followed by writing to the DR register.
 */
#define I2C_SR1_SB 0x1

/** @brief ADDR bit in SR1 if set indicates that the slave address was sent */
#define I2C_SR1_ADDR 0x02

/** @brief SR1 BTF bit, to check whether the byte transfer finished or not */
#define I2C_SR1_BTF (1<<2)

/** @brief I2C SR1 checking bit enable for TxE */
#define I2C_SR1_TXE (1 << 7)

/** @brief SR2 TRA bit, set when slave address is transmitted */
#define I2C_SR2_TRA (1<<2)

/** @brief SR2 MSL Bit is cleared after Stop condition is generated */
#define I2C_SR2_MSL (0x0001)


/** @brief method for initializing the I2C module
 * @param clock value
 */
void i2c_master_init(uint16_t clk) {
    struct rcc_reg_map *rcc = RCC_BASE;
    struct i2c_reg_map *i2c = I2C1_BASE;

    // setting PB8
    gpio_init(GPIO_B, 8, MODE_ALT, OUTPUT_OPEN_DRAIN, OUTPUT_SPEED_LOW,
              PUPD_NONE, ALT4);

    // setting the PB9
    gpio_init(GPIO_B, 9, MODE_ALT, OUTPUT_OPEN_DRAIN, OUTPUT_SPEED_LOW,
              PUPD_NONE, ALT4);

    // Setting the RCC I2C1 RCC clock enable for APB
    rcc->apb1_enr |= I2C_RCC_APB1ENR;

    // setting the peripheral clock frequency = 16MHz
    // Program the peripheral input clock in I2C_CR2
    // Register in order to generate correct timings
    i2c->CR2 |= I2C_CR2_PERCF_5_0;

    // Configure the clock control registers
    // Macro stored in i2c.h I2C_CCR_11_0
    i2c->CCR |= clk;

    // enable 7 bit addressing mode - default on reset

    // Configure the rise time registers
    i2c->TRISE = I2C_TRISE_5_0;

    // enable acknowledgment bit
    i2c->CR1 |= I2C_CR1_ACK;

    dbg_printk("Master I2C init\n clk = %u\n", clk);

    // Program the I2C_CR1 register to enable the peripheral
    i2c->CR1 |= I2C_CR1_PEN;
}

/**
 * @brief Method for sending a start condition on the SDA bus
 */
void i2c_master_start() {
    struct i2c_reg_map *i2c = I2C1_BASE;     // base address of I2C

    // set the START bit to initiate the START CONDITION
    i2c->CR1 |= I2C_CR1_STARTC;
    /* Once the Start condition is sent:
     * The SB bit is set by hardware and an interrupt is
     * generated if the ITEVFEN bit is set.
     */
    // wait for SB to be set indicating start condition started successfully
    while (!(i2c->SR1 & I2C_SR1_SB)) {
        /* wait */
    }
}

/** @brief Method for sending a stop condition on the SDA bus
 */
void i2c_master_stop() {
    struct i2c_reg_map *i2c = I2C1_BASE;

    // wait for TxE or BTF condition to be set to 1, then stop
    while ((!(i2c->SR1 & I2C_SR1_TXE)) && (!(i2c->SR1 & I2C_SR1_BTF))) {
        /* wait */
    }
    i2c->CR1 |= I2C_CR1_STOPC;

    // wait for the stop condition to be generated
    // and the interface to go back to slave mode
    while (i2c->SR2 & I2C_SR2_MSL) {
        /* wait */
    }
}

/**
 * Method for sending a slave address and then sending the data bits on the
 * SDA bus. Use individual helper methods if both steps are not required
 *
 * @param buf pointer to the buffer holding the data to be sent
 * @param len length of the data to be sent
 * @param slave_addr slave address
 * @return returns a success status (0) or fail if the data cannot be sent
 */
int i2c_master_write(uint8_t *buf, uint16_t len, uint8_t slave_addr) {

    i2c_master_send_slave_addr(slave_addr);    // send the slave address

    return i2c_master_send_data(buf, len);
}

/**
 * @brief sends the slave address on the bus
 * @param slave_addr
 */
void i2c_master_send_slave_addr(uint8_t slave_addr) {
    const uint8_t Transmit_Bit = 0xFE;

    struct i2c_reg_map *i2c = I2C1_BASE;

    // once start condition is triggered, read SR1
    // followed by writing to DR, which will clear the SB
    while (!(i2c->SR1 & I2C_SR1_SB)) {
        /* wait */
    }

    // send the slave address by writing in the Data register
    // with the write bit indicating transmission mode
    i2c->DR = (slave_addr) & Transmit_Bit;

    // As soon as the address byte is sent, the ADDR bit is set by HW
    // read SR1 & wait for the transmission of slave address;
    while (!(i2c->SR1 & I2C_SR1_ADDR)) {
        /* wait */
    }

    // read SR2; ADDR will be reset after the reading of SR2
    // SR2 must only be read if ADDR is set
    if (!(i2c->SR2 & I2C_SR2_MSL) && i2c->SR1 & I2C_SR1_ADDR) {
        printk("%s: Not in master mode while sending slave addr\n", __func__);
    }
}

/**
 * sends the given data buf, on the bus
 * @param buf The data to be sent
 * @param len The total length of the data
 */
int i2c_master_send_data(uint8_t *buf, uint16_t len) {
    const int LoopMax = 0xFFFFFF;

    struct i2c_reg_map *i2c = I2C1_BASE;
    int loops;

    if (len == 0)
        return (0);

    // wait for TxE to be set and write the data to DR
    loops = LoopMax;
    while (len > 0) {
        while (!(i2c->SR1 & I2C_SR1_TXE)) {
            if (--loops < 0) {
                printk("%s :btf timeout\n", __func__);
                return (1);
            }
        }
        i2c->DR = *buf++;
        len--;
    }
    return 0;
}


int i2c_master_read(uint8_t *buf, uint16_t len, uint8_t slave_addr) {
    (void) buf;
    (void) len;
    (void) slave_addr;

    return 0;
}