/**
 * @file  uart_polling.c
 *
 * @brief Polled serial UART functions
 *
 * @date  8-Oct-19
 *
 * @author Aanand Nayyar (aanandn)
 */


#include <gpio.h>
#include <rcc.h>
#include <unistd.h>
#include <uart_polling.h>

/** @brief The UART register map. */
struct uart_reg_map {
    volatile uint32_t SR;   /**< Status Register */
    volatile uint32_t DR;   /**<  Data Register */
    volatile uint32_t BRR;  /**<  Baud Rate Register */
    volatile uint32_t CR1;  /**<  Control Register 1 */
    volatile uint32_t CR2;  /**<  Control Register 2 */
    volatile uint32_t CR3;  /**<  Control Register 3 */
    volatile uint32_t GTPR; /**<  Guard Time and Prescaler Register */
};

/** @brief Base address for UART2 */
#define UART2_BASE  (struct uart_reg_map *) 0x40004400

/** @brief Bits for UART Config register CR1 pg 551 sec 19.6.4 */
#define UART_EN (1 << 13)   /**<  UART Enable */
#define UART_OVER8 (1 << 15) /**<  OVER8 clock mode */
#define UART_TE (1 << 3)    /**<  UART Transmitter Enable */
#define UART_RE (1 << 2)    /**<  UART Receiver Enable */

/** @brief Status Bits for UART Status register SR pg 549 */
#define UART_TXE (1 << 7)
#define UART_RXNE (1 << 5)

/** @brief for APB1 devices fpclk = sysclk = 16 Mhz
 * so BRR values (over8=0) at given baudrates from m4_ref pg#522 Table76
 */
#define UART_BAUD115200 0x008B
#define UART_BAUD57600  0x0116
#define UART_BAUD38400  0x01A1
#define UART_BAUD19200  0x0341
#define UART_BAUD9600   0x0683

/** @brief Bits for RCC_APB1ENR register see pg 119 sec 6.13.11 */
#define RCC_APB1ENR_UART2_CLKEN  (1 << 17) /**<  UART2 Clock Enable */
#define RCC_APB1RSTR_UART2_RESET  (1 << 17)  /**<  UART2 reset bit */

/**
 * @brief initializes UART to given baud rate, with 8-bit word length, 1
 * stop bit, 0 parity bits
 *
 * Enables clock, sets up PINs for RX/TX
 * Sets up baud rate : 115200, 57600, 38400, 19200, 9600 are supported
 * Sets 9600 as default if baud value specified is not one of above
 *
 * @param baud Baud rate
 */
void uart_polling_init(int baud) {
    struct rcc_reg_map *rcc = RCC_BASE;
    struct uart_reg_map *uart = UART2_BASE;
    int uart_div;

    /* configure UART2 TX pin which is PA_2 as output : stm32f401re.pdf
     * pg#45 table 9 */
    gpio_init(GPIO_A, 2, MODE_ALT, OUTPUT_PUSH_PULL, OUTPUT_SPEED_LOW,
              PUPD_NONE, ALT7);

    /* configure UART2 RX pin which is PA_3 as input : stm32f401re.pdf
     * pg#45 table 9 */
    gpio_init(GPIO_A, 3, MODE_ALT, OUTPUT_OPEN_DRAIN, OUTPUT_SPEED_LOW,
              PUPD_NONE, ALT7);

    /* rcc uart2 clock enable */
    rcc->apb1_enr |= RCC_APB1ENR_UART2_CLKEN;

    /* rcc UART2 peripheral reset */
    rcc->apb1_rstr |= RCC_APB1RSTR_UART2_RESET;
    rcc->apb1_rstr &= ~RCC_APB1RSTR_UART2_RESET;

    /* for baud: OVER8 bit is 0 is default setting at hw-reset */
    /* uart clock divider value depending on baud */
    switch (baud) {
        case 115200:
            uart_div = UART_BAUD115200;
            break;

        case 57600:
            uart_div = UART_BAUD57600;
            break;

        case 38400:
            uart_div = UART_BAUD38400;
            break;

        case 19200:
            uart_div = UART_BAUD19200;
            break;

        default:
            uart_div = UART_BAUD9600;
    } /* switch */

    uart->BRR = uart_div;

    /* 1 start, 8 data (CR1), 1 stop bits (CR2) (pg 551) are
     * the default settings at hw-reset
     * so nothing be be done for this setting */

    uart->CR1 |= UART_TE; /* uart transmitter enable */
    uart->CR1 |= UART_RE; /* uart receiver enable */

    /* uart2 enable */
    uart->CR1 |= UART_EN;

    return;
}

/**
 * @brief transmits a byte over UART
 *
 * Waits till transmit register is empty & writes the 8-bit char to the port
 *
 * @param c character to be sent
 */
void uart_polling_put_byte(char c) {
    struct uart_reg_map *uart = UART2_BASE;

    while (!(uart->SR & UART_TXE)) /* dowait */ ;

    /* unsigned since bits 31-9 need to be zero as per pg 551 19.6.3 */
    uart->DR = (uint32_t) c;
    return;
}

/**
 * @brief receives a byte over UART
 *
 * Waits till a character has been received , reads it from the UART
 *
 * @returns the char read
 */
char uart_polling_get_byte() {
    struct uart_reg_map *uart = UART2_BASE;

    while (!(uart->SR & UART_RXNE)) /* dowait */ ;

    return (char) (uart->DR);
}