/**
 * @file  uart.c
 *
 * @brief Interrupt based serial UART functions    
 *
 * @date  10-Oct-19
 *
 * @author Aanand Nayyar (aanandn)
 */

#include <unistd.h>
#include <arm.h>
#include <rcc.h>
#include <gpio.h>
#include <uart_polling.h>
#include <uart.h>
#include <nvic.h>

#define UNUSED __attribute__((unused))

/** @brief The UART register map. */
struct uart_reg_map {
    volatile uint32_t SR;       /**< Status Register */
    volatile uint32_t DR;       /**<  Data Register */
    volatile uint32_t BRR;      /**<  Baud Rate Register */
    volatile uint32_t CR1;      /**<  Control Register 1 */
    volatile uint32_t CR2;      /**<  Control Register 2 */
    volatile uint32_t CR3;      /**<  Control Register 3 */
    volatile uint32_t GTPR;     /**<  Guard Time and Pre-scaler Register */
};

/** @brief Base address for UART2 */
#define UART2_BASE  (struct uart_reg_map *) 0x40004400

/** @brief Bits for UART Config register CR1 - m4_ref#551 19.6.4 */
#define UART_CR1_EN     (1 << 13)/**<  UART Enable */
#define UART_CR1_OVER8  (1 <<15) /**<  OVER8 clock mode */
#define UART_CR1_TE     (1 << 3) /**<  UART Transmitter Enable */
#define UART_CR1_RE     (1 << 2) /**<  UART Receiver Enable */
#define UART_CR1_TXEIE  (1 << 7) /**<  UART TxE Interrupt Enable */
#define UART_CR1_RXNEIE (1 << 5) /**<  UART RxNE Interrupt Enable */

/** @brief Status Bits for UART Status register SR pg 549 */
#define UART_SR_TXE     (1 << 7) /**<  UART Tx Data Empty */
#define UART_SR_RXNE    (1 << 5) /**<  UART Rx Data NotEmpty */

/** @brief for APB1 devices fpclk = sysclk = 16 Mhz
 * so BRR values (over8=0) at given baudrates from m4_ref#522 Table76
 */
#define UART_BAUD115200 0x008B
#define UART_BAUD57600  0x0116
#define UART_BAUD38400  0x01A1
#define UART_BAUD19200  0x0341
#define UART_BAUD9600   0x0683

/** @brief Bits for RCC_APB1ENR register see m4_ref#119 6.13.11 */
#define RCC_APB1ENR_UART2_CLKEN  (1 << 17) /**<  UART2 Clock Enable */
#define RCC_APB1RSTR_UART2_RESET (1 << 17) /**<  UART2 reset bit */

/** @brief UART2_IRQ is #38 */
#define UART2_IRQNUM    38

/** @brief circular bufferx w its head/tail vars */
#define rxBUFSIZE       128       /* size N must be = 2^k */
typedef struct {
    uint16_t cb_head;        /**<  head+1 is where u put char in buffer */
    uint16_t cb_tail;        /**<  head is where u get char from buffer */
    char cb_buffer[rxBUFSIZE]; /**<  the character buffer */
} rxCircularBuffer;

#define txBUFSIZE       1024      /* size N must be = 2^k */
typedef struct {
    uint16_t cb_head;        /**<  head+1 is where u put char in buffer */
    uint16_t cb_tail;        /**<  head is where u get char from buffer */
    char cb_buffer[txBUFSIZE]; /**<  the character buffer */
} txCircularBuffer;

rxCircularBuffer rxbuffer = {0};  /* The receive buffer */

txCircularBuffer txbuffer = {0};  /* The send buffer */

/**
 * @brief initializes UART to given baud rate, with 8-bit word length, 1 stop bit, 0 parity bits
 *
 * Enables clock, sets up PINs for RX/TX
 * Sets up baud rate : 115200, 57600, 38400, 19200, 9600 are supported
 * Sets 9600 as default if baud value specified is not one of above
 *
 * @param baud Baud rate
 */
void uart_init(UNUSED int baud) {
    struct rcc_reg_map *rcc = RCC_BASE;
    struct uart_reg_map *uart = UART2_BASE;
    int uart_div;

    /* configure UART2 TX pin which is PA_2 as output : stm32f401re.pdf pg#45 table 9 */
    gpio_init(GPIO_A, 2, MODE_ALT, OUTPUT_PUSH_PULL, OUTPUT_SPEED_LOW,
              PUPD_NONE, ALT7);

    /* configure UART2 RX pin which is PA_3 as input : stm32f401re.pdf pg#45 table 9 */
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

    uart->CR1 |= UART_CR1_TE; /* uart transmitter enable */
    uart->CR1 |= UART_CR1_RE; /* uart receiver enable */

    nvic_irq(UART2_IRQNUM, IRQ_ENABLE);

    uart->CR1 |= UART_CR1_RXNEIE; /* uart RxNE Int enable */
    /* uart TXE Int is enabled when the txbuffer has bytes to send */

    /* uart2 enable */
    uart->CR1 |= UART_CR1_EN;
}

/** @brief puts char at head while uart tx ISR picks char to tx frm tail
  * @param char to place in txbuffer
  */
int uart_put_byte(UNUSED char c) {
    struct uart_reg_map *uart = UART2_BASE;
    txCircularBuffer *txbp = &txbuffer;
    int rv = -1;

    disable_interrupts();

    uint16_t next = (txbp->cb_head + 1) & (txBUFSIZE - 1);
    if (txbp->cb_tail != next) { // txbuf not full
        uart->CR1 |= UART_CR1_TXEIE; // so TXE int enabled
        txbp->cb_buffer[txbp->cb_head] = c;
        txbp->cb_head = next;

        rv = 0;
    }

    enable_interrupts();
    return rv;
}

/** @brief gets char at head while uart rx ISR places recvd chars at tail
  * @param ptr to char to get from rxbuffer
  */
int uart_get_byte(UNUSED char *cp) {
    // struct uart_reg_map *uart = UART2_BASE;
    rxCircularBuffer *rxbp = &rxbuffer;
    int rv = -1;

    disable_interrupts();

    if (rxbp->cb_head != rxbp->cb_tail) { // rxbuf not empty
        *cp = rxbp->cb_buffer[rxbp->cb_tail++];
        rxbp->cb_tail &= (rxBUFSIZE - 1);
        rv = 0;
    }

    enable_interrupts();
    return rv;
}

/** @brief receives 1 character - fast before next rx interrupt 
 * and transmits upto 16 chars in polled mode per handout
 * before the next char tx.rx interrupt can come 
 * i.e 8N1 at 115200bps its 86.8uS/2 = 43.4uS
 * Note: when ISR is called IRQs of this and lower priority are disabled
 */
void uart_irq_handler() {
    struct uart_reg_map *uart = UART2_BASE;

    if ((uart->CR1 & UART_CR1_RXNEIE) && (uart->SR & UART_SR_RXNE)) {
        uint32_t c = uart->DR;
        rxCircularBuffer *rxbp = &rxbuffer;

        uint16_t next = (rxbp->cb_head + 1) & (rxBUFSIZE - 1);
        if (rxbp->cb_tail == next) { // rxbuf full
            /* for now just throwaway the new byte */
        } else {
            rxbp->cb_buffer[rxbp->cb_head] = c;
            rxbp->cb_head = next;
        }
    }

    if ((uart->CR1 & UART_CR1_TXEIE) && (uart->SR & UART_SR_TXE)) {
        txCircularBuffer *txbp = &txbuffer;
        if (txbp->cb_head == txbp->cb_tail) { // txbuf empty
            uart->CR1 &= ~UART_CR1_TXEIE; // so TXE int disabled
            return;
        }
        uart->DR = txbp->cb_buffer[txbp->cb_tail++];
        txbp->cb_tail &= (txBUFSIZE - 1);
    }
}

/** @begin flush the transmit buffer - first disables TXE int
  * and then uses polled mode output to flush bytes in txbuffer
  *
  * Not a good idea in the kernel - but ok for Lab3
  */
void uart_flush() {
    struct uart_reg_map *uart = UART2_BASE;
    txCircularBuffer *txbp = &txbuffer;

    uart->CR1 &= ~UART_CR1_TXEIE; // disable uartTx int

    while (txbp->cb_head != txbp->cb_tail) {
        while (!(uart->SR & UART_SR_TXE)) /* dowait */ ;
        uart->DR = txbp->cb_buffer[txbp->cb_tail++];
        txbp->cb_tail &= (txBUFSIZE - 1);
    }
}  
