/**
 * @file 
 *
 * @brief      
 *
 * @date       
 *
 * @author     
 */

#ifndef _UART_H_
#define _UART_H_

void uart_init(int baud);

int uart_put_byte(char c);

int uart_get_byte(char *c);

void uart_flush();

#endif /* _UART_H_ */
