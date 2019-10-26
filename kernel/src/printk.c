/**
 * @file printk.c
 *
 * @brief      printf() implementation for KERNEL using UART
 *
 * @date       July 27 2015
 * @author     Aaron Reyes <areyes@andrew.cmu.edu>
 */

#include <unistd.h>
#include <stdarg.h>
#include <uart.h>
#include <uart_polling.h>


/**
 * allows for numbers with 64 digits/letters
 */
#define MAXBUF ( sizeof( uint32_t ) )

/**
 * static array of digits for use in printnum(s)
 */
static char digits[] = "0123456789abcdef";

void uart_wrapper ( char c ) {
//  uart_polling_put_byte( c );
   uart_put_byte( c );
}

/**
 * @brief      prints a number
 *
 * @param      base  8, 10, 16
 * @param      num   the number to print
 */
static void printnumk( uint8_t base, uint32_t num ) {
  int8_t *prefix = 0;
  int8_t buf[MAXBUF];
  int8_t *ptr = &buf[MAXBUF - 1];

  // standard radius prefixes
  if ( base == 8 ) {
    prefix = ( int8_t * )"0";
  }
  else if ( base == 16 ) {
    prefix = ( int8_t * )"0x";
  }

  // convert number to string in buffer
  do {
    *ptr-- = digits[num % base];
    num /= base;
  }
  while ( num != 0 );

  // print result
  if ( prefix ) {
    while ( *prefix ) {
      uart_wrapper( *prefix++ );
    }
  }

  while ( ++ptr != &buf[MAXBUF] ) {
    uart_wrapper( *ptr );
  }
}

/**
 * @brief      A kernel printf() function for debugging the kernel
 *
 * @param      fmt        the format string
 * @param[in]  <unnamed>  variadic input
 *
 * @return     0 on success or -1 on failure
 */
int printk( const char *fmt, ... ) {
  va_list args;
  // set up va_list and print it
  va_start( args, fmt );

  // loop through format string looking for formatting
  while ( *fmt ) {
    // handle normal characters
    if ( *fmt != '%' ) {
      uart_wrapper( *fmt++ );
      continue;
    }

    fmt++;

    // handle formatting
    switch ( *fmt ) {

    case 'd': { // signed decimal
      int32_t num = va_arg( args, int32_t );

      if ( num < 0 ) {
        uart_wrapper( '-' );
        printnumk( 10, -num );
      }
      else {
        printnumk( 10, num );
      }

      break;
    }

    case 'u': { // unsigned decimal
      uint32_t num = va_arg( args, uint32_t );
      printnumk( 10, num );
      break;
    }

    case 'o': { // octal
      uint32_t num = va_arg( args, uint32_t );
      printnumk( 8, num );
      break;
    }

    case 'x': // hex
    case 'p': { // pointer
      uint32_t num = va_arg( args, uint32_t );
      printnumk( 16, num );
      break;
    }

    case 's': { // string
      int8_t *byte_ptr = ( int8_t * )va_arg( args, int32_t );

      while ( *byte_ptr ) {
        uart_wrapper( *byte_ptr );
        byte_ptr++;
      }

      break;
    }

    case 'c': { // character
      int32_t byte = va_arg( args, int32_t );
      uart_wrapper( byte );
      break;
    }

    case '%': { // escaped percent symbol
      uart_wrapper( '%' );
      break;
    }

    default: { // error
      va_end( args );
      return -1;
    }
    }

    fmt++;
  }

  va_end( args );
  return 0;
}
