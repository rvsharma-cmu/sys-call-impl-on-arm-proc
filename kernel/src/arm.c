/**
 * @file arm.c
 *
 * @brief      Implementation of asm wrappers
 * @date       5/23/2019
 *
 * @author     Ronit Banerjee <ronitb@andrew.cmu.edu>
 */

#include "arm.h"

#include <rcc.h>
#include <stdint.h>

/* @brief Configuration and Control Register */
#define CCR ((volatile uint32_t *) 0xE000ED14)
/* @brief Refister to enable/disable fpu */
#define CPACR ((volatile uint32_t *) 0xE000ED88)
/* @brief FPU control data */
#define FPCCR ((volatile uint32_t *) 0xE000EF34)
/* @brief Interrupt COntrl and State Register */
#define ICSR ((volatile uint32_t *) 0xE000ED04)

#define SHPR2 ((volatile uint32_t *) 0xE000ED1C)


/**
 * @brief      Disables stack alignement.
 */
void init_349( void ){
  // stack alingment
  *CCR       &= ~(0x1 << 9);
  *SHPR2 = 0xFF000000;

  data_sync_barrier();
  instruction_sync_barrier();
}

/**
 * @brief      Enables the fpu.
 */
void enable_fpu( void ){
  *CPACR |= (0xF << 20);
  *FPCCR |= (0x1 << 31);
  data_sync_barrier();
  instruction_sync_barrier();
}

/**
 * @brief      Pends a pendsv.
 */
void pend_pendsv( void ){
  *ICSR |= (1 << 28);
}

/**
 * @brief      Clears pendsv.
 */
void clear_pendsv( void ){
  *ICSR |= (1 << 27);
}