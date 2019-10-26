/**
 * @file   nvic.c
 *
 * @brief  NVIC functions
 *
 * @date   August 28 2015
 * @author Kevin DeVincentis
 */

#include <nvic.h>

void nvic_irq( uint8_t irq_num, uint8_t status ) {
  uint8_t shift_num = irq_num % NVIC_REG_SIZE;
  uint8_t reg_num = irq_num / NVIC_REG_SIZE;
  struct nvic_t *nvic;

  if ( status == IRQ_ENABLE ) {
    nvic = NVIC_ISER_BASE;
  }
  else if ( status == IRQ_DISABLE ) {
    nvic = NVIC_ICER_BASE;
  }
  else {
    return;
  }

  nvic->reg[reg_num] |= ( 0x1 << shift_num );

  return;
}

void nvic_clear_pending( uint8_t irq_num ) {
  uint8_t shift_num = irq_num % NVIC_REG_SIZE;
  uint8_t reg_num = irq_num / NVIC_REG_SIZE;
  struct nvic_t *nvic = NVIC_ICPR_BASE;

  nvic->reg[reg_num] |= ( 0x1 << shift_num );
}