/**
 * @file   nvic.h
 *
 * @brief  NVIC functions
 *
 * @date   August 28 2015
 * @author Kevin DeVincentis
 */
 #ifndef _NVIC_H_
#define _NVIC_H_

#include <unistd.h>

struct nvic_t {
  volatile uint32_t reg[3];
};

#define NVIC_ISER_BASE (struct nvic_t *) 0xE000E100
#define NVIC_ICER_BASE (struct nvic_t *) 0xE000E180
#define NVIC_ICPR_BASE (struct nvic_t *) 0xE000E280
#define NVIC_REG_SIZE 32
#define IRQ_ENABLE 1
#define IRQ_DISABLE 0

void nvic_irq( uint8_t irq_num, uint8_t status );
void nvic_clear_pending( uint8_t irq_num );

#endif //_NVIC_H
