/**
 * @file timer.c
 *
 * @brief   system timer sysTick interrupt handler & start/stop
 *
 * @date    9-Oct-19   
 *
 * @author  Aanand Nayyar (aanandn)    
 */

#include <timer.h>
#include <unistd.h>

#include <rcc.h>
#include <printk.h>
#include <syscall.h>

#define UNUSED __attribute__((unused))

/** @brief The system timer sysTick register map m4_prog#246 */
struct systick_reg_map {
    volatile uint32_t cr;       /**<  Control & Status Register */
    volatile uint32_t reload;   /**<  reLoad value Register */
    volatile uint32_t val;      /**<  current value Register */
    volatile uint32_t calib;    /**<  calibration properties Reguster */
};

/** @brief Base address for system timer sysTick */
#define SYSTICK_BASE  (struct systick_reg_map *) 0xE000E010

/** @brief Bits for sysTick Control/status register  m4_prog#247 */
#define SYSTICK_CR_EN       (1)     /**<  sysTick Enable */
#define SYSTICK_CR_INTEN    (1 <<1) /**<  sysTick interupt enable */
#define SYSTICK_CR_CLKAHB   (1 <<2) /**<  sysTick clock = AHB clock */
#define SYSTICK_CR_COUNT    (1<<16) /**<  Count status flag */

/** @brief Bits for sysTick Calib register  m4_prog#250 */
#define SYSTICK_CALIB_NOREF (1<<31) /**<  no seperate reference clock */
#define SYSTICK_CALIB_SKEW  (1<<30) /**<  ????skew??? */

/** @brief value in lower 24 bits of load/cur/calib val in reload/val/calib registers */
#define SYSTICK_VAL_MASK    0xFFFFFF

/** @brief Bits for RCC clock control register m4_ref#103 */
#define RCC_CCR_HSION       (1)
#define RCC_CCR_HSIRDY      (1<<1)

#define CLOCKFREQ_AHB       16000000 /**<  AHB clock freq = 16 Mhz */

/** @brief starts timer with systick frequency specifed in Hz
 * @param frequency  specifies systick frequency in Hz
 */
int timer_start(UNUSED int frequency){
    struct rcc_reg_map *rcc = RCC_BASE;
    struct systick_reg_map *systick = SYSTICK_BASE;
    
    /* rcc selects HSI clock as the systick clock at reset m4_ref#98 */
    /* so nothing to configure in RCC */
    
    /* wait for HSI clock to be stable */
    while (!( rcc->cr & RCC_CCR_HSIRDY ))    {
        /* do wait */
    }
    
    systick->reload = (CLOCKFREQ_AHB/frequency);  /* for 'frequency' Hz ticks - chkpt 1 */
    
    systick->cr |= SYSTICK_CR_CLKAHB; // sys timer clock = AHB clock 
    systick->cr |= SYSTICK_CR_INTEN;
    systick->cr |= SYSTICK_CR_EN;
    
    return 0;
}

void timer_stop(){
    struct systick_reg_map *systick = SYSTICK_BASE;
    
    systick->cr &= ~SYSTICK_CR_INTEN;
    systick->cr &= ~SYSTICK_CR_EN;
}

void systick_c_handler(){
    static int count = 0;
    
    count += 1;
	sys_servo_position();
}
