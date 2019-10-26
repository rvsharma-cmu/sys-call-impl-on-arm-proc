/* boot.s
 * Assembler startup file for the STM32
 * Ford Seidel  2019-01-31
 * Ronit Banerjee
 *
 * Updated: Aanand Nayyar  9-Oct-19
 */

# The Cortex M4 is a thumb only processor

    .cpu cortex-m4
    .syntax unified
    .section .ivt
    .thumb

    .global ivt
ivt:
    .word   __msp_stack_top     /* stack top address */
    .word   _reset_             /* 1 Reset */
    .word   _nmi_               /* 2 NMI */
    .word   _hard_fault_        /* 3 Hard Fault */
    .word   _mm_fault_          /* 4 MM Fault */
    .word   _bus_fault_         /* 5 Bus Fault */
    .word   _usage_fault_       /* 6 Usage Fault */
    .word   spin                /* 7 RESERVED */
    .word   spin                /* 8 RESERVED */
    .word   spin                /* 9 RESERVED*/
    .word   spin                /* 10 RESERVED */
    .word   _svc_asm_handler_   /* 11 SV call */
    .word   spin                /* 12 Debug reserved */
    .word   spin                /* 13 RESERVED */
    .word   _pend_sv_           /* 14 PendSV */
    .word   _sys_tick_asm_      /* 15 SysTick */
    .word   spin                /* 16 IRQ0 Window Watchdog Interrupt */
    .word   spin                /* 17 IRQ1 PVD */
    .word   spin                /* 18 IRQ2 TAMPER */
    .word   spin                /* 19 IRQ3 RTC   */
    .word   spin                /* 20 IRQ4 FLASH   */
    .word   spin                /* 21 IRQ5 RCC */
    .word   spin                /* 22 IRQ6 EXTI0 */
    .word   spin                /* 23 IRQ7 EXTI1  */
    .word   spin                /* 24 IRQ8 EXTI2   */
    .word   spin                /* 25 IRQ9 EXTI3 */
    .word   spin                /* 26 IRQ10 EXTI4 */
    .word   spin                /* 27 IRQ11 DMA1_Channel1   */
    .word   spin                /* 28 IRQ12 DMA1_Channel2   */
    .word   spin                /* 29 IRQ13 DMA1_Channel3 */
    .word   spin                /* 30 IRQ14 DMA1_Channel4 */
    .word   spin                /* 31 IRQ15 DMA1_Channel5   */
    .word   spin                /* 32 IRQ16 DMA1_Channel6   */
    .word   spin                /* 33 IRQ17 DMA1_Channel7 */
    .word   spin                /* 34 IRQ18 ADC1_2 */
    .word   spin                /* 35 IRQ19 CAN1_TX   */
    .word   spin                /* 36 IRQ20 CAN1_TX0   */
    .word   spin                /* 37 IRQ21 CAN1_RX1 */
    .word   spin                /* 38 IRQ22 CAN1_SCE */
    .word   spin                /* 39 IRQ23 EXTI9_5   */
    .word   spin                /* 40 IRQ24 TIM1_BRK   */
    .word   spin                /* 41 IRQ25 TIM1_UP */
    .word   spin                /* 42 IRQ26 TIM1_TRG_COM */
    .word   spin                /* 43 IRQ27 TIM1_CC   */
    .word   spin                /* 44 IRQ28 TIM2   */
    .word   spin                /* 45 IRQ29 TIM3 */
    .word   spin                /* 46 IRQ30 TIM5 */
    .word   spin                /* 47 IRQ31 I2C1_EV   */
    .word   spin                /* 48 IRQ32 I2C1_ER   */
    .word   spin                /* 49 IRQ33 I2C2_EV */
    .word   spin                /* 50 IRQ34 I2C2_ER */
    .word   spin                /* 51 IRQ35 SPI1   */
    .word   spin                /* 52 IRQ36 SPI2   */
    .word   spin                /* 53 IRQ37 USART1 */
    .word   _uart2_asm_         /* 54 IRQ38 USART2 */
    .word   spin                /* 55 IRQ39 USART3   */
    .word   spin                /* 56 IRQ40 EXTI15_10   */
    .word   spin                /* 57 IRQ41 RTCAlarm */
    .word   spin                /* 58 IRQ42 OTG_FS_WKUP */
    .word   spin                /* 59 IRQ43 RESERVED   */
    .word   spin                /* 60 IRQ44 RESERVED   */
    .word   spin                /* 61 IRQ45 RESERVED */
    .word   spin                /* 62 IRQ46 RESERVED */
    .word   spin                /* 63 IRQ47 RESERVED   */
    .word   spin                /* 64 IRQ48 RESERVED   */
    .word   spin                /* 65 IRQ49 RESERVED */
    .word   spin                /* 66 IRQ50 TIM5 */
    .word   spin                /* 67 IRQ51 SPI3   */
    .word   spin                /* 68 IRQ52 UART4   */
    .word   spin                /* 69 IRQ53 UART5 */
    .word   spin                /* 70 IRQ54 TIM6 */
    .word   spin                /* 71 IRQ55 TIM7   */
    .word   spin                /* 72 IRQ56 DMA2_Channel1   */
    .word   spin                /* 73 IRQ57 DMA2_Channel2 */
    .word   spin                /* 74 IRQ58 DMA2_Channel3 */
    .word   spin                /* 75 IRQ59 DMA2_Channel4   */
    .word   spin                /* 76 IRQ60 DMA2_Channel5   */
    .word   spin                /* 77 IRQ61 ETH */
    .word   spin                /* 78 IRQ62 ETH_WKUP */
    .word   spin                /* 79 IRQ63 CAN2_TX   */
    .word   spin                /* 80 IRQ64 CAN2_RX0  */
    .word   spin                /* 81 IRQ65 CAN2_RX1 */
    .word   spin                /* 82 IRQ66 CAN2_SCE */
    .word   spin                /* 83 IRQ67 OTG_FS   */

    .section .text

    .thumb_func
    .global _reset_
_reset_:
    ldr   r0, = _erodata
    ldr   r1, =_k_data
    ldr   r2, =_data_size

    cmp   r2, #0
    beq   bss_alloc

copy_flash:
    ldrb   r4, [r0], #1
    strb   r4, [r1], #1
    subs   r2, r2, #1
    bne    copy_flash

bss_alloc:
    ldr   r0, =_bss_start
    ldr   r1, =_bss_size

    cmp   r1, #0
    beq   branch_km

    mov   r4, #0

    zero_bss:
    strb  r4, [r0], #1
    subs  r1, r1, #1
    bne   zero_bss

branch_km:
    bl kernel_main
    /* ??? what more code here */
    bkpt

/* M4 SYS-TICK interrupt handler */
    .thumb_func
    .global _sys_tick_asm_
_sys_tick_asm_:  /* for SYS-TICK int vector jumps here */
    b systick_c_handler
    bkpt        /* just a marker - never comes here */

/* M4 UART2 interrupt handler */
    .thumb_func
    .global _uart2_asm_
_uart2_asm_:    /* for UART2 interupt vector jumps here */
    b uart_irq_handler
    bkpt        /* just a marker - never comes here */

/* M4 SuperVisor Call handler */
     .thumb_func
    .global _svc_asm_handler_
_svc_asm_handler_:
    /* ???  code for system calls here */
    mrs r0, PSP
	mov r1, r0
	ldr r0, [r0, #24]
	ldrb r0, [r0, #-2]
    b svc_c_handler

    .thumb_func
_nmi_ :
    bkpt

    .thumb_func
_hard_fault_ :
    bkpt

    .thumb_func
_mm_fault_:
    bkpt

    .thumb_func
_bus_fault_ : 
    bkpt

    .thumb_func
_usage_fault_ : 
    bkpt

    .thumb_func
_pend_sv_ :
    bkpt

    .thumb_func
_spi1_handler:
    bkpt

     .thumb_func
spin:
    bkpt

