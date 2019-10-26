/** @file arm.h
 *
 *  @brief  Assembly wrappers for arm instructions.
 *
 *  @date   March 27, 2019
 *
 *  @author Ronit Banerjee <ronitb@andrew.cmu.edu>
 */
#ifndef _ARM_H_
#define _ARM_H_

#define intrinsic __attribute__( ( always_inline ) ) static inline

#include <stdint.h>

void init_349( void );

void enable_fpu( void );

/**
 * @brief      Asm wrapper for strex.
 *
 * @param      addr  Address to which you want to store exclusively.
 * @param[in]  val   TCalue to store to address.
 *
 * @return     Return 1 if successful else return 0.
 */
intrinsic uint32_t store_exclusive_register( uint32_t *addr, uint32_t val ) {
  uint32_t result;

  __asm volatile ( "strex %0, %1, [%2]" : "=r" ( result ) : "r" ( val ), "r" ( addr ) );
  return( result );
}

/**
 * @brief      Loads an exclusive register.
 *
 * @param      addr  The address
 *
 * @return     { description_of_the_return_value }
 */
intrinsic uint32_t load_exclusive_register( uint32_t *addr ) {
  uint32_t result;

  __asm volatile ( "ldrex %0, [%1]" : "=r" ( result ) : "r" ( addr ) );
  return( result );
}

/**
 * @brief      Enables the interrupts.
 */
intrinsic void enable_interrupts( void ) {
  __asm volatile( "CPSIE f" );
}

/**
 * @brief      Disables the interrupts.
 */
intrinsic void disable_interrupts( void ) {
  __asm volatile( "CPSID f" );
}

/**
 * @brief      Sets a breakpoint.
 */
intrinsic void breakpoint( void ) {
  __asm volatile( "bkpt" );
}

/**
 * @brief      DSB acts as a special data synchronization memory barrier.
 *             Instructions that come after the DSB, in program order, do not
 *             execute until the DSB instruction completes. The DSB
 *             instruction completes when all explicit memory accesses before
 *             it complete.
 */
intrinsic void data_sync_barrier( void ) {
  __asm volatile( "dsb" );
}

/**
 * @brief      Instruction Synchronization Barrier flushes the pipeline in the
 *             processor, so that all instructions following the ISB are
 *             fetched from cache or memory, after the instruction has been
 *             completed. It ensures that the effects of context altering
 *             operations, such as changing the ASID, or completed TLB
 *             maintenance operations, or branch predictor maintenance
 *             operations, as well as all changes to the CP15 registers,
 *             executed before the ISB instruction are visible to the
 *             instructions fetched after the ISB
 */
intrinsic void instruction_sync_barrier( void ) {
  __asm volatile( "isb" );
}

/**
 * @brief      Calls wfi, which is a hint instruction. It will either put the
 *             peocessor to sleep or spin.
 */
intrinsic void wait_for_interrupt( void ) {
  __asm volatile( "wfi" );
}

void pend_pendsv( void );

void clear_pendsv( void );

#undef intrinsic

#endif /* _ARM_H_ */
