/**
 * @file   debug.h
 *
 * @brief  Debugging routines.
 *
 * @date   August 28 2015
 * @author Ronit Banerjee <ronitb@andrew.cmu.edu>
 */

#ifndef _DEBUG_H_
#define _DEBUG_H_

#include <arm.h>
#include <printk.h>

#ifdef DEBUG

/**
 * @brief      When the condition X is false, a breakpoint is generated.
 *
 * @param      X     Condtion to test
 */
#define ASSERT( X ){\
  if ( !(X) ){\
    breakpoint();\
  }\
}

/**
 * @brief      Prints an error along with the function name.
 *
 * @param      ...   Variadic inputs for printk.
 */
#define DEBUG_PRINT( ... ){\
  printk( "%s: ", __func__);\
  printk( __VA_ARGS__ );\
}

/**
 * @brief      Prints an error message when the condition is not met.
 *
 * @param      X     Condtion to test.
 * @param      ...   Variadic inputs for printk.
 */
#define WARN( X, ... ){\
  if ( !(X) ){\
    DEBUG_PRINT( __VA_ARGS__ );\
  }\
}

#else

/**
 * @brief      Stub for when flag is not passed.
 *
 * @param      ...   Variadic unused
 */
//{@
#define ASSERT( ... ) do {} while( 0 );
#define DEBUG_PRINT( ... ) do {} while( 0 );
#define WARN( ... ) do {} while( 0 );
//@}

#endif /* DEBUG */

#endif /* _DEBUG_H_ */
