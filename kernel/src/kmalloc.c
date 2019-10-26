/**
 * @file kmalloc.c
 *
 * @brief      Implementation of heap management library. This is a very
 *             simple library, it keeps a struct for tracking its state. Each
 *             kmalloc struct takes a heap start and end pointer. Heaps are
 *             defined in the linker script. Each kmalloc struct can be
 *             initilized to do either aligned or unaligned allocations.
 *
 *             Aligned allocations - In this, the size of all the allocations
 *             must be set in kmalloc init. Calling k_malloc_aligned will give
 *             you a region of whatever size you initilized kmalloc to. To
 *             perform frees on aligned regions, we just add the regions to a
 *             linked list. In the next allocation, we check the linked list
 *             before moving the break pointer.
 *
 *             In unaligned allocations, the caller may specify the size they
 *             want. You do not need to support frees on unaligned regions.
 *             
 * @date       Febuary 12, 2019
 *
 * @author     Ronit Banerjee <ronitb@andrew.cmu.edu>
 */

#include "kmalloc.h"

#include <debug.h>
#include <unistd.h>

#define UNUSED __attribute__((unused))

/**
 * @brief      Initiliazes the kmalloc structure.
 *
 * @param[in]  internals        The internals
 * @param[in]  heap_low         The heap low
 * @param[in]  heap_top         The heap top
 * @param[in]  stack_size       The stack size
 * @param[in]  unaligned_alloc  This flags sets whether you can perfrom
 *                              unaligned allocations. If this flag is set,
 *                              then the stack size variable is ignored.
 *
 * @return     Returns 0 if allocation was successful, or -1 otherwise.
 */
void k_malloc_init( UNUSED kmalloc_t* internals,
                    UNUSED char* heap_low,
                    UNUSED char* heap_top,
                    UNUSED uint32_t stack_size,
                    UNUSED uint32_t unaligned ){

}

/**
 * @brief      This function lets you perform unaligned allocations. If the
 *             unaligned flag has not be set, you should fall into a
 *             breakpoint look at debug.h to see how to do this.
 *
 * @param[in]  kmalloc_t  The kmalloc t
 * @param[in]  size       The allocation size.
 *
 * @return     Returns the pointer to the allocated buffer.
 */
void* k_malloc_unaligned( UNUSED kmalloc_t* internals,
                          UNUSED uint32_t size ){
  return NULL;
}

/**
 * @brief      This function performs aligned allocations. 
 *
 * @param[in]  internals  The internals structure.
 *
 * @return     Pointer to allocated buffer, can be NULL.
 */
void* k_malloc_aligned( UNUSED kmalloc_t* internals ){
  return NULL;
}

/**
 * @brief      This function allows you to free aligned chunks.
 *
 * @param[in]  internals  The internals structure.
 * @param[in]  buffer      Pointer to a buffer that was obtained.
 *
 * @warning    The pointer to the buffer must be the original pointer that was
 *             obtained from k_malloc_aligned. For example, if you are using
 *             the buffer as a stack it must be the orignial pointer you
 *             obtained and not the current stack position.
 */
void k_free( UNUSED kmalloc_t* internals, UNUSED void* buffer ){

}
