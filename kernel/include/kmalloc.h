/**
 * @file kmalloc.h
 *
 * @brief      Header for heap management library.
 *
 *             To use, allocate a struct for kmalloc_t, then initialise
 *             with heap end and top.
 *
 * @date       Febuary 12, 2019
 *
 * @author     Ronit Banerjee <ronitb@andrew.cmu.edu>
 */

#ifndef _KMALLOC_H_
#define _KMALLOC_H_

#include <stdint.h>

/**
 * @brief      Linked list struture to track free blocks.
 */
typedef struct list_node{
  struct list_node* next;
}list_node;

/**
 * @brief      Keeps track of current heap end and top, free blocks and
 *             whether you have done any unaligned alocations.
 */
typedef struct kmalloc_t {
  list_node* free_node;
  /* You might want to add some other things here */
}kmalloc_t;

void k_malloc_init( kmalloc_t* internals,
                   char* heap_low,
                   char* heap_top,
                   uint32_t stack_size,
                   uint32_t unaligned );

void* k_malloc_unaligned( kmalloc_t* internals,
                          uint32_t size );

void* k_malloc_aligned( kmalloc_t* internals );

void k_free( kmalloc_t* internals, void* buffer );

#endif /* _KMALLOC_H_ */
