/**
 * @file   swi_stubs.S
 *
 * @brief  Stub functions for newlib and 349libc syscalls
 *
 * @date
 * @author
 */

.cpu cortex-m4
.syntax unified
.section .swi_stub
.thumb

#include "../../kernel/include/svc_num.h"

.global _start
_start:
  svc SVC_SCHD_START
  bx lr
  bkpt

.global _sbrk
_sbrk:
  svc SVC_SBRK
  bx lr
  bkpt

.global _write
_write:
  svc SVC_WRITE
  bx lr
  bkpt

.global _close
_close:
  svc SVC_CLOSE
  bx lr
  bkpt

.global _fstat
_fstat:
  svc SVC_FSTAT
  bx lr
  bkpt

.global _isatty
_isatty:
  svc SVC_ISATTY
  bx lr
  bkpt

.global _lseek
_lseek:
  svc SVC_LSEEK
  bx lr
  bkpt

.global _read
_read:
  svc SVC_READ
  bx lr
  bkpt

.global _gettimeofday
_gettimeofday:
  svc SVC_THR_TIME
  bx lr
  bkpt

.global _times
_times:
  svc SVC_TIME
  bx lr
  bkpt

.global _getpid
_getpid:
  svc SVC_GET_PID
  bx lr
  bkpt

.global _kill
_kill:
  svc SVC_KILL
  bx lr
  bkpt

.global _exit
_exit:
  svc SVC_EXIT
  bx lr
  bkpt

.global servo_enable
servo_enable:
  svc SVC_SERVO_ENABLE
  bx lr
  bkpt

.global servo_set
servo_set:
  svc SVC_SERVO_SET
  bx lr
  bkpt
