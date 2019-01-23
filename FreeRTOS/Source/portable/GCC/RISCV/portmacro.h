/*
    FreeRTOS V8.2.3 - Copyright (C) 2015 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution and was contributed
    to the project by Technolution B.V. (www.technolution.nl,
    freertos-riscv@technolution.eu) under the terms of the FreeRTOS
    contributors license.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/


#ifndef PORTMACRO_H
#define PORTMACRO_H

#ifdef __cplusplus
extern "C" {
#endif

/*-----------------------------------------------------------
 * Port specific definitions.
 *
 * The settings in this file configure FreeRTOS correctly for the
 * given hardware and compiler.
 *
 * These settings should not be altered.
 *-----------------------------------------------------------
 */

#include <portconfig.h>

#if portMACHINE_KERNEL == 1
# define KERNELMODE  m
# define EPC         mepc
# define CAUSE       mcause
# define TVAL        mtval
# define BADADDR     mbadaddr
# define STATUS      mstatus
# define TSTATUS     mtstatus
# define TTCB        mttcb
# define STATUS_xPP  0x1800 /* MPP=3 */
# define STATUS_xPPhibit 12
# define STATUS_xPPlobit 11
# define STATUS_xIE  0x0080 /* MPIE=1, enable interrupts on first mret */
# define RET         mret
# define IE          mie
# define IP          mip
#elif portSUPERVISOR_KERNEL == 1
# define KERNELMODE  s
# define EPC         sepc
# define CAUSE       scause
# define TVAL        stval
# define BADADDR     sbadaddr
# define STATUS      sstatus
# define TSTATUS     ststatus
# define TTCB        sttcb
# define STATUS_xPP  0x0100 /* SPP=1 */
# define STATUS_xPPhibit 8
# define STATUS_xPPlobit 8
# define STATUS_xIE  0x0020 /* SPIE=1, enable interrupts on first sret */
# define RET         sret
# define IE          sie
# define IP          sip
#else
  #error "Unknown kernel config"
#endif

# define STATUS_PRV1 (STATUS_xPP | STATUS_xIE)

/* Syscall interface */

#define SYS_write 64
#define SYS_exit 93
#define SYS_timer 1234
#define SYS_yield 1235
#define SYS_priv 1236 /* elevate privileges from user mode */
#define SYS_timerset 1237 /* In case of machine-mode kernel, this is the only syscall handled by m-mode directly. It resets the timer */
#define SYS_interrupt 9999

/*-----------------------------------------------------------*/

#ifdef __ASSEMBLY__

#if __riscv_xlen == 64
# define STORE    sd
# define LOAD     ld
# define REGBYTES 8
# define ASMWORD   .dword
# define XALIGN 3
#else
# define STORE    sw
# define LOAD     lw
# define REGBYTES 4
# define ASMWORD   .word
# define XALIGN 2
#endif
/*-----------------------------------------------------------*/

#else /* not __ASSEMBLY__ */

#include <stdio.h>

/* Type definitions. */
#define portCHAR		char
#define portFLOAT		float
#define portDOUBLE		double
#define portLONG		long
#define portSHORT		short
#define portBASE_TYPE	long

#if __riscv_xlen == 64
	#define portSTACK_TYPE	uint64_t
	#define portPOINTER_SIZE_TYPE	uint64_t
#else
	#define portSTACK_TYPE	uint32_t
	#define portPOINTER_SIZE_TYPE	uint32_t
#endif

typedef portSTACK_TYPE StackType_t;
typedef long BaseType_t;
typedef unsigned long UBaseType_t;

#if( configUSE_16_BIT_TICKS == 1 )
	typedef uint16_t TickType_t;
	#define portMAX_DELAY ( TickType_t ) 0xffff
#else
	typedef uint32_t TickType_t;
	#define portMAX_DELAY ( TickType_t ) 0xffffffffUL
#endif
/*-----------------------------------------------------------*/

/* Architecture specifics. */
#define portSTACK_GROWTH			( -1 )
#define portTICK_PERIOD_MS			( ( TickType_t ) (1000 / configTICK_RATE_HZ) )
#if __riscv_xlen == 64
	#define portBYTE_ALIGNMENT	8
#else
	#define portBYTE_ALIGNMENT	4
#endif
#define portCRITICAL_NESTING_IN_TCB					1
/*-----------------------------------------------------------*/

/* Startup functions */

void vPortSetupHW( void );
void vPortSetupTimer( void );
void vPortInit( void );
/*-----------------------------------------------------------*/

/*
 * Used to issue syscalls
 */
long lPortSyscall(long num, long arg0, long arg1, long arg2);
long syscall(long num, long arg0, long arg1, long arg2);
unsigned long ulPortSyscallTrap(long cause, long epc, long tval, long regs[32]);
/*-----------------------------------------------------------*/

/* Scheduler utilities. */
void vPortYield( void );
void vPortExit(BaseType_t xExitCode);
void vPortSysTickHandler( void );

#define portYIELD()					vPortYield()

#define portASSERT( x ) \
  if( ( x ) == 0 ) { \
    xPortRaisePrivilege(); \
    portDISABLE_INTERRUPTS(); \
    printf("ASSERT ("#x") failed at %s:%d\n", __FILE__, __LINE__); \
    vPortExit(1111); \
  }
/*-----------------------------------------------------------*/

/* Critical section management. */
extern int vPortSetInterruptMask( void );
extern void vPortClearInterruptMask( int );
extern void vTaskEnterCritical( void );
extern void vTaskExitCritical( void );

#if portMACHINE_KERNEL == 1
  #define portDISABLE_INTERRUPTS()				__asm volatile 	( "csrc mstatus,0x8" )
  #define portENABLE_INTERRUPTS()					__asm volatile 	( "csrs mstatus,0x8" )
#elif portSUPERVISOR_KERNEL == 1
  #define portDISABLE_INTERRUPTS()				__asm volatile 	( "csrc sstatus,0x2" )
  #define portENABLE_INTERRUPTS()					__asm volatile 	( "csrs sstatus,0x2" )
#else
  #error "Unknown kernel config"
#endif
#define portENTER_CRITICAL()					vTaskEnterCritical()
#define portEXIT_CRITICAL()						vTaskExitCritical()
#define portSET_INTERRUPT_MASK_FROM_ISR()       vPortSetInterruptMask()
#define portCLEAR_INTERRUPT_MASK_FROM_ISR( uxSavedStatusValue )       vPortClearInterruptMask( uxSavedStatusValue )
/*-----------------------------------------------------------*/

/* Task function macros as described on the FreeRTOS.org WEB site. */
#define portTASK_FUNCTION_PROTO( vFunction, pvParameters ) void vFunction( void *pvParameters )
#define portTASK_FUNCTION( vFunction, pvParameters ) void vFunction( void *pvParameters )

#define portNOP() __asm volatile 	( " nop " )

#define portINLINE	__inline

#ifndef portFORCE_INLINE
	#define portFORCE_INLINE inline __attribute__(( always_inline))
#endif
/*-----------------------------------------------------------*/


#if ( portUSING_MPU_WRAPPERS == 1 )

/* MPU defines */
#define portMPU_REGION_READ             ( 1UL << 0 )
#define portMPU_REGION_WRITE            ( 1UL << 1 )
#define portMPU_REGION_READ_WRITE       ( portMPU_REGION_READ | portMPU_REGION_WRITE )
#define portMPU_REGION_EXECUTE          ( 1UL << 2 )
#define portMPU_REGION_UTRUSTED         ( 1UL << 3 )
#define portMPU_REGION_UACK             ( 1UL << 4 )
#define portMPU_REGION_STRUSTED         ( 1UL << 5 )

#define portTOTAL_NUM_REGIONS		        ( 4UL )
#define portNUM_CONFIGURABLE_REGIONS		( portTOTAL_NUM_REGIONS )

typedef struct MPU_REGION_REGISTERS
{
	portPOINTER_SIZE_TYPE ulRegionBaseAddress;
	portPOINTER_SIZE_TYPE ulRegionBoundAddress;
	portPOINTER_SIZE_TYPE ulRegionAttribute;
} xMPU_REGION_REGISTERS;

typedef struct MPU_SETTINGS
{
	xMPU_REGION_REGISTERS xRegion[ portTOTAL_NUM_REGIONS ];
} xMPU_SETTINGS;
/*-----------------------------------------------------------*/

/* MPU macros */
#if portMACHINE_KERNEL == 1
  /**
   * x ... MPU index
   * region ... region to load. Must be of type xMPU_REGION_REGISTERS
   */
  #define LOAD_MPU_SLOT(x, region) \
    do { \
      write_csr(mpmpbase##x, (region).ulRegionBaseAddress); \
      write_csr(mpmpbound##x, (region).ulRegionBoundAddress); \
      write_csr(mpmpflags##x, (region).ulRegionAttribute); \
    } while(0)
    
  #define CLEAR_MPU_SLOT(x) \
    do { \
      write_csr(mpmpbase##x, 0); \
      write_csr(mpmpbound##x, 0); \
      write_csr(mpmpflags##x, 0); \
    } while(0)

#elif portSUPERVISOR_KERNEL == 1

  /**
   * x ... MPU index
   * region ... region to load. Must be of type xMPU_REGION_REGISTERS
   */
  #define LOAD_MPU_SLOT(x, region) \
    do { \
      write_csr(spmpbase##x, (region).ulRegionBaseAddress); \
      write_csr(spmpbound##x, (region).ulRegionBoundAddress); \
      write_csr(spmpflags##x, (region).ulRegionAttribute); \
    } while(0)
    
  #define CLEAR_MPU_SLOT(x) \
    do { \
      write_csr(spmpbase##x, 0); \
      write_csr(spmpbound##x, 0); \
      write_csr(spmpflags##x, 0); \
    } while(0)

#else
  #error "Unknown kernel config"
#endif
/*-----------------------------------------------------------*/

/* Privilege handling functions/macros */
#if ( portMACHINE_KERNEL == 1 ) 

  #define portSWITCH_TO_USER_MODE() do { \
    register unsigned long temp;    \
    asm volatile (                  \
      "li   %0, %1\n"                 \
      "csrc mstatus, %0\n"          \
      "la   %0, 1f\n"                 \
      "csrw mepc, %0\n"             \
      "mret \n"                     \
      "1:   \n"                       \
      : "=r" (temp)                 \
      : "i" (STATUS_xPP));          \
    } while(0)

#elif ( portSUPERVISOR_KERNEL == 1 )

  #define portSWITCH_TO_USER_MODE() do { \
    register unsigned long temp;    \
    asm volatile (                  \
      "li   %0, %1\n"                 \
      "csrc sstatus, %0\n"          \
      "la   %0, 1f\n"                 \
      "csrw sepc, %0\n"             \
      "sret \n"                     \
      "1:   \n"                       \
      : "=r" (temp)                 \
      : "i" (STATUS_xPP));          \
    } while(0)

#endif

/*
 * Checks to see if being called from the context of an unprivileged task, and
 * if so raises the privilege level and returns false - otherwise does nothing
 * other than return true.
 */
extern BaseType_t xPortRaisePrivilege( void );

/* Elevates privileges to kernel mode */
extern long vPortElevatePriv( void );

/*
 * Loads MPU configuration into MPU
 */
void vPortLoadTaskMPUSettings( xMPU_SETTINGS *xMPUSettings );

/* Set the privilege level to user mode if xRunningPrivileged is false. */
portFORCE_INLINE static void vPortResetPrivilege( BaseType_t xRunningPrivileged )
{
  if( xRunningPrivileged != pdTRUE )
  {
    portSWITCH_TO_USER_MODE();
  }
}

#endif /* portUSING_MPU_WRAPPERS */

#endif /* no __ASSEMBLY__ */

#ifdef __cplusplus
}
#endif

#endif /* PORTMACRO_H */

