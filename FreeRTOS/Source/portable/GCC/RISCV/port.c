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

/*-----------------------------------------------------------
 * Implementation of functions defined in portable.h for the RISC-V port.
 *----------------------------------------------------------*/

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "portmacro.h"
#include "encoding.h"
#include "clib.h"
#include <stdint.h>
#include <string.h>

#if ( portUSING_TM == 1 )
#include "enclave.h"
#endif

/* A variable is used to keep track of the critical section nesting.  This
variable has to be stored as part of the task context and must be initialised to
a non zero value to ensure interrupts don't inadvertently become unmasked before
the scheduler starts.  As it is stored as part of the task context it will
automatically be set to 0 when the first task is started. */
static UBaseType_t uxCriticalNesting = 0xaaaaaaaa;

/* Contains context when starting scheduler, save all 31 registers */
#ifdef __gracefulExit
BaseType_t xStartContext[31] = {0};
#endif

/*
 * As of version 1.10, these addresses reflect the configuration of the 
 * emulated SoC of the Spike RISC-V emulator with respect to the
 * Machine Timer Registers mtime and mtimecmp
 */
volatile uint64_t* mtime =      (uint64_t*)(CLINT_BASE + 0xbff8);
volatile uint64_t* timecmp =    (uint64_t*)(CLINT_BASE + 0x4000);

/* HTIF  interface */
volatile uint64_t tohost __attribute__((aligned(64)));
volatile uint64_t fromhost __attribute__((aligned(64)));

/* debug buffer */
static char cBuffer[ 1024 ];

/*
 * Set the next interval for the timer
 */
static void prvSetNextTimerInterrupt( void );

/*
 * Used to catch tasks that attempt to return from their implementing function.
 */
static void prvTaskExitError( void );

/*
 * Used to initialize hardware.
 */
static void prvPortSetupHW( void );

/*
 * Used to initialize the MPU.
 */
static void prvSetupMPU( void );

/*-----------------------------------------------------------*/

/* Sets the next timer interrupt
 * Reads current timer register and adds tickrate
 * Does nothing if a Clint was not found in the hardware configuration string
 * Using previous timer compare may fail if interrupts were disabled for a long time,
 * which is likely for the very first interrupt. When that happens, compare timer + 
 * tickrate may already be behind current timer and prevent correctly programming
 * the 2nd interrupt
 */
static void prvSetNextTimerInterrupt(void)
{
    /* Both, machine and supervisor mode can directly write timecmp */
    if (mtime && timecmp) 
        *timecmp = *mtime + (configTICK_CLOCK_HZ / configTICK_RATE_HZ);

#if portSUPERVISOR_KERNEL == 1
    /* Since supervisor timer interrupt is delegated from machine mode
     * we need to manually reset mip.STIP flag and enable machine-mode
     * timer interrupts again (mie.MTIE).
     * Both can only be done by machine mode
     */
	lPortSyscall(SYS_timerset, 0, 0, 0);
#endif
}
/*-----------------------------------------------------------*/

/* Sets and enable the timer interrupt */
void vPortSetupTimer(void)
{
#if ( configUSE_PREEMPTION == 1 )
	prvSetNextTimerInterrupt();

	/* Enable timer interupt */
#if portSUPERVISOR_KERNEL == 1
	__asm volatile("csrs sie, %0" : : "r"(0x20));
#elif portMACHINE_KERNEL == 1
	__asm volatile("csrs mie, %0" : : "r"(0x80));
#else
  #error "Unknown kernel config"
#endif
#endif
}
/*-----------------------------------------------------------*/

void vPortSysTickHandler( void )
{
	prvSetNextTimerInterrupt();

	/* Increment the RTOS tick. */
	if( xTaskIncrementTick() != pdFALSE )
	{
		vTaskSwitchContext();
	}
}
/*-----------------------------------------------------------*/

static void prvTaskExitError( void )
{
	/* A function that implements a task must not exit or attempt to return to
	its caller as there is nothing to return to.  If a task wants to exit it
	should instead call vTaskDelete( NULL ).

	Artificially force an assert() to be triggered if configASSERT() is
	defined, then stop here so application writers can catch the error. */
	configASSERT( uxCriticalNesting == ~0UL );
	//portDISABLE_INTERRUPTS(); does not work from user mode
	for( ;; );
}
/*-----------------------------------------------------------*/

/* Clear current interrupt mask and set given mask */
void vPortClearInterruptMask(int mask)
{
#if portMACHINE_KERNEL == 1
	__asm volatile("csrw mie, %0"::"r"(mask));
#elif portSUPERVISOR_KERNEL == 1
	__asm volatile("csrw sie, %0"::"r"(mask));
#else
  #error "Unknown kernel config"
#endif
}
/*-----------------------------------------------------------*/

/* Set interrupt mask and return current interrupt enable register */
int vPortSetInterruptMask(void)
{
	int ret;
	int mask = -1;
#if portMACHINE_KERNEL == 1
	__asm volatile("csrr %0,mie":"=r"(ret));
	__asm volatile("csrc mie,%0"::"r"(mask));
#elif portSUPERVISOR_KERNEL == 1
	__asm volatile("csrr %0,sie":"=r"(ret));
	__asm volatile("csrc sie,%0"::"r"(mask));
#else
  #error "Unknown kernel config"
#endif
	return ret;
}
/*-----------------------------------------------------------*/

static void prvPortSetupHW(void)
{
#if portUSING_MPU_WRAPPERS == 1
  /* Configure the regions in the MPU that are common to all tasks. */
  prvSetupMPU();
#endif
}
/*-----------------------------------------------------------*/

extern int main(void);

/* Starts main function. */
void vPortInit(void)
{
	prvPortSetupHW();
	int ret = main();
	exit(ret);
}
/*-----------------------------------------------------------*/

/*
 * See header file for description.
 */
#if portUSING_MPU_WRAPPERS == 1
  StackType_t *pxPortInitialiseStack( StackType_t *pxTopOfStack, TaskFunction_t pxCode, void *pvParameters, BaseType_t xRunPrivileged )
#else
  StackType_t *pxPortInitialiseStack( StackType_t *pxTopOfStack, TaskFunction_t pxCode, void *pvParameters )
#endif
{
	/* Simulate the stack frame as it would be created by a context switch
	interrupt. */
	register int *tp asm("x3");
#if portUSING_MPU_WRAPPERS == 1
	/* TODO: save privilege directly in TCB instead of stack. 
	 * Currently, a stack corruption could elevate privileges
	 */
	pxTopOfStack--;
	if (xRunPrivileged) {
	  *pxTopOfStack = (portSTACK_TYPE)STATUS_xPP; /* kernel mode */
	} else {
	  *pxTopOfStack = (portSTACK_TYPE)0; /* user mode */
	}
#endif
	pxTopOfStack--;
	*pxTopOfStack = (portSTACK_TYPE)pxCode;			/* Start address */
	pxTopOfStack -= 22;					/* Register x11-x31 */
	*pxTopOfStack = (portSTACK_TYPE)pvParameters;		/* Register x10 = a0 */
	pxTopOfStack -= 6;
								/* Register x9 = s1 */
								/* Register x8 = s0 */
								/* Register x7 = t2 */
								/* Register x6 = t1 */
								/* Register x5 = t0 */
	*pxTopOfStack = (portSTACK_TYPE)tp; 			/* Register x4 = tp (thread pointer) */
	pxTopOfStack -= 3;
								/* Register x3 = gp */
								/* Register x2 = sp */
	*pxTopOfStack = (portSTACK_TYPE)prvTaskExitError;	/* Register x1 = ra */
	return pxTopOfStack;
}
/*-----------------------------------------------------------*/

static uint64_t zeroExtend(long val)
{
	uint64_t ret = val;
	#if __riscv_xlen == 32
		ret = (0x00000000ffffffff & val);
	#endif
	return ret;
}

/* Relay syscall to host */
static uint64_t prvPortToHost(long which, long arg0, long arg1, long arg2)
{
	volatile uint64_t magic_mem[8] __attribute__((aligned(64)));

	magic_mem[0] = zeroExtend(which);
	magic_mem[1] = zeroExtend(arg0);
	magic_mem[2] = zeroExtend(arg1);
	magic_mem[3] = zeroExtend(arg2);
	__sync_synchronize();
	tohost = (uint64_t)zeroExtend(magic_mem); /* necessary for 32-bit */
	return magic_mem[0];
}
/*-----------------------------------------------------------*/

/* Exit systemcall */
static void prvPortSyscallExit(long code)
{
	uint64_t zcode = zeroExtend(code);
	tohost = ((zcode << 1) | 1);
	for(;;) { }
}
/*-----------------------------------------------------------*/

const char * const riscv_gpr_names_abi[] = {
  "zero", "ra", "sp",  "gp",  "tp", "t0",  "t1",  "t2",
  "s0",   "s1", "a0",  "a1",  "a2", "a3",  "a4",  "a5",
  "a6",   "a7", "s2",  "s3",  "s4", "s5",  "s6",  "s7",
  "s8",   "s9", "s10", "s11", "t3", "t4",  "t5",  "t6"
};

/* one additional level of indirection to allow 'csr' to be a #define macro */
#define READ_CSR(csr) read_csr(csr)

/* Trap handler */
unsigned long ulPortSyscallTrap(long cause, long epc, long tval, long regs[32])
{
	long returnValue = 0;
	/* Only process MACHINE-ecalls here, others cause an exit */
	switch (cause) {
	  case CAUSE_MACHINE_ECALL:
	  case CAUSE_SUPERVISOR_ECALL:
	  case CAUSE_USER_ECALL:
	    if (regs[17] == SYS_exit) {
	      prvPortSyscallExit(regs[10]); /* does not return */
	      break;
	    } else if (regs[17] == SYS_yield) {
	      return 0; /* Hint the caller in boot.S to do a yield */
	    } else if (regs[17] == SYS_priv) {
	      returnValue = vPortElevatePriv();
	      break;
	    } else if (regs[17] == SYS_interrupt) {
	      /* Act like an interrupt by restoring the faulting instruction
	       * Normally, this causes an endless loop, but we use it for benchmarking
	       * AEX with (fake) interrupts
	       */
	      return epc;
	      break;
	    } else {
	      returnValue = prvPortToHost(regs[17], regs[10], regs[11], regs[12]);
	      break;
	    }
	default:
	  printf("FATAL!\n");
	  unsigned long status = READ_CSR(STATUS);
	  unsigned long tstatus = READ_CSR(TSTATUS);
	  printf("cause = %ld epc = %p, tval = %p\n",  cause, (void*)epc, (void*)tval);
	  printf("mstatus = %lx\n", status);
	  printf("mtstatus = %lx\n", tstatus);
	  
	  for (unsigned int i = 1; i < 32; i++) {
	    printf("x%u (%s) = %17lu / %08lx\n", i, riscv_gpr_names_abi[i], regs[i], regs[i]);
	  }
	  vTaskList( cBuffer );
	  printf("%s", cBuffer);
	  prvPortSyscallExit(cause); /* does not return */
	  break;
	}

	regs[10] = returnValue;
	return epc + 4;
}
/*-----------------------------------------------------------*/

/* Fires a syscall */
long lPortSyscall(long num, long arg0, long arg1, long arg2)
{
	register long a7 asm("a7") = num;
	register long a0 asm("a0") = arg0;
	register long a1 asm("a1") = arg1;
	register long a2 asm("a2") = arg2;
	asm volatile ("scall":"+r"(a0) : "r"(a1), "r"(a2), "r"(a7));
	return a0;
}
/*-----------------------------------------------------------*/

void vPortYield()
{
	lPortSyscall(SYS_yield, 0, 0, 0);
}
/*-----------------------------------------------------------*/

void vPortExit(BaseType_t xExitCode)
{
	lPortSyscall(SYS_exit, xExitCode, 0, 0);
}
/*-----------------------------------------------------------*/

#if (portUSING_MPU_WRAPPERS == 1)

long xPortRaisePrivilege( void )
{
  return lPortSyscall(SYS_priv, 0, 0, 0);
}
/*-----------------------------------------------------------*/

void vPortStoreTaskMPUSettings( xMPU_SETTINGS *xMPUSettings, const struct xMEMORY_REGION * const xRegions, StackType_t *pxBottomOfStack, uint32_t ulStackDepth )
{
	(void)pxBottomOfStack;
	(void)ulStackDepth;

	printf("init MPU @ %p\n", xMPUSettings);
	extern uint32_t __RAM_segment_start__[];
	extern uint32_t __RAM_segment_end__[];

	if( xRegions == NULL )
	{
		printf("No Regions specified. Giving full RAM access\n");
		/* No MPU regions are specified so allow access to all RAM. */
		xMPUSettings->xRegion[ 0 ].ulRegionBaseAddress  = ( portPOINTER_SIZE_TYPE )__RAM_segment_start__;
		xMPUSettings->xRegion[ 0 ].ulRegionBoundAddress = ( portPOINTER_SIZE_TYPE )__RAM_segment_end__;
		xMPUSettings->xRegion[ 0 ].ulRegionAttribute = portMPU_REGION_READ_WRITE;

		/* Invalidate all other regions. */
		for( unsigned int i = 0; i < portTOTAL_NUM_REGIONS; i++ )
		{
			xMPUSettings->xRegion[ i ].ulRegionBaseAddress = 0UL;
			xMPUSettings->xRegion[ i ].ulRegionBoundAddress = 0UL;
			xMPUSettings->xRegion[ i ].ulRegionAttribute = 0UL;
		}
	}
	else
	{
		/* User is responsible for specifying stack region */
		for( unsigned int i = 0; i < portTOTAL_NUM_REGIONS; i++ )
		{
			if( ( xRegions[ i ] ).ulLengthInBytes > 0UL )
			{
				/* Translate the generic region definition contained in
				xRegions into RISC-V specific MPU settings that are then
				stored in xMPUSettings. */
				xMPUSettings->xRegion[ i ].ulRegionBaseAddress  = ( portPOINTER_SIZE_TYPE )xRegions[ i ].pvBaseAddress;
				xMPUSettings->xRegion[ i ].ulRegionBoundAddress = ( portPOINTER_SIZE_TYPE )xRegions[ i ].pvBaseAddress + xRegions[ i ].ulLengthInBytes;
				xMPUSettings->xRegion[ i ].ulRegionAttribute = xRegions[ i ].ulParameters;
			}
			else
			{
				/* Invalidate the region. */
				xMPUSettings->xRegion[ i ].ulRegionBaseAddress = 0UL;
				xMPUSettings->xRegion[ i ].ulRegionBoundAddress = 0UL;
				xMPUSettings->xRegion[ i ].ulRegionAttribute = 0UL;
			}
		}
	}
	for (unsigned int i = 0; i < portTOTAL_NUM_REGIONS; i++) {
		printf("Configured region[%u] = 0x%08lx - 0x%08lx [0x%lx]\n",
			i, 
			xMPUSettings->xRegion[ i ].ulRegionBaseAddress,
			xMPUSettings->xRegion[ i ].ulRegionBoundAddress,
			xMPUSettings->xRegion[ i ].ulRegionAttribute);
	}
}
/*-----------------------------------------------------------*/

#define ENCLAVE_TLS_INDEX 0
#define TM_SUCCESS 0

void vPortLoadTaskMPUSettings( xMPU_SETTINGS *xMPUSettings )
{
  LOAD_MPU_SLOT(0, xMPUSettings->xRegion[0]);
  LOAD_MPU_SLOT(1, xMPUSettings->xRegion[1]);
  LOAD_MPU_SLOT(2, xMPUSettings->xRegion[2]);
  LOAD_MPU_SLOT(3, xMPUSettings->xRegion[3]);
  
#if ( portUSING_TM == 1 )
  void* xTTCB = pvTaskGetThreadLocalStoragePointer( NULL, ENCLAVE_TLS_INDEX );
  if ( NULL != xTTCB ) {
    tm_encls_load_enclave( xTTCB );
  }
#endif
}

/*-----------------------------------------------------------*/

/**
 * m ... privilege mode (m or s)
 */
#define DUMP_PMP(m) { \
  printf("pmp:\n"); \
  printf("%016lx (%016lx) [%016lx]\n", read_csr(m##pmpbase0), read_csr(m##pmpbound0), read_csr(m##pmpflags0)); \
  printf("%016lx (%016lx) [%016lx]\n", read_csr(m##pmpbase1), read_csr(m##pmpbound1), read_csr(m##pmpflags1)); \
  printf("%016lx (%016lx) [%016lx]\n", read_csr(m##pmpbase2), read_csr(m##pmpbound2), read_csr(m##pmpflags2)); \
  printf("%016lx (%016lx) [%016lx]\n", read_csr(m##pmpbase3), read_csr(m##pmpbound3), read_csr(m##pmpflags3)); \
  printf("%016lx (%016lx) [%016lx]\n", read_csr(m##pmpbase4), read_csr(m##pmpbound4), read_csr(m##pmpflags4)); \
  printf("%016lx (%016lx) [%016lx]\n", read_csr(m##pmpbase5), read_csr(m##pmpbound5), read_csr(m##pmpflags5)); \
  printf("%016lx (%016lx) [%016lx]\n", read_csr(m##pmpbase6), read_csr(m##pmpbound6), read_csr(m##pmpflags6)); \
  printf("%016lx (%016lx) [%016lx]\n", read_csr(m##pmpbase7), read_csr(m##pmpbound7), read_csr(m##pmpflags7)); \
}
/*-----------------------------------------------------------*/

static void prvSetupMPU( void )
{
	extern uint32_t __normal_functions_start[];
	extern uint32_t __normal_functions_end[];
	extern uint32_t __normal_data_start[];
	extern uint32_t __normal_data_end[];
	xMPU_REGION_REGISTERS entry;

	CLEAR_MPU_SLOT(0); /* user stack */
	CLEAR_MPU_SLOT(1); /* user custom */
	CLEAR_MPU_SLOT(2); /* user custom */
	CLEAR_MPU_SLOT(3); /* user custom */

	/* map all normal code and data memory */
	entry.ulRegionBaseAddress  =  ( portPOINTER_SIZE_TYPE )__normal_functions_start;
	entry.ulRegionBoundAddress =  ( portPOINTER_SIZE_TYPE )__normal_functions_end;
	entry.ulRegionAttribute = portMPU_REGION_EXECUTE;
	LOAD_MPU_SLOT(4, entry);
	entry.ulRegionBaseAddress  =  ( portPOINTER_SIZE_TYPE )__normal_data_start;
	entry.ulRegionBoundAddress =  ( portPOINTER_SIZE_TYPE )__normal_data_end;
	entry.ulRegionAttribute = portMPU_REGION_READ_WRITE;
	LOAD_MPU_SLOT(5, entry);

	//CLEAR_MPU_SLOT(6); /* reserved for ST mode */
	//CLEAR_MPU_SLOT(7); /* reserved for ST mode */
	
	DUMP_PMP(s);
}
/*-----------------------------------------------------------*/

#endif



