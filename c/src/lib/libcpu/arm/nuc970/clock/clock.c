/*
baconxu@gmail.com
 */

#include <rtems.h>
#include <rtems/clockdrv.h>
#include <rtems/libio.h>

#include <stdlib.h>
#include <bsp.h>
#include <nuc970.h>
#include <rtems/irq.h>
#include "../irq/irq.h"
#include "../pmc/pmc.h"

/**
 * Enables clock interrupt.
 *
 * If the interrupt is always on, this can be a NOP.
 */
static void clock_isr_on(const rtems_irq_connect_data *unused)
{
  /* enable timer interrupt */
  //#error PIT->PIT_MR |= PIT_MR_PITIEN;
  outpw(REG_TMR1_TCSR, (inpw(REG_TMR1_TCSR) & 0xFFFFFF00U)| (0x1UL << 30));
}

/**
 * Disables clock interrupts
 *
 * If the interrupt is always on, this can be a NOP.
 */
static void clock_isr_off(const rtems_irq_connect_data *unused)
{
  /* disable timer interrupt */
  //#error PIT->PIT_MR &= ~PIT_MR_PITIEN;
  outpw(REG_TMR1_TCSR, inpw(REG_TMR1_TCSR) & 0xBFFFFF00U);
}

/**
 * Tests to see if clock interrupt is enabled, and returns 1 if so.
 * If interrupt is not enabled, returns 0.
 *
 * If the interrupt is always on, this always returns 1.
 */
static int clock_isr_is_on(const rtems_irq_connect_data *irq)
{
  /* check timer interrupt */
  //#error return (PIT->PIT_MR & PIT_MR_PITIEN);
  return ((inpw(REG_TMR1_TCSR) & (0x3UL << 29)) == (0x3UL << 29));
}

void Clock_isr(rtems_irq_hdl_param arg);

/* Replace the first value with the clock's interrupt name. */
rtems_irq_connect_data clock_isr_data = {
  .name   = TMR1_IRQn,
  .hdl    = Clock_isr,
  .handle = NULL,
  .on     = clock_isr_on,
  .off    = clock_isr_off,
  .isOn   = clock_isr_is_on,
};


#define Clock_driver_support_install_isr( _new, _old ) \
  do {                                                 \
      (_old) = NULL;                                   \
      BSP_install_rtems_irq_handler(&clock_isr_data);  \
  } while(0)

static void Clock_driver_support_initialize_hardware(void)
{
	uint32_t st_str;
	uint32_t clk;
	uint32_t value;

	/* the system timer is driven from SLCK */
	clk = 12000000;
	value = (uint32_t)((clk * (uint64_t)rtems_configuration_get_microseconds_per_tick()) / 1000000);



	// enable the Timer 1 clock	
	outpw(REG_CLK_PCLKEN0, inpw(REG_CLK_PCLKEN0)| 0x200);
	outp32(REG_SYS_APBIPRST0, inp32(REG_SYS_APBIPRST0) | BIT9);
	outp32(REG_SYS_APBIPRST0, inp32(REG_SYS_APBIPRST0) & ~BIT9);

	/* Calculate the match value required for our wanted tick rate. */
	outpw(REG_TMR1_TICR, value);
	sysSetInterruptPriorityLevel(TMR1_IRQn, IRQ_LEVEL_1);


	
	outpw(REG_TMR1_TCSR, (inpw(REG_TMR1_TCSR) & 0x81FFFF00) | (0xD << 27));		// 0xC means CEN and IE were enable
}

#define Clock_driver_support_at_tick() \
  do { \
	outpw(REG_TMR_TISR, 0x2);\
  } while (0)

static void Clock_driver_support_shutdown_hardware( void )
{
  BSP_remove_rtems_irq_handler(&clock_isr_data);
}

#define CLOCK_DRIVER_USE_DUMMY_TIMECOUNTER

#include "../../../../libbsp/shared/clockdrv_shell.h"
