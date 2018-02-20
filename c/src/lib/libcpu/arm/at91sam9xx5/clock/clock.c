/*
at91sam9x25
baconxu@gmail.com
 */

#include <rtems.h>
#include <rtems/clockdrv.h>
#include <rtems/libio.h>

#include <stdlib.h>
#include <bsp.h>
#include <at91sam9xx5.h>
#include <rtems/irq.h>

/**
 * Enables clock interrupt.
 *
 * If the interrupt is always on, this can be a NOP.
 */
static void clock_isr_on(const rtems_irq_connect_data *unused)
{
  /* enable timer interrupt */
  //ST_REG(ST_IER) = ST_SR_PITS;

  PIT->PIT_MR |= PIT_MR_PITIEN;
}

/**
 * Disables clock interrupts
 *
 * If the interrupt is always on, this can be a NOP.
 */
static void clock_isr_off(const rtems_irq_connect_data *unused)
{
  /* disable timer interrupt */
  //ST_REG(ST_IDR) = ST_SR_PITS;

  PIT->PIT_MR &= ~PIT_MR_PITIEN;
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
  //return ST_REG(ST_IMR) & ST_SR_PITS;
  return (PIT->PIT_MR & PIT_MR_PITIEN);
}

void Clock_isr(rtems_irq_hdl_param arg);

/* Replace the first value with the clock's interrupt name. */
rtems_irq_connect_data clock_isr_data = {
  .name   = SYS_IRQn,
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
  int clk;
  unsigned long value;

  /* the system timer is driven from SLCK */
  clk = at91sam9xx5_get_mck();
  value = ((((uint64_t)rtems_configuration_get_microseconds_per_tick() * clk) +
                      (1000000/2))/ 1000000);

  /* read the status to clear the int */
  //st_str = ST_REG(ST_SR);
  st_str = PIT->PIT_SR;
  (void) st_str; /* avoid set but not used warning */ \

  /* set priority */
  //AIC_SMR_REG(AIC_SMR_SYSIRQ) = AIC_SMR_PRIOR(0x7);
  AIC->AIC_SMR[SYS_IRQn] = AIC_SMR_PRIOR_HIGHEST;
  	//| AIC_SMR_SRCTYPE_INT_EDGE_TRIGGERED;

  /* set the timer value */
  //ST_REG(ST_PIMR) = value;
  PIT->PIT_MR = (PIT->PIT_MR & (~PIT_MR_PIV_Msk)) | (value & PIT_MR_PIV_Msk);

  /*start the pit*/
  PIT->PIT_MR |= PIT_MR_PITEN;
}

#define Clock_driver_support_at_tick() \
  do { \
    uint32_t st_str; \
    \
    /* read the status to clear the int */ \
    st_str = PIT->PIT_SR;\
	if (st_str & PIT_SR_PITS)\
		st_str = PIT->PIT_PIVR;\
    (void) st_str; /* avoid set but not used warning */ \
  } while (0)

static void Clock_driver_support_shutdown_hardware( void )
{
  BSP_remove_rtems_irq_handler(&clock_isr_data);
}

#define CLOCK_DRIVER_USE_DUMMY_TIMECOUNTER

#include "../../../../libbsp/shared/clockdrv_shell.h"
