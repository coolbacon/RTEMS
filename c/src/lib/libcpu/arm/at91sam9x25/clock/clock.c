/*
    RTEMS.CN AT91SAM9X25
    baconxu@gmail.com
 */

#include <rtems.h>
#include <rtems/clockdrv.h>
#include <rtems/libio.h>

#include <stdlib.h>
#include <bsp.h>
#include <bsp/irq.h>
#include <at91sam9x25.h>
#include <at91sam9x25_pmc.h>


static unsigned long st_pimr_reload;

/**
 * Enables clock interrupt.
 *
 * If the interrupt is always on, this can be a NOP.
 */
static void clock_isr_on(const rtems_irq_connect_data *unused)
{
    /* enable timer interrupt */
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
	PIT->PIT_MR &= ~PIT_MR_PITIEN;
    return;
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
	return (PIT->PIT_MR & PIT_MR_PITIEN);
}

rtems_isr Clock_isr(void *arg);

/* Replace the first value with the clock's interrupt name. */
rtems_irq_connect_data clock_isr_data = {AT91SAM9G25_INT_SYSIRQ,
                                         (rtems_irq_hdl)Clock_isr,
										 NULL,
                                         clock_isr_on,
                                         clock_isr_off,
                                         clock_isr_is_on};


#define Clock_driver_support_install_isr( _new, _old ) \
  do {                                                 \
      (_old) = NULL;                                   \
      BSP_install_rtems_irq_handler(&clock_isr_data);  \
  } while(0)

void Clock_driver_support_initialize_hardware(void)
{
  uint32_t st_str;
  uint32_t clk;
  uint32_t st_pimr_value;

  /* the system timer is driven from SLCK */
  clk = at91sam9g25_get_mck();
  st_pimr_value =
    (((clk * (uint64_t)rtems_configuration_get_microseconds_per_tick() + 500000) / 1000000 + 8) / 16) - 1;
  st_pimr_reload = st_pimr_value;

  /* read the status to clear the int */
  st_str = PIT->PIT_SR;

  /* set priority */
  AIC->AIC_SMR[AT91SAM9G25_INT_SYSIRQ] = AIC_SMR_PRIOR_HIGHEST | AIC_SMR_SRCTYPE_INT_EDGE_TRIGGERED;

  /* set the timer value */
  PIT->PIT_MR = (PIT->PIT_MR & (~PIT_MR_PIV_Msk)) | (st_pimr_reload & PIT_MR_PIV_Msk);

  /*start the pit*/
  PIT->PIT_MR |= PIT_MR_PITEN;
}

#if 0
uint32_t bsp_clock_nanoseconds_since_last_tick(void)
{
  uint16_t slck_counts;

  slck_counts = st_pimr_value - st_pimr_reload;
  return (rtems_configuration_get_microseconds_per_tick() * slck_counts * 1000)
     / st_pimr_value;
}
#endif

#if 0
#define Clock_driver_nanoseconds_since_last_tick \
  bsp_clock_nanoseconds_since_last_tick

#define CLOCK_VECTOR 0
#endif

#define Clock_driver_support_at_tick() \
  do { \
    uint32_t st_str; \
	uint32_t dummy;\
    \
    /* read the status to clear the int */ \
	st_str = PIT->PIT_SR;\
	if (st_str & PIT_SR_PITS)\
		dummy = PIT->PIT_PIVR;\
  } while (0)

static void Clock_driver_support_shutdown_hardware( void )
{
  BSP_remove_rtems_irq_handler(&clock_isr_data);
}

#define CLOCK_DRIVER_USE_DUMMY_TIMECOUNTER

#include "../../../../libbsp/shared/clockdrv_shell.h"
