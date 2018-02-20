/*
at91sam9x25

baconxu@gmail.com
*/
#include <bsp.h>
#include <bsp/bootcard.h>
#include <at91sam9xx5.h>

#define RSTC_KEY_PASSWORD           RSTC_MR_KEY(0xA5U)

void bsp_reset(void)
{
  rtems_interrupt_level level;

  rtems_interrupt_disable(level);
    (void) level; /* avoid set but not used warning */

    /* Enable the watchdog timer, then wait for the world to end. */
    //ST_REG(ST_WDMR) = ST_WDMR_RSTEN | 1;
	RSTC->RSTC_CR = RSTC_CR_PROCRST | RSTC_KEY_PASSWORD;
    while(1)
      ;
}
