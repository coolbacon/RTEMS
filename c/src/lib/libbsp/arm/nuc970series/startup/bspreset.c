/*
nuc977

baconxu@gmail.com
*/
#include <bsp.h>
#include <bsp/bootcard.h>


void bsp_reset(void)
{
  rtems_interrupt_level level;

  rtems_interrupt_disable(level);
    (void) level; /* avoid set but not used warning */

    /* Enable the watchdog timer, then wait for the world to end. */
#warning add some code reset system.
#warning bacon
    while(1)
      ;
}
