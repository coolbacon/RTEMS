/*
 * shoonis 9g25 
 * baconxu@gmail.com 
 */

#include <rtems.h>
#include <bsp.h>
#include <at91sam9x25.h>
#include <at91sam9x25_pmc.h>

uint16_t tstart;
bool benchmark_timer_find_average_overhead;
uint32_t tick_time;
/*
 * Set up TC0 -
 *   timer_clock2 (MCK/8)
 *   capture mode - this shouldn't matter
 */
void benchmark_timer_initialize( void )
{
}

/*
 *  The following controls the behavior of benchmark_timer_read().
 *
 *  AVG_OVEREHAD is the overhead for starting and stopping the timer.  It
 *  is usually deducted from the number returned.
 *
 *  LEAST_VALID is the lowest number this routine should trust.  Numbers
 *  below this are "noise" and zero is returned.
 */

#define AVG_OVERHEAD      0  /* It typically takes X.X microseconds */
                             /* (Y countdowns) to start/stop the timer. */
                             /* This value is in microseconds. */
#define LEAST_VALID       1  /* Don't trust a clicks value lower than this */

int benchmark_timer_read( void )
{
}

void benchmark_timer_disable_subtracting_average_overhead(bool find_flag)
{
  benchmark_timer_find_average_overhead = find_flag;
}

