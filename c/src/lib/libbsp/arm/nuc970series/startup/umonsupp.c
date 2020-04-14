/*
nuc977
baconxu@gmail.com
 */

#include <bsp.h>
#include <rtems/umon.h>

/*
 * BSP specific routine to help when calling monConnect().  This
 * returns the value known to uMon as MONCOMPTR.
 */

void *rtems_bsp_get_umon_monptr(void)
{
  return (void *)0x10000020;
}

