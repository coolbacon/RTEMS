/*
board9x25
baconxu@gmail.com
 */

#include <bsp.h>
#include <rtems/libio.h>
#include <termios.h>
#include <rtems/bspIo.h>

#include <libchip/serial.h>
#include <libchip/sersupp.h>
#include <bspopts.h>

extern const console_fns dbgu_fns;

#if (ENABLE_UMON && ENABLE_UMON_CONSOLE)
  extern const console_fns umoncons_fns;
  #define UMON_CONS_DEV 1
#else
  #define UMON_CONS_DEV 0
#endif


#define NUM_DEVS \
  (1 + UMON_CONS_DEV)

/* These are used by code in console.c */
unsigned long Console_Configuration_Count = NUM_DEVS;

/*
 * There's one item in array for each UART.
 *
 * Some of these fields are marked "NOT USED". They are not used
 * by console.c, but may be used by drivers in libchip
 *
 * when we add other types of UARTS we will need to move this
 * structure to a generic uart.c file with only this in it
 */
console_tbl Console_Configuration_Ports[] = {
  {
    "/dev/com0",       /* sDeviceName */
    SERIAL_CUSTOM,     /* deviceType */
    &dbgu_fns,         /* pDeviceFns */
    NULL,              /* deviceProbe */
    NULL,              /* pDeviceFlow */
    0,                 /* ulMargin - NOT USED */
    0,                 /* ulHysteresis - NOT USED */
    NULL,              /* pDeviceParams */
    (uint32_t)0,    /* ulCtrlPort1  - Pointer to DBGU regs */
    0,                 /* ulCtrlPort2  - NOT USED */
    0,                 /* ulDataPort  - NOT USED */
    NULL,              /* getRegister - NOT USED */
    NULL,              /* setRegister - NOT USED */
    NULL,              /* getData - NOT USED */
    NULL,              /* setData - NOT USED */
    0,                 /* ulClock - NOT USED */
    0                  /* ulIntVector - NOT USED */
  },
};

console_tbl *BSP_get_uart_from_minor(int minor)
{
    return Console_Port_Tbl[minor];
}
