/*
 *  Console driver for shoonis at91sam9g25
 *  baconxu@gmail.com
*/

#include <bsp.h>
#include <rtems/libio.h>
#include <termios.h>
#include <rtems/bspIo.h>

#include <at91sam9x25.h>
#include <at91sam9x25_dbgu.h>
#include <libchip/serial.h>
#include <libchip/sersupp.h>
#include <bspopts.h>

/* rtems console uses the following minor number */
//rtems_device_minor_number  Console_Port_Minor = 0;
extern const console_fns dbgu_fns;
#if (ENABLE_UMON && ENABLE_UMON_CONSOLE)
    extern const console_fns umoncons_fns;
#endif

#define NUM_DEVS        1

/* These are used by code in console.c */
unsigned long Console_Configuration_Count = NUM_DEVS;
//console_data  Console_Port_Data[NUM_DEVS];

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
    "/dev/com0",    /* sDeviceName */
    SERIAL_CUSTOM,     /* deviceType */
    &dbgu_fns,         /* pDeviceFns */
    NULL,              /* deviceProbe */
    NULL,              /* pDeviceFlow */
    0,                 /* ulMargin - NOT USED */
    0,                 /* ulHysteresis - NOT USED */
    NULL,              /* pDeviceParams */
    DBGU_BASE,         /* ulCtrlPort1  - Pointer to DBGU regs */
    0,                 /* ulCtrlPort2  - NOT USED */
    0,                 /* ulDataPort  - NOT USED */
    NULL,              /* getRegister - NOT USED */
    NULL,              /* setRegister - NOT USED */
    NULL,              /* getData - NOT USED */
    NULL,              /* setData - NOT USED */
    0,                 /* ulClock - NOT USED */
    0                  /* ulIntVector - NOT USED */
  },
#if 0
    {
    "/dev/usart0",    /* sDeviceName */
    SERIAL_CUSTOM,     /* deviceType */
    &usart_fns,         /* pDeviceFns */
    NULL,              /* deviceProbe */
    NULL,              /* pDeviceFlow */
    0,                 /* ulMargin - NOT USED */
    0,                 /* ulHysteresis - NOT USED */
    (void *)-1,              /* pDeviceParams */
    0,         			/* ulCtrlPort1  - Pointer to DBGU regs */
    0,                 /* ulCtrlPort2  - NOT USED */
    0,                 /* ulDataPort  - NOT USED */
    NULL,              /* getRegister - NOT USED */
    NULL,              /* setRegister - NOT USED */
    NULL,              /* getData - NOT USED */
    NULL,              /* setData - NOT USED */
    0,                 /* ulClock - NOT USED */
    5                  /* ulIntVector - NOT USED */
  },
  {
    "/dev/usart1",    /* sDeviceName */
    SERIAL_CUSTOM,     /* deviceType */
    &usart_fns,         /* pDeviceFns */
    NULL,              /* deviceProbe */
    NULL,              /* pDeviceFlow */
    0,                 /* ulMargin - NOT USED */
    0,                 /* ulHysteresis - NOT USED */
    (void *)-1,              /* pDeviceParams */
    1,         		   /* ulCtrlPort1  - Pointer to DBGU regs */
    0,                 /* ulCtrlPort2  - NOT USED */
    0,                 /* ulDataPort  - NOT USED */
    NULL,              /* getRegister - NOT USED */
    NULL,              /* setRegister - NOT USED */
    NULL,              /* getData - NOT USED */
    NULL,              /* setData - NOT USED */
    0,                 /* ulClock - NOT USED */
    6                  /* ulIntVector - NOT USED */
  },
  {
    "/dev/usart2",    /* sDeviceName */
    SERIAL_CUSTOM,     /* deviceType */
    &usart_fns,         /* pDeviceFns */
    NULL,              /* deviceProbe */
    NULL,              /* pDeviceFlow */
    0,                 /* ulMargin - NOT USED */
    0,                 /* ulHysteresis - NOT USED */
    (void *)-1,              /* pDeviceParams */
    2,         			/* ulCtrlPort1  - Pointer to DBGU regs */
    0,                 /* ulCtrlPort2  - NOT USED */
    0,                 /* ulDataPort  - NOT USED */
    NULL,              /* getRegister - NOT USED */
    NULL,              /* setRegister - NOT USED */
    NULL,              /* getData - NOT USED */
    NULL,              /* setData - NOT USED */
    0,                 /* ulClock - NOT USED */
    7                  /* ulIntVector - NOT USED */
  },
    {
    "/dev/usart3",    /* sDeviceName */
    SERIAL_CUSTOM,     /* deviceType */
    &usart_fns,         /* pDeviceFns */
    NULL,              /* deviceProbe */
    NULL,              /* pDeviceFlow */
    0,                 /* ulMargin - NOT USED */
    0,                 /* ulHysteresis - NOT USED */
    (void *)-1,              /* pDeviceParams */
    3,         			/* ulCtrlPort1  - Pointer to DBGU regs */
    0,                 /* ulCtrlPort2  - NOT USED */
    0,                 /* ulDataPort  - NOT USED */
    NULL,              /* getRegister - NOT USED */
    NULL,              /* setRegister - NOT USED */
    NULL,              /* getData - NOT USED */
    NULL,              /* setData - NOT USED */
    0,                 /* ulClock - NOT USED */
    8                  /* ulIntVector - NOT USED */
  },
#endif
};

console_tbl *BSP_get_uart_from_minor(int minor)
{
    return Console_Port_Tbl[minor];
}
