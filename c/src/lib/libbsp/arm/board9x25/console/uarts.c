/*
board9x25
baconxu@gmail.com
 */

#include <bsp.h>
#include <rtems/libio.h>
#include <termios.h>
#include <rtems/bspIo.h>

#include <at91sam9xx5.h>
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

#if ENABLE_USART0 || ENABLE_USART1 || ENABLE_USART2 || ENABLE_USART3
  extern const console_fns usart_polling_fns;
#endif

#if ENABLE_USART0
  #define USART0_DEV 1
#else
  #define USART0_DEV 0
#endif

#if ENABLE_USART1
  #define USART1_DEV 1
#else
  #define USART1_DEV 0
#endif

#if ENABLE_USART2
  #define USART2_DEV 1
#else
  #define USART2_DEV 0
#endif

#if ENABLE_USART3
  #define USART3_DEV 1
#else
  #define USART3_DEV 0
#endif

#define NUM_DEVS \
  (1 + UMON_CONS_DEV + \
  USART0_DEV + USART1_DEV + USART2_DEV + USART3_DEV)

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
    (uint32_t)DBGU,    /* ulCtrlPort1  - Pointer to DBGU regs */
    0,                 /* ulCtrlPort2  - NOT USED */
    0,                 /* ulDataPort  - NOT USED */
    NULL,              /* getRegister - NOT USED */
    NULL,              /* setRegister - NOT USED */
    NULL,              /* getData - NOT USED */
    NULL,              /* setData - NOT USED */
    0,                 /* ulClock - NOT USED */
    0                  /* ulIntVector - NOT USED */
  },
#if (ENABLE_UMON && ENABLE_UMON_CONSOLE)
  {
    "/dev/umon",       /* sDeviceName */
    SERIAL_CUSTOM,     /* deviceType */
    &umoncons_fns,     /* pDeviceFns */
    NULL,              /* deviceProbe */
    NULL,              /* pDeviceFlow */
    0,                 /* ulMargin - NOT USED */
    0,                 /* ulHysteresis - NOT USED */
    NULL,              /* pDeviceParams */
    0,                 /* ulCtrlPort1  - Pointer to UMON regs */
    0,                 /* ulCtrlPort2  - NOT USED */
    0,                 /* ulDataPort  - NOT USED */
    NULL,              /* getRegister - NOT USED */
    NULL,              /* setRegister - NOT USED */
    NULL,              /* getData - NOT USED */
    NULL,              /* setData - NOT USED */
    0,                 /* ulClock - NOT USED */
    0                  /* ulIntVector - NOT USED */
  },
#endif
#if ENABLE_USART0
  {
    "/dev/com1",       /* sDeviceName */
    SERIAL_CUSTOM,     /* deviceType */
    &usart_polling_fns,/* pDeviceFns */
    NULL,              /* deviceProbe */
    NULL,              /* pDeviceFlow */
    0,                 /* ulMargin - NOT USED */
    0,                 /* ulHysteresis - NOT USED */
    NULL,              /* pDeviceParams */
    (uint32_t)USART0,  /* ulCtrlPort1  - Pointer to USART 0 regs */
    0,                 /* ulCtrlPort2  - NOT USED */
    0,                 /* ulDataPort  - NOT USED */
    NULL,              /* getRegister - NOT USED */
    NULL,              /* setRegister - NOT USED */
    NULL,              /* getData - NOT USED */
    NULL,              /* setData - NOT USED */
    0,                 /* ulClock - NOT USED */
    0                  /* ulIntVector - NOT USED */
  },
#endif
#if ENABLE_USART1
  {
    "/dev/com2",       /* sDeviceName */
    SERIAL_CUSTOM,     /* deviceType */
    &usart_polling_fns,/* pDeviceFns */
    NULL,              /* deviceProbe */
    NULL,              /* pDeviceFlow */
    0,                 /* ulMargin - NOT USED */
    0,                 /* ulHysteresis - NOT USED */
    NULL,              /* pDeviceParams */
    (uint32_t)USART1,  /* ulCtrlPort1  - Pointer to USART 1 regs */
    0,                 /* ulCtrlPort2  - NOT USED */
    0,                 /* ulDataPort  - NOT USED */
    NULL,              /* getRegister - NOT USED */
    NULL,              /* setRegister - NOT USED */
    NULL,              /* getData - NOT USED */
    NULL,              /* setData - NOT USED */
    0,                 /* ulClock - NOT USED */
    0                  /* ulIntVector - NOT USED */
  },
#endif
#if ENABLE_USART2
  {
    "/dev/com3",       /* sDeviceName */
    SERIAL_CUSTOM,     /* deviceType */
    &usart_polling_fns,/* pDeviceFns */
    NULL,              /* deviceProbe */
    NULL,              /* pDeviceFlow */
    0,                 /* ulMargin - NOT USED */
    0,                 /* ulHysteresis - NOT USED */
    NULL,              /* pDeviceParams */
    (uint32_t)USART2,  /* ulCtrlPort1  - Pointer to USART 2 regs */
    0,                 /* ulCtrlPort2  - NOT USED */
    0,                 /* ulDataPort  - NOT USED */
    NULL,              /* getRegister - NOT USED */
    NULL,              /* setRegister - NOT USED */
    NULL,              /* getData - NOT USED */
    NULL,              /* setData - NOT USED */
    0,                 /* ulClock - NOT USED */
    0                  /* ulIntVector - NOT USED */
  },
#endif
#if ENABLE_USART3
  {
    "/dev/com4",       /* sDeviceName */
    SERIAL_CUSTOM,     /* deviceType */
    &usart_polling_fns,/* pDeviceFns */
    NULL,              /* deviceProbe */
    NULL,              /* pDeviceFlow */
    0,                 /* ulMargin - NOT USED */
    0,                 /* ulHysteresis - NOT USED */
    NULL,              /* pDeviceParams */
    (uint32_t)USART3,  /* ulCtrlPort1  - Pointer to USART 3 regs */
    0,                 /* ulCtrlPort2  - NOT USED */
    0,                 /* ulDataPort  - NOT USED */
    NULL,              /* getRegister - NOT USED */
    NULL,              /* setRegister - NOT USED */
    NULL,              /* getData - NOT USED */
    NULL,              /* setData - NOT USED */
    0,                 /* ulClock - NOT USED */
    0                  /* ulIntVector - NOT USED */
  }
#endif
};

console_tbl *BSP_get_uart_from_minor(int minor)
{
    return Console_Port_Tbl[minor];
}
