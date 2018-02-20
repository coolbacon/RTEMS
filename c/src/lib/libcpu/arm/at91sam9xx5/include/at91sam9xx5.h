#ifndef _AT91SAM9XX5_H_
#define _AT91SAM9XX5_H_

/*
 * Peripherals registers definitions
 */
#if defined sam9g15
    #include <sam9g15.h>
#elif defined sam9g25
    #include <sam9g25.h>
#elif defined sam9g35
    #include <sam9g35.h>
#elif defined sam9x25
    #include <sam9x25.h>
#elif defined sam9x35
    #include <sam9x35.h>
#else
    #warning Library does not support the specified chip, specifying atsam9g25.
    #define sam9g25
    #include <sam9g25.h>
#endif


/* Define attribute */
#if defined   ( __CC_ARM   ) /* Keil µVision 4 */
    #define WEAK __attribute__ ((weak))
#elif defined ( __ICCARM__ ) /* IAR Ewarm 5.41+ */
    #define WEAK __weak
#elif defined (  __GNUC__  ) /* GCC CS3 2009q3-68 */
    #define WEAK __attribute__ ((weak))
#endif

/* Define NO_INIT attribute and compiler specific symbols */
#if defined   ( __CC_ARM   )
    #define NO_INIT
    #define __ASM            __asm                                    /*!< asm keyword for ARM Compiler          */
    #define __INLINE         __inline                                 /*!< inline keyword for ARM Compiler       */
#elif defined ( __ICCARM__ )
    #define NO_INIT __no_init
    #define __ASM           __asm                                     /*!< asm keyword for IAR Compiler           */
    #define __INLINE        inline                                    /*!< inline keyword for IAR Compiler. Only avaiable in High optimization mode! */
#elif defined (  __GNUC__  )
    #define __ASM            asm                                      /*!< asm keyword for GNU Compiler          */
    #define __INLINE         inline                                   /*!< inline keyword for GNU Compiler       */
    #define NO_INIT
#endif


#if 0
#define CP15_PRESENT

/*
 * Peripherals
 */
//#include "at91sam9xx5/mmu.h"
#include "at91sam9xx5/rstc.h"
#include "at91sam9xx5/adc.h"
#include "at91sam9xx5/async.h"
#include "at91sam9xx5/hsmci.h"
#include "at91sam9xx5/irq.h"
#include "at91sam9xx5/pio.h"
#include "at91sam9xx5/pio_it.h"
#include "at91sam9xx5/pmc.h"
#include "at91sam9xx5/pwmc.h"
#include "at91sam9xx5/rtc.h"
#include "at91sam9xx5/spi.h"
#include "at91sam9xx5/ssc.h"
#include "at91sam9xx5/tc.h"
#include "at91sam9xx5/twi.h"
#include "at91sam9xx5/twid.h"
#include "at91sam9xx5/usart.h"
#include "at91sam9xx5/pit.h" 
#include "at91sam9xx5/dmac.h"
#include "at91sam9xx5/udphs.h"
#if defined(EMAC)||defined(EMAC0)
#include "at91sam9xx5/emac.h"
#endif
#if defined(ISI)
#include "at91sam9xx5/video.h"
#include "at91sam9xx5/isi.h"
#endif
#if defined(CAN)||defined(CAN0)
#include "at91sam9xx5/can.h"
#endif
#include "include/trace.h"
#include "include/wdt.h"
#else

#include <stdint.h>

uint32_t at91sam9xx5_get_mainclk(void);
uint32_t at91sam9xx5_get_slck(void);
uint32_t at91sam9xx5_get_mck(void);

extern void PMC_EnablePeripheral( uint32_t dwId ) ;
extern void PMC_DisablePeripheral( uint32_t dwId ) ;
extern void PMC_EnableAllPeripherals( void ) ;
extern void PMC_DisableAllPeripherals( void ) ;
extern uint32_t PMC_IsPeriphEnabled( uint32_t dwId ) ;


#endif

#endif /* _LIB_CHIP_SAM9XX5_ */
