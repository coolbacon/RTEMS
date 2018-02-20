/*
at91sam9xx5
baconxu@gmail.com
 */

#ifndef __IRQ_H__
#define __IRQ_H__

#ifndef __asm__

#include <rtems.h>
#include <rtems/irq.h>
#include <rtems/irq-extension.h>

#endif /* __asm__ */

/* possible interrupt sources on the AT91SAM9xx5 */
#define AT91SAM9XX5_MAX_INT       32

#define BSP_INTERRUPT_VECTOR_MIN 0

#define BSP_INTERRUPT_VECTOR_MAX (AT91SAM9XX5_MAX_INT - 1)

#endif /* __IRQ_H__ */
