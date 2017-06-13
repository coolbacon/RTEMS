/*
 * Interrupt handler Header file
 * shoonis 9g25 
 * baconxu@gmail.com 
 */

#ifndef __IRQ_H__
#define __IRQ_H__

#ifndef __asm__

#include <rtems.h>
#include <rtems/irq.h>
#include <rtems/irq-extension.h>

#endif /* __asm__ */

/* possible interrupt sources on the AT91SAM9G25 */
#define AT91SAM9G25_INT_FIQ        0
#define AT91SAM9G25_INT_SYSIRQ     1
#define AT91SAM9G25_INT_PIOA       2
#define AT91SAM9G25_INT_PIOB       2
#define AT91SAM9G25_INT_PIOC       3
#define AT91SAM9G25_INT_PIOD       3
#define AT91SAM9G25_INT_SMD        4
#define AT91SAM9G25_INT_US0        5
#define AT91SAM9G25_INT_US1        6
#define AT91SAM9G25_INT_US2        7
#define AT91SAM9G25_INT_US3        8
#define AT91SAM9G25_INT_TWI0       9
#define AT91SAM9G25_INT_TWI1      10
#define AT91SAM9G25_INT_TWI2      11
#define AT91SAM9G25_INT_HSMCI0    12
#define AT91SAM9G25_INT_SPI0      13
#define AT91SAM9G25_INT_SPI1      14
#define AT91SAM9G25_INT_UART0     15
#define AT91SAM9G25_INT_UART1     16
#define AT91SAM9G25_INT_TC0       17
#define AT91SAM9G25_INT_TC1       17
#define AT91SAM9G25_INT_PWM       18
#define AT91SAM9G25_INT_ADC       19
#define AT91SAM9G25_INT_DMAC0     20
#define AT91SAM9G25_INT_DMAC1     21
#define AT91SAM9G25_INT_UHPHS     22
#define AT91SAM9G25_INT_UDPHS     23
#define AT91SAM9G25_INT_EMAC      24
#define AT91SAM9G25_INT_ISI       25
#define AT91SAM9G25_INT_HSMCI1    26
#define AT91SAM9G25_INT_SSC       28
#define AT91SAM9G25_INT_IRQ       31
#define AT91SAM9G25_MAX_INT       32

#define BSP_INTERRUPT_VECTOR_MIN 0

#define BSP_INTERRUPT_VECTOR_MAX (AT91SAM9G25_MAX_INT - 1)

#endif /* __IRQ_H__ */
