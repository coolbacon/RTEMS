/*
nuc977

baconxu@gmail.com
 */

#include <bsp.h>
#include <bsp/irq-generic.h>


/* Function prototypes */
static void fix_mac_addr(void);
void bsp_usart_init(void);

/*
 * bsp_start_default - BSP initialization function
 *
 * This function is called before RTEMS is initialized and used
 * adjust the kernel's configuration.
 *
 * This function also configures the CPU's memory protection unit.
 *
 * RESTRICTIONS/LIMITATIONS:
 *   Since RTEMS is not configured, no RTEMS functions can be called.
 */
static void bsp_start_default( void )
{

  /* stop watchdog */





  
  /*
   * Some versions of the bootloader have the MAC address
   * reversed. This fixes it, if necessary.
   */
  fix_mac_addr();

  /*
   * Init rtems PIO configuration for USARTs
   */
  bsp_uart_init(); 

  /*
   * Init rtems exceptions management
   */
  /* FIXME: Use shared start.S */
  rtems_exception_init_mngt();

  /*
   * Init rtems interrupt management
   */
  bsp_interrupt_initialize();

} /* bsp_start */

/*
 * Some versions of the bootloader shipped with the CSB337
 * reverse the MAC address. This function tests for that,
 * and fixes the MAC address.
 */
static void fix_mac_addr(void)
{

}

/*
 *
 * NAME: bsp_uart_init - Function to setup the PIO in USART mode
 *       before startup
 *
 * DESCRIPTION:
 *   This function is called before usart driver is initialized and is
 *   used to setup the proper mode of PIO operation for USART.
 *
 * NOTES:
 *   The initialization could be done smarter by programming only the
 *   bits you need to program for each USART when the port is ENABLED.
 *
 */
void bsp_uart_init(void)
{

} 

/*
 *  By making this a weak alias for bsp_start_default, a brave soul
 *  can override the actual bsp_start routine used.
 */
void bsp_start (void) __attribute__ ((weak, alias("bsp_start_default")));
