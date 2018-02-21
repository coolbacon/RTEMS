/*
at91sam9xx5
baconxu@gmail.com
 */

#include <rtems/score/armv4.h>

#include <bsp.h>
#include <bsp/irq.h>
#include <bsp/irq-generic.h>
#include <at91sam9xx5.h>

void bsp_interrupt_dispatch(void)
{
  rtems_vector_number vector = AIC->AIC_IVR;

  bsp_interrupt_handler_dispatch(vector);

  AIC->AIC_EOICR = 0;
}

rtems_status_code bsp_interrupt_vector_enable(rtems_vector_number vector)
{
  AIC->AIC_IECR = 1 << vector;

  return RTEMS_SUCCESSFUL;
}

rtems_status_code bsp_interrupt_vector_disable(rtems_vector_number vector)
{
  AIC->AIC_IDCR = 1 << vector;

  return RTEMS_SUCCESSFUL;
}

rtems_status_code bsp_interrupt_facility_initialize(void)
{
  unsigned long i = 0;

  for (i = 0; i < 32; ++i) {
	AIC->AIC_SVR[i] = i;
  }

  /* disable all interrupts */
  AIC->AIC_IDCR = 0xffffffff;

  _CPU_ISR_install_vector(ARM_EXCEPTION_IRQ, _ARMV4_Exception_interrupt, NULL);

  return RTEMS_SUCCESSFUL;
}
