/*
nuc970
baconxu@gmail.com
 */

#include <rtems/score/armv4.h>

#include <bsp.h>
#include <bsp/irq.h>
#include <bsp/irq-generic.h>
#include <nuc970.h>

#define SYS_MAX_INT_SOURCE BSP_INTERRUPT_VECTOR_MAX
#define SYS_MIN_INT_SOURCE BSP_INTERRUPT_VECTOR_MIN


void bsp_interrupt_dispatch(void)
{
  	rtems_vector_number vector;

	unsigned int volatile _mIPER, _mISNR;

    _mIPER = (inpw(REG_AIC_IPER) >> 2) & 0x3f;
    _mISNR = inpw(REG_AIC_ISNR);
	vector = _mIPER;
    if (_mIPER != 0) {
        if (_mISNR != 0)
			bsp_interrupt_handler_dispatch(vector);
        outpw(REG_AIC_EOSCR, 1);
    }
}

rtems_status_code bsp_interrupt_vector_enable(rtems_vector_number vector)
{
  	//AIC->AIC_IECR = 1 << vector;

   	if ((vector > SYS_MAX_INT_SOURCE) || (vector < SYS_MIN_INT_SOURCE))
      return 1;

    if (vector < 32)
        outpw(REG_AIC_MECR, (1 << vector));
    else
        outpw(REG_AIC_MECRH, (1 << (vector - 32)));

 	return RTEMS_SUCCESSFUL;
}

rtems_status_code bsp_interrupt_vector_disable(rtems_vector_number vector)
{
    if ((vector > SYS_MAX_INT_SOURCE) || (vector < SYS_MIN_INT_SOURCE))
        return 1;

    if (vector < 32)
        outpw(REG_AIC_MDCR, (1 << vector));
    else
        outpw(REG_AIC_MDCRH, (1 << (vector - 32)));

 	 return RTEMS_SUCCESSFUL;
}

rtems_status_code bsp_interrupt_facility_initialize(void)
{

#if 0
  unsigned long i = 0;

  for (i = 0; i < 32; ++i) {
	AIC->AIC_SVR[i] = i;
  }

  /* disable all interrupts */
  AIC->AIC_IDCR = 0xffffffff;
#endif

	/*disable all interrupts*/
	outpw(REG_AIC_MDCR, 0xFFFFFFFF);
	outpw(REG_AIC_MDCRH, 0xFFFFFFFF);

  	_CPU_ISR_install_vector(ARM_EXCEPTION_IRQ, _ARMV4_Exception_interrupt, NULL);

  	return RTEMS_SUCCESSFUL;
}


/**
 *  @brief  system AIC - Change interrupt level
 *
 *  @param[in]  eIntNo  Interrupt number. \ref IRQn_Type
 *  @param[in]  uIntLevel   Interrupt Level. ( \ref FIQ_LEVEL_0 / \ref IRQ_LEVEL_1 / \ref IRQ_LEVEL_2 / \ref IRQ_LEVEL_3 /
 *                                             \ref IRQ_LEVEL_4 / \ref IRQ_LEVEL_5 / \ref IRQ_LEVEL_6 / \ref IRQ_LEVEL_7 )
 *
 *  @return   0
 */
int32_t sysSetInterruptPriorityLevel(IRQn_Type eIntNo, uint32_t uIntLevel)
{
    uint32_t  _mRegAddr;
    int     shift;

   if ((eIntNo > SYS_MAX_INT_SOURCE) || (eIntNo < SYS_MIN_INT_SOURCE))
      return 1;

    _mRegAddr = REG_AIC_SCR1 + ((eIntNo / 4) * 4);
    shift = (eIntNo % 4) * 8;
    uIntLevel &= 0x7;
    outpw(_mRegAddr, (inpw(_mRegAddr) & ~(0x07 << shift)) | (uIntLevel << shift));

    return 0;
}


int32_t sysSetInterruptType(IRQn_Type eIntNo, uint32_t uIntSourceType)
{
    uint32_t _mRegAddr;
    int     shift;

    if ((eIntNo > SYS_MAX_INT_SOURCE) || (eIntNo < SYS_MIN_INT_SOURCE))
        return 1;

    _mRegAddr = REG_AIC_SCR1 + ((eIntNo / 4) * 4);
    shift = (eIntNo % 4) * 8;
    uIntSourceType &= 0xC0;
    outpw(_mRegAddr, (inpw(_mRegAddr) & ~(0xC0 << shift)) | (uIntSourceType << shift));

    return 0;
}


uint32_t  sysGetInterruptEnableStatus(void)
{
    return (inpw(REG_AIC_IMR));
}


uint32_t  sysGetInterruptEnableStatusH(void)
{
    return (inpw(REG_AIC_IMRH));
}
