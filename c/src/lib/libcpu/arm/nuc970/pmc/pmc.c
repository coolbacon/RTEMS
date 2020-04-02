/*
nuc970.h
baconxu@gmail.com
 */

#include <rtems.h>
#include <bsp.h>
#include <nuc970.h>
#include <assert.h>
#include "pmc.h"

uint32_t sysGetPLL(uint32_t reg)
{
    uint32_t N,M,P;

    N =((inpw(reg) & 0x007F)>>0)+1;
    M =((inpw(reg) & 0x1F80)>>7)+1;
    P =((inpw(reg) & 0xE000)>>13)+1;

    return (12*N/(M*P));    /* 12MHz HXT */
}
/// @endcond HIDDEN_SYMBOLS

/**
 *  @brief  system Timer - install WDT interrupt handler
 *
 *  @param[in]  clk   clock source. \ref CLK_Type
 *
 *  @return   MHz
 */
uint32_t sysGetClock(CLK_Type clk)
{
    uint32_t src, divS, divN, reg, div;

    switch(clk)
    {
        case SYS_UPLL:
            return sysGetPLL(REG_CLK_UPLLCON);

        case SYS_APLL:
            return sysGetPLL(REG_CLK_APLLCON);

        case SYS_SYSTEM:
        {
            reg = inpw(REG_CLK_DIVCTL0);
            switch (reg & 0x18)
            {
                case 0x0:
                    src = 12;   /* HXT */
                    break;
                case 0x10:
                    src = sysGetPLL(REG_CLK_APLLCON);
                    break;
                case 0x18:
                    src = sysGetPLL(REG_CLK_UPLLCON);
                    break;
                default:
                    return 0;
            }
            divS = (reg & 0x7) + 1;
            divN = ((reg & 0xf00) >> 8) + 1;
            return (src / divS / divN);
        }

        case SYS_HCLK1:
        {
            reg = inpw(REG_CLK_DIVCTL0);
            switch (reg & 0x18)
            {
                case 0x0:
                    src = 12;   /* HXT */
                    break;
                case 0x10:
                    src = sysGetPLL(REG_CLK_APLLCON);
                    break;
                case 0x18:
                    src = sysGetPLL(REG_CLK_UPLLCON);
                    break;
                default:
                    return 0;
            }
            divS = (reg & 0x7) + 1;
            divN = ((reg & 0xf00) >> 8) + 1;
            return (src / divS / divN / 2);
        }

        case SYS_HCLK234:
        {
            reg = inpw(REG_CLK_DIVCTL0);
            switch (reg & 0x18)
            {
                case 0x0:
                    src = 12;   /* HXT */
                    break;
                case 0x10:
                    src = sysGetPLL(REG_CLK_APLLCON);
                    break;
                case 0x18:
                    src = sysGetPLL(REG_CLK_UPLLCON);
                    break;
                default:
                    return 0;
            }
            divS = (reg & 0x7) + 1;
            divN = ((reg & 0xf00) >> 8) + 1;
            div = ((reg & 0xf00000) >> 20) + 1;
            return (src / divS / divN / 2 / div);
        }

        case SYS_PCLK:
        {
            reg = inpw(REG_CLK_DIVCTL0);
            switch (reg & 0x18)
            {
                case 0x0:
                    src = 12;   /* HXT */
                    break;
                case 0x10:
                    src = sysGetPLL(REG_CLK_APLLCON);
                    break;
                case 0x18:
                    src = sysGetPLL(REG_CLK_UPLLCON);
                    break;
                default:
                    return 0;
            }
            divS = (reg & 0x7) + 1;
            divN = ((reg & 0xf00) >> 8) + 1;
            div = ((reg & 0xf000000) >> 24) + 1;
            return (src / divS / divN / 2 / div);
        }
        case SYS_CPU:
        {
            reg = inpw(REG_CLK_DIVCTL0);
            switch (reg & 0x18)
            {
                case 0x0:
                    src = 12;   /* HXT */
                    break;
                case 0x10:
                    src = sysGetPLL(REG_CLK_APLLCON);
                    break;
                case 0x18:
                    src = sysGetPLL(REG_CLK_UPLLCON);
                    break;
                default:
                    return 0;
            }
            divS = (reg & 0x7) + 1;
            divN = ((reg & 0xf00) >> 8) + 1;
            div = ((reg & 0xf0000) >> 16) + 1;
            return (src / divS / divN / div);
        }

        default:
            ;
    }
    return 0;
}