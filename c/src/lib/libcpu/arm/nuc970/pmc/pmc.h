#ifndef __PMC_H__
#define __PMC_H__

#include <stdint.h>


/** \brief  Structure type of clock source
 */
typedef enum CLKn {

    SYS_UPLL     = 1,   /*!< UPLL clock */
    SYS_APLL     = 2,   /*!< APLL clock */
    SYS_SYSTEM   = 3,   /*!< System clock */
    SYS_HCLK1    = 4,   /*!< HCLK1 clock */
    SYS_HCLK234  = 5,   /*!< HCLK234 clock */
    SYS_PCLK     = 6,   /*!< PCLK clock */
    SYS_CPU      = 7,   /*!< CPU clock */

}  CLK_Type;

uint32_t sysGetClock(CLK_Type clk);



#endif

