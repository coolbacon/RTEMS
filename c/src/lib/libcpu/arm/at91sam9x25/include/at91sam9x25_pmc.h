/*
 * AT91SAM9G25 Power Management and Clock definitions
 *  shoonis 9g25
 *  baconxu@gmail.com
 */
#ifndef __AT91SAM9G25_PMC_H__
#define __AT91SAM9G25_PMC_H__

#include <bits.h>

/***********************************************************************
 *       Power Management and Clock Control Register Offsets
 ***********************************************************************/
#ifdef __cplusplus
 extern "C" {
#endif

int at91sam9g25_get_mainclk(void);
int at91sam9g25_get_slck(void);
int at91sam9g25_get_mck(void);
void PMC_EnablePeripheral( uint32_t dwId );
void PMC_DisablePeripheral( uint32_t dwId );
uint32_t PMC_IsPeriphEnabled( uint32_t dwId );

#ifdef __cplusplus
}
#endif


#endif
