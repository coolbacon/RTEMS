/**

nuc977
baconxu@gmail.com

 */

 
#ifndef LIBBSP_ARM_NUC970_BSP_H
#define LIBBSP_ARM_NUC970_BSP_H

#include <bspopts.h>
#include <bsp/default-initial-extension.h>

#include <rtems.h>
#include <rtems/console.h>
#include <rtems/clockdrv.h>


#ifdef __cplusplus
extern "C" {
#endif


#define BSP_FEATURE_IRQ_EXTENSION

/* What is the input clock freq in hertz? */
#define BSP_MAIN_FREQ 12000000      /* 12 MHz */
#define BSP_SLCK_FREQ    32768      /* 32.768 KHz */

/* What is the last interrupt? */
#define BSP_MAX_INT 	(61)

/*
 * forward reference the type to avoid conflicts between libchip serial
 * and libchip rtc get and set register types.
 */
typedef struct _console_tbl console_tbl;
console_tbl *BSP_get_uart_from_minor(int minor);

static inline int32_t BSP_get_baud(void) {return 115200;}

#define ST_PIMR_PIV	33	/* 33 ticks of the 32.768Khz clock ~= 1msec */

/**
 * @brief Network driver configuration
 */
extern struct rtems_bsdnet_ifconfig *config;

/* Change these to match your board */

///int rtems_at91sam9x25_emac_attach(struct rtems_bsdnet_ifconfig *config, int attching);


#define RTEMS_BSP_NETWORK_DRIVER_NAME  "eth0"
//#define RTEMS_BSP_NETWORK_DRIVER_ATTACH	rtems_at91sam9x25_emac_attach

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* _BSP_H */

