/*
 * core9x25
 * baconxu@gmail.com 
 */
#ifndef _BSP_H
#define _BSP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <bspopts.h>

#include <rtems.h>
#include <rtems/console.h>
#include <rtems/clockdrv.h>
//#include <libchip/serial.h>
//#include <rtems/libi2c.h>

#define BSP_FEATURE_IRQ_EXTENSION

/* What is the input clock freq in hertz? */
#define BSP_MAIN_FREQ 12000000      /* 12 MHz */
#define BSP_SLCK_FREQ    32768       /* 32.768 KHz */


/*network definition*/
#define SUPPORT_MAC_COMM

#ifdef SUPPORT_MAC_COMM

#define MAC_ADDR_1 0x00
#define MAC_ADDR_2 0x12
#define MAC_ADDR_3 0x34
#define MAC_ADDR_4 0x56
#define MAC_ADDR_5 0x78
#define MAC_ADDR_6 0x00

#define RECV_PACKET_TYPE		(0x0902)
#define SEND_PACKET_TYPE		(0x0901)

typedef void (*TRecvPacketCallback)(void *packet, unsigned length);
void set_recv_packet_callback(TRecvPacketCallback callback);
void send_packet(void *packet, unsigned length);
#endif

/* What is the last interrupt? */
#define BSP_MAX_INT AT91SAM9G25_MAX_INT

//console_tbl *BSP_get_uart_from_minor(int minor);
static inline int32_t BSP_get_baud(void) {return 115200;}

/*
 * Network driver configuration
 */
extern struct rtems_bsdnet_ifconfig *config;

#if 0
#define I2C_DRIVER_TABLE_ENTRY   \
{                                         \
  initialization_entry:  at91sam9g25_i2c_init,  \
  open_entry:            NULL, \
  close_entry:           NULL, \
  read_entry:            NULL, \
  write_entry:           NULL, \
  control_entry:         NULL, \
}

extern rtems_status_code
at91sam9g25_i2c_init(rtems_device_major_number  major,
                     rtems_device_minor_number  minor,
                     void                      *arg);


#define SPI_DRIVER_TABLE_ENTRY \
{                                         \
  initialization_entry:  at91sam9g25_spi_init,  \
  open_entry:            NULL, \
  close_entry:           NULL, \
  read_entry:            NULL, \
  write_entry:           NULL, \
  control_entry:         NULL, \
}
extern rtems_status_code
at91sam9g25_spi_init(rtems_device_major_number  major,
                     rtems_device_minor_number  minor,
                     void                      *arg);

#define CONFIGURE_BSP_PREREQUISITE_DRIVERS \
    I2C_DRIVER_TABLE_ENTRY,\
    SPI_DRIVER_TABLE_ENTRY
#endif

/* Change these to match your board */
int rtems_at91sam9g25_emac_attach(struct rtems_bsdnet_ifconfig *config, void *attaching);
#define RTEMS_BSP_NETWORK_DRIVER_NAME	"eth0"
#define RTEMS_BSP_NETWORK_DRIVER_ATTACH	rtems_at91sam9g25_emac_attach

#ifdef __cplusplus
}
#endif

#endif /* _BSP_H */

