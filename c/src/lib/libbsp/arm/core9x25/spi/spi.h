/*
 *  RTEMS driver for at91sam9g25 SPI
 *  baconxu@Gmail.com
 */

#ifndef __spi_h__
#define __spi_h__

#define SPI0_WP 1
#define SPI1_WP 1

#ifdef __cplusplus
extern "C" {
#endif

rtems_status_code at91sam9g25_spi_init(rtems_device_major_number  major,
                                       rtems_device_minor_number  minor,
                                       void                      *arg);
uint32_t spi_open(uint8_t spi_id, uint8_t cs);
uint32_t spi_close(uint32_t spi_handle);
uint32_t spi_set_baudrate(uint32_t spi_handle, uint32_t baudrate);
uint32_t spi_write(uint32_t spi_handle,
				   uint8_t *tx1, uint8_t *rx1, uint32_t len1,
				   uint8_t *tx2, uint8_t *rx2, uint32_t len2);      

#ifdef __cplusplus
}
#endif


#endif /* __spi_h__ */
