/*
 * shoonis 9g25 i2c driver's header 
 * baconxu@gmail.com 
 */

#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

uint32_t i2c_open(uint8_t i2c_id);

uint32_t i2c_close(uint32_t handle);

uint32_t i2c_setBaudrate(uint32_t handle, uint32_t baudrate);

rtems_status_code i2c_write(uint32_t handle, uint8_t slave_addr,
							uint32_t internal_addr, uint32_t internal_addr_size,
							uint8_t *buff, uint32_t length, uint16_t timeout);

rtems_status_code i2c_read(uint32_t handle, uint8_t slave_addr,
						   uint32_t internal_addr, uint32_t internal_addr_size,
						   uint8_t *buff, uint32_t length, uint16_t timeout);
#ifdef __cplusplus
}
#endif

#endif



