#include <libchip/rtc.h>
#include <libchip/pcf8563.h>
#include <libchip/pcf2129.h>
#include <bsp/i2c.h>
#include <bsp/spi.h>

#define PCF8563_I2C_BUS_ID          1
#define PCF2129_SPI_BUS_ID			1
#define PCF2129_SPI_CS				7
#define PCF2129_SPI_CLK				1000000
#define PCF2129_SPI_TIME_ADDR		0x03

rtems_device_minor_number RTC_Minor = 0;
size_t RTC_Count = 2;

uint8_t binary2bcd(uint8_t inData)
{
	uint8_t value = inData;

		
	uint8_t lowBit = value % 10;
	uint8_t highBit = value / 10;

	return ((highBit << 4) | lowBit);
}


uint8_t bcd2binary(uint8_t inData)
{
	int value = inData;
	
	uint8_t firstPlace = value & 0x0F;
	uint8_t secondPlace = ((value & 0xF0) >> 4);

	return (secondPlace * 10 + firstPlace);
}


void  pcf8563_set_register(
  uintptr_t   ulCtrlPort,
  uint8_t     ucRegNum,
  uint32_t    ucData
)
{
    uint8_t data;
    int sc;
	uint32_t handle;

    if ((ucRegNum != PCF8563_STATUS1) && (ucRegNum != PCF8563_STATUS2))
        data = binary2bcd(ucData);
    else
        data = (uint8_t)ucData;

    /* Initialize; switch continuous conversion on */
	handle = i2c_open(PCF8563_I2C_BUS_ID);
    sc = i2c_write(handle, ulCtrlPort, ucRegNum, 1, &data, 1, 0);
	sc = i2c_close(handle);
}



uint32_t  pcf8563_get_register(
  uintptr_t   ulCtrlPort,
  uint8_t     ucRegNum
)
{
    uint8_t regval;
    int sc;
	uint32_t handle;

	handle = i2c_open(PCF8563_I2C_BUS_ID);
    i2c_read(handle, ulCtrlPort, ucRegNum, 1, &regval, 1, 0);
	i2c_close(handle);

    /* Initialize; switch continuous conversion on */
    if (ucRegNum != PCF8563_STATUS1 && ucRegNum != PCF8563_STATUS2)
    {
        if (ucRegNum == PCF8563_YEAR) 
            regval &= 0xFF;
        else if (ucRegNum == PCF8563_MONTH)
            regval &= 0x1F;
        else if (ucRegNum == PCF8563_DATE) 
            regval &= 0x3F;
        else if (ucRegNum == PCF8563_HOUR) 
            regval &= 0x3F;
        else if (ucRegNum == PCF8563_MINUTE)
            regval &= 0x7F;
        else if (ucRegNum == PCF8563_SECOND)
            regval &= 0x7F;
        else if (ucRegNum == PCF8563_DAY_OF_WEEK)
            regval &= 0x07;
        else if (ucRegNum == PCF8563_VL)
            regval &= 0x80;
        else
        {
        }
        regval = bcd2binary(regval);
    }

    if (ucRegNum == PCF8563_VL)
        regval = (regval != 0)? 1:0;

    return regval;
}

bool pcf8563_probe(int minor)
{
    return true;
}


/*------------------------------------------------------------*/

void  pcf2129_set_register(
  uintptr_t   ulCtrlPort,
  uint8_t     ucRegNum,
  uint32_t    ucData
)
{
    static uint8_t data[32];
    int sc;
	uint32_t handle;
	int isUpdate;

	isUpdate = ucRegNum & PCF2129_REFRESH_DATA;
	ucRegNum &= 0x7F;

	if (ucRegNum == PCF2129_YEAR) 
		data[7] = binary2bcd(ucData % 100);
	else if (ucRegNum == PCF2129_MONTH)
		data[6] = binary2bcd(ucData);
	else if (ucRegNum == PCF2129_DATE) 
		data[4] = binary2bcd(ucData);
	else if (ucRegNum == PCF2129_HOUR) 
		data[3] = binary2bcd(ucData);
	else if (ucRegNum == PCF2129_MINUTE)
		data[2] = binary2bcd(ucData);
	else if (ucRegNum == PCF2129_SECOND)
		data[1] = binary2bcd(ucData);
	else if (ucRegNum == PCF2129_DAY_OF_WEEK)
		data[5] = binary2bcd(ucData);
	else
	{
	}

    if (isUpdate)
	{
		handle = spi_open(PCF2129_SPI_BUS_ID, PCF2129_SPI_CS);	
		spi_set_baudrate(handle, PCF2129_SPI_CLK);
	
		data[0] = 0x23;
		spi_write(handle, data, data + 16, 12, NULL, NULL, 0);
		spi_close(handle);
	}
}



uint32_t  pcf2129_get_register(
  uintptr_t   ulCtrlPort,
  uint8_t     ucRegNum
)
{
    uint8_t regval;
    int sc;
	uint32_t handle;
	static uint8_t data[32];

	if (ucRegNum & PCF2129_REFRESH_DATA)
	{
		handle = spi_open(PCF2129_SPI_BUS_ID, PCF2129_SPI_CS);
		if (handle == 0)
			return (0);
	
		spi_set_baudrate(handle, PCF2129_SPI_CLK);
	
		//memset(data, 0, 32);
		data[0] = 0xa3;
		spi_write(handle, data, data + 16, 12, NULL, NULL, 0);
		spi_close(handle);
		printk("second:%x\n", data[1]);
	}

	ucRegNum &= 0x7F;

    /* Initialize; switch continuous conversion on */
    if (ucRegNum != PCF2129_STATUS1 && ucRegNum != PCF8563_STATUS2)
    {
        if (ucRegNum == PCF2129_YEAR) 
            regval = data[7] & 0xFF;
        else if (ucRegNum == PCF2129_MONTH)
            regval = data[6] & 0x1F;
        else if (ucRegNum == PCF2129_DATE) 
            regval = data[4] & 0x3F;
        else if (ucRegNum == PCF2129_HOUR) 
            regval = data[3] & 0x3F;
        else if (ucRegNum == PCF2129_MINUTE)
            regval = data[2] & 0x7F;
        else if (ucRegNum == PCF2129_SECOND)
            regval = data[1] & 0x7F;
        else if (ucRegNum == PCF2129_DAY_OF_WEEK)
            regval = data[5] & 0x0F;
        else if (ucRegNum == PCF2129_VL)
            regval = data[1] & 0x80;
        else
        {
        }
        regval = bcd2binary(regval);
    }

    if (ucRegNum == PCF2129_VL)
        regval = (regval != 0)? 1:0;

    return regval;
}

bool pcf2129_probe(int minor)
{
	return true;
}

rtc_tbl RTC_Table[] = {
  { 
    "/dev/rtc0",                /* sDeviceName */
    RTC_CUSTOM,                 /* deviceType */
    &pcf8563_fns,               /* pDeviceFns */
    pcf8563_probe,              /* deviceProbe */
    (void *) 0,                 /* pDeviceParams */
    PCF8563_ADDR,               /* ulCtrlPort1 */
    0,                          /* ulDataPort */
    (getRegister_f)pcf8563_get_register,       /* getRegister */
    (setRegister_f)pcf8563_set_register,       /* setRegister */
  },
  {
	"/dev/rtc1",                /* sDeviceName */
    RTC_CUSTOM,                 /* deviceType */
    &pcf2129_fns,               /* pDeviceFns */
    pcf2129_probe,              /* deviceProbe */
    (void *) 0,                 /* pDeviceParams */
    PCF2129_ADDR,               /* ulCtrlPort1 */
    0,                          /* ulDataPort */
    (getRegister_f)pcf2129_get_register,       /* getRegister */
    (setRegister_f)pcf2129_set_register,       /* setRegister */
  }
};



