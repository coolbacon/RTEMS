/*
 * shoonis 9g25 i2c driver
 * baconxu@gmail.com 
 */

#include <rtems.h>
#include <bsp.h>
#include <at91sam9x25.h>
#include <at91sam9x25_gpio.h>
#include <at91sam9x25_pmc.h>
#include <bsp/irq.h>
#include <bsp/irq-generic.h>

#define RTEMS_STATUS_CHECKS_USE_PRINTK
#include <rtems/status-checks.h>
#include <rtems/libi2c.h>

#define AT91SAM9G25_I2C_NUM                 2

#define AT91SAM9G25_I2C_0_INDEX             0
#define AT91SAM9G25_IRQ_I2C_0               AT91SAM9G25_INT_TWI0
#define AT91SAM9G25_I2C_0_REGS              TWI0

#define AT91SAM9G25_I2C_1_INDEX             1
#define AT91SAM9G25_IRQ_I2C_1               AT91SAM9G25_INT_TWI1
#define AT91SAM9G25_I2C_1_REGS              TWI1

#define AT91SAM9G25_I2C_2_INDEX             2
#define AT91SAM9G25_IRQ_I2C_2               AT91SAM9G25_INT_TWI2
#define AT91SAM9G25_I2C_2_REGS              TWI2


#define TWI_IDR_ALL (TWI_IDR_TXCOMP\
                    | TWI_IDR_RXRDY\
                    | TWI_IDR_TXRDY)
/*
    | TWI_IDR_SVACC\
    | TWI_IDR_GACC\
    | TWI_IDR_OVRE\
    | TWI_IDR_NACK\
    | TWI_IDR_ARBLST\
    | TWI_IDR_SCL_WS\
    | TWI_IDR_EOSACC)
*/

/*twi clock frequency in Hz*/
#define TWCK_WORK_CLOCK             400000
#define TWI_TIMEOUT_MAX             500000
/* Returns 1 if the TXRDY bit (ready to transmit data) is set in the given status register value.*/
#define TWI_STATUS_TXRDY(status) ((status & TWI_SR_TXRDY) == TWI_SR_TXRDY)

/* Returns 1 if the RXRDY bit (ready to receive data) is set in the given status register value.*/
#define TWI_STATUS_RXRDY(status) ((status & TWI_SR_RXRDY) == TWI_SR_RXRDY)

/* Returns 1 if the TXCOMP bit (transfer complete) is set in the given status register value.*/
#define TWI_STATUS_TXCOMP(status) ((status & TWI_SR_TXCOMP) == TWI_SR_TXCOMP)


#define TWI_ReadByte(t)     ((t)->TWI_RHR)

#define TWI_WriteByte(t, b) ((t)->TWI_THR = (b))

#define TWI_EnableIt(t, s)  ((t)->TWI_IER = (s))

#define TWI_DisableIt(t, s) ((t)->TWI_IDR = (s))

#define TWI_SendSTOPCondition(t) ((t)->TWI_CR = TWI_CR_STOP)

#define TWI_TransferComplete(t) (((t)->TWI_SR & TWI_SR_TXCOMP) == TWI_SR_TXCOMP)

#define TWI_ByteSent(t)         (((t)->TWI_SR & TWI_SR_TXRDY) == TWI_SR_TXRDY)

#define TWI_ByteReceived(t)     (((t)->TWI_SR & TWI_SR_RXRDY) == TWI_SR_RXRDY)

#define DB(x)

typedef Twi at91sam9g25_i2c;

typedef struct {
    volatile at91sam9g25_i2c *regs;
    rtems_device_minor_number minor;
    rtems_vector_number vector;
    rtems_id state_update;
    rtems_id bus_lock;

    uint8_t * volatile data;
    uint8_t * volatile end;
} at91sam9g25_i2c_bus_entry;


static void at91sam9g25_i2c_handler(rtems_irq_hdl_param arg);



static at91sam9g25_i2c_bus_entry gAt91sam9g25_i2c_bus_entry[AT91SAM9G25_I2C_NUM] = {
    #if AT91SAM9G25_I2C_NUM > 0
        {
            regs: TWI0,
            minor: AT91SAM9G25_I2C_0_INDEX,
            //vector: AT91SAM9G25_IRQ_I2C_0,
			vector:ID_TWI0,
        },
    #endif
    #if AT91SAM9G25_I2C_NUM > 1
        {
            regs: TWI1,
            minor: AT91SAM9G25_I2C_1_INDEX,
            //vector: AT91SAM9G25_IRQ_I2C_1,
			vector:ID_TWI1,
        },
    #endif
    #if AT91SAM9G25_I2C_NUM > 2
        {
            regs: TWI2,
            minor: AT91SAM9G25_I2C_2_INDEX,
            //vector: AT91SAM9G25_IRQ_I2C_2,
			vector:ID_TWI2,
        }
    #endif
};

static void TWI_ConfigureMaster(volatile at91sam9g25_i2c* pTwi, uint32_t dwTwCk, uint32_t dwMCk )
{
    uint32_t dwCkDiv = 0;
    uint32_t dwClDiv;
    uint32_t dwOk = 0;

    /* SVEN: TWI Slave Mode Enabled */
    pTwi->TWI_CR = TWI_CR_SVEN ;
    /* Reset the TWI */
    pTwi->TWI_CR = TWI_CR_SWRST ;
    pTwi->TWI_RHR ;

    /* TWI Slave Mode Disabled, TWI Master Mode Disabled. */
    pTwi->TWI_CR = TWI_CR_SVDIS ;
    pTwi->TWI_CR = TWI_CR_MSDIS ;

    /* Set master mode */
    pTwi->TWI_CR = TWI_CR_MSEN ;

    /* Configure clock */
    while ( !dwOk )
    {
        dwClDiv = ((dwMCk / (2 * dwTwCk)) - 4) / (1<<dwCkDiv) ;

        if ( dwClDiv <= 255 )
        {
            dwOk = 1 ;
        }
        else
        {
            dwCkDiv++ ;
        }
    }

    pTwi->TWI_CWGR = 0 ;
    pTwi->TWI_CWGR = (dwCkDiv << 16) | (dwClDiv << 8) | dwClDiv ;
}

static void at91sam9g25_i2c_handler(rtems_irq_hdl_param arg)
{
    at91sam9g25_i2c_bus_entry *e = (at91sam9g25_i2c_bus_entry *)arg;
    volatile at91sam9g25_i2c *regs = e->regs;
    unsigned status;
    uint8_t *data = e->data;
    uint8_t *end = e->end;
    bool notify = false;

    (printk("in i2c isr handler\n"));
    status = regs->TWI_SR;
    status &= regs->TWI_IMR;
    /* Byte received */
    if (TWI_STATUS_RXRDY(status)) {
    
        *data = (uint8_t)TWI_ReadByte(regs);
        data++;
        e->data = data;
        /* check for transfer finish */
        if (data == end)
        {
            TWI_DisableIt(regs, TWI_IDR_ALL);
            notify = true;
            e->data = 0;
            e->end = 0;
        }
        else if (data + 1 == end) 
        {
            regs->TWI_CR = TWI_CR_STOP;
        }
        else
        {

        }
    }
    /* Byte sent*/
    else if (TWI_STATUS_TXRDY(status))
    {
        if (data == end) 
        {
            TWI_DisableIt(regs, TWI_IDR_ALL);
            TWI_EnableIt(regs, TWI_IER_TXCOMP);
            TWI_SendSTOPCondition(regs);
        }
        else
        {
            regs->TWI_THR = *data;
            data++;
            e->data = data;
        }
        notify = false;
    }
    /* Transfer complete*/
    else if (TWI_STATUS_TXCOMP(status))
    {
    
        TWI_DisableIt(regs, TWI_IDR_TXCOMP);
        e->data = (uint8_t *) 0;
        e->end = (uint8_t *) 0;
        notify = true;
    }
	else if (status & TWI_SR_NACK)
	{
		e->data = (uint8_t *) 0;
		e->end = (uint8_t *) 0;
		notify = true;
	}
    
    /* Notify task if necessary */
    if (notify)
    {
        TWI_DisableIt(regs, TWI_IDR_ALL);
        rtems_semaphore_release(e->state_update);
    }
    AIC->AIC_ICCR = 1 << e->vector;
}

static const Pin pinsTwi0[] = {PINS_TWI0};
static const Pin pinsTwi1[] = {PINS_TWI1};

static uint32_t i2c_handle2index(uint32_t handle)
{
	int i;
	at91sam9g25_i2c_bus_entry *e;
	for (i = 0; i < AT91SAM9G25_I2C_NUM; i++)
	{
		e = &gAt91sam9g25_i2c_bus_entry[i];
		if (e->bus_lock == handle)
			return (i);
	}

	return (-1);
}


rtems_status_code at91sam9g25_i2c_init(rtems_device_major_number  major,
                                       rtems_device_minor_number  minor,
                                       void                      *arg)
{
    rtems_status_code sc = RTEMS_SUCCESSFUL;
    int i;
    int mclk = at91sam9g25_get_mck();


	(printk("in at91sam9g25_i2c_init...\n"));
    for (i = 0; i < AT91SAM9G25_I2C_NUM; i++)
    {
        at91sam9g25_i2c_bus_entry *e = &gAt91sam9g25_i2c_bus_entry[i];
        volatile at91sam9g25_i2c *regs = e->regs;
        /* Create semaphore */
        sc = rtems_semaphore_create (rtems_build_name ('I', '2', 'C', '0' + e->minor), 
                                     0,
                                     RTEMS_SIMPLE_BINARY_SEMAPHORE,
                                     0,
                                     &e->state_update);
        RTEMS_CHECK_SC(sc, "create status update semaphore");

        sc = rtems_semaphore_create (rtems_build_name('I', 'B', 'L', '0' + e->minor),
                                     1,
                                     RTEMS_SIMPLE_BINARY_SEMAPHORE,
                                     0,
                                     &e->bus_lock);
        RTEMS_CHECK_SC(sc, "create i2cbus lock semaphore");
        
        /* IO configuration */
        switch (e->minor) 
        {
        case 0:
            if ((PMC->PMC_PCSR & ((uint32_t)1 << ID_TWI0)) != ((uint32_t)1 << ID_TWI0))
            {
                PMC->PMC_PCER = 1 << ID_TWI0;
            }
            PIO_Configure(pinsTwi0, PIO_LISTSIZE(pinsTwi0));
            break;
        case 1:
            if ((PMC->PMC_PCSR & ((uint32_t)1 << ID_TWI1)) != ((uint32_t)1 << ID_TWI1))
            {
                PMC->PMC_PCER = 1 << ID_TWI1;
            }
            PIO_Configure(pinsTwi1, PIO_LISTSIZE(pinsTwi1));
            break;
        case 2:
        default:
            break;
        }
        
        TWI_ConfigureMaster(regs, TWCK_WORK_CLOCK, mclk);
        /* Install interrupt handler and disable this vector */
        TWI_DisableIt(regs, TWI_IDR_ALL);

        AIC->AIC_SMR[e->vector] = 0;
        sc = rtems_interrupt_handler_install(e->vector,
                                             "I2C",
                                             RTEMS_INTERRUPT_UNIQUE,
                                             at91sam9g25_i2c_handler,
                                             e);
    }
    
    
    return RTEMS_SUCCESSFUL;
}

rtems_status_code i2c_write(uint32_t handle, uint8_t slave_addr,
							uint32_t internal_addr, uint32_t internal_addr_size,
							uint8_t *buff, uint32_t length, uint16_t timeout)
{
    at91sam9g25_i2c_bus_entry *e;
    volatile at91sam9g25_i2c *regs;
    uint32_t iaddr;
    uint8_t addr;
	uint32_t numOfWrite = length;
	uint32_t minor;

	minor = i2c_handle2index(handle);
	e = &gAt91sam9g25_i2c_bus_entry[minor];
	regs = e->regs;

    if (minor >= AT91SAM9G25_I2C_NUM
        || buff == NULL
		|| internal_addr_size >= 4)
    {
        return -RTEMS_IO_ERROR;
    }
    
	addr = slave_addr >> 1;
    iaddr = regs->TWI_SR;
	iaddr = internal_addr & 0x00FFFFFFL;
    if (_System_state_Is_up(_System_state_Get()))
    {   
        e->data = (uint8_t *)(buff + 1);
        e->end = (uint8_t *)(buff + numOfWrite);
    
        /* Set slave address and number of internal address bytes. */
        regs->TWI_MMR = 0;
        regs->TWI_MMR = (internal_addr_size << 8) | (addr << 16);
    
        /* Set internal address bytes. */
        regs->TWI_IADR = 0;
        regs->TWI_IADR = iaddr;
    
        /* Write first byte to send.*/
        TWI_WriteByte(regs, *buff);
                
        TWI_EnableIt(regs, TWI_IER_TXRDY);
        DB(printk("send...\n"));
        rtems_semaphore_obtain(e->state_update, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
    }
    else
    {
        uint32_t timeout = 0;

        regs->TWI_MMR = 0;
        regs->TWI_MMR = (internal_addr_size << 8) | (addr << 16);
    
        /* Set internal address bytes. */
        regs->TWI_IADR = 0;
        regs->TWI_IADR = iaddr;
    
        /* Write first byte to send.*/
        TWI_WriteByte(regs, *buff);
        buff++;
        numOfWrite--;

         /* Send all bytes */
        while (numOfWrite > 0) {

            /* Wait before sending the next byte */
            timeout = 0;
            while(!TWI_ByteSent(regs) && (++timeout<TWI_TIMEOUT_MAX));

            TWI_WriteByte(regs, *buff);
            buff++;
            numOfWrite--;
        }

        /* Wait for actual end of transfer */
        timeout = 0;

        /* Send a STOP condition */
        TWI_SendSTOPCondition(regs);

        while(!TWI_TransferComplete(regs) && (++timeout< TWI_TIMEOUT_MAX));
        
    }
    return RTEMS_SUCCESSFUL;
}

rtems_status_code i2c_read(uint32_t handle, uint8_t slave_addr, uint32_t internal_addr, uint32_t internal_addr_size, 
                uint8_t *buff, uint32_t length, uint16_t timeout)
{
    at91sam9g25_i2c_bus_entry *e;
    volatile at91sam9g25_i2c *regs;
    uint32_t iaddr = 0;
    uint8_t addr;
	uint32_t minor;
	uint32_t numOfRead = length;

	minor = i2c_handle2index(handle);
	e = &gAt91sam9g25_i2c_bus_entry[minor];
	regs = e->regs;

    if (minor >= AT91SAM9G25_I2C_NUM
        || buff == NULL
		|| internal_addr_size >= 4)
    {
        return -RTEMS_IO_ERROR;
    }

    addr = slave_addr >> 1;
    iaddr = regs->TWI_SR;
	iaddr = internal_addr & 0x00FFFFFFL;
    if (_System_state_Is_up(_System_state_Get()))
    {
        e->data = buff;
        e->end = buff + numOfRead;

        /* Enable read interrupt and start the transfer */
        TWI_EnableIt(regs, TWI_IER_RXRDY | TWI_IER_NACK);
		
        regs->TWI_MMR = 0;
        regs->TWI_MMR = ((internal_addr_size) << 8) | TWI_MMR_MREAD | (addr << 16);
		
        regs->TWI_IADR = 0;
        regs->TWI_IADR = iaddr;
    
        /* Send START condition */
        regs->TWI_CR = TWI_CR_START;
        if (numOfRead == 1)
            regs->TWI_CR = TWI_CR_STOP;

		printk("in read...\n");
        rtems_semaphore_obtain(e->state_update, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
    }
    else
    {
        int timeout = 0;
        /* Start read*/
        regs->TWI_MMR = 0;
        regs->TWI_MMR = (internal_addr_size << 8) | TWI_MMR_MREAD | (addr << 16);
    
        /* Set internal address bytes */
        
        regs->TWI_IADR = 0;
        regs->TWI_IADR = iaddr;
    
        /* Send START condition */
        regs->TWI_CR = TWI_CR_START;

        /* Read all bytes, setting STOP before the last byte*/
        while (numOfRead > 0)
        {
            /* Last byte ?*/
            if (numOfRead == 1)
            {
                regs->TWI_CR = TWI_CR_STOP;
            }

            /* Wait for byte then read and store it*/
            timeout = 0;
            while( ! TWI_ByteReceived(regs)
                   && (++timeout < TWI_TIMEOUT_MAX) );

            *buff++ = TWI_ReadByte(regs);
            numOfRead--;
        }

        /* Wait for transfer to be complete */
        timeout = 0;
        while( !TWI_TransferComplete(regs)
               && (++timeout < TWI_TIMEOUT_MAX) );
    }
    
    return RTEMS_SUCCESSFUL;
}

uint32_t i2c_open(uint8_t i2c_id)
{
	at91sam9g25_i2c_bus_entry *e = &gAt91sam9g25_i2c_bus_entry[i2c_id];
	if (i2c_id >= AT91SAM9G25_I2C_NUM)
	{
		return (0);
	}
	rtems_semaphore_obtain(e->bus_lock, RTEMS_WAIT, RTEMS_NO_TIMEOUT);

	return (e->bus_lock);
}

uint32_t i2c_close(uint32_t handle)
{
	uint32_t minor;
	at91sam9g25_i2c_bus_entry *e;

	minor = i2c_handle2index(handle);
	e = &gAt91sam9g25_i2c_bus_entry[minor];

    if (minor >= AT91SAM9G25_I2C_NUM)
    {
        return -RTEMS_IO_ERROR;
    }
	rtems_semaphore_release(e->bus_lock);
	return (RTEMS_SUCCESSFUL);
}

uint32_t i2c_setBaudrate(uint32_t handle, uint32_t baudrate)
{
	uint32_t minor;
	at91sam9g25_i2c_bus_entry *e;
	int mclk = at91sam9g25_get_mck();
    volatile at91sam9g25_i2c *regs;

	minor = i2c_handle2index(handle);
	e = &gAt91sam9g25_i2c_bus_entry[minor];
	regs = e->regs;

    if (minor >= AT91SAM9G25_I2C_NUM
		|| baudrate < 1000
		|| baudrate > 400000)
    {
        return (-RTEMS_IO_ERROR);
    }

	TWI_ConfigureMaster(regs, baudrate, mclk);
	return (RTEMS_SUCCESSFUL);
}



