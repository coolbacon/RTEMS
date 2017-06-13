/*
    RTEMS.CN SAM9X25
    it's dbg uart driver.
    baconxu@gmail.com
*/

#include <bsp.h>
#include <rtems/libio.h>
#include <termios.h>

#include <at91sam9x25.h>
#include <at91sam9x25_gpio.h>
#include <at91sam9x25_usart.h>
#include <at91sam9x25_pmc.h>
#include <bsp/irq.h>
#include <rtems/bspIo.h>
#include <libchip/serial.h>
#include <libchip/sersupp.h>
#include <bsp/dma.h>

#define AT91SAM9G25_USART_NUM		4
#define USART_DMA_RX_BUFFER_SIZE	1
#define USART_DMA_TX_BUFFER_SIZE	20
#define USART_TX_BUFFER_SIZE		512
#define USART_UART_BAUDRATE			115200
#define USART_TIMEOUT_RATIO			15

#define PIN_USART2_RTS_PIN	{PIO_PC29, PIOC, ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT}
#define PIN_USART3_TXD 		{PIO_PC22B_TXD3, PIOC, ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT}
#define PIN_USART3_RXD		{PIO_PC23B_RXD3, PIOC, ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT}
#define PIN_USART3_RTS		{PIO_PC24B_RTS3, PIOC, ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT}

#define USART0_PINS			PIN_USART0_TXD, PIN_USART0_RXD, PIN_USART0_RTS
#define USART1_PINS			PIN_USART1_TXD,	PIN_USART1_RXD, PIN_USART1_RTS
#define USART2_PINS			PIN_USART2_TXD, PIN_USART2_RXD, PIN_USART2_RTS_PIN
#define USART3_PINS			PIN_USART3_TXD, PIN_USART3_RXD, PIN_USART3_RTS


typedef Usart at91sam9g25_usart_regs_t;
extern volatile void *at91sam9g25_usart_rxbufs;
extern volatile void *at91sam9g25_usart_txbufs;

static const Pin gUsart0Pins[] = {USART0_PINS};
static const Pin gUsart1Pins[] = {USART1_PINS};
static const Pin gUsart2Pins[] = {USART2_PINS};
static const Pin gUsart3Pins[] = {USART3_PINS};

#define set_usart_pin(i) \
	do\
	{\
		switch ((i))\
		{\
		case 0:\
			PIO_Configure(gUsart0Pins, PIO_LISTSIZE(gUsart0Pins));\
			break;\
		case 1:\
			PIO_Configure(gUsart1Pins, PIO_LISTSIZE(gUsart1Pins));\
			break;\
		case 2:\
			PIO_Configure(gUsart2Pins, PIO_LISTSIZE(gUsart2Pins));\
			break;\
		case 3:\
			PIO_Configure(gUsart3Pins, PIO_LISTSIZE(gUsart3Pins));\
			break;\
		default:\
			printk("error occur!\n");\
		}\
	} while (0)

typedef struct tagAt91sam9g25_usart {
	at91sam9g25_usart_regs_t *regs;
	uint32_t id;
	int major;
	int minor;

	uint32_t usartDmaTxChannel;
	uint32_t usartDmaRxChannel;

	rtems_id            bus_lock;
    rtems_id            state_update;

	int result;
	int rxTimeout;
} TAt91sam9g25_usart;


static void _DmaRxCallback(uint8_t status, void* pArg);
static void _DmaTxCallback(uint8_t status, void *pArg);
static void usart_int_process(void *param);

TAt91sam9g25_usart at91sam9g25_usart[AT91SAM9G25_USART_NUM] = 
{
	{
		regs: USART0,
		id: ID_USART0,
		rxTimeout: -1,
	},
	{
		regs: USART1,
		id: ID_USART1,
		rxTimeout: -1,
	},
	{
		regs: USART2,
		id: ID_USART2,
		rxTimeout: -1,
	},
	{
		regs: USART3,
		id: ID_USART3,
		rxTimeout: -1,
	}
};

#if 0

/* static function prototypes */
static int     usart_first_open(int major, int minor, void *arg);
static int     usart_last_close(int major, int minor, void *arg);
static ssize_t usart_write_support_int(int minor, const char *buf, size_t len);
static void    usart_init(int minor);
static void    usart_write_polled(int minor, char c);
static int     usart_set_attributes(int minor, const struct termios *t);
static uint32_t usart_get_base(int minor);
static uint32_t dummyRead(uint32_t value);


/* Pointers to functions for handling the UART polled. */
console_fns usart_fns = {
	libchip_serial_default_probe,       /* deviceProbe */
	usart_first_open,                   /* deviceFirstOpen */
	usart_last_close,                   /* deviceLastClose */
	NULL,                  			  /* deviceRead */
	usart_write_support_int,         	  /* deviceWrite */
	usart_init,                         /* deviceInitialize */
	NULL,                 				/* deviceWritePolled */
	usart_set_attributes,               /* deviceSetAttributes */
	TRUE                                /* TRUE if interrupt driven, FALSE if not. */
};

uint32_t usart_get_base(int minor)
{
  console_tbl *console_entry;
  uint32_t port;

  console_entry = BSP_get_uart_from_minor(minor);

  if (console_entry == NULL)
    return 0;

  port = (uint32_t) console_entry->ulCtrlPort1;

  return port;
}

/*
 * Functions called via callbacks (i.e. the ones in uart_fns
 */

/*
 * This is called the first time each device is opened. Since
 * the driver is polled, we don't have to do anything. If the driver
 * were interrupt driven, we'd enable interrupts here.
 */
static int usart_first_open(int major, int minor, void *arg)
{
	uint32_t index;
	TAt91sam9g25_usart *usart;
#ifdef USART_USE_DMA
	sDmaTransferDescriptor td;
#endif

	index = usart_get_base(minor);
	if ( index == -1 )
		return -1;
	usart = &at91sam9g25_usart[index];
	at91sam9g25_usart[index].major = major;


	/* XXX port isn't being initialized or enabled */
	/*initialize the io port*/
#ifdef USART_USE_DMA
	td.dwSrcAddr = (uint32_t)&usart->regs->US_RHR;
	td.dwDstAddr = (uint32_t) usart->dmaRxBuf;
	td.dwCtrlA   = USART_DMA_RX_BUFFER_SIZE
					| DMAC_CTRLA_SRC_WIDTH_BYTE | DMAC_CTRLA_DST_WIDTH_BYTE;
	td.dwCtrlB   = DMAC_CTRLB_SRC_DSCR | DMAC_CTRLB_DST_DSCR
					| DMAC_CTRLB_FC_PER2MEM_DMA_FC
					| DMAC_CTRLB_SRC_INCR_FIXED
					| DMAC_CTRLB_DST_INCR_INCREMENTING;
	td.dwDscAddr = 0;
	DMAD_PrepareSingleTransfer(&gDmad, usart->usartDmaRxChannel, &td);
	DMAD_StartTransfer(&gDmad, usart->usartDmaRxChannel);
#else
	if (usart->regs->US_CSR & US_CSR_RXRDY)
		dummyRead(usart->regs->US_RHR);

#endif
	return 0;
}

/*
 * This is called the last time each device is closed.  Since
 * the driver is polled, we don't have to do anything. If the driver
 * were interrupt driven, we'd disable interrupts here.
 */
static int usart_last_close(int major, int minor, void *arg)
{
	uint32_t index;
	at91sam9g25_usart_regs_t *usart;

	index = usart_get_base(minor);
	if (index == -1)
		return -1;
	usart = at91sam9g25_usart[index].regs;

	return 0;
}

/*
 *  Write character out
 */
static void usart_write_polled(int minor, char c)
{
	uint32_t index;
	at91sam9g25_usart_regs_t *usart;
	
	index = usart_get_base(minor);
	if (index == -1)
		return;
	usart = at91sam9g25_usart[index].regs;

	while ((usart->US_CSR & US_CSR_TXEMPTY) == 0);
	usart->US_THR = c;
}

/*
 * Write buffer to UART
 *
 * return 1 on success, -1 on error
 */
static ssize_t usart_write_support_int(int minor, const char *buf, size_t len)
{
#ifdef USART_USE_DMA
	uint32_t index;
	at91sam9g25_usart_regs_t *usart;
	console_data *d = &Console_Port_Data [minor];
	int i = 0;
	int out = len > USART_DMA_TX_BUFFER_SIZE ? USART_DMA_TX_BUFFER_SIZE : len;

	/*
	*  Verify the minor number
	*/
	index = usart_get_base(minor);
	if (index == -1)
		return -1;

	usart = at91sam9g25_usart[index].regs;
	if(!at91sam9g25_usart[index].isFinishedTx)
	{
		out = 0;
	}

	for (i = 0; i < out; ++i)
	{
		at91sam9g25_usart[index].dmaTxBuf[i] = buf[i];
	}
	
	if (out > 0)
	{
		sDmaTransferDescriptor td;
		at91sam9g25_usart[index].isFinishedTx = 0;
		d->pDeviceContext = (void *)out;
		d->bActive = true;

			
		td.dwSrcAddr = (uint32_t) at91sam9g25_usart[index].dmaTxBuf;
		td.dwDstAddr = (uint32_t)&usart->US_THR;
		td.dwCtrlA   = out
						| DMAC_CTRLA_SRC_WIDTH_BYTE | DMAC_CTRLA_DST_WIDTH_BYTE;
		td.dwCtrlB   = DMAC_CTRLB_SRC_DSCR | DMAC_CTRLB_DST_DSCR
						| DMAC_CTRLB_FC_MEM2PER_DMA_FC
						| DMAC_CTRLB_SRC_INCR_INCREMENTING
						| DMAC_CTRLB_DST_INCR_FIXED;
		td.dwDscAddr = 0;
		DMAD_PrepareSingleTransfer(&gDmad, at91sam9g25_usart[index].usartDmaTxChannel, &td);
		DMAD_StartTransfer(&gDmad, at91sam9g25_usart[index].usartDmaTxChannel);
		//printk("|");

	}

	return 0;
#else
	uint32_t index;
	at91sam9g25_usart_regs_t *usart;
	console_data *d = &Console_Port_Data [minor];
	int out = len > 1 ? 1 : len;

	/*
	*  Verify the minor number
	*/
	index = usart_get_base(minor);
	if (index == -1)
		return -1;

	usart = at91sam9g25_usart[index].regs;
	if(!at91sam9g25_usart[index].isFinishedTx)
	{
		out = 0;
	}
	else
	{
		if(at91sam9g25_usart[index].id == ID_USART2)
		{
			int i;
			PIO_Set(&gUsart2Pins[2]);
			for (i = 0; i < 20; i++)
				dummyRead(i);
		}
		usart->US_THR = *buf;
		usart->US_IER = US_IER_TXRDY;
		at91sam9g25_usart[index].isFinishedTx = 0;
		d->pDeviceContext = (void *)out;
		d->bActive = true;
	}

	return 0;
#endif
}


/* use the values defined in linkcmds for our use of SRAM */

/* Set up the UART. */
static void usart_init(int minor)
{
	uint32_t index;
	at91sam9g25_usart_regs_t *usart;
	rtems_status_code sc;
#ifdef USART_USE_DMA
	uint32_t dwCfg;
    uint8_t iController;
#endif

	index = usart_get_base(minor);
	if ( index == -1 )
		return;

	at91sam9g25_usart[index].minor = minor;
	usart = at91sam9g25_usart[index].regs;
#ifdef USART_USE_DMA
	DMAD_Initialize(&gDmad, 0);
    /* Enable interrupts */
	AIC->AIC_SMR[AT91SAM9G25_INT_DMAC0] = ((AIC_SMR_PRIOR_LOWEST << AIC_SMR_PRIOR_Pos) & AIC_SMR_PRIOR_Msk) 
                                | (AIC_SMR_SRCTYPE_INT_EDGE_TRIGGERED & AIC_SMR_SRCTYPE_Msk);
	AIC->AIC_SMR[AT91SAM9G25_INT_DMAC1] = ((AIC_SMR_PRIOR_LOWEST << AIC_SMR_PRIOR_Pos) & AIC_SMR_PRIOR_Msk) 
                                | (AIC_SMR_SRCTYPE_INT_EDGE_TRIGGERED & AIC_SMR_SRCTYPE_Msk);

    sc = rtems_interrupt_handler_install(AT91SAM9G25_INT_DMAC0,
                                         "DMA0",
                                         RTEMS_INTERRUPT_UNIQUE,
                                         (rtems_interrupt_handler)DMAD_Handler,
                                         &gDmad);

    sc = rtems_interrupt_handler_install(AT91SAM9G25_INT_DMAC1,
                                         "DMA1",
                                         RTEMS_INTERRUPT_UNIQUE,
                                         (rtems_interrupt_handler)DMAD_Handler,
                                         (&gDmad));
	PMC_EnablePeripheral(at91sam9g25_usart[index].id);
	PMC_EnablePeripheral(ID_PIOA);
	PMC_EnablePeripheral(ID_PIOC);
	set_usart_pin(index);


    /* Allocate DMA channels for USART */
    at91sam9g25_usart[index].usartDmaTxChannel = DMAD_AllocateChannel( &gDmad,
                                              DMAD_TRANSFER_MEMORY, at91sam9g25_usart[index].id);
    at91sam9g25_usart[index].usartDmaRxChannel = DMAD_AllocateChannel( &gDmad,
                                              at91sam9g25_usart[index].id, DMAD_TRANSFER_MEMORY);
    if (   at91sam9g25_usart[index].usartDmaTxChannel == DMAD_ALLOC_FAILED 
        || at91sam9g25_usart[index].usartDmaRxChannel == DMAD_ALLOC_FAILED )
    {
        printk("DMA channel allocat error\n\r");
        while(1);
    }
    /* Set RX callback */
    DMAD_SetCallback(&gDmad, at91sam9g25_usart[index].usartDmaRxChannel,
                    (DmadTransferCallback)_DmaRxCallback, &at91sam9g25_usart[index]);

	/*set tx callback*/
	DMAD_SetCallback(&gDmad, at91sam9g25_usart[index].usartDmaTxChannel,
					 (DmadTransferCallback)_DmaTxCallback, &at91sam9g25_usart[index]);

    /* Configure DMA RX channel */
    iController = (at91sam9g25_usart[index].usartDmaRxChannel >> 8);
    dwCfg = 0
           | DMAC_CFG_SRC_PER(DMAIF_Get_ChannelNumber( iController, at91sam9g25_usart[index].id, DMAD_TRANSFER_RX ))           
           | DMAC_CFG_SRC_H2SEL
           | DMAC_CFG_SOD
           | DMAC_CFG_FIFOCFG_ALAP_CFG;
    DMAD_PrepareChannel( &gDmad, at91sam9g25_usart[index].usartDmaRxChannel, dwCfg );
    /* Configure DMA TX channel */
    iController = (at91sam9g25_usart[index].usartDmaTxChannel >> 8);
    dwCfg = 0           
           | DMAC_CFG_DST_PER(DMAIF_Get_ChannelNumber( iController, at91sam9g25_usart[index].id, DMAD_TRANSFER_TX ))
           | DMAC_CFG_DST_H2SEL
           | DMAC_CFG_SOD
           | DMAC_CFG_FIFOCFG_ALAP_CFG;
    DMAD_PrepareChannel( &gDmad, at91sam9g25_usart[index].usartDmaTxChannel, dwCfg );
#else
	PMC_EnablePeripheral(at91sam9g25_usart[index].id);
	PMC_EnablePeripheral(ID_PIOA);
	PMC_EnablePeripheral(ID_PIOC);
	set_usart_pin(index);
	AIC->AIC_SMR[at91sam9g25_usart[index].id] = ((AIC_SMR_PRIOR_LOWEST << AIC_SMR_PRIOR_Pos) & AIC_SMR_PRIOR_Msk) 
                                | (AIC_SMR_SRCTYPE_INT_EDGE_TRIGGERED & AIC_SMR_SRCTYPE_Msk);
	usart->US_IDR = 0xFFFFFFFF;
	sc = rtems_interrupt_handler_install(at91sam9g25_usart[index].id,
                                         "UART",
                                         RTEMS_INTERRUPT_UNIQUE,
                                         usart_int_process,
                                         &at91sam9g25_usart[index]);
	dummyRead(usart->US_CSR);
	dummyRead(usart->US_RHR);
	if(at91sam9g25_usart[index].id == ID_USART2)
	{
		int i;
		PIO_Clear(&gUsart2Pins[2]);
		for (i = 0; i < 10; i++)
			dummyRead(i);
	}

	usart->US_IER = US_IER_RXRDY;
#endif
}

/* This is for setting baud rate, bits, etc. */
static int usart_set_attributes(int minor, const struct termios *t)
{
	uint32_t      brgr;
	uint32_t      mode, baud, baud_requested;
	uint32_t		index;
	at91sam9g25_usart_regs_t *usart;
	
	index = usart_get_base(minor);
	if (index == -1)
		return -1;
	usart = at91sam9g25_usart[index].regs;
	
	mode = US_MR_USART_MODE_RS485 
		| US_MR_PAR_NO 
		| US_MR_CHMODE_NORMAL;
	switch (t->c_cflag & CSIZE)
	{
	case CS5:
		mode |= US_MR_CHRL_5_BIT;
		break;
	case CS6:
		mode |= US_MR_CHRL_6_BIT;
		break;
	case CS7:
		mode |= US_MR_CHRL_7_BIT;
		break;
	case CS8:
		mode |= US_MR_CHRL_8_BIT;
		break;
	}

	if (t->c_cflag & CSTOPB)
		mode |= US_MR_NBSTOP_2_BIT;
	else
		mode |= US_MR_NBSTOP_1_BIT;

	if (t->c_cflag & PARENB)
	{
		if (t->c_cflag & PARODD)
			mode |= US_MR_PAR_ODD;
		else
			mode |= US_MR_PAR_EVEN;
	}
	else
		mode |= US_MR_PAR_NO;

	baud_requested = t->c_cflag & CBAUD;

	if (!baud_requested)
	{
		baud_requested = B115200;
	}
	baud = rtems_termios_baud_to_number(baud_requested);
	brgr = at91sam9g25_get_mck() / 16 / baud;

	if (brgr > 65535)
	{
		brgr /= 8;
		mode |= US_MR_USCLKS_DIV;
	}
	else
		mode |= US_MR_USCLKS_MCK;

	usart->US_CR = US_CR_RSTRX | US_CR_RSTTX | US_CR_RXDIS | US_CR_TXDIS | US_CR_RTSDIS;
	usart->US_MR = mode;
	usart->US_BRGR = brgr;
	dummyRead(usart->US_RHR);
	usart->US_CR = US_CR_TXEN | US_CR_RXEN | US_CR_RTSEN;

	return 0;
}


#ifdef USART_USE_DMA
/**
 *  \brief Callback function for DMA receiving.
 */
static void _DmaRxCallback( uint8_t status, void* pArg )
{
	TAt91sam9g25_usart *usart = (TAt91sam9g25_usart*)pArg;
    sDmaTransferDescriptor td;

	rtems_termios_enqueue_raw_characters(Console_Port_Data[usart->minor].termios_data, 
										 (char *)usart->dmaRxBuf,
										 USART_DMA_RX_BUFFER_SIZE);
	td.dwSrcAddr = (uint32_t)&usart->regs->US_RHR;
	td.dwDstAddr = (uint32_t) usart->dmaRxBuf;
	td.dwCtrlA   = USART_DMA_RX_BUFFER_SIZE
					| DMAC_CTRLA_SRC_WIDTH_BYTE | DMAC_CTRLA_DST_WIDTH_BYTE;
	td.dwCtrlB   = DMAC_CTRLB_SRC_DSCR | DMAC_CTRLB_DST_DSCR
					| DMAC_CTRLB_FC_PER2MEM_DMA_FC
					| DMAC_CTRLB_SRC_INCR_FIXED
					| DMAC_CTRLB_DST_INCR_INCREMENTING;
	td.dwDscAddr = 0;
	DMAD_PrepareSingleTransfer(&gDmad, usart->usartDmaRxChannel, &td);
	DMAD_StartTransfer(&gDmad, usart->usartDmaRxChannel);  
}

static void _DmaTxCallback(uint8_t status, void *pArg)
{
	TAt91sam9g25_usart *usart = (TAt91sam9g25_usart*)pArg;
	console_data *cd = &Console_Port_Data[usart->minor];
	int rv = 0;
	int chars_to_dequeue = (int)cd->pDeviceContext;

	usart->isFinishedTx = 1;

	rv = rtems_termios_dequeue_characters(cd->termios_data, chars_to_dequeue);
	if (!rv)
	{
		cd->pDeviceContext = 0;
		cd->bActive = false;
	}
}

#else

void usart_int_process(void *param)
{
	TAt91sam9g25_usart *usart = (TAt91sam9g25_usart*)param;
	at91sam9g25_usart_regs_t *regs;
	uint32_t status;
	console_data *cd = &Console_Port_Data[usart->minor];
	int rv = 0;
	int chars_to_dequeue = (int)cd->pDeviceContext;
	char c;

	regs = usart->regs;
	status = regs->US_CSR;
	status = status & regs->US_IMR;
	if (status & US_CSR_RXRDY)
	{
		c = regs->US_RHR;
		rtems_termios_enqueue_raw_characters(cd->termios_data, (char *)&c, 1);
	}
	else if (status & US_CSR_TXRDY)
	{
		usart->isFinishedTx = 1;
		regs->US_IDR = US_IDR_TXRDY;
		if(usart->id == ID_USART2)
		{
			int i;
			PIO_Clear(&gUsart2Pins[2]);	
			for (i = 0; i < 10; i++)
				dummyRead(i);
		}
		rv = rtems_termios_dequeue_characters(cd->termios_data, chars_to_dequeue);
		if (!rv)
		{
			cd->pDeviceContext = 0;
			cd->bActive = false;
		}
	}
	else
	{

	}
	regs->US_CR = US_CR_RSTSTA;
}


#endif


#endif


//-----------------------------------------------------------------------------------------------------------------------------------------
uint32_t dummyRead(uint32_t value)
{
	asm volatile ("nop");
	return value;
}


/* Set up the UART. */
rtems_status_code usart_uart_init(rtems_device_major_number  major,
								  rtems_device_minor_number  minor,
								  void                      *arg)
{
	uint32_t index;
	at91sam9g25_usart_regs_t *usart;
	rtems_status_code sc;
	uint32_t dwCfg;
    uint8_t iController;
	uint32_t      brgr;
	uint32_t      mode;


	for (index = 0; index < AT91SAM9G25_USART_NUM; index++) {
	
		at91sam9g25_usart[index].major = major;
		at91sam9g25_usart[index].minor = index;
		usart = at91sam9g25_usart[index].regs;
	
		DMAD_Initialize(&gDmad, 0);
		/* Enable interrupts */
		AIC->AIC_SMR[AT91SAM9G25_INT_DMAC0] = ((AIC_SMR_PRIOR_LOWEST << AIC_SMR_PRIOR_Pos) & AIC_SMR_PRIOR_Msk) 
									| (AIC_SMR_SRCTYPE_INT_EDGE_TRIGGERED & AIC_SMR_SRCTYPE_Msk);
		AIC->AIC_SMR[AT91SAM9G25_INT_DMAC1] = ((AIC_SMR_PRIOR_LOWEST << AIC_SMR_PRIOR_Pos) & AIC_SMR_PRIOR_Msk) 
									| (AIC_SMR_SRCTYPE_INT_EDGE_TRIGGERED & AIC_SMR_SRCTYPE_Msk);
	
		sc = rtems_interrupt_handler_install(AT91SAM9G25_INT_DMAC0,
											 "DMA0",
											 RTEMS_INTERRUPT_UNIQUE,
											 (rtems_interrupt_handler)DMAD_Handler,
											 &gDmad);
	
		sc = rtems_interrupt_handler_install(AT91SAM9G25_INT_DMAC1,
											 "DMA1",
											 RTEMS_INTERRUPT_UNIQUE,
											 (rtems_interrupt_handler)DMAD_Handler,
											 (&gDmad));
		PMC_EnablePeripheral(at91sam9g25_usart[index].id);
		PMC_EnablePeripheral(ID_PIOA);
		PMC_EnablePeripheral(ID_PIOC);
		set_usart_pin(index);
	
	
		/* Allocate DMA channels for USART */
		at91sam9g25_usart[index].usartDmaTxChannel = DMAD_AllocateChannel( &gDmad,
												  DMAD_TRANSFER_MEMORY, at91sam9g25_usart[index].id);
		at91sam9g25_usart[index].usartDmaRxChannel = DMAD_AllocateChannel( &gDmad,
												  at91sam9g25_usart[index].id, DMAD_TRANSFER_MEMORY);
		if (   at91sam9g25_usart[index].usartDmaTxChannel == DMAD_ALLOC_FAILED 
			|| at91sam9g25_usart[index].usartDmaRxChannel == DMAD_ALLOC_FAILED )
		{
			printk("DMA channel allocat error\n\r");
			while(1);
		}
		/* Set RX callback */
		DMAD_SetCallback(&gDmad, at91sam9g25_usart[index].usartDmaRxChannel,
						(DmadTransferCallback)_DmaRxCallback, &at91sam9g25_usart[index]);
	
		/*set tx callback*/
		DMAD_SetCallback(&gDmad, at91sam9g25_usart[index].usartDmaTxChannel,
						 (DmadTransferCallback)_DmaTxCallback, &at91sam9g25_usart[index]);
	
		/* Configure DMA RX channel */
		iController = (at91sam9g25_usart[index].usartDmaRxChannel >> 8);
		dwCfg = 0
			   | DMAC_CFG_SRC_PER(DMAIF_Get_ChannelNumber( iController, at91sam9g25_usart[index].id, DMAD_TRANSFER_RX ))           
			   | DMAC_CFG_SRC_H2SEL
			   | DMAC_CFG_SOD
			   | DMAC_CFG_FIFOCFG_ALAP_CFG;
		DMAD_PrepareChannel( &gDmad, at91sam9g25_usart[index].usartDmaRxChannel, dwCfg );
		/* Configure DMA TX channel */
		iController = (at91sam9g25_usart[index].usartDmaTxChannel >> 8);
		dwCfg = 0           
			   | DMAC_CFG_DST_PER(DMAIF_Get_ChannelNumber( iController, at91sam9g25_usart[index].id, DMAD_TRANSFER_TX ))
			   | DMAC_CFG_DST_H2SEL
			   | DMAC_CFG_SOD
			   | DMAC_CFG_FIFOCFG_ALAP_CFG;
		DMAD_PrepareChannel( &gDmad, at91sam9g25_usart[index].usartDmaTxChannel, dwCfg );
	
		sc = rtems_semaphore_create (rtems_build_name ('U', 'R', 'T', '0' + index), 
									 0,
									 RTEMS_SIMPLE_BINARY_SEMAPHORE,
									 0,
									 &at91sam9g25_usart[index].state_update);
		
		sc = rtems_semaphore_create (rtems_build_name ('U', 'B', 'L', '0' + index), 
									 1,
									 RTEMS_SIMPLE_BINARY_SEMAPHORE,
									 0,
									 &at91sam9g25_usart[index].bus_lock);
	
		mode = US_MR_USART_MODE_RS485 
			| US_MR_PAR_NO 
			| US_MR_CHMODE_NORMAL
			| US_MR_CHRL_8_BIT
			| US_MR_NBSTOP_1_BIT
			| US_MR_PAR_NO;
	
		brgr = at91sam9g25_get_mck() / 16 / USART_UART_BAUDRATE;
	
		if (brgr > 65535)
		{
			brgr /= 8;
			mode |= US_MR_USCLKS_DIV;
		}
		else
			mode |= US_MR_USCLKS_MCK;
	
		usart->US_CR = US_CR_RSTRX | US_CR_RSTTX | US_CR_RXDIS | US_CR_TXDIS | US_CR_RTSDIS;
		usart->US_MR = mode;
		usart->US_BRGR = brgr;
		dummyRead(usart->US_RHR);
		if (at91sam9g25_usart[index].id == ID_USART2)
			PIO_Clear(&gUsart2Pins[2]);

		AIC->AIC_SMR[at91sam9g25_usart[index].id] = ((AIC_SMR_PRIOR_LOWEST << AIC_SMR_PRIOR_Pos) & AIC_SMR_PRIOR_Msk) 
                                | (AIC_SMR_SRCTYPE_INT_EDGE_TRIGGERED & AIC_SMR_SRCTYPE_Msk);
		usart->US_IDR = 0xFFFFFFFF;
		sc = rtems_interrupt_handler_install(at91sam9g25_usart[index].id,
											 "UART",
											 RTEMS_INTERRUPT_UNIQUE,
											 usart_int_process,
											 &at91sam9g25_usart[index]);
		dummyRead(usart->US_CSR);
		dummyRead(usart->US_RHR);
	}

	return (sc);

}

int usart_transfer(int index, unsigned char *txData, uint32_t txSize, unsigned char *rxData, uint32_t rxSize)
{
	sDmaTransferDescriptor td;
	at91sam9g25_usart_regs_t *usart;
	usart = at91sam9g25_usart[index].regs;

	if (txData != NULL && txSize > 0)
	{
		if (at91sam9g25_usart[index].id == ID_USART2)
			PIO_Set(&gUsart2Pins[2]);
		usart->US_RTOR = 0;
		usart->US_IDR = US_IDR_TIMEOUT;
		usart->US_CR = US_CR_TXEN | US_CR_RXDIS | US_CR_RTSEN;
	
		td.dwSrcAddr = (uint32_t) txData;
		td.dwDstAddr = (uint32_t)&usart->US_THR;
		td.dwCtrlA   = txSize
						| DMAC_CTRLA_SRC_WIDTH_BYTE | DMAC_CTRLA_DST_WIDTH_BYTE;
		td.dwCtrlB   = DMAC_CTRLB_SRC_DSCR | DMAC_CTRLB_DST_DSCR
						| DMAC_CTRLB_FC_MEM2PER_DMA_FC
						| DMAC_CTRLB_SRC_INCR_INCREMENTING
						| DMAC_CTRLB_DST_INCR_FIXED;
		td.dwDscAddr = 0;
		DMAD_PrepareSingleTransfer(&gDmad, at91sam9g25_usart[index].usartDmaTxChannel, &td);
		DMAD_StartTransfer(&gDmad, at91sam9g25_usart[index].usartDmaTxChannel);
		rtems_semaphore_obtain(at91sam9g25_usart[index].state_update, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
		if (at91sam9g25_usart[index].id == ID_USART2)
		{
			int i;
			for (i = 0; i < 1000; i++)
				dummyRead(i);
			PIO_Clear(&gUsart2Pins[2]);
		}
	}
	
	if (rxData != NULL && rxSize > 0)
	{
		
		usart->US_CR = US_CR_RXEN | US_CR_TXDIS | US_CR_RTSEN;
		if (at91sam9g25_usart[index].rxTimeout < 0)
		{
			usart->US_RTOR = rxSize * USART_TIMEOUT_RATIO;
		}
		else
		{
			usart->US_RTOR = at91sam9g25_usart[index].rxTimeout;
		}

		usart->US_CR = US_CR_STTTO;
		usart->US_IER = US_IER_TIMEOUT;

		td.dwSrcAddr = (uint32_t)&usart->US_RHR;
		td.dwDstAddr = (uint32_t) rxData;
		td.dwCtrlA   = rxSize
						| DMAC_CTRLA_SRC_WIDTH_BYTE | DMAC_CTRLA_DST_WIDTH_BYTE;
		td.dwCtrlB   = DMAC_CTRLB_SRC_DSCR | DMAC_CTRLB_DST_DSCR
						| DMAC_CTRLB_FC_PER2MEM_DMA_FC
						| DMAC_CTRLB_SRC_INCR_FIXED
						| DMAC_CTRLB_DST_INCR_INCREMENTING;
		td.dwDscAddr = 0;
		DMAD_PrepareSingleTransfer(&gDmad, at91sam9g25_usart[index].usartDmaRxChannel, &td);
		DMAD_StartTransfer(&gDmad, at91sam9g25_usart[index].usartDmaRxChannel);
		rtems_semaphore_obtain(at91sam9g25_usart[index].state_update, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
		usart->US_RTOR = 0;
		usart->US_IDR = US_IDR_TIMEOUT;
	}

	usart->US_CR = US_CR_RXDIS | US_CR_TXDIS | US_CR_RTSDIS;
	return (at91sam9g25_usart[index].result);
}

int start_usart(int index)
{
	if (index >= AT91SAM9G25_USART_NUM)
	{
		return (-1);
	}
	rtems_semaphore_obtain(at91sam9g25_usart[index].bus_lock, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
	return (0);
}

int usart_settings(int index, int baudrate)
{
	uint32_t mode;
	uint32_t brgr;
	at91sam9g25_usart_regs_t *usart;

	usart = at91sam9g25_usart[index].regs;

	if (index >= AT91SAM9G25_USART_NUM)
	{
		return (-1);
	}
	mode = US_MR_USART_MODE_RS485 
		| US_MR_PAR_NO 
		| US_MR_CHMODE_NORMAL
		| US_MR_CHRL_8_BIT
		| US_MR_NBSTOP_1_BIT
		| US_MR_PAR_NO;

	brgr = at91sam9g25_get_mck() / 16 / baudrate;

	if (brgr > 65535)
	{
		brgr /= 8;
		mode |= US_MR_USCLKS_DIV;
	}
	else
		mode |= US_MR_USCLKS_MCK;

	usart->US_CR = US_CR_RSTRX | US_CR_RSTTX | US_CR_RXDIS | US_CR_TXDIS | US_CR_RTSDIS;
	usart->US_MR = mode;
	usart->US_BRGR = brgr;
	dummyRead(usart->US_RHR);
	return (0);
}

int usart_rxTimeout(int index, int time)
{
	if (index >= AT91SAM9G25_USART_NUM)
	{
		return (-1);
	}

	at91sam9g25_usart[index].rxTimeout = time;
	return (0);
}

int end_usart(int index)
{
	if (index >= AT91SAM9G25_USART_NUM)
	{
		return (-1);
	}
	rtems_semaphore_release(at91sam9g25_usart[index].bus_lock);
	return (0);
}

/**
 *  \brief Callback function for DMA receiving.
 */
static void _DmaRxCallback( uint8_t status, void* pArg )
{
	TAt91sam9g25_usart *uart = (TAt91sam9g25_usart*)pArg;
    rtems_semaphore_release(uart->state_update);
	uart->result = 0;
}

static void _DmaTxCallback(uint8_t status, void *pArg)
{
	TAt91sam9g25_usart *uart = (TAt91sam9g25_usart*)pArg;
	rtems_semaphore_release(uart->state_update);
	uart->result = 0;
}

void usart_int_process(void *param)
{
	TAt91sam9g25_usart *uart = (TAt91sam9g25_usart*)param;
	at91sam9g25_usart_regs_t *usart;
	uint32_t status;

	usart = uart->regs;
	status = usart->US_CSR;
	status = status & usart->US_IMR;
	if (status & US_CSR_TIMEOUT)
	{
		DMAC_DisableChannel(&gDmad, uart->usartDmaRxChannel);
		rtems_semaphore_release(uart->state_update);
		usart->US_IDR = US_IDR_TIMEOUT;
		uart->result = -1;
	}
	else
	{

	}
	usart->US_CR = US_CR_RSTSTA;
}


