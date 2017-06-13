/*  SPI driver for shoonis 9g25
 *  baconxu@gmail.com
 * 
 */
#include <rtems.h>
#include <bsp.h>
#include <at91sam9x25.h>
#include <at91sam9x25_gpio.h>
#include <at91sam9x25_pmc.h>
#include <bsp/irq.h>
#include <bsp/irq-generic.h>
#include <bsp/dma.h>
#define RTEMS_STATUS_CHECKS_USE_PRINTK
#include <rtems/status-checks.h>
#include <bsp/spi.h>


/** DMA support */
#define USE_SPI_DMA

/** DMA Link List size for spi transation*/
#define DMA_SPI_LLI     2

/** An unspecified error has occured.*/
#define SPID_ERROR          1

/** SPI driver is currently in use.*/
#define SPID_ERROR_LOCK     2

/*----------------------------------------------------------------------------
 *        Macros
 *----------------------------------------------------------------------------*/

/** Calculates the value of the SCBR field of the Chip Select Register given MCK and SPCK.*/
#define SPID_CSR_SCBR(mck, spck)    SPI_CSR_SCBR((mck) / (spck))

/** Calculates the value of the DLYBS field of the Chip Select Register given delay in ns and MCK.*/
#define SPID_CSR_DLYBS(mck, delay)  SPI_CSR_DLYBS((((delay) * ((mck) / 1000000)) / 1000) + 1)

/** Calculates the value of the DLYBCT field of the Chip Select Register given delay in ns and MCK.*/
#define SPID_CSR_DLYBCT(mck, delay) SPI_CSR_DLYBCT((((delay) / 32 * ((mck) / 1000000)) / 1000) + 1)
/** Calculate the PCS field value given the chip select NPCS value */
#define SPI_PCS(npcs)       ((~(1 << npcs) & 0xF) << 16)

/** Calculates the value of the CSR SCBR field given the baudrate and MCK. */
#define SPI_SCBR(baudrate, masterClock) ((uint32_t) (masterClock / baudrate) << 8)

/** Calculates the value of the CSR DLYBS field given the desired delay (in ns) */
#define SPI_DLYBS(delay, masterClock) ((uint32_t) (((masterClock / 1000000) * delay) / 1000) << 16)

/** Calculates the value of the CSR DLYBCT field given the desired delay (in ns) */
#define SPI_DLYBCT(delay, masterClock) ((uint32_t) (((masterClock / 1000000) * delay) / 32000) << 24)

/** SPI chip select configuration value. */
#define SPI_CSRb(mck, spck)   (SPI_CSR_NCPHA | \
                               SPI_CSR_CSAAT | \
                               SPID_CSR_DLYBCT((mck), 100) | \
                               SPID_CSR_DLYBS((mck), 5) | \
                               SPID_CSR_SCBR((mck), (spck)))


#define SPI_CSR(mck, spck) (SPI_CSR_NCPHA \
                           | SPI_CSR_CSAAT \
                           | SPID_CSR_DLYBCT(mck, 250)\
                           | SPID_CSR_DLYBS(mck, 250)\
                           | SPID_CSR_SCBR(mck, spck))

/*----------------------------------------------------------------------------
 *        Types
 *----------------------------------------------------------------------------*/

/** SPI transfer complete callback. */
typedef void (*SpidCallback)( uint8_t, void* ) ;

/** \brief Spi Transfer Request prepared by the application upper layer.
 *
 * This structure is sent to the SPI_SendCommand function to start the transfer.
 * At the end of the transfer, the callback is invoked by the interrupt handler.
 */
typedef struct _SpidCmd
{
    /** Pointer to the command data. */
    uint8_t *pCmd;
    /** Command size in bytes. */
    uint8_t cmdSize;
    /** Pointer to the data to be sent. */
    uint8_t *pData;
    /** Data size in bytes. */
    uint16_t dataSize;
    /** SPI chip select. */
    uint8_t spiCs;
    /** Callback function invoked at the end of transfer. */
    SpidCallback callback;
    /** Callback arguments. */
    void *pArgument;
} SpidCmd ;

/** Constant structure associated with SPI port. This structure prevents
    client applications to have access in the same time. */
typedef struct _Spid
{
    /** Pointer to SPI Hardware registers */
    Spi* pSpiHw ;
    /** Current SpiCommand being processed */
    SpidCmd *pCurrentCommand ;
    /** Pointer to DMA driver */
    sDmad* pDmad;
    /** SPI Id as defined in the product datasheet */
    uint8_t spiId ;
    /** Mutual exclusion semaphore. */
    volatile int8_t semaphore ;

    //rtems_vector_number vector;
    rtems_id            bus_lock;
    rtems_id            state_update;
	uint32_t			cs;
    uint32_t            bus_clock;

#if defined(USE_SPI_DMA)	/*  DMA driver instance */
	sDmaTransferDescriptor dmaTxLinkList[DMA_SPI_LLI];
	sDmaTransferDescriptor dmaRxLinkList[DMA_SPI_LLI];

	uint32_t spiDmaTxChannel;
	uint32_t spiDmaRxChannel;
#endif
} Spid ;


/*
    function prototype.
*/
uint32_t SPID_Configure( Spid* pSpid,
                                Spi* pSpiHw,
                                uint8_t spiId,
                                sDmad* pDmad ) ;

void SPID_ConfigureCS( Spid* pSpid, uint32_t dwCS, uint32_t dwCsr ) ;

uint32_t SPID_SendCommand( Spid* pSpid, SpidCmd* pCommand ) ;

void SPID_Handler( Spid* pSpid ) ;

void SPID_DmaHandler( Spid *pSpid );

uint32_t SPID_IsBusy( const Spid* pSpid ) ;

static void enable_spi0_wp();
static void enable_spi1_wp();
static rtems_status_code spi_transfer(uint32_t spiBusId,
                               uint8_t *writeData,
                               uint32_t wdSize,
                               uint8_t *readData,
                               uint32_t rdSize);
static void spi1_cs(uint32_t cs);

/*----------------------------------------------------------------------------
 *        Macros
 *----------------------------------------------------------------------------*/

#define AT91SAM9G25_SPI_DRIVER_NUM          2
#define SPI0_SUB_BUS_NUM                    1
#define SPI1_SUB_BUS_NUM                    15
#define DEFAULT_SPCK                        6000000

#define POLLING_MODE                        0
#define SPI0_PINS                           PINS_SPI0, PIN_SPI0_NPCS0

#define SPI1_NPCS0                          {PIO_PA17, PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT}
#define SPI1_NPCS1                          {PIO_PA18, PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT}
#define SPI1_NPCS2                          {PIO_PA19, PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT}
#define SPI1_NPCS3                          {PIO_PA20, PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT}
#define SPI1_PINS                           PINS_SPI1, SPI1_NPCS0, SPI1_NPCS1, SPI1_NPCS2, SPI1_NPCS3

#if defined(debug_for_spi1) //debug for spi1
#define SPI1_NPCS3_REAL						{PIO_PA30B_SPI1_NPCS3, PIOA, ID_PIOA, PIO_PERIPH_B, PIO_DEFAULT}
#define PIN_TWI_TWD0   						{PIO_PA30A_TWD0, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
#endif

/*note: the pins_spi1 contains 3 pins.*/
#define SPI1_CS0_H()                          PIO_Set(&spi1Pins[3])
#define SPI1_CS0_L()                          PIO_Clear(&spi1Pins[3])

#define SPI1_CS1_H()                          PIO_Set(&spi1Pins[4])
#define SPI1_CS1_L()                          PIO_Clear(&spi1Pins[4])

#define SPI1_CS2_H()                          PIO_Set(&spi1Pins[5])
#define SPI1_CS2_L()                          PIO_Clear(&spi1Pins[5])

#define SPI1_CS3_H()                          PIO_Set(&spi1Pins[6])
#define SPI1_CS3_L()                          PIO_Clear(&spi1Pins[6])
/*----------------------------------------------------------------------------
 *        Local Variables
 *----------------------------------------------------------------------------*/
static const Pin spi0Pins[] = {SPI0_PINS};
static const Pin spi1Pins[] = {SPI1_PINS};

#if defined(debug_for_spi1) //debug for spi1
static const Pin spi1SpecialPin[] = {SPI1_NPCS3_REAL, PIN_TWI_TWD0};
#define setSpi1SpecialPin() 	PIO_Configure(&spi1SpecialPin[0], 1)
#define restoreSpi1SpecialPin() PIO_Configure(&spi1SpecialPin[1], 1)
#endif

static Spid gSpid[AT91SAM9G25_SPI_DRIVER_NUM];


rtems_status_code at91sam9g25_spi_init(rtems_device_major_number  major,
                                       rtems_device_minor_number  minor,
                                       void                      *arg)
{
    rtems_status_code sc;
    int i;

    /* Initialize DMA driver instance with interrupt mode */
    DMAD_Initialize(&gDmad, POLLING_MODE);
#if POLLING_MODE == 0
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
	
    printk("DMA driver initialized with IRQ %X\n", AIC->AIC_IMR);
#else
    printk("DMA driver initialized without IRQ, polling\n");
#endif
    PIO_Configure(spi0Pins, PIO_LISTSIZE(spi0Pins));
    PIO_Configure(spi1Pins, PIO_LISTSIZE(spi1Pins));
    SPID_Configure(&gSpid[0], SPI0, ID_SPI0, &gDmad);
    SPID_Configure(&gSpid[1], SPI1, ID_SPI1, &gDmad);
    for (i = 0; i < AT91SAM9G25_SPI_DRIVER_NUM; i++)
	{
		gSpid[i].cs = -1;

        /*read write semaphore*/
		sc = rtems_semaphore_create (rtems_build_name ('S', 'P', 'I', '0' + i), 
									 0,
									 RTEMS_SIMPLE_BINARY_SEMAPHORE,
									 0,
									 &gSpid[i].state_update);
        /*bus lock*/
		sc = rtems_semaphore_create (rtems_build_name ('S', 'B', 'L', '0' + i), 
									 1,
									 RTEMS_SIMPLE_BINARY_SEMAPHORE,
									 0,
									 &gSpid[i].bus_lock);
        gSpid[i].bus_clock = DEFAULT_SPCK;
	}

    enable_spi0_wp();
    enable_spi1_wp();

    return (RTEMS_SUCCESSFUL);
}

static void spi_callback(uint8_t spi, void *arg)
{
    Spid *pSpid = (Spid *)arg;
    if (pSpid != NULL)
        rtems_semaphore_release(pSpid->state_update);
}

int spi_end(uint32_t spiBusId)
{
	if (spiBusId < AT91SAM9G25_SPI_DRIVER_NUM)
	{
		gSpid[spiBusId].cs = -1;
		if (spiBusId == 1)
		{
			spi1_cs(8); 
		#if defined(debug_for_spi1) //debug for spi1
			restoreSpi1SpecialPin();
		#endif
		}
		rtems_semaphore_release(gSpid[spiBusId].bus_lock);
	}
    return 0;
}



uint32_t spi_open(uint8_t spi_id, uint8_t cs)
{
    uint32_t spi_handle = 0;
    int mck;

    if ((spi_id >= AT91SAM9G25_SPI_DRIVER_NUM)
        ||(spi_id == 0 && cs >= SPI0_SUB_BUS_NUM)
        ||(spi_id == 1 && cs >= SPI1_SUB_BUS_NUM))
    {
        return (0);
    }

    rtems_semaphore_obtain(gSpid[spi_id].bus_lock, RTEMS_WAIT, RTEMS_NO_TIMEOUT);

    gSpid[spi_id].cs = cs;
    mck = at91sam9g25_get_mck();
    SPID_ConfigureCS(&gSpid[spi_id], ((spi_id == 0)? cs:0), SPI_CSR(mck, gSpid[spi_id].bus_clock));
    if(spi_id == 1)
    {
        spi1_cs(cs);
    }

    return (gSpid[spi_id].bus_lock);
}


uint32_t spi_close(uint32_t spi_handle)
{
    if (spi_handle == 0)
    {
        return (RTEMS_IO_ERROR);
    }

    if (gSpid[0].bus_lock == spi_handle)
    {
        rtems_semaphore_release(gSpid[0].bus_lock);
    }
    else if (gSpid[1].bus_lock == spi_handle)
    {
        spi1_cs(0xF);
        gSpid[1].cs = 0xF;
        rtems_semaphore_release(gSpid[1].bus_lock);
    }
    else
    {
        return (RTEMS_IO_ERROR);
    }

    return (RTEMS_SUCCESSFUL);//success
}


uint32_t spi_set_baudrate(uint32_t spi_handle, uint32_t baudrate)
{
    uint32_t busid;
    uint32_t mck;

    if (gSpid[0].bus_lock == spi_handle)
    {
         busid = 0;
    }
    else if (gSpid[1].bus_lock == spi_handle)
    {
         busid = 1;
    }
    else
    {
        return (RTEMS_IO_ERROR);
    }

    mck = at91sam9g25_get_mck();
    SPID_ConfigureCS(&gSpid[busid], ((busid == 0)? gSpid[busid].cs:0),
                     SPI_CSR(mck, baudrate));
    gSpid[busid].bus_clock = baudrate;

    return (RTEMS_SUCCESSFUL);
}


uint32_t spi_write(uint32_t spi_handle,
                   uint8_t *tx1, uint8_t *rx1, uint32_t len1,
                   uint8_t *tx2, uint8_t *rx2, uint32_t len2)
{
    uint8_t ret = RTEMS_SUCCESSFUL;
    uint32_t busid;

    if (gSpid[0].bus_lock == spi_handle)
    {
         busid = 0;
    }
    else if (gSpid[1].bus_lock == spi_handle)
    {
         busid = 1;
    }
    else
    {
        return (RTEMS_IO_ERROR);
    }
    if(tx2 == NULL || rx2 == NULL || len2 == 0)
    {
        ret = spi_transfer(busid, tx1, len1, rx1, len1);   
    }
    else
    {
        ret = spi_transfer(busid, tx1, len1, tx2, len2);
    }
    
    return ret;
}
/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/
void spi1_cs(uint32_t cs)
{
#if 0
    PIOA->PIO_SODR |= 0xF << 17;    
    PIOA->PIO_CODR |= (~(cs & 0xF)) << 17;
#else
    switch (cs) 
    {
    case 0:
        SPI1_CS0_L();
        SPI1_CS1_L();
        SPI1_CS2_L();
        SPI1_CS3_H();
        break;
    case 1:
        SPI1_CS0_H();
        SPI1_CS1_L();
        SPI1_CS2_L();
        SPI1_CS3_H();
        break;
    case 2:
        SPI1_CS0_L();
        SPI1_CS1_H();
        SPI1_CS2_L();
        SPI1_CS3_H();
        break;
    case 3:
        SPI1_CS0_H();
        SPI1_CS1_H();
        SPI1_CS2_L();
        SPI1_CS3_H();
        break;
    case 7:
        SPI1_CS0_H();
        SPI1_CS1_H();
        SPI1_CS2_H();
        SPI1_CS3_L();
		break;
	default:
		SPI1_CS0_L();
        SPI1_CS1_L();
        SPI1_CS2_H();
        SPI1_CS3_L();
    }
#endif
}

rtems_status_code spi_transfer(uint32_t spiBusId,
                               uint8_t *writeData,
                               uint32_t wdSize,
                               uint8_t *readData,
                               uint32_t rdSize)
{
    SpidCmd cmd;
    if (writeData == NULL)
    {   
        cmd.pCmd        = readData;
        cmd.cmdSize     = rdSize;
        cmd.pData       = NULL;
        cmd.dataSize    = 0;
        if (readData == NULL)
        {
            return RTEMS_INVALID_ADDRESS;
        }
    }
    else
    {
        cmd.pCmd = writeData;
        cmd.cmdSize = wdSize;
        cmd.pData = readData;
        cmd.dataSize = rdSize;
    }

	cmd.spiCs       = gSpid[spiBusId].cs;
	if(spiBusId == 1)
	{
		cmd.spiCs = 0;
	}

    if (_System_state_Is_up(_System_state_Get()))
    {
        cmd.callback    = spi_callback;
    
        if (spiBusId < AT91SAM9G25_SPI_DRIVER_NUM)
        {
            cmd.pArgument = &gSpid[spiBusId];
            SPID_SendCommand(&gSpid[spiBusId], &cmd);
            rtems_semaphore_obtain(gSpid[spiBusId].state_update, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
        }
        else
            return RTEMS_IO_ERROR;    
    }
    else
    {
        cmd.callback    = NULL;
        cmd.pArgument   = NULL;
    
        if (spiBusId < AT91SAM9G25_SPI_DRIVER_NUM)
        {
            SPID_SendCommand(&gSpid[spiBusId], &cmd);
        }
        else
            return RTEMS_IO_ERROR;    
    }
    return (RTEMS_SUCCESSFUL);
}

static void enable_spi1_wp()
{
    /* 使能SPI1保护引脚     */ 
    PIOA->PIO_PER  |= 1<<16;
    PIOA->PIO_OER  |= 1<<16;
    
#if (SPI1_WP != 0)
    PIOA->PIO_SODR |= 1<<16; 
#else   
    PIOA->PIO_CODR |= 1<<16;
#endif  
}

static void enable_spi0_wp()
{
    /* 使能SPI0保护引脚     */
    PIOA->PIO_PER  |= 1<<15;
    PIOA->PIO_OER  |= 1<<15;
    
#if (SPI0_WP != 0)
    PIOA->PIO_SODR |= 1<<15;  
#else
    PIOA->PIO_CODR |= 1<<15;
#endif 
}


/**
 * \brief Enables a SPI peripheral.
 *
 * \param spi  Pointer to an Spi instance.
 */
static void SPI_Enable( Spi* spi )
{
    spi->SPI_CR = SPI_CR_SPIEN ;
}

/**
 * \brief Disables a SPI peripheral.
 *
 * \param spi  Pointer to an Spi instance.
 */
static void SPI_Disable( Spi* spi )
{
    spi->SPI_CR = SPI_CR_SPIDIS ;
}

/**
 * \brief Enables one or more interrupt sources of a SPI peripheral.
 *
 * \param spi  Pointer to an Spi instance.
 * \param sources Bitwise OR of selected interrupt sources.
 */
static void SPI_EnableIt( Spi* spi, uint32_t dwSources )
{
    spi->SPI_IER = dwSources ;
}

/**
 * \brief Disables one or more interrupt sources of a SPI peripheral.
 *
 * \param spi  Pointer to an Spi instance.
 * \param sources Bitwise OR of selected interrupt sources.
 */
static void SPI_DisableIt( Spi* spi, uint32_t dwSources )
{
    spi->SPI_IDR = dwSources ;
}

/**
 * \brief Configures a SPI peripheral as specified. The configuration can be computed
 * using several macros (see \ref spi_configuration_macros).
 *
 * \param spi  Pointer to an Spi instance.
 * \param id   Peripheral ID of the SPI.
 * \param configuration  Value of the SPI configuration register.
 */
static void SPI_Configure( Spi* spi, uint32_t dwId, uint32_t dwConfiguration )
{
    PMC_EnablePeripheral( dwId ) ;
    spi->SPI_CR = SPI_CR_SPIDIS ;

    /* Execute a software reset of the SPI twice */
    spi->SPI_CR = SPI_CR_SWRST ;
    spi->SPI_CR = SPI_CR_SWRST ;
    spi->SPI_MR = dwConfiguration ;
}

/**
 * \brief Configures SPI chip select.
 *
 * \param spi  Pointer to an Spi instance.
 * \param cS  Chip select of NPSCx.
 */
static void SPI_ChipSelect( Spi* spi, uint8_t cS)
{
    spi->SPI_MR |= SPI_MR_PCS_Msk ;
    spi->SPI_MR &= ~(SPI_MR_PCS ( cS )) ;
}

/**
 * \brief Configures SPI Mode Register.
 *
 * \param spi  Pointer to an Spi instance.
 * \param configuration  Value of the SPI mode register.
 */
static void SPI_SetMode( Spi* spi, 
                         uint32_t dwConfiguration )
{
    spi->SPI_MR = dwConfiguration ;
}

/**
 * \brief Configures SPI to release last used CS line.
 *
 * \param spi  Pointer to an Spi instance.
 */
static void SPI_ReleaseCS( Spi* spi )
{
    spi->SPI_CR = SPI_CR_LASTXFER ;
}


/**
 * \brief Configures a chip select of a SPI peripheral. The chip select configuration
 * is computed using several macros (see \ref spi_configuration_macros).
 *
 * \param spi   Pointer to an Spi instance.
 * \param npcs  Chip select to configure (0, 1, 2 or 3).
 * \param configuration  Desired chip select configuration.
 */
void SPI_ConfigureNPCS( Spi* spi, uint32_t dwNpcs, uint32_t dwConfiguration )
{
    spi->SPI_CSR[dwNpcs] = dwConfiguration ;
}

/**
 * \brief Configures a chip select active mode of a SPI peripheral.
 *
 * \param spi   Pointer to an Spi instance.
 * \param dwNpcs  Chip select to configure (0, 1, 2 or 3).
 * \param bReleaseOnLast CS controlled by last transfer.
 *                       SPI_ReleaseCS() is used to deactive CS. 
 */
void SPI_ConfigureCSMode( Spi* spi, uint32_t dwNpcs, uint32_t bReleaseOnLast )
{
    if (bReleaseOnLast)
    {
        spi->SPI_CSR[dwNpcs] |=  SPI_CSR_CSAAT;
    }
    else
    {
        spi->SPI_CSR[dwNpcs] &= ~SPI_CSR_CSAAT;
    }
}

/**
 * \brief Get the current status register of the given SPI peripheral.
 * \note This resets the internal value of the status register, so further
 * read may yield different values.
 * \param spi   Pointer to a Spi instance.
 * \return  SPI status register.
 */
static uint32_t SPI_GetStatus( Spi* spi )
{
    return spi->SPI_SR ;
}

/**
 * \brief Reads and returns the last word of data received by a SPI peripheral. This
 * method must be called after a successful SPI_Write call.
 *
 * \param spi  Pointer to an Spi instance.
 *
 * \return readed data.
 */
static uint32_t SPI_Read( Spi* spi )
{
    while ( (spi->SPI_SR & SPI_SR_RDRF) == 0 ) ;

    return spi->SPI_RDR & 0xFFFF ;
}

/**
 * \brief Sends data through a SPI peripheral. If the SPI is configured to use a fixed
 * peripheral select, the npcs value is meaningless. Otherwise, it identifies
 * the component which shall be addressed.
 *
 * \param spi   Pointer to an Spi instance.
 * \param npcs  Chip select of the component to address (0, 1, 2 or 3).
 * \param data  Word of data to send.
 */
static void SPI_Write( Spi* spi, uint32_t dwNpcs, uint16_t wData )
{
    /* Send data */
    while ( (spi->SPI_SR & SPI_SR_TXEMPTY) == 0 ) ;
    spi->SPI_TDR = wData | SPI_PCS( dwNpcs ) ;
    while ( (spi->SPI_SR & SPI_SR_TDRE) == 0 ) ;
}

/**
 * \brief Sends last data through a SPI peripheral.
 * If the SPI is configured to use a fixed peripheral select, the npcs value is
 * meaningless. Otherwise, it identifies the component which shall be addressed.
 *
 * \param spi   Pointer to an Spi instance.
 * \param npcs  Chip select of the component to address (0, 1, 2 or 3).
 * \param data  Word of data to send.
 */
static void SPI_WriteLast( Spi* spi, uint32_t dwNpcs, uint16_t wData )
{
    /* Send data */
    while ( (spi->SPI_SR & SPI_SR_TXEMPTY) == 0 ) ;
    spi->SPI_TDR = wData | SPI_PCS( dwNpcs ) | SPI_TDR_LASTXFER ;
    while ( (spi->SPI_SR & SPI_SR_TDRE) == 0 ) ;
}

/**
 * \brief Check if SPI transfer finish.
 *
 * \param spi  Pointer to an Spi instance.
 *
 * \return Returns 1 if there is no pending write operation on the SPI; otherwise
 * returns 0.
 */
static uint32_t SPI_IsFinished( Spi* spi )
{
    return ((spi->SPI_SR & SPI_SR_TXEMPTY) != 0) ;
}


#if defined(USE_SPI_DMA)
/**
 * \brief SPI DMA Rx callback
 * Invoked on SPi DMA reception done.
 * \param dmaStatus DMA status.
 * \param pArg Pointer to callback argument - Pointer to Spid instance.   
 */ 
static void SPID_Rx_Cb(uint32_t dmaStatus, Spid* pArg)
{
    SpidCmd *pSpidCmd = pArg->pCurrentCommand;
    Spi *pSpiHw = pArg->pSpiHw;

    if (dmaStatus == DMAD_PARTIAL_DONE)
        return;

    /* Disable the SPI TX & RX */
    SPI_Disable ( pSpiHw );
    
    /* Disable the SPI Peripheral */
    PMC_DisablePeripheral ( pArg->spiId );
    
    /* Release CS */
    SPI_ReleaseCS(pSpiHw);
    
    /* Release the DMA channels */
    DMAD_FreeChannel(pArg->pDmad, pArg->spiDmaRxChannel);
    DMAD_FreeChannel(pArg->pDmad, pArg->spiDmaTxChannel);

    /* Release the dataflash semaphore */
    pArg->semaphore++;
        
    /* Invoke the callback associated with the current command */
    if (pSpidCmd && pSpidCmd->callback) {
    
        pSpidCmd->callback(0, pSpidCmd->pArgument);
    }
}

/**
 * \brief Configure the DMA Channels: 0 RX, 1 TX.
 * Channels are disabled after configure.
 * \returns 0 if the dma channel configuration successfully; otherwise returns
 * SPID_ERROR_XXX.
 */
static uint8_t _spid_configureDmaChannels( Spid* pSpid )
{
    uint32_t dwCfg;
    uint8_t iController;

    /* Allocate a DMA channel for SPI0 TX. */
    pSpid->spiDmaTxChannel = DMAD_AllocateChannel( pSpid->pDmad,
                                            DMAD_TRANSFER_MEMORY, pSpid->spiId);
    {
        if ( pSpid->spiDmaTxChannel == DMAD_ALLOC_FAILED ) 
        {
            return SPID_ERROR;
        }
    }
    /* Allocate a DMA channel for SPI0 RX. */
    pSpid->spiDmaRxChannel = DMAD_AllocateChannel( pSpid->pDmad,
                                            pSpid->spiId, DMAD_TRANSFER_MEMORY);
    {
        if ( pSpid->spiDmaRxChannel == DMAD_ALLOC_FAILED ) 
        {
            return SPID_ERROR;
        }
    }
    iController = (pSpid->spiDmaRxChannel >> 8);
    /* Setup callbacks for SPI0 RX */
    DMAD_SetCallback(pSpid->pDmad, pSpid->spiDmaRxChannel,
                     (DmadTransferCallback)SPID_Rx_Cb, pSpid);

    /* Configure the allocated DMA channel for SPI0 RX. */
    dwCfg = 0
           | DMAC_CFG_SRC_PER(
              DMAIF_Get_ChannelNumber( iController, pSpid->spiId, DMAD_TRANSFER_RX ))
           | DMAC_CFG_DST_PER(
              DMAIF_Get_ChannelNumber( iController, pSpid->spiId, DMAD_TRANSFER_RX ))
           | DMAC_CFG_SRC_H2SEL
           | DMAC_CFG_SOD
           | DMAC_CFG_FIFOCFG_ALAP_CFG;

    if (DMAD_PrepareChannel( pSpid->pDmad, pSpid->spiDmaRxChannel, dwCfg ))
        return SPID_ERROR;

    iController = (pSpid->spiDmaTxChannel >> 8);
    /* Setup callbacks for SPI0 TX (ignored) */
    DMAD_SetCallback(pSpid->pDmad, pSpid->spiDmaTxChannel, NULL, NULL);

    /* Configure the allocated DMA channel for SPI0 TX. */
    dwCfg = 0
           | DMAC_CFG_SRC_PER(
              DMAIF_Get_ChannelNumber( iController, pSpid->spiId, DMAD_TRANSFER_TX ))
           | DMAC_CFG_DST_PER(
              DMAIF_Get_ChannelNumber( iController, pSpid->spiId, DMAD_TRANSFER_TX ))
           | DMAC_CFG_DST_H2SEL
           | DMAC_CFG_SOD
           | DMAC_CFG_FIFOCFG_ALAP_CFG;

    if ( DMAD_PrepareChannel( pSpid->pDmad, pSpid->spiDmaTxChannel, dwCfg ))
        return SPID_ERROR;
    return 0;
}

/**
 * \brief Configure the DMA source and destination with Linker List mode.
 *
 * \param pCommand Pointer to command
  * \returns 0 if the dma multibuffer configuration successfully; otherwise returns
 * SPID_ERROR_XXX.
 */
static uint8_t _spid_configureLinkList(Spi *pSpiHw, void *pDmad, SpidCmd *pCommand)
{
    int index;
    if (pSpiHw == SPI0)
        index = 0;
    else
        index = 1;

    /* Setup RX Link List */
    gSpid[index].dmaRxLinkList[0].dwSrcAddr = (uint32_t)&pSpiHw->SPI_RDR;
    gSpid[index].dmaRxLinkList[0].dwDstAddr = (uint32_t)pCommand->pCmd;
    gSpid[index].dmaRxLinkList[0].dwCtrlA   = pCommand->cmdSize | DMAC_CTRLA_SRC_WIDTH_BYTE | DMAC_CTRLA_DST_WIDTH_BYTE;
    gSpid[index].dmaRxLinkList[0].dwCtrlB   = DMAC_CTRLB_FC_PER2MEM_DMA_FC | DMAC_CTRLB_SRC_INCR_FIXED | DMAC_CTRLB_DST_INCR_INCREMENTING;

    /* Setup TX Link List */                           
    gSpid[index].dmaTxLinkList[0].dwSrcAddr = (uint32_t)pCommand->pCmd;
    gSpid[index].dmaTxLinkList[0].dwDstAddr = (uint32_t)&pSpiHw->SPI_TDR;
    gSpid[index].dmaTxLinkList[0].dwCtrlA   = pCommand->cmdSize | DMAC_CTRLA_SRC_WIDTH_BYTE  | DMAC_CTRLA_DST_WIDTH_BYTE;
    gSpid[index].dmaTxLinkList[0].dwCtrlB   = DMAC_CTRLB_FC_MEM2PER_DMA_FC | DMAC_CTRLB_SRC_INCR_INCREMENTING | DMAC_CTRLB_DST_INCR_FIXED;

    /* In case command only */
    if (pCommand->pData == 0) {

        gSpid[index].dmaRxLinkList[0].dwDscAddr = 0;
        gSpid[index].dmaTxLinkList[0].dwDscAddr = 0;
    }
    /* In case Command & data */
    else {
        gSpid[index].dmaRxLinkList[0].dwDscAddr = (uint32_t)&gSpid[index].dmaRxLinkList[1];
        gSpid[index].dmaRxLinkList[1].dwSrcAddr = (uint32_t)&pSpiHw->SPI_RDR;
        gSpid[index].dmaRxLinkList[1].dwDstAddr = (uint32_t)pCommand->pData;
        gSpid[index].dmaRxLinkList[1].dwCtrlA   = pCommand->dataSize | DMAC_CTRLA_SRC_WIDTH_BYTE | DMAC_CTRLA_DST_WIDTH_BYTE;
        gSpid[index].dmaRxLinkList[1].dwCtrlB   = DMAC_CTRLB_SRC_DSCR | DMAC_CTRLB_DST_DSCR | DMAC_CTRLB_FC_PER2MEM_DMA_FC
                                  | DMAC_CTRLB_SRC_INCR_FIXED | DMAC_CTRLB_DST_INCR_INCREMENTING;
        gSpid[index].dmaRxLinkList[1].dwDscAddr = 0;
        gSpid[index].dmaTxLinkList[0].dwDscAddr = (uint32_t)&gSpid[index].dmaTxLinkList[1];
        gSpid[index].dmaTxLinkList[1].dwSrcAddr = (uint32_t)pCommand->pData;
        gSpid[index].dmaTxLinkList[1].dwDstAddr = (uint32_t)&pSpiHw->SPI_TDR;
        gSpid[index].dmaTxLinkList[1].dwCtrlA   = pCommand->dataSize | DMAC_CTRLA_SRC_WIDTH_BYTE | DMAC_CTRLA_DST_WIDTH_BYTE;
        gSpid[index].dmaTxLinkList[1].dwCtrlB   =  DMAC_CTRLB_SRC_DSCR | DMAC_CTRLB_DST_DSCR | DMAC_CTRLB_FC_MEM2PER_DMA_FC
                                  | DMAC_CTRLB_SRC_INCR_INCREMENTING | DMAC_CTRLB_DST_INCR_FIXED;
        gSpid[index].dmaTxLinkList[1].dwDscAddr    = 0;
    }
    if ( DMAD_PrepareMultiTransfer( pDmad, gSpid[index].spiDmaRxChannel, &gSpid[index].dmaRxLinkList[0]))
        return SPID_ERROR;
    if ( DMAD_PrepareMultiTransfer( pDmad, gSpid[index].spiDmaTxChannel, &gSpid[index].dmaTxLinkList[0]))
        return SPID_ERROR;
    return 0;   
}
#endif

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/
/**
 * \brief Initializes the Spid structure and the corresponding SPI & DMA hardware.
 * select value.
 * The driver will uses DMA channel 0 for RX and DMA channel 1 for TX.
 * The DMA channels are freed automatically when no SPI command processing.
 *
 * \param pSpid  Pointer to a Spid instance.
 * \param pSpiHw Associated SPI peripheral.
 * \param spiId  SPI peripheral identifier.
 * \param pDmad  Pointer to a Dmad instance. 
 */
uint32_t SPID_Configure( Spid *pSpid ,
                         Spi *pSpiHw , 
                         uint8_t spiId,
                         sDmad *pDmad )
{
    /* Initialize the SPI structure */
    pSpid->pSpiHw = pSpiHw;
    pSpid->spiId  = spiId;
    pSpid->semaphore = 1;
    pSpid->pCurrentCommand = 0;
    pSpid->pDmad = pDmad;

    /* Enable the SPI Peripheral ,Execute a software reset of the SPI, Configure SPI in Master Mode*/
    SPI_Configure ( pSpiHw, pSpid->spiId, SPI_MR_MSTR | SPI_MR_MODFDIS | SPI_MR_PCS_Msk );
    /* Disable the SPI Peripheral */
    PMC_DisablePeripheral (pSpid->spiId );
    return 0;
}

/**
 * \brief Configures the parameters for the device corresponding to the cs value.
 *
 * \param pSpid  Pointer to a Spid instance.
 * \param cs number corresponding to the SPI chip select.
 * \param csr SPI_CSR value to setup.
 */
void SPID_ConfigureCS( Spid *pSpid, 
                       uint32_t dwCS, 
                       uint32_t dwCsr)
{
    Spi *pSpiHw = pSpid->pSpiHw;
    
    /* Enable the SPI Peripheral */
    PMC_EnablePeripheral (pSpid->spiId );
    /* Configure SPI Chip Select Register */
    SPI_ConfigureNPCS( pSpiHw, dwCS, dwCsr );
    /* Disable the SPI Peripheral */
    PMC_DisablePeripheral (pSpid->spiId );
}

/**
 * \brief Starts a SPI master transfer. This is a non blocking function. It will
 *  return as soon as the transfer is started.
 *
 * \param pSpid  Pointer to a Spid instance.
 * \param pCommand Pointer to the SPI command to execute.
 * \returns 0 if the transfer has been started successfully; otherwise returns
 * SPID_ERROR_LOCK is the driver is in use, or SPID_ERROR if the command is not
 * valid.
 */
uint32_t SPID_SendCommand( Spid *pSpid, SpidCmd *pCommand)
{
    Spi *pSpiHw = pSpid->pSpiHw;
         
    /* Try to get the dataflash semaphore */
    if (pSpid->semaphore == 0) {
    
         return SPID_ERROR_LOCK;
    }
    pSpid->semaphore--;

    /* Enable the SPI Peripheral */
    PMC_EnablePeripheral (pSpid->spiId);
    
    /* SPI chip select */
    SPI_ChipSelect (pSpiHw, 1 << pCommand->spiCs);

    if (!_System_state_Is_up(_System_state_Get()))
    {
        /* Initialize the callback */
        pSpid->pCurrentCommand = pCommand;
        /* Enables the SPI to transfer and receive data. */
        SPI_Enable (pSpiHw);
    
        {
            uint32_t i;
            /* Transfer command */
            for (i = 0; i < pCommand->cmdSize; i ++)
            {
                SPI_Write(pSpiHw, pCommand->spiCs, pCommand->pCmd[i]);
                pCommand->pCmd[i] = SPI_Read(pSpiHw);
            }
            /* Transfer data */
            for (i = 0; i < pCommand->dataSize; i ++)
            {
                SPI_Write(pSpiHw, pCommand->spiCs, pCommand->pData[i]);
                pCommand->pData[i] = SPI_Read(pSpiHw);
            }
            SPI_ReleaseCS(pSpiHw);
            
            /* Unlock */
            pSpid->semaphore ++;
            
            /* Callback */
            if (pCommand->callback)
            {
                pCommand->callback(0, pCommand->pArgument);
            }
        }
    }
    else
    {
        /* Initialize DMA controller using channel 0 for RX, 1 for TX. */
        if (_spid_configureDmaChannels(pSpid) )
            return SPID_ERROR_LOCK;
        if (_spid_configureLinkList(pSpiHw, pSpid->pDmad, pCommand))
            return SPID_ERROR_LOCK;

        // Initialize the callback
        pSpid->pCurrentCommand = pCommand;
        /* Enables the SPI to transfer and receive data. */
        SPI_Enable (pSpiHw );
    
        /* Start DMA 0(RX) && 1(TX) */
        if (DMAD_StartTransfer( pSpid->pDmad, pSpid->spiDmaRxChannel ))
            return SPID_ERROR_LOCK;
        if (DMAD_StartTransfer( pSpid->pDmad, pSpid->spiDmaTxChannel )) 
            return SPID_ERROR_LOCK;
    }
    return 0;    
}

/**
 * \brief SPI transfer ISR.
 */
void SPID_Handler( Spid *pSpid )
{
    pSpid = pSpid;
}

/**
 * \brief DMA transfer ISR for SPI driver.
 */
void SPID_DmaHandler( Spid *pSpid )
{
    DMAD_Handler( pSpid->pDmad );
}

/**
 * \brief Check if the SPI driver is busy.
 *
 * \param pSpid  Pointer to a Spid instance.
 * \returns 1 if the SPI driver is currently busy executing a command; otherwise
 */
uint32_t SPID_IsBusy(const Spid *pSpid)
{
    if (pSpid->semaphore == 0) {

        return 1;
    }
    else {

        return 0;
    }
}


