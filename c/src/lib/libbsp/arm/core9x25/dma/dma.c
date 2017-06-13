/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support 
 * ----------------------------------------------------------------------------
 * Copyright (c) 2011, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

/** \addtogroup dmad_module
 *
 * \section DmaHw Dma Hardware Interface Usage
 * <ul>
 * <li> The DMA controller can handle the transfer between peripherals and memory 
 * and so receives the triggers from the peripherals. The hardware interface number
 * are getting from DMAIF_Get_ChannelNumber().</li>
 
 * <li> DMAIF_IsValidatedPeripherOnDma() helps to check if the given DMAC has associated 
 * peripheral identifier coded by the given  peripheral.</li>
 * 
 * </ul>
*/
/*@{*/
/*@}*/

 /*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/
#include <bsp.h>
#include <at91sam9x25.h>
#include <bsp/dma.h>
#include <at91sam9x25_pmc.h>

#define PERI_DBGU    32
/** Number of DMA channels */
#define DMAC_CHANNEL_NUM          8
/** Max DMA single transfer size */
#define DMAC_MAX_BT_SIZE          0xFFFF

#define DB(x)

/** DMA hardware interface */
typedef struct _DmaHardwareInterface {
    uint8_t bDmac;                  /**< DMA Controller number */
    uint32_t bPeriphID;             /**< Peripheral ID */
    uint8_t bTransfer;              /**< Transfer type 0: Tx, 1 :Rx*/
    uint8_t bIfID;                  /**< DMA Interface ID */
} DmaHardwareInterface;




/** DMA channel control A */
typedef struct _DmaCtrlA {
    uint32_t btSize:16,     /**< Buffer Transfer size */
             scSize:3,      /**< Source Chunk Transfer size */
             reserve1:1,
             dcSize:3,      /**< Destination Chunk Transfer size */
             reserve2:1,
             srcWidth:2,    /**< Source width */
             reserve3:2,
             dstWidth:2,    /**< Destination width */
             reserve4:1,
             done:1;        /**< The transfer is done */
} sDmaCtrlA;

/** DMA channel control B */
typedef struct _DmaCtrlB {
    uint32_t sIf:2,         /**< Source Interface Selection Field */
             reserve1:2,
             dIf:2,         /**< Destination Interface Selection Field */
             reserve2:2,
             srcPip:1,      /**< Source Picture-in-picture mode enable */
             reserve3:3,
             dstPip:1,      /**< Destination Picture-in-picture mode enable */
             reserve4:3,
             srcDscr:1,     /**< Source Descriptor disabled */
             reserve5:3,
             dstDscr:1,     /**< Destination Descriptor disabled */
             fc:3,          /**< Flow Controller */
             srcIncr:2,     /**< Source Fixed/Dec/Inc setting */
             reserve6,
             dstIncr:2,     /**< Destination Fixed/Dec/Inc setting */
             iEn:1,         /**< Active low to enable interrupt */
             autoEn:1;      /**< Automatic multiple buffer transfer */
} sDmaCtrlB;

/** DMA channel Picture-In-Picture */
typedef struct _DmaPip {
    uint32_t pipHole:16,    /**< Hole size */
             pipBoundary:10,/**< Number of transfers to perform before
                                 hole increse */
             reserve:6;
} sDmaPIP;

/** Global DMA driver instance for all DMA transfers in application. */
sDmad gDmad;

/*hardware access function*/
void DMAC_Modified_Arbiter( Dmac *pDmac);
void DMAC_Enable( Dmac *pDmac );
void DMAC_Disable( Dmac *pDmac );
void DMAC_EnableIt (Dmac *pDmac, uint32_t dwInteruptMask );
void DMAC_DisableIt (Dmac *pDmac, uint32_t dwInteruptMask );
uint32_t DMAC_GetInterruptMask( Dmac *pDmac );
uint32_t DMAC_GetStatus( Dmac *pDmac );
uint32_t DMAC_GetMaskedStatus( Dmac *pDmac );
void DMAC_EnableChannel( Dmac *pDmac, uint8_t channel );
void DMAC_EnableChannels( Dmac *pDmac, uint8_t bmChannels );
void DMAC_DisableChannel( Dmac *pDmac, uint8_t channel );
void DMAC_DisableChannels( Dmac *pDmac, uint8_t bmChannels );
void DMAC_SuspendChannel( Dmac *pDmac, uint8_t channel );
void DMAC_KeepChannel( Dmac *pDmac, uint8_t channel );
void DMAC_RestoreChannel( Dmac *pDmac, uint8_t channel );
uint32_t DMAC_GetChannelStatus( Dmac *pDmac );
void DMAC_SetSourceAddr( Dmac *pDmac,
                                uint8_t channel,
                                uint32_t saddr );
uint32_t DMAC_GetSourceAddr( Dmac * pDmac,
                                uint8_t channel );
void DMAC_SetDestinationAddr( Dmac *pDmac,
                                     uint8_t channel,
                                     uint32_t daddr );
uint32_t DMAC_GetDestinationAddr( Dmac * pDmac,
                                     uint8_t channel );
void DMAC_SetDescriptorAddr( Dmac *pDmac,
                                    uint8_t channel,
                                    uint32_t descr,
                                    uint8_t descrif );
void DMAC_SetControlA( Dmac *pDmac,
                              uint8_t channel,
                              uint32_t controlA );
void DMAC_SetBufferSize( Dmac *pDmac,
                                uint8_t channel,
                                uint16_t bsize);
void DMAC_SetSingleTransferSize ( Dmac *pDmac,
                                         uint8_t channel,
                                         uint8_t srcWidth,
                                         uint8_t dstWidth );
void DMAC_SetChunkTransferSize ( Dmac *pDmac,
                                        uint8_t channel,
                                        uint8_t scSize,
                                        uint8_t dcSize);
void DMAC_SetControlB( Dmac *pDmac,
                              uint8_t channel,
                              uint32_t controlB );
void DMAC_EnableAutoMode( Dmac *pDmac, uint8_t channel );
void DMAC_DisableAutoMode( Dmac *pDmac, uint8_t channel );
void DMAC_SelectAHBInterface( Dmac *pDmac,
                                     uint8_t channel,
                                     uint8_t srcIf,
                                     uint8_t dstIf );
void DMAC_SetPipMode( Dmac *pDmac,
                             uint8_t channel,
                             uint8_t srcPip,
                             uint8_t dstPip );
void DMAC_SetDescFetchMode( Dmac *pDmac,
                                   uint8_t channel,
                                   uint8_t srcDscr,
                                   uint8_t dstDscr );
void DMAC_SetFlowControl( Dmac *pDmac,
                                 uint8_t channel,
                                 uint8_t flowControl );
void DMAC_SetCFG( Dmac *pDmac,
                         uint8_t channel,
                         uint32_t configuration );
void DMAC_SetReloadMode( Dmac *pDmac,
                                uint8_t channel,
                                uint8_t srcRep,
                                uint8_t dstRep );
void DMAC_SethandshakeInterface( Dmac *pDmac,
                                        uint8_t channel,
                                        uint8_t srcH2sel,
                                        uint8_t dstH2sel );
void DMAC_SetSourcePip( Dmac *pDmac,
                               uint8_t channel,
                               uint16_t pipHole,
                               uint16_t pipBoundary);
void DMAC_SetDestPip( Dmac *pDmac,
                             uint8_t channel,
                             uint16_t pipHole,
                             uint16_t pipBoundary);


/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/
/** Array of DMA Channel definition for SAM9XX5 chip*/
static const DmaHardwareInterface dmaHwIf[] = {
    /* dmac, peripheral,  T/R, Channel Number*/
       {0,   ID_HSMCI0,   0,   0},
       {0,   ID_HSMCI0,   1,   0},
       {0,   ID_SPI0,     0,   1},
       {0,   ID_SPI0,     1,   2},
       {0,   ID_USART0,   0,   3},
       {0,   ID_USART0,   1,   4},
       {0,   ID_USART1,   0,   5},
       {0,   ID_USART1,   1,   6},
       {0,   ID_TWI0,     0,   7},
       {0,   ID_TWI0,     1,   8},
       {0,   ID_TWI1,     0,   9},
       {0,   ID_TWI1,     1,  10},
       {0,   ID_UART0,    0,  11},
       {0,   ID_UART0,    1,  12},
       {0,   ID_SSC,      0,  13},
       {0,   ID_SSC,      1,  14},
    /* dmac 1 */   
       {1,   ID_HSMCI1,   0,   0},
       {1,   ID_HSMCI1,   1,   0},
       {1,   ID_SPI1,     0,   1},
       {1,   ID_SPI1,     1,   2},
       {1,   ID_SMD,      0,   3},
       {1,   ID_SMD,      1,   4},
       {1,   ID_TWI1,     0,   5},
       {1,   ID_TWI1,     1,   6},
       {1,   ID_ADC,      1,   7},
       {1,   PERI_DBGU,   0,   8},
       {1,   PERI_DBGU,   1,   9},
       {1,   ID_UART1,    0,  10},
       {1,   ID_UART1,    1,  11},
       {1,   ID_USART2,   0,  12},
       {1,   ID_USART2,   1,  13},
	   {1, 	 ID_USART3,   0,  14},
	   {1,   ID_USART3,   1,  15},
};

/*----------------------------------------------------------------------------
 *        Consts
 *----------------------------------------------------------------------------*/
 /** Number of recognized peripheral identifier code for DMA0/1. */
#define NUMPERIPHERAL   (sizeof(dmaHwIf) / sizeof (DmaHardwareInterface))

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Get peripheral identifier coded for hardware handshaking interface
 *
 * \param bDmac      DMA Controller number.
 * \param bPeriphID  Peripheral ID.
 * \param bTransfer  Transfer type 0: Tx, 1 :Rx.
 * \return 0-15 peripheral identifier coded.
 *         0xff : no associated peripheral identifier coded.
 */
uint8_t DMAIF_Get_ChannelNumber (uint8_t bDmac,
                                 uint8_t bPeriphID, 
                                 uint8_t bTransfer)
{
    uint8_t i;
    for (i = 0; i < NUMPERIPHERAL; i++)
    {
        if ((dmaHwIf[i].bDmac == bDmac) && (dmaHwIf[i].bPeriphID == bPeriphID) && (dmaHwIf[i].bTransfer == bTransfer))
        {
            return dmaHwIf[i].bIfID;
        }
    }
    return 0xff;
}

/**
 * \brief Check if the given DMAC has associated peripheral identifier coded by 
 * the given  peripheral.
 *
 * \param bDmac      DMA Controller number.
 * \param bPeriphID  Peripheral ID (0xff : memory only).
 * \return 1:  Is a validated peripher. 0: no associated peripheral identifier coded.
 */
uint8_t DMAIF_IsValidatedPeripherOnDma(uint8_t bDmac, uint8_t bPeriphID)
{
    uint8_t i;
    /* It is always validated when transfer to memory */
    if (bPeriphID == 0xFF) {
        return 1;
    }
    for (i = 0; i < NUMPERIPHERAL; i++)
    {
        if ((dmaHwIf[i].bDmac == bDmac) && (dmaHwIf[i].bPeriphID == bPeriphID))
        {
            return 1;
        }
    }
    return 0;
}

/**
 * \brief Try to allocate a DMA channel for on given controller.
 * \param pDmad  Pointer to DMA driver instance.
 * \param bDmac  DMA controller ID (0 ~ 1).
 * \param bSrcID Source peripheral ID, 0xFF for memory.
 * \param bDstID Destination peripheral ID, 0xFF for memory.
 * \return Channel number if allocation sucessful, return 
 * DMAD_ALLOC_FAILED if allocation failed.
 */
static uint32_t DMAD_AllocateDmacChannel( sDmad *pDmad,
                                          uint8_t bDmac,
                                          uint8_t bSrcID, 
                                          uint8_t bDstID)
{
    uint32_t i;

    /* Can't support peripheral to peripheral */
    if ((( bSrcID != DMAD_TRANSFER_MEMORY ) && ( bDstID != DMAD_TRANSFER_MEMORY ))) 
    {
        return DMAD_ALLOC_FAILED;
    }
    /* dma transfer from peripheral to memory */
    if ( bDstID == DMAD_TRANSFER_MEMORY)
    {
        if( (!DMAIF_IsValidatedPeripherOnDma(bDmac, bSrcID)) )
        {
            return DMAD_ALLOC_FAILED;
        }
    }
    /* dma transfer from memory to peripheral */
    if ( bSrcID == DMAD_TRANSFER_MEMORY )
    {
        if( (!DMAIF_IsValidatedPeripherOnDma(bDmac, bDstID)) )
        {
            return DMAD_ALLOC_FAILED;
        }
    }
    for (i = 0; i < pDmad->numChannels; i ++)
    {
        if ( pDmad->dmaChannels[bDmac][i].state == DMAD_FREE ) 
        {
            /* Allocate the channel */
            pDmad->dmaChannels[bDmac][i].state = DMAD_IN_USE;
            /* Get general informations */
            pDmad->dmaChannels[bDmac][i].bSrcPeriphID = bSrcID;
            pDmad->dmaChannels[bDmac][i].bDstPeriphID = bDstID;
            pDmad->dmaChannels[bDmac][i].bSrcTxIfID =
                DMAIF_Get_ChannelNumber(bDmac, bSrcID, 0);
            pDmad->dmaChannels[bDmac][i].bSrcRxIfID =
                DMAIF_Get_ChannelNumber(bDmac, bSrcID, 1);
            pDmad->dmaChannels[bDmac][i].bDstTxIfID =
                DMAIF_Get_ChannelNumber(bDmac, bDstID, 0);
            pDmad->dmaChannels[bDmac][i].bDstTxIfID =
                DMAIF_Get_ChannelNumber(bDmac, bDstID, 1);

            return ((bDmac << 8)) | ((i) & 0xFF);
        }
    }
    return DMAD_ALLOC_FAILED;
}

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Initialize DMA driver instance.
 * \param pDmad Pointer to DMA driver instance.
 * \param bPollingMode Polling DMA transfer:
 *                     1. Via DMAD_IsTransferDone(); or
 *                     2. Via DMAD_Handler().
 */
void DMAD_Initialize( sDmad *pDmad,
                      uint8_t bPollingMode )
{
    uint32_t i, j;
    static uint8_t isInit = 0;

	/*prevent from reinitializing the dma software module.*/
	if (isInit) 
		return ;
	isInit = 1;

	pDmad->pDmacs[0] = DMAC0;
    pDmad->pDmacs[1] = DMAC1;
    pDmad->pollingMode = bPollingMode;
    pDmad->numControllers = 2;
    pDmad->numChannels    = 8;
    
    for (i = 0; i < pDmad->numControllers; i ++)
    {
        for (j = 0; j < pDmad->numChannels; j ++)
        {
            pDmad->dmaChannels[i][j].fCallback = 0;
            pDmad->dmaChannels[i][j].pArg      = 0;

            pDmad->dmaChannels[i][j].bIrqOwner    = 0;
            pDmad->dmaChannels[i][j].bSrcPeriphID = 0;
            pDmad->dmaChannels[i][j].bDstPeriphID = 0;
            pDmad->dmaChannels[i][j].bSrcTxIfID   = 0;
            pDmad->dmaChannels[i][j].bSrcRxIfID   = 0;
            pDmad->dmaChannels[i][j].bDstTxIfID   = 0;
            pDmad->dmaChannels[i][j].bDstRxIfID   = 0;
            
            pDmad->dmaChannels[i][j].state = DMAD_FREE;
        }
    }
}

/**
 * \brief DMA interrupt handler
 * \param pDmad Pointer to DMA driver instance.
 */
void DMAD_Handler( sDmad *pDmad )
{
    Dmac *pDmac;
    sDmadChannel *pCh;
    uint32_t _iController, iChannel;
    uint32_t dmaSr, chSr;
    uint32_t dmaRc = DMAD_OK;
    
	DB(printk("in dmad_handler.\n"));
    for (_iController = 0; _iController < pDmad->numControllers; _iController ++)
    {
        pDmac = pDmad->pDmacs[_iController];

        /* Check raw status but not masked one for polling mode support */
        dmaSr = DMAC_GetStatus( pDmac );
        if ((dmaSr & 0x00FFFFFF) == 0) continue;

        chSr  = DMAC_GetChannelStatus( pDmac );

        for (iChannel = 0; iChannel < pDmad->numChannels; iChannel ++)
        {
            uint8_t bExec = 1;

            pCh = &pDmad->dmaChannels[_iController][iChannel];
            /* Error */
            if (dmaSr & (DMAC_EBCIDR_ERR0 << iChannel))
            {
                DMAC_DisableChannel( pDmac, iChannel );
                if (pCh->state > DMAD_IN_USE)   pCh->state = DMAD_STALL;
                dmaRc = DMAD_ERROR;
            }
            /* Chained buffer complete */
            else if (dmaSr & (DMAC_EBCIDR_CBTC0 << iChannel))
            {
                DMAC_DisableChannel( pDmac, iChannel );
                if (pCh->state > DMAD_IN_USE)   pCh->state = DMAD_IN_USE;
                dmaRc = DMAD_OK;
                
            }
            /* Buffer complete */
            else if (dmaSr & (DMAC_EBCIDR_BTC0 << iChannel))
            {
                dmaRc = DMAD_PARTIAL_DONE;
                /* Re-enable */
                if ((chSr & (DMAC_CHSR_ENA0 << iChannel)) == 0)
                {
                    DMAC_EnableChannel( pDmac, iChannel );
                }
            }
            else
            {
                bExec = 0;
            }
            /* Execute callback */
            if (bExec && pCh->fCallback)
            {
                pCh->fCallback(dmaRc, pCh->pArg);
            }
        }
    }
}

/**
 * \brief Allocate a DMA channel for upper layer.
 * \param pDmad  Pointer to DMA driver instance.
 * \param bSrcID Source peripheral ID, 0xFF for memory.
 * \param bDstID Destination peripheral ID, 0xFF for memory.
 * \return Channel number if allocation sucessful, return 
 * DMAD_ALLOC_FAILED if allocation failed.
 */
uint32_t DMAD_AllocateChannel( sDmad *pDmad,
                               uint8_t bSrcID, 
                               uint8_t bDstID)
{
    uint32_t _iController;
    uint32_t _dwChannel = DMAD_ALLOC_FAILED;

    for ( _iController = 0; _iController < pDmad->numControllers; _iController ++)
    {
        _dwChannel = DMAD_AllocateDmacChannel( pDmad, _iController,
                                              bSrcID, bDstID );
        if (_dwChannel != DMAD_ALLOC_FAILED)
            break;
    }
    return _dwChannel;
}

/**
 * \brief Free the specified DMA channel.
 * \param pDmad     Pointer to DMA driver instance.
 * \param _dwChannel ControllerNumber << 8 | ChannelNumber.
 */
eDmadRC DMAD_FreeChannel( sDmad *pDmad, uint32_t _dwChannel )
{
    uint8_t _iController = (_dwChannel >> 8);
    uint8_t iChannel    = (_dwChannel) & 0xFF;
    
    switch ( pDmad->dmaChannels[_iController][iChannel].state )
    {

    case DMAD_IN_XFR:
        return DMAD_BUSY;

    case DMAD_IN_USE:
        pDmad->dmaChannels[_iController][iChannel].state = DMAD_FREE;
        break;
    }   
	return DMAD_OK;
}

/**
 * \brief Set the callback function for DMA channel transfer.
 * \param pDmad     Pointer to DMA driver instance.
 * \param _dwChannel ControllerNumber << 8 | ChannelNumber.
 * \param fCallback Pointer to callback function.
 * \param pArg Pointer to optional argument for callback.
 */
eDmadRC DMAD_SetCallback( sDmad *pDmad, uint32_t _dwChannel,
                          DmadTransferCallback fCallback, void* pArg )
{
    uint8_t _iController = (_dwChannel >> 8);
    uint8_t iChannel    = (_dwChannel) & 0xFF;
    
    if ( pDmad->dmaChannels[_iController][iChannel].state == DMAD_FREE )
        return DMAD_ERROR;
    else if ( pDmad->dmaChannels[_iController][iChannel].state == DMAD_IN_XFR )
        return DMAD_BUSY;
    
    pDmad->dmaChannels[_iController][iChannel].fCallback = fCallback;
    pDmad->dmaChannels[_iController][iChannel].pArg = pArg;
    
    return DMAD_OK;
}

/**
 * \brief Configure Picture-in-Picture mode for DMA transfer.
 * \param pDmad     Pointer to DMA driver instance.
 * \param _dwChannel ControllerNumber << 8 | ChannelNumber.
 * \param srcPIP    Source PIP setting.
 * \param dstPIP    Destination PIP setting.
 */
eDmadRC DMAD_ConfigurePIP( sDmad *pDmad, 
                           uint32_t _dwChannel,
                           uint32_t dwSrcPIP, 
                           uint32_t dwDstPIP )
{
    uint8_t _iController = (_dwChannel >> 8);
    uint8_t iChannel    = (_dwChannel) & 0xFF;
    
    Dmac *pDmac = pDmad->pDmacs[_iController];
    if ( pDmad->dmaChannels[_iController][iChannel].state == DMAD_FREE )
        return DMAD_ERROR;
    else if ( pDmad->dmaChannels[_iController][iChannel].state == DMAD_IN_XFR )
        return DMAD_BUSY;

    DMAC_SetPipMode(pDmac, iChannel, dwSrcPIP, dwDstPIP);
    return DMAD_OK;
}

/**
 * \brief Enable clock of the DMA peripheral, Enable the dma peripheral, 
 * configure configuration register for DMA transfer.
 * \param pDmad     Pointer to DMA driver instance.
 * \param _dwChannel ControllerNumber << 8 | ChannelNumber.
 * \param dwCfg     Configuration value.
 */
eDmadRC DMAD_PrepareChannel( sDmad *pDmad, 
                             uint32_t _dwChannel,
                             uint32_t dwCfg )
{
    uint8_t _iController = (_dwChannel >> 8);
    uint8_t iChannel    = (_dwChannel) & 0xFF;
    uint32_t _dwdmaId;

    Dmac *pDmac = pDmad->pDmacs[_iController];

    if ( pDmad->dmaChannels[_iController][iChannel].state == DMAD_FREE )
        return DMAD_ERROR;
    else if ( pDmad->dmaChannels[_iController][iChannel].state == DMAD_IN_XFR )
        return DMAD_BUSY;
    DMAC_SetCFG( pDmac, iChannel, dwCfg );

    _dwdmaId = (_iController == 0) ? ID_DMAC0 : ID_DMAC1;
    /* Enable clock of the DMA peripheral */
    if (!PMC_IsPeriphEnabled( _dwdmaId ))
    {
        PMC_EnablePeripheral( _dwdmaId );
    }
    /* Enables the DMAC peripheral. */
    DMAC_Enable( pDmac );
    /* Disables DMAC interrupt for the given channel. */
    DMAC_DisableIt (pDmac, 
                    (DMAC_EBCIDR_BTC0 << iChannel)
                   |(DMAC_EBCIDR_CBTC0 << iChannel)
                   |(DMAC_EBCIDR_ERR0 << iChannel) );
    /* Disable the given dma channel. */
    DMAC_DisableChannel( pDmac, iChannel );
    /* Clear dummy status */
    DMAC_GetChannelStatus( pDmac );
    DMAC_GetStatus (pDmac);
    return DMAD_OK;
}

/**
 * \brief Check if DMA transfer is finished.
 *        In polling mode DMAD_Handler() is polled.
 * \param pDmad     Pointer to DMA driver instance.
 * \param _dwChannel ControllerNumber << 8 | ChannelNumber.
 */
eDmadRC DMAD_IsTransferDone( sDmad *pDmad, uint32_t _dwChannel )
{
    uint8_t _iController = (_dwChannel >> 8);
    uint8_t iChannel    = (_dwChannel) & 0xFF;

    if ( pDmad->dmaChannels[_iController][iChannel].state == DMAD_FREE )
        return DMAD_ERROR;
    else if ( pDmad->dmaChannels[_iController][iChannel].state == DMAD_IN_XFR )
    {
        if ( pDmad->pollingMode ) DMAD_Handler( pDmad );
        return DMAD_BUSY;
    }
    return DMAD_OK;
}

/**
 * \brief Clear the automatic mode that services the next-to-last 
    buffer transfer.
 * \param pDmad     Pointer to DMA driver instance.
 * \param _dwChannel ControllerNumber << 8 | ChannelNumber.
 */
void DMAD_ClearAuto( sDmad *pDmad, uint32_t _dwChannel )
{
    uint8_t _iController = (_dwChannel >> 8);
    uint8_t iChannel    = (_dwChannel) & 0xFF;
    Dmac *pDmac;
    
    pDmac = pDmad->pDmacs[_iController];
    DMAC_DisableAutoMode( pDmac, iChannel );   
}

/**
 * \brief Start DMA transfer.
 * \param pDmad     Pointer to DMA driver instance.
 * \param _dwChannel ControllerNumber << 8 | ChannelNumber.
 */
eDmadRC DMAD_StartTransfer( sDmad *pDmad, uint32_t _dwChannel )
{
    uint8_t _iController = (_dwChannel >> 8);
    uint8_t iChannel    = (_dwChannel) & 0xFF;
    
    Dmac *pDmac = pDmad->pDmacs[_iController];

    if ( pDmad->dmaChannels[_iController][iChannel].state == DMAD_FREE )
        return DMAD_ERROR;
    else if ( pDmad->dmaChannels[_iController][iChannel].state == DMAD_IN_XFR )
        return DMAD_BUSY;
    /* Change state to transferring */
    pDmad->dmaChannels[_iController][iChannel].state = DMAD_IN_XFR;
    DMAC_EnableChannel(pDmac, iChannel);
    if ( pDmad->pollingMode == 0 )
    {
        /* Monitor status in interrupt handler */
        DMAC_EnableIt(pDmac, (DMAC_EBCIDR_BTC0 << iChannel)
                            |(DMAC_EBCIDR_CBTC0 << iChannel)
                            |(DMAC_EBCIDR_ERR0 << iChannel) );
    }
    
    return DMAD_OK;
}

/**
 * \brief Start DMA transfers on the same controller.
 * \param pDmad      Pointer to DMA driver instance.
 * \param bDmac      DMA Controller number. 
 * \param bmChannels Channels bitmap.
 */
eDmadRC DMAD_StartTransfers( sDmad *pDmad, uint8_t bDmac, uint32_t bmChannels )
{
    uint32_t iChannel;
    uint32_t bmChs = 0, bmIts = 0;

    Dmac *pDmac = pDmad->pDmacs[bDmac];

    for (iChannel = 0; iChannel < pDmad->numChannels; iChannel ++)
    {
        uint32_t bmChBit = 1 << iChannel;

        /* Skipped channels */
        if ( pDmad->dmaChannels[bDmac][iChannel].state == DMAD_FREE )
            continue;
        else if ( pDmad->dmaChannels[bDmac][iChannel].state == DMAD_IN_XFR )
            continue;
        /* Log to start bit map */
        if (bmChannels & bmChBit)
        {
            bmChs |= bmChBit;
            bmIts |= (  (DMAC_EBCIDR_BTC0 << iChannel)
                       |(DMAC_EBCIDR_CBTC0 << iChannel)
                       |(DMAC_EBCIDR_ERR0 << iChannel) );
            /* Change state */
            pDmad->dmaChannels[bDmac][iChannel].state = DMAD_IN_XFR;
        }
    }

    DMAC_EnableChannels(pDmac, bmChs);
    if ( pDmad->pollingMode == 0 )
    {
        /* Monitor status in interrupt handler */
        DMAC_EnableIt( pDmac, bmIts );
    }
    
    return DMAD_OK;
}

/**
 * \brief Stop DMA transfer.
 * \param pDmad     Pointer to DMA driver instance.
 * \param _dwChannel ControllerNumber << 8 | ChannelNumber.
 */
eDmadRC DMAD_StopTransfer( sDmad *pDmad, uint32_t _dwChannel )
{
    uint8_t _iController = (_dwChannel >> 8);
    uint8_t iChannel    = (_dwChannel) & 0xFF;
    
    Dmac *pDmac = pDmad->pDmacs[_iController];
    sDmadChannel *pCh = &pDmad->dmaChannels[_iController][iChannel];

    uint32_t to = 0x1000;

    if ( pDmad->dmaChannels[_iController][iChannel].state == DMAD_FREE )
        return DMAD_ERROR;

    if ( pDmad->dmaChannels[_iController][iChannel].state != DMAD_IN_XFR )
        return DMAD_OK;

    /* Suspend */
    DMAC_SuspendChannel(pDmac, iChannel);

    /* Poll empty */
    for (;to; to --)
    {
        if (DMAC_GetChannelStatus(pDmac) & (DMAC_CHSR_EMPT0 << iChannel))
        {
            break;
        }
    }

    /* Disable channel */
    DMAC_DisableChannel(pDmac, iChannel);
    /* Disable interrupts */
    DMAC_DisableIt(pDmac, (DMAC_EBCIDR_BTC0 << iChannel)
                         |(DMAC_EBCIDR_CBTC0 << iChannel)
                         |(DMAC_EBCIDR_ERR0 << iChannel) );
    /* Clear pending status */
    DMAC_GetChannelStatus(pDmac);
    DMAC_GetStatus(pDmac);
    /* Resume */
    DMAC_RestoreChannel(pDmac, iChannel);
    /* Change state */
    pDmad->dmaChannels[_iController][iChannel].state = DMAD_IN_USE;
    /* Invoke callback */
    if (pCh->fCallback) pCh->fCallback(DMAD_CANCELED, pCh->pArg);
    return DMAD_OK;
}

/**
 * \brief Configure DMA for a single transfer.
 * \param pDmad     Pointer to DMA driver instance.
 * \param _dwChannel ControllerNumber << 8 | ChannelNumber.
 */
eDmadRC DMAD_PrepareSingleTransfer( sDmad *pDmad, 
                                    uint32_t _dwChannel,
                                    sDmaTransferDescriptor *pXfrDesc )
{
    uint8_t _iController = (_dwChannel >> 8);
    uint8_t iChannel    = (_dwChannel) & 0xFF;
    Dmac *pDmac = pDmad->pDmacs[_iController];

    if ( pDmad->dmaChannels[_iController][iChannel].state == DMAD_FREE )
        return DMAD_ERROR;
    if ( pDmad->dmaChannels[_iController][iChannel].state == DMAD_IN_XFR )
        return DMAD_BUSY;
    
    DMAC_SetSourceAddr(pDmac, iChannel, pXfrDesc->dwSrcAddr);
    DMAC_SetDestinationAddr(pDmac, iChannel, pXfrDesc->dwDstAddr);
    DMAC_SetDescriptorAddr(pDmac, iChannel, 0, 0);
    DMAC_SetControlA(pDmac, iChannel, pXfrDesc->dwCtrlA);
    DMAC_SetControlB(pDmac, iChannel, pXfrDesc->dwCtrlB);

    return DMAD_OK;
}

/**
 * \brief Configure DMA multi-buffer transfers using linked lists
 * \param pDmad     Pointer to DMA driver instance.
 * \param _dwChannel ControllerNumber << 8 | ChannelNumber.
 * \param pXfrDesc  Pointer to DMA Linked List.
 */
eDmadRC DMAD_PrepareMultiTransfer( sDmad *pDmad, 
                                   uint32_t _dwChannel,
                                   sDmaTransferDescriptor *pXfrDesc )
{
    uint8_t _iController = (_dwChannel >> 8);
    uint8_t iChannel    = (_dwChannel) & 0xFF;
    
    Dmac *pDmac = pDmad->pDmacs[_iController];

    if ( pDmad->dmaChannels[_iController][iChannel].state == DMAD_FREE )
        return DMAD_ERROR;
    if ( pDmad->dmaChannels[_iController][iChannel].state == DMAD_IN_XFR )
        return DMAD_BUSY;
    
    DMAC_SetDescriptorAddr( pDmac, iChannel, (uint32_t)pXfrDesc, 0 );
    DMAC_SetControlB( pDmac, iChannel, 0);

    return DMAD_OK;
}


/**
 * \brief Configures an DMAC peripheral with modified round robin arbiter.
 *
 * \param pDmac  Pointer to the DMAC peripheral.
 */
void DMAC_Modified_Arbiter( Dmac *pDmac)
{
    pDmac->DMAC_GCFG = DMAC_GCFG_ARB_CFG ;
}

/**
 * \brief Enables a DMAC peripheral.
 *
 * \param pDmac  Pointer to the DMAC peripheral.
 */
void DMAC_Enable( Dmac *pDmac )
{
    pDmac->DMAC_EN = DMAC_EN_ENABLE;
}

/**
 * \brief Disables a DMAC peripheral.
 *
 * \param pDmac Pointer to the DMAC peripheral .
 */
void DMAC_Disable( Dmac *pDmac )
{
    pDmac->DMAC_EN = ~(uint32_t)DMAC_EN_ENABLE;
}

/**
 * \brief Enables DMAC interrupt.
 *
 * \param pDmac Pointer to the DMAC peripheral.
 * \param dwInteruptMask IT to be enabled.
 */
void DMAC_EnableIt (Dmac *pDmac, uint32_t dwInteruptMask )
{
    pDmac->DMAC_EBCIER = dwInteruptMask;
}

/**
 * \brief Disables DMAC interrupt
 *
 * \param pDmac Pointer to the DMAC peripheral.
 * \param dwInteruptMask IT to be enabled
 */
void DMAC_DisableIt (Dmac *pDmac, uint32_t dwInteruptMask )
{
    pDmac->DMAC_EBCIDR = dwInteruptMask;
}

/**
 * \brief Get DMAC Interrupt Mask Status.
 *
 * \param pDmac Pointer to the DMAC peripheral.
 * \return DMAC Error, buffer transfer and chained buffer
 *  transfer interrupt mask register value.
 */
uint32_t DMAC_GetInterruptMask( Dmac *pDmac )
{
    return (pDmac->DMAC_EBCIMR);
}

/**
 * \brief Get DMAC Error, buffer transfer and chained buffer
 *  transfer status register.
 *
 * \param pDmac Pointer to the DMAC peripheral.
 * \return DMAC Error, buffer transfer and chained buffer
 *  transfer status register.
 */
uint32_t DMAC_GetStatus( Dmac *pDmac )
{
    return (pDmac->DMAC_EBCISR);
}

/**
 * \brief Get DMAC Error, buffer transfer and chained buffer
 *  transfer status register of the given DMAC peripheral, but
 *  masking interrupt sources which are not currently enabled.
 *
 * \param pDmac Pointer to the DMAC peripheral.
 * \return DMAC Error, buffer transfer and chained buffer
 *  transfer status register.
 */
uint32_t DMAC_GetMaskedStatus( Dmac *pDmac )
{
    uint32_t _dwStatus;
    _dwStatus = pDmac->DMAC_EBCISR;
    _dwStatus &= pDmac->DMAC_EBCIMR;
    return _dwStatus;
}

/**
 * \brief enables the relevant channel of given DMAC.
 *
 * \param pDmac Pointer to the DMAC peripheral.
 * \param channel Particular channel number.
 */
void DMAC_EnableChannel( Dmac *pDmac, uint8_t channel )
{
    pDmac->DMAC_CHER |= DMAC_CHER_ENA0 << channel;
}

/**
 * \brief enables the relevant channels of given DMAC.
 *
 * \param pDmac Pointer to the DMAC peripheral.
 * \param bmChannels Channels bitmap.
 */
void DMAC_EnableChannels( Dmac *pDmac, uint8_t bmChannels )
{
    pDmac->DMAC_CHER = bmChannels;
}

/**
 * \brief Disables the relevant channel of given DMAC.
 *
 * \param pDmac Pointer to the DMAC peripheral.
 * \param channel Particular channel number.
 */
void DMAC_DisableChannel( Dmac *pDmac, uint8_t channel )
{
    pDmac->DMAC_CHDR |= DMAC_CHDR_DIS0 << channel;
}

/**
 * \brief Disables the relevant channels of given DMAC.
 *
 * \param pDmac Pointer to the DMAC peripheral.
 * \param bmChannels Channels bitmap.
 */
void DMAC_DisableChannels( Dmac *pDmac, uint8_t bmChannels )
{
    pDmac->DMAC_CHDR = bmChannels;
}

/**
 * \brief freezes the relevant channel of given DMAC.
 *
 * \param pDmac Pointer to the DMAC peripheral.
 * \param channel Particular channel number.
 */
void DMAC_SuspendChannel( Dmac *pDmac, uint8_t channel )
{
    pDmac->DMAC_CHER |= DMAC_CHER_SUSP0 << channel;
}

/**
 * \brief resumes the current channel from an automatic
 * stall state of given DMAC.
 *
 * \param pDmac Pointer to the DMAC peripheral.
 * \param channel Particular channel number.
 */
void DMAC_KeepChannel( Dmac *pDmac, uint8_t channel )
{
    pDmac->DMAC_CHER |= DMAC_CHER_KEEP0 << channel;
}

/**
 * \brief resume the channel transfer restoring its context.
 *
 * \param pDmac Pointer to the DMAC peripheral.
 * \param channel Particular channel number.
 */
void DMAC_RestoreChannel( Dmac *pDmac, uint8_t channel )
{
    pDmac->DMAC_CHDR |= DMAC_CHDR_RES0 << channel;
}

/**
 * \brief Get DMAC channel handler Status.
 *
 * \param pDmac Pointer to the DMAC peripheral.
 * \return DMAC channel handler status register.
 */
uint32_t DMAC_GetChannelStatus( Dmac *pDmac )
{
    return (pDmac->DMAC_CHSR);
}

/**
 * \brief Set DMAC source address in a DMAC channel.
 *
 * \param pDmac Pointer to the DMAC peripheral.
 * \param channel Particular channel number.
 * \param saddr sources address.
 * \note This register must be aligned with the source transfer width.
 */
void DMAC_SetSourceAddr( Dmac *pDmac,
                         uint8_t channel,
                         uint32_t saddr )
{
    pDmac->DMAC_CH_NUM[channel].DMAC_SADDR = saddr;
}

/**
 * \brief Return DMAC source address of a DMAC channel.
 *
 * \param pDmac Pointer to the DMAC peripheral.
 * \param channel Particular channel number.
 */
uint32_t DMAC_GetSourceAddr( Dmac *pDmac,
                             uint8_t channel )
{
    return pDmac->DMAC_CH_NUM[channel].DMAC_SADDR;
}

/**
 * \brief Set DMAC destination address in a DMAC channel.
 *
 * \param pDmac Pointer to the DMAC peripheral.
 * \param channel Particular channel number.
 * \param daddr sources address.
 * \note This register must be aligned with the source transfer width.
 */
void DMAC_SetDestinationAddr( Dmac *pDmac,
                              uint8_t channel,
                              uint32_t daddr )
{
    pDmac->DMAC_CH_NUM[channel].DMAC_DADDR = daddr;
}

/**
 * \brief Return DMAC destination address of a DMAC channel.
 *
 * \param pDmac Pointer to the DMAC peripheral.
 * \param channel Particular channel number.
 */
uint32_t DMAC_GetDestinationAddr( Dmac *pDmac,
                                  uint8_t channel )
{
    return pDmac->DMAC_CH_NUM[channel].DMAC_DADDR;
}

/**
 * \brief Set DMAC descriptor address used by a DMAC channel.
 *
 * \param pDmac Pointer to the DMAC peripheral.
 * \param channel Particular channel number.
 * \param descr Buffer Transfer descriptor address
 * \param descrIf AHB-Lite interface to be fetched
 */
void DMAC_SetDescriptorAddr( Dmac *pDmac,
                             uint8_t channel,
                             uint32_t descr,
                             uint8_t descrif )
{
    //pDmac->DMAC_CH_NUM[channel].DMAC_DSCR = DMAC_DSCR_DSCR( descr ) | descrif;
    pDmac->DMAC_CH_NUM[channel].DMAC_DSCR = ( descr & 0xFFFFFFFC ) | descrif;
}

/**
 * \brief Set DMAC controlA used by a DMAC channel.
 *
 * \param pDmac Pointer to the DMAC peripheral.
 * \param channel Particular channel number.
 * \param controlA Configuration for controlA register.
 */
void DMAC_SetControlA( Dmac *pDmac,
                       uint8_t channel,
                       uint32_t controlA )
{
    pDmac->DMAC_CH_NUM[channel].DMAC_CTRLA = controlA;
}


/**
 * \brief Set DMAC buffer transfer size used by a DMAC channel.
 *
 * \param pDmac Pointer to the DMAC peripheral.
 * \param channel Particular channel number.
 * \param bsize number of transfers to be performed.
 */
void DMAC_SetBufferSize( Dmac *pDmac,
                         uint8_t channel,
                         uint16_t bsize)
{
   pDmac->DMAC_CH_NUM[channel].DMAC_CTRLA &= ~DMAC_CTRLA_BTSIZE_Msk;
   pDmac->DMAC_CH_NUM[channel].DMAC_CTRLA |= DMAC_CTRLA_BTSIZE( bsize );
}

/**
 * \brief Set DMAC single transfer size used by a DMAC channel.
 *
 * \param pDmac Pointer to the DMAC peripheral.
 * \param channel Particular channel number.
 * \param srcWidth source width for single transfer.
 * \param dstWidth destination width for single transfer.
 */
 void DMAC_SetSingleTransferSize ( Dmac *pDmac,
                                  uint8_t channel,
                                  uint8_t srcWidth,
                                  uint8_t dstWidth )
{
   pDmac->DMAC_CH_NUM[channel].DMAC_CTRLA &= ~DMAC_CTRLA_SRC_WIDTH_Msk;
   pDmac->DMAC_CH_NUM[channel].DMAC_CTRLA &= ~DMAC_CTRLA_DST_WIDTH_Msk;
   pDmac->DMAC_CH_NUM[channel].DMAC_CTRLA |= srcWidth | dstWidth;
}

/**
 * \brief Set DMAC single transfer size used by a DMAC channel.
 *
 * \param pDmac Pointer to the DMAC peripheral.
 * \param channel Particular channel number.
 * \param scSize Size of source chunk transfer.
 * \param dcSize Size of destination chunk transfer.
 */
void DMAC_SetChunkTransferSize ( Dmac *pDmac,
                                 uint8_t channel,
                                 uint8_t scSize,
                                 uint8_t dcSize)
{
   pDmac->DMAC_CH_NUM[channel].DMAC_CTRLA &= ~DMAC_CTRLA_SCSIZE_Msk;
   pDmac->DMAC_CH_NUM[channel].DMAC_CTRLA &= ~DMAC_CTRLA_DCSIZE_Msk;
   pDmac->DMAC_CH_NUM[channel].DMAC_CTRLA |= scSize | dcSize;
}

/**
 * \brief Set DMAC controlB used by a DMAC channel.
 *
 * \param pDmac Pointer to the DMAC peripheral.
 * \param channel Particular channel number.
 * \param controlB Configuration for controlA register.
 */
void DMAC_SetControlB( Dmac *pDmac,
                       uint8_t channel,
                       uint32_t controlB )
{
    pDmac->DMAC_CH_NUM[channel].DMAC_CTRLB = controlB;
}

/**
 * \brief Enables DMAC automatic multiple buffer transfer 
 *        mode used by a DMAC channel.
 *
 * \param pDmac Pointer to the DMAC peripheral.
 * \param channel Particular channel number.
 */
void DMAC_EnableAutoMode( Dmac *pDmac, uint8_t channel )
{
    pDmac->DMAC_CH_NUM[channel].DMAC_CTRLB |= DMAC_CTRLB_AUTO;
}

/**
 * \brief Disable DMAC automatic multiple buffer transfer 
 *        mode used by a DMAC channel.
 *
 * \param pDmac Pointer to the DMAC peripheral.
 * \param channel Particular channel number.
 */
void DMAC_DisableAutoMode( Dmac *pDmac, uint8_t channel )
{
    pDmac->DMAC_CH_NUM[channel].DMAC_CTRLB &= ~DMAC_CTRLB_AUTO;
}

/**
 * \brief Select DMAC AHB source interface.
 *
 * \param pDmac Pointer to the DMAC peripheral.
 * \param channel Particular channel number.
 * \param srcIf Source AHB-Lite interface.
 * \param dstIf Destination AHB-Lite interface.
 */
void DMAC_SelectAHBInterface( Dmac *pDmac,
                              uint8_t channel,
                              uint8_t srcIf,
                              uint8_t dstIf )
{
    pDmac->DMAC_CH_NUM[channel].DMAC_CTRLB &= ~DMAC_CTRLB_SIF_Msk;
    pDmac->DMAC_CH_NUM[channel].DMAC_CTRLB &= ~DMAC_CTRLB_DIF_Msk;
    pDmac->DMAC_CH_NUM[channel].DMAC_CTRLB |= srcIf | dstIf;
}


/**
 * \brief Set DMAC Picture-in-Picture mode for source and destination.
 *
 * \param pDmac Pointer to the DMAC peripheral.
 * \param channel Particular channel number.
 * \param srcPip Source picture-in-picture mode.
 * \param srcPip destination picture-in-picture mode.
 */
void DMAC_SetPipMode( Dmac *pDmac,
                      uint8_t channel,
                      uint8_t srcPip,
                      uint8_t dstPip )
{
    pDmac->DMAC_CH_NUM[channel].DMAC_CTRLB &= ~DMAC_CTRLB_SRC_PIP;
    pDmac->DMAC_CH_NUM[channel].DMAC_CTRLB &= ~DMAC_CTRLB_DST_PIP;
    pDmac->DMAC_CH_NUM[channel].DMAC_CTRLB |= srcPip | dstPip;
}

/**
 * \brief Set DMAC buffer descriptor fetch mode.
 *
 * \param pDmac Pointer to the DMAC peripheral.
 * \param channel Particular channel number.
 * \param srcDscr Source buffer descriptor fetch mode.
 * \param dstDscr destination buffer descriptor fetch mode.
 */
void DMAC_SetDescFetchMode( Dmac *pDmac,
                            uint8_t channel,
                            uint8_t srcDscr,
                            uint8_t dstDscr )
{
    pDmac->DMAC_CH_NUM[channel].DMAC_CTRLB &= ~DMAC_CTRLB_SRC_DSCR;
    pDmac->DMAC_CH_NUM[channel].DMAC_CTRLB &= ~DMAC_CTRLB_DST_DSCR;
    pDmac->DMAC_CH_NUM[channel].DMAC_CTRLB |= srcDscr | dstDscr;
}

/**
 * \brief Set DMAC control B register Flow control bit field.
 *
 * \param pDmac Pointer to the DMAC peripheral.
 * \param channel Particular channel number.
 * \param flow which device controls the size of the buffer transfer.
 */
void DMAC_SetFlowControl( Dmac *pDmac,
                          uint8_t channel,
                          uint8_t flowControl )
{
    pDmac->DMAC_CH_NUM[channel].DMAC_CTRLB &= ~DMAC_CTRLB_FC_Msk;
    pDmac->DMAC_CH_NUM[channel].DMAC_CTRLB |= flowControl;
}

/**
 * \brief Set DMAC CFG register used by a DMAC channel.
 *
 * \param pDmac Pointer to the DMAC peripheral.
 * \param channel Particular channel number.
 * \param configuration Configuration for CFG register.
 */
void DMAC_SetCFG( Dmac *pDmac,
                  uint8_t channel,
                  uint32_t configuration )
{
    pDmac->DMAC_CH_NUM[channel].DMAC_CFG = configuration;
}

/**
 * \brief Set DMAC buffer reload mode.
 *
 * \param pDmac Pointer to the DMAC peripheral.
 * \param channel Particular channel number.
 * \param srcRep Source buffer reload mode.
 * \param dstRep Destination buffer reload mode.
 */
void DMAC_SetReloadMode( Dmac *pDmac,
                         uint8_t channel,
                         uint8_t srcRep,
                         uint8_t dstRep )
{
    pDmac->DMAC_CH_NUM[channel].DMAC_CFG &= ~DMAC_CFG_SRC_REP;
    pDmac->DMAC_CH_NUM[channel].DMAC_CFG &= ~DMAC_CFG_DST_REP;
    pDmac->DMAC_CH_NUM[channel].DMAC_CFG |= srcRep | dstRep;
}

/**
 * \brief Set DMAC SW/HW handshaking interface used to 
 *        trigger a transfer request.
 *
 * \param pDmac Pointer to the DMAC peripheral.
 * \param channel Particular channel number.
 * \param srcH2sel Source handshaking interface. 
 * \param dstH2sel Destination handshaking interface.
 */
void DMAC_SethandshakeInterface( Dmac *pDmac,
                                 uint8_t channel,
                                 uint8_t srcH2sel,
                                 uint8_t dstH2sel )
{
    pDmac->DMAC_CH_NUM[channel].DMAC_CFG &= ~DMAC_CFG_SRC_H2SEL;
    pDmac->DMAC_CH_NUM[channel].DMAC_CFG &= ~DMAC_CFG_DST_H2SEL;
    pDmac->DMAC_CH_NUM[channel].DMAC_CFG |= srcH2sel | dstH2sel;
}


/**
 * \brief Set DMAC source PIP configuration used by a DMAC channel.
 *
 * \param pDmac Pointer to the DMAC peripheral.
 * \param channel Particular channel number.
 * \param pipHole the value to add to the address when the programmable
 *                boundary has been reached.
 * \param pipBoundary the number of source transfers to perform before
 *                the automatic address increment operation.
 */
void DMAC_SetSourcePip( Dmac *pDmac,
                        uint8_t channel,
                        uint16_t pipHole,
                        uint16_t pipBoundary)

{
   pDmac->DMAC_CH_NUM[channel].DMAC_SPIP = DMAC_SPIP_SPIP_HOLE( pipHole ) |
                                           DMAC_SPIP_SPIP_BOUNDARY( pipBoundary );
}

/**
 * \brief Set DMAC destination PIP configuration used by a DMAC channel.
 *
 * \param pDmac Pointer to the DMAC peripheral.
 * \param channel Particular channel number.
 * \param pipHole the value to add to the address when the programmable
 *                boundary has been reached.
 * \param pipBoundary the number of source transfers to perform before
 *                the automatic address increment operation.
 */
void DMAC_SetDestPip( Dmac *pDmac,
                      uint8_t channel,
                      uint16_t pipHole,
                      uint16_t pipBoundary)

{
   pDmac->DMAC_CH_NUM[channel].DMAC_DPIP = DMAC_DPIP_DPIP_HOLE( pipHole ) |
                                           DMAC_DPIP_DPIP_BOUNDARY( pipBoundary );
}



