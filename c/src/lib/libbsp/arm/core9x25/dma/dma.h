
#ifndef __DMA_H__
#define __DMA_H__

#define DMAD_TRANSFER_MEMORY  0xFF   /**< DMA transfer from or to memory */
#define DMAD_ALLOC_FAILED     0xFFFF /**< Channel allocate failed */

#define DMAD_TRANSFER_TX      0
#define DMAD_TRANSFER_RX      1


/** DMA status or return code */
typedef enum _DmadStatus {
    DMAD_OK = 0,        /**< Operation is sucessful */
    DMAD_BUSY,          /**< Channel occupied or transfer not finished */
    DMAD_PARTIAL_DONE,  /**< Partial of LLI finished (on each buffer done) */
    DMAD_ERROR,         /**< Operation failed */
    DMAD_CANCELED       /**< Operation canceled */
} eDmadStatus, eDmadRC;

/** DMA state for channel */
typedef enum _DmadState {
    DMAD_FREE = 0,      /**< Free channel */
    DMAD_IN_USE,        /**< Allocated to some peripheral */
    DMAD_IN_XFR,        /**< Transferring something */
    DMAD_STALL          /**< Error stall */
} eDmadState;

/** DMA transfer callback */
typedef void (*DmadTransferCallback)(uint32_t status, void* pArg);

/** DMA driver channel */
typedef struct _DmadChannel {
    DmadTransferCallback fCallback; /**< Callback */
    void* pArg;                     /**< Callback argument */

    uint8_t bIrqOwner;              /**< Uses DMA handler or external one */
    uint8_t bSrcPeriphID;           /**< HW ID for source */
    uint8_t bDstPeriphID;           /**< HW ID for destination */
    uint8_t bSrcTxIfID;             /**< DMA Tx Interface ID for source */
    uint8_t bSrcRxIfID;             /**< DMA Rx Interface ID for source */
    uint8_t bDstTxIfID;             /**< DMA Tx Interface ID for destination */
    uint8_t bDstRxIfID;             /**< DMA Rx Interface ID for destination */

    volatile uint8_t state;         /**< DMA channel state */
} sDmadChannel;

#if 0
/** DMA driver controller */
typedef struct _DmadController {
    Dmac *pHw;                  /**< Pointer to controler HW base */
    sDmadChannel* dmadChannels; /**< Pointer to driver channels list */
    uint8_t* dmadIfUsage;       /**< AHB Interface usage counting */
    uint8_t  bHwID;             /**< ID for DMAC */
    uint8_t  bNumChannels;      /**< Number of channels for this controller */
    uint8_t  bNumAHBIf;         /**< Number of AHB Interface */
    uint8_t  reserved;
} sDmadController;
#endif

/** DMA driver instance */
typedef struct _Dmad {
    Dmac *pDmacs[2];
    sDmadChannel dmaChannels[2][8];
    uint8_t  numControllers;
    uint8_t  numChannels;  
    uint8_t  pollingMode;
    uint8_t  pollingTimeout;
} sDmad;

/** DMA driver channel ID */
typedef struct _DmadChannelID {
    uint8_t bDmac;      /**< DMA Controller number */
    uint8_t bChannel;   /**< Channel number */
} sDmadChannelID;


/** DMA Transfer Descriptor as well as Linked List Item */
typedef struct _DmaTransferDescriptor {
    uint32_t dwSrcAddr;     /**< Source buffer address */
    uint32_t dwDstAddr;     /**< Destination buffer address */
    uint32_t dwCtrlA;       /**< Control A register settings */
    uint32_t dwCtrlB;       /**< Control B register settings */
    uint32_t dwDscAddr;     /**< Next descriptor address */
} sDmaTransferDescriptor;

#ifdef __cplusplus
 extern "C" {
#endif
extern void DMAD_Initialize( sDmad *pDmad,
                             uint8_t bPollingMode );

extern void DMAD_Handler( sDmad *pDmad );

extern uint32_t DMAD_AllocateChannel( sDmad *pDmad,
                                      uint8_t bSrcID, uint8_t bDstID);
extern eDmadRC DMAD_FreeChannel( sDmad *pDmad, uint32_t dwChannel );

extern eDmadRC DMAD_SetCallback( sDmad *pDmad, uint32_t dwChannel,
                                  DmadTransferCallback fCallback, void* pArg );
  
extern eDmadRC DMAD_ConfigurePIP( sDmad *pDmad, uint32_t dwChannel,
                                  uint32_t dwSrcPIP, uint32_t dwDstPIP );
  
extern eDmadRC DMAD_PrepareChannel( sDmad *pDmad, uint32_t dwChannel,
                                   uint32_t dwCfg );
   
extern eDmadRC DMAD_IsTransferDone( sDmad *pDmad, uint32_t dwChannel );

extern eDmadRC DMAD_StartTransfer( sDmad *pDmad, uint32_t dwChannel );
extern eDmadRC DMAD_StartTransfers( sDmad *pDmad,
                                    uint8_t bDmac, uint32_t bmChannels );

extern eDmadRC DMAD_StopTransfer( sDmad *pDmad, uint32_t dwChannel );
extern void DMAD_ClearAuto( sDmad *pDmad, uint32_t _dwChannel );
extern eDmadRC DMAD_PrepareSingleTransfer( sDmad *pDmad, uint32_t dwChannel,
                                           sDmaTransferDescriptor *pXfrDesc );
   
extern eDmadRC DMAD_PrepareMultiTransfer( sDmad *pDmad, uint32_t dwChannel,
                                          sDmaTransferDescriptor *pXfrDesc );
extern uint8_t DMAIF_IsValidatedPeripherOnDma( uint8_t bDmac, uint8_t bPeriphID);
extern uint8_t DMAIF_Get_ChannelNumber (uint8_t bDmac, uint8_t bPeriphID, uint8_t bTransfer);

extern sDmad gDmad;

#ifdef __cplusplus
}
#endif

#endif



