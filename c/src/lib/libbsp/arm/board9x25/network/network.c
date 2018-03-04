/*
at91sam9x25
baconxu@gmail.com
 */

#include <rtems.h>
#include <rtems/rtems_bsdnet.h>
#include <at91sam9xx5.h>

#include <stdio.h>
#include <string.h>

#include <errno.h>
#include <rtems/error.h>
#include <assert.h>

#include <sys/param.h>
#include <sys/mbuf.h>
#include <sys/socket.h>
#include <sys/sockio.h>

#include <net/if.h>

#include <netinet/in.h>
#include <netinet/if_ether.h>

#include <bsp/irq.h>
#include <bspopts.h>

#include <at91sam9xx5.h>
#include "emac.h"

/* enable debugging of the PHY code */
#define PHY_DBG

/* enable debugging of the EMAC code */
/* #define EMAC_DBG */

/* interrupt stuff */
#define EMAC_INT_PRIORITY       0       /* lowest priority */

/*  RTEMS event used by interrupt handler to start receive daemon. */
#define START_RECEIVE_EVENT  RTEMS_EVENT_1

/* RTEMS event used to start transmit daemon. */
#define START_TRANSMIT_EVENT    RTEMS_EVENT_2

static void at91sam9x25_emac_isr (void *);


/* Set up EMAC hardware */
/* Number of Receive and Transmit Buffers and Buffer Descriptors */
#define NUM_RXBDS 64
#define NUM_TXBDS 1
#define TX_BUFFER_SIZE  0x600   //1538, 1518 is maximum size of eth frame length
#define RX_BUFFER_SIZE  0x80
#define EMAC_TX_UNITSIZE TX_BUFFER_SIZE
#define EMAC_RX_UNITSIZE RX_BUFFER_SIZE

#define ETH_PHY_RETRY_MAX 		10000
/** EMAC PHY address */
#define ETH_EMAC_PHY_ADDR     	0x00


/*-----------------------------------------------------------------------------

EMAC0 PINS Definitions

-----------------------------------------------------------------------------*/
/** EMAC pin TXCK */
#define PIN_EMAC0_TXCK    {PIO_PB4A_E0_TXCK,PIOB, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT}
/** EMAC pin TX0 */
#define PIN_EMAC0_TX0     {PIO_PB9A_E0_TX0, PIOB, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT}
/** EMAC pin TX1 */
#define PIN_EMAC0_TX1     {PIO_PB10A_E0_TX1,PIOB, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT}
/** EMAC pin TX2 */
#define PIN_EMAC0_TX2     {PIO_PB11A_E0_TX2,PIOB, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT}
/** EMAC pin TX3 */
#define PIN_EMAC0_TX3     {PIO_PB12A_E0_TX3,PIOB, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT}
/** EMAC pin TXEN */
#define PIN_EMAC0_TXEN    {PIO_PB7A_E0_TXEN,PIOB, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT}
/** EMAC pin RX0 */
#define PIN_EMAC0_RXER    {PIO_PB2A_E0_RXER,PIOB, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT}
/** EMAC pin RXDV */
#define PIN_EMAC0_RXDV    {PIO_PB3A_E0_RXDV,PIOB, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT}
/** EMAC pin RX0 */
#define PIN_EMAC0_RX0     {PIO_PB0A_E0_RX0, PIOB, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT}
/** EMAC pin RX1 */
#define PIN_EMAC0_RX1     {PIO_PB1A_E0_RX1, PIOB, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT}
/** EMAC pin RX2 */
#define PIN_EMAC0_RX2     {PIO_PB13A_E0_RX2,PIOB, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT}
/** EMAC pin RX3 */
#define PIN_EMAC0_RX3     {PIO_PB14A_E0_RX3,PIOB, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT}

/** PHY pin MDC */
#define PIN_EMAC0_MDC     {PIO_PB6A_E0_MDC, PIOB, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT}
/** PHY pin MDIO */
#define PIN_EMAC0_MDIO    {PIO_PB5A_E0_MDIO,PIOB, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT}
/** PHY pin INTR */
#define PIN_EMAC0_INTR    {PIO_PB8A_E0_TXER,PIOB, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT}


/** EMAC pins definition for RMII */
#define PINS_EMAC0_RMII PIN_EMAC0_TXCK, PIN_EMAC0_TXEN, \
						PIN_EMAC0_TX0, PIN_EMAC0_TX1, \
						PIN_EMAC0_RX0, PIN_EMAC0_RX1, \
						PIN_EMAC0_RXDV, PIN_EMAC0_RXER, \
						PIN_EMAC0_MDC, PIN_EMAC0_MDIO


/*-----------------------------------------------------------------------------

EMAC1 PINS Definitions

-----------------------------------------------------------------------------*/

/** EMAC pin TXCK */
#define PIN_EMAC1_TXCK    {PIO_PC29B_E1_TXCK, PIOC, ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT}
						/** EMAC pin TX0 */
#define PIN_EMAC1_TX0     {PIO_PC18B_E1_TX0, PIOC, ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT}
						/** EMAC pin TX1 */
#define PIN_EMAC1_TX1     {PIO_PC19B_E1_TX1,PIOC, ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT}
						
						/** EMAC pin TXEN */
#define PIN_EMAC1_TXEN    {PIO_PC27B_E1_TXEN,PIOC, ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT}
						/** EMAC pin RX0 */
#define PIN_EMAC1_RXER    {PIO_PC16B_E1_RXER,PIOC, ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT}
						/** EMAC pin RXDV */
#define PIN_EMAC1_RXDV    {PIO_PC28B_E1_CRSDV,PIOC, ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT}
						/** EMAC pin RX0 */
#define PIN_EMAC1_RX0     {PIO_PC20B_E1_RX0, PIOC, ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT}
						/** EMAC pin RX1 */
#define PIN_EMAC1_RX1     {PIO_PC21B_E1_RX1, PIOC, ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT}
						
						/** PHY pin MDC */
#define PIN_EMAC1_MDC     {PIO_PC30B_E1_MDC, PIOC, ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT}
						/** PHY pin MDIO */
#define PIN_EMAC1_MDIO    {PIO_PC31B_E1_MDIO,PIOC, ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT}
						
/** EMAC pins definition for RMII */
#define PINS_EMAC1_RMII PIN_EMAC1_TXCK, PIN_EMAC1_TXEN, \
						PIN_EMAC1_TX0, PIN_EMAC1_TX1, \
						PIN_EMAC1_RX0, PIN_EMAC1_RX1, \
						PIN_EMAC1_RXDV, PIN_EMAC1_RXER, \
						PIN_EMAC1_MDC, PIN_EMAC1_MDIO 



/** The PINs for EMAC */
static const Pin gEmac0Pins[] = {PINS_EMAC0_RMII};


#define tr(...)			printk(__VA_ARGS__)


/** RX callback */
typedef void (*fEmacdTransferCallback)(uint32_t status);
/** Wakeup callback */
typedef void (*fEmacdWakeupCallback)(void);


extern int bsp_section_vector_end;
/* use the values defined in linkcmds for our use of SRAM */
#define SRAM_AREA __attribute__((section(".vector")))
#define DMA_ALIGN __attribute__((aligned(8)))

/* use internal SRAM for buffers and descriptors
 * also insure that the receive descriptors
 * start on a 64byte boundary
 * Receive Buffer Descriptor Header
 */
#define DMA_ALIGN_SIZE 64
static sEmacTxDescriptor *get_e0_txbuf_hdrs_addr()
{
	uint32_t addr = (uint32_t)&bsp_section_vector_end;
	addr = ((addr + (DMA_ALIGN_SIZE - 1)) / DMA_ALIGN_SIZE) * DMA_ALIGN_SIZE;
	return (sEmacTxDescriptor *)addr;
}

static sEmacRxDescriptor *get_e0_rxbuf_hdrs_addr()
{
	uint32_t addr = (uint32_t)get_e0_txbuf_hdrs_addr() + NUM_TXBDS * sizeof(sEmacTxDescriptor);
	addr = ((addr + (DMA_ALIGN_SIZE - 1)) / DMA_ALIGN_SIZE) * DMA_ALIGN_SIZE;
	return (sEmacRxDescriptor *)addr;
}

static uint8_t *get_e0_txbuf_addr()
{
	uint32_t addr = (uint32_t)get_e0_rxbuf_hdrs_addr() + NUM_RXBDS * sizeof(sEmacRxDescriptor);
	addr = ((addr + (DMA_ALIGN_SIZE - 1)) / DMA_ALIGN_SIZE) * DMA_ALIGN_SIZE;
	return (uint8_t *)addr;
}


static uint8_t *get_e0_rxbuf_addr()
{
	uint32_t addr = (uint32_t)get_e0_txbuf_addr() + NUM_TXBDS * TX_BUFFER_SIZE * sizeof(uint8_t);
	addr = ((addr + (DMA_ALIGN_SIZE - 1)) / DMA_ALIGN_SIZE) * DMA_ALIGN_SIZE;
	return (uint8_t *)addr;
}

static sEmacTxDescriptor *get_e1_txbuf_hdrs_addr()
{
	uint32_t addr = (uint32_t)get_e0_rxbuf_addr() + NUM_RXBDS * RX_BUFFER_SIZE * sizeof(uint8_t);
	addr = ((addr + (DMA_ALIGN_SIZE - 1)) / DMA_ALIGN_SIZE) * DMA_ALIGN_SIZE;
	return (sEmacTxDescriptor *)addr;
}

static sEmacRxDescriptor *get_e1_rxbuf_hdrs_addr()
{
	uint32_t addr = (uint32_t)get_e1_txbuf_hdrs_addr() + NUM_TXBDS * sizeof(sEmacTxDescriptor);
	addr = ((addr + (DMA_ALIGN_SIZE - 1)) / DMA_ALIGN_SIZE) * DMA_ALIGN_SIZE;
	return (sEmacRxDescriptor *)addr;
}

static uint8_t *get_e1_txbuf_addr()
{
	uint32_t addr = (uint32_t)get_e1_rxbuf_hdrs_addr() + NUM_RXBDS * sizeof(sEmacRxDescriptor);
	addr = ((addr + (DMA_ALIGN_SIZE - 1)) / DMA_ALIGN_SIZE) * DMA_ALIGN_SIZE;
	return (uint8_t *)addr;
}


static uint8_t *get_e1_rxbuf_addr()
{
	uint32_t addr = (uint32_t)get_e1_txbuf_addr() + NUM_TXBDS * TX_BUFFER_SIZE * sizeof(uint8_t);
	extern int _sram_start;
	extern int _sram_length;
	addr = ((addr + (DMA_ALIGN_SIZE - 1)) / DMA_ALIGN_SIZE) * DMA_ALIGN_SIZE;
	assert((addr + NUM_RXBDS * RX_BUFFER_SIZE * sizeof(uint8_t)) < ((uint32_t)&_sram_start + (uint32_t)&_sram_length));
	tr("0x%X, 0x%X\r\n", &_sram_start, &_sram_length);
	return (uint8_t *)addr;
}



#if 0
SRAM_AREA DMA_ALIGN volatile static sEmacRxDescriptor e1_rxbuf_hdrs[NUM_RXBDS];
SRAM_AREA DMA_ALIGN volatile static sEmacTxDescriptor e1_txbuf_hdrs[NUM_TXBDS];
SRAM_AREA DMA_ALIGN volatile static uint8_t e1_txbuf[NUM_TXBDS * TX_BUFFER_SIZE];
SRAM_AREA DMA_ALIGN volatile static uint8_t e1_rxbuf[NUM_RXBDS * RX_BUFFER_SIZE];
volatile static fEmacdTransferCallback e1TxCbs[NUM_TXBDS];
#endif

int delay_cnt;




/**
 * ETH driver struct.
 */
typedef struct _EthDriver {
    Emac *pHw;/** Pointer to HW register base */

    /** Pointer to allocated TX buffer
        Section 3.6 of AMBA 2.0 spec states that burst should not cross
        1K Boundaries.
        Receive buffer manager writes are burst of 2 words => 3 lsb bits
        of the address shall be set to 0
        */
    uint8_t *pTxBuffer;
    /** Pointer to allocated RX buffer */
    uint8_t *pRxBuffer;

    /** Pointer to Rx TDs (must be 8-byte aligned) */
    sEmacRxDescriptor *pRxD;
    /** Pointer to Tx TDs (must be 8-byte aligned) */
    sEmacTxDescriptor *pTxD;

    /** Optional callback to be invoked once a frame has been received */
    //fEmacdTransferCallback fRxCb;

	/** Optional callback to be invoked once several TD have been released */
    //fEmacdWakeupCallback fWakupCb;
    
    /** Optional callback list to be invoked once TD has been processed */
    //fEmacdTransferCallback *fTxCbList;

    /** RX TD list size */
    uint16_t wRxListSize;
    /** RX index for current processing TD */
    uint16_t wRxI;

    /** TX TD list size */
    uint16_t wTxListSize;
    /** Circular buffer head pointer by upper layer (buffer to be sent) */
    uint16_t wTxHead;
    /** Circular buffer tail pointer incremented by handlers (buffer sent) */
    uint16_t wTxTail;

    /** Number of free TD before wakeup callback is invoked */
    uint8_t  bWakeupThreshold;
    /** HW ID */
    uint8_t bId;

	/*phy parts*/
	uint32_t retryMax;  /**< The retry & timeout settings */
    uint8_t phyAddress; /**< PHY address ( pre-defined by pins on reset ) */
    uint8_t speed;      /**< 100M/10M speed */
    uint8_t fullDuplex; /**< Full duplex mode */
    uint8_t RMII;       /**< RMII/MII mode */
} TEthDrv;


/*
 * Hardware-specific storage
 */
typedef struct
{
    /*
     * Connection to networking code
     * This entry *must* be the first in the sonic_softc structure.
     */
    struct arpcom                   arpcom;

    /*
     * Interrupt vector
     */
    rtems_vector_number             vector;

    /*
     *  Indicates configuration
     */
    int                             acceptBroadcast;

    /*
     * Task waiting for interrupts
     */
    rtems_id                        rxDaemonTid;
    rtems_id                        txDaemonTid;

    /*
     * current receive header
     */
    int                				rx_buf_idx;
	/*hardware description*/
	TEthDrv							ethDrv;

    /*
     * Statistics
     */
    unsigned long                   Interrupts;
    unsigned long                   rxInterrupts;
    unsigned long                   rxMissed;
    unsigned long                   rxGiant;
    unsigned long                   rxNonOctet;
    unsigned long                   rxBadCRC;
    unsigned long                   rxCollision;

    unsigned long                   txInterrupts;
    unsigned long                   txSingleCollision;
    unsigned long                   txMultipleCollision;
    unsigned long                   txCollision;
    unsigned long                   txDeferred;
    unsigned long                   txUnderrun;
    unsigned long                   txLateCollision;
    unsigned long                   txExcessiveCollision;
    unsigned long                   txExcessiveDeferral;
    unsigned long                   txLostCarrier;
    unsigned long                   txRawWait;
} at91sam9x25_emac_softc_t;

static at91sam9x25_emac_softc_t softc;

/*----------------------------------------------------------------------------
 *         Definitions
 *----------------------------------------------------------------------------*/

/** Error bits for TX */
#define EMAC_TX_ERR_BITS  \
    (EMAC_TXD_bmERROR | EMAC_TXD_bmUNDERRUN | EMAC_TXD_bmEXHAUSTED)

/*---------------------------------------------------------------------------
 * Circular buffer management
 *---------------------------------------------------------------------------*/

/** Return count in buffer */
#define CIRC_CNT(head,tail,size) (((head) - (tail)) % (size))

/** Return space available, 0..size-1.
    We always leave one free char as a completely full buffer 
    has head == tail, which is the same as empty */
#define CIRC_SPACE(head,tail,size) CIRC_CNT((tail),((head)+1),(size))

/** Return count up to the end of the buffer.  
   Carefully avoid accessing head and tail more than once,
   so they can change underneath us without returning inconsistent results */
#define CIRC_CNT_TO_END(head,tail,size) \
   ({int end = (size) - (tail); \
     int n = ((head) + end) % (size); \
     n < end ? n : end;})

/** Return space available up to the end of the buffer */
#define CIRC_SPACE_TO_END(head,tail,size) \
   ({int end = (size) - 1 - (head); \
     int n = (end + (tail)) % (size); \
     n <= end ? n : end+1;})

/** Increment head or tail */
#define CIRC_INC(headortail,size) \
        headortail++;             \
        if(headortail >= size) {  \
            headortail = 0;       \
        }
/** Circular buffer is empty ? */
#define CIRC_EMPTY(head, tail)     (head == tail)
/** Clear circular buffer */
#define CIRC_CLEAR(head, tail)     (head = tail = 0)


/* The AT91RM9200 ethernet fifos are very undersized. Therefore
 * we use the internal SRAM to hold 4 receive packets and one
 * transmit packet.  Note that the AT91RM9200 can only queue
 * one transmit packet at a time.
 */

/* function prototypes */
int rtems_at91sam9x25_emac_attach (struct rtems_bsdnet_ifconfig *config,
                                  void *chip);
void at91sam9x25_emac_init(void *arg);
void at91sam9x25_emac_init_hw(at91sam9x25_emac_softc_t *sc);
void at91sam9x25_emac_start(struct ifnet *ifp);
void at91sam9x25_emac_stop (at91sam9x25_emac_softc_t *sc);
void at91sam9x25_emac_txDaemon (void *arg);
void at91sam9x25_emac_sendpacket (at91sam9x25_emac_softc_t *sc, struct ifnet *ifp, struct mbuf *m);
void at91sam9x25_emac_rxDaemon(void *arg);
void at91sam9x25_emac_stats (at91sam9x25_emac_softc_t *sc);
static int at91sam9x25_emac_ioctl (struct ifnet *ifp,
                                  ioctl_command_t command,
                                  caddr_t data);


static uint8_t ETH_WaitPhy( Emac *pHw, uint32_t retry );
static uint8_t ETH_ReadPhy(Emac *pHw,
                            uint8_t PhyAddress,
                            uint8_t Address,
                            uint32_t *pValue,
                            uint32_t retry);
static uint8_t ETH_WritePhy(Emac *pHw,
                             uint8_t PhyAddress,
                             uint8_t Address,
                             uint32_t  Value,
                             uint32_t  retry);

static uint8_t ETH_AutoNegotiate(TEthDrv *pEthDrv, uint8_t rmiiMode);
static uint8_t ETH_InitPhy(TEthDrv  *pEthDrv, uint32_t mck, uint8_t phyAddress);
static uint8_t ETH_GetLinkSpeed(TEthDrv *pEmacd, uint8_t applySetting);
uint8_t ETH_FindValidPhy(TEthDrv *pEthDrv, uint8_t addrStart);
void ETH_DumpPhyRegisters(TEthDrv *pEthDrv);
uint8_t ETH_ResetPhy(TEthDrv *pEthDrv);
void ETH_SetupTimeout(TEthDrv *pEthDrv, uint32_t toMax);

static void EMAC_ResetTx(TEthDrv *pDrv);
static void EMAC_ResetRx(TEthDrv *pDrv);
static uint8_t EMAC_Receive(TEthDrv * pDrv, uint8_t *pFrame, uint32_t frameSize, uint32_t *pRcvSize);



int rtems_at91sam9x25_emac_attach (
    struct rtems_bsdnet_ifconfig *config,
    void *chip  /* only one ethernet, so no chip number */
    )
{
    struct ifnet *ifp;
    int mtu;
    int unitnumber;
    char *unitname;

    /*
     * Parse driver name
     */
    if ((unitnumber = rtems_bsdnet_parse_driver_name (config, &unitname)) < 0)
        return 0;

    /*
     * Is driver free?
     */
    if (unitnumber != 0) {
        printk ("Bad at91sam9x25 EMAC unit number.\n");
        return 0;
    }
    ifp = &softc.arpcom.ac_if;
    if (ifp->if_softc != NULL) {
        printk ("Driver already in use.\n");
        return 0;
    }

    /*
     *  zero out the control structure
     */
    memset( &softc, 0, sizeof(softc) );


    /* get the MAC address from the chip */
    softc.arpcom.ac_enaddr[0] = 0x00;
    softc.arpcom.ac_enaddr[1] = 0x12;
    softc.arpcom.ac_enaddr[2] = 0x34;
    softc.arpcom.ac_enaddr[3] = 0x56;
    softc.arpcom.ac_enaddr[4] = 0x78;
    softc.arpcom.ac_enaddr[5] = 0x9a;
	#if 0
	if (chip == NULL)
	{
		softc.ethDrv.bId = ID_EMAC0;
	}
	else
	{
		if (*(uint32_t *)chip == 0)
			softc.ethDrv.bId = ID_EMAC0;
		else
			softc.ethDrv.bId = ID_EMAC1;
	}
	#endif
	softc.ethDrv.bId = ID_EMAC0;
	printk("chip id:%u sizeof(struct ether_header):%d\n\r", softc.ethDrv.bId, sizeof(struct ether_header));
    #if 0
      printk( "MAC=%02x:%02x:%02x:%02x:%02x:%02x\n",
        softc.arpcom.ac_enaddr[0],
        softc.arpcom.ac_enaddr[1],
        softc.arpcom.ac_enaddr[2],
        softc.arpcom.ac_enaddr[3],
        softc.arpcom.ac_enaddr[4],
        softc.arpcom.ac_enaddr[5]
      );
    #endif

    if (config->mtu) {
        mtu = config->mtu;
    } else {
        mtu = ETHERMTU;
    }

    softc.acceptBroadcast = !config->ignore_broadcast;

    /*
     * Set up network interface values
     */
    ifp->if_softc = &softc;
    ifp->if_unit = unitnumber;
    ifp->if_name = unitname;
    ifp->if_mtu = mtu;
    ifp->if_init = at91sam9x25_emac_init;
    ifp->if_ioctl = at91sam9x25_emac_ioctl;
    ifp->if_start = at91sam9x25_emac_start;
    ifp->if_output = ether_output;
    ifp->if_flags = IFF_BROADCAST;
    if (ifp->if_snd.ifq_maxlen == 0) {
        ifp->if_snd.ifq_maxlen = ifqmaxlen;
    }

    softc.rx_buf_idx = 0;


	get_e1_rxbuf_addr();

    /*
     * Attach the interface
     */
    if_attach (ifp);
    ether_ifattach (ifp);
    return 1;
}

void at91sam9x25_emac_init(void *arg)
{
    at91sam9x25_emac_softc_t     *sc = arg;
    struct ifnet *ifp = &sc->arpcom.ac_if;
    rtems_status_code status = RTEMS_SUCCESSFUL;
	Emac *emac;

    /*
     *This is for stuff that only gets done once (at91rm9200_emac_init()
     * gets called multiple times
     */
    if (sc->txDaemonTid == 0) {
        /* Set up EMAC hardware */
        at91sam9x25_emac_init_hw(sc);

        /*      Start driver tasks */
        sc->rxDaemonTid = rtems_bsdnet_newproc("ENrx",
                                               4096,
                                               at91sam9x25_emac_rxDaemon,
                                               sc);
        sc->txDaemonTid = rtems_bsdnet_newproc("ENtx",
                                               4096,
                                               at91sam9x25_emac_txDaemon,
                                               sc);
    } /* if txDaemonTid */

    /* set our priority in the AIC */
    AIC->AIC_SMR[sc->ethDrv.bId] = (((EMAC_INT_PRIORITY) << AIC_SMR_PRIOR_Pos) & AIC_SMR_PRIOR_Msk)
    		| (AIC_SMR_SRCTYPE_INT_EDGE_TRIGGERED & AIC_SMR_SRCTYPE_Msk);

    /* install the interrupt handler */
    status = rtems_interrupt_handler_install(
        sc->ethDrv.bId,
        "Network",
        RTEMS_INTERRUPT_UNIQUE,
        at91sam9x25_emac_isr,
        arg
    );

	emac = sc->ethDrv.pHw;
    /* Clear all status bits in the receive status register. */
    EMAC_ClearRxStatus(emac, EMAC_RSR_OVR | EMAC_RSR_REC | EMAC_RSR_BNA);

    /* Clear all status bits in the transmit status register */
    EMAC_ClearTxStatus(emac, EMAC_TSR_UBR | EMAC_TSR_COL | EMAC_TSR_RLES
                            | EMAC_TSR_BEX | EMAC_TSR_COMP | EMAC_TSR_UND);	

	/* Clear interrupts */
    EMAC_GetItStatus(emac);


	/* Enable the copy of data into the buffers
	ignore broadcasts, and don't copy FCS. */
    EMAC_Configure(emac, EMAC_GetConfigure(emac) | EMAC_NCFGR_DRFCS | EMAC_NCFGR_PAE);


	EMAC_CpyAllEnable(emac, 1);
    EMAC_BroadcastDisable(emac, 0);

	

    /* EMAC doesn't support promiscuous, so ignore requests */
    if (ifp->if_flags & IFF_PROMISC) {
        printk ("Warning - AT91SAM9x25 Ethernet driver"
                " doesn't support Promiscuous Mode!\n");
    }

    /*
     * Tell the world that we're running.
     */
    ifp->if_flags |= IFF_RUNNING;

	/*start statistics*/
	emac->EMAC_NCR |=  EMAC_NCR_WESTAT;
	/* Enable TX/RX and clear the statistics counters */
	emac->EMAC_NCR |= EMAC_NCR_TE | EMAC_NCR_RE | EMAC_NCR_CLRSTAT;


	//开启所有中断
    EMAC_EnableIt(emac,    
					  EMAC_IER_RXUBR
                    | EMAC_IER_TUND
                    | EMAC_IER_RLE
                    | EMAC_IER_TXERR
                    | EMAC_IER_TCOMP
                    | EMAC_IER_RCOMP
                    | EMAC_IER_ROVR
                    | EMAC_IER_HRESP
                    | EMAC_IER_PFR
                    | EMAC_IER_PTZ);
	printk("%s:(0x%x)\n\r", __func__, emac->EMAC_IMR);
} /* at91sam9x25_emac_init() */

void  at91sam9x25_emac_init_hw(at91sam9x25_emac_softc_t *sc)
{
    int i;

	if (sc == NULL)
		return;

	if (sc->ethDrv.bId == ID_EMAC0)
	{
	    /* Enable the clock to the EMAC */		
	    PMC_EnablePeripheral(sc->ethDrv.bId);
		tr("emac0 status: 0x%x\n\r", PMC_IsPeriphEnabled(sc->ethDrv.bId));
		/*
		EMAC_DisableIt(&sc->ethDrv,    
					  EMAC_IER_RXUBR
                    | EMAC_IER_TUND
                    | EMAC_IER_RLE
                    | EMAC_IER_TXERR
                    | EMAC_IER_TCOMP
                    | EMAC_IER_RCOMP
                    | EMAC_IER_ROVR
                    | EMAC_IER_HRESP
                    | EMAC_IER_PFR
                    | EMAC_IER_PTZ);
        */
		sc->ethDrv.pHw = EMAC0;
		sc->ethDrv.pRxD = get_e0_rxbuf_hdrs_addr();
		sc->ethDrv.pTxD = get_e0_txbuf_hdrs_addr();
		
		sc->ethDrv.pRxBuffer = get_e0_rxbuf_addr();
		sc->ethDrv.wRxListSize = NUM_RXBDS;
		
		sc->ethDrv.pTxBuffer = get_e0_txbuf_addr();
		sc->ethDrv.wTxListSize = NUM_TXBDS;

		tr("ADDR:0x%x, 0x%x, 0x%x, 0x%x\n\r", sc->ethDrv.pRxD, sc->ethDrv.pTxD, sc->ethDrv.pRxBuffer, sc->ethDrv.pTxBuffer);


		/* Configure shared pins for Ethernet, not GPIO */
		/* Configure PIO */
	  	PIO_Configure(gEmac0Pins, PIO_LISTSIZE(gEmac0Pins));
	}
	else
	{
	}
	

	EMAC_SetAddress(sc->ethDrv.pHw, 0, sc->arpcom.ac_enaddr);

    /* initialize our receive buffer descriptors */
	/* point to our receive buffer queue */
	EMAC_ReceiveEnable(sc->ethDrv.pHw, 0);
	EMAC_TransmitEnable(sc->ethDrv.pHw, 0);
	EMAC_ResetRx(&sc->ethDrv);
	EMAC_ResetTx(&sc->ethDrv);

	ETH_InitPhy(&sc->ethDrv, at91sam9xx5_get_mck(), ETH_EMAC_PHY_ADDR);

	if (!ETH_AutoNegotiate(&sc->ethDrv, 1))
    {
        tr("P: Auto Negotiate ERROR!\n\r");
    }

	if (ETH_GetLinkSpeed(&sc->ethDrv, 1) == 0)
		tr("P: Link can't be detected.\n\r");
} /* at91sam9x25_emac_init_hw() */

void at91sam9x25_emac_start(struct ifnet *ifp)
{
    at91sam9x25_emac_softc_t *sc = ifp->if_softc;

    rtems_bsdnet_event_send(sc->txDaemonTid, START_TRANSMIT_EVENT);
    ifp->if_flags |= IFF_OACTIVE;

	//EMAC_TransmitEnable(sc->ethDrv.pHw, 1);
    //EMAC_ReceiveEnable(sc->ethDrv.pHw, 1);
	tr("at91sam9x25_emac_start\r\n");
}

void at91sam9x25_emac_stop (at91sam9x25_emac_softc_t *sc)
{

    struct ifnet *ifp = &sc->arpcom.ac_if;

    ifp->if_flags &= ~IFF_RUNNING;

    /*
     * Stop the transmitter and receiver.
     */
    EMAC_TransmitEnable(sc->ethDrv.pHw, 0);
    EMAC_ReceiveEnable(sc->ethDrv.pHw, 0);
	tr("at91sam9x25_emac_stop\r\n");
}

/*
 * Driver transmit daemon
 */
void at91sam9x25_emac_txDaemon (void *arg)
{
    at91sam9x25_emac_softc_t *sc = (at91sam9x25_emac_softc_t *)arg;
    struct ifnet *ifp = &sc->arpcom.ac_if;
    struct mbuf *m;
    rtems_event_set events;

    for (;;)
    {
        /* turn on TX interrupt, then wait for one */
        //EMAC_REG(EMAC_IER) = EMAC_INT_TCOM;     /* Transmit complete */

        rtems_bsdnet_event_receive(
            START_TRANSMIT_EVENT,
            RTEMS_EVENT_ANY | RTEMS_WAIT,
            RTEMS_NO_TIMEOUT,
            &events);

        /* Send packets till queue is empty */
#if 0
        for (;;)
        {
            /* Get the next mbuf chain to transmit. */
            IF_DEQUEUE(&ifp->if_snd, m);
            if (!m)
                break;
            at91sam9x25_emac_sendpacket (sc, ifp, m);
        }
#else
		printk("%s:(0x%x)\n\r", __func__, sc->ethDrv.pHw->EMAC_IMR);
		IF_DEQUEUE(&ifp->if_snd, m);
		if (m)
			at91sam9x25_emac_sendpacket(sc, ifp, m);
#endif
        ifp->if_flags &= ~IFF_OACTIVE;
    }
}

/* Send packet */
void at91sam9x25_emac_sendpacket (at91sam9x25_emac_softc_t *sc, struct ifnet *ifp, struct mbuf *m)
{
    struct mbuf *l = NULL;
    unsigned int pkt_offset = 0;

	Emac *pHw = sc->ethDrv.pHw;
	TEthDrv *pEthDrv = &sc->ethDrv;

    volatile sEmacTxDescriptor      *pTxTd;
	
	/* Pointers to the current TxTd */
    pTxTd = &pEthDrv->pTxD[0];

    /* copy the mbuf chain into the transmit buffer */
    l = m;
    while (l != NULL) {
        memcpy(((char *)pTxTd->addr + pkt_offset),  /* offset into pkt for mbuf */
               (char *)mtod(l, void *),       /* cast to void */
               l->m_len);                     /* length of this mbuf */

        pkt_offset += l->m_len;               /* update offset */
        l = l->m_next;                        /* get next mbuf, if any */
    }

    /* free the mbuf chain we just copied */
    m_freem(m);

	{
		int i;
		tr("tx packet:\n\r");
		for (i = 0; i < pkt_offset; i++)
		{
			tr("%02X  ", ((uint8_t *)pTxTd->addr)[i]);
		}
		tr("\n\r");
	}
	/* Tx Callback */
	/* Update TD status */
	/* The buffer size defined is length of ethernet frame
	   so it's always the last buffer of the frame. */
	pTxTd->status.val = 
		(pkt_offset & EMAC_TXD_LEN_MASK) | EMAC_TXD_bmLAST | EMAC_TXD_bmWRAP;
	
	/* Now start to transmit if it is not already done */
	EMAC_TransmissionStart(pHw);
	tr("send a packet\n\r");
} /* at91sam9x25_emac_sendpacket () */


/* SONIC reader task */
void at91sam9x25_emac_rxDaemon(void *arg)
{
    at91sam9x25_emac_softc_t *sc = (at91sam9x25_emac_softc_t *)arg;
    struct ifnet *ifp = &sc->arpcom.ac_if;
    struct mbuf *m;
    struct ether_header *eh;
    rtems_event_set events;
    uint32_t pktlen;
	int rc = 0;
	
    /* Input packet handling loop */
    for (;;) {
        rtems_bsdnet_event_receive(
            START_RECEIVE_EVENT,
            RTEMS_EVENT_ANY | RTEMS_WAIT,
            RTEMS_NO_TIMEOUT,
            &events);
		
		//tr("start receive...\n\r");
		do {

            /* get an mbuf this packet */
            MGETHDR(m, M_WAIT, MT_DATA);

            /* now get a cluster pointed to by the mbuf */
            /* since an mbuf by itself is too small */
            MCLGET(m, M_WAIT);

            /* set the type of mbuf to ifp (ethernet I/F) */
            m->m_pkthdr.rcvif = ifp;
            m->m_nextpkt = 0;

			rc = EMAC_Receive(&sc->ethDrv, m->m_ext.ext_buf, EMAC_FRAME_LENTGH_MAX, &pktlen);
			//tr("emac rc:%d\n\r", rc);
			
			if (rc)
			{
				int i;
				
	            /* set the length of the mbuf */
	            m->m_len = pktlen - (sizeof(struct ether_header) + 4);
	            m->m_pkthdr.len = m->m_len;

				

	            /* strip off the ethernet header from the mbuf */
	            /* but save the pointer to it */
	            eh = mtod (m, struct ether_header *);

				tr("rx pkt:\n\r");
				for (i = 0; i < pktlen; i++)
				{
					tr("%02X  ", ((uint8_t *)m->m_data)[i]);
				}
				tr("\n\r");
				
	            //memcpy(&eh, m->m_data, sizeof(struct ether_header));
	            m->m_data += sizeof(struct ether_header);
				//memmove(m->m_data, m->m_data + sizeof(struct ether_header), m->m_len);

	            /* increment the buffer index */
	            sc->rx_buf_idx++;
	            if (sc->rx_buf_idx >= NUM_RXBDS) {
	                sc->rx_buf_idx = 0;
	            }

	            /* give all this stuff to the stack */
	            ether_input(ifp, eh, m);
			}
			else
			{
				m_freem(m);
			}

		} while (rc);
    } /* for (;;) */
} /* at91sam9x25_emac_rxDaemon() */


/* Show interface statistics */
void at91sam9x25_emac_stats (at91sam9x25_emac_softc_t *sc)
{
    printf (" Total Interrupts:%-8lu",          sc->Interrupts);
    printf ("    Rx Interrupts:%-8lu",          sc->rxInterrupts);
    printf ("            Giant:%-8lu",          sc->rxGiant);
    printf ("        Non-octet:%-8lu\n",                sc->rxNonOctet);
    printf ("          Bad CRC:%-8lu",          sc->rxBadCRC);
    printf ("        Collision:%-8lu",          sc->rxCollision);
    printf ("           Missed:%-8lu\n",                sc->rxMissed);

    printf (    "    Tx Interrupts:%-8lu",      sc->txInterrupts);
    printf (  "           Deferred:%-8lu",      sc->txDeferred);
    printf ("        Lost Carrier:%-8lu\n",     sc->txLostCarrier);
    printf (   "Single Collisions:%-8lu",       sc->txSingleCollision);
    printf ( "Multiple Collisions:%-8lu",       sc->txMultipleCollision);
    printf ("Excessive Collisions:%-8lu\n",     sc->txExcessiveCollision);
    printf (   " Total Collisions:%-8lu",       sc->txCollision);
    printf ( "     Late Collision:%-8lu",       sc->txLateCollision);
    printf ("            Underrun:%-8lu\n",     sc->txUnderrun);
    printf (   "  Raw output wait:%-8lu\n",     sc->txRawWait);
}


/*  Driver ioctl handler */
static int
at91sam9x25_emac_ioctl (struct ifnet *ifp, ioctl_command_t command, caddr_t data)
{
    at91sam9x25_emac_softc_t *sc = ifp->if_softc;
    int error = 0;

    switch (command) {
    case SIOCGIFADDR:
    case SIOCSIFADDR:
        ether_ioctl (ifp, command, data);
        break;

    case SIOCSIFFLAGS:
        switch (ifp->if_flags & (IFF_UP | IFF_RUNNING))
        {
        case IFF_RUNNING:
            at91sam9x25_emac_stop (sc);
            break;

        case IFF_UP:
            at91sam9x25_emac_init (sc);
            break;

        case IFF_UP | IFF_RUNNING:
            at91sam9x25_emac_stop (sc);
            at91sam9x25_emac_init (sc);
            break;

        default:
            break;
        } /* switch (if_flags) */
        break;

    case SIO_RTEMS_SHOW_STATS:
        at91sam9x25_emac_stats (sc);
        break;

        /*
         * FIXME: All sorts of multicast commands need to be added here!
         */
    default:
        error = EINVAL;
        break;
    } /* switch (command) */
    return error;
}

/* interrupt handler */
static void at91sam9x25_emac_isr (void * arg)
{
    uint32_t isr;
	uint32_t rsr;
	uint32_t tsr;
	at91sam9x25_emac_softc_t *sc = (at91sam9x25_emac_softc_t *)arg;
	TEthDrv *pEthDrv = &sc->ethDrv;
	uint32_t rxStatusFlag;
    uint32_t txStatusFlag;
	Emac *pHw = sc->ethDrv.pHw;
	sEmacTxDescriptor      *pTxTd;
    fEmacdTransferCallback *pTxCb = NULL;
	

    /* get the ISR status and determine RX or TX */
    isr = pHw->EMAC_ISR;
	rsr = pHw->EMAC_RSR;
	tsr = pHw->EMAC_TSR;

	//printk("|0x%x, 0x%x|", pHw->EMAC_IMR, isr);
	isr &= ~(pHw->EMAC_IMR | 0xFFC300);

	/* RX packet */
	if ((isr & EMAC_ISR_RCOMP) || (rsr & EMAC_RSR_REC))
	{
		rxStatusFlag = EMAC_RSR_REC;

		/* Check OVR */
		if (rsr & EMAC_RSR_OVR)
		{
			rxStatusFlag |= EMAC_RSR_OVR;
		}
		/* Check BNA */
		if (rsr & EMAC_RSR_BNA)
		{
			rxStatusFlag |= EMAC_RSR_BNA;
		}
		/* Clear status */
		EMAC_ClearRxStatus(pHw, rxStatusFlag);

		printk("]");
		rtems_bsdnet_event_send (sc->rxDaemonTid, START_RECEIVE_EVENT);
	}
	
	    /* TX packet */
    if ((isr & EMAC_ISR_TCOMP) || (tsr & EMAC_TSR_COMP)) {

        txStatusFlag = EMAC_TSR_COMP;

        /* A frame transmitted */

        /* Check RLE */
        if (tsr & EMAC_TSR_RLES)
        {
            /* Status RLE & Number of discarded buffers */
            txStatusFlag = EMAC_TSR_RLES
                         | CIRC_CNT(pEthDrv->wTxHead,
                                    pEthDrv->wTxTail,
                                    pEthDrv->wTxListSize)
                         ;

			EMAC_TransmitEnable(pHw, 0);
            EMAC_ResetTx(pEthDrv);
            tr("Tx RLE!!\n\r");
            EMAC_TransmitEnable(pHw, 1);
        }
        /* Check COL */
        if (tsr & EMAC_TSR_COL)
        {
            txStatusFlag |= EMAC_TSR_COL;
        }
        /* Check BEX */
        if (tsr & EMAC_TSR_BEX)
        {
            txStatusFlag |= EMAC_TSR_BEX;
        }
        /* Check UND */
        if (tsr & EMAC_TSR_UND)
        {
            txStatusFlag |= EMAC_TSR_UND;
        }
        /* Clear status */
        EMAC_ClearTxStatus(pHw, txStatusFlag);

		printk("}");
		rtems_bsdnet_event_send (softc.txDaemonTid, START_TRANSMIT_EVENT);
    }
	

	/* PAUSE Frame */
    if (isr & EMAC_ISR_PFRE)
    {
        tr("Pause!\n\r");
    }
    if (isr & EMAC_ISR_PTZ)
    {
        tr("Pause TO!\n\r");
    }
	//AIC->AIC_ICCR = 1 << ID_EMAC0;
}




/*-----------------------------------------------------------------------------
local function
-----------------------------------------------------------------------------*/

#define BOARD_EMAC_PHY_COMP_LAN8700

/*---------------------------------------------------------------------------
 *         Definitions
 *---------------------------------------------------------------------------*/

/** \addtogroup mii_registers PHY registers Addresses
    @{*/
#define MII_BMCR        0   /**< Basic Mode Control Register */
#define MII_BMSR        1   /**< Basic Mode Status Register */
#define MII_PHYID1      2   /**< PHY Idendifier Register 1 */
#define MII_PHYID2      3   /**< PHY Idendifier Register 2 */
#define MII_ANAR        4   /**< Auto_Negotiation Advertisement Register */
#define MII_ANLPAR      5   /**< Auto_negotiation Link Partner Ability Register */
#define MII_ANER        6   /**< Auto-negotiation Expansion Register */
#define MII_DSCR       16   /**< Specified Configuration Register */
#define MII_DSCSR      17   /**< Specified Configuration and Status Register */
#define MII_10BTCSR    18   /**< 10BASE-T Configuration and Satus Register */
#define MII_PWDOR      19   /**< Power Down Control Register */
#define MII_CONFIGR    20   /**< Specified config Register */
#define MII_MDINTR     21   /**< Specified Interrupt Register */
#define MII_RECR       22   /**< Specified Receive Error Counter Register */
#define MII_DISCR      23   /**< Specified Disconnect Counter Register */
#define MII_RLSR       24   /**< Hardware Reset Latch State Register */
/** @}*/

/** \addtogroup phy_bmcr Basic Mode Control Register (BMCR, 0)
    List Bit definitions: \ref MII_BMCR
    @{*/
#define MII_RESET             (1ul << 15) /**< 1= Software Reset; 0=Normal Operation */
#define MII_LOOPBACK          (1ul << 14) /**< 1=loopback Enabled; 0=Normal Operation */
#define MII_SPEED_SELECT      (1ul << 13) /**< 1=100Mbps; 0=10Mbps */
#define MII_AUTONEG           (1ul << 12) /**< Auto-negotiation Enable */
#define MII_POWER_DOWN        (1ul << 11) /**< 1=Power down 0=Normal operation */
#define MII_ISOLATE           (1ul << 10) /**< 1 = Isolates 0 = Normal operation */
#define MII_RESTART_AUTONEG   (1ul << 9)  /**< 1 = Restart auto-negotiation 0 = Normal operation */
#define MII_DUPLEX_MODE       (1ul << 8)  /**< 1 = Full duplex operation 0 = Normal operation */
#define MII_COLLISION_TEST    (1ul << 7)  /**< 1 = Collision test enabled 0 = Normal operation */
/** Reserved bits: 6 to 0, Read as 0, ignore on write */
/** @}*/

/** \addtogroup phy_bmsr Basic Mode Status Register (BMSR, 1)
    List Bit definitions: \ref MII_BMSR
    @{*/
#define MII_100BASE_T4        (1ul << 15) /**< 100BASE-T4 Capable */
#define MII_100BASE_TX_FD     (1ul << 14) /**< 100BASE-TX Full Duplex Capable */
#define MII_100BASE_T4_HD     (1ul << 13) /**< 100BASE-TX Half Duplex Capable */
#define MII_10BASE_T_FD       (1ul << 12) /**< 10BASE-T Full Duplex Capable */
#define MII_10BASE_T_HD       (1ul << 11) /**< 10BASE-T Half Duplex Capable */
/** Reserved bits: 10 to 7, Read as 0, ignore on write */
#define MII_MF_PREAMB_SUPPR   (1ul << 6)  /**< MII Frame Preamble Suppression */
#define MII_AUTONEG_COMP      (1ul << 5)  /**< Auto-negotiation Complete */
#define MII_REMOTE_FAULT      (1ul << 4)  /**< Remote Fault */
#define MII_AUTONEG_ABILITY   (1ul << 3)  /**< Auto Configuration Ability */
#define MII_LINK_STATUS       (1ul << 2)  /**< Link Status */
#define MII_JABBER_DETECT     (1ul << 1)  /**< Jabber Detect */
#define MII_EXTEND_CAPAB      (1ul << 0)  /**< Extended Capability */
/** @}*/

/** \addtogroup phy_id PHY ID Identifier Register (PHYID, 2,3)
    List definitions: \ref MII_PHYID1, \ref MII_PHYID2
    @{*/
#define MII_LSB_MASK             0x3F   /**< Mask for PHY ID LSB */

#if defined(BOARD_EMAC_PHY_COMP_DM9161)
#define MII_OUI_MSB            0x0181
#define MII_OUI_LSB              0x2E
//#define MII_PHYID1_OUI         0x606E   // OUI: Organizationally Unique Identifier
//#define MII_ID             0x0181b8a0
#elif defined(BOARD_EMAC_PHY_COMP_LAN8700)
#define MII_OUI_MSB            0x0007
#define MII_OUI_LSB              0x30
#else
#error no PHY Ethernet component defined !
#endif
/** @}*/

/** \addtogroup phy_neg Auto-negotiation (ANAR, 4; ANLPAR, 5)
    - Auto-negotiation Advertisement Register (ANAR)
    - Auto-negotiation Link Partner Ability Register (ANLPAR)
    Lists Bit definitions: \ref MII_ANAR, \ref MII_ANLPAR
    @{*/
#define MII_NP               (1ul << 15) /**< Next page Indication */
#define MII_ACK              (1ul << 14) /**< Acknowledge */
#define MII_RF               (1ul << 13) /**< Remote Fault */
/** Reserved: 12 to 11, Write as 0, ignore on read */
#define MII_FCS              (1ul << 10) /**< Flow Control Support */
#define MII_T4               (1ul << 9)  /**< 100BASE-T4 Support */
#define MII_TX_FDX           (1ul << 8)  /**< 100BASE-TX Full Duplex Support */
#define MII_TX_HDX           (1ul << 7)  /**< 100BASE-TX Support */
#define MII_10_FDX           (1ul << 6)  /**< 10BASE-T Full Duplex Support */
#define MII_10_HDX           (1ul << 5)  /**< 10BASE-T Support */
/** Selector: 4 to 0, Protocol Selection Bits */
#define MII_AN_IEEE_802_3      0x0001
/** @}*/

/** \addtogroup phy_neg_exp Auto-negotiation Expansion Register (ANER, 6)
    List Bit definitions: \ref MII_ANER
    @{*/
/** Reserved: 15 to 5, Read as 0, ignore on write */
#define MII_PDF              (1ul << 4) /**< Local Device Parallel Detection Fault */
#define MII_LP_NP_ABLE       (1ul << 3) /**< Link Partner Next Page Able */
#define MII_NP_ABLE          (1ul << 2) /**< Local Device Next Page Able */
#define MII_PAGE_RX          (1ul << 1) /**< New Page Received */
#define MII_LP_AN_ABLE       (1ul << 0) /**< Link Partner Auto-negotiation Able */
/** @}*/

/** \addtogroup phy_dscr Specified Configuration Register (DSCR, 16)
    List Bit definitions: \ref MII_DSCR
    @{*/
#define MII_BP4B5B           (1ul << 15) /**< Bypass 4B5B Encoding and 5B4B Decoding */
#define MII_BP_SCR           (1ul << 14) /**< Bypass Scrambler/Descrambler Function */
#define MII_BP_ALIGN         (1ul << 13) /**< Bypass Symbol Alignment Function */
#define MII_BP_ADPOK         (1ul << 12) /**< BYPASS ADPOK */
#define MII_REPEATER         (1ul << 11) /**< Repeater/Node Mode */
#define MII_TX               (1ul << 10) /**< 100BASE-TX Mode Control */
#define MII_FEF              (1ul << 9)  /**< Far end Fault enable */
#define MII_RMII_ENABLE      (1ul << 8)  /**< Reduced MII Enable */
#define MII_F_LINK_100       (1ul << 7)  /**< Force Good Link in 100Mbps */
#define MII_SPLED_CTL        (1ul << 6)  /**< Speed LED Disable */
#define MII_COLLED_CTL       (1ul << 5)  /**< Collision LED Enable */
#define MII_RPDCTR_EN        (1ul << 4)  /**< Reduced Power Down Control Enable */
#define MII_SM_RST           (1ul << 3)  /**< Reset State Machine */
#define MII_MFP_SC           (1ul << 2)  /**< MF Preamble Suppression Control */
#define MII_SLEEP            (1ul << 1)  /**< Sleep Mode */
#define MII_RLOUT            (1ul << 0)  /**< Remote Loopout Control */
/** @}*/

/** \addtogroup phy_dscsr Specified Configuration and Status Register (DSCSR, 17)
    List Bit definitions: \ref MII_DSCSR
    @{*/
#define MII_100FDX           (1ul << 15) /**< 100M Full Duplex Operation Mode */
#define MII_100HDX           (1ul << 14) /**< 100M Half Duplex Operation Mode */
#define MII_10FDX            (1ul << 13) /**< 10M Full Duplex Operation Mode */
#define MII_10HDX            (1ul << 12) /**< 10M Half Duplex Operation Mode */
/** @}*/

/** \addtogroup phy_10btcsr 10BASE-T Configuration/Status (10BTCSR, 18)
    List Bit definitions: \ref MII_10BTCSR
    @{*/
/** Reserved: 18 to 15, Read as 0, ignore on write */
#define MII_LP_EN            (1ul << 14) /**< Link Pulse Enable */
#define MII_HBE              (1ul << 13) /**< Heartbeat Enable */
#define MII_SQUELCH          (1ul << 12) /**< Squelch Enable */
#define MII_JABEN            (1ul << 11) /**< Jabber Enable */
#define MII_10BT_SER         (1ul << 10) /**< 10BASE-T GPSI Mode */
/** Reserved: 9 to  1, Read as 0, ignore on write */
#define MII_POLR             (1ul << 0)  /**< Polarity Reversed */
/** @}*/

/** \addtogroup phy_mdintr Specified Interrupt Register (MDINTR, 21)
    List Bit definitions: \ref MII_MDINTR
    @{*/
#define MII_INTR_PEND        (1ul << 15) /**< Interrupt Pending */
/** Reserved: 14 to 12, Reserved */
#define MII_FDX_MASK         (1ul << 11) /**< Full-duplex Interrupt Mask */
#define MII_SPD_MASK         (1ul << 10) /**< Speed Interrupt Mask */
#define MII_LINK_MASK        (1ul << 9)  /**< Link Interrupt Mask */
#define MII_INTR_MASK        (1ul << 8)  /**< Master Interrupt Mask */
/** Reserved: 7 to 5, Reserved */
#define MII_FDX_CHANGE       (1ul << 4)  /**< Duplex Status Change Interrupt */
#define MII_SPD_CHANGE       (1ul << 3)  /**< Speed Status Change Interrupt */
#define MII_LINK_CHANGE      (1ul << 2)  /**< Link Status Change Interrupt */
/** Reserved: 1, Reserved */
#define MII_INTR_STATUS      (1ul << 0)  /**< Interrupt Status */




/**
 * Wait PHY operation complete.
 * Return 1 if the operation completed successfully.
 * May be need to re-implemented to reduce CPU load.
 * \param retry: the retry times, 0 to wait forever until complete.
 */
static uint8_t ETH_WaitPhy( Emac *pHw, uint32_t retry )
{
    volatile uint32_t retry_count = 0;

    while (!EMAC_IsIdle(pHw))
    {
        if(retry == 0) continue;
        retry_count ++;
        if (retry_count >= retry)
        {
            return 0;
        }
    }
    return 1;
}

/**
 * Read PHY register.
 * Return 1 if successfully, 0 if timeout.
 * \param pHw HW controller address
 * \param PhyAddress PHY Address
 * \param Address Register Address
 * \param pValue Pointer to a 32 bit location to store read data
 * \param retry The retry times, 0 to wait forever until complete.
 */
static uint8_t ETH_ReadPhy(Emac *pHw,
                            uint8_t PhyAddress,
                            uint8_t Address,
                            uint32_t *pValue,
                            uint32_t retry)
{
    EMAC_PHYMaintain(pHw, PhyAddress, Address, 1, 0);
    if ( ETH_WaitPhy(pHw, retry) == 0 )
    {
        tr("TimeOut EMAC_ReadPhy\n\r");
        return 0;
    }
    *pValue = EMAC_PHYData(pHw);
    return 1;
}


/**
 * Write PHY register
 * Return 1 if successfully, 0 if timeout.
 * \param pHw HW controller address
 * \param PhyAddress PHY Address
 * \param Address Register Address
 * \param Value Data to write ( Actually 16 bit data )
 * \param retry The retry times, 0 to wait forever until complete.
 */
static uint8_t ETH_WritePhy(Emac *pHw,
                             uint8_t PhyAddress,
                             uint8_t Address,
                             uint32_t  Value,
                             uint32_t  retry)
{
    EMAC_PHYMaintain(pHw, PhyAddress, Address, 0, Value);
    if ( ETH_WaitPhy(pHw, retry) == 0 )
    {
        tr("TimeOut EMAC_WritePhy\n\r");
        return 0;
    }
    return 1;
}

/**
 * Find a valid PHY Address ( from addrStart to 31 ).
 * Check BMSR register ( not 0 nor 0xFFFF )
 * Return 0xFF when no valid PHY Address found.
 * \param pMacb Pointer to the MACB instance
 */
uint8_t ETH_FindValidPhy(TEthDrv *pEthDrv, uint8_t addrStart)
{
    TEthDrv *pDrv = pEthDrv;
    Emac *pHw = pDrv->pHw;

    uint32_t  retryMax;
    uint32_t  value=0;
    uint8_t rc;
    uint8_t phyAddress;
    uint8_t cnt;

    tr("ETH_FindValidPhy\n\r");

    EMAC_ManagementEnable(pHw, 1);

    phyAddress = pDrv->phyAddress;
    retryMax = pDrv->retryMax;

    /* Check current phyAddress */
    rc = phyAddress;
    if( ETH_ReadPhy(pHw, phyAddress, MII_PHYID1, &value, retryMax) == 0 )
    {
        tr("MACB PROBLEM\n\r");
    }
    tr("_PHYID1  : 0x%X, addr: %d\n\r", value, phyAddress);

    /* Find another one */
    if (value != MII_OUI_MSB)
    {
        rc = 0xFF;
        for(cnt = addrStart; cnt < 32; cnt ++)
        {
            phyAddress = (phyAddress + 1) & 0x1F;
            if( ETH_ReadPhy(pHw, phyAddress, MII_PHYID1, &value, retryMax) == 0 )
            {
                tr("MACB PROBLEM\n\r");
            }
            tr("_PHYID1  : 0x%X, addr: %d\n\r", value, phyAddress);
            if (value == MII_OUI_MSB)
            {
                rc = phyAddress;
                break;
            }
        }
    }

    EMAC_ManagementEnable(pHw, 0);

    if (rc != 0xFF)
    {

        tr("** Valid PHY Found: %d\n\r", rc);
        ETH_ReadPhy(pHw, phyAddress, MII_DSCSR, &value, retryMax);
        tr("_DSCSR  : 0x%X, addr: %d\n\r", value, phyAddress);

    }
    return rc;
}

/**
 * Dump all the useful registers
 * \param pMacb          Pointer to the MACB instance
 */
void ETH_DumpPhyRegisters(TEthDrv *pEthDrv)
{
    TEthDrv *pDrv = pEthDrv;
    Emac *pHw = pDrv->pHw;

    uint8_t phyAddress;
    uint32_t retryMax;
    uint32_t value;

    tr("ETH_DumpPhyRegisters\n\r");

    EMAC_ManagementEnable(pHw, 1);

    phyAddress = pDrv->phyAddress;
    retryMax = pDrv->retryMax;

    tr("%cMII MACB (@%d) Registers:\n\r",
        pDrv->RMII ? 'R' : ' ',
        phyAddress);

    ETH_ReadPhy(pHw, phyAddress, MII_BMCR, &value, retryMax);
    tr(" _BMCR   : 0x%X\n\r", value);
    ETH_ReadPhy(pHw, phyAddress, MII_BMSR, &value, retryMax);
    tr(" _BMSR   : 0x%X\n\r", value);
    ETH_ReadPhy(pHw, phyAddress, MII_ANAR, &value, retryMax);
    tr(" _ANAR   : 0x%X\n\r", value);
    ETH_ReadPhy(pHw, phyAddress, MII_ANLPAR, &value, retryMax);
    tr(" _ANLPAR : 0x%X\n\r", value);
    ETH_ReadPhy(pHw, phyAddress, MII_ANER, &value, retryMax);
    tr(" _ANER   : 0x%X\n\r", value);
    ETH_ReadPhy(pHw, phyAddress, MII_DSCR, &value, retryMax);
    tr(" _DSCR   : 0x%X\n\r", value);
    ETH_ReadPhy(pHw, phyAddress, MII_DSCSR, &value, retryMax);
    tr(" _DSCSR  : 0x%X\n\r", value);
    ETH_ReadPhy(pHw, phyAddress, MII_10BTCSR, &value, retryMax);
    tr(" _10BTCSR: 0x%X\n\r", value);
    ETH_ReadPhy(pHw, phyAddress, MII_PWDOR, &value, retryMax);
    tr(" _PWDOR  : 0x%X\n\r", value);
    ETH_ReadPhy(pHw, phyAddress, MII_CONFIGR, &value, retryMax);
    tr(" _CONFIGR: 0x%X\n\r", value);
    ETH_ReadPhy(pHw, phyAddress, MII_MDINTR, &value, retryMax);
    tr(" _MDINTR : 0x%X\n\r", value);
    ETH_ReadPhy(pHw, phyAddress, MII_RECR, &value, retryMax);
    tr(" _RECR   : 0x%X\n\r", value);
    ETH_ReadPhy(pHw, phyAddress, MII_DISCR, &value, retryMax);
    tr(" _DISCR  : 0x%X\n\r", value);
    ETH_ReadPhy(pHw, phyAddress, MII_RLSR, &value, retryMax);
    tr(" _RLSR   : 0x%X\n\r", value);

    EMAC_ManagementEnable(pHw, 0);
}

/**
 * Setup the maximum timeout count of the driver.
 * \param pMacb   Pointer to the MACB instance
 * \param toMax   Timeout maxmum count.
 */
void ETH_SetupTimeout(TEthDrv *pEthDrv, uint32_t toMax)
{
    pEthDrv->retryMax = toMax;
}

/**
 * Issue a SW reset to reset all registers of the PHY
 * Return 1 if successfully, 0 if timeout.
 * \param pMacb   Pointer to the MACB instance
 */
uint8_t ETH_ResetPhy(TEthDrv *pEthDrv)
{
    TEthDrv *pDrv = pEthDrv;
    Emac *pHw = pDrv->pHw;

    uint32_t retryMax;
    uint32_t bmcr = MII_RESET;
    uint8_t phyAddress;
    uint32_t timeout = 10;
    uint8_t ret = 1;

    tr(" MACB_ResetPhy\n\r");

    phyAddress = pDrv->phyAddress;
    retryMax = pDrv->retryMax;

    EMAC_ManagementEnable(pHw, 1);

    bmcr = MII_RESET;
    ETH_WritePhy(pHw, phyAddress, MII_BMCR, bmcr, retryMax);

    do
    {
        ETH_ReadPhy(pHw, phyAddress, MII_BMCR, &bmcr, retryMax);
        timeout--;
    } while ((bmcr & MII_RESET) && timeout);

    EMAC_ManagementEnable(pHw, 0);

    if (!timeout)
    {
        ret = 0;
    }

    return( ret );
}

#define PHY1_RST_PIN      {PIO_PB18, PIOB, ID_PIOB, PIO_OUTPUT_1, PIO_PULLUP}
#define PHY2_RST_PIN      {PIO_PC25, PIOC, ID_PIOC, PIO_OUTPUT_1, PIO_PULLUP}

static const Pin gResetPin[] = {PHY1_RST_PIN, PHY2_RST_PIN};
uint8_t ETH_InitPhy(TEthDrv  *pEthDrv, uint32_t mck, uint8_t phyAddress)
{
    TEthDrv *pDrv = pEthDrv;
    Emac *pHw = pDrv->pHw;

    uint8_t rc = 1;
    uint8_t phy;

    pEthDrv->phyAddress = phyAddress;
    /* Initialize timeout by default */
    pEthDrv->retryMax = ETH_PHY_RETRY_MAX;
	
	PIO_Configure(gResetPin, sizeof(gResetPin) / sizeof(Pin));
	
    /* Configure EMAC runtime pins */
    //PIO_Configure(pEmacPins, nbEmacPins);
    rc = EMAC_SetClock( pHw, mck );
    if (!rc)
    {
        tr("No Valid MDC clock\n\r");
        return 0;
    }

    /* Check PHY Address */
    phy = ETH_FindValidPhy(pEthDrv, 0);
    if (phy == 0xFF)
    {
        tr("PHY Access fail\n\r");
        return 0;
    }

	if(phy != pDrv->phyAddress)
    {
        pDrv->phyAddress = phy;
        ETH_ResetPhy(pDrv);
    }

    return rc;
}

/**
 * Issue a Auto Negotiation of the PHY
 * Return 1 if successfully, 0 if timeout.
 * \param pEmacd   Pointer to the TEthDrv instance
 */
uint8_t ETH_AutoNegotiate(TEthDrv *pEthDrv, uint8_t rmiiMode)
{
    TEthDrv *pDrv = pEthDrv;
    Emac *pHw = pDrv->pHw;

    uint32_t retryMax;
    uint32_t value;
    uint32_t phyAnar;
    uint32_t phyAnalpar;
    uint32_t retryCount= 0;
    uint8_t phyAddress;
    uint8_t bFD = 0;
    uint8_t bSP = 0;
    uint8_t rc = 1;

    pDrv->RMII = rmiiMode;

    phyAddress = pDrv->phyAddress;
    retryMax = pDrv->retryMax * 100;

    EMAC_ManagementEnable(pHw, 1);

    if (!ETH_ReadPhy(pHw, phyAddress, MII_PHYID1, &value, retryMax))
    {
        tr("Pb EMAC_ReadPhy Id1\n\r");
        rc = 0;
        goto AutoNegotiateExit;
    }
    tr("ReadPhy Id1 0x%X, addresse: %d\n\r", value, phyAddress);
    if (!ETH_ReadPhy(pHw, phyAddress, MII_PHYID2, &phyAnar, retryMax))
    {
        tr("Pb EMAC_ReadPhy Id2\n\r");
        rc = 0;
        goto AutoNegotiateExit;
    }
    tr("ReadPhy Id2 0x%X\n\r", phyAnar);

    if( ( value == MII_OUI_MSB )
     && ( ((phyAnar>>10)&MII_LSB_MASK) == MII_OUI_LSB ) )
    {
        tr("Vendor Number Model = 0x%X\n\r", ((phyAnar>>4)&0x3F));
        tr("Model Revision Number = 0x%X\n\r", (phyAnar&0x7));
    }
    else
    {
        tr("Problem OUI value\n\r");
    }        

    /* Setup control register */
    rc  = ETH_ReadPhy(pHw, phyAddress, MII_BMCR, &value, retryMax);
    if (rc == 0)
    {
        goto AutoNegotiateExit;
    }

    value &= ~MII_AUTONEG;   /* Remove autonegotiation enable */
    value &= ~(MII_LOOPBACK|MII_POWER_DOWN);
    value |=  MII_ISOLATE;   /* Electrically isolate PHY */
    rc = ETH_WritePhy(pHw, phyAddress, MII_BMCR, value, retryMax);
    if (rc == 0)
    {
        goto AutoNegotiateExit;
    }

    /* Set the Auto_negotiation Advertisement Register
       MII advertising for Next page
       100BaseTxFD and HD, 10BaseTFD and HD, IEEE 802.3 */
    phyAnar = MII_TX_FDX | MII_TX_HDX |
              MII_10_FDX | MII_10_HDX | MII_AN_IEEE_802_3;
    rc = ETH_WritePhy(pHw, phyAddress, MII_ANAR, phyAnar, retryMax);
    if (rc == 0)
    {
        goto AutoNegotiateExit;
    }

    /* Read & modify control register */
    rc  = ETH_ReadPhy(pHw, phyAddress, MII_BMCR, &value, retryMax);
    if (rc == 0)
    {
        goto AutoNegotiateExit;
    }

    value |= MII_SPEED_SELECT | MII_AUTONEG | MII_DUPLEX_MODE;
    rc = ETH_WritePhy(pHw, phyAddress, MII_BMCR, value, retryMax);
    if (rc == 0)
    {
        goto AutoNegotiateExit;
    }

    /* Restart Auto_negotiation */
    value |=  MII_RESTART_AUTONEG;
    value &= ~MII_ISOLATE;
    rc = ETH_WritePhy(pHw, phyAddress, MII_BMCR, value, retryMax);
    if (rc == 0)
    {
        goto AutoNegotiateExit;
    }
    tr(" _BMCR: 0x%X\n\r", value);

    /* Check AutoNegotiate complete */
    while (1)
    {
        rc  = ETH_ReadPhy(pHw, phyAddress, MII_BMSR, &value, retryMax);
        if (rc == 0)
        {
            tr("_BMSR Rd err\n\r");
            goto AutoNegotiateExit;
        }
        /* Done successfully */
        if (value & MII_AUTONEG_COMP)
        {
            tr("AutoNegotiate complete\n\r");
            break;
        }

        /* Timeout check */
        if (retryMax)
        {
            if (++ retryCount >= retryMax)
            {
                ETH_DumpPhyRegisters(pEthDrv);
                tr("TimeOut: %u\n\r", retryCount);
                rc = 0;
                goto AutoNegotiateExit;
            }
        }
    }

    /* Get the AutoNeg Link partner base page */
    rc  = ETH_ReadPhy(pHw, phyAddress, MII_ANLPAR, &phyAnalpar, retryMax);
    if (rc == 0)
    {
        goto AutoNegotiateExit;
    }

    /* Setup the EMAC link speed */
    if ((phyAnar & phyAnalpar) & MII_TX_FDX)
    {
        /* set MII for 100BaseTX and Full Duplex */
        bSP = 1; bFD = 1;
    }
    else if ((phyAnar & phyAnalpar) & MII_10_FDX)
    {
        /* set MII for 10BaseT and Full Duplex */
        bSP = 0; bFD = 1;
    }
    else if ((phyAnar & phyAnalpar) & MII_TX_HDX)
    {
        // set MII for 100BaseTX and half Duplex
        bSP = 1; bFD = 0;
    }
    else if ((phyAnar & phyAnalpar) & MII_10_HDX)
    {
        // set MII for 10BaseT and half Duplex
        bSP = 0; bFD = 0;
    }
    EMAC_SetSpeed(pHw, bSP);
    EMAC_FullDuplexEnable(pHw, bFD);

    EMAC_RMIIEnable(pHw, rmiiMode);
    EMAC_TransceiverClockEnable(pHw, 1);

AutoNegotiateExit:
    EMAC_ManagementEnable(pHw, 0);
    return rc;
}

/**
 * Get the Link & speed settings, and automatically setup the EMAC with the
 * settings.
 * Return 1 if link found, 0 if no ethernet link.
 * \param pMacb          Pointer to the MACB instance
 * \param applySetting Apply the settings to EMAC interface
 */
uint8_t ETH_GetLinkSpeed(TEthDrv *pEmacd, uint8_t applySetting)
{
    TEthDrv *pDrv = pEmacd;
    Emac *pHw = pDrv->pHw;

    uint32_t retryMax;
    uint32_t stat1;
    uint32_t stat2;
    uint8_t phyAddress, bSP, bFD;
    uint8_t rc = 1;
       
    tr("MACB_GetLinkSpeed\n\r");
    
    EMAC_ManagementEnable(pHw, 1);

    phyAddress = pDrv->phyAddress;
    retryMax = pDrv->retryMax;

    rc  = ETH_ReadPhy(pHw, phyAddress, MII_BMSR, &stat1, retryMax);
    if (rc == 0)
    {
        goto GetLinkSpeedExit;
    }

    if ((stat1 & MII_LINK_STATUS) == 0)
    {
        tr("Pb: LinkStat: 0x%x\n\r", stat1);

        rc = 0;
        goto GetLinkSpeedExit;
    }

    if (applySetting == 0)
    {
        tr("Speed #%d not applied\n\r", applySetting);
        goto GetLinkSpeedExit;
    }

    /* Re-configure Link speed */
    #ifdef BOARD_EMAC_PHY_COMP_LAN8700
    #undef MII_DSCSR
    #define MII_DSCSR  31
    #endif
    rc  = ETH_ReadPhy(pHw, phyAddress, MII_DSCSR, &stat2, retryMax);
    if (rc == 0)
    {
        tr("Pb _DSCSR: rc 0x%x\n\r", rc);
        goto GetLinkSpeedExit;
    }

    if (((stat2 & 0x1C) >> 2) == 6)
    {
        /* set Emac for 100BaseTX and Full Duplex */
        bSP = 1; bFD = 1;
    }

    if (((stat2 & 0x1C) >> 2) == 5)
    {
        /* set MII for 10BaseT and Full Duplex */
        bSP= 0; bFD = 1;
    }

    if (((stat2 & 0x1C) >> 2) == 2)
    {
        /* set MII for 100BaseTX and Half Duplex */
        bSP = 1; bFD = 0;
    }

    if (((stat2 & 0x1C) >> 2) == 1)
    {
        /* set MII for 10BaseT and Half Duplex */
        bSP = 0; bFD = 0;
    }
    EMAC_SetSpeed(pHw, bSP);
    EMAC_FullDuplexEnable(pHw, bFD);

    /* Start the EMAC transfers */
    tr("MACB_GetLinkSpeed passed\n\r");

GetLinkSpeedExit:
    EMAC_ManagementEnable(pHw, 0);
    return rc;
}


/**
 * Disable TX & reset registers and descriptor list
 * \param pDrv Pointer to EMAC Driver instance.
 */
static void EMAC_ResetTx(TEthDrv *pDrv)
{
    Emac    *pHw = pDrv->pHw;
    uint8_t *pTxBuffer = pDrv->pTxBuffer;
    sEmacTxDescriptor *pTd = pDrv->pTxD;

    uint32_t Index;
    uint32_t Address;
	uint32_t i;
#if 0
    /* Disable TX */
    //EMAC_TransmitEnable(pHw, 0);

    /* Setup the TX descriptors. */
    CIRC_CLEAR(pDrv->wTxHead, pDrv->wTxTail);
    for(Index = 0; Index < pDrv->wTxListSize; Index++)
    {
        Address = (uint32_t)(&(pTxBuffer[Index * EMAC_TX_UNITSIZE]));
        pTd[Index].addr = Address;
        pTd[Index].status.val = EMAC_TXD_bmUSED;
    }
    pTd[pDrv->wTxListSize - 1].status.val = EMAC_TXD_bmUSED | EMAC_TXD_bmWRAP;
#else
	/* initialize our transmitter buffer descriptors */
	for (i = 0; i < NUM_TXBDS; i++)
	{
		uint32_t address = (uint32_t)(&(pTxBuffer[i * TX_BUFFER_SIZE]));
		pTd[i].addr = address;
		pTd[i].status.val = EMAC_TXD_bmUSED;
	}
	pTd[NUM_TXBDS - 1].status.val = EMAC_TXD_bmUSED | EMAC_TXD_bmWRAP;
#endif
    /* Transmit Buffer Queue Pointer Register */
    EMAC_SetTxQueue(pHw, (uint32_t)pTd);
}

/**
 * Disable RX & reset registers and descriptor list
 * \param pDrv Pointer to EMAC Driver instance.
 */
static void EMAC_ResetRx(TEthDrv *pDrv)
{
    Emac    *pHw = pDrv->pHw;
    uint8_t *pRxBuffer = pDrv->pRxBuffer;
    sEmacRxDescriptor *pRd = pDrv->pRxD;

    uint32_t Index;
    uint32_t Address;

    /* Disable RX */
    //EMAC_ReceiveEnable(pHw, 0);

    /* Setup the RX descriptors. */
    pDrv->wRxI = 0;
    for(Index = 0; Index < pDrv->wRxListSize; Index++)
    {
        Address = (uint32_t)(&(pRxBuffer[Index * EMAC_RX_UNITSIZE]));
        /* Remove EMAC_RXD_bmOWNERSHIP and EMAC_RXD_bmWRAP */
        pRd[Index].addr.val = Address & EMAC_RXD_ADDR_MASK;
        pRd[Index].status.val = 0;
    }
    pRd[pDrv->wRxListSize - 1].addr.val |= EMAC_RXD_bmWRAP;

    /* Receive Buffer Queue Pointer Register */
    EMAC_SetRxQueue(pHw, (uint32_t) pRd);
}

uint8_t EMAC_Receive(TEthDrv * pDrv, uint8_t *pFrame, uint32_t frameSize, uint32_t *pRcvSize)
{
    uint16_t bufferLength;
    uint32_t tmpFrameSize=0;
    uint8_t  *pTmpFrame=0;
    uint32_t tmpIdx = pDrv->wRxI;
    volatile sEmacRxDescriptor *pRxTd = &pDrv->pRxD[pDrv->wRxI];
    char isFrame = 0;

    if (pFrame == NULL) return 0;

    /* Set the default return value */
    *pRcvSize = 0;

    /* Process received RxTd */
    while ((pRxTd->addr.val & EMAC_RXD_bmOWNERSHIP) == EMAC_RXD_bmOWNERSHIP)
    {
        /* A start of frame has been received, discard previous fragments */
        if ((pRxTd->status.val & EMAC_RXD_bmSOF) == EMAC_RXD_bmSOF)
        {
            /* Skip previous fragment */
            while (tmpIdx != pDrv->wRxI)
            {
                pRxTd = &pDrv->pRxD[pDrv->wRxI];
                pRxTd->addr.val &= ~(EMAC_RXD_bmOWNERSHIP);
                CIRC_INC(pDrv->wRxI, pDrv->wRxListSize);
            }
            /* Reset the temporary frame pointer */
            pTmpFrame = pFrame;
            tmpFrameSize = 0;
            /* Start to gather buffers in a frame */
            isFrame = 1;
        }

        /* Increment the pointer */
        CIRC_INC(tmpIdx, pDrv->wRxListSize);

        /* Copy data in the frame buffer */
        if (isFrame)
        {
            if (tmpIdx == pDrv->wRxI)
            {
                tr("no EOF (Invalid of buffers too small)\n\r");
                do
                {

                    pRxTd = &pDrv->pRxD[pDrv->wRxI];
                    pRxTd->addr.val &= ~(EMAC_RXD_bmOWNERSHIP);
                    CIRC_INC(pDrv->wRxI, pDrv->wRxListSize);
                } while(tmpIdx != pDrv->wRxI);
                return 0;
            }
            /* Copy the buffer into the application frame */
            bufferLength = EMAC_RX_UNITSIZE;
            if ((tmpFrameSize + bufferLength) > frameSize)
            {
                bufferLength = frameSize - tmpFrameSize;
            }

            memcpy(pTmpFrame, (void*)(pRxTd->addr.val & EMAC_RXD_ADDR_MASK), bufferLength);
            pTmpFrame += bufferLength;
            tmpFrameSize += bufferLength;
            
            /* An end of frame has been received, return the data */
            if ((pRxTd->status.val & EMAC_RXD_bmEOF) == EMAC_RXD_bmEOF)
            {
                /* Frame size from the EMAC */
                *pRcvSize = (pRxTd->status.val & EMAC_RXD_LEN_MASK);
                
                //tr("packet %d-%d (%d)\n\r", pDrv->wRxI, tmpIdx, *pRcvSize);
                /* All data have been copied in the application frame buffer => release TD */
                while (pDrv->wRxI != tmpIdx)
                {
                    pRxTd = &pDrv->pRxD[pDrv->wRxI];
                    pRxTd->addr.val &= ~(EMAC_RXD_bmOWNERSHIP);
                    CIRC_INC(pDrv->wRxI, pDrv->wRxListSize);
                }

                /* Application frame buffer is too small all data have not been copied */
                if (tmpFrameSize < *pRcvSize)
                {
                    tr("size req %u size allocated %u\n\r", *pRcvSize, frameSize);

                    return 0;
                }

                return 1;
            }
        }
        /* SOF has not been detected, skip the fragment */
        else
        {
           pRxTd->addr.val &= ~(EMAC_RXD_bmOWNERSHIP);
           pDrv->wRxI = tmpIdx;
        }
       
        /* Process the next buffer */
        pRxTd = &pDrv->pRxD[tmpIdx];
    }
    
    return 0;
}





