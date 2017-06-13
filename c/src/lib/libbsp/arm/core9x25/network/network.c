/*
 *  AT91SAM9G25 shoonis
 *  at91sam9g25 + RTL8201CP
 *  baconxu@gmail.com
 */

#include <rtems.h>
#include <rtems/rtems_bsdnet.h>
#include <at91sam9x25.h>
#include <at91sam9x25_emac.h>
#include <at91sam9x25_gpio.h>
#include <at91sam9x25_pmc.h>

#include <stdio.h>
#include <string.h>

#include <errno.h>
#include <rtems/error.h>

#include <sys/param.h>
#include <sys/mbuf.h>
#include <sys/socket.h>
#include <sys/sockio.h>

#include <net/if.h>

#include <netinet/in.h>
#include <netinet/if_ether.h>

#include <bsp/irq.h>
#include <bspopts.h>
#include <bsp.h>




/** The buffer addresses written into the descriptors must be aligned so the
    last few bits are zero.  These bits have special meaning for the EMAC
    peripheral and cannot be used as part of the address. */
#define EMAC_RXD_ADDR_MASK      0xFFFFFFFC
#define EMAC_RXD_bmWRAP         (1ul << 1)  /**< Wrap bit */
#define EMAC_RXD_bmOWNERSHIP    (1ul << 0)  /**< Ownership bit */

#define EMAC_RXD_bmBROADCAST    (1ul << 31) /**< Broadcast detected */
#define EMAC_RXD_bmMULTIHASH    (1ul << 30) /**< Multicast hash match */
#define EMAC_RXD_bmUNIHASH      (1ul << 29) /**< Unicast hash match */
#define EMAC_RXD_bmEXTADDR      (1ul << 28) /**< External address match */
#define EMAC_RXD_bmADDR1        (1ul << 26) /**< Address 1 match */
#define EMAC_RXD_bmADDR2        (1ul << 25) /**< Address 2 match */
#define EMAC_RXD_bmADDR3        (1ul << 24) /**< Address 3 match */
#define EMAC_RXD_bmADDR4        (1ul << 23) /**< Address 4 match */
#define EMAC_RXD_bmTYPE         (1ul << 22) /**< Type ID match */
#define EMAC_RXD_bmVLAN         (1ul << 21) /**< VLAN tag detected */
#define EMAC_RXD_bmPRIORITY     (1ul << 20) /**< Prority tag detected */
#define EMAC_RXD_PRIORITY_MASK  (3ul << 17) /**< VLAN prority */
#define EMAC_RXD_bmCFI          (1ul << 16) /**< Concatenation Format Indicator
                                                 only if bit 21 is set */
#define EMAC_RXD_bmEOF          (1ul << 15) /**< End of frame */
#define EMAC_RXD_bmSOF          (1ul << 14) /**< Start of frame */
#define EMAC_RXD_OFFSET_MASK    (3ul << 12) /**< Receive buffer offset */
#define EMAC_RXD_LEN_MASK       (0xFFF)     /**< Length of frame including FCS
                                                 (if selected) */
#define EMAC_RXD_LENJUMBO_MASK  (0x3FFF)    /**< Jumbo frame length */

#define EMAC_TXD_bmUSED         (1ul << 31) /**< Frame is transmitted */
#define EMAC_TXD_bmWRAP         (1ul << 30) /**< Last descriptor */
#define EMAC_TXD_bmERROR        (1ul << 29) /**< Retry limit exceed, error */
#define EMAC_TXD_bmUNDERRUN     (1ul << 28) /**< Transmit underrun */
#define EMAC_TXD_bmEXHAUSTED    (1ul << 27) /**< Buffer exhausted */
#define EMAC_TXD_bmNOCRC        (1ul << 16) /**< No CRC */
#define EMAC_TXD_bmLAST         (1ul << 15) /**< Last buffer in frame */
#define EMAC_TXD_LEN_MASK       (0x7FF)     /**< Length of buffer */


/** The MAC can support frame lengths up to 1536 bytes. */
#define EMAC_FRAME_LENTGH_MAX       1536
#define MIN_EMAC_PKG_LEN			60
#define MAX_EMAC_PKG_LEN			1518

/** RX callback */
typedef void (*fEmacdTransferCallback)(uint32_t status);
/** Wakeup callback */
typedef void (*fEmacdWakeupCallback)(void);

/** Receive buffer descriptor struct */
typedef struct _EmacRxDescriptor {
    union _EmacRxAddr {
        uint32_t val;
        struct _EmacRxAddrBM {
            uint32_t bOwnership:1,  /**< User clear, EMAC set this to one once
                                         it has successfully written a frame to
                                         memory */
                     bWrap:1,       /**< Marks last descriptor in receive buffer */
                     addrDW:30;     /**< Address in number of DW */
        } bm;
    } addr;                    /**< Address, Wrap & Ownership */
    union _EmacRxStatus {
        uint32_t val;
        struct _EmacRxStatusBM {
            uint32_t len:12,                /** Length of frame including FCS */
                     offset:2,              /** Receive buffer offset,
                                                bits 13:12 of frame length for jumbo
                                                frame */
                     bSof:1,                /** Start of frame */
                     bEof:1,                /** End of frame */
                     bCFI:1,                /** Concatenation Format Indicator */
                     vlanPriority:3,        /** VLAN priority (if VLAN detected) */
                     bPriorityDetected:1,   /** Priority tag detected */
                     bVlanDetected:1,       /**< VLAN tag detected */
                     bTypeIDMatch:1,        /**< Type ID match */
                     bAddr4Match:1,         /**< Address register 4 match */
                     bAddr3Match:1,         /**< Address register 3 match */
                     bAddr2Match:1,         /**< Address register 2 match */
                     bAddr1Match:1,         /**< Address register 1 match */
                     reserved:1,
                     bExtAddrMatch:1,       /**< External address match */
                     bUniHashMatch:1,       /**< Unicast hash match */
                     bMultiHashMatch:1,     /**< Multicast hash match */
                     bBroadcastDetected:1;  /**< Global all ones broadcast
                                                 address detected */
        } bm;
    } status;
} __attribute__ ((packed, aligned(8))) sEmacRxDescriptor;

/** Transmit buffer descriptor struct */
typedef struct _EmacTxDescriptor {
    uint32_t addr;
    union _EmacTxStatus {
        uint32_t val;
        struct _EmacTxStatusBM {
            uint32_t len:11,        /**< Length of buffer */
                     reserved:4,
                     bLastBuffer:1, /**< Last buffer (in the current frame) */
                     bNoCRC:1,      /**< No CRC */
                     reserved1:10,
                     bExhausted:1,  /**< Buffer exhausted in mid frame */
                     bUnderrun:1,   /**< Transmit underrun */
                     bError:1,      /**< Retry limit exceeded, error detected */
                     bWrap:1,       /**< Marks last descriptor in TD list */
                     bUsed:1;       /**< User clear, EMAC sets this once a frame
                                         has been successfully transmitted */
        } bm;
    } status;
}__attribute__ ((packed, aligned(8))) sEmacTxDescriptor;


/* enable debugging of the EMAC code */
/* #define EMAC_DBG */

/* interrupt stuff */
#define EMAC_INT_PRIORITY       3       /* lowest priority */

/*  RTEMS event used by interrupt handler to start receive daemon. */
#define START_RECEIVE_EVENT  RTEMS_EVENT_1

/* RTEMS event used to start transmit daemon. */
#define START_TRANSMIT_EVENT    RTEMS_EVENT_2


#define PHY_ADDRESS				1
#define PHY_RETRY				0
#define PHY_RETRY_AUTONEG       100000
#ifdef EMAC_DBG
#define DB(x)                   x
#else
#define DB(x)
#endif


static void at91sam9g25_emac_isr (rtems_irq_hdl_param unused);
static void at91sam9g25_emac_isr_on(const rtems_irq_connect_data *unused);
static void at91sam9g25_emac_isr_off(const rtems_irq_connect_data *unused);
static int at91sam9g25_emac_isr_is_on(const rtems_irq_connect_data *irq);

/* Replace the first value with the clock's interrupt name. */
rtems_irq_connect_data at91sam9g25_emac_isr_data = {
    AT91SAM9G25_INT_EMAC,
    at91sam9g25_emac_isr,
    AT91SAM9G25_INT_EMAC,
    at91sam9g25_emac_isr_on,
    at91sam9g25_emac_isr_off,
    at91sam9g25_emac_isr_is_on
};


/* Number of Receive and Transmit Buffers and Buffer Descriptors */
#define NUM_RXBDS 192
#define NUM_TXBDS 1
#define TX_BUFFER_SIZE  0x600
#define RX_BUFFER_SIZE  0x80

/* use the values defined in linkcmds for our use of SRAM */
extern volatile void * at91sam9g25_emac_buf_hdrs;
extern volatile void * at91sam9g25_emac_txbufs;
extern volatile void * at91sam9g25_emac_rxbufs;

/* Set up EMAC hardware */


/* use internal SRAM for buffers and descriptors
 * also insure that the receive descriptors
 * start on a 64byte boundary
 * Receive Buffer Descriptor Header
 */


volatile sEmacRxDescriptor *rxbuf_hdrs;
volatile sEmacTxDescriptor *txbuf_hdrs;
volatile fEmacdTransferCallback *tx_cbs;

volatile unsigned char   *txbuf;
volatile unsigned char   *rxbuf;

/*
 * Hardware-specific storage
 */
typedef struct
{
    /*
     * Connection to networking code
     * This entry *must* be the first in the sonic_softc structure.
     */
    struct arpcom                    arpcom;

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
    int                             link_status;
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
} at91sam9g25_emac_softc_t;

static at91sam9g25_emac_softc_t softc;

/* function prototypes */
int rtems_at91sam9g25_emac_attach (struct rtems_bsdnet_ifconfig *config,
                                  void *chip);
void at91sam9g25_emac_init(void *arg);
void at91sam9g25_emac_init_hw(at91sam9g25_emac_softc_t *sc);
void at91sam9g25_emac_start(struct ifnet *ifp);
void at91sam9g25_emac_stop (at91sam9g25_emac_softc_t *sc);
void at91sam9g25_emac_txDaemon (void *arg);
void at91sam9g25_emac_sendpacket (struct ifnet *ifp, struct mbuf *m);
void at91sam9g25_emac_rxDaemon(void *arg);
void at91sam9g25_emac_stats (at91sam9g25_emac_softc_t *sc);
static int at91sam9g25_emac_ioctl (struct ifnet *ifp,
                                  ioctl_command_t command,
                                  caddr_t data);

static void at91_emac_isr_handle(void);




uint32_t phywait(uint32_t retry);
int phyread(uint8_t phyaddr, uint8_t addr, uint32_t *data);
int phywrite(uint8_t phyaddr, uint8_t addr, uint32_t data);
void dumpPhyRegs();
uint8_t resetPhy();
uint8_t autoNegotiate( );
uint8_t getLinkSpeed(uint8_t applySetting);


int rtems_at91sam9g25_emac_attach (
    struct rtems_bsdnet_ifconfig *config,
    void *chip  /* only one ethernet, so no chip number */
    )
{
    struct ifnet *ifp;
    int mtu;
    int unitnumber;
    char *unitname;
    void *p;

    /* an array of receive buffer descriptors -- avoid type punned warning */
    p = (void *)&at91sam9g25_emac_buf_hdrs;
    rxbuf_hdrs = (sEmacRxDescriptor *)p;
	txbuf_hdrs = (sEmacTxDescriptor *)((uint32_t)((char *)p + NUM_RXBDS * sizeof(sEmacRxDescriptor) + 0x10) & 0xFFFFFFF0);

    /* one transmit buffer, 1536 bytes maximum */
    txbuf = (unsigned char *)&at91sam9g25_emac_txbufs;

    /* receive buffers starting address */
    rxbuf = (unsigned char *)&at91sam9g25_emac_rxbufs;
    /*
     * Parse driver name
     */
    if ((unitnumber = rtems_bsdnet_parse_driver_name (config, &unitname)) < 0)
        return 0;

    /*
     * Is driver free?
     */
    if (unitnumber != 0) {
        printk ("Bad AT91SAM9G25 EMAC unit number.\n");
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

	/*
	configure the mac address.
	*/

	/* get the MAC address from the chip */
    softc.arpcom.ac_enaddr[0] = 0x00;
    softc.arpcom.ac_enaddr[1] = 0x12;
    softc.arpcom.ac_enaddr[2] = 0x34;
    softc.arpcom.ac_enaddr[3] = 0x56;
    softc.arpcom.ac_enaddr[4] = 0x78;
    softc.arpcom.ac_enaddr[5] = 0x00;

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
    ifp->if_init = at91sam9g25_emac_init;
    ifp->if_ioctl = at91sam9g25_emac_ioctl;
    ifp->if_start = at91sam9g25_emac_start;
    ifp->if_output = ether_output;
    ifp->if_flags = IFF_BROADCAST;
    if (ifp->if_snd.ifq_maxlen == 0) {
        ifp->if_snd.ifq_maxlen = ifqmaxlen;
    }

    softc.rx_buf_idx = 0;

    /*
     * Attach the interface
     */
    if_attach (ifp);
    ether_ifattach (ifp);
    return 1;
}

void at91sam9g25_emac_init(void *arg)
{
    at91sam9g25_emac_softc_t     *sc = arg;
    struct ifnet *ifp = &sc->arpcom.ac_if;
	uint32_t dummy;
    /*
     *This is for stuff that only gets done once (at91rm9200_emac_init()
     * gets called multiple times
     */
    if (sc->txDaemonTid == 0) 
    {
        /* Set up EMAC hardware */
        at91sam9g25_emac_init_hw(sc);

        /*      Start driver tasks */
        sc->rxDaemonTid = rtems_bsdnet_newproc("ENrx",
                                               4096,
                                               at91sam9g25_emac_rxDaemon,
                                               sc);
        sc->txDaemonTid = rtems_bsdnet_newproc("ENtx",
                                               4096,
                                               at91sam9g25_emac_txDaemon,
                                               sc);
    } /* if txDaemonTid */

    /* set our priority in the AIC */
	AIC->AIC_SMR[AT91SAM9G25_INT_EMAC]
                            = (((EMAC_INT_PRIORITY) << AIC_SMR_PRIOR_Pos) & AIC_SMR_PRIOR_Msk) 
                                | (AIC_SMR_SRCTYPE_INT_EDGE_TRIGGERED & AIC_SMR_SRCTYPE_Msk);

    /* install the interrupt handler */
    BSP_install_rtems_irq_handler(&at91sam9g25_emac_isr_data);

    /* clear any pending interrupts */
	EMAC->EMAC_TSR = EMAC_TSR_UBR | EMAC_TSR_COL | EMAC_TSR_RLES 
		| EMAC_TSR_BEX | EMAC_TSR_COMP | EMAC_TSR_UND;
	EMAC->EMAC_RSR = EMAC_RSR_OVR | EMAC_RSR_REC | EMAC_RSR_BNA;
	

	dummy = EMAC->EMAC_ISR;
	dummy = EMAC->EMAC_NCFGR;

	/* Enable the copy of data into the buffers
       ignore broadcasts, and don't copy FCS. */
	EMAC->EMAC_NCFGR = dummy | EMAC_NCFGR_DRFCS | EMAC_NCFGR_PAE;

	DB(printk("ncfgr:%x\n", EMAC->EMAC_NCFGR));

	/*start statistics*/
	EMAC->EMAC_NCR |=  EMAC_NCR_WESTAT;
	/* Enable TX/RX and clear the statistics counters */
	EMAC->EMAC_NCR |= EMAC_NCR_TE | EMAC_NCR_RE | EMAC_NCR_CLRSTAT;

	/* EMAC doesn't support promiscuous, so ignore requests */
    if (ifp->if_flags & IFF_PROMISC) {
        printk ("Warning - AT91SAM9G25 Ethernet driver"
                " doesn't support Promiscuous Mode!\n");
    }

	/*
     * Tell the world that we're running.
     */
    ifp->if_flags |= IFF_RUNNING;
}

static const Pin at91sam9g25_emac_mii_pins[] = { PINS_EMAC0_MII };


void  at91sam9g25_emac_init_hw(at91sam9g25_emac_softc_t *sc)
{
    int i;
	uint8_t bCLK = 0;
	uint32_t dwMck;
	uint32_t rstc_mr;

	if ((PMC->PMC_PCSR & (1 << ID_PIOB)) != (1 << ID_PIOB)) 
	{
		PMC->PMC_PCER = 1 << ID_PIOB;
	}
	PIO_Configure(at91sam9g25_emac_mii_pins, PIO_LISTSIZE(at91sam9g25_emac_mii_pins));

    /* Enable the clock to the EMAC */
	if ((PMC->PMC_PCSR & (1 << ID_EMAC)) != (1 << ID_EMAC))
	{
		PMC->PMC_PCER = 1 << ID_EMAC;
	}
	
	rstc_mr = RSTC->RSTC_MR;
	rstc_mr &= ~(RSTC_MR_KEY_Msk | RSTC_MR_ERSTL_Msk);
	rstc_mr |= RSTC_MR_ERSTL(0xD);
	RSTC->RSTC_MR = rstc_mr | RSTC_MR_KEY(0xA5U);
   
	RSTC->RSTC_CR = RSTC_CR_EXTRST | RSTC_MR_KEY(0xA5U);
	while ((RSTC->RSTC_SR & RSTC_SR_NRSTL) == 0);

	dwMck = at91sam9g25_get_mck();
	DB(printk("mck:%dMHz\n", dwMck / 1000 / 1000));
    /* Not supported */
    if (dwMck > 160*1000*1000)
    {
        printk("not support...this clock...\n");
    }
    else if (dwMck > 80*1000*1000)
    {
        bCLK = 3;
    }
    else if (dwMck > 40*1000*1000)
    {
        bCLK = 2;
    }
    else if (dwMck > 20*1000*1000)
    {
        bCLK = 1;
    }

    EMAC->EMAC_NCFGR =  (EMAC_NCFGR_CLK_Msk & ((bCLK) << EMAC_NCFGR_CLK_Pos));

	/*disable tx & rx*/
	EMAC->EMAC_NCR &= ~(EMAC_NCR_TE | EMAC_NCR_RE);
	
	/* initialize our transmitter buffer descriptors */		
	for (i = 0; i < NUM_TXBDS; i++)
	{
		uint32_t address = (uint32_t)(&(txbuf[i * TX_BUFFER_SIZE]));
		txbuf_hdrs[i].addr = address;
		txbuf_hdrs[i].status.val = EMAC_TXD_bmUSED;
	}
	txbuf_hdrs[NUM_TXBDS - 1].status.val = EMAC_TXD_bmUSED | EMAC_TXD_bmWRAP;
	/* point to our transmitter buffer queue */
	EMAC->EMAC_TBQP = EMAC_TBQP_ADDR_Msk & ((uint32_t)txbuf_hdrs);

    /* initialize our receive buffer descriptors */
    for (i = 0; i < NUM_RXBDS; i++) {
		uint32_t address = (uint32_t)(&(rxbuf[i * RX_BUFFER_SIZE]));
        rxbuf_hdrs[i].addr.val = ((uint32_t)address) & EMAC_RXD_ADDR_MASK;
        rxbuf_hdrs[i].status.val = 0;
        DB(printk("%d:%X\n", i, rxbuf_hdrs[i].addr.val));
    }
	rxbuf_hdrs[NUM_RXBDS - 1].addr.val |= EMAC_RXD_bmWRAP;
	/* point to our receive buffer queue */
	EMAC->EMAC_RBQP = EMAC_RBQP_ADDR_Msk & ((uint32_t)rxbuf_hdrs);

	EMAC->EMAC_SA[0].EMAC_SAxB = (softc.arpcom.ac_enaddr[3] << 24)
                                     | (softc.arpcom.ac_enaddr[2] << 16)
                                     | (softc.arpcom.ac_enaddr[1] <<  8)
                                     | (softc.arpcom.ac_enaddr[0]      )
                                     ;
    EMAC->EMAC_SA[0].EMAC_SAxT = (softc.arpcom.ac_enaddr[5] <<  8)
                                     | (softc.arpcom.ac_enaddr[4]      )
                                     ;
    resetPhy();
    sc->link_status = autoNegotiate();
}


void at91sam9g25_emac_start(struct ifnet *ifp)
{
    at91sam9g25_emac_softc_t *sc = ifp->if_softc;

    rtems_event_send(sc->txDaemonTid, START_TRANSMIT_EVENT);
    ifp->if_flags |= IFF_OACTIVE;
}

void at91sam9g25_emac_stop (at91sam9g25_emac_softc_t *sc)
{
    struct ifnet *ifp = &sc->arpcom.ac_if;

    ifp->if_flags &= ~IFF_RUNNING;

    /*
     * Stop the transmitter and receiver.
     */
    EMAC->EMAC_NCR &= ~(EMAC_NCR_TE | EMAC_NCR_RE);
}


int at91sam9g25_emac_check_busy(void)
{
	uint32_t    regval;

	regval = EMAC->EMAC_TSR;
	if ((regval & EMAC_TSR_TGO) == EMAC_TSR_TGO)
	{
		return 1;
	}

	if ((txbuf_hdrs->status.val & EMAC_TXD_bmUSED) == 0)
	{
		return 1;
	}

	return 0;
}



#ifdef SUPPORT_MAC_COMM
typedef struct {
	unsigned char *ptr;
	unsigned length;
} TSendBuffInfo;

TSendBuffInfo gSend_buff_info = 
{
	.ptr = NULL, 
	.length = 0
};
static rtems_id gSend_packet_sem;
static rtems_id gSend_packet_comp;
#endif

/*
 * Driver transmit daemon
 */
void at91sam9g25_emac_txDaemon (void *arg)
{
    at91sam9g25_emac_softc_t *sc = (at91sam9g25_emac_softc_t *)arg;
    struct ifnet *ifp = &sc->arpcom.ac_if;
    struct mbuf *m;
    rtems_event_set events;
    rtems_status_code rsc;

#ifdef SUPPORT_MAC_COMM
	rsc = rtems_semaphore_create (rtems_build_name('P', 'a', 'T', 'x'),
			1,
			RTEMS_PRIORITY |
			RTEMS_BINARY_SEMAPHORE |
			RTEMS_INHERIT_PRIORITY |
			RTEMS_NO_PRIORITY_CEILING |
			RTEMS_LOCAL,
			0,
			&gSend_packet_sem);

	if (rsc != RTEMS_SUCCESSFUL)
	{
		rtems_panic("can't not allocate rtems semaphore.");
	}

	rsc = rtems_semaphore_create(rtems_build_name('T', 'x', 'C', 'p'),
			0,
			RTEMS_PRIORITY |
			RTEMS_BINARY_SEMAPHORE |
			RTEMS_INHERIT_PRIORITY |
			RTEMS_NO_PRIORITY_CEILING |
			RTEMS_LOCAL,
			0,
			&gSend_packet_comp);
	if (rsc != RTEMS_SUCCESSFUL)
	{
		rtems_panic("can't not allocate rtems semaphore.");
	}
#endif

    for (;;)
    {
        rsc = rtems_bsdnet_event_receive(
            START_TRANSMIT_EVENT,
            RTEMS_EVENT_ANY | RTEMS_WAIT,
            RTEMS_NO_TIMEOUT, //10 * ticks_per_second,
            &events);
#if 0
        if (rsc == RTEMS_TIMEOUT) 
        {
            uint32_t value;
            EMAC->EMAC_NCR |=  EMAC_NCR_MPE;
            phyread(PHY_ADDRESS, MII_BMSR, &value);
            EMAC->EMAC_NCR &=  ~EMAC_NCR_MPE;

            if ((value & MII_LINK_STATUS) == 0 || !sc->link_status)
            {
                printk("WARNING:Network lost connection.\n");
                resetPhy();
                sc->link_status = autoNegotiate();
                if (sc->link_status)
                {
                    printk("Found a connection.\n");
                }
                else
                {
                    dumpPhyRegs();
					rtems_bsdnet_show_mbuf_stats();
					rtems_bsdnet_show_icmp_stats();
					printk("IPR:%x\n", AIC->AIC_IPR);
                    printk("Not found a connection.\n");
                }
            }
        }

        if (rsc != RTEMS_SUCCESSFUL)
        {
            continue;
        }
#endif

        /* Send packets till queue is empty */
        DB(printk("."));
        for (;;)
        {
            if (at91sam9g25_emac_check_busy())
            {
                break;
            }
#ifdef SUPPORT_MAC_COMM
			if ((gSend_buff_info.length != 0) && 
				(gSend_buff_info.ptr != NULL))
			{
				memcpy(txbuf, gSend_buff_info.ptr, gSend_buff_info.length);
				/*get the mac address*/
				memcpy(&txbuf[6], softc.arpcom.ac_enaddr, 6);
				txbuf[12] = (SEND_PACKET_TYPE >> 8) & 0xff;
				txbuf[13] = (SEND_PACKET_TYPE >> 0) & 0xff;

				txbuf_hdrs->addr  = (uint32_t)txbuf;
				txbuf_hdrs->status.val = (gSend_buff_info.length & EMAC_TXD_LEN_MASK)
										| EMAC_TXD_bmLAST | EMAC_TXD_bmWRAP; 

				EMAC->EMAC_NCR |= EMAC_NCR_TSTART;
				gSend_buff_info.length = 0;
				gSend_buff_info.ptr = NULL;
				rtems_semaphore_release(gSend_packet_comp);
				rtems_semaphore_release(gSend_packet_sem);
				continue;
			}
#endif
			/* Get the next mbuf chain to transmit. */
            IF_DEQUEUE(&ifp->if_snd, m);
            if (!m)
                break;
			else
				at91sam9g25_emac_sendpacket (ifp, m);
        }
        ifp->if_flags &= ~IFF_OACTIVE;
    }
}

/* Send packet */
void at91sam9g25_emac_sendpacket (struct ifnet *ifp, struct mbuf *m)
{
    struct mbuf *l = NULL;
    unsigned int pkt_offset = 0;

    /* copy the mbuf chain into the transmit buffer */
    l = m;
    while (l != NULL) {
        memcpy(((char *)txbuf + pkt_offset),  /* offset into pkt for mbuf */
               (char *)mtod(l, void *),       /* cast to void */
               l->m_len);                     /* length of this mbuf */

        pkt_offset += l->m_len;               /* update offset */
        l = l->m_next;                        /* get next mbuf, if any */
    }
    /* free the mbuf chain we just copied */
    m_freem(m);

	txbuf_hdrs->addr  = (uint32_t)txbuf;
	txbuf_hdrs->status.val = (pkt_offset & EMAC_TXD_LEN_MASK)
		| EMAC_TXD_bmLAST | EMAC_TXD_bmWRAP; 

	EMAC->EMAC_NCR |= EMAC_NCR_TSTART;
} /* at91rm9200_emac_sendpacket () */


#ifdef SUPPORT_MAC_COMM
/**
 * @brief it's for send packet of SEND_PACKET_TYPE
 * 
 * @author bacon (10/5/2015)
 * 
 * @param packet paket data, it's a unsigned char buffer.
 * @param length it's length of buffer.
 *  
 */
void send_packet(void *packet, unsigned length)
{
	rtems_event_send (softc.txDaemonTid, START_TRANSMIT_EVENT);
	rtems_semaphore_obtain(gSend_packet_sem, RTEMS_NO_TIMEOUT, 0);
	gSend_buff_info.ptr = packet;
	gSend_buff_info.length = length;
	rtems_semaphore_obtain(gSend_packet_comp, RTEMS_NO_TIMEOUT, 0);
}
#endif

static uint32_t at91sam9g25_get_pkg_num(void)
{
	int i;
	uint32_t num = 0;

	for (i = 0; i < NUM_RXBDS; i++)
	{
		if (((rxbuf_hdrs[i].status.val & EMAC_RXD_bmEOF) > 0) &&
            ((rxbuf_hdrs[i].addr.val & EMAC_RXD_bmOWNERSHIP) > 0))
			
            num++;
	}

	return (num);
}

static uint32_t at91sam9g25_get_pkg_size(int *startidx, int *endidx)
{
	while (((rxbuf_hdrs[*startidx].status.val & EMAC_RXD_bmSOF) == 0) ||
			((rxbuf_hdrs[*startidx].addr.val & EMAC_RXD_bmOWNERSHIP) == 0))
	{
		(*startidx)++;
		
		if (*startidx >= NUM_RXBDS)
			*startidx = 0;
	}

	*endidx = *startidx;
	while (((rxbuf_hdrs[*endidx].status.val & EMAC_RXD_bmEOF) == 0) ||
			((rxbuf_hdrs[*endidx].addr.val & EMAC_RXD_bmOWNERSHIP) == 0))
	{
		(*endidx)++;

		if (*endidx >= NUM_RXBDS)
			*endidx = 0;

		if (((rxbuf_hdrs[*endidx].status.val & EMAC_RXD_bmSOF) > 0) &&
			((rxbuf_hdrs[*endidx].addr.val & EMAC_RXD_bmOWNERSHIP) > 0))
		{
			while (*startidx != *endidx)
			{
				rxbuf_hdrs[*startidx].addr.val &= ~EMAC_RXD_bmOWNERSHIP;
				(*startidx)++;
				if (*startidx >= NUM_RXBDS)
					*startidx = 0;
			}
			*startidx = *endidx;
		}
	}
	
	return (rxbuf_hdrs[*endidx].status.val & EMAC_RXD_LEN_MASK);
}

void at91sam9g25_discard_pkg(int startidx, int endidx)
{
	while (startidx != endidx)
	{
		rxbuf_hdrs[startidx].addr.val &= ~EMAC_RXD_bmOWNERSHIP;
		startidx++;
		if (startidx >= NUM_RXBDS)
			startidx = 0;
	}
	rxbuf_hdrs[endidx].addr.val &= ~EMAC_RXD_bmOWNERSHIP;
}

#ifdef SUPPORT_MAC_COMM
static TRecvPacketCallback gAt91sam9g25_callback = NULL;
void set_recv_packet_callback(TRecvPacketCallback callback)
{
	if (callback == NULL)
		return ;
	gAt91sam9g25_callback = callback;
}
#endif

static void at91sam9g25_rev_pkg(struct ifnet *ifp, int startidx, int endidx)
{
	struct mbuf *m;
	struct ether_header *eh;
	unsigned char *p, *q;
	int offset;
	int pktlen;
	int left;
	int error = 0;

	/* get an mbuf this packet */
    MGETHDR(m, M_WAIT, MT_DATA);
	
    /* now get a cluster pointed to by the mbuf */
    /* since an mbuf by itself is too small */
    MCLGET(m, M_WAIT);
        
    /* set the type of mbuf to ifp (ethernet I/F) */
    m->m_pkthdr.rcvif = ifp;
    m->m_nextpkt = 0;

	p = (unsigned char *)m->m_data;
	offset = (rxbuf_hdrs[startidx].status.val & EMAC_RXD_OFFSET_MASK) >> 12;
	pktlen = (rxbuf_hdrs[endidx].status.val & EMAC_RXD_LEN_MASK);
	
	left = pktlen;
	/* copy the packet into the cluster pointed to by the mbuf */
	while (startidx != endidx)
	{
		q = (unsigned char *)(rxbuf_hdrs[startidx].addr.val & EMAC_RXD_ADDR_MASK);
		if ((rxbuf_hdrs[startidx].status.val & EMAC_RXD_bmSOF) > 0)
		{
			if (0 == error)
			{
				memcpy(p, q + offset, RX_BUFFER_SIZE - offset);
				p += (RX_BUFFER_SIZE - offset);
			}

			if ((left - (RX_BUFFER_SIZE - offset)) >= 0)
			{
				left -= (RX_BUFFER_SIZE - offset);
				offset = 0;
			}
			else
				error = 1;
		}
		else
		{
			if (0 == error)
			{
				memcpy(p, q, RX_BUFFER_SIZE);
				p += (RX_BUFFER_SIZE);
			}

			if (left - RX_BUFFER_SIZE >= 0)
				left -= RX_BUFFER_SIZE;
			else
				error = 1;
		}

		rxbuf_hdrs[startidx].addr.val &= ~EMAC_RXD_bmOWNERSHIP;
		startidx++;
		if (startidx >= NUM_RXBDS)
			startidx = 0;
	}


	q = (unsigned char *)(rxbuf_hdrs[startidx].addr.val & EMAC_RXD_ADDR_MASK);
	if (0 == error)
	{
		memcpy(p, q + offset, left - offset);
	}
	rxbuf_hdrs[startidx].addr.val &= ~EMAC_RXD_bmOWNERSHIP;
 
    if (error) 
    {
        m_freem(m);
        return ;
    }
    /* set the length of the mbuf */
    m->m_len = pktlen - (sizeof(struct ether_header));
    m->m_pkthdr.len = m->m_len;

#ifdef SUPPORT_MAC_COMM
	p = (unsigned char *)m->m_data;
#endif
    /* strip off the ethernet header from the mbuf */
    /* but save the pointer to it */
    eh = mtod (m, struct ether_header *);
    m->m_data += sizeof(struct ether_header);

#ifdef SUPPORT_MAC_COMM
	if ((p[12] == ((RECV_PACKET_TYPE >> 8) & 0xFF)) &&
		(p[13] == ((RECV_PACKET_TYPE >> 0) & 0xFF)) &&
		(0 == error)) 
	{
		/*here, we received a packet for RECV_PACKET_TYPE.*/
		if (gAt91sam9g25_callback == NULL)
		{
			m_freem(m);
			return ;
		}
		
		gAt91sam9g25_callback(p, pktlen);
		m_freem(m);
		return ;
	}
#endif

    /* give all this stuff to the stack */
	if (0 == error)
    	ether_input(ifp, eh, m);
	else
		m_freem(m);
}

/* at91sam9g25 reader task */
void at91sam9g25_emac_rxDaemon(void *arg)
{
    at91sam9g25_emac_softc_t *sc = (at91sam9g25_emac_softc_t *)arg;
    struct ifnet *ifp = &sc->arpcom.ac_if;
    struct mbuf *m;
    struct ether_header *eh;
    rtems_event_set events;
    int pktlen;
	int revpkg;
	int start;
	int end;
	rtems_status_code status;
	
    /* Input packet handling loop */
    for (;;) {
        /* turn on RX interrupts, then wait for one */
        status = rtems_bsdnet_event_receive(
            START_RECEIVE_EVENT,
            RTEMS_EVENT_ANY | RTEMS_WAIT,
            1, //RTEMS_NO_TIMEOUT,
            &events);
		
		if (status == RTEMS_TIMEOUT)
		{
			at91_emac_isr_handle();
			continue;
		}

		revpkg = at91sam9g25_get_pkg_num();
		while (revpkg > 0)
		{
			revpkg--;
			start = sc->rx_buf_idx;
			pktlen = at91sam9g25_get_pkg_size(&start, &end);
			
			sc->rx_buf_idx = end + 1;
			if (sc->rx_buf_idx >= NUM_RXBDS)
				sc->rx_buf_idx = 0;

			/*if error occur, just release the package*/
			if ((pktlen < MIN_EMAC_PKG_LEN) || (pktlen > MAX_EMAC_PKG_LEN))
			{
				while (start != end)
				{
					rxbuf_hdrs[start].addr.val &= ~EMAC_RXD_bmOWNERSHIP;
					start++;
					if (start >= NUM_RXBDS)
						start = 0;
				}
				rxbuf_hdrs[end].addr.val &= ~EMAC_RXD_bmOWNERSHIP;
				continue;
			}
			at91sam9g25_rev_pkg(ifp, start, end);
		}
    } /* for (;;) */
} /* at91sam9g25_emac_rxDaemon() */


/* Show interface statistics */
void at91sam9g25_emac_stats (at91sam9g25_emac_softc_t *sc)
{
    printf ("    Total Interrupts:%-8lu\n",          sc->Interrupts);
    printf ("       Rx Interrupts:%-8lu\n",          sc->rxInterrupts);
    printf ("               Giant:%-8lu\n",          sc->rxGiant);
    printf ("           Non-octet:%-8lu\n",                sc->rxNonOctet);
    printf ("             Bad CRC:%-8lu\n",          sc->rxBadCRC);
    printf ("           Collision:%-8lu\n",          sc->rxCollision);
    printf ("              Missed:%-8lu\n",                sc->rxMissed);

    printf ("       Tx Interrupts:%-8lu\n",      sc->txInterrupts);
    printf ("            Deferred:%-8lu\n",      sc->txDeferred);
    printf ("        Lost Carrier:%-8lu\n",     sc->txLostCarrier);
    printf ("   Single Collisions:%-8lu\n",       sc->txSingleCollision);
    printf (" Multiple Collisions:%-8lu\n",       sc->txMultipleCollision);
    printf ("Excessive Collisions:%-8lu\n",     sc->txExcessiveCollision);
    printf ("    Total Collisions:%-8lu\n",       sc->txCollision);
    printf ("      Late Collision:%-8lu\n",       sc->txLateCollision);
    printf ("            Underrun:%-8lu\n",     sc->txUnderrun);
    printf ("     Raw output wait:%-8lu\n",     sc->txRawWait);

    dumpPhyRegs();
}


/* Enables at91sam9g25_emac interrupts. */
static void at91sam9g25_emac_isr_on(const rtems_irq_connect_data *unused)
{
    /* Enable various TX/RX interrupts */ 
	EMAC->EMAC_IER     =  EMAC_IER_RXUBR
                        | EMAC_IER_TUND
                        | EMAC_IER_RLE
                        | EMAC_IER_TCOMP
						| EMAC_IER_RCOMP
                        | EMAC_IER_ROVR
                        | EMAC_IER_HRESP;
	DB(printk("in at91sam9g25_emac_isr_on\n"));

    return;
}

/* Disables at91sam9g25_emac interrupts */
static void at91sam9g25_emac_isr_off(const rtems_irq_connect_data *unused)
{
    /* disable all various TX/RX interrupts */
	//EMAC->EMAC_IDR = 0x3CFF;
    return;
}

/* Tests to see if at91sam9g25_emac interrupts are enabled, and
 * returns non-0 if so.
 * If interrupt is not enabled, returns 0.
 */
static int at91sam9g25_emac_isr_is_on(const rtems_irq_connect_data *irq)
{
	/* any interrupts enabled? */
    return (EMAC->EMAC_IMR);
}

/*  Driver ioctl handler */
static int
at91sam9g25_emac_ioctl (struct ifnet *ifp, ioctl_command_t command, caddr_t data)
{
    at91sam9g25_emac_softc_t *sc = ifp->if_softc;
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
            at91sam9g25_emac_stop (sc);
            break;

        case IFF_UP:
            at91sam9g25_emac_init (sc);
            break;

        case IFF_UP | IFF_RUNNING:
            at91sam9g25_emac_stop (sc);
            at91sam9g25_emac_init (sc);
            break;

        default:
            break;
        } /* switch (if_flags) */
        break;

    case SIO_RTEMS_SHOW_STATS:
        at91sam9g25_emac_stats (sc);
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



static void at91_emac_isr_handle(void)
{
	uint32_t status32;

    /* get the ISR status and determine RX or TX */
    status32 = EMAC->EMAC_ISR;

	softc.Interrupts++;
    if (status32 & EMAC_ISR_RCOMP)
	{
		/* disable the RX interrupts */
		softc.rxInterrupts++;
        rtems_event_send (softc.rxDaemonTid, START_RECEIVE_EVENT);
    }

    if (status32 & EMAC_ISR_TCOMP) 
	{      /* Transmit buffer register empty */
		softc.txInterrupts++;
		if (status32 & (EMAC_ISR_TUND | EMAC_ISR_RLEX))
		{
			/*tx error*/
		}
		
        rtems_event_send (softc.txDaemonTid, START_TRANSMIT_EVENT);

    }

	if (status32 & EMAC_ISR_RXUBR)
	{
		uint32_t ctl = EMAC->EMAC_NCR;
		EMAC->EMAC_NCR = ctl & (~EMAC_NCR_RE);
		asm volatile ("nop");
		asm volatile ("nop");
		EMAC->EMAC_NCR = ctl | EMAC_NCR_RE;
	}

	if (status32 & EMAC_ISR_ROVR)
	{
		printk("EMAC_ISR_ROVR error\n\r");
	}
}

/* interrupt handler */
static void at91sam9g25_emac_isr (rtems_irq_hdl_param unused)
{
	at91_emac_isr_handle();
    AIC->AIC_ICCR = 1 << AT91SAM9G25_INT_EMAC;
}






/*-------------------------------------------------------------------------------------------------------------------







---------------------------------------------------------------------------------------------------------------------
*/


/** \addtogroup mii_registers PHY registers Addresses
    @{*/
#define MII_BMCR        0   /**< Basic Mode Control Register */
#define MII_BMSR        1   /**< Basic Mode Status Register */
#define MII_PHYID1      2   /**< PHY Idendifier Register 1 */
#define MII_PHYID2      3   /**< PHY Idendifier Register 2 */
#define MII_ANAR        4   /**< Auto_Negotiation Advertisement Register */
#define MII_ANLPAR      5   /**< Auto_negotiation Link Partner Ability Register */
#define MII_ANER        6   /**< Auto-negotiation Expansion Register */
#define MII_NSR        16   /**< Specified Configuration Register */
#define MII_LBREMR     17   /**< Specified Configuration and Status Register */
#define MII_REC        18   /**< 10BASE-T Configuration and Satus Register */
#define MII_SNR        19   /**< Power Down Control Register */
#define MII_TEST       25   /**< Hardware Test */
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
#define MII_OUI_MSB            0x0000
#define MII_OUI_LSB            0x8201


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
/** @}*/




/*PINS_EMAC0_MII*/

#define phy_access(phyAddr, addr, bRw, data) \
	do {\
		/* Wait until bus idle */ \
    	while((EMAC->EMAC_NSR & EMAC_NSR_IDLE) == 0); \
    	/* Write maintain register */ \
    	EMAC->EMAC_MAN = EMAC_MAN_CODE(10) | EMAC_MAN_SOF(0x1)\
                    | EMAC_MAN_PHYA((phyAddr))\
                    | EMAC_MAN_REGA((addr))\
                    | EMAC_MAN_RW(((bRw) ? 0x2 : 0x1))\
                    | EMAC_MAN_DATA((data))\
                    ;\
	} while (0)

#define phy_data(data) \
	do {\
		/* Wait until bus idle */ \
    	while((EMAC->EMAC_NSR & EMAC_NSR_IDLE) == 0); \
    	/* Return data */ \
        data = (uint16_t)(EMAC->EMAC_MAN & EMAC_MAN_DATA_Msk);\
	} while (0)


#define phy_is_idle() ((EMAC->EMAC_NSR & EMAC_NSR_IDLE) > 0)


uint32_t phywait(uint32_t retry)
{
    volatile uint32_t retry_count = 0;

    while (!phy_is_idle())
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

/*
 * phyread(): Read the PHY
 */
int phyread(uint8_t phyaddr, uint8_t addr, uint32_t *data)
{ 
	phy_access(phyaddr, addr, 1, 0);
	if (phywait(PHY_RETRY) == 0)
	{
		return (0);
	}
	phy_data(*data);//micro, can return data directly.
					//not a cite.
	return (1);
}

/**
 * phywrite(): Write the PHY
 */
int phywrite(uint8_t phyaddr, uint8_t addr, uint32_t data)
{
	phy_access(phyaddr, addr, 0, data);
	if (phywait(PHY_RETRY) == 0)
		return 0;
#if defined(EMAC_DBG)
	printk("EMAC: Phy %d, Reg %d, Write 0x%04x.\n", phyaddr, addr, data);
#endif 

	/* wait for phy write to complete (was udelay(5000)) */
	rtems_task_wake_after(1);
	return 1;
}

/**
 * Dump all the useful registers
 * \param pMacb          Pointer to the MACB instance
 */
void dumpPhyRegs()
{
    uint8_t phyAddress;
	uint32_t value;
 
    EMAC->EMAC_NCR |=  EMAC_NCR_MPE;

    phyAddress = PHY_ADDRESS;

    printf("\n\rRTL8201cp Registers:\n\r", phyAddress);

    phyread(phyAddress, MII_BMCR, &value);
    printf(" _BMCR   : 0x%X\n\r", value);
	
    phyread(phyAddress, MII_BMSR, &value);
    printf(" _BMSR   : 0x%X\n\r", value);
	
    phyread(phyAddress, MII_PHYID1, &value);
    printf(" _PHYID1 : 0x%X ", value);

    phyread(phyAddress, MII_PHYID2, &value);
    printf(" _PHYID2 : 0x%X\n\r", value);

    phyread(phyAddress, MII_ANAR, &value);
    printf(" _ANAR   : 0x%X\n\r", value);
	
    phyread(phyAddress, MII_ANLPAR, &value);
    printf(" _ANLPAR : 0x%X\n\r", value);
	
    phyread(phyAddress, MII_ANER, &value);
    printf(" _ANER   : 0x%X\n\r", value);

	phyread(phyAddress, MII_NSR, &value);
    printf(" _NSR    : 0x%X\n\r", value);

    phyread(phyAddress, MII_LBREMR, &value);
    printf(" _LBREMR : 0x%X\n\r", value);
	
    phyread(phyAddress, MII_REC, &value);
    printf(" _REC    : 0x%X\n\r", value);
	
    phyread(phyAddress, MII_SNR, &value);
    printf(" _SNR    : 0x%X\n\r", value);
	
    EMAC->EMAC_NCR &= ~EMAC_NCR_MPE;
}

/**
 * Issue a SW reset to reset all registers of the PHY
 * Return 1 if successfully, 0 if timeout.
 * \param pMacb   Pointer to the MACB instance
 */
uint8_t resetPhy()
{
    uint32_t bmcr = MII_RESET;
    uint8_t phyAddress;
    uint32_t timeout = 10;
    uint8_t ret = 1;
 
    phyAddress = PHY_ADDRESS; 
    EMAC->EMAC_NCR |=  EMAC_NCR_MPE;

    bmcr = MII_RESET;
    phywrite(phyAddress, MII_BMCR, bmcr);

    do
    {
        phyread(phyAddress, MII_BMCR, &bmcr);
        timeout--;
    } while ((bmcr & MII_RESET) && timeout);

    EMAC->EMAC_NCR &= ~EMAC_NCR_MPE;
    if (!timeout)
    {
        ret = 0;
    }

    return( ret );
}



/**
 * Issue a Auto Negotiation of the PHY
 * Return 1 if successfully, 0 if timeout.
 * \param pMacb   Pointer to the MACB instance
 */
uint8_t autoNegotiate( )
{
	uint32_t value;
    uint32_t phyAnar;
    uint32_t phyAnalpar;
    uint32_t retryCount= 0;
    uint8_t phyAddress;
    uint8_t bFD = 0;
    uint8_t bSP = 0;
    uint8_t rc = 1;
 
    phyAddress = PHY_ADDRESS;

    EMAC->EMAC_NCR |=  EMAC_NCR_MPE;

    if (!phyread(phyAddress, MII_PHYID1, &value))
    {
        DB(printk("Pb EMAC_ReadPhy Id1\n\r"));
        rc = 0;
        goto AutoNegotiateExit;
    }
    DB(printk("ReadPhy Id1 0x%X, addresse: %d\n\r", value, phyAddress));
    if (!phyread(phyAddress, MII_PHYID2, &phyAnar))
    {
        DB(printk("Pb EMAC_ReadPhy Id2\n\r"));
        rc = 0;
        goto AutoNegotiateExit;
    }
    DB(printk("ReadPhy Id2 0x%X\n\r", phyAnar));

    if( ( value == MII_OUI_MSB )
     && ( (phyAnar & 0xFFFF) == MII_OUI_LSB ) )
    {
        DB(printk("Vendor Number Model = 0x%X\n\r", ((phyAnar>>4)&0x3F)));
        DB(printk("Model Revision Number = 0x%X\n\r", (phyAnar&0x7)));
    }
    else
    {
        DB(printk("Problem OUI value\n\r"));
    }        

    /* Setup control register */
    rc  = phyread(phyAddress, MII_BMCR, &value);
    if (rc == 0)
    {
        goto AutoNegotiateExit;
    }
	DB(printk("MII_BMCR:%d\n", value));
	
    value &= ~MII_AUTONEG;   /* Remove autonegotiation enable */
    value &= ~(MII_LOOPBACK|MII_POWER_DOWN);
	//value = MII_DUPLEX_MODE | MII_SPEED_SELECT;
    rc = phywrite(phyAddress, MII_BMCR, value);
    if (rc == 0)
    {
        goto AutoNegotiateExit;
    }


    /* Set the Auto_negotiation Advertisement Register
       MII advertising for Next page
       100BaseTxFD and HD, 10BaseTFD and HD, IEEE 802.3 */
    phyAnar = MII_TX_FDX | MII_TX_HDX |
              MII_10_FDX | MII_10_HDX | MII_AN_IEEE_802_3;
    rc = phywrite(phyAddress, MII_ANAR, phyAnar);
    if (rc == 0)
    {
        goto AutoNegotiateExit;
    }

    /* Read & modify control register */
    rc  = phyread(phyAddress, MII_BMCR, &value);
    if (rc == 0)
    {
        goto AutoNegotiateExit;
    }

    value |= MII_SPEED_SELECT | MII_AUTONEG | MII_DUPLEX_MODE;
    rc = phywrite(phyAddress, MII_BMCR, value);
    if (rc == 0)
    {
        goto AutoNegotiateExit;
    }

    /* Restart Auto_negotiation */
    value |=  MII_RESTART_AUTONEG;
    value &= ~MII_ISOLATE;
    rc = phywrite(phyAddress, MII_BMCR, value);
    if (rc == 0)
    {
        goto AutoNegotiateExit;
    }

    DB(printk(" _BMCR: 0x%X\n\r", value));


    /* Check AutoNegotiate complete */
    while (1)
    {
        rc  = phyread(phyAddress, MII_BMSR, &value);
        if (rc == 0)
        {
            DB(printk("_BMSR Rd err\n\r"));
            goto AutoNegotiateExit;
        }
        /* Done successfully */
        if (value & MII_AUTONEG_COMP)
        {
            DB(printk("AutoNegotiate complete, %d wait cycle.\n\r", retryCount));
            break;
        }

        /* Timeout check */
        if (PHY_RETRY_AUTONEG)
        {
            if (++ retryCount >= PHY_RETRY_AUTONEG)
            {
                DB(printk("AutoNegotiate TimeOut\n\r"));
                EMAC->EMAC_NCFGR |=  EMAC_NCFGR_SPD | EMAC_NCFGR_FD;
                rc = 0;
                goto AutoNegotiateExit;
            }
        }
    }

    /* Get the AutoNeg Link partner base page */
    rc  = phyread(phyAddress, MII_ANLPAR, &phyAnalpar);
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

	//emac set speed
	if (bSP)// 100 Mbps
	{
		EMAC->EMAC_NCFGR |=  EMAC_NCFGR_SPD;
		DB(printk("100Mbps\n"));
	}
    else	// 10Mpbs
	{
		EMAC->EMAC_NCFGR &= ~EMAC_NCFGR_SPD;
		DB(printk("10Mbps\n"));
	}
	 
	if (bFD)// 100 base Tx, full duplex
	{
		EMAC->EMAC_NCFGR |=  EMAC_NCFGR_FD;
		DB(printk("full duplex.\n"));
	}
    else	// half duplex
	{
		EMAC->EMAC_NCFGR &= ~EMAC_NCFGR_FD;
		DB(printk("half duplex.\n"));
	}

	//disable rmii
	EMAC->EMAC_USRIO &= ~EMAC_USRIO_RMII;
	
    //enable the transeiver clock 
    EMAC->EMAC_USRIO |=  EMAC_USRIO_CLKEN;
	DB(printk("auto negotiate is finished.\n"));

AutoNegotiateExit: 
	EMAC->EMAC_NCR &= ~EMAC_NCR_MPE;
    return rc;
}

/**
 * Get the Link & speed settings, and automatically setup the EMAC with the
 * settings.
 * Return 1 if link found, 0 if no ethernet link. 
 * \param applySetting Apply the settings to EMAC interface
 */
uint8_t getLinkSpeed(uint8_t applySetting)
{
    uint32_t stat1;
    uint32_t stat2;
    uint8_t phyAddress, bSP, bFD;
    uint8_t rc = 1;

    EMAC->EMAC_NCR |= EMAC_NCR_MPE;

    phyAddress = PHY_ADDRESS;

    rc  = phyread(phyAddress, MII_BMSR, &stat1);
    if (rc == 0)
    {
        goto GetLinkSpeedExit;
    }

	printk("MII_BMSR:%X\n", stat1);
    if ((stat1 & MII_LINK_STATUS) == 0)
    {
        rc = 0;
		printk("no link?!\n");
        goto GetLinkSpeedExit;
    }

    if (applySetting == 0)
    {
        goto GetLinkSpeedExit;
    }

    /* Re-configure Link speed */
    rc  = phyread(phyAddress, MII_LBREMR, &stat2);
    if (rc == 0)
    {
        goto GetLinkSpeedExit;
    }
    if ((stat1 & MII_100BASE_TX_FD) && (stat2 & MII_100FDX))
    {
        /* set Emac for 100BaseTX and Full Duplex */
        bSP = 1; bFD = 1;
    }

    if ((stat1 & MII_10BASE_T_FD) && (stat2 & MII_10FDX))
    {
        /* set MII for 10BaseT and Full Duplex */
        bSP= 0; bFD = 1;
    }

    if ((stat1 & MII_100BASE_T4_HD) && (stat2 & MII_100HDX))
    {
        /* set MII for 100BaseTX and Half Duplex */
        bSP = 1; bFD = 0;
    }

    if ((stat1 & MII_10BASE_T_HD) && (stat2 & MII_10HDX))
    {
        /* set MII for 10BaseT and Half Duplex */
        bSP = 0; bFD = 0;
    }

	//emac set speed
	if (bSP)// 100 Mbps
	{
		EMAC->EMAC_NCFGR |=  EMAC_NCFGR_SPD;
		printk("100Mbps\n");
	}
    else	// 10Mpbs
	{
		EMAC->EMAC_NCFGR &= ~EMAC_NCFGR_SPD;
		printk("10Mbps\n");
	}
	 
	if (bFD)// 100 base Tx, full duplex
	{
		EMAC->EMAC_NCFGR |=  EMAC_NCFGR_FD;
		printk("full duplex.\n");
	}
    else	// half duplex
	{
		EMAC->EMAC_NCFGR &= ~EMAC_NCFGR_FD;
		printk("half duplex.\n");
	}

GetLinkSpeedExit:
	EMAC->EMAC_NCR &= ~EMAC_NCR_MPE;
    return rc;
}






