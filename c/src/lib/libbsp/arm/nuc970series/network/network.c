/*

NUC970 Series Network driver
bacon@rtems.cn
*/



#include <rtems.h>
#include <rtems/rtems_bsdnet.h>
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
#include <nuc970.h>




#define NUM_NUC970_MACS			1


/* Generic MII registers. */

#define MII_BMCR            0x00        /* Basic mode control register */
#define MII_BMSR            0x01        /* Basic mode status register  */
#define MII_PHYSID1         0x02        /* PHYS ID 1                   */
#define MII_PHYSID2         0x03        /* PHYS ID 2                   */
#define MII_ADVERTISE       0x04        /* Advertisement control reg   */
#define MII_LPA             0x05        /* Link partner ability reg    */
#define MII_EXPANSION       0x06        /* Expansion register          */
#define MII_DCOUNTER        0x12        /* Disconnect counter          */
#define MII_FCSCOUNTER      0x13        /* False carrier counter       */
#define MII_NWAYTEST        0x14        /* N-way auto-neg test reg     */
#define MII_RERRCOUNTER     0x15        /* Receive error counter       */
#define MII_SREVISION       0x16        /* Silicon revision            */
#define MII_RESV1           0x17        /* Reserved...                 */
#define MII_LBRERROR        0x18        /* Lpback, rx, bypass error    */
#define MII_PHYADDR         0x19        /* PHY address                 */
#define MII_RESV2           0x1a        /* Reserved...                 */
#define MII_TPISTATUS       0x1b        /* TPI status for 10mbps       */
#define MII_NCONFIG         0x1c        /* Network interface config    */

/* Basic mode control register. */
#define BMCR_RESV               0x007f  /* Unused...                   */
#define BMCR_CTST               0x0080  /* Collision test              */
#define BMCR_FULLDPLX           0x0100  /* Full duplex                 */
#define BMCR_ANRESTART          0x0200  /* Auto negotiation restart    */
#define BMCR_ISOLATE            0x0400  /* Disconnect DP83840 from MII */
#define BMCR_PDOWN              0x0800  /* Powerdown the DP83840       */
#define BMCR_ANENABLE           0x1000  /* Enable auto negotiation     */
#define BMCR_SPEED100           0x2000  /* Select 100Mbps              */
#define BMCR_LOOPBACK           0x4000  /* TXD loopback bits           */
#define BMCR_RESET              0x8000  /* Reset the DP83840           */

/* Basic mode status register. */
#define BMSR_ERCAP              0x0001  /* Ext-reg capability          */
#define BMSR_JCD                0x0002  /* Jabber detected             */
#define BMSR_LSTATUS            0x0004  /* Link status                 */
#define BMSR_ANEGCAPABLE        0x0008  /* Able to do auto-negotiation */
#define BMSR_RFAULT             0x0010  /* Remote fault detected       */
#define BMSR_ANEGCOMPLETE       0x0020  /* Auto-negotiation complete   */
#define BMSR_RESV               0x07c0  /* Unused...                   */
#define BMSR_10HALF             0x0800  /* Can do 10mbps, half-duplex  */
#define BMSR_10FULL             0x1000  /* Can do 10mbps, full-duplex  */
#define BMSR_100HALF            0x2000  /* Can do 100mbps, half-duplex */
#define BMSR_100FULL            0x4000  /* Can do 100mbps, full-duplex */
#define BMSR_100BASE4           0x8000  /* Can do 100mbps, 4k packets  */

/* Advertisement control register. */
#define ADVERTISE_SLCT          0x001f  /* Selector bits               */
#define ADVERTISE_CSMA          0x0001  /* Only selector supported     */
#define ADVERTISE_10HALF        0x0020  /* Try for 10mbps half-duplex  */
#define ADVERTISE_10FULL        0x0040  /* Try for 10mbps full-duplex  */
#define ADVERTISE_100HALF       0x0080  /* Try for 100mbps half-duplex */
#define ADVERTISE_100FULL       0x0100  /* Try for 100mbps full-duplex */
#define ADVERTISE_100BASE4      0x0200  /* Try for 100mbps 4k packets  */
#define ADVERTISE_RESV          0x1c00  /* Unused...                   */
#define ADVERTISE_RFAULT        0x2000  /* Say we can detect faults    */
#define ADVERTISE_LPACK         0x4000  /* Ack link partners response  */
#define ADVERTISE_NPAGE         0x8000  /* Next page bit               */

#define RX_DESCRIPTOR_NUM 8    // Max Number of Rx Frame Descriptors
#define TX_DESCRIPTOR_NUM 1    	// Max number of Tx Frame Descriptors
#define ETH_PKT_BUFF_SIZE	1600
#define PACKET_BUFFER_SIZE  1520

#define CONFIG_PHY_ADDR     1


// Frame Descriptor's Owner bit
#define OWNERSHIP_EMAC 0x80000000  // 1 = EMAC
//#define OWNERSHIP_CPU 0x7fffffff  // 0 = CPU



// Rx Frame Descriptor Status
#define RXFD_RXGD    0x00100000  // Receiving Good Packet Received
#define RXFD_RTSAS   0x00800000  // RX Time Stamp Available 


// Tx Frame Descriptor's Control bits
#define TXFD_TTSEN    0x08    // Tx Time Stamp Enable
#define TXFD_INTEN    0x04    // Interrupt Enable
#define TXFD_CRCAPP   0x02    // Append CRC
#define TXFD_PADEN    0x01    // Padding Enable

// Tx Frame Descriptor Status
#define TXFD_TXCP    0x00080000  // Transmission Completion
#define TXFD_TTSAS   0x08000000  // TX Time Stamp Available


#define ETH0_TRIGGER_RX()    outpw(REG_EMAC0_RSDR, 0)
#define ETH0_TRIGGER_TX()    outpw(REG_EMAC0_TSDR, 0)
#define ETH0_ENABLE_TX()     outpw(REG_EMAC0_MCMDR, inpw(REG_EMAC0_MCMDR) | 0x100)
#define ETH0_ENABLE_RX()     outpw(REG_EMAC0_MCMDR, inpw(REG_EMAC0_MCMDR) | 0x1)
#define ETH0_DISABLE_TX()    outpw(REG_EMAC0_MCMDR, inpw(REG_EMAC0_MCMDR) & ~0x100)
#define ETH0_DISABLE_RX()    outpw(REG_EMAC0_MCMDR, inpw(REG_EMAC0_MCMDR) & ~0x1)



/*  RTEMS event used by interrupt handler to start receive daemon. */
#define START_RECEIVE_EVENT  	RTEMS_EVENT_1

/* RTEMS event used to start transmit daemon. */
#define START_TRANSMIT_EVENT    RTEMS_EVENT_2

// Tx/Rx buffer descriptor structure
struct eth_descriptor;
struct eth_descriptor {
    unsigned int  status1;
    unsigned char *buf;
    unsigned int  status2;
    struct eth_descriptor *next;
};

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

	volatile struct eth_descriptor			*cur_tx_desc_ptr;
	volatile struct eth_descriptor			*cur_rx_desc_ptr;
	volatile struct eth_descriptor			*fin_tx_desc_ptr;

 
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
} nuc970_emac_softc_t;

static nuc970_emac_softc_t softc[NUM_NUC970_MACS];

#define __ALIGNED(size)		__attribute__ ((aligned(size)))

static struct eth_descriptor rx_desc[RX_DESCRIPTOR_NUM] __ALIGNED(32);
static struct eth_descriptor tx_desc[TX_DESCRIPTOR_NUM] __ALIGNED(32);
static uint8_t eth_rx_buff[RX_DESCRIPTOR_NUM * NUM_NUC970_MACS][PACKET_BUFFER_SIZE] __ALIGNED(32);
static uint8_t eth_tx_buff[TX_DESCRIPTOR_NUM * NUM_NUC970_MACS][PACKET_BUFFER_SIZE] __ALIGNED(32);


static void mdio_write(uint8_t addr, uint8_t reg, uint16_t val);
static uint16_t mdio_read(uint8_t addr, uint8_t reg);
static int reset_phy(void);

static void init_tx_desc(nuc970_emac_softc_t *sc);
static void init_rx_desc(nuc970_emac_softc_t *sc);
static void set_mac_addr(uint8_t *addr);
static void ETH0_halt(void);
static void nuc970_emac_rx_isr (void * arg);
static void nuc970_emac_tx_isr (void * arg);
static void nuc970_emac_init(void *arg);
static int nuc970_emac_ioctl (struct ifnet *ifp, ioctl_command_t command, caddr_t data);
static void nuc970_emac_start(struct ifnet *ifp);
void nuc970_emac_rxDaemon(void *arg);
void nuc970_emac_txDaemon (void *arg);


int rtems_nuc970_emac_attach (
    struct rtems_bsdnet_ifconfig *config,
    void *chip  /* only one ethernet, so no chip number */
    )
{
    struct ifnet *ifp;
    int mtu;
    int unitnumber;
    char *unitname;
	nuc970_emac_softc_t *sc;

    /*
     * Parse driver name
     */
    if ((unitnumber = rtems_bsdnet_parse_driver_name (config, &unitname)) < 0)
        return 0;

    /*
     * Is driver free?
     */
    if (unitnumber >= NUM_NUC970_MACS) {
        printk ("Bad nuc970 EMAC unit number.\n");
        return 0;
    }


	sc = &softc[0];
	
    ifp = &sc->arpcom.ac_if;
    if (ifp->if_softc != NULL) {
        printk ("Driver already in use.\n");
        return 0;
    }


    /*
     *  zero out the control structure
     */
    memset(sc, 0, sizeof(nuc970_emac_softc_t) );

    /* get the MAC address from the chip */
    sc->arpcom.ac_enaddr[0] = 0x00 + unitnumber;
    sc->arpcom.ac_enaddr[1] = 0x12;
    sc->arpcom.ac_enaddr[2] = 0x34;
    sc->arpcom.ac_enaddr[3] = 0x56;
    sc->arpcom.ac_enaddr[4] = 0x78;
    sc->arpcom.ac_enaddr[5] = 0x9a;

    if (config->mtu) {
        mtu = config->mtu;
    } else {
        mtu = ETHERMTU;
    }

    sc->acceptBroadcast = !config->ignore_broadcast;

    /*
     * Set up network interface values
     */
    ifp->if_softc = sc;
    ifp->if_unit = unitnumber;
    ifp->if_name = unitname;
    ifp->if_mtu = mtu;
    ifp->if_init = nuc970_emac_init;
    ifp->if_ioctl = nuc970_emac_ioctl;
    ifp->if_start = nuc970_emac_start;
    ifp->if_output = ether_output;
    ifp->if_flags = IFF_BROADCAST;
    if (ifp->if_snd.ifq_maxlen == 0) {
        ifp->if_snd.ifq_maxlen = ifqmaxlen;
    }

    /*
     * Attach the interface
     */
    if_attach (ifp);
    ether_ifattach (ifp);
    return 1;
}


void nuc970_emac_start(struct ifnet *ifp)
{
    nuc970_emac_softc_t *sc = ifp->if_softc;

	/*
	enable tx, rx
	*/
	outpw(REG_EMAC0_MCMDR, inpw(REG_EMAC0_MCMDR) | 0x101);

    rtems_bsdnet_event_send(sc->txDaemonTid, START_TRANSMIT_EVENT);
    ifp->if_flags |= IFF_OACTIVE;

	//ETH0_TRIGGER_TX();
}

void nuc970_emac_stop (nuc970_emac_softc_t *sc)
{

    struct ifnet *ifp = &sc->arpcom.ac_if;

    ifp->if_flags &= ~IFF_RUNNING;

    /*
     * Stop the transmitter and receiver.
     */
	ETH0_halt();
}


void nuc970_emac_init(void *arg)
{

    nuc970_emac_softc_t     *sc = arg;
    struct ifnet *ifp = &sc->arpcom.ac_if;
    rtems_status_code status = RTEMS_SUCCESSFUL;

    /*
     *This is for stuff that only gets done once (at91rm9200_emac_init()
     * gets called multiple times
     */
    if (sc->txDaemonTid == 0) {
        // Reset MAC
    	outpw(REG_EMAC0_MCMDR, 0x1000000);
        outpw(REG_CLK_HCLKEN, inpw(REG_CLK_HCLKEN) & ~(1 << 16));            // EMAC0 clk
        /* Set up EMAC hardware */
        outpw(REG_CLK_HCLKEN, inpw(REG_CLK_HCLKEN) | (1 << 16));            // EMAC0 clk
		outpw(REG_CLK_DIVCTL8, (inpw(REG_CLK_DIVCTL8) & ~0xFF) | 0xA0);     // MDC clk divider
		
		// Multi function pin setting
		outpw(REG_SYS_GPF_MFPL, 0x11111111);
		outpw(REG_SYS_GPF_MFPH, (inpw(REG_SYS_GPF_MFPH) & ~0xFF) | 0x11);

		// Reset MAC
    	outpw(REG_EMAC0_MCMDR, 0x1000000);

		init_tx_desc(sc);
    	init_rx_desc(sc);


		set_mac_addr(sc->arpcom.ac_enaddr);  // need to reconfigure hardware address 'cos we just RESET emc...
    	reset_phy();

    	outpw(REG_EMAC0_MCMDR, inpw(REG_EMAC0_MCMDR) | 0x121); // strip CRC, TX on, Rx on
    	outpw(REG_EMAC0_MIEN, inpw(REG_EMAC0_MIEN) | 0x01250C11);  // Except tx/rx ok, enable rdu, txabt, tx/rx bus error.

        /*      Start driver tasks */
        sc->rxDaemonTid = rtems_bsdnet_newproc("ENrx",
                                               4096,
                                               nuc970_emac_rxDaemon,
                                               sc);
        sc->txDaemonTid = rtems_bsdnet_newproc("ENtx",
                                               4096,
                                               nuc970_emac_txDaemon,
                                               sc);
    } /* if txDaemonTid */

    /* install the interrupt handler */
    status = rtems_interrupt_handler_install(
        EMC0_RX_IRQn,
        "NWRX",
        RTEMS_INTERRUPT_UNIQUE,
        nuc970_emac_rx_isr,
        arg
    );

	status = rtems_interrupt_handler_install(
        EMC0_TX_IRQn,
        "NWTX",
        RTEMS_INTERRUPT_UNIQUE,
        nuc970_emac_tx_isr,
        arg
    );

	/*
	enable tx, rx
	*/
	//outpw(REG_EMAC0_MCMDR, inpw(REG_EMAC0_MCMDR) | 0x101);

	ETH0_TRIGGER_RX();
	//ETH0_TRIGGER_TX();

    /* EMAC doesn't support promiscuous, so ignore requests */
    if (ifp->if_flags & IFF_PROMISC) {
        printk ("Warning - NUC977 Ethernet driver"
                " doesn't support Promiscuous Mode!\n");
    }

    /*
     * Tell the world that we're running.
     */
    ifp->if_flags |= IFF_RUNNING;
} /* nuc970_emac_init() */

/*
 * Driver transmit daemon
 */
void nuc970_emac_txDaemon (void *arg)
{
    nuc970_emac_softc_t *sc = (nuc970_emac_softc_t *)arg;
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
		//printk("%s:(0x%x)\n\r", __func__, sc->ethDrv.pHw->EMAC_IMR);
        if (sc->cur_tx_desc_ptr->status1 & OWNERSHIP_EMAC)
            continue;
        
		IF_DEQUEUE(&ifp->if_snd, m);
		if (m)
        {
			nuc970_emac_sendpacket(sc, ifp, m);
        }



#endif
        ifp->if_flags &= ~IFF_OACTIVE;
    }
}

void nuc970_emac_sendpacket (nuc970_emac_softc_t *sc, struct ifnet *ifp, struct mbuf *m)
{
	struct eth_descriptor volatile *desc;
	struct mbuf *l = NULL;
    unsigned int pkt_offset = 0;
	uint8_t *buff;


	l = m;
	buff = (uint8_t *) sc->cur_tx_desc_ptr->buf;
    while (l != NULL) {
        memcpy(((char *)buff + pkt_offset),  /* offset into pkt for mbuf */
               (char *)mtod(l, void *),       /* cast to void */
               l->m_len);                     /* length of this mbuf */

        pkt_offset += l->m_len;               /* update offset */
        l = l->m_next;                        /* get next mbuf, if any */
    }
    {
        int i ;
        printk("\r\n");
        for (i = 0; i < pkt_offset; i++)
            printk("%02x ", buff[i]);
        printk("\r\n");
    }

    /* free the mbuf chain we just copied */
    m_freem(m);

    if ((sc->cur_tx_desc_ptr->status1 & OWNERSHIP_EMAC) == OWNERSHIP_EMAC) 
        printk("#");
    else
        printk("@");

	sc->cur_tx_desc_ptr->status2 = (unsigned int)pkt_offset;
	desc = sc->cur_tx_desc_ptr->next;    // in case TX is transmitting and overwrite next pointer before we can update cur_tx_desc_ptr
	sc->cur_tx_desc_ptr->status1 = OWNERSHIP_EMAC |  TXFD_CRCAPP | TXFD_PADEN |   TXFD_INTEN;
	sc->cur_tx_desc_ptr = desc;


	ETH0_TRIGGER_TX();
}


void nuc970_emac_rxDaemon(void *arg)
{
    nuc970_emac_softc_t *sc = (nuc970_emac_softc_t *)arg;
    struct ifnet *ifp = &sc->arpcom.ac_if;
    struct mbuf *m;
    struct ether_header eh;
    rtems_event_set events;
    uint32_t pktlen;
	uint32_t status;
	
    /* Input packet handling loop */
    for (;;) {
        rtems_bsdnet_event_receive(
            START_RECEIVE_EVENT,
            RTEMS_EVENT_ANY | RTEMS_WAIT,
            RTEMS_NO_TIMEOUT,
            &events);
		
		//tr("start receive...\n\r");
		do {
			status = sc->cur_rx_desc_ptr->status1;
			//printk(".");

			if (status & OWNERSHIP_EMAC)
			{
				//printk("X");
				break;
			}

			if (status & RXFD_RXGD) {

				//ethernetif_input0(status & 0xFFFF, sc->cur_rx_desc_ptr->buf);
				/* get an mbuf this packet */
				MGETHDR(m, M_WAIT, MT_DATA);

				/* now get a cluster pointed to by the mbuf */
				/* since an mbuf by itself is too small */
				MCLGET(m, M_WAIT);

				/* set the type of mbuf to ifp (ethernet I/F) */
				m->m_pkthdr.rcvif = ifp;
				m->m_nextpkt = 0;

				pktlen = status & 0xFFFF;

				memcpy(&eh, sc->cur_rx_desc_ptr->buf, sizeof(struct ether_header));
				memcpy(m->m_ext.ext_buf,
						sc->cur_rx_desc_ptr->buf + sizeof(struct ether_header), 
						pktlen - sizeof(struct ether_header));
                
                {
                    int i ;
                    printk("rx:\r\n");
                    for (i = 0; i < pktlen; i++)
                        printk("%02x ", sc->cur_rx_desc_ptr->buf[i]);
                    printk("\r\n");
                }

				/* give all this stuff to the stack */
				ether_input(ifp, &eh, m);
			}

			sc->cur_rx_desc_ptr->status1 = OWNERSHIP_EMAC;
			sc->cur_rx_desc_ptr = sc->cur_rx_desc_ptr->next;
		} while (1);
		ETH0_TRIGGER_RX();
    } /* for (;;) */
} /* nuc970_emac_rxDaemon() */

/* Show interface statistics */
void nuc970_emac_stats (nuc970_emac_softc_t *sc)
{
    printf (" Total Interrupts:%-8lu",          sc->Interrupts);
    printf ("    Rx Interrupts:%-8lu",          sc->rxInterrupts);
    printf ("            Giant:%-8lu",          sc->rxGiant);
    printf ("        Non-octet:%-8lu\n",        sc->rxNonOctet);
    printf ("          Bad CRC:%-8lu",          sc->rxBadCRC);
    printf ("        Collision:%-8lu",          sc->rxCollision);
    printf ("           Missed:%-8lu\n",        sc->rxMissed);

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
nuc970_emac_ioctl (struct ifnet *ifp, ioctl_command_t command, caddr_t data)
{
    nuc970_emac_softc_t *sc = ifp->if_softc;
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
            nuc970_emac_stop (sc);
            break;

        case IFF_UP:
            nuc970_emac_init (sc);
            break;

        case IFF_UP | IFF_RUNNING:
            nuc970_emac_stop (sc);
            nuc970_emac_init (sc);
            break;

        default:
            break;
        } /* switch (if_flags) */
        break;

    case SIO_RTEMS_SHOW_STATS:
        nuc970_emac_stats (sc);
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
static void nuc970_emac_rx_isr (void * arg)
{
	nuc970_emac_softc_t *sc = (nuc970_emac_softc_t*) arg;
	unsigned int status;

    status = inpw(REG_EMAC0_MISTA) & 0xFFFF;
    outpw(REG_EMAC0_MISTA, status);
    
    if (status & 0x800) {
        // Shouldn't goes here, unless descriptor corrupted
    }

#if 0
    do {
        status = sc->cur_rx_desc_ptr->status1;

        if(status & OWNERSHIP_EMAC)
            break;

        if (status & RXFD_RXGD) {

            //ethernetif_input0(status & 0xFFFF, sc->cur_rx_desc_ptr->buf);
			

        }

        sc->cur_rx_desc_ptr->status1 = OWNERSHIP_EMAC;
        sc->cur_rx_desc_ptr = sc->cur_rx_desc_ptr->next;

    } while (1);

    ETH0_TRIGGER_RX();
#else
	rtems_bsdnet_event_send (sc->rxDaemonTid, START_RECEIVE_EVENT);
	//printk("<");
#endif

}

static void nuc970_emac_tx_isr (void * arg)
{
	nuc970_emac_softc_t *sc = (nuc970_emac_softc_t*) arg;

	unsigned int cur_entry, status;

    status = inpw(REG_EMAC0_MISTA) & 0xFFFF0000;
    outpw(REG_EMAC0_MISTA, status);

    if(status & 0x1000000) {
        // Shouldn't goes here, unless descriptor corrupted
        return;
    }

    cur_entry = inpw(REG_EMAC0_CTXDSA);

    while (cur_entry != (uint32_t)sc->fin_tx_desc_ptr) {

        sc->fin_tx_desc_ptr = sc->fin_tx_desc_ptr->next;
    }
	rtems_bsdnet_event_send (sc->txDaemonTid, START_TRANSMIT_EVENT);
	printk(">");
}


/*
Internal function
*/

static void mdio_write(uint8_t addr, uint8_t reg, uint16_t val)
{ 
    
    outpw(REG_EMAC0_MIID, val);
    outpw(REG_EMAC0_MIIDA, (addr << 8) | reg | 0xB0000);

    while (inpw(REG_EMAC0_MIIDA) & 0x20000);    // wait busy flag clear

}


static uint16_t mdio_read(uint8_t addr, uint8_t reg)
{
    outpw(REG_EMAC0_MIIDA, (addr << 8) | reg | 0xA0000);
    while (inpw(REG_EMAC0_MIIDA) & 0x20000);    // wait busy flag clear

    return inpw(REG_EMAC0_MIID);
}



static int reset_phy(void)
{

    uint16_t reg;
    uint32_t delay;


    mdio_write(CONFIG_PHY_ADDR, MII_BMCR, BMCR_RESET);

    delay = 2000;
    while(delay-- > 0) {
        if((mdio_read(CONFIG_PHY_ADDR, MII_BMCR) & BMCR_RESET) == 0)
            break;

    }

    if(delay == 0) {
        printk("Reset phy failed\n");
        return(-1);
    }

    mdio_write(CONFIG_PHY_ADDR, MII_ADVERTISE, ADVERTISE_CSMA |
               ADVERTISE_10HALF |
               ADVERTISE_10FULL |
               ADVERTISE_100HALF |
               ADVERTISE_100FULL);

    reg = mdio_read(CONFIG_PHY_ADDR, MII_BMCR);
    mdio_write(CONFIG_PHY_ADDR, MII_BMCR, reg | BMCR_ANRESTART);

    delay = 200000;
    while(delay-- > 0) {
        if((mdio_read(CONFIG_PHY_ADDR, MII_BMSR) & (BMSR_ANEGCOMPLETE | BMSR_LSTATUS))
                == (BMSR_ANEGCOMPLETE | BMSR_LSTATUS))
            break;
    }

    if(delay == 0) {
        printk("AN failed. Set to 100 FULL\n");
        outpw(REG_EMAC0_MCMDR, inpw(REG_EMAC0_MCMDR) | 0x140000);
        //plugged = 0;
        return(-1);
    } else {
        reg = mdio_read(CONFIG_PHY_ADDR, MII_LPA);
        //plugged = 1;

        if(reg & ADVERTISE_100FULL) {
            printk("100 full\n");
            outpw(REG_EMAC0_MCMDR, inpw(REG_EMAC0_MCMDR) | 0x140000);
        } else if(reg & ADVERTISE_100HALF) {
            printk("100 half\n");
            outpw(REG_EMAC0_MCMDR, (inpw(REG_EMAC0_MCMDR) & ~0x40000) | 0x100000);
        } else if(reg & ADVERTISE_10FULL) {
            printk("10 full\n");
            outpw(REG_EMAC0_MCMDR, (inpw(REG_EMAC0_MCMDR) & ~0x100000) | 0x40000);
        } else {
            printk("10 half\n");
            outpw(REG_EMAC0_MCMDR, inpw(REG_EMAC0_MCMDR) & ~0x140000);
        }
    }

    return(0);
}


static void init_tx_desc(nuc970_emac_softc_t *sc)
{
    uint32_t i;
	const uint32_t offset = 0x0; //0x80000000

    sc->cur_tx_desc_ptr = sc->fin_tx_desc_ptr = (struct eth_descriptor *)((uint32_t)(&tx_desc[0]) | offset);
	
    for(i = 0; i < TX_DESCRIPTOR_NUM; i++) {
        tx_desc[i].status1 = TXFD_PADEN | TXFD_CRCAPP | TXFD_INTEN;
        tx_desc[i].buf = (unsigned char *)((uint32_t)(&eth_tx_buff[i][0]) | offset);
        tx_desc[i].status2 = 0;
        tx_desc[i].next = (struct eth_descriptor *)((uint32_t)(&tx_desc[(i + 1) % TX_DESCRIPTOR_NUM]) | offset);
    }
    outpw(REG_EMAC0_TXDLSA, (unsigned int)&tx_desc[0] | offset);
    return;
}

static void init_rx_desc(nuc970_emac_softc_t *sc)
{
    uint32_t i;
	const uint32_t offset = 0x0; //0x80000000

    sc->cur_rx_desc_ptr = (struct eth_descriptor *)((uint32_t)(&rx_desc[0]) | offset);

    for(i = 0; i < RX_DESCRIPTOR_NUM; i++) {
        rx_desc[i].status1 = OWNERSHIP_EMAC;
        rx_desc[i].buf = (unsigned char *)((uint32_t)(&eth_rx_buff[i][0]) | offset);
        rx_desc[i].status2 = 0;
        rx_desc[i].next = (struct eth_descriptor *)((uint32_t)(&rx_desc[(i + 1) % RX_DESCRIPTOR_NUM]) | offset);
    }
    outpw(REG_EMAC0_RXDLSA, (unsigned int)&rx_desc[0] | offset);
    return;
}

static void set_mac_addr(uint8_t *addr)
{

    outpw(REG_EMAC0_CAMxM_Reg(0), (addr[0] << 24) |
                                  (addr[1] << 16) |
                                  (addr[2] << 8) |
                                  addr[3]);
    outpw(REG_EMAC0_CAMxL_Reg(0), (addr[4] << 24) |
                                  (addr[5] << 16));
    outpw(REG_EMAC0_CAMCMR, 0x16);
    outpw(REG_EMAC0_CAMEN, 1);    // Enable CAM entry 0

}


static void ETH0_halt(void)
{

    outpw(REG_EMAC0_MCMDR, inpw(REG_EMAC0_MCMDR) & ~0x101); // disable tx/rx on
    
}

