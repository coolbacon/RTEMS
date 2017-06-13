/*
 * at91sam9g25 
 * baconxu@gmail.com 
 */
#ifndef __AT91SAM9G25_GPIO_H__
#define __AT91SAM9G25_GPIO_H__

/*  The pin is controlled by the associated signal of peripheral A. */
#define PIO_PERIPH_A                0
/*  The pin is controlled by the associated signal of peripheral B. */
#define PIO_PERIPH_B                1
/*  The pin is controlled by the associated signal of peripheral C. */
#define PIO_PERIPH_C                2
/*  The pin is controlled by the associated signal of peripheral D. */
#define PIO_PERIPH_D                3
/*  The pin is an input. */
#define PIO_INPUT                   4
/*  The pin is an output and has a default level of 0. */
#define PIO_OUTPUT_0                5
/*  The pin is an output and has a default level of 1. */
#define PIO_OUTPUT_1                6

/*  Default pin configuration (no attribute). */
#define PIO_DEFAULT                 (0 << 0)
/*  The internal pin pull-up is active. */
#define PIO_PULLUP                  (1 << 0)
/*  The internal glitch filter is active. */
#define PIO_DEGLITCH                (1 << 1)
/*  The pin is open-drain. */
#define PIO_OPENDRAIN               (1 << 2)

/*  The internal debouncing filter is active. */
#define PIO_DEBOUNCE                (1 << 3)

/*  Enable additional interrupt modes. */
#define PIO_IT_AIME                 (1 << 4)

/*  Interrupt High Level/Rising Edge detection is active. */
#define PIO_IT_RE_OR_HL             (1 << 5)
/*  Interrupt Edge detection is active. */
#define PIO_IT_EDGE                 (1 << 6)

/*  Low level interrupt is active */
#define PIO_IT_LOW_LEVEL            (0               | 0 | PIO_IT_AIME)
/*  High level interrupt is active */
#define PIO_IT_HIGH_LEVEL           (PIO_IT_RE_OR_HL | 0 | PIO_IT_AIME)
/*  Falling edge interrupt is active */
#define PIO_IT_FALL_EDGE            (0               | PIO_IT_EDGE | PIO_IT_AIME)
/*  Rising edge interrupt is active */
#define PIO_IT_RISE_EDGE            (PIO_IT_RE_OR_HL | PIO_IT_EDGE | PIO_IT_AIME)

#define PIO_WPMR_WPEN_EN            ( 0x01     << 0 )

#define PIO_WPMR_WPEN_DIS           ( 0x00     << 0 )

#define PIO_WPMR_WPKEY_VALID        ( 0x50494F << 8 )




/*--------------------------------------------------------------------------------*/
/** List of all DBGU pin definitions. */

/** DBGU Monitor IO pin (detect any DBGU operation). */
#define PIN_DBGU_MON {PIO_PA9A_DRXD, PIOA, ID_PIOA, PIO_INPUT, PIO_IT_RISE_EDGE}
/** DBGU pin definition. */
#define PINS_DBGU   {PIO_PA9A_DRXD | PIO_PA10A_DTXD, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}

/** List of all USART pin definitions. */

/** USART0 TXD pin definition. */
#define PIN_USART0_TXD  {PIO_PA0A_TXD0, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
/** USART0 RXD pin definition. */
#define PIN_USART0_RXD  {PIO_PA1A_RXD0, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
/** USART0 RTS pin definition. */
#define PIN_USART0_RTS  {PIO_PA2A_RTS0, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
/** USART0 CTS pin definition. */
#define PIN_USART0_CTS  {PIO_PA3A_CTS0, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
/** USART0 SCK pin definition. */
#define PIN_USART0_SCK  {PIO_PA4A_SCK0, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}

/** USART1 TXD pin definition. */
#define PIN_USART1_TXD  {PIO_PA5A_TXD1, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
/** USART1 RXD pin definition. */
#define PIN_USART1_RXD  {PIO_PA6A_RXD1, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
/** USART1 RTS pin definition. */
#define PIN_USART1_RTS  {PIO_PC27C_RTS1, PIOC, ID_PIOC, PIO_PERIPH_C, PIO_DEFAULT}
/** USART1 CTS pin definition. */
#define PIN_USART1_CTS  {PIO_PC28C_CTS1, PIOC, ID_PIOC, PIO_PERIPH_C, PIO_DEFAULT}
/** USART1 SCK pin definition. */
#define PIN_USART1_SCK  {PIO_PC29C_SCK1, PIOC, ID_PIOC, PIO_PERIPH_C, PIO_DEFAULT}

/** USART2 TXD pin definition. */
#define PIN_USART2_TXD  {PIO_PA7A_TXD2, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
/** USART2 RXD pin definition. */
#define PIN_USART2_RXD  {PIO_PA8A_RXD2, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
/** USART2 RTS pin definition. */
#define PIN_USART2_RTS  {PIO_PB0B_RTS2, PIOB, ID_PIOB, PIO_PERIPH_B, PIO_DEFAULT}
/** USART2 CTS pin definition. */
#define PIN_USART2_CTS  {PIO_PB1B_CTS2, PIOB, ID_PIOB, PIO_PERIPH_B, PIO_DEFAULT}
/** USART2 SCK pin definition. */
#define PIN_USART2_SCK  {PIO_PB2B_SCK2, PIOB, ID_PIOB, PIO_PERIPH_B, PIO_DEFAULT}


/** List of all TWI pin definitions. */

/** TWI0 data pin */
#define PIN_TWI_TWD0   {PIO_PA30A_TWD0, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
/** TWI0 clock pin */
#define PIN_TWI_TWCK0  {PIO_PA31A_TWCK0, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
/** TWI0 pins */
#define PINS_TWI0      PIN_TWI_TWD0, PIN_TWI_TWCK0



/** TWI1 data pin */
#define PIN_TWI_TWD1   {PIO_PC0C_TWD1, PIOC, ID_PIOC, PIO_PERIPH_C, PIO_DEFAULT}
/** TWI1 clock pin */
#define PIN_TWI_TWCK1  {PIO_PC1C_TWCK1, PIOC, ID_PIOC, PIO_PERIPH_C, PIO_DEFAULT}
/** TWI1 pins */
#define PINS_TWI1      PIN_TWI_TWD1, PIN_TWI_TWCK1








/** List of all SPI pin definitions. */

/** SPI0 MISO pin definition. */
#define PIN_SPI0_MISO     {PIO_PA11A_SPI0_MISO, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
/** SPI0 MOSI pin definition. */
#define PIN_SPI0_MOSI     {PIO_PA12A_SPI0_MOSI, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
/** SPI0 SPCK pin definition. */
#define PIN_SPI0_SPCK     {PIO_PA13A_SPI0_SPCK, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
/** SPI0 chip select pin definition. */
#define PIN_SPI0_NPCS0    {PIO_PA14A_SPI0_NPCS0, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
/** List of SPI0 pin definitions (MISO, MOSI & SPCK). */
#define PINS_SPI0         PIN_SPI0_MISO, PIN_SPI0_MOSI, PIN_SPI0_SPCK

/** SPI1 MISO pin definition. */
#define PIN_SPI1_MISO     {PIO_PA21B_SPI1_MISO, PIOA, ID_PIOA, PIO_PERIPH_B, PIO_DEFAULT}
/** SPI1 MOSI pin definition. */
#define PIN_SPI1_MOSI     {PIO_PA22B_SPI1_MOSI, PIOA, ID_PIOA, PIO_PERIPH_B, PIO_DEFAULT}
/** SPI1 SPCK pin definition. */
#define PIN_SPI1_SPCK     {PIO_PA23B_SPI1_SPCK, PIOA, ID_PIOA, PIO_PERIPH_B, PIO_DEFAULT}
/** SPI1 chip select pin definition. */
#define PIN_SPI1_NPCS0    {PIO_PA8B_SPI1_NPCS0, PIOA, ID_PIOA, PIO_PERIPH_B, PIO_DEFAULT}
/** List of SPI1 pin definitions (MISO, MOSI & SPCK). */
#define PINS_SPI1         PIN_SPI1_MISO, PIN_SPI1_MOSI, PIN_SPI1_SPCK

/** List of all SSC pin definitions. */

/** SSC pin Transmitter Data (TD) */
#define PIN_SSC_TD        {PIO_PA26B_TD, PIOA, ID_PIOA, PIO_PERIPH_B, PIO_DEFAULT}
/** SSC pin Transmitter Clock (TK) */
#define PIN_SSC_TK        {PIO_PA24B_TK, PIOA, ID_PIOA, PIO_PERIPH_B, PIO_DEFAULT}
/** SSC pin Transmitter FrameSync (TF) */
#define PIN_SSC_TF        {PIO_PA25B_TF, PIOA, ID_PIOA, PIO_PERIPH_B, PIO_DEFAULT}
/** SSC pin RD */
#define PIN_SSC_RD        {PIO_PA27B_RD, PIOA, ID_PIOA, PIO_PERIPH_B, PIO_DEFAULT}
/** SSC pin RK */
#define PIN_SSC_RK        {PIO_PA28B_RK, PIOA, ID_PIOA, PIO_PERIPH_B, PIO_DEFAULT}
/** SSC pin RF */
#define PIN_SSC_RF        {PIO_PA29B_RF, PIOA, ID_PIOA, PIO_PERIPH_B, PIO_DEFAULT}
/** SSC pins definition for codec. */
#define PINS_SSC_CODEC    PIN_SSC_TD, PIN_SSC_TK, PIN_SSC_TF, \
                          PIN_SSC_RD, PIN_SSC_RK, PIN_SSC_RF



/** EMAC pin TXCK */
#define PIN_EMAC0_TXCK    {PIO_PB4A_ETXCK,PIOB, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT}
/** EMAC pin TX0 */
#define PIN_EMAC0_TX0     {PIO_PB9A_ETX0, PIOB, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT}
/** EMAC pin TX1 */
#define PIN_EMAC0_TX1     {PIO_PB10A_ETX1,PIOB, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT}
/** EMAC pin TX2 */
#define PIN_EMAC0_TX2     {PIO_PB11A_ETX2,PIOB, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT}
/** EMAC pin TX3 */
#define PIN_EMAC0_TX3     {PIO_PB12A_ETX3,PIOB, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT}
/** EMAC pin TXEN */
#define PIN_EMAC0_TXEN    {PIO_PB7A_ETXEN,PIOB, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT}
/** EMAC pin RX0 */
#define PIN_EMAC0_RXER    {PIO_PB2A_ERXER,PIOB, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT}
/** EMAC pin RXDV */
#define PIN_EMAC0_RXDV    {PIO_PB3A_ERXDV,PIOB, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT}
/** EMAC pin RX0 */
#define PIN_EMAC0_RX0     {PIO_PB0A_ERX0, PIOB, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT}
/** EMAC pin RX1 */
#define PIN_EMAC0_RX1     {PIO_PB1A_ERX1, PIOB, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT}
/** EMAC pin RX2 */
#define PIN_EMAC0_RX2     {PIO_PB13A_ERX2,PIOB, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT}
/** EMAC pin RX3 */
#define PIN_EMAC0_RX3     {PIO_PB14A_ERX3,PIOB, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT}

/** PHY pin MDC */
#define PIN_EMAC0_MDC     {PIO_PB6A_EMDC, PIOB, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT}
/** PHY pin MDIO */
#define PIN_EMAC0_MDIO    {PIO_PB5A_EMDIO,PIOB, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT}
/** PHY pin INTR */
#define PIN_EMAC0_INTR    {PIO_PB8A_ETXER,PIOB, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT}

#define PIN_EMAC0_ERXCK	  {PIO_PB15A_ERXCK, PIOB, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT}

#define PIN_EMAC0_ECRS    {PIO_PB16A_ECRS, PIOB, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT}

#define PIN_EMAC0_ECOL	  {PIO_PB17A_ECOL, PIOB, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT}

/** EMAC pins definition for MII */
#define PINS_EMAC0_MII  PIN_EMAC0_TXCK, PIN_EMAC0_TXEN, \
                        PIN_EMAC0_TX0, PIN_EMAC0_TX1, PIN_EMAC0_TX2, PIN_EMAC0_TX3, \
                        PIN_EMAC0_RXDV, PIN_EMAC0_RXER, \
                        PIN_EMAC0_RX0, PIN_EMAC0_RX1, PIN_EMAC0_RX2, PIN_EMAC0_RX3, \
                        PIN_EMAC0_MDC, PIN_EMAC0_MDIO, PIN_EMAC0_ERXCK, \
						PIN_EMAC0_ECRS, PIN_EMAC0_ECOL/*, PIN_EMAC0_INTR*/

/** EMAC pins definition for RMII */
#define PINS_EMAC0_RMII PIN_EMAC0_TXCK, PIN_EMAC0_TXEN, \
                        PIN_EMAC0_TX0, PIN_EMAC0_TX1, \
                        PIN_EMAC0_RX0, PIN_EMAC0_RX1, \
                        PIN_EMAC0_RXDV, PIN_EMAC0_RXER, \
                        PIN_EMAC0_MDC, PIN_EMAC0_MDIO/*, PIN_EMAC0_INTR*/





/*----------------------------------------------------------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif

/*
 *          Global Macros
 */

/**
 *  Calculates the size of an array of Pin instances. The array must be defined
 *  locally (i.e. not a pointer), otherwise the computation will not be correct.
 *  \param pPins  Local array of Pin instances.
 *  \return Number of elements in array.
 */
#define PIO_LISTSIZE(pPins)    (sizeof(pPins) / sizeof(Pin))

/*
 *         Global Types
 */


/*
 *  Describes the type and attribute of one PIO pin or a group of similar pins.
 *  The #type# field can have the following values:
 *     - PIO_PERIPH_A
 *     - PIO_PERIPH_B
 *     - PIO_OUTPUT_0
 *     - PIO_OUTPUT_1
 *     - PIO_INPUT
 *
 *  The #attribute# field is a bitmask that can either be set to PIO_DEFAULt,
 *  or combine (using bitwise OR '|') any number of the following constants:
 *     - PIO_PULLUP
 *     - PIO_DEGLITCH
 *     - PIO_DEBOUNCE
 *     - PIO_OPENDRAIN
 *     - PIO_IT_LOW_LEVEL
 *     - PIO_IT_HIGH_LEVEL
 *     - PIO_IT_FALL_EDGE
 *     - PIO_IT_RISE_EDGE
 */
typedef struct _Pin
{
    /*  Bitmask indicating which pin(s) to configure. */
    uint32_t mask;
    /*  Pointer to the PIO controller which has the pin(s). */
    Pio    *pio;
    /*  Peripheral ID of the PIO controller which has the pin(s). */
    uint8_t id;
    /*  Pin type. */
    uint8_t type;
    /*  Pin attribute. */
    uint8_t attribute;
} Pin ;

/*
 *         Global Access Macros
 */

/*
 *         Global Functions
 */

extern uint8_t PIO_Configure( const Pin *list, uint32_t size ) ;
extern void PIO_Set( const Pin *pin ) ;
extern void PIO_Clear( const Pin *pin ) ;
extern uint8_t PIO_Get( const Pin *pin ) ;
extern uint8_t PIO_GetOutputDataStatus( const Pin *pin ) ;
extern void PIO_SetDebounceFilter( const Pin *pin, uint32_t cuttoff );
extern void PIO_EnableWriteProtect( const Pin *pin );
extern void PIO_DisableWriteProtect( const Pin *pin );
extern uint32_t PIO_GetWriteProtectViolationInfo( const Pin * pin );



#endif
