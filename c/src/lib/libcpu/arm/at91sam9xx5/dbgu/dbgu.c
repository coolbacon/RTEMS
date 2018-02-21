/*
at91sam9x25
baconxu@gmail.com
 */
#include <bsp.h>
#include <rtems/libio.h>
#include <termios.h>

#include <at91sam9xx5.h>

#include <rtems/bspIo.h>
#include <libchip/serial.h>
#include <libchip/sersupp.h>

volatile int dbg_dly;
typedef Dbgu at91sam9xx5_dbgu_regs_t;

/* static function prototypes */
static int     dbgu_first_open(int major, int minor, void *arg);
static int     dbgu_last_close(int major, int minor, void *arg);
static int     dbgu_read(int minor);
static ssize_t dbgu_write(int minor, const char *buf, size_t len);
static void    dbgu_init(int minor);
static void    dbgu_write_polled(int minor, char c);
static int     dbgu_set_attributes(int minor, const struct termios *t);

/* Pointers to functions for handling the UART. */
const console_fns dbgu_fns =
{
    libchip_serial_default_probe,
    dbgu_first_open,
    dbgu_last_close,
    dbgu_read,
    dbgu_write,
    dbgu_init,
    dbgu_write_polled,   /* not used in this driver */
    dbgu_set_attributes,
    FALSE      /* TRUE if interrupt driven, FALSE if not. */
};
/*********************************************************************/
/* Functions called via callbacks (i.e. the ones in uart_fns */
/*********************************************************************/
/*
 * This is called the first time each device is opened. Since
 * the driver is polled, we don't have to do anything. If the driver
 * were interrupt driven, we'd enable interrupts here.
 */
static int dbgu_first_open(int major, int minor, void *arg)
{
    return 0;
}


/*
 * This is called the last time each device is closed.  Since
 * the driver is polled, we don't have to do anything. If the driver
 * were interrupt driven, we'd disable interrupts here.
 */
static int dbgu_last_close(int major, int minor, void *arg)
{
    return 0;
}


/*
 * Read one character from UART.
 *
 * return -1 if there's no data, otherwise return
 * the character in lowest 8 bits of returned int.
 */
static int dbgu_read(int minor)
{
    char c;
    console_tbl *console_entry;
    at91sam9xx5_dbgu_regs_t *dbgu;

    console_entry = BSP_get_uart_from_minor(minor);

    if (console_entry == NULL) {
        return -1;
    }

    dbgu = (at91sam9xx5_dbgu_regs_t *)console_entry->ulCtrlPort1;

    if (!(dbgu->DBGU_SR & DBGU_SR_RXRDY)) {
        return -1;
    }

    c = dbgu->DBGU_RHR & 0xff;

    return c;
}


/*
 * Write buffer to UART
 *
 * return 1 on success, -1 on error
 */
static ssize_t dbgu_write(int minor, const char *buf, size_t len)
{
    int i, x;
    char c;
    console_tbl *console_entry;
    at91sam9xx5_dbgu_regs_t *dbgu;

    console_entry = BSP_get_uart_from_minor(minor);

    if (console_entry == NULL) {
        return -1;
    }

    dbgu = (at91sam9xx5_dbgu_regs_t *)console_entry->ulCtrlPort1;

    for (i = 0; i < len; i++) {
        /* Wait for fifo to have room */
        while(1) {
            if (dbgu->DBGU_SR & DBGU_SR_TXEMPTY) {
                break;
            }
        }

        c = (char) buf[i];
        dbgu->DBGU_THR = c;

        /* the TXRDY flag does not seem to update right away (is this true?) */
        /* so we wait a bit before continuing */
        for (x = 0; x < 100; x++) {
            dbg_dly++; /* using a global so this doesn't get optimized out */
        }
    }

    return 1;
}


/* Set up the UART. */
static void dbgu_init(int minor)
{
    console_tbl *console_entry;
    at91sam9xx5_dbgu_regs_t *dbgu;

    console_entry = BSP_get_uart_from_minor(minor);

    if (console_entry == NULL) {
        return;
    }

    dbgu = (at91sam9xx5_dbgu_regs_t *)console_entry->ulCtrlPort1;
    /* Set port to no parity, no loopback */
    dbgu->DBGU_MR = DBGU_MR_CHMODE_NORM | DBGU_MR_PAR_NONE
                          | US_MR_CHRL_8_BIT;


	/* Reset and disable receiver & transmitter */
    dbgu->DBGU_CR = DBGU_CR_RSTRX | DBGU_CR_RSTTX;
    dbgu->DBGU_IDR = 0xFFFFFFFF;
    dbgu->DBGU_CR = DBGU_CR_RSTRX | DBGU_CR_RSTTX | DBGU_CR_RXDIS | DBGU_CR_TXDIS;
	
    /* Set the baud rate */
	dbgu->DBGU_BRGR = (at91sam9xx5_get_mck() / 16) / BSP_get_baud();

	/* Enable the DBGU */
    dbgu->DBGU_CR = DBGU_CR_RXEN | DBGU_CR_TXEN;
}

/* This is used for getchark support */
static void dbgu_write_polled(int minor, char c)
{
    dbgu_write(minor, &c, 1);
}

/* This is for setting baud rate, bits, etc. */
static int dbgu_set_attributes(int minor, const struct termios *t)
{
    return 0;
}

/***********************************************************************/
/*
 * The following functions are not used by TERMIOS, but other RTEMS
 * functions use them instead.
 */
/***********************************************************************/
/*
 * Read from UART. This is used in the exit code, and can't
 * rely on interrupts.
 */
static int dbgu_poll_read(int minor)
{
    return dbgu_read(minor);
}


/*
 * Write a character to the console. This is used by printk() and
 * maybe other low level functions. It should not use interrupts or any
 * RTEMS system calls. It needs to be very simple
 */
static void _BSP_put_char( char c ) {
  dbgu_write_polled(0, c);
  if ( c == '\n' )
    dbgu_write_polled(0, '\r');
}

BSP_output_char_function_type     BSP_output_char = _BSP_put_char;

static int _BSP_poll_char(void)
{
  return dbgu_poll_read(0);
}

BSP_polling_getchar_function_type BSP_poll_char = _BSP_poll_char;
