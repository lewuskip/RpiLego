/**
 ******************************************************************************
 *
 * @file       pios_uart.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 *              Parts by Thorsten Klose (tk@midibox.org) (tk@midibox.org)
 * @brief      UDP commands. Inits UDPs, controls UDPs & Interupt handlers.
 * @see        The GNU Public License (GPL) Version 3
 * @defgroup   PIOS_UDP UDP Functions
 * @{
 *
 *****************************************************************************/


/* Project Includes */
#include <stdio.h>
#include <unistd.h>         //Used for UART
#include <fcntl.h>          //Used for UART
#include <termios.h>        //Used for UART

#include "pios.h"

#include <signal.h>
#include <pios_uart_rpi.h>
#include <pios_logs.h>

#define SAMPLE_PERIOD_MS 1000
#define TASK_PRIORITY       (tskIDLE_PRIORITY + 3)


/* Provide a COM driver */
static void PIOS_USART_ChangeBaud(uint32_t uart_id, uint32_t baud);
static void PIOS_USART_RegisterRxCallback(uint32_t usart_id, pios_com_callback rx_in_cb, uint32_t context);
static void PIOS_USART_RegisterTxCallback(uint32_t usart_id, pios_com_callback tx_out_cb, uint32_t context);
static void PIOS_USART_TxStart(uint32_t usart_id, uint16_t tx_bytes_avail);
static void PIOS_USART_RxStart(uint32_t usart_id, uint16_t rx_bytes_avail);
static void *PIOS_USART_RxThread(uint32_t *uart_id);
static void PIOS_USART_Flush(uint32_t *uart_id);

const struct pios_com_driver pios_usart_com_driver = {
    .set_baud   = PIOS_USART_ChangeBaud,
    .tx_start   = PIOS_USART_TxStart,
    .rx_start   = PIOS_USART_RxStart,
    .bind_tx_cb = PIOS_USART_RegisterTxCallback,
    .bind_rx_cb = PIOS_USART_RegisterRxCallback,
};

#define PIOS_USART_RX_BUFFER_SIZE     1024
#define PIOS_USART_TX_BUFFER_SIZE     32

struct pios_usart_dev {
    const struct USART_Rpi_config   *cfg;

    xTaskHandle                     rxThread;

    int                             fd;
    pios_com_callback               tx_out_cb;
    uint32_t                        tx_out_context;
    pios_com_callback               rx_in_cb;
    uint32_t                        rx_in_context;

    int                             rx_cnt;
    int                             tx_cnt;

    uint8_t                         rx_buffer[PIOS_USART_RX_BUFFER_SIZE];
    uint8_t                         tx_buffer[PIOS_USART_TX_BUFFER_SIZE];

    uint32_t                        guard;
}pios_usart_dev;

static struct pios_usart_dev *PIOS_USART_alloc(void)
{
    struct pios_usart_dev *usart_dev;

    usart_dev = (struct pios_usart_dev *)pvPortMalloc(sizeof(struct pios_usart_dev));
    if (!usart_dev) {
        return NULL;
    }

    memset(usart_dev, 0, sizeof(struct pios_usart_dev));
    return usart_dev;
}

int PIOS_USART_Init(uint32_t *uart_id, const struct usart_rpi_config_t *cfg)
{
    struct pios_usart_dev *usart_dev;
    PIOS_DEBUG_Assert(usart_id);
    PIOS_DEBUG_Assert(cfg);

    usart_dev = (struct pios_usart_dev *)PIOS_USART_alloc();
    if (!usart_dev) {
        goto out_fail;
    }

    /* Open device */
    usart_dev->fd = open(cfg->devname, O_RDWR | O_NOCTTY | O_NDELAY);

    if (usart_dev->fd == -1) {
        goto out_fail;
      }

    *uart_id = (uint32_t)usart_dev;

    //CONFIGURE THE UART
    //The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
    //  Baud rate:- B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800, B500000, B576000, B921600, B1000000, B1152000, B1500000, B2000000, B2500000, B3000000, B3500000, B4000000
    //  CSIZE:- CS5, CS6, CS7, CS8
    //  CLOCAL - Ignore modem status lines
    //  CREAD - Enable receiver
    //  IGNPAR = Ignore characters with parity errors
    //  ICRNL - Map CR to NL on input (Use for ASCII comms where you want to auto correct end of line characters - don't use for bianry comms!)
    //  PARENB - Parity enable
    //  PARODD - Odd parity (else even)
    struct termios options;
    tcgetattr(usart_dev->fd, &options);

    switch(cfg->USART_BaudRate)
    {
        case 9600:
            options.c_cflag = B9600 | CS8 | CLOCAL | CREAD;     //<Set baud rate
            break;

        case 19200:
            options.c_cflag = B19200 | CS8 | CLOCAL | CREAD;     //<Set baud rate
            break;

        case 57600:
            options.c_cflag = B57600 | CS8 | CLOCAL | CREAD;     //<Set baud rate
            break;

        case 115200:
            options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;     //<Set baud rate
            break;

        default:
            goto out_fail;
    }

    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcflush(usart_dev->fd, TCIFLUSH);
    tcsetattr(usart_dev->fd, TCSANOW, &options);

    /* Create transmit thread for this connection */
    xTaskCreate((pdTASK_CODE)PIOS_USART_RxThread, (const signed char *)"USART task", 4096, (void *)usart_dev, TASK_PRIORITY, &usart_dev->rxThread);

    return 0;

out_fail:
    return -1;
}

static void PIOS_USART_Flush(uint32_t *uart_id)
{
    struct pios_usart_dev *usart_dev = (struct pios_usart_dev *)uart_id;
    int rx_count = 0;
    uint8_t tmp_buff[256];

    do
    {
        rx_count = read(usart_dev->fd, (void*)tmp_buff, 256);
    }while(rx_count > 0);
}



/**
 * RxThread
 */
static void *PIOS_USART_RxThread(uint32_t *uart_id)
{
    struct pios_usart_dev *usart_dev;
    portTickType lastSysTime;

    PIOS_Assert(uart_id);

    usart_dev = (struct pios_usart_dev *)uart_id;


    PIOS_USART_Flush(uart_id);

    // Main task loop
    lastSysTime = xTaskGetTickCount();

    while(1)
    {
        bool rx_need_yield = false;

        //----- CHECK FOR ANY RX BYTES -----
        if (usart_dev->fd != -1)
        {
            usart_dev->rx_cnt = read(usart_dev->fd, (void*)usart_dev->rx_buffer, PIOS_USART_RX_BUFFER_SIZE);

            if (usart_dev->rx_cnt < 0)
            {
                PIOS_PError("Error usart_dev->rx_cnt = %d\r\n", usart_dev->rx_cnt);
            }
            else if (usart_dev->rx_cnt == 0)
            {}
            else
            {
                if (usart_dev->rx_in_cb) {
                    (void)(usart_dev->rx_in_cb)(usart_dev->rx_in_context, usart_dev->rx_buffer, usart_dev->rx_cnt, NULL, &rx_need_yield);
                }
            }
        }

        if (usart_dev->rx_cnt < PIOS_USART_RX_BUFFER_SIZE)
        {
            vTaskDelayUntil(&lastSysTime, SAMPLE_PERIOD_MS / portTICK_RATE_MS);
        }
        else
        {
            vTaskDelayUntil(&lastSysTime, 20 / portTICK_RATE_MS);
        }

    }

    return NULL;
}

static void PIOS_USART_ChangeBaud(uint32_t uart_id, uint32_t baud)
{}

static void PIOS_USART_RxStart(uint32_t uart_id, uint16_t rx_bytes_avail)
{}

static void PIOS_USART_TxStart(uint32_t uart_id, uint16_t tx_bytes_avail)
{}

static void PIOS_USART_RegisterRxCallback(uint32_t uart_id, pios_com_callback rx_in_cb, uint32_t context)
{
    struct pios_usart_dev *usart_dev = (struct pios_usart_dev *)uart_id;

    PIOS_Assert(usart_dev);

    PIOS_PInfo("PIOS_USART_RegisterRxCallback\r\n");
    /*
     * Order is important in these assignments since ISR uses _cb
     * field to determine if it's ok to dereference _cb and _context
     */
    usart_dev->rx_in_context = context;
    usart_dev->rx_in_cb = rx_in_cb;
}

static void PIOS_USART_RegisterTxCallback(uint32_t uart_id, pios_com_callback tx_out_cb, uint32_t context)
{
    struct pios_usart_dev *usart_dev = (struct pios_usart_dev *)uart_id;

    PIOS_Assert(usart_dev);

    PIOS_PInfo("PIOS_USART_RegisterTxCallback\r\n");
    /*
     * Order is important in these assignments since ISR uses _cb
     * field to determine if it's ok to dereference _cb and _context
     */
    usart_dev->tx_out_context = context;
    usart_dev->tx_out_cb = tx_out_cb;

}


