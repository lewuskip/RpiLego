/*
 * pios_uart_rpi.h
 *
 *  Created on: 23 Nov 2013
 *      Author: lewy
 */

#ifndef PIOS_UART_RPI_H_
#define PIOS_UART_RPI_H_

/**
  * @brief  USART Init Structure definition
  */

typedef struct usart_rpi_config_t
{
    const char *devname;
    uint32_t    USART_BaudRate;
    uint16_t    USART_WordLength;
    uint16_t    USART_StopBits;
    uint16_t    USART_Parity;
    uint16_t    USART_Mode;
} usart_rpi_config_t;

extern const struct pios_com_driver pios_usart_com_driver;

int PIOS_USART_Init(uint32_t *uart_id, const struct usart_rpi_config_t *cfg);

#endif /* PIOS_UART_RPI_H_ */
