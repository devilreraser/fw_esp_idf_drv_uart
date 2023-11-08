/* *****************************************************************************
 * File:   drv_uart.h
 * Author: Dimitar Lilov
 *
 * Created on 2022 06 18
 * 
 * Description: ...
 * 
 **************************************************************************** */
#pragma once

#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */


/* *****************************************************************************
 * Header Includes
 **************************************************************************** */
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/uart.h"
    
/* *****************************************************************************
 * Configuration Definitions
 **************************************************************************** */

/* *****************************************************************************
 * Constants and Macros Definitions
 **************************************************************************** */

/* *****************************************************************************
 * Enumeration Definitions
 **************************************************************************** */

/* *****************************************************************************
 * Type Definitions
 **************************************************************************** */

/* *****************************************************************************
 * Function-Like Macro
 **************************************************************************** */

/* *****************************************************************************
 * Variables External Usage
 **************************************************************************** */ 

/* *****************************************************************************
 * Function Prototypes
 **************************************************************************** */
esp_err_t drv_uart_init(int uart_num, int nBaudRate, uart_parity_t eParity, size_t rx_buffer);
size_t drv_uart_send_data(int uart_num, uint8_t* pData, size_t nSize);
size_t drv_uart_read_data(int uart_num, uint8_t* pData, size_t nSize, TickType_t ticks_to_wait);
size_t drv_uart_read_size_get(int uart_num);


#ifdef __cplusplus
}
#endif /* __cplusplus */

