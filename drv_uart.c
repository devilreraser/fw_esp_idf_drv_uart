/* *****************************************************************************
 * File:   drv_uart.c
 * Author: Dimitar Lilov
 *
 * Created on 2022 06 18
 * 
 * Description: ...
 * 
 **************************************************************************** */

/* *****************************************************************************
 * Header Includes
 **************************************************************************** */
#include "drv_uart.h"

#include "esp_system.h"
#include "esp_log.h"

#include "driver/gpio.h"

#if CONFIG_DRV_STDIO_USE
#include "drv_stdio.h"
#endif

/* *****************************************************************************
 * Configuration Definitions
 **************************************************************************** */
#define TAG "drv_uart"

#define UART0_BAUD      115200
#define UART0_PARITY    UART_PARITY_DISABLE
#define TXD0_PIN        CONFIG_DRV_UART_TXD0
#define RXD0_PIN        CONFIG_DRV_UART_RXD0
#define RTS0_PIN        CONFIG_DRV_UART_RTS0
#define CTS0_PIN        UART_PIN_NO_CHANGE

#define UART1_BAUD      9600
#define UART1_PARITY    UART_PARITY_DISABLE
#define TXD1_PIN        CONFIG_DRV_UART_TXD1
#define RXD1_PIN        CONFIG_DRV_UART_RXD1
#define RTS1_PIN        CONFIG_DRV_UART_RTS1
#define CTS1_PIN        UART_PIN_NO_CHANGE

#define UART2_BAUD      9600
#define UART2_PARITY    UART_PARITY_DISABLE
#define TXD2_PIN        CONFIG_DRV_UART_TXD2
#define RXD2_PIN        CONFIG_DRV_UART_RXD2
#define RTS2_PIN        CONFIG_DRV_UART_RTS2
#define CTS2_PIN        UART_PIN_NO_CHANGE


/* *****************************************************************************
 * Constants and Macros Definitions
 **************************************************************************** */

/* *****************************************************************************
 * Enumeration Definitions
 **************************************************************************** */

/* *****************************************************************************
 * Type Definitions
 **************************************************************************** */
typedef struct
{
    int txd_pin;
    int rxd_pin;
    int rts_pin;
    int cts_pin;
}sUartPins_t;


/* *****************************************************************************
 * Function-Like Macros
 **************************************************************************** */

/* *****************************************************************************
 * Variables Definitions
 **************************************************************************** */
uart_config_t uart_config[SOC_UART_NUM] = 
{
    {
        .baud_rate = UART0_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART0_PARITY,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    },
    {
        .baud_rate = UART1_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART1_PARITY,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    },
    {
        .baud_rate = UART2_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART2_PARITY,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    },
};

sUartPins_t sUartPins[SOC_UART_NUM] = 
{
    {
        TXD0_PIN,
        RXD0_PIN,
        RTS0_PIN,
        CTS0_PIN,
    },
    {
        TXD1_PIN,
        RXD1_PIN,
        RTS1_PIN,
        CTS1_PIN,
    },
    {
        TXD2_PIN,
        RXD2_PIN,
        RTS2_PIN,
        CTS2_PIN,
    },
};

/* *****************************************************************************
 * Prototype of functions definitions
 **************************************************************************** */

/* *****************************************************************************
 * Functions
 **************************************************************************** */
esp_err_t drv_uart_init(int uart_num, int nBaudRate, uart_parity_t eParity, size_t rx_buffer)
{
    esp_err_t eError = ESP_ERR_INVALID_ARG;
    //esp_log_level_set(TAG, ESP_LOG_INFO);
    if (uart_num < SOC_UART_NUM)
    {
        uart_config[uart_num].baud_rate = nBaudRate;
        uart_config[uart_num].parity = eParity;
        eError = ESP_OK;

        if (uart_num == 0)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));            /* wait log to be flushed */
        }

        if (uart_is_driver_installed(uart_num))
        {
            ESP_LOGI(TAG, "drv_uart_init : UART %d uart_is_driver_installed -> delete", uart_num);
            vTaskDelay(pdMS_TO_TICKS(1000));            /* wait external buffers to be flushed */
            eError = uart_flush(uart_num);
            if (eError == ESP_OK)
            {
                eError = uart_flush_input(uart_num);
                if (eError == ESP_OK)
                {
                    eError = uart_driver_delete(uart_num);
                    if (eError != ESP_OK)
                    {
                        ESP_LOGE(TAG, "drv_uart_init : UART %d uart_driver_delete failure", uart_num);
                    }
                }
                else
                {
                    ESP_LOGE(TAG, "drv_uart_init : UART %d uart_flush_input failure", uart_num);
                }
            }
            else
            {
                ESP_LOGE(TAG, "drv_uart_init : UART %d uart_flush failure", uart_num);
            }  
        }

        if (eError == ESP_OK)
        {
            #if CONFIG_UART_ISR_IN_IRAM
            eError = uart_driver_install(uart_num, rx_buffer, 1024, 0, NULL, ESP_INTR_FLAG_IRAM);
            #else
            eError = uart_driver_install(uart_num, rx_buffer, 1024, 0, NULL, 0);
            #endif
            if (eError == ESP_OK)
            {
                eError = uart_param_config(uart_num, &uart_config[uart_num]);
                if (eError == ESP_OK)
                {
                    eError = uart_set_pin(uart_num, sUartPins[uart_num].txd_pin, sUartPins[uart_num].rxd_pin, sUartPins[uart_num].rts_pin, sUartPins[uart_num].cts_pin);
                    if (eError == ESP_OK)
                    {
                        if (sUartPins[uart_num].rts_pin != UART_PIN_NO_CHANGE)
                        {
                            eError = uart_set_mode(uart_num, UART_MODE_RS485_HALF_DUPLEX);
                            if (eError != ESP_OK)
                            {
                                ESP_LOGE(TAG, "drv_uart_init : UART %d uart_set_mode failure", uart_num);
                            }
                        }
                        
                        if (eError == ESP_OK)
                        {
                            ESP_LOGI(TAG, "drv_uart_init : UART %d Initialization Success", uart_num);
                        }
                    }
                    else
                    {
                        ESP_LOGE(TAG, "drv_uart_init : UART %d uart_set_pin failure", uart_num);
                    }
                }
                else
                {
                    ESP_LOGE(TAG, "drv_uart_init : UART %d uart_param_config failure", uart_num);
                }
            }
            else
            {
                ESP_LOGE(TAG, "drv_uart_init : UART %d uart_driver_install failure", uart_num);
            }
        }
    }
    else
    {
        ESP_LOGE(TAG, "drv_uart_init : UART %d invalid", uart_num);
    }
    return eError;
}

size_t drv_uart_send_data(int uart_num, uint8_t* pData, size_t nSize)
{
    return uart_write_bytes(uart_num, pData, nSize);
}
size_t drv_uart_read_data(int uart_num, uint8_t* pData, size_t nSize, TickType_t ticks_to_wait)
{
    #if CONFIG_DRV_STDIO_USE
    if ((uart_num == UART_NUM_0) && drv_stdio_is_redirect_uart0())
    {
        //use stream from stdio
        size_t nRedirectBytes = drv_stdio_redirect_stream_pull(pData, nSize);
        if (nRedirectBytes)
        {
            ESP_LOGW(TAG, "drv_uart_read_data : drv_stdio_is_redirect_uart0 %d bytes", nRedirectBytes);
            ESP_LOG_BUFFER_HEX(TAG, pData, nRedirectBytes);
        }
        return nRedirectBytes;
    }
    else
    #endif
    {
        return uart_read_bytes(uart_num, pData, nSize, ticks_to_wait);
    }
    
}
size_t drv_uart_read_size_get(int uart_num)
{
    size_t rxBytes;
    uart_get_buffered_data_len(uart_num, &rxBytes);
    return rxBytes;
}
