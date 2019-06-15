
#include "nordic_common.h"
#include "app_uart.h"
#include "bsp.h"
#include "uart_k50.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#define MAX_TEST_DATA_BYTES     (15U)                /**< max number of test bytes to be used for tx and rx. */
#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */

struct Send_HRS gSend_HRS;

void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}

void uart_init(void)
{
	uint32_t err_code;
	
	const app_uart_comm_params_t comm_params =
      {
          RX_PIN_NUMBER,
          TX_PIN_NUMBER,
          RTS_PIN_NUMBER,
          CTS_PIN_NUMBER,
          APP_UART_FLOW_CONTROL_DISABLED,
          false,
          NRF_UART_BAUDRATE_115200
      };

    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOWEST,
                         err_code);

    APP_ERROR_CHECK(err_code);
			
	gSend_HRS.head = 0x10;
	gSend_HRS.status = NOT_CONNECTED;
	gSend_HRS.hrs_data = 0x00;
	gSend_HRS.CRC = 0x00;
}

uint16_t uart_read(uint8_t * buff, uint16_t len)
{
	uint8_t cr;
	uint16_t i = 0;
	
	for( i=0; i<len; i++ )
	{
		if(app_uart_get(&cr) == NRF_SUCCESS)
		{
			buff[i] = cr;
		}
		else
			break;
	}
	return i;
}

uint16_t uart_write(uint8_t * buff, uint16_t len)
{
	uint16_t i = 0;
	
	for( i=0; i<len; i++ )
	{
		if(app_uart_put(buff[i]) != NRF_SUCCESS)
		{
			break;
		}
	}
	return i;
}

uint8_t And_Check(uint8_t * data, uint16_t len)
{
		uint16_t sum = 0;
		uint16_t i = 0;
	
	for(i=0; i<len-1; i++)
	{
		sum += data[i];
	}
	
	return (sum&0xff);
	
}

void uart_tx_timeout_handler(void * p_context)
{
	//NRF_LOG_INFO("uart_tx_timeout_handler\n");
	//nrf_gpio_pin_set(BSP_LED_0);
	
	gSend_HRS.CRC = And_Check((uint8_t *)&gSend_HRS, sizeof(struct Send_HRS));
	uart_write((uint8_t *)&gSend_HRS, sizeof(struct Send_HRS));
}

void uart_rx_timeout_handler(void * p_context)
{
	uint8_t i = 0;
	uint16_t len = 0;
	uint8_t read_buff[10];

	//nrf_gpio_pin_toggle(BSP_LED_0);
	
	len = uart_read(read_buff, 10);
	//NRF_LOG_INFO("len = %d", len);
	if(len != 0)
	{
		uart_write(read_buff, len);
		for(i=0; i<len; i++)
			NRF_LOG_INFO("read[%d] = %c", i, read_buff[i]);
	}
}
