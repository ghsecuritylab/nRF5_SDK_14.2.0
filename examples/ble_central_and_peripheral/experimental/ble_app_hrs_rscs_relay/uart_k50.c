
#include "nordic_common.h"
#include "app_uart.h"
#include "bsp.h"
#include "uart_k50.h"
#include "main.h"
#include <ble_gap.h>
#include <ble_hci.h>

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
struct Scan_Result gScan_Result;

enum e_CONNECT_STATUS Connect_Status = CONNECT_IDLE;


//查询一个MAC地址是否已经存在
bool find_mac_addr(uint8_t * src, struct Scan_Result * dts)
{
	uint32_t i = 0;

	NRF_LOG_INFO("src = %x %x %x %x %x %x", src[0], src[1], src[2], src[3], src[4], src[5]);

	for(i=0; i<DEV_INOF_MAX; i++)
	{
		if(memcmp(src, dts->dev_inof[i].addr.addr, BLE_GAP_ADDR_LEN) == 0)
			return true;
	}
	return false;
}

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
	gSend_HRS.type = 0x02;
	gSend_HRS.status = NOT_CONNECTED;
	gSend_HRS.hrs_data = 0x00;
	gSend_HRS.CRC = 0x55;
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
	
	//gSend_HRS.CRC = And_Check((uint8_t *)&gSend_HRS, sizeof(struct Send_HRS));
	//uart_write((uint8_t *)&gSend_HRS, sizeof(struct Send_HRS));
}

void uart_rx_timeout_handler(void * p_context)
{
	ret_code_t err_code;
	uint16_t len = 0;
	uint8_t read_buff[10];
	
	len = uart_read(read_buff, 10);
	if(len != 0)
	{
		//开始扫描指令
		if( (read_buff[0] == 0xf1) && (read_buff[1] == 0x01) && (read_buff[3] == 0x55) )
		{
			Connect_Status = CONNECT_SCAN;
		}
		//连接指令
		if( (read_buff[0] == 0xf1) && (read_buff[1] == 0x02) && (read_buff[3] == 0x55)  )
		{
			err_code = sd_ble_gap_connect(&gScan_Result.dev_inof[read_buff[2]].addr,
                                                  &m_scan_params,
                                                  &m_connection_param,
                                                  1);
            if (err_code != NRF_SUCCESS)
            {
                NRF_LOG_INFO("Connection Request Failed, reason %d", err_code);
            }
			else
			{
				Connect_Status = CONNECT_SEND;
			}
		}
	}
}


void connect_fsm_handler(void * p_context)
{
	ret_code_t err_code;
	static uint32_t count = 0;
	static uint32_t send_count = 0;

	switch(Connect_Status)
	{
		//空闲状态
		case CONNECT_IDLE:
		{
		
		}
		break;
		//扫描开始
		case CONNECT_SCAN:
		{
			//先断开之前的连接
			sd_ble_gap_disconnect(m_conn_handle_hrs_c, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
		
			NRF_LOG_INFO("scan_start!!!!!!!!!!!!!");
			scan_start();
			Connect_Status = CONNECT_SCAN_ING;
			count = 0;
			memset(&gScan_Result, 0x00, sizeof(struct Scan_Result));
		}
		break;
		//扫描中
		case CONNECT_SCAN_ING:
		{
			//计时,100ms一次
			count ++;
			if(count > 50)
			{
				Connect_Status = CONNECT_SCAN_COMPLETED;
			}
		}
		break;
		//扫描完成
		case CONNECT_SCAN_COMPLETED:
		{
			(void) sd_ble_gap_scan_stop();
		
			//发送数据给K50
			gScan_Result.head = 0x1F;
			gScan_Result.type = 0x01;

			NRF_LOG_INFO("scan dev num = %d", gScan_Result.num );
			for(uint32_t i = 0; i<gScan_Result.num; i++)
			{
				NRF_LOG_INFO("dev %d name = %s", i, gScan_Result.dev_inof[i].name );
				NRF_LOG_FLUSH();
				NRF_LOG_INFO("dev smac = %x %x %x %x %x %x", gScan_Result.dev_inof[i].addr.addr[0], gScan_Result.dev_inof[i].addr.addr[1],
					gScan_Result.dev_inof[i].addr.addr[2], gScan_Result.dev_inof[i].addr.addr[3], gScan_Result.dev_inof[i].addr.addr[4], 
					gScan_Result.dev_inof[i].addr.addr[5]);
			}

			uart_write((uint8_t *)&gScan_Result, sizeof(struct Scan_Result));
			Connect_Status = CONNECT_IDLE;
		}
		break;
		//发送心率数据
		case CONNECT_SEND:
		{
			send_count ++;
			if( send_count >= 10 )
			{
				send_count = 0;
				uart_write((uint8_t *)&gSend_HRS, sizeof(struct Send_HRS));
			}
		}
		break;
	}
}

