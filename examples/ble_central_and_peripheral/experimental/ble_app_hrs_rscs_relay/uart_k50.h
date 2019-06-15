
#ifndef UART_K50_H
#define UART_K50_H

#include <stdint.h>

enum e_OH1_STATUS
{
	NOT_CONNECTED,	//δ����
	CONNECTED,			//������
	RECV_HRS,				//���ڽ�������
};

struct __attribute__((packed)) Send_HRS
{
	uint8_t head;
	enum e_OH1_STATUS status; 
	uint16_t hrs_data;
	uint8_t CRC;
};

void uart_init(void);
void uart_tx_timeout_handler(void * p_context);
void uart_rx_timeout_handler(void * p_context);


extern struct Send_HRS gSend_HRS;

#endif
