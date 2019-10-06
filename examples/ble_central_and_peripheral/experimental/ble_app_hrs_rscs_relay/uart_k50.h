
#ifndef UART_K50_H
#define UART_K50_H

#include <stdint.h>
#include <ble_gap.h>

enum e_OH1_STATUS
{
	NOT_CONNECTED,	//δ����
	CONNECTED,			//������
	RECV_HRS,				//���ڽ�������
};

//���ʰ�
struct __attribute__((packed)) Send_HRS
{
	uint8_t head;
	uint8_t type;
	enum e_OH1_STATUS status; 
	uint16_t hrs_data;
	uint8_t CRC;
};

#define DEV_NAME_MAX (20)

//�豸����Ϣ
struct Dev_Inof
{
	uint8_t name[DEV_NAME_MAX];	//�豸������
	ble_gap_addr_t addr;		//�豸��mac��ַ
};

#define DEV_INOF_MAX (10)

//ɨ������
struct Scan_Result
{
	uint8_t head;	//��ͷ
	uint8_t type;	//����
	uint8_t num;	//�豸����
	struct Dev_Inof dev_inof[DEV_INOF_MAX];	//�豸����Ϣ
};

//����״̬��
enum e_CONNECT_STATUS
{
	CONNECT_IDLE,		//����
	CONNECT_SCAN,		//����ɨ��
	CONNECT_SCAN_ING,	//ɨ����
	CONNECT_SCAN_COMPLETED,		//ɨ�����
	CONNECT_SEND,		//������������״̬
};

extern enum e_CONNECT_STATUS Connect_Status;


void uart_init(void);
void uart_tx_timeout_handler(void * p_context);
void uart_rx_timeout_handler(void * p_context);
void connect_fsm_handler(void * p_context);
bool find_mac_addr(uint8_t * src, struct Scan_Result * dts);


extern struct Send_HRS gSend_HRS;
extern struct Scan_Result gScan_Result;

#endif
