
#ifndef UART_K50_H
#define UART_K50_H

#include <stdint.h>
#include <ble_gap.h>

enum e_OH1_STATUS
{
	NOT_CONNECTED,	//未连接
	CONNECTED,			//已连接
	RECV_HRS,				//正在接收数据
};

//心率包
struct __attribute__((packed)) Send_HRS
{
	uint8_t head;
	uint8_t type;
	enum e_OH1_STATUS status; 
	uint16_t hrs_data;
	uint8_t CRC;
};

#define DEV_NAME_MAX (20)

//设备的信息
struct Dev_Inof
{
	uint8_t name[DEV_NAME_MAX];	//设备的名称
	ble_gap_addr_t addr;		//设备的mac地址
};

#define DEV_INOF_MAX (10)

//扫描结果包
struct Scan_Result
{
	uint8_t head;	//包头
	uint8_t type;	//类型
	uint8_t num;	//设备数量
	struct Dev_Inof dev_inof[DEV_INOF_MAX];	//设备的信息
};

//连接状态机
enum e_CONNECT_STATUS
{
	CONNECT_IDLE,		//空闲
	CONNECT_SCAN,		//启动扫描
	CONNECT_SCAN_ING,	//扫描中
	CONNECT_SCAN_COMPLETED,		//扫描完成
	CONNECT_SEND,		//发送心率数据状态
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
