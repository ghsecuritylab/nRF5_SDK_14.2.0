
#ifndef MAIN_H
#define MAIN_H


void scan_start(void);

extern ble_gap_scan_params_t const m_scan_params;
extern ble_gap_conn_params_t const m_connection_param;
extern uint16_t m_conn_handle_hrs_c;


#endif 

