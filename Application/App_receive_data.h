#ifndef APP_RECEIVE_DATA_H
#define APP_RECEIVE_DATA_H

#include "Int_SI24R1.h"
#include "Com_config.h"
#include "Com_debug.h"
#include "Int_VL53L1X.h"


//定义帧头校验位
#define FRAME_HEAD_1 'L'
#define FRAME_HEAD_2 'Y'
#define FRAME_HEAD_3 'Q'

//最大重试次数
#define MAX_RETRY_COUNT 10

//1没收到数据或校验失败，0收到数据
uint8_t App_receive_data(void);

//处理连接状态
void App_process_connect_state(uint8_t res);
//处理飞行状态
void App_process_flight_state(void);

#endif // APP_RECEIVE_DATA_H
