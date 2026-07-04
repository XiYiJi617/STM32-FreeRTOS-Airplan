#ifndef APP_FREE_RTOS_TASK_H
#define APP_FREE_RTOS_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "Com_debug.h"
#include "Com_config.h"
#include "Int_IP5305T.h"
#include "Int_motor.h"
#include "Int_led.h"
#include "Int_SI24R1.h"
#include "App_receive_data.h"
#include "App_flight.h"

void App_freeRTOS_start(void);

#endif // APP_FREE_RTOS_TASK_H
