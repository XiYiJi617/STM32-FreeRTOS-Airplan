#ifndef INT_IP5305T_H
#define INT_IP5305T_H

//需要main里关于接口的宏定义
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

void Int_IP5305T_start(void);

//软件执行关机指令
void Int_IP5305T_shutdown(void);

#endif // INT_IP5305T_H
