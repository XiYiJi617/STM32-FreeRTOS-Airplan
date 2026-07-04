#include "App_freeRTOS_Task.h"

//LED结构体
LED_Struct left_top_led = {.port = LED1_GPIO_Port,.pin = LED1_Pin};
LED_Struct right_top_led = {.port = LED2_GPIO_Port,.pin = LED2_Pin};
LED_Struct right_bottom_led = {.port = LED3_GPIO_Port,.pin = LED3_Pin};
LED_Struct left_bottom_led = {.port = LED4_GPIO_Port,.pin = LED4_Pin};

//表示当前连接状态
Remote_State remote_state = REMOTE_DISCONNECTED;

//表示当前飞行状态
Flight_State flight_state = IDLE;

//拓展获取接收的遥感数据，用于执行关机指令
Remote_Data remote_data = {.thr = 0,.yaw = 500,.pit = 500,.rol = 500,.fix_height = 0,.shutdown = 0};//必须赋值，不然全0除了油门前后左右横滚都有数值浆要打转了

//按下定高时的高度
uint16_t fix_height = 0;

//电源管理任务，最高优先级防止低功耗自动关机，器件特性10s判断一次
void power_task(void *pvParameters);
#define POWER_TASK_STACK_SIZE 128
//最高优先级
#define POWER_TASK_PRIORITY 4
TaskHandle_t power_task_handle;
#define POWER_TASK_PERIOD 10000

//飞行控制任务
void flight_task(void *pvParameters);
#define FLIGHT_TASK_STACK_SIZE 128
#define FLIGHT_TASK_PRIORITY 3
TaskHandle_t flight_task_handle;
#define FLIGHT_TASK_PERIOD 6

//LED任务，可视化飞机运行状态和遥控器连接状态
void led_task(void *pvParameters);
#define LED_TASK_STACK_SIZE 128
#define LED_TASK_PRIORITY 1
TaskHandle_t led_task_handle;
#define LED_TASK_PERIOD 100

//通讯任务
void com_task(void *pvParameters);
#define COM_TASK_STACK_SIZE 128
#define COM_TASK_PRIORITY 2
TaskHandle_t com_task_handle;
#define COM_TASK_PERIOD 6

void App_freeRTOS_start(void)
{
	//1.创建任务
    xTaskCreate(power_task,"power_task",POWER_TASK_STACK_SIZE,NULL,POWER_TASK_PRIORITY,&power_task_handle);
    xTaskCreate(flight_task,"flight_task",FLIGHT_TASK_STACK_SIZE,NULL,FLIGHT_TASK_PRIORITY,&flight_task_handle);
	xTaskCreate(led_task,"led_task",LED_TASK_STACK_SIZE,NULL,LED_TASK_PRIORITY,&led_task_handle);
	xTaskCreate(com_task,"com_task",COM_TASK_STACK_SIZE,NULL,COM_TASK_PRIORITY,&com_task_handle);
    //2.启动任务调度
	vTaskStartScheduler();
}

//电源任务
void power_task(void *pvParameters)
{
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	while(1)
	{
		//遥控器发送关机指令会发送任务通知，在这里接住通知做判断即可
		uint32_t res = ulTaskNotifyTake(pdTRUE,POWER_TASK_PERIOD);
		if(res != 0)
		{
			//关机
			Int_IP5305T_shutdown();
		}
		else
		{
			//保持电源正常工作
			Int_IP5305T_start();
		}
	}
}

//飞行控制任务
void flight_task(void *pvParameters)
{
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	uint8_t count = 0;
	App_flight_init();
	while(1)
	{
		//姿态解算得到欧拉角
		App_flight_get_euler_angle();
		//根据欧拉角，进行PID计算控制
		App_flight_pid_process();
		//判断定高
		if(flight_state == FIX_HEIGHT)
		{
			//按了定高才计算PID，激光测距仪20ms采集一次数据，这个任务6ms过快
			count++;
			if(count >= 4)
			{
				App_flight_fix_height_pid_process();
				count = 0;
			}
		}
		//根据PID计算结果对电机进行控制
		App_flight_control_motor();

		//打印激光测距仪得到的距离值
		// uint16_t distance = Int_VL53L1X_GetDistance();
		// debug_printf("distance: %d\r\n",distance);
		vTaskDelayUntil(&xLastWakeTime,FLIGHT_TASK_PERIOD);
	}
}

//灯控任务
void led_task(void *pvParameters)
{
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	uint8_t count = 0;
	while(1)
	{
		count++;
		//前两个灯表示连接状态
		//判断当前连接状态
		if(remote_state == REMOTE_CONNECTED)
		{
			//点亮前两个灯
			Int_led_turn_on(&left_top_led);
			Int_led_turn_on(&right_top_led);
		}
		else if (remote_state == REMOTE_DISCONNECTED)
		{
			//熄灭前两个灯
			Int_led_turn_off(&left_top_led);
			Int_led_turn_off(&right_top_led);
		}
		
		//后两个灯表示飞行状态
		//判断当前飞行状态
		if(flight_state == IDLE)
		{
			//灯慢闪烁，500ms亮，500ms灭
			if(count % 5 == 0)
			{
				Int_led_toggle(&left_bottom_led);
				Int_led_toggle(&right_bottom_led);
			}
		}
		else if (flight_state == NORMAL)
		{
			//灯快闪烁，200ms亮，200ms灭
			if(count % 2 == 0)
			{
				Int_led_toggle(&left_bottom_led);
				Int_led_toggle(&right_bottom_led);
			}
		}
		else if (flight_state == FIX_HEIGHT)
		{
			//后两个灯常亮
			Int_led_turn_on(&left_bottom_led);
			Int_led_turn_on(&right_bottom_led);
		}
		else if (flight_state == FAIL)
		{
			//后两个灯灭
			Int_led_turn_off(&left_bottom_led);
			Int_led_turn_off(&right_bottom_led);
		}

		//count计数重置，防止溢出变0翻转电平
		if(count == 10)
		{
			count = 0;
		}
		vTaskDelayUntil(&xLastWakeTime,LED_TASK_PERIOD);
	}
}


//通讯任务
void com_task(void *pvParameters)
{
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	while(1)
	{
		//接收数据，有数据res不为0
		uint8_t res = App_receive_data();
		//处理连接状态
		App_process_connect_state(res);
		//处理关机命令
		if (remote_data.shutdown == 1)
		{
			//使用freeRTOS通知机制，通知电源管理任务关机
			xTaskNotifyGive(power_task_handle);
		}
		//处理飞行状态 
		App_process_flight_state();
		vTaskDelayUntil(&xLastWakeTime,COM_TASK_PERIOD);
	}
}
