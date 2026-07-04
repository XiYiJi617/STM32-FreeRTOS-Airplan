#ifndef COM_CONFIG_H
#define COM_CONFIG_H

#include "main.h"

//遥控器连接状态
typedef enum
{
    REMOTE_CONNECTED = 0,
    REMOTE_DISCONNECTED,
}Remote_State;

//飞行状态
typedef enum
{
    IDLE = 0,//空闲
    NORMAL,
    FIX_HEIGHT,
    FAIL,
}Flight_State;

//接收到的遥控数据
typedef struct
{
	int16_t thr;
	int16_t yaw;
	int16_t pit;
	int16_t rol;
    uint8_t shutdown;
    uint8_t fix_height;
}Remote_Data;

//油门解锁状态
typedef enum
{
    FREE = 0,
    MAX,
    LEAVE_MAX,
    MIN,
    UNLOCK,
}Thr_state;

// 陀螺仪数据
//1.角速度
typedef struct
{
    int16_t gyro_x; // 往右飞为正   表示横滚角
    int16_t gyro_y; // 向前飞转动为正 表示俯仰角
    int16_t gyro_z; // 逆时针转动为正  表示偏航角
} Gyro_struct;

//2.加速度
typedef struct
{
    int16_t accel_x; // 往前为正
    int16_t accel_y; // 往左为正
    int16_t accel_z; // 朝上的加速度为正
} Accel_struct;

//3.六轴数据
typedef struct
{
    Gyro_struct gyro;
    Accel_struct accel;
} Gyro_Accel_Struct;

// 解算得到的欧拉角
typedef struct
{
    float yaw;
    float pitch;
    float roll;
} Euler_struct;


#endif // COM_CONFIG_H
