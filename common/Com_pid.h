#ifndef COM_PID_H
#define COM_PID_H

#include "main.h"
#define PID_PERIOD 0.006//单次测量的时间周期，0.006s

//pid结构体
//kp,ki,kd需要在初始化时确定，后续来测，目标值和测量值需要在计算时传递，即前面得出的欧拉角
typedef struct
{
    float kp;//比例部分常量，越大响应目标值越快
    float ki;//积分部分常量，解决稳态误差，无人机用得少
    float kd;//微分部分常量，值越大抑制效果越好，解决过调震荡
    float err;//误差值
    float desire;//目标值
    float measure;//测量值实际值
    float last_err;//上一次误差
    float integral;//积分累积
    float output;//输出值
} PID_Struct;

//单次pid运算
void Com_PID_Calc(PID_Struct *pid);

//串级pid运算
void Com_PID_Calc_Chain(PID_Struct *out_pid, PID_Struct *in_pid);

//电机速度上限
int16_t Com_limit(int16_t speed, int16_t max_speed,int16_t min_speed);


#endif // COM_PID_H
