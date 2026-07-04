#include "Int_motor.h"

//设置占空比，实现不同转速
void Int_motor_set_speed(Motor_Struct *motor)
{
    if(motor->speed > 1000)
    {
        debug_printf("motor speed too high\r\n");
        return;
    }

    __HAL_TIM_SET_COMPARE(motor->tim, motor->channel, motor->speed);
}

//开启电机
void Int_motor_start(Motor_Struct *motor)
{
    __HAL_TIM_SET_COMPARE(motor->tim, motor->channel, 0);
    HAL_TIM_PWM_Start(motor->tim, motor->channel);
}
