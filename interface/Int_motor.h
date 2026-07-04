#ifndef INT_MOTOR_H
#define INT_MOTOR_H

#include "tim.h"
#include "Com_debug.h"

typedef struct
{
	TIM_HandleTypeDef *tim;
	uint16_t channel;
    int16_t speed;//与PID得出结果尽量一致，原来不能表示负数，可能实际为0，表现出极大值，得到理想相反效果
}Motor_Struct;

void Int_motor_set_speed(Motor_Struct *motor);
void Int_motor_start(Motor_Struct *motor);

#endif // INT_MOTOR_H
