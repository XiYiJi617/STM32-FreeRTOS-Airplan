#include "Com_debug.h"

//串口重定向，使用printf将打印内容通过串口发送到电脑上位机
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 1000);
    return ch;
}
