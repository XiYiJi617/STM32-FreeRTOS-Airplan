#ifndef COM_DEBUG_H
#define COM_DEBUG_H

#include "usart.h"
#include "stdio.h"
#include "stdarg.h"
#include "string.h"

//日志输出打印在CPU运行时很占资源=>通过比特率计算知打印10帧8字节大概1ms 影响飞行
//因此在飞机正常飞行时需要关闭打印功能
//设计一个日志输出打印开关
#define DEBUG_LOG_ENABLE 1

#ifdef DEBUG_LOG_ENABLE

//使用宏定义的方式 只打印文件名称 不打印路径名称
//strrchr 从后向前查找字符串中第一次出现字符的位置 返回指针
#define __FILE__NAME__   (strrchr(__FILE__, '\\') ? (strrchr(__FILE__, '\\') + 1) : __FILE__)
#define __FILE__NAME     (strrchr(__FILE__NAME__, '/') ? (strrchr(__FILE__NAME__, '/') + 1) : __FILE__NAME__)

//使用宏定义的方式实现打印日志之前 先添加文件名和行号
#define debug_printf(format, ...) printf("[%s:%d]  "format, __FILE__NAME, __LINE__, ##__VA_ARGS__); 

#else
//如果没有开启日志输出打印
#define debug_printf(format, ...) 

#endif 
#endif // COM_DEBUG_H
