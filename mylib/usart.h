#ifndef USART_H__
#define USART_H__

#include "ti_msp_dl_config.h"
#include "stdint.h"
//负责封装串口函数库

// 封装发送字节函数，传入串口基地址指针
void UART_SendByte(UART_Regs *uart, uint8_t data);

//封装发送多个字节函数，传入串口地址和数据
void UART_SendBytes(UART_Regs *uart,uint8_t *data,uint16_t length);

// 封装发送字符串函数
void USART_SendString(UART_Regs *uart, char *str);
#endif 