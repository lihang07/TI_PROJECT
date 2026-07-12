#include "ti_msp_dl_config.h"
#include "mylib/delay.h"
#ifndef __KEY_H__
#define __KEY_H__
#define KEY_PRESS 0
#define KEY_RELEASE 1
//获取按键输入并处理消抖
//使用前先在主函数声明结构体类型变量key1

typedef struct
{
    GPIO_Regs *port;   // GPIO端口
    uint32_t   pin;    // 引脚名称
    uint8_t    key_up; // 状态标志（用于消抖）
}Key_t;

uint8_t key_scan(Key_t *key);

#endif