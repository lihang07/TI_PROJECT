#include "ti_msp_dl_config.h"
#include "mylib/delay.h"
#ifndef __KEY_H__
#define __KEY_H__
#define KEY_PRESS 0
#define KEY_RELEASE 1
#define KEY_NUM 3//自定义按键使用数量

//获取按键输入并处理消抖
//使用前先在主函数声明结构体类型变量key1
//按键控制与状态管理



typedef struct
{
    GPIO_Regs *port;   // GPIO端口
    uint32_t   pin;    // 引脚名称
    uint8_t    key_up; // 状态标志（用于消抖）
}Key_t;

extern Key_t key1;
extern Key_t key2;
extern Key_t key3;
extern Key_t key4;


//按键消抖
uint8_t key_scan(Key_t *key);

//按键状态扫描
uint8_t key_read(void);

#endif