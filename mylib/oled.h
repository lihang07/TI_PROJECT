//////////////////////////////////////////////////////////////////////////////////
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//中景园电子
//店铺地址：http://shop73023976.taobao.com/?spm=2013.1.0.0.M4PqC2
//
//  文 件 名   : oled.h
//  版 本 号   : v2.0
//  作    者   : HuangKai
//  生成日期   : 2014-0101
//  最近修改   : 2026-07-12 移植到MSPM0G3507，适配硬件I2C
//  功能描述   : 0.96寸OLED 接口演示例程(MSPM0G3507系列)
//  驱动IC     : SSD1306/SSD1309
//              说明:
//              ----------------------------------------------------------------
//              GND    电源地
//              VCC    接3.3v电源
//              SCL    PA31（I2C0_SCL，SysConfig中"OLED"组）
//              SDA    PA28（I2C0_SDA，SysConfig中"OLED"组）
//              RST    PB6（复位，手动GPIO初始化）
//              DC     PB8（数据/命令，I2C模式下未使用）
//              CS     PB10（片选，I2C模式下未使用）
//              ----------------------------------------------------------------
//  注意：I2C通信使用SysConfig配置的硬件I2C0外设（OLED_INST），
//        RST/DC/CS引脚通过代码手动初始化GPIO。
// 修改历史   :
// 日    期   : 2026-07-12
// 作    者   : 移植到MSPM0G3507
// 修改内容   : 使用硬件I2C0替代软件模拟I2C
//版权所有，盗版必究。
//Copyright(C) 中景园电子2014/3/16
//All rights reserved
//******************************************************************************/
#ifndef __OLED_H
#define __OLED_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

#define OLED_CMD  0  /* 写命令 */
#define OLED_DATA 1  /* 写数据 */

/* SSD1306的7位I2C地址（SA0引脚接地时为0x3C） */
#define OLED_I2C_ADDR           0x3C

/*
 * 非I2C控制引脚定义（硬编码，不依赖SysConfig）
 *
 * SCL(PA31)和SDA(PA28)由SysConfig的I2C0外设管理，此处不定义。
 * 以下引脚由代码手动初始化为GPIO输出：
 *   OLED_RST_PORT / OLED_RST_PIN  -> PB6 (复位)
 *   OLED_DC_PORT  / OLED_DC_PIN   -> PB8 (数据/命令，I2C模式下保留)
 *   OLED_CS_PORT  / OLED_CS_PIN   -> PB10 (片选，I2C模式下保留)
 */
/* 复位 RST -> PB6 */
#define OLED_RST_PORT           GPIOB
#define OLED_RST_PIN            DL_GPIO_PIN_6

/* 数据/命令 DC -> PB8（I2C模式下保留兼容） */
#define OLED_DC_PORT            GPIOB
#define OLED_DC_PIN             DL_GPIO_PIN_8

/* 片选 CS -> PB10（I2C模式下保留兼容） */
#define OLED_CS_PORT            GPIOB
#define OLED_CS_PIN             DL_GPIO_PIN_10

/*
 * GPIO引脚操作宏
 * DL_GPIO_setPins:   设置指定引脚输出高电平
 * DL_GPIO_clearPins: 设置指定引脚输出低电平
 */
/* RST复位线操作 */
#define OLED_RST_Set()          DL_GPIO_setPins(OLED_RST_PORT, OLED_RST_PIN)
#define OLED_RST_Clr()          DL_GPIO_clearPins(OLED_RST_PORT, OLED_RST_PIN)

/* DC数据/命令线操作（I2C模式保留） */
#define OLED_DC_Set()           DL_GPIO_setPins(OLED_DC_PORT, OLED_DC_PIN)
#define OLED_DC_Clr()           DL_GPIO_clearPins(OLED_DC_PORT, OLED_DC_PIN)

/* CS片选线操作（I2C模式保留） */
#define OLED_CS_Set()           DL_GPIO_setPins(OLED_CS_PORT, OLED_CS_PIN)
#define OLED_CS_Clr()           DL_GPIO_clearPins(OLED_CS_PORT, OLED_CS_PIN)

/* 开发板LED灯（使用SysConfig中GPIO2 LED组的引脚） */
#define LED_Set()               DL_GPIO_setPins(LED_PORT, LED_PIN22_PIN)
#define LED_Clr()               DL_GPIO_clearPins(LED_PORT, LED_PIN22_PIN)

/* OLED显示参数 */
#define SIZE        16
#define XLevelL     0x02
#define XLevelH     0x10
#define Max_Column  128
#define Max_Row     64
#define Brightness  0xFF
#define X_WIDTH     128
#define Y_WIDTH     64

/* OLED控制用函数声明 */
void OLED_WR_Byte(uint8_t dat, uint8_t cmd);
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Init(void);
void OLED_Clear(void);
void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t t);
void OLED_Fill(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t dot);
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr);
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size2);
void OLED_ShowString(uint8_t x, uint8_t y, uint8_t *p);
void OLED_Set_Pos(unsigned char x, unsigned char y);
void OLED_ShowCHinese(uint8_t x, uint8_t y, uint8_t no);
void OLED_DrawBMP(unsigned char x0, unsigned char y0, unsigned char x1, unsigned char y1, unsigned char BMP[]);

#endif