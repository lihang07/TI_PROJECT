//////////////////////////////////////////////////////////////////////////////////
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//中景园电子
//店铺地址：http://shop73023976.taobao.com/?spm=2013.1.0.0.M4PqC2
//
//  文 件 名   : oled.c
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
//              SCL    PA31（I2C0_SCL，SysConfig配置）
//              SDA    PA28（I2C0_SDA，SysConfig配置）
//              RST    PB6（复位，手动初始化）
//              DC     PB8（数据/命令，I2C模式下未使用）
//              CS     PB10（片选，I2C模式下未使用）
//              ----------------------------------------------------------------
// 修改历史   :
// 日    期   : 2026-07-12
// 作    者   : 移植到MSPM0G3507
// 修改内容   : 使用硬件I2C0（DL_I2C）替代软件模拟I2C
//               I2C初始化由SysConfig生成的SYSCFG_DL_OLED_init()完成
//版权所有，盗版必究。
//Copyright(C) 中景园电子2014/3/16
//All rights reserved
//******************************************************************************/
#include "ti_msp_dl_config.h"
#include "mylib/oled.h"
#include "mylib/delay.h"
#include "oledfont.h"

/*
 * OLED的显存
 * 存放格式如下:
 * [0]0 1 2 3 ... 127
 * [1]0 1 2 3 ... 127
 * [2]0 1 2 3 ... 127
 * [3]0 1 2 3 ... 127
 * [4]0 1 2 3 ... 127
 * [5]0 1 2 3 ... 127
 * [6]0 1 2 3 ... 127
 * [7]0 1 2 3 ... 127
 */

/*
 * 通过硬件I2C向SSD1306写入控制字节+数据字节
 *
 * 说明：SSD1306的I2C协议要求每个写操作包含2个字节：
 *       第1字节 = 控制字节（0x00=命令, 0x40=数据）
 *       第2字节 = 实际数据
 *       本函数将这两个字节合并为一次I2C传输，通过I2C0外设发出。
 *
 *       通信流程：
 *       1. 等待I2C控制器空闲（前一次传输完成）
 *       2. 将控制字节和数据字节填入TX FIFO
 *       3. 启动I2C传输，发送完成后自动产生STOP条件
 *
 * DL_I2C_fillControllerTXFIFO: 将数据写入I2C控制器的发送FIFO
 * DL_I2C_startControllerTransfer: 启动I2C控制器传输（含START/STOP）
 * DL_I2C_CONTROLLER_DIRECTION_TX: 控制器发送方向
 * DL_I2C_CONTROLLER_STATUS_BUSY: 控制器忙状态标志
 */
static void OLED_I2C_Write(uint8_t dat, uint8_t cmd)
{
    uint8_t txBuf[2];

    /* 控制字节 + 数据字节 */
    txBuf[0] = cmd ? 0x40 : 0x00;   /* 控制字节：0x40=数据, 0x00=命令 */
    txBuf[1] = dat;                  /* 实际数据 */

    /*
     * 等待I2C控制器空闲
     * DL_I2C_getControllerStatus: 读取I2C控制器状态寄存器
     * DL_I2C_CONTROLLER_STATUS_BUSY: 总线忙标志位掩码
     */
    while (DL_I2C_getControllerStatus(OLED_INST) &
           DL_I2C_CONTROLLER_STATUS_BUSY) {}

    /* 将2字节数据填入TX FIFO */
    DL_I2C_fillControllerTXFIFO(OLED_INST, txBuf, 2);

    /*
     * 启动I2C控制器传输
     * 参数: I2C实例, 7位从机地址, 发送方向, 传输字节数
     * 传输完成后自动发送STOP条件
     */
    DL_I2C_startControllerTransfer(OLED_INST, OLED_I2C_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX, 2);
}

/*
 * 向SSD1306写入一个字节
 * dat: 要写入的数据/命令
 * cmd: 数据/命令标志，0=命令(OLED_CMD)，1=数据(OLED_DATA)
 */
void OLED_WR_Byte(uint8_t dat, uint8_t cmd)
{
    OLED_I2C_Write(dat, cmd);
}

/*
 * 设置OLED显示坐标位置
 * x: 列地址 (0~127)
 * y: 页地址 (0~7)
 */
void OLED_Set_Pos(unsigned char x, unsigned char y)
{
    OLED_WR_Byte(0xb0 + y, OLED_CMD);                   /* 设置页地址 */
    OLED_WR_Byte(((x & 0xf0) >> 4) | 0x10, OLED_CMD);   /* 设置列高地址 */
    OLED_WR_Byte((x & 0x0f) | 0x01, OLED_CMD);           /* 设置列低地址 */
}

/*
 * 开启OLED显示
 */
void OLED_Display_On(void)
{
    OLED_WR_Byte(0X8D, OLED_CMD);  /* SET DCDC命令 */
    OLED_WR_Byte(0X14, OLED_CMD);  /* DCDC ON */
    OLED_WR_Byte(0XAF, OLED_CMD);  /* DISPLAY ON */
}

/*
 * 关闭OLED显示
 */
void OLED_Display_Off(void)
{
    OLED_WR_Byte(0X8D, OLED_CMD);  /* SET DCDC命令 */
    OLED_WR_Byte(0X10, OLED_CMD);  /* DCDC OFF */
    OLED_WR_Byte(0XAE, OLED_CMD);  /* DISPLAY OFF */
}

/*
 * 清屏函数
 * 清完屏后整个屏幕是黑色的
 */
void OLED_Clear(void)
{
    uint8_t i, n;
    for (i = 0; i < 8; i++)
    {
        OLED_WR_Byte(0xb0 + i, OLED_CMD);   /* 设置页地址（0~7） */
        OLED_WR_Byte(0x00, OLED_CMD);        /* 设置显示位置—列低地址 */
        OLED_WR_Byte(0x10, OLED_CMD);        /* 设置显示位置—列高地址 */
        for (n = 0; n < 128; n++)
        {
            OLED_WR_Byte(0, OLED_DATA);
        }
    }
}

/*
 * 在指定位置显示一个字符
 * x: 0~127（列坐标）
 * y: 0~63（行坐标，按页对齐）
 * chr: 要显示的字符
 *
 * 字体大小由SIZE宏决定：SIZE=16时使用8x16字体，否则使用6x8字体
 */
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr)
{
    unsigned char c = 0, i = 0;
    c = chr - ' ';  /* 得到偏移后的值 */
    if (x > Max_Column - 1)
    {
        x = 0;
        y = y + 2;
    }
    if (SIZE == 16)
    {
        OLED_Set_Pos(x, y);
        for (i = 0; i < 8; i++)
        {
            OLED_WR_Byte(F8X16[c * 16 + i], OLED_DATA);
        }
        OLED_Set_Pos(x, y + 1);
        for (i = 0; i < 8; i++)
        {
            OLED_WR_Byte(F8X16[c * 16 + i + 8], OLED_DATA);
        }
    }
    else
    {
        OLED_Set_Pos(x, y + 1);
        for (i = 0; i < 6; i++)
        {
            OLED_WR_Byte(F6x8[c][i], OLED_DATA);
        }
    }
}

/*
 * m^n幂运算函数
 * m: 底数
 * n: 指数
 * 返回: m的n次方
 */
static uint32_t oled_pow(uint8_t m, uint8_t n)
{
    uint32_t result = 1;
    while (n--)
    {
        result *= m;
    }
    return result;
}

/*
 * 显示数字
 * x, y : 起点坐标
 * len  : 数字的位数
 * size2: 字体大小
 * num  : 数值(0~4294967295)
 */
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size2)
{
    uint8_t t, temp;
    uint8_t enshow = 0;
    for (t = 0; t < len; t++)
    {
        temp = (num / oled_pow(10, len - t - 1)) % 10;
        if (enshow == 0 && t < (len - 1))
        {
            if (temp == 0)
            {
                OLED_ShowChar(x + (size2 / 2) * t, y, ' ');
                continue;
            }
            else
            {
                enshow = 1;
            }
        }
        OLED_ShowChar(x + (size2 / 2) * t, y, temp + '0');
    }
}

/*
 * 显示字符串
 * x, y: 起点坐标
 * chr:  指向字符串的指针（以'\0'结尾）
 */
void OLED_ShowString(uint8_t x, uint8_t y, uint8_t *chr)
{
    unsigned char j = 0;
    while (chr[j] != '\0')
    {
        OLED_ShowChar(x, y, chr[j]);
        x += 8;
        if (x > 120)
        {
            x = 0;
            y += 2;
        }
        j++;
    }
}

/*
 * 显示汉字
 * x, y: 起点坐标
 * no:   汉字在字库Hzk中的索引号
 */
void OLED_ShowCHinese(uint8_t x, uint8_t y, uint8_t no)
{
    uint8_t t;
    OLED_Set_Pos(x, y);
    for (t = 0; t < 16; t++)
    {
        OLED_WR_Byte(Hzk[2 * no][t], OLED_DATA);
    }
    OLED_Set_Pos(x, y + 1);
    for (t = 0; t < 16; t++)
    {
        OLED_WR_Byte(Hzk[2 * no + 1][t], OLED_DATA);
    }
}

/*
 * 显示BMP图片
 * x0, y0: 起始点坐标
 * x1, y1: 结束点坐标（x1范围0~127，y1为页范围0~7）
 * BMP[]:  图片数据数组
 */
void OLED_DrawBMP(unsigned char x0, unsigned char y0,
                  unsigned char x1, unsigned char y1,
                  unsigned char BMP[])
{
    unsigned int j = 0;
    unsigned char x, y;

    if (y1 % 8 == 0)
    {
        y = y1 / 8;
    }
    else
    {
        y = y1 / 8 + 1;
    }
    for (y = y0; y < y1; y++)
    {
        OLED_Set_Pos(x0, y);
        for (x = x0; x < x1; x++)
        {
            OLED_WR_Byte(BMP[j++], OLED_DATA);
        }
    }
}

/*
 * 绘制一个点
 * x, y: 坐标
 * t:    1=点亮, 0=熄灭
 */
void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t t)
{
    /* 预留接口，SSD1306页寻址模式下逐点操作效率较低 */
    (void)x;
    (void)y;
    (void)t;
}

/*
 * 填充矩形区域
 * x1, y1: 左上角坐标
 * x2, y2: 右下角坐标
 * dot:    填充内容（0xFF=全亮, 0x00=全灭）
 */
void OLED_Fill(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t dot)
{
    uint8_t x, y;
    for (y = y1; y <= y2; y++)
    {
        OLED_Set_Pos(x1, y);
        for (x = x1; x <= x2; x++)
        {
            OLED_WR_Byte(dot, OLED_DATA);
        }
    }
}

/*
 * 初始化SSD1306 OLED显示屏
 *
 * I2C外设初始化：由SysConfig生成的SYSCFG_DL_OLED_init()完成，
 *   在SYSCFG_DL_init()中自动调用，包括：
 *   - 配置I2C0时钟（BUSCLK, 8分频 → 4MHz功能时钟）
 *   - 配置SDA(PA28)和SCL(PA31)引脚为I2C功能
 *   - 使能模拟毛刺滤波器
 *
 * 注意：SysConfig生成的代码未调用DL_I2C_enableController()，
 *       因此本函数中需要手动使能I2C控制器并设置总线速率。
 *
 * 本函数职责：
 *   1. 使能I2C控制器并设置总线速率
 *   2. 手动初始化非I2C引脚（RST复位、DC数据/命令、CS片选）
 *   3. 执行硬件复位时序
 *   4. 发送SSD1306初始化命令序列
 */
void OLED_Init(void)
{
    /*
     * 使能I2C控制器并设置总线速率为100kHz
     *
     * SCL频率计算公式：
     *   SCL = I2C功能时钟 / ((1 + TPR) * (SCL_LP + SCL_HP))
     *        = 4MHz / ((1 + 3) * (6 + 4)) = 4MHz / 40 = 100kHz
     *
     * DL_I2C_enableController: 设置MCR.ACTIVE位，激活I2C控制器
     * DL_I2C_setTimerPeriod:   设置MTPR寄存器，控制SCL时钟周期
     */
    DL_I2C_enableController(OLED_INST);
    DL_I2C_setTimerPeriod(OLED_INST, 3);   /* TPR=3 → 100kHz */

    /*
     * 手动初始化非I2C控制引脚为数字输出
     * SCL(PA31)和SDA(PA28)已由SysConfig初始化为I2C功能，此处无需处理
     * DL_GPIO_initDigitalOutput: 将指定IOMUX引脚初始化为GPIO数字输出
     */
    /* 复位 RST -> PB6 (IOMUX_PINCM23) */
    DL_GPIO_initDigitalOutput(IOMUX_PINCM23);
    /* 数据/命令 DC -> PB8 (IOMUX_PINCM25) */
    DL_GPIO_initDigitalOutput(IOMUX_PINCM25);
    /* 片选 CS -> PB10 (IOMUX_PINCM27) */
    DL_GPIO_initDigitalOutput(IOMUX_PINCM27);

    /*
     * 硬件复位序列
     * DL_GPIO_setPins: 设置指定引脚输出高电平
     * DL_GPIO_clearPins: 设置指定引脚输出低电平
     */
    OLED_RST_Set();
    delay_ms(100);
    OLED_RST_Clr();
    delay_ms(100);
    OLED_RST_Set();

    /* SSD1306初始化命令序列 */
    OLED_WR_Byte(0xAE, OLED_CMD);  /* 关闭OLED面板 */
    OLED_WR_Byte(0x00, OLED_CMD);  /* 设置列低地址 */
    OLED_WR_Byte(0x10, OLED_CMD);  /* 设置列高地址 */
    OLED_WR_Byte(0x40, OLED_CMD);  /* 设置起始行地址 (0x00~0x3F) */
    OLED_WR_Byte(0x81, OLED_CMD);  /* 设置对比度控制寄存器 */
    OLED_WR_Byte(0xCF, OLED_CMD);  /* 设置SEG输出电流亮度 */
    OLED_WR_Byte(0xA1, OLED_CMD);  /* 设置SEG/列映射 0xA0左右反置 0xA1正常 */
    OLED_WR_Byte(0xC8, OLED_CMD);  /* 设置COM/行扫描方向 0xC0上下反置 0xC8正常 */
    OLED_WR_Byte(0xA6, OLED_CMD);  /* 设置正常显示 */
    OLED_WR_Byte(0xA8, OLED_CMD);  /* 设置多路复用比(1 to 64) */
    OLED_WR_Byte(0x3f, OLED_CMD);  /* 1/64 duty */
    OLED_WR_Byte(0xD3, OLED_CMD);  /* 设置显示偏移 (0x00~0x3F) */
    OLED_WR_Byte(0x00, OLED_CMD);  /* 无偏移 */
    OLED_WR_Byte(0xd5, OLED_CMD);  /* 设置显示时钟分频比/振荡器频率 */
    OLED_WR_Byte(0x80, OLED_CMD);  /* 设置分频比，100 Frames/Sec */
    OLED_WR_Byte(0xD9, OLED_CMD);  /* 设置预充电周期 */
    OLED_WR_Byte(0xF1, OLED_CMD);  /* 预充电15个时钟，放电1个时钟 */
    OLED_WR_Byte(0xDA, OLED_CMD);  /* 设置COM引脚硬件配置 */
    OLED_WR_Byte(0x12, OLED_CMD);
    OLED_WR_Byte(0xDB, OLED_CMD);  /* 设置VCOMH */
    OLED_WR_Byte(0x40, OLED_CMD);  /* 设置VCOM取消选择电平 */
    OLED_WR_Byte(0x20, OLED_CMD);  /* 设置页寻址模式 (0x00/0x01/0x02) */
    OLED_WR_Byte(0x02, OLED_CMD);
    OLED_WR_Byte(0x8D, OLED_CMD);  /* 设置电荷泵使能/禁用 */
    OLED_WR_Byte(0x14, OLED_CMD);  /* 使能电荷泵 (0x10=禁用) */
    OLED_WR_Byte(0xA4, OLED_CMD);  /* 禁用全屏显示 (0xA4/0xA5) */
    OLED_WR_Byte(0xA6, OLED_CMD);  /* 禁用反色显示 (0xA6/0xA7) */
    OLED_WR_Byte(0xAF, OLED_CMD);  /* 开启OLED面板 */

    OLED_WR_Byte(0xAF, OLED_CMD);  /* display ON */
    OLED_Clear();
    OLED_Set_Pos(0, 0);
}