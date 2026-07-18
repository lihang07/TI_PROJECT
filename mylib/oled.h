#ifndef __OLED_H
#define __OLED_H 


#include "stdlib.h"	



/* SCL时钟线控制 - 通过GPIO模拟I2C时钟 */
#define OLED_SCLK_Clr() DL_GPIO_clearPins(I2C_PORT, I2C_SCL_PIN)   /* SCL拉低 */
#define OLED_SCLK_Set() DL_GPIO_setPins(I2C_PORT, I2C_SCL_PIN)       /* SCL拉高 */

/* SDA数据线控制 - 通过GPIO模拟I2C数据 */
#define OLED_SDIN_Clr() DL_GPIO_clearPins(I2C_PORT, I2C_SDA_PIN)   /* SDA拉低 */
#define OLED_SDIN_Set() DL_GPIO_setPins(I2C_PORT, I2C_SDA_PIN)       /* SDA拉高 */

/* ============================================
 * OLED命令/数据模式选择
 * SSD1306通过这一个引脚区分命令和数据:
 *   OLED_CMD  = 0: 表示发送的是命令
 *   OLED_DATA = 1: 表示发送的是数据
 * ============================================ */
#define OLED_CMD  0	 /* 写命令模式 */
#define OLED_DATA 1	 /* 写数据模式 */

/* ============================================
 * 类型定义简化
 * ============================================ */
#define u8 unsigned char    /* 8位无符号整数 */
#define u32 unsigned int    /* 32位无符号整数 */

/* ============================================
 * OLED屏幕操作函数声明
 * ============================================ */

/* OLED_ColorTurn: 设置显示颜色模式
 * 参数: i - 0=正常显示, 1=反色显示
 * 说明: 反色模式下，颜色会翻转，适合某些特殊显示效果 */
void OLED_ColorTurn(u8 i);

/* OLED_DisplayTurn: 设置屏幕显示方向
 * 参数: i - 0=正常显示, 1=翻转180度显示
 * 说明: 翻转模式下，屏幕会上下颠倒显示 */
void OLED_DisplayTurn(u8 i);

/* I2C_Start: 产生I2C起始信号
 * 说明: 在I2C通信开始前，必须产生起始信号
 *       SDA从高变低，SCL保持高电平 */
void I2C_Start(void);

/* I2C_Stop: 产生I2C停止信号
 * 说明: I2C通信结束后产生停止信号
 *       SDA从低变高，SCL保持高电平 */
void I2C_Stop(void);

/* I2C_WaitAck: 等待从机应答信号
 * 说明: 发送完一个字节后，检测从机是否给出应答
 *       软件模拟I2C中，通过时钟脉冲判断从机响应 */
void I2C_WaitAck(void);

/* Send_Byte: 通过I2C发送一个字节
 * 参数: dat - 要发送的8位数据
 * 说明: 从高位到低位依次发送，每位数据在SCL低电平时更新 */
void Send_Byte(u8 dat);

/* OLED_WR_Byte: 向OLED发送一个字节（命令或数据）
 * 参数:
 *   dat  - 要发送的数据
 *   mode - OLED_CMD(命令) 或 OLED_DATA(数据)
 * 说明: 这是OLED最底层的通信函数，所有其他函数都基于此 */
void OLED_WR_Byte(u8 dat,u8 mode);

/* OLED_DisPlay_On: 开启OLED显示
 * 说明: 开启显示后，OLED会显示之前写入显存的内容 */
void OLED_DisPlay_On(void);

/* OLED_DisPlay_Off: 关闭OLED显示
 * 说明: 关闭显示后，屏幕熄灭但显存内容保持不变 */
void OLED_DisPlay_Off(void);

/* OLED_Refresh: 将显存数据刷新到OLED屏幕
 * 说明: SSD1306采用显存分离架构，需要调用此函数
 *       将GRAM数组中的数据实际写入屏幕显示
 *       每帧128x64像素分成8页，每页128字节 */
void OLED_Refresh(void);

/* OLED_Clear: 清屏函数
 * 说明: 将GRAM显存全部清零，然后刷新到屏幕
 *       执行后屏幕变为全黑 */
void OLED_Clear(void);

/* OLED_DrawPoint: 在指定位置画一个点
 * 参数:
 *   x - X坐标(0~127)
 *   y - Y坐标(0~63)
 * 说明: SSD1306屏幕分辨率为128x64，每个像素非黑即白
 *       坐标超出范围时函数会自动忽略 */
void OLED_DrawPoint(u8 x,u8 y);

/* OLED_ClearPoint: 清除指定位置的点
 * 参数:
 *   x - X坐标(0~127)
 *   y - Y坐标(0~63)
 * 说明: 将指定坐标的像素设置为黑色 */
void OLED_ClearPoint(u8 x,u8 y);

/* OLED_DrawLine: 在两点之间画直线
 * 参数:
 *   x1, y1 - 起点坐标
 *   x2, y2 - 终点坐标
 * 说明: 支持水平线、垂直线和斜线
 *       使用Bresenham算法绘制 */
void OLED_DrawLine(u8 x1,u8 y1,u8 x2,u8 y2);

/* OLED_DrawCircle: 画圆函数
 * 参数:
 *   x - 圆心X坐标
 *   y - 圆心Y坐标
 *   r - 圆的半径
 * 说明: 使用八分圆法绘制，先画1/8圆弧然后对称复制 */
void OLED_DrawCircle(u8 x,u8 y,u8 r);

/* OLED_ShowChar: 在指定位置显示一个字符
 * 参数:
 *   x     - X坐标(0~127)
 *   y     - Y坐标(0~63)
 *   chr   - 要显示的字符(ASCII码)
 *   size1 - 字体大小，支持12/16/24三种字号
 * 说明: 根据字号从字库数组中取出点阵数据，逐像素绘制
 *       字符之间自动留有适当间距 */
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size1);

/* OLED_ShowString: 显示字符串
 * 参数:
 *   x     - 起始X坐标
 *   y     - 起始Y坐标
 *   chr   - 字符串指针
 *   size1 - 字体大小
 * 说明: 自动调用OLED_ShowChar逐字符显示
 *       当字符串超出屏幕右边界时会自动换行 */
void OLED_ShowString(u8 x,u8 y,u8 *chr,u8 size1);

/* OLED_ShowNum: 显示数字
 * 参数:
 *   x,y   - 显示位置
 *   num   - 要显示的数字(无符号32位整数)
 *   len   - 数字的位数
 *   size1 - 字体大小
 * 说明: 将数字分解为各位数字后逐个显示
 *       例如显示123，长度为3 */
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size1);

/* OLED_ShowChinese: 显示汉字
 * 参数:
 *   x     - X坐标
 *   y     - Y坐标
 *   num   - 汉字在字库中的索引号
 *   size1 - 字体大小(16/24/32/64)
 * 说明: 从汉字点阵库Hzk中取出对应字模数据
 *       汉字占2个字节位置(宽为size1，高为size1/8*8) */
void OLED_ShowChinese(u8 x,u8 y,u8 num,u8 size1);

/* OLED_ScrollDisplay: 横向滚动显示多个汉字
 * 参数:
 *   num   - 要显示的汉字个数
 *   space - 每次滚动的间隔时间
 * 说明: 用于显示长字符串，汉字会从右向左滚动进入屏幕 */
void OLED_ScrollDisplay(u8 num,u8 space);

/* OLED_WR_BP: 设置后续数据写入的起始位置
 * 参数:
 *   x - 列地址(0~127)
 *   y - 页地址(0~7)
 * 说明: 这是页寻址模式下的坐标设置函数
 *       用于OLED_ShowPicture中配置图片显示位置 */
void OLED_WR_BP(u8 x,u8 y);

/* OLED_ShowPicture: 显示图片
 * 参数:
 *   x0, y0 - 图片左上角坐标
 *   x1, y1 - 图片右下角坐标
 *   BMP[]  - 图片点阵数据数组
 * 说明: 图片以单色点阵形式存储，每字节代表8个像素列
 *       图片数据需要提前转换为OLED支持的格式 */
void OLED_ShowPicture(u8 x0,u8 y0,u8 x1,u8 y1,u8 BMP[]);

/* OLED_ShowFloat: 显示浮点数
 * 参数:
 *   x,y     - 显示位置
 *   num     - 要显示的浮点数
 *   int_len - 整数部分的位数
 *   dec_len - 小数部分的位数
 *   size1   - 字体大小
 * 示例: OLED_ShowFloat(0, 0, 3.1415, 2, 3, 16) 显示 "3.141"
 * 说明: 浮点数会被拆分为整数部分和小数部分分别显示
 *       整数部分显示完整的int_len位(不足左边补0)
 *       小数部分显示dec_len位(四舍五入) */
void OLED_ShowFloat(u8 x, u8 y, float num, u8 int_len, u8 dec_len, u8 size1);

/* OLED_Init: OLED屏幕初始化
 * 说明: 配置SSD1306控制器的所有必要参数
 *       必须在使用OLED其他函数之前调用
 *       初始化流程包括: 关闭显示→设置时钟→设置显存模式→开启电荷泵→清屏 */
void OLED_Init(void);

#endif