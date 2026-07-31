#include "mylib/oled.h"
#include "stdlib.h"
#include "mylib/oledfont.h"  	 
#include "mylib/delay.h"

/* ============================================
 * OLED显存数组
 * SSD1306屏幕分辨率为128x64像素
 * 显存结构: 144列 x 8页，每页8行像素
 * 
 * OLED_GRAM[x][y] 表示第x列、第y页的一个字节
 * 每个字节的8位对应8个垂直像素(bit0在最上方)
 * 
 * 寻址方式:
 *   x范围: 0~143 (实际显示只用0~127)
 *   y范围: 0~7   (页模式，每页8行)
 * ============================================ */
u8 OLED_GRAM[144][8];

/* 记录上次显示值，确保OLED最多每秒刷新一次，不拖慢循迹周期。 */
static uint32_t g_oled_last_time_seconds = 0xFFFFFFFFU;
static uint8_t g_oled_last_running = 0xFFU;
static uint32_t g_oled_stopped_time_ms = 0U;

/* ============================================
 * OLED_ColorTurn: 设置显示颜色模式
 * 参数: i - 0=正常显示, 1=反色显示
 * 说明: 
 *   正常模式: 1=白色像素，0=黑色像素
 *   反色模式: 1=黑色像素，0=白色像素
 *   通过设置SSD1306的INVEX寄存器(0xA6/0xA7)实现
 * ============================================ */
void OLED_ColorTurn(u8 i)
{
	if(i==0)
	{
		OLED_WR_Byte(0xA6,OLED_CMD);  /* 0xA6: 正常显示模式 */
	}
	if(i==1)
	{
		OLED_WR_Byte(0xA7,OLED_CMD);  /* 0xA7: 反色显示模式 */
	}
}

/* ============================================
 * OLED_DisplayTurn: 设置屏幕显示方向
 * 参数: i - 0=正常显示, 1=翻转180度显示
 * 说明: 
 *   正常模式: SEG0在左侧，COM0在最上方
 *   翻转模式: SEG0在右侧，COM63在最下方(上下颠倒)
 *   适用于将屏幕安装在设备反方向的情况
 * ============================================ */
void OLED_DisplayTurn(u8 i)
{
	if(i==0)
	{
		OLED_WR_Byte(0xC8,OLED_CMD);  /* 0xC8: 正常COM扫描方向 */
		OLED_WR_Byte(0xA1,OLED_CMD);  /* 0xA1: 正常SEG列方向 */
	}
	if(i==1)
	{
		OLED_WR_Byte(0xC0,OLED_CMD);  /* 0xC0: 反转COM扫描方向 */
		OLED_WR_Byte(0xA0,OLED_CMD);  /* 0xA0: 反转SEG列方向 */
	}
}

/* ============================================
 * I2C_Start: 产生I2C起始信号
 * 说明:
 *   I2C起始条件: SCL高电平时，SDA从高变低
 *   时序:
 *   1. SDA拉高
 *   2. SCL拉高
 *   3. 延时满足setup time
 *   4. SDA拉低(产生下降沿)
 *   5. 延时满足hold time
 *   6. SCL拉低(准备发送数据)
 * ============================================ */
void I2C_Start(void)
{
	OLED_SDIN_Set();    /* SDA数据线拉高 */
	OLED_SCLK_Set();    /* SCL时钟线拉高 */
	delay_us(2);        /* 等待SDA稳定 */
	OLED_SDIN_Clr();    /* SDA产生下降沿 */
	delay_us(2);        /* 保持起始条件 */
	OLED_SCLK_Clr();    /* SCL拉低，准备发送数据 */
}

/* ============================================
 * I2C_Stop: 产生I2C停止信号
 * 说明:
 *   I2C停止条件: SCL高电平时，SDA从低变高
 *   时序:
 *   1. SCL拉高
 *   2. SDA拉低(准备产生上升沿)
 *   3. 延时
 *   4. SDA拉高(产生上升沿)
 *   5. 结束传输
 * ============================================ */
void I2C_Stop(void)
{
	OLED_SCLK_Set();    /* SCL时钟线拉高 */
	delay_us(2);        /* 建立时间 */
	OLED_SDIN_Clr();    /* SDA数据线拉低 */
	delay_us(2);        /* 停止条件保持时间 */
	OLED_SDIN_Set();    /* SDA产生上升沿 */
}

/* ============================================
 * I2C_WaitAck: 等待从机应答信号
 * 说明:
 *   软件模拟I2C的应答检测
 *   从机收到一个字节后，会在下一个时钟周期拉低SDA表示应答
 *   本函数产生一个时钟脉冲并检测SDA状态
 *   注意: 简化版本没有实际检测返回值
 * ============================================ */
void I2C_WaitAck(void)
{
	OLED_SCLK_Set();    /* 拉高时钟线 */
	delay_us(2);        /* ACK时钟高电平保持时间 */
	OLED_SCLK_Clr();    /* 拉低时钟线 */
	delay_us(2);        /* ACK时钟低电平保持时间 */
}

/* ============================================
 * Send_Byte: 通过I2C发送一个字节
 * 参数: dat - 要发送的8位数据
 * 说明:
 *   采用MSB first方式，从最高位到最低位依次发送
 *   每个bit的时序:
 *   1. SCL拉低(允许SDA数据变化)
 *   2. 根据发送的bit设置SDA电平
 *   3. 延时等待数据稳定
 *   4. SCL拉高(从机采样SDA)
 *   5. 延时满足时钟高电平时间
 *   6. SCL拉低(准备下一bit)
 * ============================================ */
void Send_Byte(u8 dat)
{
	u8 i;
	for(i=0;i<8;i++)
	{
		OLED_SCLK_Clr();    /* SCL拉低，允许SDA变化 */
		if(dat & 0x80)      /* 发送当前最高位 */
		{
			OLED_SDIN_Set();  /* bit=1时SDA拉高 */
		}
		else
		{
			OLED_SDIN_Clr();  /* bit=0时SDA拉低 */
		}
		delay_us(2);        /* 数据建立时间 */
		OLED_SCLK_Set();    /* SCL产生上升沿，从机采样 */
		delay_us(2);        /* SCL高电平保持时间 */
		OLED_SCLK_Clr();    /* SCL拉低，准备下一bit */
		dat <<= 1;          /* 左移，准备发送下一位 */
	}
}

/* ============================================
 * OLED_WR_Byte: 向OLED发送一个字节
 * 参数:
 *   dat  - 要发送的数据
 *   mode - OLED_CMD(命令) 或 OLED_DATA(数据)
 * 说明:
 *   这是OLED通信的最底层函数
 *   每次发送包含: 起始信号 → 从机地址 → 命令/数据标志 → 数据 → 停止信号
 *   OLED从机地址为0x78(7位地址0x3C，末尾R/W位为0表示写)
 * ============================================ */
void OLED_WR_Byte(u8 dat,u8 mode)
{
	I2C_Start();             /* 产生I2C起始信号 */
	Send_Byte(0x78);         /* 发送OLED从机地址(写操作) */
	I2C_WaitAck();           /* 等待从机应答 */
	
	if(mode)                  /* 判断是命令还是数据 */
	{
		Send_Byte(0x40);     /* 0x40: 后续发送的是数据 */
	}
	else
	{
		Send_Byte(0x00);     /* 0x00: 后续发送的是命令 */
	}
	I2C_WaitAck();           /* 等待从机应答 */
	Send_Byte(dat);          /* 发送实际数据 */
	I2C_WaitAck();           /* 等待从机应答 */
	I2C_Stop();              /* 产生I2C停止信号 */
}

/* ============================================
 * OLED_DisPlay_On: 开启OLED显示
 * 说明:
 *   1. 先开启电荷泵(0x8D)
 *   2. 使能电荷泵输出(0x14)
 *   3. 发送显示开启命令(0xAF)
 *   注意: 必须先开启电荷泵，否则屏幕不会亮
 * ============================================ */
void OLED_DisPlay_On(void)
{
	OLED_WR_Byte(0x8D,OLED_CMD);  /* 0x8D: 设置电荷泵 */
	OLED_WR_Byte(0x14,OLED_CMD);  /* 0x14: 开启电荷泵 */
	OLED_WR_Byte(0xAF,OLED_CMD);  /* 0xAF: 开启显示 */
}

/* ============================================
 * OLED_DisPlay_Off: 关闭OLED显示
 * 说明:
 *   关闭显示后屏幕熄灭，但GRAM显存内容保持不变
 *   适合实现息屏显示等省电功能
 * ============================================ */
void OLED_DisPlay_Off(void)
{
	OLED_WR_Byte(0x8D,OLED_CMD);  /* 0x8D: 设置电荷泵 */
	OLED_WR_Byte(0x10,OLED_CMD);  /* 0x10: 关闭电荷泵 */
	OLED_WR_Byte(0xAF,OLED_CMD);  /* 0xAF: 关闭显示 */
}

/* ============================================
 * OLED_Refresh: 将显存数据刷新到OLED屏幕
 * 说明:
 *   SSD1306采用显存分离架构:
 *   程序中维护一个GRAM数组(软件显存)
 *   调用此函数将GRAM数据通过I2C发送到OLED
 *   
 *   SSD1306使用页寻址模式:
 *   - 屏幕分为8页(每页8行像素)
 *   - 每页有128列
 *   - 设置页地址和列地址后，连续写入128字节数据
 *   
 *   刷新流程:
 *   1. 设置起始页(0xB0~0xB7)
 *   2. 设置起始列低4位(0x00~0x0F)
 *   3. 设置起始列高4位(0x10~0x1F)
 *   4. 连续写入128字节数据(一次刷新一整页)
 * ============================================ */
void OLED_Refresh(void)
{
	u8 i,n;
	for(i=0;i<8;i++)           /* 遍历8页 */
	{
		OLED_WR_Byte(0xB0+i,OLED_CMD);  /* 设置页地址: 0xB0~0xB7 */
		OLED_WR_Byte(0x00,OLED_CMD);    /* 设置列地址低4位: 0x00 */
		OLED_WR_Byte(0x10,OLED_CMD);    /* 设置列地址高4位: 0x10 */
		
		for(n=0;n<128;n++)             /* 写入一整页(128列)数据 */
		{
			OLED_WR_Byte(OLED_GRAM[n][i],OLED_DATA);
		}
	}
}

/* ============================================
 * OLED_Clear: 清屏函数
 * 说明:
 *   将OLED_GRAM显存数组全部清零
 *   然后调用OLED_Refresh将清空的数据刷新到屏幕
 *   执行后屏幕变为全黑状态
 * ============================================ */
void OLED_Clear(void)
{
	u8 i,n;
	for(i=0;i<8;i++)           /* 遍历8页 */
	{
		for(n=0;n<128;n++)     /* 遍历128列 */
		{
			OLED_GRAM[n][i]=0;  /* 清空当前字节 */
		}
	}
	OLED_Refresh();            /* 刷新到屏幕 */
}

/* ============================================
 * OLED_DrawPoint: 在指定位置画一个点
 * 参数:
 *   x - X坐标(0~127)
 *   y - Y坐标(0~63)
 * 说明:
 *   SSD1306每个像素非黑即白(1=亮，0=灭)
 *   显存按页存储，每字节8个垂直像素
 *   坐标转换:
 *   - y坐标除以8得到页号(i)
 *   - y坐标模8得到在该页内的位号(m)
 *   - 使用1<<m创建一个只在目标位为1的掩码
 * ============================================ */
void OLED_DrawPoint(u8 x,u8 y)
{
	u8 i,m,n;
	i = y/8;       /* 计算y坐标所在的页号(0~7) */
	m = y%8;       /* 计算在该页内的位号(0~7) */
	n = 1<<m;      /* 创建掩码，只有第m位为1 */
	OLED_GRAM[x][i] |= n;  /* 将该位置1(点亮) */
}

/* ============================================
 * OLED_ClearPoint: 清除指定位置的点
 * 参数:
 *   x - X坐标(0~127)
 *   y - Y坐标(0~63)
 * 说明:
 *   将指定像素设置为黑色
 *   使用取反-或-取反的方式，只影响目标位而不改变其他位
 *   原理: (~n) & OLED_GRAM[x][i] 会将第m位清零
 * ============================================ */
void OLED_ClearPoint(u8 x,u8 y)
{
	u8 i,m,n;
	i = y/8;       /* 计算页号 */
	m = y%8;       /* 计算位号 */
	n = 1<<m;      /* 创建掩码 */
	OLED_GRAM[x][i] = ~OLED_GRAM[x][i];  /* 先取反 */
	OLED_GRAM[x][i] |= n;                /* 目标位置1(其他位变0) */
	OLED_GRAM[x][i] = ~OLED_GRAM[x][i];  /* 再取反，结果是目标位清零 */
}

/* ============================================
 * OLED_DrawLine: 在两点之间画直线
 * 参数:
 *   x1, y1 - 起点坐标
 *   x2, y2 - 终点坐标
 * 说明:
 *   支持三种直线:
 *   - 垂直线: x1==x2
 *   - 水平线: y1==y2
 *   - 斜线: 其他情况，使用Bresenham算法
 *   
 *   Bresenham算法原理:
 *   每列选择一个像素点，通过误差累积决定y是否递增
 *   k = dy*10/dx 表示每步y的增量
 * ============================================ */
void OLED_DrawLine(u8 x1,u8 y1,u8 x2,u8 y2)
{
	u8 i,k,k1,k2,t;
	
	/* 边界检查: 坐标超出屏幕范围则忽略 */
	if((x1<0)||(x2>128)||(y1<0)||(y2>64)||(x1>x2)||(y1>y2)) return;
	
	if(x1==x2)    /* 垂直线 */
	{
		if(y1>y2)  /* 交换y坐标确保从下往上画 */
		{
			t=y1; y1=y2; y2=t;
		}
		for(i=0;i<(y2-y1);i++)
		{
			OLED_DrawPoint(x1,y1+i);
		}
	}
	else if(y1==y2)   /* 水平线 */
	{
		if(x1>x2)  /* 交换x坐标确保从左往右画 */
		{
			t=x1; x1=x2; x2=t;
		}
		for(i=0;i<(x2-x1);i++)
		{
			OLED_DrawPoint(x1+i,y1);
		}
	}
	else      /* 斜线 - 使用Bresenham算法 */
	{
		k1=y2-y1;  /* dy: y方向总增量 */
		k2=x2-x1;  /* dx: x方向总增量 */
		k=k1*10/k2; /* 计算误差累积值 */
		for(i=0;i<(x2-x1);i++)
		{
			OLED_DrawPoint(x1+i,y1+i*k/10);
		}
	}
}

/* ============================================
 * OLED_DrawCircle: 画圆函数
 * 参数:
 *   x - 圆心X坐标
 *   y - 圆心Y坐标
 *   r - 圆的半径
 * 说明:
 *   使用八分圆法(Bresenham圆算法):
 *   1. 先画第一象限的1/8圆弧
 *   2. 利用圆的对称性，通过镜像复制画出整个圆
 *   
 *   八卦弧上的8个对称点:
 *   (x+a, y+b), (x-a, y+b), (x-a, y-b), (x+a, y-b)
 *   (x+b, y+a), (x+b, y-a), (x-b, y-a), (x-b, y+a)
 *   
 *   算法原理:
 *   - 从圆顶(0,r)开始，每次x增加1
 *   - 根据决策参数决定y是否需要递减
 *   - 画点后检查是否超出圆的范围
 * ============================================ */
void OLED_DrawCircle(u8 x,u8 y,u8 r)
{
	int a, b, num;
	a = 0;
	b = r;
	while(2 * b * b >= r * r)   /* 循环直到x超过y的对角线 */
	{
		/* 画8个对称点 */
		OLED_DrawPoint(x + a, y - b);
		OLED_DrawPoint(x - a, y - b);
		OLED_DrawPoint(x - a, y + b);
		OLED_DrawPoint(x + a, y + b);
		
		OLED_DrawPoint(x + b, y + a);
		OLED_DrawPoint(x + b, y - a);
		OLED_DrawPoint(x - b, y - a);
		OLED_DrawPoint(x - b, y + a);
		
		a++;   /* x坐标递增 */
		/* 计算当前点到圆心的平方距离与半径平方的差 */
		num = (a * a + b * b) - r * r;
		if(num > 0)   /* 如果距离大于半径平方，y需要递减 */
		{
			b--;     /* y递减 */
			a--;     /* 修正x的回退 */
		}
	}
}

/* ============================================
 * OLED_ShowChar: 在指定位置显示一个字符
 * 参数:
 *   x     - X坐标(0~127)
 *   y     - Y坐标(0~63)
 *   chr   - 要显示的字符(ASCII码)
 *   size1 - 字体大小，支持12/16/24三种字号
 * 说明:
 *   字符点阵从asc2_1206/asc2_1608/asc2_2412字库数组中获取
 *   每个字符占size1/2×size1字节的点阵数据
 *   例如16号字体: 每个字符占16/2×16=128位=16字节
 *   
 *   显示流程:
 *   1. 计算字符在字库中的索引(chr-' ')
 *   2. 取出每个字节的点阵数据
 *   3. 按位检查，为1画点，为0清除点
 *   4. 逐列扫描直到填满整个字符区域
 * ============================================ */
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size1)
{
	u8 i,m,temp,size2,chr1;
	u8 y0=y;
	
	/* 计算字符点阵占用的字节数
	 * size1/8 向上取整 × size1/2
	 * 例如16号字体: (16/8+(0?1:0))×8 = 16字节 */
	size2 = (size1/8+((size1%8)?1:0))*(size1/2);
	
	/* 计算字符在字库中的索引(ASCII码减去空格(0x20)的偏移) */
	chr1 = chr - ' ';
	
	/* 遍历字符点阵的每个字节 */
	for(i=0;i<size2;i++)
	{
		/* 根据字号选择对应的字库数组 */
		if(size1==12)
		{
			temp = asc2_1206[chr1][i];   /* 12号字体点阵 */
		}
		else if(size1==16)
		{
			temp = asc2_1608[chr1][i];   /* 16号字体点阵 */
		}
		else if(size1==24)
		{
			temp = asc2_2412[chr1][i];   /* 24号字体点阵 */
		}
		else return;   /* 不支持的字号直接返回 */
		
		/* 解析字节中的8个像素(从高位到低位) */
		for(m=0;m<8;m++)
		{
			if(temp & 0x80)   /* 最高位为1，画点 */
			{
				OLED_DrawPoint(x,y);
			}
			else               /* 最高位为0，清点 */
			{
				OLED_ClearPoint(x,y);
			}
			temp <<= 1;        /* 左移，准备检查下一位 */
			y++;               /* y坐标递增(向下移动) */
			
			/* 如果y方向已经填满size1高度，换到下一列 */
			if((y-y0) == size1)
			{
				y = y0;          /* y坐标回到起始位置 */
				x++;             /* x坐标递增(向右移动) */
				break;           /* 跳出当前字节的处理 */
			}
		}
	}
}

/* ============================================
 * OLED_ShowString: 显示字符串
 * 参数:
 *   x     - 起始X坐标
 *   y     - 起始Y坐标
 *   chr   - 字符串指针
 *   size1 - 字体大小
 * 说明:
 *   自动调用OLED_ShowChar逐字符显示
 *   字符宽度为size1/2，每显示一个字符x增加size1/2
 *   当字符串超出屏幕右边界(x>128-size1)时自动换行
 *   只显示ASCII可见字符(空格到波浪号)，其他字符忽略
 * ============================================ */
void OLED_ShowString(u8 x,u8 y,u8 *chr,u8 size1)
{
	while((*chr>=' ')&&(*chr<='~'))   /* 只处理ASCII可见字符 */
	{
		OLED_ShowChar(x,y,*chr,size1);  /* 显示当前字符 */
		x += size1/2;                    /* x移动到下一个字符位置 */
		
		if(x>128-size1)   /* 如果右边界超出屏幕，换行 */
		{
			x = 0;          /* x回到最左 */
			y += 2;         /* y向下移动2行(字体高度方向) */
		}
		chr++;             /* 指向下一个字符 */
	}
}

/* ============================================
 * OLED_Pow: 计算m的n次幂
 * 参数:
 *   m - 底数
 *   n - 指数
 * 返回值: m的n次方结果
 * 说明: 用于OLED_ShowNum中分解数字为各位
 * ============================================ */
u32 OLED_Pow(u8 m,u8 n)
{
	u32 result=1;
	while(n--)
	{
		result *= m;
	}
	return result;
}

/* ============================================
 * OLED_ShowNum: 显示数字
 * 参数:
 *   x,y   - 显示位置
 *   num   - 要显示的数字(无符号32位整数)
 *   len   - 数字的位数
 *   size1 - 字体大小
 * 说明:
 *   将数字分解为各位后逐个显示
 *   从最高位开始，通过除以10的幂取余得到每位数字
 *   例如显示123，长度为3:
 *   - 第一位: 123/100 % 10 = 1
 *   - 第二位: 123/10 % 10 = 2
 *   - 第三位: 123 % 10 = 3
 * ============================================ */
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size1)
{
	u8 t,temp;
	for(t=0;t<len;t++)
	{
		/* 提取当前位的数字 */
		temp = (num / OLED_Pow(10,len-t-1)) % 10;
		
		/* 如果是数字0且不是最高位，显示0 */
		if(temp==0)
		{
			OLED_ShowChar(x+(size1/2)*t,y,'0',size1);
		}
		else
		{
			/* 否则显示实际数字(转换为ASCII码) */
			OLED_ShowChar(x+(size1/2)*t,y,temp+'0',size1);
		}
	}
}

/* ============================================
 * OLED_ShowFloat: 显示浮点数
 * 参数:
 *   x,y     - 显示位置
 *   num     - 要显示的浮点数
 *   int_len - 整数部分的位数
 *   dec_len - 小数部分的位数
 *   size1   - 字体大小
 * 说明:
 *   浮点数被拆分为整数部分和小数部分分别显示
 *   显示格式: [符号][整数部分].[小数部分]
 *   例如: num=3.1415, int_len=2, dec_len=3 -> 显示 "03.141"
 *
 *   处理步骤:
 *   1. 判断正负，负数显示'-'号
 *   2. 对小数部分进行四舍五入
 *   3. 分离整数和小数部分
 *   4. 分别显示整数部分(不足int_len位左边补0)
 *   5. 显示小数点'.'
 *   6. 显示小数部分(不足dec_len位右边补0)
 * ============================================ */
void OLED_ShowFloat(u8 x,u8 y,float num,u8 int_len,u8 dec_len,u8 size1)
{
	u8 i;
	u32 integer_part;
	u32 decimal_part;
	float temp;

	/* 处理负数 */
	if(num < 0)
	{
		num = -num;
		OLED_ShowChar(x, y, '-', size1);
		x += size1/2;
	}

	/* 对小数部分进行四舍五入 */
	temp = num;
	for(i=0; i<dec_len; i++)
	{
		temp *= 10.0f;
	}
	integer_part = (u32)(temp + 0.5f);

	/* 分离整数和小数部分 */
	decimal_part = integer_part % OLED_Pow(10, dec_len);
	integer_part = integer_part / OLED_Pow(10, dec_len);

	/* 显示整数部分(不足int_len位左边补0) */
	OLED_ShowNum(x, y, integer_part, int_len, size1);
	x += (int_len * size1 / 2);

	/* 显示小数点 */
	OLED_ShowChar(x, y, '.', size1);
	x += size1 / 2;

	/* 显示小数部分(不足dec_len位右边补0) */
	OLED_ShowNum(x, y, decimal_part, dec_len, size1);
}

/* ============================================
 * OLED_ShowChinese: 显示汉字
 * 参数:
 *   x     - X坐标
 *   y     - Y坐标
 *   num   - 汉字在字库中的索引号
 *   size1 - 字体大小(16/24/32/64)
 * 说明:
 *   从汉字点阵库Hzk中取出对应字模数据
 *   汉字占2个字节位置(宽度为size1，高度为size1)
 *   
 *   点阵存储格式:
 *   - 汉字点阵按列存储
 *   - 每个字节8个像素(垂直)
 *   - 索引计算: num * (size1/8) + n(当前字节在字中的位置)
 * ============================================ */
void OLED_ShowChinese(u8 x,u8 y,u8 num,u8 size1)
{
	u8 i,m,n=0,temp,chr1;
	u8 x0=x, y0=y;
	u8 size3 = size1/8;   /* 计算汉字占用的字节数(按8行分) */
	
	while(size3--)   /* 遍历汉字点阵的每个字节 */
	{
		chr1 = num * size1/8 + n;   /* 计算当前字节在字库中的索引 */
		n++;
		
		for(i=0;i<size1;i++)   /* 遍历每个字节的8位 */
		{
			/* 根据字号选择对应的汉字字库 */
			if(size1==16)
			{
				temp = Hzk1[chr1][i];   /* 16x16点阵字库 */
			}
			else if(size1==24)
			{
				temp = Hzk2[chr1][i];   /* 24x24点阵字库 */
			}
			else if(size1==32)
			{
				temp = Hzk3[chr1][i];   /* 32x32点阵字库 */
			}
			else if(size1==64)
			{
				temp = Hzk4[chr1][i];   /* 64x64点阵字库 */
			}
			else return;   /* 不支持的字号直接返回 */
			
			/* 按位解析点阵数据 */
			for(m=0;m<8;m++)
			{
				if(temp & 0x01)   /* 最低位为1，画点 */
				{
					OLED_DrawPoint(x,y);
				}
				else               /* 最低位为0，清点 */
				{
					OLED_ClearPoint(x,y);
				}
				temp >>= 1;        /* 右移，准备检查下一位 */
				y++;               /* y向下移动 */
			}
			x++;                   /* x向右移动 */
			
			/* 如果x方向已经填满size1宽度，换到下一行 */
			if((x-x0) == size1)
			{
				x = x0;             /* x回到起始列 */
				y0 = y0 + 8;        /* y移动到下一行(8像素高) */
				y = y0;
			}
		}
	}
}

/* ============================================
 * OLED_ScrollDisplay: 横向滚动显示多个汉字
 * 参数:
 *   num   - 要显示的汉字个数
 *   space - 滚动间隔(像素数)
 * 说明:
 *   用于显示长字符串，汉字会从右向左滚动进入屏幕
 *   滚动原理:
 *   1. 将所有汉字点阵存入GRAM
 *   2. 每次左移1列(整个显存数据左移)
 *   3. 移位后刷新显示
 *   4. 达到间隔后写入下一个汉字
 * ============================================ */
void OLED_ScrollDisplay(u8 num,u8 space)
{
	u8 i,n,t=0,m=0,r;
	
	while(1)
	{
		if(m==0)   /* 当m为0时，表示需要写入一个新汉字 */
		{
			OLED_ShowChinese(128,24,t,16);   /* 在屏幕右侧外写入汉字 */
			t++;   /* 汉字索引递增 */
		}
		
		if(t==num)   /* 所有汉字都已写入 */
		{
			/* 延时space指定的像素移位次数 */
			for(r=0;r<16*space;r++)
			{
				/* 显存整体左移1列 */
				for(i=0;i<144;i++)
				{
					for(n=0;n<8;n++)
					{
						OLED_GRAM[i-1][n] = OLED_GRAM[i][n];
					}
				}
				OLED_Refresh();   /* 刷新显示 */
			}
			t = 0;   /* 重置汉字索引，循环显示 */
		}
		
		m++;   /* 计数器递增 */
		if(m==16){m=0;}   /* m达到16时重置 */
		
		/* 每次循环左移1列 */
		for(i=0;i<144;i++)
		{
			for(n=0;n<8;n++)
			{
				OLED_GRAM[i-1][n] = OLED_GRAM[i][n];
			}
		}
		OLED_Refresh();   /* 刷新显示 */
	}
}

/* ============================================
 * OLED_WR_BP: 设置后续数据写入的起始位置
 * 参数:
 *   x - 列地址(0~127)
 *   y - 页地址(0~7)
 * 说明:
 *   页寻址模式下的坐标设置
 *   设置后，后续写入的数据会从(x,y)开始
 *   主要用于OLED_ShowPicture中配置图片显示位置
 *   
 *   列地址设置分高低两位:
 *   - 0x00~0x0F: 列地址低4位
 *   - 0x10~0x1F: 列地址高4位
 * ============================================ */
void OLED_WR_BP(u8 x,u8 y)
{
	OLED_WR_Byte(0xB0+y,OLED_CMD);                       /* 设置页地址 */
	OLED_WR_Byte(((x&0xF0)>>4)|0x10,OLED_CMD);          /* 设置列地址高4位 */
	OLED_WR_Byte((x&0x0F)|0x01,OLED_CMD);                /* 设置列地址低4位 */
}

/* ============================================
 * OLED_ShowPicture: 显示图片
 * 参数:
 *   x0, y0 - 图片左上角坐标
 *   x1, y1 - 图片右下角坐标
 *   BMP[]  - 图片点阵数据数组
 * 说明:
 *   图片以单色点阵形式存储
 *   每字节8个垂直像素(bit0在最上方)
 *   图片数据需要提前用取模工具转换为OLED兼容格式
 *   
 *   显示流程:
 *   1. 计算图片高度对应的页数
 *   2. 设置起始页和起始列
 *   3. 连续写入图片数据
 * ============================================ */
void OLED_ShowPicture(u8 x0,u8 y0,u8 x1,u8 y1,u8 BMP[])
{
	u32 j=0;
	u8 x=0,y=0;
	
	/* y坐标不是8的倍数时，需要调整起始页
	 * 因为SSD1306按页寻址，y必须对齐到8的边界 */
	if(y0%8==0)
	{
		y = y0/8;   /* 直接计算页号 */
	}
	else
	{
		y = y0/8 + 1;   /* 向上取整 */
	}
	
	/* 按行遍历，从上到下显示 */
	for(y=y0;y<y1;y++)
	{
		OLED_WR_BP(x0,y);   /* 设置当前行的起始位置 */
		
		/* 从左到右写入一行的所有像素 */
		for(x=x0;x<x1;x++)
		{
			OLED_WR_Byte(BMP[j],OLED_DATA);   /* 写入1字节(8个像素) */
			j++;   /* 指向下一个字节 */
		}
	}
}

/* ============================================
 * OLED_Init: OLED屏幕初始化
 * 说明:
 *   配置SSD1306控制器的所有必要参数
 *   必须在使用OLED其他函数之前调用
 *   
 *   初始化流程:
 *   1. 延时200ms等待屏幕上电稳定
 *   2. 关闭显示(防止初始化过程中闪烁)
 *   3. 设置时钟分频和刷新率
 *   4. 设置内存寻址模式
 *   5. 设置屏幕方向(SEG/COM映射)
 *   6. 设置对比度
 *   7. 开启电荷泵
 *   8. 清屏
 *   9. 开启显示
 *   
 *   注意: 注释掉的RST复位代码在本驱动中未使用
 *         因为软件I2C接口不控制OLED的RES引脚
 * ============================================ */
void OLED_Init(void)
{
	/* 延时200ms等待OLED完全上电稳定
	 * 部分OLED模块需要较长的上电时间 */
	delay_ms(200);
	
	/* 以下是SSD1306的初始化命令序列 */
	
	OLED_WR_Byte(0xAE,OLED_CMD);  /* 0xAE: 关闭显示(Display OFF) */
	OLED_WR_Byte(0x00,OLED_CMD);  /* 0x00: 设置列低地址 */
	OLED_WR_Byte(0x10,OLED_CMD);  /* 0x10: 设置列高地址 */
	OLED_WR_Byte(0x40,OLED_CMD);  /* 0x40: 设置显示起始行(RAM地址) */
	OLED_WR_Byte(0x81,OLED_CMD);  /* 0x81: 设置对比度命令 */
	OLED_WR_Byte(0xCF,OLED_CMD);  /* 0xCF: 对比度值(0x00~0xFF) */
	OLED_WR_Byte(0xA1,OLED_CMD);  /* 0xA1: 设置SEG列映射(0xA0=正常, 0xA1=反转) */
	OLED_WR_Byte(0xC8,OLED_CMD);  /* 0xC8: 设置COM扫描方向(0xC0=反向, 0xC8=正常) */
	OLED_WR_Byte(0xA6,OLED_CMD);  /* 0xA6: 设置正常/反色显示(0xA6=正常, 0xA7=反色) */
	OLED_WR_Byte(0xA8,OLED_CMD);  /* 0xA8: 设置多路复用比 */
	OLED_WR_Byte(0x3F,OLED_CMD);  /* 0x3F: 1/64复用比(对应63) */
	OLED_WR_Byte(0xD3,OLED_CMD);  /* 0xD3: 设置显示偏移 */
	OLED_WR_Byte(0x00,OLED_CMD);  /* 0x00: 无偏移 */
	OLED_WR_Byte(0xD5,OLED_CMD);  /* 0xD5: 设置时钟分频 */
	OLED_WR_Byte(0x80,OLED_CMD);  /* 0x80: 分频因子=1, 时钟=100帧/秒 */
	OLED_WR_Byte(0xD9,OLED_CMD);  /* 0xD9: 设置预充电周期 */
	OLED_WR_Byte(0xF1,OLED_CMD);  /* 0xF1: 充电周期=15 clocks, 放电周期=1 clock */
	OLED_WR_Byte(0xDA,OLED_CMD);  /* 0xDA: 设置COM引脚硬件配置 */
	OLED_WR_Byte(0x12,OLED_CMD);  /* 0x12: 备选COM引脚配置 */
	OLED_WR_Byte(0xDB,OLED_CMD);  /* 0xDB: 设置VCOMH电压 */
	OLED_WR_Byte(0x40,OLED_CMD);  /* 0x40: VCOMH = 0.77×VCC */
	OLED_WR_Byte(0x20,OLED_CMD);  /* 0x20: 设置内存寻址模式 */
	OLED_WR_Byte(0x02,OLED_CMD);  /* 0x02: 页寻址模式 */
	OLED_WR_Byte(0x8D,OLED_CMD);  /* 0x8D: 电荷泵设置命令 */
	OLED_WR_Byte(0x14,OLED_CMD);  /* 0x14: 开启电荷泵(0x10=关闭) */
	OLED_WR_Byte(0xA4,OLED_CMD);  /* 0xA4: 关闭全部显示(0xA5=开启) */
	OLED_WR_Byte(0xA6,OLED_CMD);  /* 0xA6: 设置显示模式(0xA6=正常, 0xA7=反色) */
	OLED_WR_Byte(0xAF,OLED_CMD);  /* 0xAF: 开启显示(Display ON) */
	
	OLED_Clear();   /* 清屏，确保初始状态干净 */
}

void OLED_RunTimeReset(void)
{
    g_oled_last_time_seconds = 0xFFFFFFFFU;
    g_oled_last_running = 0xFFU;
    g_oled_stopped_time_ms = 0U;
    OLED_ShowRunTime(0U, 0U);
}

/*
 * 只刷新计时文字占用的一页。
 * 每个字节沿用原OLED_Refresh的发送方式，保证软件I2C应答时序可靠，
 * 避免16像素字体的第二页没有写入而只显示上半个字符。
 */
static void OLED_RefreshRunTimePage(u8 page)
{
    u8 x;

    OLED_WR_Byte((u8)(0xB0U + page), OLED_CMD);
    OLED_WR_Byte(0x00U, OLED_CMD);
    OLED_WR_Byte(0x10U, OLED_CMD);
    for (x = 0U; x < 128U; ++x) {
        OLED_WR_Byte(OLED_GRAM[x][page], OLED_DATA);
    }
}

void OLED_ShowRunTime(uint32_t elapsed_ms, uint8_t running)
{
    uint32_t displayed_seconds;
    uint32_t seconds;

    running = (running != 0U) ? 1U : 0U;

    /* 从运行切换到停止时锁存最终时间，之后保持该结果不再增加。 */
    if (running == 0U) {
        if (g_oled_last_running == 1U) {
            g_oled_stopped_time_ms = elapsed_ms;
        }
        elapsed_ms = g_oled_stopped_time_ms;
    }

    displayed_seconds = elapsed_ms / 1000U;
    if (displayed_seconds == g_oled_last_time_seconds &&
        running == g_oled_last_running) {
        return;
    }

    g_oled_last_time_seconds = displayed_seconds;
    g_oled_last_running = running;
    seconds = displayed_seconds;
    if (seconds > 999U) seconds = 999U;

    OLED_ShowString(0U, 0U, (u8 *)"TIME:", 16U);
    OLED_ShowNum(48U, 0U, seconds, 3U, 16U);
    OLED_ShowChar(72U, 0U, 's', 16U);
    OLED_ShowString(80U, 0U, (u8 *)"   ", 16U);

    if (running != 0U) {
        OLED_ShowString(0U, 24U, (u8 *)"STATE: RUNNING ", 16U);
    } else {
        OLED_ShowString(0U, 24U, (u8 *)"STATE: STOPPED ", 16U);
    }
    OLED_RefreshRunTimePage(0U);
    OLED_RefreshRunTimePage(1U);
    OLED_RefreshRunTimePage(3U);
    OLED_RefreshRunTimePage(4U);
}
