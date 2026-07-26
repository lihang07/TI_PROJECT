#ifndef __ICM42688_H__
#define __ICM42688_H__

/* ICM42688 6-axis IMU Driver for MSPM0G3507 (I2C)
 * ================================================
 * ICM42688是一款高性能6轴惯性测量单元(IMU),
 * 集成了3轴加速度计和3轴陀螺仪
 */
#include <stdint.h>
/* ============================================
 * I2C通讯配置
 * ============================================ */
#define ICM42688_I2C_ADDR                 0x69U  /* ICM42688的I2C从机地址(AD0接高电平) */

/* ============================================
 * 寄存器地址定义 - Bank 0
 * ICM42688有多个寄存器bank,不同bank存放不同功能的寄存器
 * ============================================ */
#define ICM42688_REG_DEVICE_CONFIG        0x11U  /* 设备配置寄存器 */
#define ICM42688_REG_DRIVE_CONFIG         0x13U  /* IO驱动配置 */
#define ICM42688_REG_INT_CONFIG           0x14U  /* 中断配置 */
#define ICM42688_REG_FIFO_CONFIG          0x16U  /* FIFO配置 */
#define ICM42688_REG_TEMP_DATA1           0x1DU  /* 温度数据高位 */
#define ICM42688_REG_TEMP_DATA0           0x1EU  /* 温度数据低位 */
#define ICM42688_REG_ACCEL_DATA_X1        0x1FU  /* 加速度X轴数据高位 */
#define ICM42688_REG_ACCEL_DATA_X0        0x20U  /* 加速度X轴数据低位 */
#define ICM42688_REG_ACCEL_DATA_Y1        0x21U  /* 加速度Y轴数据高位 */
#define ICM42688_REG_ACCEL_DATA_Y0        0x22U  /* 加速度Y轴数据低位 */
#define ICM42688_REG_ACCEL_DATA_Z1        0x23U  /* 加速度Z轴数据高位 */
#define ICM42688_REG_ACCEL_DATA_Z0        0x24U  /* 加速度Z轴数据低位 */
#define ICM42688_REG_GYRO_DATA_X1         0x25U  /* 陀螺仪X轴数据高位 */
#define ICM42688_REG_GYRO_DATA_X0         0x26U  /* 陀螺仪X轴数据低位 */
#define ICM42688_REG_GYRO_DATA_Y1         0x27U  /* 陀螺仪Y轴数据高位 */
#define ICM42688_REG_GYRO_DATA_Y0         0x28U  /* 陀螺仪Y轴数据低位 */
#define ICM42688_REG_GYRO_DATA_Z1         0x29U  /* 陀螺仪Z轴数据高位 */
#define ICM42688_REG_GYRO_DATA_Z0         0x2AU  /* 陀螺仪Z轴数据低位 */
#define ICM42688_REG_TMST_FSYNCH          0x2BU  /* 时间戳配置 */
#define ICM42688_REG_TMST_FSYNCL          0x2CU  /* 时间戳数据 */
#define ICM42688_REG_INT_STATUS           0x2DU  /* 中断状态寄存器 */
#define ICM42688_REG_FIFO_COUNTH          0x2EU  /* FIFO计数高位 */
#define ICM42688_REG_FIFO_COUNTL          0x2FU  /* FIFO计数低位 */
#define ICM42688_REG_FIFO_DATA            0x30U  /* FIFO数据端口 */
#define ICM42688_REG_SIGNAL_PATH_RESET    0x4BU  /* 信号路径复位 */
#define ICM42688_REG_INTF_CONFIG0         0x4CU  /* 接口配置0 */
#define ICM42688_REG_INTF_CONFIG1         0x4DU  /* 接口配置1 */
#define ICM42688_REG_PWR_MGMT0            0x4EU  /* 电源管理寄存器 - 非常重要! */
#define ICM42688_REG_GYRO_CONFIG0         0x4FU  /* 陀螺仪配置 - 量程和ODR */
#define ICM42688_REG_ACCEL_CONFIG0        0x50U  /* 加速度计配置 - 量程和ODR */
#define ICM42688_REG_GYRO_CONFIG1         0x51U  /* 陀螺仪配置1 */
#define ICM42688_REG_GYRO_ACCEL_CONFIG0   0x52U  /* 陀螺仪和加速度计联合配置 */
#define ICM42688_REG_ACCEL_CONFIG1        0x53U  /* 加速度计配置1 */
#define ICM42688_REG_TMST_CONFIG          0x54U  /* 时间戳配置 */
#define ICM42688_REG_APEX_CONFIG0         0x56U  /* APEX(高级功能)配置0 */
#define ICM42688_REG_SMD_CONFIG           0x57U  /* 运动检测配置 */
#define ICM42688_REG_FIFO_CONFIG1         0x5FU  /* FIFO配置1 */
#define ICM42688_REG_FIFO_CONFIG2         0x60U  /* FIFO配置2 */
#define ICM42688_REG_FIFO_CONFIG3         0x61U  /* FIFO配置3 */
#define ICM42688_REG_FSYNC_CONFIG         0x62U  /* 帧同步配置 */
#define ICM42688_REG_INT_CONFIG0          0x63U  /* 中断配置0 */
#define ICM42688_REG_INT_CONFIG1          0x64U  /* 中断配置1 */
#define ICM42688_REG_INT_SOURCE0          0x65U  /* 中断源配置0 */
#define ICM42688_REG_INT_SOURCE1          0x66U  /* 中断源配置1 */
#define ICM42688_REG_INT_SOURCE3          0x68U  /* 中断源配置3 */
#define ICM42688_REG_INT_SOURCE4          0x69U  /* 中断源配置4 */
#define ICM42688_REG_FIFO_LOST_PKT0       0x6CU  /* FIFO丢失数据包计数 */
#define ICM42688_REG_FIFO_LOST_PKT1       0x6DU  /* FIFO丢失数据包计数 */
#define ICM42688_REG_SELF_TEST_CONFIG     0x70U  /* 自检配置 */
#define ICM42688_REG_WHO_AM_I             0x75U  /* 器件ID寄存器 - 用于验证通信 */
#define ICM42688_REG_BANK_SEL             0x76U  /* 寄存器Bank选择寄存器 */

/* ============================================
 * 寄存器地址定义 - Bank 1
 * ============================================ */
#define ICM42688_REG_SENSOR_CONFIG0       0x03U  /* 传感器配置 */
#define ICM42688_REG_GYRO_CONFIG_STATIC2  0x0BU  /* 陀螺仪静态配置2 */
#define ICM42688_REG_GYRO_CONFIG_STATIC3  0x0CU  /* 陀螺仪静态配置3 */
#define ICM42688_REG_GYRO_CONFIG_STATIC4  0x0DU  /* 陀螺仪静态配置4 */
#define ICM42688_REG_GYRO_CONFIG_STATIC5  0x0EU  /* 陀螺仪静态配置5 */
#define ICM42688_REG_GYRO_CONFIG_STATIC6  0x0FU  /* 陀螺仪静态配置6 */
#define ICM42688_REG_GYRO_CONFIG_STATIC7  0x10U  /* 陀螺仪静态配置7 */
#define ICM42688_REG_GYRO_CONFIG_STATIC8  0x11U  /* 陀螺仪静态配置8 */
#define ICM42688_REG_GYRO_CONFIG_STATIC9  0x12U  /* 陀螺仪静态配置9 */
#define ICM42688_REG_GYRO_CONFIG_STATIC10 0x13U /* 陀螺仪静态配置10 */
#define ICM42688_REG_XG_ST_DATA           0x5FU  /* X轴陀螺仪自检数据 */
#define ICM42688_REG_YG_ST_DATA           0x60U  /* Y轴陀螺仪自检数据 */
#define ICM42688_REG_ZG_ST_DATA           0x61U  /* Z轴陀螺仪自检数据 */
#define ICM42688_REG_TMSTVAL0             0x62U  /* 时间戳值0 */
#define ICM42688_REG_TMSTVAL1             0x63U  /* 时间戳值1 */
#define ICM42688_REG_TMSTVAL2             0x64U  /* 时间戳值2 */
#define ICM42688_REG_INTF_CONFIG4         0x7AU  /* 接口配置4 */
#define ICM42688_REG_INTF_CONFIG5         0x7BU  /* 接口配置5 */
#define ICM42688_REG_INTF_CONFIG6         0x7CU  /* 接口配置6 */

/* ============================================
 * 寄存器地址定义 - Bank 2
 * ============================================ */
#define ICM42688_REG_ACCEL_CONFIG_STATIC2 0x03U  /* 加速度计静态配置2 */
#define ICM42688_REG_ACCEL_CONFIG_STATIC3 0x04U  /* 加速度计静态配置3 */
#define ICM42688_REG_ACCEL_CONFIG_STATIC4 0x05U  /* 加速度计静态配置4 */
#define ICM42688_REG_XA_ST_DATA           0x3BU  /* X轴加速度计自检数据 */
#define ICM42688_REG_YA_ST_DATA           0x3CU  /* Y轴加速度计自检数据 */
#define ICM42688_REG_ZA_ST_DATA           0x3DU  /* Z轴加速度计自检数据 */

/* ============================================
 * 寄存器地址定义 - Bank 4
 * ============================================ */
#define ICM42688_REG_GYRO_ON_OFF_CONFIG   0x0EU  /* 陀螺仪开关配置 */
#define ICM42688_REG_APEX_CONFIG1         0x40U  /* APEX配置1 */
#define ICM42688_REG_APEX_CONFIG9         0x48U  /* APEX配置9 */
#define ICM42688_REG_OFFSET_USER0         0x77U  /* 用户偏移量0 */
#define ICM42688_REG_OFFSET_USER8         0x7FU  /* 用户偏移量8 */

/* ============================================
 * 器件ID和常量
 * ============================================ */
#define ICM42688_WHO_AM_I_VALUE           0x47U  /* ICM42688的WHO_AM_I寄存器值,用于验证 */

/* ============================================
 * 加速度计量程(FS - Full Scale)配置
 * 加速度计输出范围: ±2g, ±4g, ±8g, ±16g
 * ============================================ */
#define ICM42688_ACCEL_FS_16G             0x00U  /* ±16g - 适用于高振动环境 */
#define ICM42688_ACCEL_FS_8G              0x01U  /* ±8g  - 适用于一般应用 */
#define ICM42688_ACCEL_FS_4G              0x02U  /* ±4g  - 适用于姿态检测(默认) */
#define ICM42688_ACCEL_FS_2G              0x03U  /* ±2g  - 适用于精确测量 */

/* ============================================
 * 加速度计输出数据速率(ODR - Output Data Rate)配置
 * 决定传感器多久更新一次数据
 * ============================================ */
#define ICM42688_ACCEL_ODR_8000HZ         0x03U  /* 8000Hz - 超高速采样 */
#define ICM42688_ACCEL_ODR_1000HZ         0x06U  /* 1000Hz - 高速采样 */
#define ICM42688_ACCEL_ODR_200HZ          0x07U  /* 200Hz  - 中等速率 */
#define ICM42688_ACCEL_ODR_100HZ          0x08U  /* 100Hz  - 标准速率(本项目使用) */
#define ICM42688_ACCEL_ODR_50HZ           0x09U  /* 50Hz   - 低功耗 */
#define ICM42688_ACCEL_ODR_25HZ           0x0AU  /* 25Hz   - 更低功耗 */
#define ICM42688_ACCEL_ODR_12_5HZ         0x0BU  /* 12.5Hz - 极低功耗 */
#define ICM42688_ACCEL_ODR_1_5625HZ       0x0EU  /* 1.56Hz - 最低功耗 */
#define ICM42688_ACCEL_ODR_500HZ          0x0FU  /* 500Hz  - 中高速 */

/* ============================================
 * 陀螺仪量程(FS - Full Scale)配置
 * 陀螺仪输出范围: ±15.125dps ~ ±2000dps
 * ============================================ */
#define ICM42688_GYRO_FS_2000DPS          0x00U  /* ±2000dps - 适用于高速旋转应用 */
#define ICM42688_GYRO_FS_1000DPS          0x01U  /* ±1000dps - 适用于一般应用(本项目使用) */
#define ICM42688_GYRO_FS_500DPS           0x02U  /* ±500dps  - 适用于姿态检测 */
#define ICM42688_GYRO_FS_250DPS           0x03U  /* ±250dps  - 适用于精确测量 */
#define ICM42688_GYRO_FS_125DPS           0x04U  /* ±125dps  - 高精度应用 */
#define ICM42688_GYRO_FS_62_5DPS          0x05U  /* ±62.5dps - 超高精度 */
#define ICM42688_GYRO_FS_31_25DPS         0x06U  /* ±31.25dps */
#define ICM42688_GYRO_FS_15_125DPS        0x07U  /* ±15.125dps - 最高精度 */

/* ============================================
 * 陀螺仪输出数据速率(ODR)配置
 * ============================================ */
#define ICM42688_GYRO_ODR_8000HZ          0x03U  /* 8000Hz */
#define ICM42688_GYRO_ODR_1000HZ          0x06U  /* 1000Hz */
#define ICM42688_GYRO_ODR_200HZ           0x07U  /* 200Hz */
#define ICM42688_GYRO_ODR_100HZ           0x08U  /* 100Hz - 本项目使用 */
#define ICM42688_GYRO_ODR_50HZ            0x09U  /* 50Hz */
#define ICM42688_GYRO_ODR_25HZ            0x0AU  /* 25Hz */
#define ICM42688_GYRO_ODR_12_5HZ          0x0BU  /* 12.5Hz */
#define ICM42688_GYRO_ODR_500HZ           0x0FU  /* 500Hz */

/* ============================================
 * 电源模式定义
 * ============================================ */
#define ICM42688_MODE_OFF                 0x00U  /* 关闭 - 最低功耗 */
#define ICM42688_MODE_STANDBY             0x01U  /* 待机 - 快速唤醒 */
#define ICM42688_MODE_LOW_POWER           0x02U  /* 低功耗模式 - 省电但噪声较大 */
#define ICM42688_MODE_LOW_NOISE           0x03U  /* 低噪声模式 - 最佳精度(本项目使用) */
#define ICM42688_PWR_GYRO_MODE_SHIFT      2U     /* 陀螺仪电源模式在寄存器中的位偏移 */
#define ICM42688_PWR_ACCEL_MODE_SHIFT     0U     /* 加速度计电源模式在寄存器中的位偏移 */

/* ============================================
 * 寄存器Bank选择
 * ICM42688有多个寄存器bank,通过BANK_SEL寄存器切换
 * ============================================ */
#define ICM42688_BANK_0                   0x00U  /* Bank 0 - 最常用,存放主要配置和数据寄存器 */
#define ICM42688_BANK_1                   0x01U  /* Bank 1 - 陀螺仪自检和校准相关 */
#define ICM42688_BANK_2                   0x02U  /* Bank 2 - 加速度计自检和校准相关 */
#define ICM42688_BANK_4                   0x04U  /* Bank 4 - APEX功能配置 */

/* ============================================
 * 数据结构定义
 * ============================================ */
/* 原始传感器数据 - 16位有符号整型
 * 加速度计和陀螺仪输出的原始ADC值,需要转换为物理单位 */
typedef struct { int16_t x; int16_t y; int16_t z; } icm42688_raw_data_t;

/* 实际物理数据 - 单精度浮点型
 * 转换后的加速度(g)和角速度(dps)值 */
typedef struct { float   x; float   y; float   z; } icm42688_real_data_t;

/* ============================================
 * 公共API函数声明
 * ============================================ */

/* ICM42688_Init: 初始化ICM42688传感器
 * 返回值: 0=成功, -1=失败(WHO_AM_I验证失败)
 * 初始化流程:
 *   1. 软件复位
 *   2. 验证器件ID(WHO_AM_I)
 *   3. 配置加速度计量程±4g, ODR=100Hz
 *   4. 配置陀螺仪量程±1000dps, ODR=100Hz
 *   5. 设置为低噪声模式
 */
int8_t ICM42688_Init(void);

/* ICM42688_SetAccelConfig: 配置加速度计
 * 参数:
 *   fs  - 量程选择,使用ACCEL_FS_xx宏定义
 *   odr - 输出数据速率,使用ACCEL_ODR_xx宏定义
 */
void   ICM42688_SetAccelConfig(uint8_t fs, uint8_t odr);

/* ICM42688_SetGyroConfig: 配置陀螺仪
 * 参数:
 *   fs  - 量程选择,使用GYRO_FS_xx宏定义
 *   odr - 输出数据速率,使用GYRO_ODR_xx宏定义
 */
void   ICM42688_SetGyroConfig(uint8_t fs, uint8_t odr);

/* ICM42688_ReadAccelRaw: 读取加速度计原始ADC值
 * 参数: accData - 存储加速度原始数据(x,y,z各为16位有符号整数)
 * 返回值: 0=成功
 */
int8_t ICM42688_ReadAccelRaw(icm42688_raw_data_t *accData);

/* ICM42688_ReadGyroRaw: 读取陀螺仪原始ADC值
 * 参数: gyroData - 存储陀螺仪原始数据(x,y,z各为16位有符号整数)
 * 返回值: 0=成功
 */
int8_t ICM42688_ReadGyroRaw(icm42688_raw_data_t *gyroData);

/* ICM42688_ReadTemperature: 读取芯片内部温度传感器
 * 参数: tempCelsius - 存储温度值(摄氏度)
 * 返回值: 0=成功
 * 注意: 温度传感器主要用于校准补偿,不是环境温度测量
 */
int8_t ICM42688_ReadTemperature(float *tempCelsius);

/* ICM42688_ReadMotion6: 同时读取加速度计和陀螺仪数据(推荐使用)
 * 参数:
 *   accData - 加速度数据输出(物理单位g)
 *   gyroData - 陀螺仪数据输出(物理单位dps)
 * 返回值: 0=成功
 * 优点: 使用burst read一次性获取所有数据,效率高且保证数据同步性
 */
int8_t ICM42688_ReadMotion6(icm42688_real_data_t *accData,
                             icm42688_real_data_t *gyroData);

/* ICM42688_AccelToFloat: 将加速度原始ADC值转换为物理单位(g)
 * 参数:
 *   raw  - 原始ADC值
 *   real - 转换后的物理值(g)
 * 转换公式: real_value = raw_value * sensitivity
 *   其中sensitivity取决于所选的量程
 */
void   ICM42688_AccelToFloat(const icm42688_raw_data_t *raw,
                              icm42688_real_data_t *real);

/* ICM42688_GyroToFloat: 将陀螺仪原始ADC值转换为物理单位(dps - degrees per second)
 * 参数:
 *   raw  - 原始ADC值
 *   real - 转换后的物理值(dps)
 */
void   ICM42688_GyroToFloat(const icm42688_raw_data_t *raw,
                             icm42688_real_data_t *real);

/* ICM42688_ReadReg: 读取指定bank的寄存器
 * 参数:
 *   reg  - 寄存器地址
 *   bank - 寄存器bank(使用BANK_xx宏定义)
 * 返回值: 寄存器值
 */
uint8_t ICM42688_ReadReg(uint8_t reg, uint8_t bank);

/* ICM42688_WriteReg: 写入指定bank的寄存器
 * 参数:
 *   reg   - 寄存器地址
 *   bank  - 寄存器bank
 *   value - 要写入的值
 */
void   ICM42688_WriteReg(uint8_t reg, uint8_t bank, uint8_t value);

#endif /* __ICM42688_H__ */