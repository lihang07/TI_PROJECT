#include "icm42688.h"
#include "I2C_communication.h"
#include "ti_msp_dl_config.h"
#include <stdio.h>

/* ============================================
 * 模块级变量(静态全局)
 * ============================================ */
/* gAccelSensitivity: 加速度计灵敏度(LSB/g)
 * 表示1g加速度对应的ADC计数值,用于原始值到物理值的转换
 * 不同量程下该值不同: ±2g=16384, ±4g=8192, ±8g=4096, ±16g=2048 */
static float gAccelSensitivity;

/* gGyroSensitivity: 陀螺仪灵敏度(LSB/dps)
 * 表示1°/s角速度对应的ADC计数值
 * 不同量程下该值不同: ±250dps=131, ±500dps=65.5, ±1000dps=32.8, ±2000dps=16.4 */
static float gGyroSensitivity;

/* gCurrentBank: 当前选中的寄存器Bank
 * 初始值0xFF确保第一次操作会切换bank
 * 用于优化,避免重复切换同一bank */
static uint8_t gCurrentBank = 0xFF;

/* ============================================
 * 内部辅助函数
 * ============================================ */

/* delayUs: 微秒级延时
 * 参数: us - 延时微秒数
 * 通过CPU时钟周期计算实现精确延时 */
static void delayUs(uint32_t us)
{ delay_cycles((CPUCLK_FREQ/1000000UL)*us); }

/* delayMs: 毫秒级延时
 * 参数: ms - 延时毫秒数 */
static void delayMs(uint32_t ms)
{ delay_cycles((CPUCLK_FREQ/1000UL)*ms); }

/* selectBank: 选择ICM42688寄存器Bank
 * 参数: bank - 要选择的bank号(0-4)
 * 说明: ICM42688有多个寄存器bank,需要通过BANK_SEL寄存器切换
 *       为了提高效率,只在切换到不同bank时才执行写操作 */
static int8_t selectBank(uint8_t bank) {
    if (bank != gCurrentBank) {
        int8_t status = I2C_WriteReg(ICM42688_I2C_ADDR,
                                     ICM42688_REG_BANK_SEL, &bank, 1);
        if (status != 0) return status;
        gCurrentBank = bank;
    }
    return 0;
}

/* ICM42688_ReadReg: 读取指定bank的寄存器
 * 参数:
 *   reg  - 寄存器地址(8位)
 *   bank - 寄存器bank号
 * 返回值: 读取到的8位寄存器值
 * 实现: 先切换bank,然后通过I2C读取单个字节 */
uint8_t ICM42688_ReadReg(uint8_t reg, uint8_t bank) {
    uint8_t v=0;
    if (selectBank(bank) != 0) return 0;
    (void)I2C_ReadReg(ICM42688_I2C_ADDR, reg, &v, 1);
    return v;
}

/* ICM42688_WriteReg: 写入指定bank的寄存器
 * 参数:
 *   reg  - 寄存器地址
 *   bank - 寄存器bank号
 *   val  - 要写入的值(8位) */
void ICM42688_WriteReg(uint8_t reg, uint8_t bank, uint8_t val) {
    if (selectBank(bank) != 0) return;
    (void)I2C_WriteReg(ICM42688_I2C_ADDR, reg, &val, 1);
}

/* accelSens: 计算加速度计灵敏度
 * 参数: fs - 量程选择(ACCEL_FS_xx)
 * 返回值: 灵敏度值(LSB/g)
 * 说明: 灵敏度表示1g重力加速度对应的ADC输出值
 *       量程越小,分辨率越高,灵敏度越大 */
static float accelSens(uint8_t fs) {
    switch(fs) {
    case ICM42688_ACCEL_FS_2G:  return 2.0f / 32768.0f;
    case ICM42688_ACCEL_FS_4G:  return 4.0f / 32768.0f;
    case ICM42688_ACCEL_FS_8G:  return 8.0f / 32768.0f;
    default:                    return 16.0f / 32768.0f;
    }
}

/* gyroSens: 计算陀螺仪灵敏度
 * 参数: fs - 量程选择(GYRO_FS_xx)
 * 返回值: 灵敏度值(LSB/dps)
 * 说明: 灵敏度表示1°/s角速度对应的ADC输出值
 *       量程越小,分辨率越高,灵敏度越大 */
static float gyroSens(uint8_t fs) {
    switch(fs) {
    case ICM42688_GYRO_FS_15_125DPS: return 15.125f / 32768.0f;
    case ICM42688_GYRO_FS_31_25DPS:  return 31.25f / 32768.0f;
    case ICM42688_GYRO_FS_62_5DPS:   return 62.5f / 32768.0f;
    case ICM42688_GYRO_FS_125DPS:    return 125.0f / 32768.0f;
    case ICM42688_GYRO_FS_250DPS:    return 250.0f / 32768.0f;
    case ICM42688_GYRO_FS_500DPS:    return 500.0f / 32768.0f;
    case ICM42688_GYRO_FS_1000DPS:   return 1000.0f / 32768.0f;
    default:                          return 2000.0f / 32768.0f;
    }
}

/* ICM42688_SetAccelConfig: 配置加速度计量程和ODR
 * 参数:
 *   fs  - 量程选择
 *   odr - 输出数据速率
 * 实现: 保存灵敏度值,并通过I2C写入配置寄存器 */
void ICM42688_SetAccelConfig(uint8_t fs, uint8_t odr) {
    gAccelSensitivity = accelSens(fs);
    ICM42688_WriteReg(ICM42688_REG_ACCEL_CONFIG0, ICM42688_BANK_0,
                      (fs<<5)|(odr&0x0F));
}

/* ICM42688_SetGyroConfig: 配置陀螺音量程和ODR
 * 参数:
 *   fs  - 量程选择
 *   odr - 输出数据速率 */
void ICM42688_SetGyroConfig(uint8_t fs, uint8_t odr) {
    gGyroSensitivity = gyroSens(fs);
    ICM42688_WriteReg(ICM42688_REG_GYRO_CONFIG0, ICM42688_BANK_0,
                      (fs<<5)|(odr&0x0F));
}

/* softReset: 软件复位ICM42688
 * 实现: 向DEVICE_CONFIG寄存器写入0x01
 * 复位后所有寄存器恢复默认值,需要重新初始化
 * 延时50ms等待复位完成 */
static void softReset(void) {
    ICM42688_WriteReg(ICM42688_REG_DEVICE_CONFIG, ICM42688_BANK_0, 0x01);
    delayMs(50);
    gCurrentBank = 0;  /* 复位后默认bank为0 */
}

/* ICM42688_Init: ICM42688初始化函数
 * 返回值: 0=成功, -1=失败
 * 初始化步骤:
 *   1. 标记需要切换bank(0xFF确保切换)
 *   2. 执行软件复位
 *   3. 延时10ms等待复位稳定
 *   4. 读取WHO_AM_I寄存器验证器件ID(期望值0x47)
 *   5. 配置加速度计: ±4g量程, 100Hz ODR
 *   6. 配置陀螺仪: ±1000dps量程, 100Hz ODR
 *   7. 设置加速度计和陀螺仪为低噪声模式
 *   8. 延时1ms等待配置生效
 */
int8_t ICM42688_Init(void) {
    uint8_t who, reg;
    gCurrentBank = 0xFF;
    softReset();
    delayMs(10);

    /* 验证器件ID */
    who = ICM42688_ReadReg(ICM42688_REG_WHO_AM_I, ICM42688_BANK_0);
    printf("ICM42688 WHO_AM_I: 0x%02X\r\n", who);
    if (who != ICM42688_WHO_AM_I_VALUE) return -1;

    /* 配置加速度计和陀螺仪 */
    ICM42688_SetAccelConfig(ICM42688_ACCEL_FS_4G, ICM42688_ACCEL_ODR_100HZ);
    ICM42688_SetGyroConfig(ICM42688_GYRO_FS_1000DPS, ICM42688_GYRO_ODR_100HZ);

    /* 设置电源模式为低噪声 */
    reg = (ICM42688_MODE_LOW_NOISE<<ICM42688_PWR_GYRO_MODE_SHIFT)
        | (ICM42688_MODE_LOW_NOISE<<ICM42688_PWR_ACCEL_MODE_SHIFT);
    ICM42688_WriteReg(ICM42688_REG_PWR_MGMT0, ICM42688_BANK_0, reg);
    delayMs(50);
    return 0;
}

/* burstRead: 突发读取多个字节
 * 参数:
 *   reg - 起始寄存器地址
 *   buf - 数据接收缓冲区
 *   len - 要读取的字节数
 * 说明: 突发读取可以在一个I2C事务中连续读取多个寄存器
 *       常用于一次性获取一组相关数据(如传感器6轴数据) */
static int8_t burstRead(uint8_t reg, uint8_t *buf, uint8_t len) {
    int8_t status = selectBank(ICM42688_BANK_0);
    if (status != 0) return status;
    return I2C_ReadReg(ICM42688_I2C_ADDR, reg, buf, len);
}

/* parseTrip: 解析三轴数据(6字节->3个int16)
 * 参数:
 *   b - 原始字节数组(大端格式: 高字节在前)
 *   o - 输出的三轴数据
 * 说明: ICM42688的数据寄存器中,高字节在前(MSB first)
 *       需要将两个字节组合成一个16位有符号整数 */
static void parseTrip(const uint8_t *b, icm42688_raw_data_t *o) {
    o->x=(int16_t)(((uint16_t)b[0]<<8)|b[1]);
    o->y=(int16_t)(((uint16_t)b[2]<<8)|b[3]);
    o->z=(int16_t)(((uint16_t)b[4]<<8)|b[5]);
}

/* ICM42688_ReadAccelRaw: 读取加速度计原始数据
 * 参数: d - 存储加速度原始数据
 * 返回值: 0=成功
 * 使用burst read从ACCEL_DATA_X1开始连续读取6字节 */
int8_t ICM42688_ReadAccelRaw(icm42688_raw_data_t *d) {
    uint8_t b[6];
    int8_t status = burstRead(ICM42688_REG_ACCEL_DATA_X1,b,6);
    if (status != 0) return status;
    parseTrip(b,d);
    return 0;
}

/* ICM42688_ReadGyroRaw: 读取陀螺仪原始数据
 * 参数: d - 存储陀螺仪原始数据
 * 返回值: 0=成功 */
int8_t ICM42688_ReadGyroRaw(icm42688_raw_data_t *d) {
    uint8_t b[6];
    int8_t status = burstRead(ICM42688_REG_GYRO_DATA_X1,b,6);
    if (status != 0) return status;
    parseTrip(b,d);
    return 0;
}

/* ICM42688_ReadTemperature: 读取温度传感器
 * 参数: t - 存储温度值(摄氏度)
 * 返回值: 0=成功
 * 说明: ICM42688内置温度传感器,主要用于陀螺仪漂移补偿
 *       温度数据为16位有符号整型,25°C时输出为0
 *       温度转换公式: T(°C) = Raw / 132.48 + 25 */
int8_t ICM42688_ReadTemperature(float *t) {
    uint8_t b[2]; int16_t r;
    int8_t status = burstRead(ICM42688_REG_TEMP_DATA1,b,2);
    if (status != 0) return status;
    r=(int16_t)(((uint16_t)b[0]<<8)|b[1]);
    *t=(float)r/132.48f+25.0f;
    return 0;
}

/* ICM42688_AccelToFloat: 将加速度原始值转换为物理单位(g)
 * 参数:
 *   raw  - 原始ADC值
 *   real - 转换后的物理值
 * 说明: 通过预先计算的灵敏度值进行转换
 *       公式: g_value = raw_value / sensitivity
 *            实际: g_value = raw_value * sensitivity (因为sensitivity = FS/32768) */
void ICM42688_AccelToFloat(const icm42688_raw_data_t *raw,
                            icm42688_real_data_t *real) {
    real->x=(float)raw->x*gAccelSensitivity;
    real->y=(float)raw->y*gAccelSensitivity;
    real->z=(float)raw->z*gAccelSensitivity;
}

/* ICM42688_GyroToFloat: 将陀螺仪原始值转换为物理单位(dps)
 * 参数:
 *   raw  - 原始ADC值
 *   real - 转换后的物理值
 * dps = degrees per second, 即角速度 °/s */
void ICM42688_GyroToFloat(const icm42688_raw_data_t *raw,
                           icm42688_real_data_t *real) {
    real->x=(float)raw->x*gGyroSensitivity;
    real->y=(float)raw->y*gGyroSensitivity;
    real->z=(float)raw->z*gGyroSensitivity;
}

/* ICM42688_ReadMotion6: 同时读取加速度计和陀螺仪(推荐)
 * 参数:
 *   acc  - 加速度数据输出(g)
 *   gyro - 陀螺仪数据输出(dps)
 * 返回值: 0=成功
 * 优点:
 *   1. 一次I2C传输获取全部6轴数据,效率高
 *   2. 确保加速度和陀螺仪数据是同一时刻采样
 *   3. 数据一次性获取避免了读取过程中的时间差
 * 数据格式(12字节):
 *   [AX_H, AX_L, AY_H, AY_L, AZ_H, AZ_L, GX_H, GX_L, GY_H, GY_L, GZ_H, GZ_L] */
int8_t ICM42688_ReadMotion6(icm42688_real_data_t *acc,
                             icm42688_real_data_t *gyro) {
    uint8_t b[12];
    int8_t status = burstRead(ICM42688_REG_ACCEL_DATA_X1,b,12);
    if (status != 0) return status;
    icm42688_raw_data_t ar,gr;
    /* 解析加速度数据 */
    ar.x=(int16_t)(((uint16_t)b[0]<<8)|b[1]);
    ar.y=(int16_t)(((uint16_t)b[2]<<8)|b[3]);
    ar.z=(int16_t)(((uint16_t)b[4]<<8)|b[5]);
    /* 解析陀螺仪数据 */
    gr.x=(int16_t)(((uint16_t)b[6]<<8)|b[7]);
    gr.y=(int16_t)(((uint16_t)b[8]<<8)|b[9]);
    gr.z=(int16_t)(((uint16_t)b[10]<<8)|b[11]);
    /* 转换为物理单位 */
    ICM42688_AccelToFloat(&ar,acc);
    ICM42688_GyroToFloat(&gr,gyro);
    return 0;
}