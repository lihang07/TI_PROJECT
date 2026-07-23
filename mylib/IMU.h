#ifndef __IMU_H__
#define __IMU_H__

/* IMU (Inertial Measurement Unit) 姿态解算模块
 * =============================================
 * 本模块实现基于IMU传感器的姿态解算算法(AHRS),
 * 输出yaw(偏航角)、pitch(俯仰角)、roll(翻滚角)
 */

#include "ti_msp_dl_config.h"
#include <math.h>

/* xyz_f_t: 三轴浮点数据结构
 * 用于存储3D向量数据,如加速度、角速度、姿态角等 */
typedef struct { float x; float y; float z; } xyz_f_t;

/* 全局变量声明 - 用于外部访问姿态数据 */
extern xyz_f_t north, west;                        /* 地理坐标系下的北向和西向向量(算法内部使用) */
extern volatile float yaw[5];                      /*  yaw角度缓存(用于平滑滤波) */
extern float motion6[7];                           /* 6轴传感器数据缓存[ax,ay,az,gx,gy,gz] */

/* IMU_init: 初始化IMU模块
 * 流程:
 *   1. 初始化ICM42688传感器
 *   2. 重置四元数q0=1, q1=q2=q3=0
 *   3. 重置PI控制器积分项 */
void IMU_init(void);

/* IMU_getYawPitchRoll: 获取姿态角(Yaw, Pitch, Roll)
 * 参数: ypr - 输出数组[yaw, pitch, roll] 单位:度(°)
 * 说明: 基于AHRS算法融合加速度计和陀螺仪数据
 *       使用四元数表示姿态,通过欧拉角转换得到最终角度 */
int8_t IMU_getYawPitchRoll(float *ypr, float dt_seconds);

/* IMU_TT_getgyro: 获取原始陀螺仪数据
 * 参数: zsjganda - 输出数组[ax,ay,az,gx,gy,gz,0]
 * 说明: 直接获取传感器原始数据,用于调试或外部滤波 */
void IMU_TT_getgyro(float *zsjganda);

/* Return the most recent gyro sample after stationary bias compensation. */
void IMU_GetCorrectedGyro(float *gyro);

/* MPU6050_InitAng_Offset: 传感器零偏初始化(兼容MPU6050接口)
 * 说明: 本项目中未使用,仅保持接口兼容性 */
void MPU6050_InitAng_Offset(void);

#endif /* __IMU_H__ */
