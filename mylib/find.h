#ifndef MYLIB_FIND_H_
#define MYLIB_FIND_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * 当前六路红外模块采用I2C通信：实测7位地址0x5C，状态寄存器0x05。
 * 六个探头按从车头前方看过去的左到右顺序处理。
 */
/* task2 中每隔 5 ms 调用一次各循迹函数。 */
#define RING_TRACK_PERIOD_MS 5U

typedef enum {
    RING_TRACK_IDLE = 0,
    RING_TRACK_AB,
    RING_TRACK_BC,
    RING_TRACK_CD,
    RING_TRACK_DA,
    RING_TRACK_EMERGENCY_BRAKING,
    RING_TRACK_FINISHED,
    RING_TRACK_FAULT
} RingTrackState_t;

/* 程序启动时调用一次，清空循迹模块数据。 */
void RingTrack_Init(void);

/* 开始新的一圈：清零编码器和循迹状态，并使能电机。 */
void RingTrack_Start(void);

/* 通过I2C读取六路红外探头，保存黑线位置、压线数量和编码器里程。 */
void RingTrack_ReadSensors(void);

/* 根据当前里程更新AB、BC、CD、DA路段和基础速度。 */
void RingTrack_UpdateSpeedProfile(void);

/* 直接设置基础速度，供不同题目组合各自的速度曲线。 */
void RingTrack_SetBaseSpeed(float speed);

/* 根据黑线位置计算PD转向修正量。 */
void RingTrack_CalculateSteering(void);

/* task2专用：按AB/BC/CD/DA预定轨迹前进，红外仅在偏差较大时辅助。 */
void RingTrack_CalculatePresetSteering(void);

/* task2陀螺仪航向控制：开始时记录零度，运行中按路段目标航向控制。 */
void RingTrack_GyroReset(float initial_yaw);
void RingTrack_CalculateGyroSteering(float current_yaw);

/* task2混合控制：20 ms更新IMU，5 ms计算平滑的融合转向。 */
void RingTrack_UpdateGyro(float current_yaw);
void RingTrack_CalculateHybridSteering(uint16_t period_ms);

/* 将当前基础速度和转向修正量输出到左右电机。 */
void RingTrack_OutputMotor(void);

/* 当全部传感器都未检测到黑线时返回 true。 */
bool RingTrack_IsLineLost(void);

/* 编码器到达第一段1.5 m直线终点时返回true。 */
bool RingTrack_FirstStraightComplete(void);

/* 丢线时保持、搜索或故障停车。 */
void RingTrack_HandleLineLost(uint16_t period_ms);

/* task2起步1秒后，至少4路探头同时识别黑线时启动反推急停。 */
bool RingTrack_CheckFinish(uint16_t period_ms);

/* task2反推急停过程；反推结束后自动抱刹并返回true。 */
bool RingTrack_UpdateEmergencyBrake(uint16_t period_ms);

/*
 * 只检测是否再次通过A点横线，不控制电机。
 * 供 task5/task6 在通过A点后开始1秒延时停车。
 */
bool RingTrack_EndLineDetected(uint16_t period_ms);

/* 按指定的压黑探头数量检测横线，供不同题目使用不同宽度的标志线。 */
bool RingTrack_MarkerDetected(uint8_t minimum_black_count,
                              uint16_t period_ms);

/* 手动停止小车，并为下次任务做好准备。 */
void RingTrack_Stop(void);

/* 获取当前路段或完成/故障状态。 */
RingTrackState_t RingTrack_GetState(void);

#endif
