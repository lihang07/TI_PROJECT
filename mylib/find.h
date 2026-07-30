#ifndef MYLIB_FIND_H_
#define MYLIB_FIND_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * 商家八路数字灰度模块：黑线时数字输出为高电平，因此设为 1U。
 * G1～G8 已按从车头前方看过去的左到右顺序连接。
 * 若实车验证黑线时输出为低电平，只需把本行改回 0U。
 */
#define LINE_SENSOR_BLACK_LEVEL 1U

/* task2 中每隔 5 ms 调用一次各循迹函数。 */
#define RING_TRACK_PERIOD_MS 5U

typedef enum {
    RING_TRACK_IDLE = 0,
    RING_TRACK_AB,
    RING_TRACK_BC,
    RING_TRACK_CD,
    RING_TRACK_DA,
    RING_TRACK_FINISHED,
    RING_TRACK_FAULT
} RingTrackState_t;

/* 程序启动时调用一次，清空循迹模块数据。 */
void RingTrack_Init(void);

/* 开始新的一圈：清零编码器和循迹状态，并使能电机。 */
void RingTrack_Start(void);

/* 读取八路数字传感器，保存黑线位置、黑线数量和行驶距离。 */
void RingTrack_ReadSensors(void);

/* 根据当前里程更新AB、BC、CD、DA路段和基础速度。 */
void RingTrack_UpdateSpeedProfile(void);

/* 根据黑线位置计算PD转向修正量。 */
void RingTrack_CalculateSteering(void);

/* 将当前基础速度和转向修正量输出到左右电机。 */
void RingTrack_OutputMotor(void);

/* 当全部传感器都未检测到黑线时返回 true。 */
bool RingTrack_IsLineLost(void);

/* 丢线时保持、搜索或故障停车。 */
void RingTrack_HandleLineLost(uint16_t period_ms);

/* 满足跑够一圈且重新压到A点宽黑线时自动停车并返回 true。 */
bool RingTrack_CheckFinish(uint16_t period_ms);

/* 手动停止小车，并为下次任务做好准备。 */
void RingTrack_Stop(void);

/* 获取当前路段或完成/故障状态。 */
RingTrackState_t RingTrack_GetState(void);

#endif
