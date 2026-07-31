#include "ti_msp_dl_config.h"
#include "mylib/Motor.h"
#include "mylib/find.h"
#include "mylib/key.h"
#include "mylib/oled.h"
#include "mylib/IMU.h"
#include "mylib/delay.h"
#include <stdint.h>

/* ==================== 环形循迹参数 ==================== */

#define RING_PERIOD_MS             5U
#define RING_START_SPEED           15.36f
#define RING_HIGH_SPEED            30.72f
#define RING_LOW_SPEED             23.04f
#define RING_SEARCH_SPEED          11
#define RING_KP                    4.0f
#define RING_KD                    5.6f
#define RING_CORRECTION_MAX        19.2f
#define RING_CORRECTION_STEP       1.5f
#define RING_MARK_COUNT            6U
#define RING_MARK_CONFIRM_MS       25U
#define RING_LOST_HOLD_MS          50U
#define RING_LOST_STOP_MS          400U
#define RING_LAP_PULSES            9000L
#define RING_FORWARD_SIGN          1

/* ==================== 第四问A到B循迹参数 ==================== */

#define TASK4_PERIOD_MS            5U
#define TASK4_START_SPEED          12.8f
#define TASK4_CRUISE_SPEED         22.4f
#define TASK4_ACCEL_END_MS         3125U
#define TASK4_DECEL_START_MS       11406U
#define TASK4_SLOW_SPEED           17.92f
#define TASK4_DECEL_END_MS         13281U
#define TASK4_STOP_DELAY_MS        1000U

/* ==================== 第五问整圈循迹参数 ==================== */

/* task5同样每5 ms进行一次传感器读取和PD循迹。 */
#define TASK5_PERIOD_MS            5U
#define TASK5_START_SPEED          12.8f
#define TASK5_CRUISE_SPEED         22.4f
#define TASK5_ACCEL_TIME_MS        2000U
#define TASK5_STOP_DELAY_MS        1000U
#define TASK5_FINISH_ENABLE_MS     20000U

/* ==================== 主任务状态 ==================== */

typedef enum {
    status_stop = 0,
    status_task2,
    status_task4,
    status_task5,
    status_task6
} SystemState_t;

/* task5的三个运行阶段。 */
typedef enum {
    TASK5_ACCEL_AND_CRUISE = 0,
    TASK5_DECELERATING,
    TASK5_STOPPED
} Task5Phase_t;

static uint8_t key_status = 0U;
static SystemState_t state = status_stop;
static volatile uint32_t g_ms_tick = 0U;
static uint32_t g_task2_start_ms = 0U;
static uint32_t g_task2_last_ms = 0U;
static uint8_t g_task2_started = 0U;
static uint32_t g_task2_imu_last_ms = 0U;
static float g_task2_ypr[3] = {0.0f, 0.0f, 0.0f};
static uint32_t g_task4_start_ms = 0U;
static uint32_t g_task4_last_ms = 0U;
static uint8_t g_task4_started = 0U;
static uint8_t g_task4_finished = 0U;
static uint32_t g_task5_start_ms = 0U;
static uint32_t g_task5_last_ms = 0U;
static uint32_t g_task5_decel_start_ms = 0U;
static uint8_t g_task5_started = 0U;
static Task5Phase_t g_task5_phase = TASK5_ACCEL_AND_CRUISE;
static uint32_t g_task6_start_ms = 0U;
static uint32_t g_task6_last_ms = 0U;
static uint32_t g_task6_decel_start_ms = 0U;
static uint8_t g_task6_started = 0U;
static Task5Phase_t g_task6_phase = TASK5_ACCEL_AND_CRUISE;

void stop(void);
void task2(void);
void task4(void);
void task5(void);
void task6(void);

static float task4_get_base_speed(uint32_t elapsed_ms);
static float task5_get_running_speed(uint32_t elapsed_ms);

int main(void)
{
    uint16_t imu_sample;

    /* 初始化 SysConfig 中配置的 GPIO、PWM和定时器。 */
    SYSCFG_DL_init();

    /* 复位后初始化OLED，并把行程时间清零。 */
    OLED_Init();
    OLED_RunTimeReset();

    /* 上电时保持车体静止约2秒，让陀螺仪完成零偏学习。 */
    IMU_init();
    for (imu_sample = 0U; imu_sample < 100U; ++imu_sample) {
        IMU_getYawPitchRoll(g_task2_ypr);
        delay_ms(20U);
    }

    /* 初始化编码器和环形循迹模块，然后保持停车。 */
    Motor_Encoder_Init();
    RingTrack_Init();
    Motor_Brake();

    /* 开启 10 ms 编码器测速中断和 1 ms 系统节拍中断。 */
    NVIC_ClearPendingIRQ(TIMER_count_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_count_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(TIMER_TICK_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_TICK_INST_INT_IRQN);
    __enable_irq();

// if ZDT_TEST_MODE
//     /*
//      * This branch is deliberately independent of the key scanner and task
//      * state machine.  It verifies only the STP/DIR/EN electrical interface.
//      */
//     ZDT_X42S_Pulse_Init(&g_zdt_x42s,
//                          -ZDT_TEST_LIMIT_PULSE,
//                          ZDT_TEST_LIMIT_PULSE,
//                          ZDT_TEST_MAX_RATE_HZ,
//                          ZDT_TEST_ACCEL_PULSE_S2,
//                          0);
//     ZDT_X42S_Pulse_Enable(&g_zdt_x42s, true);

//     /* Wait one second after reset, then alternate direction once per second. */
//     uint32_t next_change_ms = g_ms_tick + ZDT_TEST_HOLD_MS;
//     bool move_positive = true;
//     ZDT_X42S_Pulse_SetTarget(&g_zdt_x42s, ZDT_TEST_MOVE_PULSE);

//     while (1) {
//         if ((int32_t)(g_ms_tick - next_change_ms) >= 0) {
//             move_positive = !move_positive;
//             ZDT_X42S_Pulse_SetTarget(
//                 &g_zdt_x42s,
//                 move_positive ? ZDT_TEST_MOVE_PULSE : -ZDT_TEST_MOVE_PULSE);
//             next_change_ms += ZDT_TEST_HOLD_MS;
//         }
//     }
// #else
    while (1) {
        /* KEY2 选择 task2，执行黑色环形路线循迹。 */
        key_status = key_read();
        if (key_status == 0U) state = status_stop;
        else if (key_status == 1U) state = status_task2;
        else if (key_status == 2U) state = status_task4;
        else if (key_status == 3U) state = status_task5;
        else if (key_status == 4U) state = status_task6;

        switch (state) {
        case status_stop:  stop();  break;
        case status_task2: task2(); break;
        case status_task4: task4(); break;
        case status_task5: task5(); break;
        case status_task6: task6(); break;
        default:           stop();  break;
        }
    }
}

/* 每 10 ms 更新一次左右编码器的位置和速度。 */
void TIMG7_IRQHandler(void)
{
    DL_TimerG_clearInterruptStatus(
        TIMER_count_INST, DL_TIMERG_INTERRUPT_LOAD_EVENT);
    Motor_Encoder_UpdateSpeed();
}

/* 每 1 ms 把系统时间加一。 */
void TIMA0_IRQHandler(void)
{
    DL_TimerA_clearInterruptStatus(
        TIMER_TICK_INST, DL_TIMERA_INTERRUPT_LOAD_EVENT);
    ++g_ms_tick;
#if ZDT_TEST_MODE
    /* Advance the acceleration, position limit and PWM pulse generator. */
    ZDT_X42S_Pulse_Update1ms(&g_zdt_x42s);
#endif
}

void TIMER_0_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(TIMER_0_INST)) {
    case DL_TIMER_IIDX_ZERO:
        /* I2C读取不能放在中断中；task2主循环每20 ms读取一次IMU。 */
        break;
    default:
        break;
    }
}

void stop(void)
{
    RingTrack_Stop();
    g_task2_started = 0U;
    g_task4_started = 0U;
    g_task5_started = 0U;
    g_task6_started = 0U;
}

/* 任务2：按固定顺序组合 find.c 中的各个循迹功能。 */
void task2(void)
{
    /* 第一次进入任务时，清零里程并开始新的一圈。 */
    if (g_task2_started == 0U) {
        RingTrack_Start();
        OLED_RunTimeReset();
        g_task2_start_ms = g_ms_tick;
        g_task2_last_ms = g_ms_tick;
        IMU_getYawPitchRoll(g_task2_ypr);
        g_task2_imu_last_ms = g_ms_tick;
        RingTrack_GyroReset(g_task2_ypr[0]);
        g_task2_started = 1U;
        g_task4_started = 0U;
        return;
    }

    /* 检出启停线后反推60 ms，再抱刹并冻结最终行程时间。 */
    if (RingTrack_GetState() == RING_TRACK_EMERGENCY_BRAKING) {
        if ((uint32_t)(g_ms_tick - g_task2_last_ms) <
            RING_TRACK_PERIOD_MS) {
            return;
        }
        g_task2_last_ms += RING_TRACK_PERIOD_MS;
        if (RingTrack_UpdateEmergencyBrake(RING_TRACK_PERIOD_MS)) {
            OLED_ShowRunTime(
                (uint32_t)(g_ms_tick - g_task2_start_ms), 0U);
        }
        return;
    }

    /* 已完成一圈或因丢线故障停车后，保持停车直到重新选择任务。 */
    if (RingTrack_GetState() == RING_TRACK_FINISHED ||
        RingTrack_GetState() == RING_TRACK_FAULT) {
        OLED_ShowRunTime((uint32_t)(g_ms_tick - g_task2_start_ms), 0U);
        return;
    }

    /* 每5 ms运行一次，保证PD计算和速度斜坡的周期固定。 */
    if ((uint32_t)(g_ms_tick - g_task2_last_ms) < RING_TRACK_PERIOD_MS) {
        return;
    }
    g_task2_last_ms += RING_TRACK_PERIOD_MS;
    OLED_ShowRunTime((uint32_t)(g_ms_tick - g_task2_start_ms), 1U);

    /* 1. 通过I2C读取六路红外探头和编码器里程。 */
    RingTrack_ReadSensors();

    /* IMU算法每20 ms更新一次，5 ms电机周期复用最近航向。 */
    if ((uint32_t)(g_ms_tick - g_task2_imu_last_ms) >= 20U) {
        g_task2_imu_last_ms += 20U;
        IMU_getYawPitchRoll(g_task2_ypr);
        RingTrack_UpdateGyro(g_task2_ypr[0]);
    }

    /* 2. 起步1秒后，检测到终点横线时停车；不依赖编码器里程。 */
    if (RingTrack_CheckFinish(RING_TRACK_PERIOD_MS)) {
        return;
    }

    /* 3. 丢线时执行保持、搜索或故障停车，不再进行正常循迹。 */
    /* 4. 更新分段匀速变化的基础速度。 */
    RingTrack_UpdateSpeedProfile();

    /* 5. 按预定路段差速前进，红外只在偏差较大时辅助纠正。 */
    RingTrack_CalculateHybridSteering(RING_TRACK_PERIOD_MS);
    RingTrack_OutputMotor();
}





/* 根据运行时间生成第四问的匀加速、匀速和匀减速曲线。 */
static float task4_get_base_speed(uint32_t elapsed_ms)
{
    float progress;

    if (elapsed_ms < TASK4_ACCEL_END_MS) {
        progress = (float)elapsed_ms / (float)TASK4_ACCEL_END_MS;
        return TASK4_START_SPEED +
               (TASK4_CRUISE_SPEED - TASK4_START_SPEED) * progress;
    }

    if (elapsed_ms < TASK4_DECEL_START_MS) {
        return TASK4_CRUISE_SPEED;
    }

    if (elapsed_ms < TASK4_DECEL_END_MS) {
        progress = (float)(elapsed_ms - TASK4_DECEL_START_MS) /
                   (float)(TASK4_DECEL_END_MS - TASK4_DECEL_START_MS);
        return TASK4_CRUISE_SPEED -
               (TASK4_CRUISE_SPEED - TASK4_SLOW_SPEED) * progress;
    }

    return TASK4_SLOW_SPEED;
}

/* 第四问循迹：再次降速后约11.406秒通过AB，再循迹1秒刹停。 */
void task4(void)
{
    uint32_t elapsed_ms;

    /* 第一次进入时重新开始循迹并记录起步时间。 */
    if (g_task4_started == 0U) {
        RingTrack_Start();
        OLED_RunTimeReset();
        g_task4_start_ms = g_ms_tick;
        g_task4_last_ms = g_ms_tick;
        g_task4_started = 1U;
        g_task4_finished = 0U;
        g_task2_started = 0U;
        return;
    }

    /* 丢线保护已经停车时，不再继续输出电机速度。 */
    if (g_task4_finished != 0U ||
        RingTrack_GetState() == RING_TRACK_FAULT) {
        OLED_ShowRunTime((uint32_t)(g_ms_tick - g_task4_start_ms), 0U);
        return;
    }

    /* 固定每5 ms执行一次，保持PD计算周期稳定。 */
    if ((uint32_t)(g_ms_tick - g_task4_last_ms) < TASK4_PERIOD_MS) {
        return;
    }
    g_task4_last_ms += TASK4_PERIOD_MS;
    elapsed_ms = (uint32_t)(g_ms_tick - g_task4_start_ms);
    OLED_ShowRunTime(elapsed_ms, 1U);

    /* 1. 通过I2C读取六路红外探头。 */
    RingTrack_ReadSensors();

    /* 编码器确认完成第一段1.5 m直线后立即结束task4。 */
    if (RingTrack_FirstStraightComplete()) {
        Motor_Brake();
        g_task4_finished = 1U;
        OLED_ShowRunTime(elapsed_ms, 0U);
        RingTrack_Stop();
        return;
    }

    /* 2. 丢线时暂时保持并搜索，超时后保护停车。 */
    if (RingTrack_IsLineLost()) {
        RingTrack_HandleLineLost(TASK4_PERIOD_MS);
        if (RingTrack_GetState() == RING_TRACK_FAULT) {
            OLED_ShowRunTime(elapsed_ms, 0U);
        }
        return;
    }

    /* 3. 设置第四问专用的基础速度曲线。 */
    RingTrack_SetBaseSpeed(task4_get_base_speed(elapsed_ms));

    /* 4. 复用PD循迹修正并输出左右轮速度。 */
    RingTrack_CalculateSteering();
    RingTrack_OutputMotor();
}






/*
 * task5起步和整圈运行速度：
 * 前2秒从12.8线性增加到22.4，之后保持22.4直到再次通过A点。
 */
static float task5_get_running_speed(uint32_t elapsed_ms)
{
    float progress;

    if (elapsed_ms >= TASK5_ACCEL_TIME_MS) {
        return TASK5_CRUISE_SPEED;
    }

    progress = (float)elapsed_ms / (float)TASK5_ACCEL_TIME_MS;
    return TASK5_START_SPEED +
           (TASK5_CRUISE_SPEED - TASK5_START_SPEED) * progress;
}

/*
 * 第五问：KEY4启动。
 * 匀加速到22.4后保持该速度完成整圈；再次通过A点后延时1秒停车。
 */
void task5(void)
{
    uint32_t elapsed_ms;
    uint32_t decel_elapsed_ms;
    float base_speed;

    /* 第一次进入时清零循迹状态，并开始记录整圈时间。 */
    if (g_task5_started == 0U) {
        RingTrack_Start();
        OLED_RunTimeReset();
        g_task5_start_ms = g_ms_tick;
        g_task5_last_ms = g_ms_tick;
        g_task5_decel_start_ms = 0U;
        g_task5_phase = TASK5_ACCEL_AND_CRUISE;
        g_task5_started = 1U;
        g_task2_started = 0U;
        g_task4_started = 0U;
        return;
    }

    /* 已经完成减速或因丢线保护停车后，不再输出新的电机速度。 */
    if (g_task5_phase == TASK5_STOPPED ||
        RingTrack_GetState() == RING_TRACK_FAULT) {
        OLED_ShowRunTime((uint32_t)(g_ms_tick - g_task5_start_ms), 0U);
        return;
    }

    /* 固定每5 ms执行一次，使速度斜坡和PD计算保持稳定。 */
    if ((uint32_t)(g_ms_tick - g_task5_last_ms) < TASK5_PERIOD_MS) {
        return;
    }
    g_task5_last_ms += TASK5_PERIOD_MS;
    elapsed_ms = (uint32_t)(g_ms_tick - g_task5_start_ms);
    OLED_ShowRunTime(elapsed_ms, 1U);

    /* 1. 通过I2C读取六路红外探头。 */
    RingTrack_ReadSensors();

    /* 2. 丢线时先保持并搜索，超时才保护停车。 */
    if (RingTrack_IsLineLost()) {
        RingTrack_HandleLineLost(TASK5_PERIOD_MS);
        if (RingTrack_GetState() == RING_TRACK_FAULT) {
            OLED_ShowRunTime(elapsed_ms, 0U);
        }
        return;
    }

    if (g_task5_phase == TASK5_ACCEL_AND_CRUISE) {
        /*
         * 再次通过A点后继续正常循迹1秒，再立即刹车。
         * A点检测内部会忽略起步后的前1秒，避免把起点误判为终点。
         */
        if (elapsed_ms >= TASK5_FINISH_ENABLE_MS &&
            RingTrack_EndLineDetected(TASK5_PERIOD_MS)) {
            g_task5_phase = TASK5_DECELERATING;
            g_task5_decel_start_ms = g_ms_tick;
            base_speed = TASK5_CRUISE_SPEED;
        } else {
            base_speed = task5_get_running_speed(elapsed_ms);
        }
    } else {
        /* 检出启停线后继续正常循迹1秒，再直接刹停。 */
        decel_elapsed_ms =
            (uint32_t)(g_ms_tick - g_task5_decel_start_ms);
        base_speed = TASK5_CRUISE_SPEED;

        if (decel_elapsed_ms >= TASK5_STOP_DELAY_MS) {
            Motor_Brake();
            g_task5_phase = TASK5_STOPPED;
            OLED_ShowRunTime(elapsed_ms, 0U);
            RingTrack_Stop();
            return;
        }
    }

    /* 3. 设置基础速度，叠加PD循迹修正后输出左右轮。 */
    RingTrack_SetBaseSpeed(base_speed);
    RingTrack_CalculateSteering();
    RingTrack_OutputMotor();
}

/*
 * task6与task5功能相同，共用六路I2C循迹和速度曲线，
 * 但使用独立运行状态，避免两个任务相互干扰。
 */
void task6(void)
{
    uint32_t elapsed_ms;
    uint32_t decel_elapsed_ms;
    float base_speed;

    if (g_task6_started == 0U) {
        RingTrack_Start();
        OLED_RunTimeReset();
        g_task6_start_ms = g_ms_tick;
        g_task6_last_ms = g_ms_tick;
        g_task6_decel_start_ms = 0U;
        g_task6_phase = TASK5_ACCEL_AND_CRUISE;
        g_task6_started = 1U;
        g_task2_started = 0U;
        g_task4_started = 0U;
        g_task5_started = 0U;
        return;
    }

    if (g_task6_phase == TASK5_STOPPED ||
        RingTrack_GetState() == RING_TRACK_FAULT) {
        OLED_ShowRunTime((uint32_t)(g_ms_tick - g_task6_start_ms), 0U);
        return;
    }

    if ((uint32_t)(g_ms_tick - g_task6_last_ms) < TASK5_PERIOD_MS) {
        return;
    }
    g_task6_last_ms += TASK5_PERIOD_MS;
    elapsed_ms = (uint32_t)(g_ms_tick - g_task6_start_ms);
    OLED_ShowRunTime(elapsed_ms, 1U);

    RingTrack_ReadSensors();
    if (RingTrack_IsLineLost()) {
        RingTrack_HandleLineLost(TASK5_PERIOD_MS);
        if (RingTrack_GetState() == RING_TRACK_FAULT) {
            OLED_ShowRunTime(elapsed_ms, 0U);
        }
        return;
    }

    if (g_task6_phase == TASK5_ACCEL_AND_CRUISE) {
        if (elapsed_ms >= TASK5_FINISH_ENABLE_MS &&
            RingTrack_EndLineDetected(TASK5_PERIOD_MS)) {
            g_task6_phase = TASK5_DECELERATING;
            g_task6_decel_start_ms = g_ms_tick;
            base_speed = TASK5_CRUISE_SPEED;
        } else {
            base_speed = task5_get_running_speed(elapsed_ms);
        }
    } else {
        decel_elapsed_ms =
            (uint32_t)(g_ms_tick - g_task6_decel_start_ms);
        base_speed = TASK5_CRUISE_SPEED;

        if (decel_elapsed_ms >= TASK5_STOP_DELAY_MS) {
            Motor_Brake();
            g_task6_phase = TASK5_STOPPED;
            OLED_ShowRunTime(elapsed_ms, 0U);
            RingTrack_Stop();
            return;
        }
    }

    RingTrack_SetBaseSpeed(base_speed);
    /* task6使用原来的PD修正参数，与task2、task4、task5保持一致。 */
    RingTrack_CalculateSteering();
    RingTrack_OutputMotor();
}


