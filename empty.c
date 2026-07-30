#include "ti_msp_dl_config.h"
#include "mylib/Motor.h"
#include "mylib/find.h"
#include "mylib/key.h"
#include <stdint.h>

typedef enum {
    status_stop = 0,
    status_task1,
    status_task2,
    status_task3,
    status_task4
} SystemState_t;

static uint8_t key_status = 0U;
static SystemState_t state = status_stop;
static volatile uint32_t g_ms_tick = 0U;
static uint32_t g_task2_last_ms = 0U;
static uint8_t g_task2_started = 0U;

void stop(void);
void task1(void);
void task2(void);
void task4(void);
void task8(void);

int main(void)
{
    /* 初始化 SysConfig 中配置的 GPIO、PWM和定时器。 */
    SYSCFG_DL_init();

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

    while (1) {
        /* KEY2 选择 task2，执行黑色环形路线循迹。 */
        key_status = key_read();
        if (key_status == 0U) state = status_stop;
        else if (key_status == 1U) state = status_task1;
        else if (key_status == 2U) state = status_task2;
        else if (key_status == 3U) state = status_task3;
        else if (key_status == 4U) state = status_task4;

        switch (state) {
        case status_stop:  stop();  break;
        case status_task1: task1(); break;
        case status_task2: task2(); break;
        case status_task3: task4(); break;
        case status_task4: task8(); break;
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
}

/* 停车，并让 task2 下次进入时从头开始。 */
void stop(void)
{
    RingTrack_Stop();
    g_task2_started = 0U;
}

/* 其他题目暂时保留空函数入口。 */
void task1(void) {}
void task4(void) {}
void task8(void) {}

/* 任务2：按固定顺序组合 find.c 中的各个循迹功能。 */
void task2(void)
{
    /* 第一次进入任务时，清零里程并开始新的一圈。 */
    if (g_task2_started == 0U) {
        RingTrack_Start();
        g_task2_last_ms = g_ms_tick;
        g_task2_started = 1U;
        return;
    }

    /* 已完成一圈或因丢线故障停车后，保持停车直到重新选择任务。 */
    if (RingTrack_GetState() == RING_TRACK_FINISHED ||
        RingTrack_GetState() == RING_TRACK_FAULT) {
        return;
    }

    /* 每5 ms运行一次，保证PD计算和速度斜坡的周期固定。 */
    if ((uint32_t)(g_ms_tick - g_task2_last_ms) < RING_TRACK_PERIOD_MS) {
        return;
    }
    g_task2_last_ms += RING_TRACK_PERIOD_MS;

    /* 1. 读取八路黑线位置和编码器里程。 */
    RingTrack_ReadSensors();

    /* 2. 跑够一圈后压到A点宽黑线，立即停车。 */
    if (RingTrack_CheckFinish(RING_TRACK_PERIOD_MS)) {
        return;
    }

    /* 3. 丢线时执行保持、搜索或故障停车，不再进行正常循迹。 */
    if (RingTrack_IsLineLost()) {
        RingTrack_HandleLineLost(RING_TRACK_PERIOD_MS);
        return;
    }

    /* 4. 更新分段匀速变化的基础速度。 */
    RingTrack_UpdateSpeedProfile();

    /* 5. 计算转向，再把左右轮速度输出给电机。 */
    RingTrack_CalculateSteering();
    RingTrack_OutputMotor();
}
