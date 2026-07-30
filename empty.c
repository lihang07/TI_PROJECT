#include "ti_msp_dl_config.h"
#include "mylib/Motor.h"
#include "mylib/key.h"
#include "mylib/zdt_x42s_pulse.h"
#include <stdbool.h>
#include <stdint.h>

/* ==================== 环形循迹参数 ==================== */

/* task2 每隔 5 ms 计算一次循迹。 */
#define RING_PERIOD_MS             5U

/* AB 从 24 加速到 48，BC 保持 48，CD 从 48 减速到 36，DA 保持 36。 */
#define RING_START_SPEED           24.0f
#define RING_HIGH_SPEED            48.0f
#define RING_LOW_SPEED             36.0f

/* 丢线后使用 18 的低速度寻找黑线。 */
#define RING_SEARCH_SPEED          18

/* 循迹 PD 参数。KP 决定转向强度，KD 用于减小左右摆动。 */
#define RING_KP                    5.0f
#define RING_KD                    7.0f

/* 转向修正最大为 24，而且每 5 ms 最多改变 2，防止突然急转。 */
#define RING_CORRECTION_MAX        24.0f
#define RING_CORRECTION_STEP       2.0f

/* 至少六个探头连续看到黑色 25 ms，才认为检测到了 A 点横线。 */
#define RING_MARK_COUNT            6U
#define RING_MARK_CONFIRM_MS       25U

/* 丢线 50 ms 内保持原速度；丢线达到 400 ms 后停车。 */
#define RING_LOST_HOLD_MS          50U
#define RING_LOST_STOP_MS          400U

/*
 * 必须实车标定：小车完整行驶一圈时，
 * (左编码器绝对值 + 右编码器绝对值) / 2 的结果。
 * 6000 只是临时占位值，正式使用前必须换成实测值。
 */
#define RING_LAP_PULSES            6000L

/* 正数能让小车前进就保持 1；如果正数让小车后退，就改成 -1。 */
#define RING_FORWARD_SIGN          1

/*
 * ==================== X42S pulse-interface bench test ====================
 *
 * Set ZDT_TEST_MODE to 1 before wiring verification.  The test keeps the
 * original vehicle task loop intact under #else, then automatically drives
 * the X42S between +ZDT_TEST_MOVE_PULSE and -ZDT_TEST_MOVE_PULSE.
 *
 * Signal mapping is supplied by SysConfig through zdt_x42s_pulse.h:
 * PB14 -> STP/PUL, PA7 -> DIR, PB1 -> EN.
 *
 * Start with the small values below.  The software limit is intentionally
 * wider than the test travel so the mechanism cannot continue moving if the
 * test state is changed later.  Set the mode back to 0 for normal tasks.
 */
#define ZDT_TEST_MODE               1
#define ZDT_TEST_MOVE_PULSE         10
#define ZDT_TEST_LIMIT_PULSE        20
#define ZDT_TEST_MAX_RATE_HZ        400U
#define ZDT_TEST_ACCEL_PULSE_S2     1000U
#define ZDT_TEST_HOLD_MS            1000U

/* ==================== 主任务状态 ==================== */

typedef enum {
    status_stop = 0,
    status_task1,
    status_task2,
    status_task3,
    status_task4
} SystemState_t;

typedef enum {
    RING_NOT_STARTED = 0,   /* task2 还没有开始运行。 */
    RING_AB_ACCEL,          /* 当前在 AB 直线加速。 */
    RING_BC_CRUISE,         /* 当前在 BC 半圆保持 48。 */
    RING_CD_DECEL,          /* 当前在 CD 直线减速。 */
    RING_DA_CRUISE,         /* 当前在 DA 半圆保持 36。 */
    RING_FINISHED,          /* 已经回到 A 点并停车。 */
    RING_FAULT              /* 长时间丢线，已经保护停车。 */
} RingState_t;

/* 保存按键选择的主任务。 */
static uint8_t key_status = 0U;
static SystemState_t state = status_stop;

/* TIMA0 每 1 ms 加 1，task2 用它保证固定的 5 ms 控制周期。 */
static volatile uint32_t g_ms_tick = 0U;
static ZDT_X42S_Pulse g_zdt_x42s;

/* task2 当前所在的环形路段。 */
static RingState_t g_ring_state = RING_NOT_STARTED;

/* ==================== 函数声明 ==================== */

void stop(void);
void task1(void);
void task2(void);
void task4(void);
void task8(void);

int main(void)
{
    /* 初始化 SysConfig 中配置的 GPIO、PWM、定时器和串口等外设。 */
    SYSCFG_DL_init();

    /* 初始化左右编码器，上电后先保持停车。 */
    Motor_Encoder_Init();
    Motor_Brake();

    /* 使能用于编码器更新的 TIMG7 中断。 */
    NVIC_ClearPendingIRQ(TIMER_count_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_count_INST_INT_IRQN);

    /* 使能 1 ms 时间基准 TIMA0 中断。 */
    NVIC_ClearPendingIRQ(TIMER_TICK_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_TICK_INST_INT_IRQN);
    __enable_irq();

#if ZDT_TEST_MODE
    /*
     * This branch is deliberately independent of the key scanner and task
     * state machine.  It verifies only the STP/DIR/EN electrical interface.
     */
    ZDT_X42S_Pulse_Init(&g_zdt_x42s,
                         -ZDT_TEST_LIMIT_PULSE,
                         ZDT_TEST_LIMIT_PULSE,
                         ZDT_TEST_MAX_RATE_HZ,
                         ZDT_TEST_ACCEL_PULSE_S2,
                         0);
    ZDT_X42S_Pulse_Enable(&g_zdt_x42s, true);

    /* Wait one second after reset, then alternate direction once per second. */
    uint32_t next_change_ms = g_ms_tick + ZDT_TEST_HOLD_MS;
    bool move_positive = true;
    ZDT_X42S_Pulse_SetTarget(&g_zdt_x42s, ZDT_TEST_MOVE_PULSE);

    while (1) {
        if ((int32_t)(g_ms_tick - next_change_ms) >= 0) {
            move_positive = !move_positive;
            ZDT_X42S_Pulse_SetTarget(
                &g_zdt_x42s,
                move_positive ? ZDT_TEST_MOVE_PULSE : -ZDT_TEST_MOVE_PULSE);
            next_change_ms += ZDT_TEST_HOLD_MS;
        }
    }
#else
    while (1) {
        /* key_read 会记住最后按下的按键；KEY2 对应 task2 环形循迹。 */
        key_status = key_read();

        if (key_status == 0U) state = status_stop;
        else if (key_status == 1U) state = status_task1;
        else if (key_status == 2U) state = status_task2;
        else if (key_status == 3U) state = status_task3;
        else if (key_status == 4U) state = status_task4;

        /* 根据按键选择调用对应任务。 */
        switch (state) {
        case status_stop:
            stop();
            break;
        case status_task1:
            task1();
            break;
        case status_task2:
            task2();
            break;
        case status_task3:
            task4();
            break;
        case status_task4:
            task8();
            break;
        default:
            stop();
            break;
        }
    }
#endif
}

/* TIMG7 每 10 ms 更新一次左右编码器的位置和速度。 */
void TIMG7_IRQHandler(void)
{
    /* 清除中断标志，否则下一次中断不能正常进入。 */
    DL_TimerG_clearInterruptStatus(
        TIMER_count_INST, DL_TIMERG_INTERRUPT_LOAD_EVENT);

    /* 读取新的编码器计数。 */
    Motor_Encoder_UpdateSpeed();
}

/* TIMA0 每 1 ms 提供一次时间计数。 */
void TIMA0_IRQHandler(void)
{
    /* 清除本次 1 ms 定时器中断标志。 */
    DL_TimerA_clearInterruptStatus(
        TIMER_TICK_INST, DL_TIMERA_INTERRUPT_LOAD_EVENT);

    /* 系统毫秒计数加 1。 */
    ++g_ms_tick;
#if ZDT_TEST_MODE
    /* Advance the acceleration, position limit and PWM pulse generator. */
    ZDT_X42S_Pulse_Update1ms(&g_zdt_x42s);
#endif
}

/* 停止状态让两个电机保持停止，并允许下次重新开始 task2。 */
void stop(void)
{
    Motor_Brake();
    g_ring_state = RING_NOT_STARTED;
}

/* 其他任务暂时保留空入口，以后可以继续填写。 */
void task1(void) {}
void task4(void) {}
void task8(void) {}

/*
 * =====================================================================
 * 任务2：沿题目规定的黑色环形路线行驶一圈
 *
 * 速度过程：
 * AB：24 → 48
 * BC：保持 48
 * CD：48 → 36
 * DA：保持 36
 * 返回 A 点：停车
 *
 * G1～G8 和左右轮都按照“从车头前方看过去”的左右方向定义。
 * =====================================================================
 */
void task2(void)
{
    /* 上一次真正执行控制计算的时间。 */
    static uint32_t last_run_ms = 0U;

    /* 滤波后的黑线位置，用于减少探头跳变造成的小车摆动。 */
    static float filtered_position = 0.0f;

    /* 上一次黑线位置，用来计算 PD 中的 D 项。 */
    static float last_position = 0.0f;

    /* 当前正在使用的左右轮转向修正量。 */
    static float correction = 0.0f;

    /* 上一次发送给左右轮的速度，短暂丢线时继续保持。 */
    static int16_t last_left_speed = 0;
    static int16_t last_right_speed = 0;

    /* 连续丢线时间和连续检测 A 点横线的时间。 */
    static uint16_t lost_ms = 0U;
    static uint16_t mark_ms = 0U;

    /* 记录小车是否已经离开启动时的 A 点横线。 */
    static bool left_start_mark = false;

    /* G1～G8 的位置数字，G1 为最左侧，G8 为最右侧。 */
    static const int8_t weight[8] = {-7, -5, -3, -1, 1, 3, 5, 7};

    /* 保存八个探头是否检测到黑线；商家规定低电平 0 表示黑线。 */
    uint8_t black[8];

    /* 保存本次检测到黑线的探头数量和二进制组合。 */
    uint8_t line_count = 0U;
    uint8_t line_mask = 0U;

    /* sum 保存所有黑线探头的位置数字之和。 */
    int16_t position_sum = 0;

    /* 当前循环使用的临时变量。 */
    uint8_t i;
    float raw_position;
    float base_speed;
    float target_correction;
    float correction_change;
    float progress;
    int16_t front_left_speed;
    int16_t front_right_speed;

    /* 左右编码器位置及其平均里程。 */
    int32_t left_encoder;
    int32_t right_encoder;
    int32_t distance;

    /* B、C、D 点在一圈编码器脉冲中的理论位置。 */
    const int32_t point_b = (RING_LAP_PULSES * 2442L) / 10000L;
    const int32_t point_c = RING_LAP_PULSES / 2L;
    const int32_t point_d = (RING_LAP_PULSES * 7442L) / 10000L;

    /* 第一次按 KEY2 进入 task2 时，开始一圈新任务。 */
    if (g_ring_state == RING_NOT_STARTED) {
        /* 从 A 点重新开始记录左右轮行驶距离。 */
        Motor_ResetLeftEncoder();
        Motor_ResetRightEncoder();

        /* 清除上一圈留下的循迹数据。 */
        filtered_position = 0.0f;
        last_position = 0.0f;
        correction = 0.0f;
        last_left_speed = 0;
        last_right_speed = 0;
        lost_ms = 0U;
        mark_ms = 0U;
        left_start_mark = false;

        /* 新任务从 AB 加速段开始。 */
        g_ring_state = RING_AB_ACCEL;
        last_run_ms = g_ms_tick;

        /* 使能电机驱动芯片。 */
        Motor_Enable();
        return;
    }

    /* 已经完成一圈或故障停车后，不再输出新的电机速度。 */
    if (g_ring_state == RING_FINISHED || g_ring_state == RING_FAULT) {
        return;
    }

    /* 距离上次计算不足 5 ms 时直接返回，保证 PD 周期固定。 */
    if ((uint32_t)(g_ms_tick - last_run_ms) < RING_PERIOD_MS) {
        return;
    }
    last_run_ms += RING_PERIOD_MS;

    /* ==================== 第一步：读取八路红外 ==================== */

    /* 每个表达式为 true 表示对应探头读取到低电平黑线。 */
    black[0] = (DL_GPIO_readPins(XG_G1_PORT, XG_G1_PIN) == 0U);
    black[1] = (DL_GPIO_readPins(XG_G2_PORT, XG_G2_PIN) == 0U);
    black[2] = (DL_GPIO_readPins(XG_G3_PORT, XG_G3_PIN) == 0U);
    black[3] = (DL_GPIO_readPins(XG_G4_PORT, XG_G4_PIN) == 0U);
    black[4] = (DL_GPIO_readPins(XG_G5_PORT, XG_G5_PIN) == 0U);
    black[5] = (DL_GPIO_readPins(XG_G6_PORT, XG_G6_PIN) == 0U);
    black[6] = (DL_GPIO_readPins(XG_G7_PORT, XG_G7_PIN) == 0U);
    black[7] = (DL_GPIO_readPins(XG_G8_PORT, XG_G8_PIN) == 0U);

    /* 逐个统计检测到黑线的探头。 */
    for (i = 0U; i < 8U; ++i) {
        if (black[i] != 0U) {
            /* 在 line_mask 对应位置 1，方便调试查看八路组合。 */
            line_mask |= (uint8_t)(1U << i);

            /* 把当前探头的位置数字加到总和中。 */
            position_sum += weight[i];

            /* 检测到黑线的探头数量加 1。 */
            ++line_count;
        }
    }

    /* line_mask 供调试器观察，强制读取一次避免编译器警告。 */
    (void)line_mask;

    /* ==================== 第二步：读取编码器里程 ==================== */

    /* 读取左右轮累计编码器位置。 */
    left_encoder = Motor_GetLeftEncoderPosition();
    right_encoder = Motor_GetRightEncoderPosition();

    /* 不关心计数正负，只取两个轮子的实际累计脉冲大小。 */
    if (left_encoder < 0) left_encoder = -left_encoder;
    if (right_encoder < 0) right_encoder = -right_encoder;

    /* 两个轮子里程的平均值近似表示小车中心走过的距离。 */
    distance = (left_encoder + right_encoder) / 2;

    /* ==================== 第三步：判断一圈终点 ==================== */

    if (line_count < RING_MARK_COUNT) {
        /* 没有六路同时看到黑色，清除 A 点连续检测时间。 */
        mark_ms = 0U;

        /* 行驶超过一圈的 5% 后，确认小车已经离开起点 A。 */
        if (distance > (RING_LAP_PULSES / 20L)) {
            left_start_mark = true;
        }
    } else if (left_start_mark &&
               distance >= ((RING_LAP_PULSES * 8L) / 10L)) {
        /*
         * 已经离开过起点、里程超过一圈的 80%，并且至少六路看到黑线，
         * 这时才开始累计 A 点横线确认时间。
         */
        if (mark_ms < RING_MARK_CONFIRM_MS) {
            mark_ms += RING_PERIOD_MS;
        }

        /* 连续检测达到 25 ms 后，确认已经绕一圈回到 A 点。 */
        if (mark_ms >= RING_MARK_CONFIRM_MS) {
            Motor_Brake();
            g_ring_state = RING_FINISHED;
            return;
        }
    }

    /* ==================== 第四步：处理完全丢线 ==================== */

    if (line_count == 0U) {
        /* 每丢线一个控制周期，丢线时间增加 5 ms。 */
        if (lost_ms < RING_LOST_STOP_MS) {
            lost_ms += RING_PERIOD_MS;
        }

        if (lost_ms <= RING_LOST_HOLD_MS) {
            /* 50 ms 内可能只是信号抖动，继续使用上一次左右轮速度。 */
            Motor_SetSpeed(
                RING_FORWARD_SIGN * last_left_speed,
                RING_FORWARD_SIGN * last_right_speed);
            return;
        }

        if (lost_ms < RING_LOST_STOP_MS) {
            /* 黑线最后在 G1 一侧时，向 G1 一侧低速寻找。 */
            if (filtered_position < 0.0f) {
                Motor_SetSpeed(
                    RING_FORWARD_SIGN * (-RING_SEARCH_SPEED),
                    RING_FORWARD_SIGN * RING_SEARCH_SPEED);
            } else {
                /* 黑线最后在 G8 一侧时，向 G8 一侧低速寻找。 */
                Motor_SetSpeed(
                    RING_FORWARD_SIGN * RING_SEARCH_SPEED,
                    RING_FORWARD_SIGN * (-RING_SEARCH_SPEED));
            }
            return;
        }

        /* 连续 400 ms 找不到黑线时停车，防止小车完全冲出赛道。 */
        Motor_Brake();
        g_ring_state = RING_FAULT;
        return;
    }

    /* 重新找到黑线后，把连续丢线时间清零。 */
    lost_ms = 0U;

    /* ==================== 第五步：计算黑线位置 ==================== */

    /* 黑线位置 = 所有黑线探头的位置之和 ÷ 黑线探头数量。 */
    raw_position = (float)position_sum / (float)line_count;

    /* 新位置占 35%，旧位置占 65%，减少传感器跳变造成的左右摆动。 */
    filtered_position =
        0.65f * filtered_position + 0.35f * raw_position;

    /* ==================== 第六步：按照路段计算基础速度 ==================== */

    if (distance < point_b) {
        /* AB：按照已行驶比例从 24 均匀增加到 48。 */
        g_ring_state = RING_AB_ACCEL;
        progress = (float)distance / (float)point_b;
        base_speed = RING_START_SPEED +
                     (RING_HIGH_SPEED - RING_START_SPEED) * progress;
    } else if (distance < point_c) {
        /* BC：不再加减速，基础速度保持 48。 */
        g_ring_state = RING_BC_CRUISE;
        base_speed = RING_HIGH_SPEED;
    } else if (distance < point_d) {
        /* CD：按照已行驶比例从 48 均匀降低到 36。 */
        g_ring_state = RING_CD_DECEL;
        progress = (float)(distance - point_c) /
                   (float)(point_d - point_c);
        base_speed = RING_HIGH_SPEED -
                     (RING_HIGH_SPEED - RING_LOW_SPEED) * progress;
    } else {
        /* DA：不再加减速，基础速度保持 36。 */
        g_ring_state = RING_DA_CRUISE;
        base_speed = RING_LOW_SPEED;
    }

    /* ==================== 第七步：计算 PD 转向修正 ==================== */

    /*
     * KP×当前位置：黑线离中心越远，转向越明显。
     * KD×位置变化：黑线位置变化越快，越早进行抑制，减少来回摆动。
     */
    target_correction =
        RING_KP * filtered_position +
        RING_KD * (filtered_position - last_position);

    /* 保存本次位置，下一次计算 D 项时使用。 */
    last_position = filtered_position;

    /* 把目标修正限制在 -24～24，避免一个车轮突然反转。 */
    if (target_correction > RING_CORRECTION_MAX) {
        target_correction = RING_CORRECTION_MAX;
    }
    if (target_correction < -RING_CORRECTION_MAX) {
        target_correction = -RING_CORRECTION_MAX;
    }

    /* 计算目标修正与当前修正之间还差多少。 */
    correction_change = target_correction - correction;

    /* 每 5 ms 最多增加或减少 2，防止车头突然急转。 */
    if (correction_change > RING_CORRECTION_STEP) {
        correction_change = RING_CORRECTION_STEP;
    }
    if (correction_change < -RING_CORRECTION_STEP) {
        correction_change = -RING_CORRECTION_STEP;
    }
    correction += correction_change;

    /* ==================== 第八步：合成并输出左右轮速度 ==================== */

    /*
     * 从车头前方看：左轮加修正、右轮减修正。
     * 两轮一加一减，所以两轮平均速度仍等于 base_speed。
     */
    front_left_speed = (int16_t)(base_speed + correction);
    front_right_speed = (int16_t)(base_speed - correction);

    /* 把两个电机命令限制在 Motor.h 规定的 -100～100。 */
    if (front_left_speed > MOTOR_SPEED_MAX) front_left_speed = MOTOR_SPEED_MAX;
    if (front_left_speed < MOTOR_SPEED_MIN) front_left_speed = MOTOR_SPEED_MIN;
    if (front_right_speed > MOTOR_SPEED_MAX) front_right_speed = MOTOR_SPEED_MAX;
    if (front_right_speed < MOTOR_SPEED_MIN) front_right_speed = MOTOR_SPEED_MIN;

    /* 保存本次速度，短暂丢线时继续使用。 */
    last_left_speed = front_left_speed;
    last_right_speed = front_right_speed;

    /* 每个控制周期只在这里调用一次正常电机输出，避免控制命令互相覆盖。 */
    Motor_SetSpeed(
        RING_FORWARD_SIGN * front_left_speed,
        RING_FORWARD_SIGN * front_right_speed);
}
