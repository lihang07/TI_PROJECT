#ifndef ZDT_X42S_PULSE_H
#define ZDT_X42S_PULSE_H

#include "ti_msp_dl_config.h"
#include <stdbool.h>
#include <stdint.h>

/*
 * X42S 脉冲控制接口
 *
 * 本库只使用 STP、DIR、EN 三根 IO：定时器 PWM 产生 STP 脉冲，DIR 决定
 * 正反方向，EN 决定驱动器是否输出保持力矩。下面宏的默认值对应当前
 * SysConfig：PB14=STP，PA7=DIR，PB1=EN。迁移工程时，只需要在包含本
 * 文件前重定义这些宏，无需修改 .c 文件。
 */

/*
 * 步进脉冲所用的定时器实例。
 * 默认 BJ_INST 是 SysConfig 中名为 BJ 的 PWM 外设，即 TIMG12。
 * 选择它的意义：PB14 的复用功能已经连接到该定时器 CC1 输出。
 */
#ifndef ZDT_X42S_STEP_INST
#define ZDT_X42S_STEP_INST              BJ_INST
#endif

/*
 * 步进定时器的实际输入时钟频率（Hz）。默认取 SysConfig 自动生成的
 * BJ_INST_CLK_FREQ（当前为 32 MHz），而不是手写 32000000，避免以后修改
 * 时钟树后脉冲频率计算错误。
 */
#ifndef ZDT_X42S_STEP_CLK_HZ
#define ZDT_X42S_STEP_CLK_HZ            BJ_INST_CLK_FREQ
#endif

/*
 * PWM 比较通道号。PB14 使用的是 TIMG12 的 CC1，所以固定为 CC1。
 * 该参数决定库实际修改哪一路 PWM；选错时 PB14 不会出现脉冲。
 */
#ifndef ZDT_X42S_STEP_CC_INDEX
#define ZDT_X42S_STEP_CC_INDEX          DL_TIMER_CC_1_INDEX
#endif

/* DIR 引脚所在 GPIO 端口及引脚位：当前 PA7。 */
#ifndef ZDT_X42S_DIR_PORT
#define ZDT_X42S_DIR_PORT               BJin_DIR_PORT
#endif
#ifndef ZDT_X42S_DIR_PIN
#define ZDT_X42S_DIR_PIN                BJin_DIR_PIN
#endif

/* EN 引脚所在 GPIO 端口及引脚位：当前 PB1。 */
#ifndef ZDT_X42S_EN_PORT
#define ZDT_X42S_EN_PORT                BJin_EN_PORT
#endif
#ifndef ZDT_X42S_EN_PIN
#define ZDT_X42S_EN_PIN                 BJin_EN_PIN
#endif

/*
 * EN 有效电平。X42S 出厂默认“高电平使能”，故设为 1。
 * 含义：1 时本库输出高电平使能、低电平释放；仅当你在电机菜单中将 EN
 * 改为低有效后，才将该值改为 0，否则电机会始终处于错误使能状态。
 */
#ifndef ZDT_X42S_EN_ACTIVE_HIGH
#define ZDT_X42S_EN_ACTIVE_HIGH         1
#endif

/*
 * 正方向对应的 DIR 电平。默认 1 表示“目标脉冲增大时 PA7 输出高电平”。
 * 这是软件坐标定义，并非机械方向规定；若正目标使摆杆朝不希望的方向运动，
 * 将此值改为 0 即可反转方向，无需改控制算法中的正负号。
 */
#ifndef ZDT_X42S_DIR_POSITIVE_HIGH
#define ZDT_X42S_DIR_POSITIVE_HIGH      1
#endif

/*
 * 单个 X42S 轴的运行状态。
 *
 * 脉冲位置是“软件发出的累计脉冲数”，不是编码器的真实绝对位置。因此上电
 * 后必须在已知机械零点调用 Init 或 SetCurrentPosition；限幅只是软件保护，
 * 不能代替机械限位开关。
 */
typedef struct {
    /* 软件允许的最小/最大位置（单位：脉冲）。用于限制摆杆上下摆幅。 */
    int32_t minimum_pulse;
    int32_t maximum_pulse;

    /*
     * 最大 STP 频率（Hz）。它决定最高转速；值越大动作越快，但过大可能丢步、
     * 产生冲击或使摆杆超调。X42S 手册允许的上限很高，但本项目应按机构实测选取。
     */
    uint32_t maximum_rate_hz;

    /*
     * 加速度（脉冲/s²）。它决定每秒能增加多少脉冲频率；值越大响应越快，
     * 但启动/换向冲击越大。Update1ms 将其除以 1000 作为每毫秒的速度改变量。
     */
    uint32_t acceleration_pulse_s2;

    /* 当前软件指令位置和目标位置，单位均为脉冲。 */
    int32_t command_pulse;
    int32_t target_pulse;

    /* Q16 小数脉冲累加器，防止低速时因整除而丢失位移。 */
    int32_t fractional_pulse_q16;

    /* 当前有符号脉冲频率（Hz）；正负号同时表示运动方向。 */
    int32_t signed_rate_hz;

    /* true 表示 EN 已有效；false 时 Update1ms 不输出 STP 脉冲。 */
    bool enabled;
} ZDT_X42S_Pulse;

/*
 * 初始化一个步进轴。
 *
 * motor：状态结构体地址，必须是长期有效的静态/全局变量。
 * minimum_pulse：机械零点向负方向允许的最远软件脉冲数，例如 -200。
 *   取值依据：按实际可摆动行程换算，必须留出机械余量，防止撞击上下限。
 * maximum_pulse：机械零点向正方向允许的最远软件脉冲数，例如 +200。
 *   意义同上；通常和 minimum_pulse 对称，但允许因机构不对称而设不同值。
 * maximum_rate_hz：最大脉冲频率，例如调试时 400 Hz。
 *   400 Hz 便于肉眼观察且冲击小；正式值应由“不丢步且摆杆不过冲”的实测结果决定。
 * acceleration_pulse_s2：加速度，例如调试时 1000 脉冲/s²。
 *   1000 表示频率每 1 ms 最多约变化 1 Hz，起停平缓；机构可靠后再逐步增大。
 * zero_pulse：当前机械位置对应的软件脉冲坐标，通常设 0。
 *   它不是自动回零；只有当机构确实在零点时才能传 0。
 */
void ZDT_X42S_Pulse_Init(ZDT_X42S_Pulse *motor,
                          int32_t minimum_pulse,
                          int32_t maximum_pulse,
                          uint32_t maximum_rate_hz,
                          uint32_t acceleration_pulse_s2,
                          int32_t zero_pulse);

/*
 * 设置 EN 状态。
 * enable=true：按 EN 有效电平使能驱动器，允许输出 STP；
 * enable=false：先停 STP，再释放/禁用驱动器，适合急停或上电保护。
 */
void ZDT_X42S_Pulse_Enable(ZDT_X42S_Pulse *motor, bool enable);

/*
 * 设置目标软件位置（脉冲）。超出 minimum_pulse/maximum_pulse 的值会被
 * 自动截断；这样上层控制即使给出异常目标，也不会继续扩大摆杆运动范围。
 */
void ZDT_X42S_Pulse_SetTarget(ZDT_X42S_Pulse *motor, int32_t target_pulse);

/*
 * 每 1 ms 调用一次的速度规划和 PWM 更新函数。
 * 调用周期必须保持为 1 ms，因为加速度、积分与制动距离均按该周期计算；若改为
 * 其他周期，必须同步修改 .c 中 /1000 的换算关系。
 */
void ZDT_X42S_Pulse_Update1ms(ZDT_X42S_Pulse *motor);

/* 立刻停止 STP 脉冲并清零速度；不改变 command_pulse，电机保持当前位置。 */
void ZDT_X42S_Pulse_Hold(ZDT_X42S_Pulse *motor);

/*
 * 将当前已知机械位置重新记为 pulse，并停止当前运动。
 * pulse 仍会被限幅；仅在机械已回零、或人工确认当前位置时调用。
 */
void ZDT_X42S_Pulse_SetCurrentPosition(ZDT_X42S_Pulse *motor, int32_t pulse);

/* 分别读取目标位置与当前软件指令位置；空指针返回 0。 */
int32_t ZDT_X42S_Pulse_GetTarget(const ZDT_X42S_Pulse *motor);
int32_t ZDT_X42S_Pulse_GetCommand(const ZDT_X42S_Pulse *motor);

#endif
