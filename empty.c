/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"
#include "mylib/Motor.h"
#include "mylib/delay.h"
#include "mylib/usart.h"
#include "mylib/find.h"
#include "mylib/oled.h"
// #include "mylib/oledfont.h"

/*
 * ==================== 循迹控制 —— 比例控制法 ====================
 *
 * 【传感器布局】 8个传感器从左到右：
 *   G1   G2   G3   G4   G5   G6   G7   G8
 *   ←—— 左侧半区 ——→|←—— 右侧半区 ——→
 *
 * 【核心公式】
 *   speed_diff = position × TURN_GAIN
 *   left_speed  = BASE_SPEED - speed_diff
 *   right_speed = BASE_SPEED + speed_diff
 *
 * 【为什么比 if-else 更丝滑？】
 *   if-else：偏差大=猛转，偏差小=也猛转 → 永远在猛打方向
 *   比例控制：偏差大→猛转，偏差小→微调，偏差0→直行 → 丝滑跟随
 *
 * 【调参指南】
 *   BASE_SPEED：先降到 25~30 跑稳，再逐步加快
 *   TURN_GAIN：转弯太猛→减小，转弯不够→增大
 *   方向反了：把 DIRECTION_SWAP 从 0 改成 1
 */

/* ==================== 可调参数 ==================== */

/* 方向修正：如果方向反了，把 0 改成 1 */
#define DIRECTION_SWAP  0

/* 基础速度：建议从 25 开始，跑稳后再加快 */
#define BASE_SPEED      55

/* 转向灵敏度：值越大转弯越猛（建议 3~6） */
#define TURN_GAIN       4

/* 丢线后寻找黑线的速度 */
#define SEARCH_SPEED    15

/* 循环间隔(ms)：控制采样频率，太快会抖动 */
#define LOOP_DELAY_MS   5

//===================== OLED初始化 ==================*/


int main(void)
{
    SYSCFG_DL_init();
OLED_Init();
OLED_ColorTurn(0);//0 正常 1 反转
OLED_DisplayTurn(0); //0 正常 1 反转
OLED_Refresh();

    /* ==================== OLED 测试代码 ==================== */
      /* 显示2秒后清屏，进入循迹程序 */

    // OLED_Clear();
    // OLED_ShowString(0, 0, (u8 *)"MOTOR READY",16);
    // OLED_Refresh();
    /* ==================== OLED 测试结束 ==================== */

    uint8_t  ir[8];          /* 8路传感器数据 */
    int16_t  position;       /* 黑线位置（-7 ~ +7） */
    int16_t  speed_diff;     /* 左右轮速度差 */
    int16_t  left_speed;     /* 左轮计算速度 */
    int16_t  right_speed;    /* 右轮计算速度 */
    uint8_t  lost_count;     /* 连续丢线计数 */

    Motor_Enable();
    lost_count = 0;

    while (1) {
    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)"MOTOR READY",16);
    OLED_Refresh();
    
        /* ---- 步骤1：读取传感器 ---- */
        IR_Read(ir);

        /* ---- 步骤2：计算黑线位置 ---- */
        position = IR_GetPosition(ir);

        /*
         * ---- 步骤3：比例控制 ----
         *
         * 核心公式：speed_diff = position × TURN_GAIN
         *
         * 举例（TURN_GAIN = 4）：
         *   position =  0  → speed_diff =   0 → 直行
         *   position = -1  → speed_diff =  -4 → 微左转
         *   position = -3  → speed_diff = -12 → 中左转
         *   position = -7  → speed_diff = -28 → 大左转
         *   position = +7  → speed_diff = +28 → 大右转
         */
        speed_diff = position * TURN_GAIN;

        /*
         * ---- 步骤4：计算左右轮速度 ----
         *
         * 左轮 = 基础速度 - 速度差
         * 右轮 = 基础速度 + 速度差
         *
         * 例：position = -3, TURN_GAIN = 4 → speed_diff = -12
         *     左轮 = 25 - (-12) = 37（加速）
         *     右轮 = 25 + (-12) = 13（减速）
         *     → 左轮快、右轮慢 → 左转
         */
#if DIRECTION_SWAP == 0
        left_speed  = BASE_SPEED - speed_diff;
        right_speed = BASE_SPEED + speed_diff;
#else
        /* 方向反转：左右对调 */
        left_speed  = BASE_SPEED + speed_diff;
        right_speed = BASE_SPEED - speed_diff;
#endif

        /* ---- 步骤5：限幅保护 ---- */
        if (left_speed  > 100) left_speed  = 100;
        if (left_speed  < -100) left_speed = -100;
        if (right_speed > 100) right_speed = 100;
        if (right_speed < -100) right_speed = -100;

        /* ---- 步骤6：处理丢线 ---- */
        if (IR_IsLineLost(ir)) {
            lost_count++;

            if (lost_count < 10) {
                /*
                 * 刚丢线（<50ms）：根据上次位置缓慢旋转找回
                 * 注意：position 在丢线时保持为上一次有效值
                 */
                if (position < 0) {
                    /* 上次线偏左，缓慢左转找线 */
                    Motor_SetSpeed(-SEARCH_SPEED, SEARCH_SPEED);
                } else {
                    /* 上次线偏右或居中，缓慢右转找线 */
                    Motor_SetSpeed(SEARCH_SPEED, -SEARCH_SPEED);
                }
            } else {
                /* 丢线超过50ms：停止，避免乱跑 */
                Motor_Stop();
            }
        } else {
            /* 正常循迹 */
            lost_count = 0;
            Motor_SetSpeed(left_speed, right_speed);
        }

        /* ---- 步骤7：循环延时 ---- */
        delay_ms(LOOP_DELAY_MS);
    }
}