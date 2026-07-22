#include "ti_msp_dl_config.h"
#include "mylib/Motor.h"
#include "mylib/key.h"
#include "mylib/delay.h"
#include "mylib/usart.h"
#include "mylib/find.h"
#include "mylib/oled.h"
#include "mylib/icm42688.h"
#include "mylib/IMU.h"
#include "mylib/I2C_communication.h"
#include <stdio.h>
#include <math.h>


#define PI 3.14159
//定义任务状态结构体
typedef enum 
{
    status_stop,
    status_task1,
    status_task2,
    status_task3,
    status_task4
}Cartask;

/* 重定向printf到串口 */
int fputc(int ch, FILE *stream)
{
    while (DL_UART_isBusy(UART_XG_INST) == true) {}
    DL_UART_Main_transmitData(UART_XG_INST, ch);
    return ch;
}

int fputs(const char *restrict s, FILE *restrict stream)
{
    uint16_t len = 0;
    while (*s) {
        while (DL_UART_isBusy(UART_XG_INST) == true) {}
        DL_UART_Main_transmitData(UART_XG_INST, *s++);
        len++;
    }
    return len;
}

int puts(const char *_ptr) {
    while (*_ptr) {
        while (DL_UART_isBusy(UART_XG_INST) == true) {}
        DL_UART_Main_transmitData(UART_XG_INST, *_ptr++);
    }
    while (DL_UART_isBusy(UART_XG_INST) == true) {}
    DL_UART_Main_transmitData(UART_XG_INST, '\n');
    return 0;
}


//参数定义区
    static float kp = 0.8;
    static float kd = 0.14;
    static float ir_last_err = 0;

    static int Timer_count = 0;

    static float base_speed = 50;

float IR_PID_Control(float err);

void stop(void);
void task1(void);
void task2(void);
void task3(void);
void task4(void);


//==========  全局变量定义区 =============//

//任务1区域：速度闭环控制
static Motor_PID_t g_speed_pid;         /* 速度闭环PID结构体 */
static float      g_speed_target = 0;   /* 目标速度(pulse/s) */
static uint8_t    g_speed_loop_active = 0; /* 速度闭环激活标志 */
static uint8_t    g_speed_print_cnt = 0;   /* 调试打印计数器，降频用 */




/* ==================== 主函数 ==================== */
int main(void)
{
    SYSCFG_DL_init();

    //初始状态
    Cartask state = status_stop;

    OLED_Init();
    //IMU_init();
    printf("IMU init done. Press KEY4 for ICM42688 test.\r\n");
    /* 初始化编码器 */
    Motor_Encoder_Init();
    Motor_ResetLeftEncoder();
    Motor_ResetRightEncoder();
    /* 使能TIMER_count的NVIC中断（sysconfig漏掉了这一步）
     * 否则TIMG7_IRQHandler永远不会被调用 */
    NVIC_EnableIRQ(TIMER_count_INST_INT_IRQN);
    NVIC_EnableIRQ(DL_TIMERG_INTERRUPT_LOAD_EVENT);

    uint8_t ir[IR_NUM];

    uint8_t key_status = 0;
   
    while (1) {
      //  printf("right encoder: %d\r\n",Motor_GetRightEncoderPosition());
      //  printf("left encoder : %d\r\n",Motor_GetLeftEncoderPosition());
      //  printf("GB2 :%d\r\n",DL_GPIO_readPins(Motor_GB2_PORT,Motor_GB2_PIN));
      
        key_status = key_read();
        if (key_status == 0) state = status_stop;
        else if (key_status == 1) state = status_task1;
        else if (key_status == 2) state = status_task2;
        else if (key_status == 3) state = status_task3;
        else if (key_status == 4) state = status_task4;
       
        switch(state)
        {
            case(status_stop):
            
                stop();
                break;
            case(status_task1):
            DL_GPIO_togglePins(LED_PORT,LED_PIN22_PIN);
            delay_ms(100);
                task1();
                break;
            case(status_task2):
                task2();
                break;
            case(status_task3):
                task3();
                break;
            case(status_task4):
                task4();
                break;

        }
       
    //    IR_Read(ir);//读取循迹
    //    float err = IR_GetError(ir);//计算误差
    //    printf("Error: %f\r\n", err);
       
    //    float correction = IR_PID_Control(err);
    // float k = 0.8;//差速控制
    // float diff = k* correction *fabs(correction);
    
    // if(diff > 40 )diff = 40;
    // if(diff < -40)diff = -40;
    // printf("diff : %f\r\n",diff);
    // float base = 40 -1 *fabs(err);
    // if (base <15) base = 15;

    // int right = base -diff;
    // int left = base +diff;
    // Motor_SetSpeed(left, right);

    //    printf("Left Speed:  %d pulse/s\r\n", Motor_GetLeftEncoderSpeed());
    //     printf("Right Speed: %d pulse/s\r\n", Motor_GetRightEncoderSpeed());
    //    printf("correction : %f\r\n",correction);
    // delay_ms(100);
    


    // if(IR_GetSensorCount(ir) == 0)
    // {
    //     if(Timer_count >2000){
    //     Motor_Disable();
    //     break;
    //     }
    //     Timer_count =0;
    // }
        



    }
}

/* 简介: TIMER_count中断处理函数（50ms定时器）
 * 功能: 每50ms自动调用Motor_Encoder_UpdateSpeed()更新编码器速度和位置
 * 说明: sysconfig中TIMER_count已配置为5ms周期、自动启动 */
void TIMG7_IRQHandler(void)
{
    DL_TimerG_clearInterruptStatus(TIMER_count_INST, DL_TIMERG_INTERRUPT_LOAD_EVENT);
    Motor_Encoder_UpdateSpeed();
    Timer_count++;
}

float IR_PID_Control(float err)
{
        float output = kp * err + kd *(err- ir_last_err);
        ir_last_err = err;
        return output;
}

void stop(void)
{
    g_speed_loop_active = 0;    /* 关闭速度闭环 */

    Motor_Disable();//关闭电机
   // printf("current_left pos:%d\r\n",Motor_GetLeftEncoderPosition());

    //LED快闪
    DL_GPIO_togglePins(LED_PORT,LED_PIN22_PIN);
    delay_ms(100);
    // static float ypr[3];
    // IMU_getYawPitchRoll(ypr);
    // printf("yaw:%f pitch:%f roll:%f\r\n",ypr[0],ypr[1],ypr[2]);
    // delay_ms(100);
    
}

/*
 * 梯形速度规划：根据当前行驶距离，计算目标速度
 * 加速段(0~30cm) → 匀速段(30~70cm) → 减速段(70~100cm)
 * 参数可根据实际测试调整
 */
static float plan_speed(float current_dist, float target_dist)
{
    const float max_speed   = 400.0f;  /* 最大目标速度(pulse/s)，对应约22%PWM */
    const float min_speed   = 50.0f;   /* 最低目标速度，避免起步时PID输出为0 */
    const float accel_dist  = 30.0f;   /* 加速段距离(cm) */
    const float decel_dist  = 40.0f;   /* 减速段距离(cm) */
    float speed;

    if (current_dist < accel_dist) {
        /* 加速段：速度从min_speed线性增大到max_speed */
        speed = min_speed + (max_speed - min_speed) * (current_dist / accel_dist);
    } else if (current_dist > target_dist - decel_dist) {
        /* 减速段：速度从max_speed线性减小到min_speed */
        float remain = target_dist - current_dist;
        if (remain < 0) remain = 0;
        speed = min_speed + (max_speed - min_speed) * (remain / decel_dist);
    } else {
        /* 匀速段 */
        speed = max_speed;
    }

    return speed;
}

//任务一：直线行驶（速度闭环控制 + 梯形速度规划）
void task1(void)
{
    static uint8_t state = 0;               /* 状态机: 0=初始化, 1=运行中, 2=完成 */
    const float wheel_c       = PI * 6.5f;  /* 轮子周长(cm) */
    const float pulse_per_cm  = (ENCODER_PPR * 28.0f) / wheel_c; /* 每厘米脉冲数 = 13*28/周长 */
    const float target_dist   = 100.0f;     /* 目标距离(cm) */

    /* 如果速度闭环未激活（被stop()关闭），重置状态机 */
    if (!g_speed_loop_active) {
        state = 0;
    }

    /* ===== 状态0：初始化 ===== */
    if (state == 0) {
        Motor_ResetLeftEncoder();           /* 重置左编码器位置 */
        Motor_ResetRightEncoder();          /* 重置右编码器位置 */
        Motor_Enable();                     /* 使能电机 */

 
        Motor_PID_Init(&g_speed_pid, 0.45f, 0.2f, 0.0f); /* 初始化速度PID */
        g_speed_target      = 0;
        g_speed_loop_active = 1;            /* 激活速度闭环 */
        state = 1;
    }

    /* ===== 状态1：运行中 ===== */
    if (state == 1) {
        /* 等待编码器中断触发新的采样（约5ms一次） */
        if (Timer_count >= 1) {
            Timer_count = 0;                /* 消耗本次采样标志 */

            /* ① 计算当前行驶距离 */
            float current_dist = Motor_GetLeftEncoderPosition() / pulse_per_cm;

            /* ② 到达目标 → 停止 */
            if (current_dist >= target_dist) {
                Motor_Disable();
                /* 不在这里清零 g_speed_loop_active！
                 * 否则下次 task1() 会检测到 !g_speed_loop_active → state=0 → Motor_Enable()
                 * 重置由 stop()/按键0 负责 */
                printf("Task1 done! Final dist: %.1f cm\r\n", current_dist);
                state = 2;
                return;
            }

            static float ypr[3];
            IMU_getYawPitchRoll(ypr);
            printf("yaw:%f pitch:%f roll:%f\r\n",ypr[0],ypr[1],ypr[2]);
            
            /* ③ 梯形速度规划：根据当前距离计算目标速度 */
            g_speed_target = plan_speed(current_dist, target_dist);

            /* ④ 读取实际速度（左右轮平均） */
            float actual_speed = (Motor_GetLeftEncoderSpeed()
                                + Motor_GetRightEncoderSpeed()) / 2.0f;

            /* ⑤ 速度闭环PID计算 */
            g_speed_pid.target = g_speed_target;
            float output = Motor_PID_Calculate(&g_speed_pid, actual_speed);

            /* ⑥ 输出限幅（output越大=需要越快） */
            if (output > 85.0f) output = 85.0f;
            if (output < 5.0f)  output = 5.0f;

            /* ⑦ 驱动电机
             * Motor_SetSpeed映射：值越大→电机越慢（0=全速, 100=停止）
             * PID输出：值越大→需要越快，因此用 100-output 反转映射 */
            int16_t motor_cmd = 100 - (int16_t)output;
            Motor_SetSpeed(motor_cmd, motor_cmd);

            /* 调试打印：每200ms(40次)打印一次，避免串口刷屏 */
            g_speed_print_cnt++;
            if (g_speed_print_cnt >= 40) {
                g_speed_print_cnt = 0;
                printf("dist:%.1f target:%.1f actual:%.1f out:%.1f cmd:%d\r\n",
                       current_dist, g_speed_target, actual_speed, output, motor_cmd);
            }
        }
    }

    /* 状态2：完成，什么都不做 */
}

void task2(void)
{

    Motor_Enable();
    Motor_SetSpeed(20, 20);
    
}

void task3(void)
{
    static uint8_t first = 1;
    icm42688_real_data_t acc, gyro;
    icm42688_raw_data_t  raw_acc, raw_gyro;
    float temp;

    if (first) {
        first = 0;
        printf("\r\n===== ICM42688 Sensor Test =====\r\n");
        printf("  I2C Addr : 0x69 (PA10=SDA, PA11=SCL)\r\n");
        printf("  Accel FS : +/-4g, ODR=100Hz\r\n");
        printf("  Gyro  FS : +/-1000dps, ODR=100Hz\r\n");
        printf("================================\r\n\r\n");
    }


    ICM42688_ReadTemperature(&temp);
    printf("Temp : %.2f C\r\n", temp);


    ICM42688_ReadAccelRaw(&raw_acc);
    printf("Acc  Raw : X=%6d  Y=%6d  Z=%6d\r\n",
           raw_acc.x, raw_acc.y, raw_acc.z);


    ICM42688_ReadGyroRaw(&raw_gyro);
    printf("Gyro Raw : X=%6d  Y=%6d  Z=%6d\r\n",
           raw_gyro.x, raw_gyro.y, raw_gyro.z);


    ICM42688_ReadMotion6(&acc, &gyro);
    printf("Acc  : X=%+7.3fg  Y=%+7.3fg  Z=%+7.3fg\r\n",
           acc.x, acc.y, acc.z);
    printf("Gyro : X=%+8.2f  Y=%+8.2f  Z=%+8.2f dps\r\n",
           gyro.x, gyro.y, gyro.z);
    printf("------------------------\r\n");

    delay_ms(1000);
}

/*
 * 任务四：编码器验证测试
 * 功能：关闭电机，手动转动轮子，观察编码器位置变化
 *       用于验证 PPR、QEI 补偿系数、速度计算是否正确
 *
 * 串口输出格式：
 *   [序号] 左位置 增量 | 右位置 增量 | 左速度 右速度
 *
 * 验证方法：
 *   1. 手动转动输出轴（轮子）恰好一圈
 *   2. 观察编码器位置变化量
 *   3. 理论值：输出轴1圈 = 电机轴28圈 = 28*13 = 364 原始脉冲
 *      经 QEI 2X + 代码/4 补偿后，预期增量 ≈ 182
 */
void task4(void)
{
    static uint8_t  state = 0;
    static int32_t  last_left_pos  = 0;
    static int32_t  last_right_pos = 0;
    static uint32_t print_count    = 0;

    if (state == 0) {
        Motor_Disable();                    /* 关闭电机，手动转动 */
        Motor_ResetLeftEncoder();           /* 左编码器归零 */
        Motor_ResetRightEncoder();          /* 右编码器归零 */
        last_left_pos  = 0;
        last_right_pos = 0;
        print_count    = 0;

        printf("\r\n========================================\r\n");
        printf("      Encoder Verification Test\r\n");
        printf("========================================\r\n");
        printf("  PPR           = %d\r\n", ENCODER_PPR);
        printf("  Sample period = %d ms\r\n", ENCODER_SAMPLE_MS);
        printf("  Gear ratio    = 1:28\r\n");
        printf("  Encoder on motor shaft\r\n");
        printf("----------------------------------------\r\n");
        printf("  Expected (output shaft 1 rev):\r\n");
        printf("    Raw QEI 4X counts = %d\r\n", ENCODER_PPR * 28 * 4);
        printf("    After /4 comp     = %d\r\n", ENCODER_PPR * 28);
        printf("    (Left = Right)\r\n");
        printf("----------------------------------------\r\n");
        printf("  Motor DISABLED.\r\n");
        printf("  Rotate wheel by hand to observe.\r\n");
        printf("  Press key 0 to exit.\r\n");
        printf("========================================\r\n\r\n");
        state = 1;
    }

    if (state == 1) {
        /* 每200ms打印一次编码器数据 */
        if (Timer_count >= 40) {
            Timer_count = 0;

            int32_t left_pos   = Motor_GetLeftEncoderPosition();
            int32_t right_pos  = Motor_GetRightEncoderPosition();
            int32_t left_speed = Motor_GetLeftEncoderSpeed();
            int32_t right_speed= Motor_GetRightEncoderSpeed();

            int32_t left_delta  = left_pos  - last_left_pos;
            int32_t right_delta = right_pos - last_right_pos;

            printf("[%3lu] L:%6ld (+%4ld) | R:%6ld (+%4ld) | spd L:%4ld R:%4ld\r\n",
                   print_count,
                   left_pos,  left_delta,
                   right_pos, right_delta,
                   left_speed, right_speed);

            last_left_pos  = left_pos;
            last_right_pos = right_pos;
            print_count++;
        }
    }
}
