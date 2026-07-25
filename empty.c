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
#include "mylib/pid.h"
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

    static volatile int Timer_count = 0;



float IR_PID_Control(float err);

void stop(void);
void task1(void);
void task2(void);
void task3(void);
void task4(void);

//临时函数任务区
void task5(void);
void task6(void);
void task7(void);
void task8(void);



//==========  全局变量定义区 =============//

//任务1区域：速度闭环控制
static Motor_PID_t g_speed_pid;         /* 速度闭环PID结构体 */
static float      g_speed_target = 0;   /* 目标速度(pulse/s) */
static uint8_t    g_speed_loop_active = 0; /* 速度闭环激活标志 */
static uint8_t    g_speed_print_cnt = 0;   /* 调试打印计数器，降频用 */
static volatile uint32_t TIMA0_count = 0;

//==========  IMU全局数据区（主循环读取，各任务共享） ==========//
static float g_imu_ypr[3] = {0};        /* 姿态角 [yaw, pitch, roll]，单位：度 */
static float g_imu_gyro[3] = {0};       /* 陀螺仪角速度 [gx, gy, gz]，单位：度/秒 */
static uint8_t g_imu_data_valid = 0;    /* 数据有效标志：1=有效，0=无效 */
static int8_t g_imu_status = 0;         /* IMU状态：0=正常，-2=校准中，其他=错误 */
static volatile uint8_t g_imu_ready = 0; /* IMU更新标志：1=有新数据待读取，由定时器中断置位 */
static uint32_t g_imu_last_tick = 0;     /* 上次IMU读取时的TIMA0_count，用于计算真实dt */
static float g_imu_dt = 0.01f;           /* 最近一次IMU读取的真实dt（秒），各任务可直接使用 */
static volatile uint32_t g_imu_sample_seq = 0;

//==========IMU闭环控制区==========//
PID_t g_yaw_pid;
static float g_yaw_target = 0.0f;
static uint32_t last_Timer_Count = 0;

/* ==================== 主函数 ==================== */
int main(void)
{
    SYSCFG_DL_init();
    

    //初始状态
    Cartask state = status_stop;

    OLED_Init();
    OLED_ColorTurn(0);
    OLED_DisplayTurn(0);
    IMU_init();
    OLED_Clear();
    OLED_Refresh();

    /* 初始化编码器 */
    Motor_Encoder_Init();
    Motor_ResetLeftEncoder();
    Motor_ResetRightEncoder();
    /* 使能TIMER_count的NVIC中断（sysconfig漏掉了这一步）
     * 否则TIMG7_IRQHandler永远不会被调用 */
    NVIC_EnableIRQ(TIMER_count_INST_INT_IRQN);
    NVIC_EnableIRQ(DL_TIMERG_INTERRUPT_LOAD_EVENT);
    NVIC_EnableIRQ(TIMER_TICK_INST_INT_IRQN);
    



    uint8_t key_status = 0;
   
    while (1) {
        /* ==================== 主循环IMU数据采集 ==================== */
        /* 基于TIMG7 10ms定时器中断，每10ms触发一次IMU读取 */
        if (g_imu_ready) {
            g_imu_ready = 0;  /* 清除标志（不清零Timer_count，不影响其他任务） */

            /* 计算真实dt：基于TIMA0_count时间戳差值 */
            uint32_t now_tick = TIMA0_count;
            uint32_t elapsed = now_tick - g_imu_last_tick;
            float dt = (float)elapsed * (33.0f / 32768.0f);

            /* dt合理性检查：0.005s~0.1s之间（约5ms~100ms） */
            if (dt < 0.005f || dt > 0.1f) {
                dt = 0.01f;  /* 异常时使用默认值 */
            }

            g_imu_status = IMU_getYawPitchRoll(g_imu_ypr, dt);
            if (g_imu_status == 0) {
                g_imu_data_valid = 1;
                /* 获取陀螺仪原始角速度数据 */
                IMU_GetCorrectedGyro(g_imu_gyro);
                g_imu_sample_seq++;
            } else {
                g_imu_data_valid = 0;
            }

            g_imu_dt = dt;                 /* 保存真实dt供各任务使用 */
            g_imu_last_tick = now_tick;  /* 更新本次时间戳 */
        }
       
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
                task1();
                break;
            case(status_task2):
                task2();
                break;
            case(status_task3):
               
                task3();
                break;
            case(status_task4):
                task5();
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

/* 简介: TIMER_count中断处理函数（5ms定时器）
 * 功能: 每5ms自动调用Motor_Encoder_UpdateSpeed()更新编码器速度和位置
 * 说明: sysconfig中TIMER_count已配置为5ms周期、自动启动 */
void TIMG7_IRQHandler(void)
{
    DL_TimerG_clearInterruptStatus(TIMER_count_INST, DL_TIMERG_INTERRUPT_LOAD_EVENT);
    Motor_Encoder_UpdateSpeed();
    Timer_count++;
    g_imu_ready = 1;  /* 置位IMU更新标志，由主循环读取后清除 */
}
//1ms定时器
void TIMA0_IRQHandler(void)
{
    DL_TimerA_clearInterruptStatus(TIMER_TICK_INST, DL_TIMERA_INTERRUPT_LOAD_EVENT);
    
    TIMA0_count++;


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
    const float max_speed   = 300.0f;  /* 最大目标速度(pulse/s)，对应约18%PWM */
    const float min_speed   = 40.0f;   /* 最低目标速度，避免起步时PID输出为0 */
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
// void task1(void)
// {
//     static uint8_t state = 0;               /* 状态机: 0=初始化, 1=运行中, 2=完成 */
//     const float wheel_c       = PI * 6.5f;  /* 轮子周长(cm) */
//     const float pulse_per_cm  = (ENCODER_PPR * 28.0f) / wheel_c; /* 每厘米脉冲数 = 13*28/周长 */
//     const float target_dist   = 100.0f;     /* 目标距离(cm) */

//     /* 如果速度闭环未激活（被stop()关闭），重置状态机 */
//     if (!g_speed_loop_active) {
//         state = 0;
//     }

//     /* ===== 状态0：初始化 ===== */
//     if (state == 0) {
//         Motor_ResetLeftEncoder();           /* 重置左编码器位置 */
//         Motor_ResetRightEncoder();          /* 重置右编码器位置 */
//         Motor_Enable();                     /* 使能电机 */

 
//         Motor_PID_Init(&g_speed_pid, 0.45f, 0.2f, 0.0f); /* 初始化速度PID */
//         g_speed_target      = 0;
//         g_speed_loop_active = 1;            /* 激活速度闭环 */
//         state = 1;
//     }

//     /* ===== 状态1：运行中 ===== */
//     if (state == 1) {
//         /* 等待编码器中断触发新的采样（约5ms一次） */
//         if (Timer_count >= 1) {
//             Timer_count = 0;                /* 消耗本次采样标志 */

//             /* ① 计算当前行驶距离 */
//             float current_dist = Motor_GetLeftEncoderPosition() / pulse_per_cm;

//             /* ② 到达目标 → 停止 */
//             if (current_dist >= target_dist) {
//                 Motor_Disable();
//                 /* 不在这里清零 g_speed_loop_active！
//                  * 否则下次 task1() 会检测到 !g_speed_loop_active → state=0 → Motor_Enable()
//                  * 重置由 stop()/按键0 负责 */
//                 printf("Task1 done! Final dist: %.1f cm\r\n", current_dist);
//                 state = 2;
//                 return;
//             }

//             printf("yaw:%f pitch:%f roll:%f\r\n",
//                    g_imu_ypr[0], g_imu_ypr[1], g_imu_ypr[2]);
            
//             /* ③ 梯形速度规划：根据当前距离计算目标速度 */
//             g_speed_target = plan_speed(current_dist, target_dist);

//             /* ④ 读取实际速度（左右轮平均） */
//             float actual_speed = (Motor_GetLeftEncoderSpeed()
//                                 + Motor_GetRightEncoderSpeed()) / 2.0f;

//             /* ⑤ 速度闭环PID计算 */
//             g_speed_pid.target = g_speed_target;
//             float output = Motor_PID_Calculate(&g_speed_pid, actual_speed);

//             /* ⑥ 输出限幅（output越大=需要越快） */
//             if (output > 85.0f) output = 85.0f;
//             if (output < 5.0f)  output = 5.0f;

//             /* ⑦ 驱动电机
//              * Motor_SetSpeed映射：值越大→电机越慢（0=全速, 100=停止）
//              * PID输出：值越大→需要越快，因此用 100-output 反转映射 */
//             int16_t motor_cmd = 100 - (int16_t)output;
//             Motor_SetSpeed(motor_cmd, motor_cmd);

//             /* 调试打印：每200ms(40次)打印一次，避免串口刷屏 */
//             g_speed_print_cnt++;
//             if (g_speed_print_cnt >= 40) {
//                 g_speed_print_cnt = 0;
//                 printf("dist:%.1f target:%.1f actual:%.1f out:%.1f cmd:%d\r\n",
//                        current_dist, g_speed_target, actual_speed, output, motor_cmd);
//             }
//         }
//     }

//     /* 状态2：完成，什么都不做 */
// }

/* 任务1暂未启用，保留安全实现以满足主循环调用。 */
void task1(void)
{
    stop();
}

void task2(void)
{
    static uint8_t state = 0;
    static PID_t left_pid;
    static PID_t right_pid;
    static uint8_t print_count = 0;

    const float left_target_pps = 500.0f;
    const float right_target_pps = 500.0f;

    /* 按下停止键后，允许下一次重新初始化 */
    if (!g_speed_loop_active) {
        state = 0;
    }

    /* 第一次进入task2时初始化 */
    if (state == 0) {
        Motor_ResetLeftEncoder();
        Motor_ResetRightEncoder();

        /*
         * 左右轮分别使用一个PID。
         * 初期可以使用相同参数，之后再分别调整。
         */
        PID_Init(&left_pid,  0.91f, 0.005f, 0.1f, 500.0f, -1250.0f, 1250.0f);
        PID_Init(&right_pid, 0.91f, 0.005f, 0.0f, 500.0f, -1250.0f, 1250.0f);

        Timer_count = 0;
        g_speed_loop_active = 1;

        Motor_Enable();
        state = 1;
    }

    /* TIMG7每10ms更新一次编码器速度 */
    if (state == 1 && Timer_count >= 1) {
        Timer_count = 0;

        /* 分别读取左右轮实际速度 */
        float left_speed =
            fabsf((float)Motor_GetLeftEncoderSpeed());

        float right_speed =
            fabsf((float)Motor_GetRightEncoderSpeed());

        /* 两个PID分别进行计算 */
        float left_output =
            PID_Calc(&left_pid, left_target_pps, left_speed);

        float right_output =
            PID_Calc(&right_pid, right_target_pps, right_speed);

        /* 限制PID输出范围，对应约5%～85%的PWM */

        /*
         * 本工程Motor_SetSpeed参数越小，实际PWM越大，
         * 因此需要使用100-output进行反向转换。
         */
        /* Target, feedback, PID correction, and motor target are all pulse/s. */
        float left_motor_pps = left_target_pps + left_output;
        float right_motor_pps = right_target_pps + right_output;


        /* 左右轮使用不同的控制量 */
        Motor_SetSpeedPps(left_motor_pps, right_motor_pps);

        /* 每200ms打印一次，观察调速效果 */
        if (++print_count >= 20) {
            print_count = 0;

            printf(
                "target L:%.0f R:%.0f pps | "
                "speed L:%.0f R:%.0f pps | "
                "pidout L:%.0f R:%.0f pps | "
                "motor L:%.0f R:%.0f pps\r\n",
                left_target_pps,
                right_target_pps,
                left_speed,
                right_speed,
                left_output,
                right_output,
                left_motor_pps,
                right_motor_pps
            );
        }
    }
}



void task3(void)
{
    static uint8_t task3_state = 0;

    /* 初始化：清除OLED */
    if (task3_state == 0) {
        OLED_Clear();
        OLED_Refresh();
        task3_state = 1;
        printf("task3 start, GYRO_CONFIG0=0x%02X (expected 0x28)\r\n",
               ICM42688_ReadReg(ICM42688_REG_GYRO_CONFIG0, ICM42688_BANK_0));
        return;
    }

    /* 检查IMU数据是否有效 */
    if (g_imu_data_valid == 0) {
        return;
    }

    /* 使用全局IMU数据（由主循环每10ms更新一次） */
    float yaw = g_imu_ypr[0];
    float pitch = g_imu_ypr[1];
    float roll = g_imu_ypr[2];
    float gz = g_imu_gyro[2];  /* z轴角速度，度/秒 */

    /* 累计yaw角度（积分） */
    static float yaw_total = 0;
    static float gyro_total_x = 0.0f;
    static float gyro_total_y = 0.0f;
    static float gyro_total_z = 0.0f;
    static uint8_t print_div = 0;
    static uint32_t last_sample_seq = 0;
    static float last_yaw = 0.0f;
    static uint8_t yaw_initialized = 0;
    if (g_imu_sample_seq == last_sample_seq) {
        return;
    }
    last_sample_seq = g_imu_sample_seq;

    gyro_total_x += g_imu_gyro[0] * g_imu_dt;
    gyro_total_y += g_imu_gyro[1] * g_imu_dt;
    gyro_total_z += g_imu_gyro[2] * g_imu_dt;

    if (!yaw_initialized) {
        last_yaw = yaw;
        yaw_initialized = 1;
    } else {
        float yaw_delta = yaw - last_yaw;
        if (yaw_delta > 180.0f) yaw_delta -= 360.0f;
        if (yaw_delta < -180.0f) yaw_delta += 360.0f;
        yaw_total += yaw_delta;
        last_yaw = yaw;
    }

    /* 每40次（200ms）打印和更新OLED一次 */
    if (++print_div >= 20) {
        icm42688_raw_data_t raw_gyro;
        print_div = 0;
        (void)ICM42688_ReadGyroRaw(&raw_gyro);
        printf("dt:%.4f YPR:%.2f %.2f %.2f | gyro:%.2f %.2f %.2f rawZ:%d | sum:%.2f %.2f %.2f\r\n",
               g_imu_dt, yaw, pitch, roll,
               g_imu_gyro[0], g_imu_gyro[1], g_imu_gyro[2],
               raw_gyro.z,
               gyro_total_x, gyro_total_y, gyro_total_z);
        OLED_ShowString(0, 0, (uint8_t *)"yaw:", 16);
        OLED_ShowFloat(25, 0, yaw, 3, 2, 16);
        OLED_ShowString(0, 18, (uint8_t *)"pitch:", 16);
        OLED_ShowFloat(48, 18, pitch, 3, 2, 16);
        OLED_ShowString(0, 36, (uint8_t *)"roll:", 16);
        OLED_ShowFloat(25, 36, roll, 3, 2, 16);
        OLED_Refresh();
    }
}


void task4(void)
{
    printf("TimA0:%d\r\n",TIMA0_count);
    delay_ms(100);
}



void task5(void)
{
    //角度环PID控制
    static uint8_t task5_state = 0;
    static int16_t base_speed = 0;
    static float turn_out = 0;
    static uint8_t print_div = 0;

    //初始化
    if(task5_state == 0){
        Motor_ResetLeftEncoder();
        Motor_ResetRightEncoder();
        Motor_Enable();
        IMU_init();

        if(g_imu_data_valid){//如果已经初始化陀螺仪，则闭环目标为当前角度
            g_yaw_target = g_imu_ypr[0];
        } else {
            g_yaw_target = 0.0f;
        }

        PID_Init(&g_yaw_pid, 0.45f, 0.0f, 0.1f, 100, -30, 30);
        task5_state = 1;
    }

    if(!g_imu_data_valid)return;

    float current_yaw = g_imu_ypr[0];
    float err = Yaw_Error(g_yaw_target,current_yaw);

    turn_out = PID_Calc(&g_yaw_pid, err, 0.0f);

    int16_t left_ctrl  = base_speed + (int16_t)turn_out;
    int16_t right_ctrl = base_speed - (int16_t)turn_out;

    if (left_ctrl > 20) left_ctrl = 20;
    if (left_ctrl < -20) left_ctrl = -20;
    if (right_ctrl > 20) right_ctrl = 20;
    if (right_ctrl < -20) right_ctrl = -20;

    Motor_SetSpeed(left_ctrl, right_ctrl);
    
    if(Timer_count >last_Timer_Count )
    {
        last_Timer_Count = Timer_count;
        print_div++;
    }

    //数据输出
    if (print_div >= 20) {
        print_div = 0;
        
        //输出调试
        printf("out:%f,%f,%d,%d,%f\r\n",g_yaw_target,current_yaw,left_ctrl,right_ctrl,turn_out);

        //
        OLED_ShowString(0, 0, (uint8_t *)"yaw:", 16);
        OLED_ShowFloat(25, 0, current_yaw, 3, 2, 16);
        OLED_Refresh();
    }
}
