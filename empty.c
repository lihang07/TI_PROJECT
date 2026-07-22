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

    static volatile int Timer_count = 0;

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
        OLED_ColorTurn(1);
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

    uint8_t ir[IR_NUM];

    uint8_t key_status = 0;
   
    while (1) {
      //  printf("right encoder: %d\r\n",Motor_GetRightEncoderPosition());
      //  printf("left encoder : %d\r\n",Motor_GetLeftEncoderPosition());
      //  printf("GB2 :%d\r\n",DL_GPIO_readPins(Motor_GB2_PORT,Motor_GB2_PIN));
      OLED_ShowNum(0,0,1,1,14);
      OLED_Refresh();
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
            (void)IMU_getYawPitchRoll(ypr, ENCODER_SAMPLE_MS / 1000.0f);
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
// 在你的循环里加这个
static float yaw_sum = 0;  // 累计角度
static uint32_t cnt = 0;
static uint32_t last_t = 0;
uint32_t now_t = (uint32_t)Timer_count * ENCODER_SAMPLE_MS;
float dt = (now_t - last_t) / 1000.0f;  // 如果你的timer是1ms精度
last_t = now_t;

icm42688_real_data_t av, gv;
ICM42688_ReadMotion6(&av, &gv);  // 直接读原始dps

printf("gx=%.2f gy=%.2f gz=%.2f  |  yaw_sum=%.1f\r\n",
       gv.x, gv.y, gv.z, yaw_sum);

yaw_sum += gv.z * dt;  // gv.z is already degrees per second
cnt++;
}

void task3(void)
{
    static uint8_t task3_state = 0;

    if(!task3_state){


    OLED_Clear();
    OLED_Refresh();
    task3_state = 1;
    }
    
    if(task3_state)
    {
        
        static uint8_t print_div = 0;//打印间隔
        static int last_tick = 0;//上一时刻值
        static float ypr[3];
        float motion[7];
        float dt;//时间
        int now_tick;//当前时间
        int elapsed_ticks;//时间间隔
        static int first = 1;
        int8_t status;

        if (first) {
            last_tick = Timer_count;
            first = 0;
            return;
        }

        now_tick = Timer_count;
        elapsed_ticks = now_tick - last_tick;
        if (elapsed_ticks <= 0) return;
        last_tick = now_tick;
        dt = elapsed_ticks * ENCODER_SAMPLE_MS / 1000.0f;//计算时间
        if (dt > 0.1f) dt = 0.1f;//限制时间

        status = IMU_getYawPitchRoll(ypr, dt);//获取陀螺仪数据并检测是否正常
        if (status != 0) {
            printf("ICM42688 AHRS read failed: %d\r\n", status);
            return;
        }
        IMU_TT_getgyro(motion);//获取陀螺仪数据

        if (++print_div >= 40) {
            //计算加速度
            // float acc_pitch = atan2f(-motion[0],
            //                         sqrtf(motion[1] * motion[1] + motion[2] * motion[2]))
            //                 * 180.0f / PI;
            // float acc_roll = atan2f(motion[1], motion[2]) * 180.0f / PI;
            // float acc_norm = sqrtf(motion[0] * motion[0] + motion[1] * motion[1]
            //                     + motion[2] * motion[2]);
            print_div = 0;
            printf("yaw:%f pitch:%f roll:%f\r\n",ypr[0],ypr[1],ypr[2]);
            OLED_ShowString(0,0,"yaw:",14);
            OLED_ShowFloat(25,0,ypr[0],3,2,14);
            OLED_ShowString(0,10,"pitch:",14);
            OLED_ShowFloat(25,10,ypr[1],3,2,14);
            OLED_ShowString(0,20,"roll:",14);
            OLED_ShowFloat(25,20,ypr[2],3,2,14);
            OLED_Refresh();
        }
    }
    
}


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
