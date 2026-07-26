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
PID_t g_speed_pid;         /* 速度闭环PID结构体 */
PID_t g_yaw_pid;         /* 陀螺仪闭环PID结构体 */

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
                task6();
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






void task2(void)
{
    static uint8_t state = 0;
    static PID_t left_pid;
    static PID_t right_pid;
    static uint8_t print_count = 0;

    const float left_target_pps = 500.0f;
    const float right_target_pps = 500.0f;



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
                "out:%f,%f,%f,%f\r\n",
                left_target_pps,
                right_target_pps,
                left_output,
                right_output
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
    //初始化任务2，进入直行状态，直到采集到循迹数据
    static uint8_t task2_state = 0;
    static uint8_t task2_state_straight = 1;//进入直线行驶

    if(task2_state == 0){
        Motor_ResetLeftEncoder();
        Motor_ResetRightEncoder();
        Motor_Enable();
        IMU_init();
        PID_Init(&g_yaw_pid, 0.45f, 0.0f, 0.1f, 100, -30, 30);
        task2_state = 1;
    }

    if(task2_state){
        if(task2_state_straight){
            g_yaw_target = g_imu_ypr[0];//获取当前陀螺仪数值作为目标进行直线闭环
            
            if(!g_imu_data_valid)return;
            
            float current_yaw = g_imu_ypr[0];
            //角度和速度双环

        }   
        
        else{
            //循迹和速度双环
        }
    }


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


void task6(void)
{                                                       /* task6状态机开始 */
    static uint8_t g_task6_reset_request;               /* task6的软件复位标志 */

    static uint8_t state = 0;                           /* 状态：0初始化，1第一次直行，2第一次循迹，3第二次直行，4第二次循迹，5停车，6第一次出弯补转，7第二次出弯补转 */
    static uint8_t ir[8] = {0};                         /* 保存8路循迹传感器的采样值 */
    static uint8_t black_line_count = 0;                /* 连续检测到黑线的次数 */
    static uint8_t no_line_count = 0;                   /* 连续没有检测到黑线的次数 */
    static uint8_t print_div = 0;                       /* 调试打印分频计数器 */
    static uint8_t curve_slow_flag = 0;                 /* 弯道末期降速标志 */
    static uint16_t turn_hold_count = 0;                /* 丢线后继续转弯的计数器 */
    static int16_t last_track_left = 28;                /* 丢线前最后一次循迹左轮速度 */
    static int16_t last_track_right = 28;               /* 丢线前最后一次循迹右轮速度 */

    const float wheel_c = PI * 6.5f;                    /* 轮子周长，单位cm */
    const float pulse_per_cm = (ENCODER_PPR * 28.0f) / wheel_c; /* 每厘米对应的编码器脉冲数 */
    const float wheel_base = 16.5f;                     /* 两个轮胎中心之间的距离，单位cm */
    const float target_dist = 100.0f;                   /* 直线段参考距离，单位cm */
    const int16_t straight_speed = 28;                  /* 陀螺仪直线行驶时的基础速度 */
    const int16_t track_base_speed = straight_speed;    /* 循迹基础速度与直线速度保持一致，进入循迹时不主动减速 */
    const int16_t track_slow_speed = 20;                /* 弯道末期的循迹低速 */
    const int16_t max_diff = 35;                        /* 循迹时左右轮最大差速限制 */
    const float curve_slow_angle = 150.0f;              /* 估算转角超过该角度后，认为接近出弯并开始降速 */
    const uint16_t turn_hold_ticks = 100;               /* 丢线后继续转弯约0.5s，定时器周期约5ms */
    const uint8_t line_confirm_count = 3;               /* 连续检测到黑线达到该次数后，才确认找到黑线 */
    const uint8_t lost_line_confirm_count = 5;          /* 连续丢失黑线达到该次数后，才确认离开黑线 */

    if (g_task6_reset_request) {                        /* 判断是否需要重新开始task6 */
        state = 0;                                      /* 回到初始化状态 */
        black_line_count = 0;                           /* 清零黑线确认计数 */
        no_line_count = 0;                              /* 清零丢线确认计数 */
        curve_slow_flag = 0;                            /* 清零弯道末期降速标志 */
        turn_hold_count = 0;                            /* 清零出弯补转计数器 */
        last_track_left = straight_speed;               /* 复位丢线前左轮速度记录 */
        last_track_right = straight_speed;              /* 复位丢线前右轮速度记录 */
        ir_last_err = 0.0f;                             /* 清零循迹PID的上一次误差 */
        g_task6_reset_request = 0;                      /* 清除复位请求 */
    }                                                   /* 复位处理结束 */

    if (state == 0) {                                   /* 状态0：任务初始化 */
        Motor_ResetLeftEncoder();                       /* 左轮编码器清零 */
        Motor_ResetRightEncoder();                      /* 右轮编码器清零 */
        Timer_count = 0;                                /* 控制定时计数清零 */
        black_line_count = 0;                           /* 初始化时默认还没有确认黑线 */
        no_line_count = 0;                              /* 初始化时默认还没有确认丢线 */
        curve_slow_flag = 0;                            /* 初始化时不启用弯道降速 */
        turn_hold_count = 0;                            /* 初始化出弯补转计数器 */
        last_track_left = straight_speed;               /* 初始化丢线前左轮速度记录 */
        last_track_right = straight_speed;              /* 初始化丢线前右轮速度记录 */
        ir_last_err = 0.0f;                             /* 清除循迹PID历史误差 */
        g_yaw_target = 0.0f;                            /* 第一次直行目标角度设为陀螺仪0度 */
        PID_Init(&g_yaw_pid, 0.35f, 0.0f, 0.08f,         /* 按task1的参数初始化直线行驶角度环PID */
                 100.0f, -20.0f, 20.0f);               /* 设置角度环PID积分限幅和输出限幅 */
        PID_Init(&g_speed_pid, 0.45f, 0.1f, 0.01f,       /* 按task1的参数初始化直线行驶速度环PID */
                 50.0f, 5.0f, 60.0f);                  /* 设置速度环PID积分限幅和输出限幅 */
        state = 1;                                      /* 进入第一次直线行驶状态 */
        printf("task6: gyro straight 1\r\n");          /* 串口打印当前进入第一次直行 */
    }                                                   /* 状态0处理结束 */

    if (Timer_count < 1) {                              /* 判断是否到达一次控制周期 */
        return;                                         /* 控制周期未到，直接返回 */
    }                                                   /* 控制周期判断结束 */
    Timer_count = 0;                                    /* 清零控制周期计数，开始本次控制 */

    IR_Read(ir);                                        /* 读取8路循迹传感器 */
    if (IR_GetSensorCount(ir) > 0) {                    /* 判断当前是否至少有一路检测到黑线 */
        if (black_line_count < line_confirm_count) {    /* 防止黑线计数超过确认阈值 */
            black_line_count++;                         /* 黑线连续检测次数加1 */
        }                                               /* 黑线计数限幅结束 */
        no_line_count = 0;                              /* 检测到黑线后，丢线计数清零 */
    } else {                                            /* 当前没有任何传感器检测到黑线 */
        if (no_line_count < lost_line_confirm_count) {  /* 防止丢线计数超过确认阈值 */
            no_line_count++;                            /* 丢线连续次数加1 */
        }                                               /* 丢线计数限幅结束 */
        black_line_count = 0;                           /* 没检测到黑线，黑线确认计数清零 */
    }                                                   /* 循迹传感器防抖处理结束 */

    if (state == 1 || state == 3) {                     /* 状态1或3：使用陀螺仪保持直线行驶 */
        if (!g_imu_data_valid) {                        /* 判断陀螺仪数据是否有效 */
            Motor_Disable();                            /* 陀螺仪无效时关闭电机 */
            return;                                     /* 等待下一次有效陀螺仪数据 */
        }                                               /* 陀螺仪有效性判断结束 */

        float current_yaw = g_imu_ypr[0];               /* 读取当前偏航角 */
        float turn_out = PID_Calc(&g_yaw_pid, g_yaw_target, current_yaw); /* 用角度环PID计算左右轮修正量 */
        float left_dist = Motor_GetLeftEncoderPosition() / pulse_per_cm; /* 计算左轮已经行驶的距离 */
        float right_dist = Motor_GetRightEncoderPosition() / pulse_per_cm; /* 计算右轮已经行驶的距离 */
        float current_position = (left_dist + right_dist) * 0.5f; /* 用左右轮平均距离作为直线段当前位置 */
        float speed_out = PID_Calc(&g_speed_pid, target_dist, current_position); /* 用速度环PID计算基础速度 */

        if (speed_out < straight_speed) speed_out = straight_speed; /* 直线末期不允许速度低于匀速值，避免减速进入循迹 */
        if (speed_out > 60.0f) speed_out = 60.0f;        /* 限制基础速度最大值 */

        int16_t left = (int16_t)(speed_out - turn_out);  /* 左轮速度等于基础速度减角度修正 */
        int16_t right = (int16_t)(speed_out + turn_out); /* 右轮速度等于基础速度加角度修正 */
        if (left > 60) left = 60;                        /* 左轮速度上限保护 */
        if (left < 10) left = 10;                        /* 左轮速度下限保护 */
        if (right > 60) right = 60;                      /* 右轮速度上限保护 */
        if (right < 10) right = 10;                      /* 右轮速度下限保护 */
        Motor_Enable();                                  /* 使能电机驱动 */
        Motor_SetSpeed(left, right);                     /* 设置左右轮速度，实现双环PID直行 */

        if (black_line_count >= line_confirm_count) {    /* 连续检测到黑线后，准备切换到循迹 */
            no_line_count = 0;                           /* 切换前清零丢线计数 */
            ir_last_err = 0.0f;                          /* 切换前清零循迹PID历史误差 */
            curve_slow_flag = 0;                         /* 进入新弯道前清零弯道降速标志 */
            Motor_ResetLeftEncoder();                    /* 进入循迹弯道时清零左轮编码器，用于估算转过角度 */
            Motor_ResetRightEncoder();                   /* 进入循迹弯道时清零右轮编码器，用于估算转过角度 */
            state = (state == 1) ? 2 : 4;                /* 第一次直行后进入第一次循迹，第二次直行后进入第二次循迹 */
            printf("task6: track %d\r\n", (state == 2) ? 1 : 2); /* 串口打印当前进入第几段循迹 */
        }                                               /* 直行切换到循迹处理结束 */
    } else if (state == 2 || state == 4) {               /* 状态2或4：根据循迹模块信号进行循迹 */
        if (no_line_count >= lost_line_confirm_count) {  /* 连续丢线后，认为当前循迹段结束 */
            black_line_count = 0;                        /* 清零黑线确认计数 */
            ir_last_err = 0.0f;                          /* 清零循迹PID历史误差 */
            turn_hold_count = 0;                         /* 准备开始出弯补转计时 */

            if (state == 2) {                            /* 如果结束的是第一段循迹 */
                state = 6;                               /* 进入第一次出弯补转状态 */
                printf("task6: turn hold 1\r\n");       /* 串口打印第一次出弯补转 */
            } else {                                     /* 如果结束的是第二段循迹 */
                state = 7;                               /* 进入第二次出弯补转状态 */
                printf("task6: turn hold 2\r\n");       /* 串口打印第二次出弯补转 */
            }                                           /* 循迹结束后的状态切换完成 */
            return;                                      /* 状态已经切换，本周期直接返回 */
        }                                               /* 丢线结束判断完成 */

        float err = IR_GetError(ir);                     /* 根据8路循迹信号计算黑线位置误差 */
        float correction = IR_PID_Control(err);          /* 根据循迹误差计算PID修正量 */
        float diff = 0.8f * correction * fabsf(correction); /* 对修正量做非线性放大，误差越大转向越强 */
        float track_left_dist = fabsf(Motor_GetLeftEncoderPosition() / pulse_per_cm); /* 计算进入弯道后左轮累计距离 */
        float track_right_dist = fabsf(Motor_GetRightEncoderPosition() / pulse_per_cm); /* 计算进入弯道后右轮累计距离 */
        float curve_angle = fabsf(track_left_dist - track_right_dist) / wheel_base * 180.0f / PI; /* 根据左右轮距离差估算车身转过角度 */

        if (curve_angle >= curve_slow_angle) {           /* 当弯道估算角度接近180度时 */
            curve_slow_flag = 1;                         /* 打开弯道末期降速标志 */
        }                                                /* 弯道降速判断结束 */

        if (diff > max_diff) diff = max_diff;            /* 限制正方向最大差速 */
        if (diff < -max_diff) diff = -max_diff;          /* 限制负方向最大差速 */

        int16_t base = curve_slow_flag ? track_slow_speed : track_base_speed; /* 接近出弯时使用低速，否则使用正常循迹速度 */
        int16_t left = base + (int16_t)diff;             /* 根据基础速度和差速计算左轮循迹速度 */
        int16_t right = base - (int16_t)diff;            /* 根据基础速度和差速计算右轮循迹速度 */

        if (left > 60) left = 60;                        /* 左轮速度上限保护 */
        if (left < 10) left = 10;                        /* 左轮速度下限保护 */
        if (right > 60) right = 60;                      /* 右轮速度上限保护 */
        if (right < 10) right = 10;                      /* 右轮速度下限保护 */

        Motor_SetSpeed(left, right);                     /* 输出循迹时的左右轮速度 */
        last_track_left = left;                          /* 记录当前循迹左轮速度，供丢线后继续转弯使用 */
        last_track_right = right;                        /* 记录当前循迹右轮速度，供丢线后继续转弯使用 */

        if (++print_div >= 20) {                         /* 每20个控制周期打印一次调试信息 */
            print_div = 0;                               /* 清零打印分频计数器 */
            printf("task6 state:%d err:%f angle:%f speed:%d,%d\r\n", state, err, curve_angle, left, right); /* 打印状态、循迹误差、弯道角度和左右轮速度 */
        }                                               /* 调试打印处理结束 */
    } else if (state == 6 || state == 7) {               /* 状态6或7：循迹丢线后继续按原来的转弯趋势补转 */
        Motor_Enable();                                  /* 保持电机使能 */
        Motor_SetSpeed(last_track_left, last_track_right); /* 沿用丢线前最后一次循迹速度继续转弯 */
        turn_hold_count++;                               /* 出弯补转计数加1 */

        if (turn_hold_count >= turn_hold_ticks) {        /* 补转时间达到约0.5s后 */
            turn_hold_count = 0;                         /* 清零补转计数器 */
            black_line_count = 0;                        /* 清零黑线确认计数 */
            no_line_count = 0;                           /* 清零丢线确认计数 */
            curve_slow_flag = 0;                         /* 清零弯道末期降速标志 */

            if (state == 6) {                            /* 如果结束的是第一次出弯补转 */
                Motor_ResetLeftEncoder();                /* 第二次直行开始前清零左轮编码器 */
                Motor_ResetRightEncoder();               /* 第二次直行开始前清零右轮编码器 */
                if (g_imu_data_valid) {                  /* 如果当前陀螺仪数据有效 */
                    g_yaw_target = g_imu_ypr[0];         /* 把补转后的车身方向作为新的直行目标 */
                }                                        /* 当前方向记录结束 */
                PID_Init(&g_yaw_pid, 0.35f, 0.0f, 0.08f, /* 重新初始化角度环PID，清除上一段积分和微分记忆 */
                         100.0f, -20.0f, 20.0f);        /* 设置角度环PID积分限幅和输出限幅 */
                PID_Init(&g_speed_pid, 0.45f, 0.1f, 0.01f, /* 重新初始化速度环PID，清除上一段积分和微分记忆 */
                         50.0f, 5.0f, 60.0f);           /* 设置速度环PID积分限幅和输出限幅 */
                state = 3;                               /* 进入第二次直线行驶 */
                printf("task6: gyro straight 2\r\n");   /* 串口打印当前进入第二次直行 */
            } else {                                     /* 如果结束的是第二次出弯补转 */
                Motor_Disable();                         /* 关闭电机，车辆停车 */
                state = 5;                               /* 进入任务完成状态 */
                printf("task6 done\r\n");               /* 串口打印任务完成 */
            }                                           /* 出弯补转后的状态切换结束 */
        }                                               /* 补转时间判断结束 */
    } else {                                            /* 其他状态，包括任务完成状态 */
        Motor_Disable();                                /* 保持电机关闭 */
    }                                                   /* task6状态判断结束 */
}                                                       /* task6函数结束 */





void task1(void)
{
    static uint8_t state = 0;               /* 状态机: 0=初始化, 1=运行中, 2=完成 */
    const float wheel_c       = PI * 6.5f;  /* 轮子周长(cm) */
    const float pulse_per_cm  = (ENCODER_PPR * 28.0f) / wheel_c; /* 每厘米脉冲数 = 13*28/周长 */
    const float target_dist   = 100.0f;     /* 目标距离(cm) */
    static uint8_t print_div = 0;


    /* ===== 状态0：初始化 ===== */
    if (state == 0) {
        Motor_ResetLeftEncoder();           /* 重置左编码器位置 */
        Motor_ResetRightEncoder();          /* 重置右编码器位置 */
        Motor_Enable();                     /* 使能电机 */
        IMU_init();                         //初始化陀螺仪
        
        if(g_imu_data_valid){//如果已经初始化陀螺仪，则闭环目标为当前角度
            g_yaw_target = g_imu_ypr[0];
        } else {
            g_yaw_target = 0.0f;
        }
        PID_Init(&g_yaw_pid, 0.35f, 0.0f, 0.08f, 100, -20, 20);//初始化角度环PID
        PID_Init(&g_speed_pid, 0.45f, 0.1f, 0.01f, 50, 5, 60);//初始化速度环PID
        state = 1;
    }

    /* ===== 状态1：运行中 ===== */
    if (state == 1) {
        // /* 等待编码器中断触发新的采样（约10ms一次） */
        // if (Timer_count >= 1) {
        //     Timer_count = 0;                /* 消耗本次采样标志 */
        
        
        // }
    if(!g_imu_data_valid)return;

    static float turn_out = 0.0f;
    static float speed_out = 0.0f;
    static const int16_t base_speed = 0;
    const float target_dist   = 100.0f;     /* 目标距离(cm) */

    float current_yaw = g_imu_ypr[0];
    float yaw_err = Yaw_Error(g_yaw_target,current_yaw);

    turn_out = PID_Calc(&g_yaw_pid, g_yaw_target, current_yaw);

    

    float current_position = Motor_GetLeftEncoderPosition() / pulse_per_cm;
    float speed_err = PID_GetError(target_dist,current_position);

    speed_out = PID_Calc(&g_speed_pid, target_dist, current_position);

    int8_t left_out = speed_out - turn_out;
    int8_t right_out = speed_out + turn_out;

    if (left_out > 60) left_out = 20;
    if (left_out < -20) left_out = -20;
    if (right_out > 60) right_out = 60;
    if (right_out < -20) right_out = -20;

    Motor_SetSpeed(left_out, right_out);

    if(Timer_count >last_Timer_Count )
    {
        last_Timer_Count = Timer_count;
        print_div++;
    }

    //数据输出
    if (print_div >= 5) {
        print_div = 0;
        
        //输出调试
        printf("out:%f,%f,%d,%d\r\n",target_dist,current_position,left_out,right_out);

        //
        OLED_ShowString(0, 0, (uint8_t *)"yaw:", 16);
        OLED_ShowFloat(25, 0, current_yaw, 3, 2, 16);
        OLED_ShowFloat(25,20, Motor_GetLeftEncoderPosition(), 6, 0, 16);
        OLED_Refresh();
    }

    if(current_position >= 100.0f)Motor_Disable();

    }

    
}
