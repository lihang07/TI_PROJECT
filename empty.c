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
#define SEARCH_SPEED 10
#define WHEEL_C PI*6.5f
#define TURN_SPEED 15
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
void task6(void);                                      /* 声明A到B到C到D再回A的任务函数 */
void task6_reset(void);                                /* 声明task6重新进入时的复位函数 */
void task7(void);
void task8(void);


//==========  全局变量定义区 =============//


PID_t g_speed_pid;         /* 速度闭环PID结构体 */
PID_t g_speed_pid_right;   /* 右轮速度闭环PID结构体 */
PID_t g_yaw_pid;         /* 陀螺仪闭环PID结构体 */
PID_t g_ir_pid;         /* 循迹闭环PID结构体 */

static volatile uint32_t TIMA0_count = 0;

//==========  IMU全局数据区（主循环读取，各任务共享） ==========//
static volatile float g_imu_ypr[3] = {0}; /* 姿态角由中断更新，使用volatile保证任务读取最新值 */
static float g_imu_gyro[3] = {0};       /* 陀螺仪角速度 [gx, gy, gz]，单位：度/秒 */
static int g_imu_data_valid = 1;
static int8_t g_imu_status = 0;         /* IMU状态：0=正常，-2=校准中，其他=错误 */
static volatile uint8_t g_imu_ready = 0; /* IMU更新标志：1=有新数据待读取，由定时器中断置位 */
static uint32_t g_imu_last_tick = 0;     /* 上次IMU读取时的TIMA0_count，用于计算真实dt */
static float g_imu_dt = 0.01f;           /* 最近一次IMU读取的真实dt（秒），各任务可直接使用 */
static volatile uint32_t g_imu_sample_seq = 0;

//==========IMU闭环控制区==========//
static float g_yaw_target = 0.0f;
static uint32_t last_Timer_Count = 0;
static volatile uint8_t g_task6_reset_request = 0U; /* task6重新进入时的软件复位请求标志 */

//=============循迹控制区==================//
static uint8_t ir[8];

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
    delay_ms(100);
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
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    



    uint8_t key_status = 0;
    Cartask previous_state = status_stop;
   
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

        if (state == status_task2 && previous_state != status_task2) {
            task6_reset();
        }
        previous_state = state;
       
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
               
                task4();
                break;
            case(status_task4):
                task8();
                break;

        }
       



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

}
//1ms定时器
void TIMA0_IRQHandler(void)
{
    DL_TimerA_clearInterruptStatus(TIMER_TICK_INST, DL_TIMERA_INTERRUPT_LOAD_EVENT);
    
    TIMA0_count++;


}

void TIMER_0_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(TIMER_0_INST)) {
    case DL_TIMER_IIDX_ZERO:
        IMU_getYawPitchRoll((float *)g_imu_ypr);  // 获取当前姿态角
        g_imu_data_valid = 1;                    // 已获得一帧姿态数据，允许任务使用陀螺仪闭环
        break;
    default:
        break;
    }
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
    printf("out:%.2f,%.2f,%.2f\r\n",g_imu_ypr[0],g_imu_ypr[1],g_imu_ypr[2]);
    // delay_ms(100);
    
}






void task2(void)
{
    static uint8_t task2_state = 0;
    static float speed_target_P = 35.0f;
    static float speed_err = 0.0f;
    static float speed_left = 0.0f;
    static float speed_right = 0.0f;
    static float speed_out_P = 0.0f;
    static float speed_out_E = 0.0f;

    if(task2_state == 0){
        Motor_ResetLeftEncoder();
        Motor_ResetRightEncoder();
        Motor_Enable();
        task2_state = 1;
        PID_Init(&g_speed_pid, 0.3f, 0.1f, 0.0f, 100, 230, 80*23);
    }

    speed_left = Motor_GetLeftEncoderSpeed();
    speed_right = 0;//Motor_GetRightEncoderSpeed();
    float current_speed_E = (speed_left + speed_right) ;//* 0.5f;
    float speed_target_E = speed_target_P * 23.0f; 
    
    speed_out_E = PID_Calc(&g_speed_pid, speed_target_E, current_speed_E);

    if(speed_out_E > 80*23.0f)speed_out_E = 80*23.0f;
    if(speed_out_E < 10*23.0f)speed_out_E = 10*23.0f;
    speed_out_P = speed_out_E/23.0f;
    
    Motor_SetSpeed(speed_out_P,speed_out_P);
    printf("out:%.2f,%.2f,%.2f\r\n",speed_target_E,speed_out_P,current_speed_E);
    delay_ms(10);
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

    static float gyro_total_x = 0.0f;
    static float gyro_total_y = 0.0f;
    static float gyro_total_z = 0.0f;
    static uint8_t print_div = 0;
    static uint32_t last_sample_seq = 0;
    if (g_imu_sample_seq == last_sample_seq) {
        return;
    }
    last_sample_seq = g_imu_sample_seq;

    gyro_total_x += g_imu_gyro[0] * g_imu_dt;
    gyro_total_y += g_imu_gyro[1] * g_imu_dt;
    gyro_total_z += g_imu_gyro[2] * g_imu_dt;

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
    //任务3：交叉环循迹控制
    static uint8_t task3_state = 0;             //任务3 状态机
    static uint8_t print_div = 0;               //打印次数计数器
    static uint8_t task3_runing_state = 0;    //任务3 运行状态
    static uint8_t ir_lose_count = 0;           //丢线次数计数器
    static float ir_err_filtered = 0.0f;        //IR误差低通滤波值
    static float ir_last_valid_err = 0.0f;      //最近一次有效循迹误差
    static uint32_t task4_last_control_tick = 0; //task4上一次控制时刻
    
    static float speed_target = 20.0f;        //设置基础速度
    static float turn_out = 0.0f;               //转向系数
    static float speed_left = 0.0f;             //左轮速度
    static float speed_right = 0.0f;            //右轮速度
    static int32_t current_dist = 0;            //当前距离
    static int32_t current_pos = 0;             //当前位置

    static float current_yaw = 0.0f;            //当前角度
    static float turn_start_yaw =0.0f;
    static float turn_end_yaw = 0.0f;

    const float pulse_per_cm  = (ENCODER_PPR * 28.0f) / WHEEL_C;
    const float target_dist1   = 138.0f;     /* 目标距离(cm) */
    static const float ir_target = 0.0f;      //IR目标值
    static int16_t position = 0;              //当前位置
    static uint16_t search_speed = 10;       //搜索速度
    static int16_t speed_diff = 0;
    static uint8_t first_track = 1;
    static float position_filtered = 0.0f;
    

    if(task3_state == 0){
        //初始化
        Motor_ResetLeftEncoder();
        Motor_ResetRightEncoder();
      //  IMU_init();不用初始化
      
        delay_ms(500);
        Motor_Enable();
        task3_state = 1;
        task4_last_control_tick = Timer_count;
        ir_err_filtered = 0.0f;
        ir_last_valid_err = 0.0f;
        turn_out = 0.0f;
        if(g_imu_data_valid){//如果已经初始化陀螺仪，则闭环目标为当前角度
            g_yaw_target = g_imu_ypr[0];
            
            } else {
                g_yaw_target = 0.0f;
        }
        PID_Init(&g_yaw_pid, 0.45f, 0.0f, 0.1f, 100, -30, 30);
        PID_Init(&g_ir_pid, 2.5f, 0.0f, 0.5f, 100, -30, 30);
    }
   
    if(task3_state == 1){
        /* 循迹控制固定在10ms执行一次，避免主循环速度变化影响控制效果 */
        if (TIMA0_count <= 2) {
            return;
        }
        TIMA0_count = 0;

        //执行任务
        /*状态1：直线行驶，有循迹信号时切换状态；
          状态2：循迹行驶，连续无循迹信号时切换状态；
          状态3：姿态调整，；
          状态4：直线行驶；
          状态5：循迹行驶；
          状态6：姿态调整*/


        
        if(task3_runing_state == 0){
            IR_Read(ir);
            //状态1：直线行驶，有循迹信号时切换状态
            current_yaw = g_imu_ypr[0];
            float err = Yaw_Error(g_yaw_target,current_yaw);
            //printf("current_yaw:%f err:%f\r\n",current_yaw,err);  
            
            turn_out = PID_Calc(&g_yaw_pid, err, 0.0f);

            speed_left = speed_target - turn_out;
            speed_right = speed_target + turn_out;

            if(speed_left > 70)speed_left = 70;
            if(speed_left < 10)speed_left = 10;
            if(speed_right > 70)speed_right = 70;
            if(speed_right < 10)speed_right = 10;

            Motor_SetSpeed(speed_left,speed_right);

            //判断状态和过线保护
            current_pos = (Motor_GetLeftEncoderPosition() + Motor_GetRightEncoderPosition()) / 2;
            current_dist = current_pos * WHEEL_C / 340  ;
            //printf("current_dist:%d\r\n",current_dist);
            
            if(IR_GetSensorCount(ir) > 0){
                ir_lose_count = 0;//切换到状态1前清零丢线计数
                task3_runing_state = 1;
                turn_start_yaw = g_imu_ypr[0];
                return ;
            }
            
            if (current_dist > target_dist1){//超距，判断是否有巡线信号
                
                Motor_TurnRight(TURN_SPEED);//右转
                
                if(IR_GetSensorCount(ir) == 0){     
                
                    ir_lose_count++;
                    if(ir_lose_count >100){//超时，判定丢线
                        Motor_Disable();
                    }
                  
                } else {
                    ir_lose_count = 0;
                    task3_runing_state = 1;
                    turn_start_yaw = g_imu_ypr[0];
                }

                
            }
            
            



        }
        else if(task3_runing_state == 1){
            //状态2：循迹行驶，连续无循迹信号时切换状态
            Motor_Enable();
            IR_Read(ir);
            
            if(first_track){
                position = IR_GetPosition(ir);
                position_filtered = position;
                first_track = 0;
            }
            else{
                position = IR_GetPosition(ir);
                
            }
            //position_filtered = 0.6 * position + 0.4 * position_filtered;
            speed_diff = (-position/*_filtered*/) * 3.5f;

            speed_left = speed_target - speed_diff;
            speed_right = speed_target + speed_diff;
           
            if(speed_left > 70)speed_left = 70;
            if(speed_left < -70)speed_left = -70;
            if(speed_right > 70)speed_right = 70;
            if(speed_right < -70)speed_right = -70;

            //处理丢线
            if(IR_IsLineLost(ir)){
                if(ir_lose_count == 0) turn_end_yaw = g_imu_ypr[0];
                ir_lose_count++;
                if(ir_lose_count < 10){
                    if(position < 0){
                        Motor_SetSpeed(-search_speed, search_speed);
                    }else{
                        Motor_SetSpeed(search_speed, -search_speed);
                    }
                }else {
                    ir_lose_count = 0;
                    Motor_Disable();
                    float turn_yaw = Yaw_Error(turn_end_yaw,turn_start_yaw);
                    if(turn_yaw > 0) g_yaw_target = turn_end_yaw + 30.0f;
                    else g_yaw_target = turn_end_yaw - 30.0f;
                    task3_runing_state = 2;
                    return;
                }

            }else {
                ir_lose_count = 0;
                Motor_SetSpeed(speed_left,speed_right);
            }
        }
        else if(task3_runing_state == 2){
            Motor_Enable();
            //状态3：姿态调整，有循迹信号时切换状态
            current_yaw = g_imu_ypr[0];
            float yaw_err = Yaw_Error(g_yaw_target,current_yaw);
            turn_out = PID_Calc(&g_yaw_pid, yaw_err, 0.0f);
            float turn_real_out = turn_out + 5;
            Motor_SetSpeed(-turn_real_out , turn_real_out);
            if(fabs(yaw_err) < 5.0f){
                Motor_Disable();
            }

        }
            

    }
//=================调试==================//
    if(Timer_count >last_Timer_Count )
    {
        last_Timer_Count = Timer_count;
        print_div++;
    }

    //数据输出
    // if (print_div >= 20) {
    //     print_div = 0;
        
    //     //输出调试
    //     printf("out:%f,%f,%f,%f,%d\r\n",position,speed_left,speed_right,turn_out,speed_diff);

    //     //
    //     OLED_ShowString(0, 0, (uint8_t *)"left:", 16);
    //     OLED_ShowFloat(25, 0, speed_left, 3, 2, 16);
    //     OLED_ShowString(0, 16, (uint8_t *)"right:", 16);
    //     OLED_ShowFloat(25, 16, speed_right, 3, 2, 16);
    //     OLED_Refresh();
    // }


}

void task8(void)
{
    int16_t  position;       /* 黑线位置（-7 ~ +7） */
    int16_t  speed_diff;     /* 左右轮速度差 */
    int16_t  left_speed;     /* 左轮计算速度 */
    int16_t  right_speed;    /* 右轮计算速度 */
    uint8_t  lost_count = 0;     /* 连续丢线计数 */
    static uint32_t task4_last_control_tick = 0;
    static uint8_t search_speed = 15;
    static int16_t speed_target = 20;
    Motor_Enable();

    if (Timer_count == task4_last_control_tick) {
        return;
    }
    task4_last_control_tick = Timer_count;

    IR_Read(ir);
    position = IR_GetPosition(ir);

    speed_diff = (-position) * 4;
    left_speed = speed_target - speed_diff;
    right_speed = speed_target + speed_diff;

    if(left_speed > 70)left_speed = 70;
    if(left_speed < -70)left_speed = -70;
    if(right_speed > 70)right_speed = 70;
    if(right_speed < -70)right_speed = -70;

    //丢线处理
    if (IR_IsLineLost(ir)){
        lost_count++;

        if(lost_count < 10){
            if(position < 0 ){
                Motor_SetSpeed(-search_speed, search_speed);
            }else{
                Motor_SetSpeed(search_speed, -search_speed);
            }
        }else{
            Motor_Disable();
        }
    }else{
        lost_count = 0;
        Motor_SetSpeed(left_speed, right_speed);
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
    printf("yaw : %f\r\n",current_yaw);

    turn_out = PID_Calc(&g_yaw_pid, err, 0.0f);

    int16_t left_ctrl  = base_speed - (int16_t)turn_out;
    int16_t right_ctrl = base_speed + (int16_t)turn_out;

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

void task1(void)
{
    static uint8_t state = 0;               /* 状态机: 0=初始化, 1=运行中, 2=完成 */
    const float wheel_c       = PI * 6.5f;  /* 轮子周长(cm) */
    const float pulse_per_cm  = (ENCODER_PPR * 28.0f) / wheel_c; /* 每厘米脉冲数 = 13*28/周长 */
    static uint8_t print_div = 0;
    const float target_dist   = 100.0f;     /* 目标距离(cm) */

    /* ===== 状态0：初始化 ===== */
    if (state == 0) {
        Motor_ResetLeftEncoder();           /* 重置左编码器位置 */
        Motor_ResetRightEncoder();          /* 重置右编码器位置 */
        Motor_Enable();                     /* 使能电机 */
        IMU_init();                         //初始化陀螺仪
        delay_ms(100);
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
    

    float current_yaw = g_imu_ypr[0];
    float yaw_err = Yaw_Error(g_yaw_target,current_yaw);
    printf("yaw : %f\r\n",current_yaw);

    turn_out = PID_Calc(&g_yaw_pid, yaw_err, 0.0f);

    

    float current_position = Motor_GetLeftEncoderPosition() / pulse_per_cm;
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


void task6(void)                                        /* 执行A到B到C到D再回A的任务 */
{                                                       /* A到B到C到D再回A的控制状态机开始 */
    typedef enum {                                      /* 定义task6内部运行状态 */
        TASK6_STATE_INIT = 0,                           /* 初始化状态 */
        TASK6_STATE_WAIT_IMU,                           /* 等待陀螺仪数据有效状态 */
        TASK6_STATE_IMU_SETTLE,                         /* 陀螺仪有效后的静置状态 */
        TASK6_STATE_STRAIGHT_AB,                        /* A到B双环直行状态 */
        TASK6_STATE_TRACK_BC,                           /* B到C灰度巡线状态 */
        TASK6_STATE_STRAIGHT_CD,                        /* C到D双环直行状态 */
        TASK6_STATE_TRACK_DA,                           /* D到A灰度巡线状态 */
        TASK6_STATE_FINISHED                            /* 回到A后的停车状态 */
    } Task6_State_t;                                    /* 完成task6状态类型定义 */

    static Task6_State_t state = TASK6_STATE_INIT;      /* 保存task6当前运行状态 */
    static uint8_t ir_value[IR_NUM] = {0U};             /* 保存八路灰度传感器的归一化结果 */
    static uint8_t line_count = 0U;                     /* 保存连续检测到黑线的次数 */
    static uint8_t lost_count = 0U;                     /* 保存连续检测不到黑线的次数 */
    static uint8_t print_div = 0U;                      /* 保存串口调试输出分频计数 */
    static int32_t left_position_origin = 0;            /* 保存当前直线段左轮编码器起点 */
    static int32_t right_position_origin = 0;           /* 保存当前直线段右轮编码器起点 */

    const float wheel_c = PI * 6.5f;                    /* 按直径6.5厘米计算车轮周长 */
    const float pulse_per_cm = (ENCODER_PPR * 28.0f) / wheel_c; /* 按减速比28计算每厘米编码器脉冲数 */
    const float ab_position_target = 100.0f;            /* 根据赛道图设置A到B位置环参考距离为100厘米 */
    const float cd_position_target = 100.0f;            /* C到D也以100厘米作位置环调速参考且不用于切段 */
    const float line_diff_gain = 0.8f;                  /* 设置巡线PID输出的非线性差速增益 */
    const int16_t straight_min_speed = 28;              /* 设置双环直行的最低前进速度 */
    const int16_t straight_max_speed = 55;              /* 设置双环直行的最高前进速度 */
    const int16_t track_base_speed = 36;                /* 设置弧线巡线的基础前进速度 */
    const int16_t motor_min_speed = 10;                 /* 设置行驶中单轮最低有效速度 */
    const int16_t motor_max_speed = 60;                 /* 设置行驶中单轮最高安全速度 */
    const float max_track_diff = 35.0f;                 /* 设置巡线时左右轮最大差速 */
    const uint8_t line_confirm_count = 3U;              /* 连续三次检测到黑线后确认进入巡线 */
    const uint8_t lost_confirm_count = 5U;              /* 连续五次检测不到黑线后确认离开弧线 */
    const uint16_t imu_settle_ticks = 10U;              /* 按十毫秒控制周期设置一百毫秒静置时间 */

    if (g_task6_reset_request != 0U) {                  /* 判断主程序是否请求重新开始task6 */
        state = TASK6_STATE_INIT;                       /* 把状态机恢复到初始化状态 */
        line_count = 0U;                                /* 清零黑线确认计数 */
        lost_count = 0U;                                /* 清零丢线确认计数 */
        print_div = 0U;                                 /* 清零调试输出分频计数 */
        g_task6_reset_request = 0U;                     /* 清除已经处理的软件复位请求 */
    }                                                   /* task6软件复位处理结束 */

    if (state == TASK6_STATE_INIT) {                    /* 第一次进入或重新进入task6时执行初始化 */
        Motor_Disable();                                /* 初始化期间关闭电机以防车辆误动 */
        left_position_origin = Motor_GetLeftEncoderPosition(); /* 记录左轮当前位置并避开QEI硬件未清零问题 */
        right_position_origin = Motor_GetRightEncoderPosition(); /* 记录右轮当前位置作为相对位置起点 */
        Timer_count = 0;                                /* 清零十毫秒控制周期计数 */
        line_count = 0U;                                /* 清零黑线确认计数 */
        lost_count = 0U;                                /* 清零丢线确认计数 */
        print_div = 0U;                                 /* 清零调试输出分频计数 */
        state = TASK6_STATE_WAIT_IMU;                   /* 转入等待陀螺仪数据有效状态 */
        printf("task6: wait imu\r\n");                /* 通过串口提示正在等待陀螺仪 */
        return;                                         /* 本周期保持车辆静止 */
    }                                                   /* task6初始化处理结束 */

    if (state == TASK6_STATE_WAIT_IMU) {                /* 等待主函数中的陀螺仪中断产生有效姿态 */
        Motor_Disable();                                /* 等待期间继续关闭电机 */
        if (g_imu_data_valid == 0U) {                   /* 判断是否仍未收到有效姿态数据 */
            return;                                     /* 数据无效时继续等待下一次调用 */
        }                                               /* 陀螺仪有效性判断结束 */
        Timer_count = 0;                                /* 从数据有效时刻开始计算静置时间 */
        state = TASK6_STATE_IMU_SETTLE;                 /* 转入陀螺仪静置状态 */
        printf("task6: imu settle 100ms\r\n");        /* 通过串口提示静置校准阶段 */
        return;                                         /* 本周期不启动电机 */
    }                                                   /* 等待陀螺仪状态处理结束 */

    if (state == TASK6_STATE_IMU_SETTLE) {              /* 在起步前保持一百毫秒静止 */
        Motor_Disable();                                /* 静置期间继续关闭电机 */
        if (g_imu_data_valid == 0U) {                   /* 检查静置期间姿态数据是否失效 */
            state = TASK6_STATE_WAIT_IMU;               /* 数据失效时返回等待状态 */
            Timer_count = 0;                            /* 重新开始有效数据等待计时 */
            return;                                     /* 本周期结束且保持停车 */
        }                                               /* 静置期间数据检查结束 */
        if (Timer_count < imu_settle_ticks) {           /* 判断一百毫秒静置时间是否达到 */
            return;                                     /* 静置时间不足时继续等待 */
        }                                               /* 静置时间判断结束 */
        Timer_count = 0;                                /* 清零控制周期计数以准备起步 */
        g_yaw_target = g_imu_ypr[0];                    /* 把A点车头方向保存为A到B目标角度 */
        left_position_origin = Motor_GetLeftEncoderPosition(); /* 记录A点左轮编码器读数作为位置零点 */
        right_position_origin = Motor_GetRightEncoderPosition(); /* 记录A点右轮编码器读数作为位置零点 */
        PID_Init(&g_yaw_pid, 0.35f, 0.0f, 0.08f, 100.0f, -20.0f, 20.0f); /* 初始化直行角度环PID */
        PID_Init(&g_speed_pid, 0.45f, 0.10f, 0.01f, 50.0f, 0.0f, 55.0f); /* 初始化直行位置环PID */
        PID_Init(&g_ir_pid, 0.8f, 0.0f, 0.14f, 0.0f, -10.0f, 10.0f); /* 初始化灰度位置PID */
        state = TASK6_STATE_STRAIGHT_AB;                /* 转入A到B双环直行状态 */
        printf("task6: straight AB\r\n");             /* 通过串口提示开始A到B直行 */
        return;                                         /* 下一控制周期再输出电机命令 */
    }                                                   /* 陀螺仪静置处理结束 */

    if (state == TASK6_STATE_FINISHED) {                /* 判断整圈任务是否已经完成 */
        Motor_Disable();                                /* 完成后持续关闭电机 */
        return;                                         /* 完成状态不再执行后续控制 */
    }                                                   /* 完成停车状态处理结束 */

    if (Timer_count < 1) {                              /* 判断十毫秒控制周期是否到达 */
        return;                                         /* 控制周期未到时不重复计算PID */
    }                                                   /* 控制周期判断结束 */
    Timer_count = 0;                                    /* 消耗本次十毫秒控制周期 */
    IR_Read(ir_value);                                  /* 读取八路灰度传感器且一表示黑线 */

    if (state == TASK6_STATE_STRAIGHT_AB || state == TASK6_STATE_STRAIGHT_CD) { /* 处理两个无黑线直线段 */
        float current_yaw;                              /* 保存当前车身偏航角 */
        float yaw_error;                                /* 保存经过正负一百八十度包角的角度误差 */
        float turn_out;                                 /* 保存角度环输出的左右轮差速修正 */
        float left_distance;                            /* 保存左轮累计行驶距离 */
        float right_distance;                           /* 保存右轮累计行驶距离 */
        float current_position;                         /* 保存双轮平均行驶位置 */
        float position_target;                          /* 保存当前直线段的位置环参考值 */
        float base_out;                                 /* 保存位置环输出的基础速度 */
        int16_t left_cmd;                               /* 保存从车头正前方看到的左轮命令 */
        int16_t right_cmd;                              /* 保存从车头正前方看到的右轮命令 */
        uint8_t active_count;                           /* 保存当前检测到黑线的传感器数量 */

        if (g_imu_data_valid == 0U) {                   /* 直行前检查陀螺仪数据是否有效 */
            Motor_Disable();                            /* 姿态数据异常时立即安全停车 */
            return;                                     /* 等待姿态数据恢复后再继续 */
        }                                               /* 直行姿态有效性检查结束 */

        active_count = IR_GetSensorCount(ir_value);     /* 统计当前检测到黑线的灰度通道数 */
        if (active_count > 0U) {                        /* 判断是否接触到下一段黑色弧线 */
            if (line_count < line_confirm_count) {      /* 防止黑线计数超过确认阈值 */
                line_count++;                           /* 增加连续黑线检测次数 */
            }                                           /* 黑线计数限幅处理结束 */
        } else {                                        /* 当前没有任何灰度通道检测到黑线 */
            line_count = 0U;                            /* 清零不连续的黑线检测计数 */
        }                                               /* 直线段黑线防抖处理结束 */

        current_yaw = g_imu_ypr[0];                    /* 读取当前偏航角用于角度环反馈 */
        yaw_error = Yaw_Error(g_yaw_target, current_yaw); /* 计算跨正负一百八十度连续的偏航误差 */
        turn_out = PID_Calc(&g_yaw_pid, yaw_error, 0.0f); /* 用包角后的误差计算角度环修正 */
        left_distance = fabsf((float)(Motor_GetLeftEncoderPosition() - left_position_origin)) / pulse_per_cm; /* 把左轮相对脉冲换算成正向厘米数 */
        right_distance = fabsf((float)(Motor_GetRightEncoderPosition() - right_position_origin)) / pulse_per_cm; /* 把右轮相对脉冲换算成正向厘米数 */
        current_position = (left_distance + right_distance) * 0.5f; /* 用双轮平均距离作为位置环反馈 */
        position_target = (state == TASK6_STATE_STRAIGHT_AB) ? ab_position_target : cd_position_target; /* 选择当前直线段位置参考 */
        base_out = PID_Calc(&g_speed_pid, position_target, current_position); /* 用位置环输出直行基础速度 */
        if (base_out < (float)straight_min_speed) base_out = (float)straight_min_speed; /* 保证到达参考距离后仍寻找黑线 */
        if (base_out > (float)straight_max_speed) base_out = (float)straight_max_speed; /* 限制位置环给出的最高速度 */
        left_cmd = (int16_t)(base_out - turn_out);       /* 按当前实测符号计算正面所见左轮速度 */
        right_cmd = (int16_t)(base_out + turn_out);      /* 按当前实测符号计算正面所见右轮速度 */
        if (left_cmd < motor_min_speed) left_cmd = motor_min_speed; /* 限制左轮最低前进速度 */
        if (left_cmd > motor_max_speed) left_cmd = motor_max_speed; /* 限制左轮最高前进速度 */
        if (right_cmd < motor_min_speed) right_cmd = motor_min_speed; /* 限制右轮最低前进速度 */
        if (right_cmd > motor_max_speed) right_cmd = motor_max_speed; /* 限制右轮最高前进速度 */
        Motor_Enable();                                 /* 确保双路电机驱动已经使能 */
        Motor_SetSpeed(left_cmd, right_cmd);             /* 按正面所见左轮和右轮顺序输出双环控制量 */

        if (line_count >= line_confirm_count) {         /* 连续检测到黑线后确认直线段结束 */
            line_count = 0U;                            /* 清零黑线确认计数 */
            lost_count = 0U;                            /* 清零即将开始的丢线确认计数 */
            PID_Init(&g_ir_pid, 0.8f, 0.0f, 0.14f, 0.0f, -10.0f, 10.0f); /* 清除上一段留下的巡线PID历史量 */
            state = (state == TASK6_STATE_STRAIGHT_AB) ? TASK6_STATE_TRACK_BC : TASK6_STATE_TRACK_DA; /* 进入对应的黑色弧线 */
            printf("task6: track %s\r\n", (state == TASK6_STATE_TRACK_BC) ? "BC" : "DA"); /* 输出当前巡线区段 */
            return;                                     /* 状态切换后等待下一控制周期 */
        }                                               /* 直线切换巡线处理结束 */

        if (++print_div >= 20U) {                       /* 每二百毫秒输出一次直线调试数据 */
            print_div = 0U;                             /* 清零串口输出分频计数 */
            printf("task6:S%u pos=%.2f/%.2f yawErr=%.2f pwm=%d,%d\r\n", (unsigned int)state, current_position, position_target, yaw_error, left_cmd, right_cmd); /* 输出双环反馈和正面所见左右轮命令 */
        }                                               /* 直线调试输出处理结束 */
    } else if (state == TASK6_STATE_TRACK_BC || state == TASK6_STATE_TRACK_DA) { /* 处理两个有黑线弧线段 */
        float line_error;                               /* 保存灰度阵列计算出的黑线位置误差 */
        float line_out;                                 /* 保存灰度位置PID的输出 */
        float track_diff;                               /* 保存非线性放大后的巡线差速 */
        int16_t left_cmd;                               /* 保存从车头正前方看到的左轮命令 */
        int16_t right_cmd;                              /* 保存从车头正前方看到的右轮命令 */
        uint8_t active_count;                           /* 保存当前检测到黑线的传感器数量 */

        active_count = IR_GetSensorCount(ir_value);     /* 统计当前检测到黑线的灰度通道数 */
        if (active_count == 0U) {                       /* 判断当前是否完全离开黑色弧线 */
            if (lost_count < lost_confirm_count) {      /* 防止丢线计数超过确认阈值 */
                lost_count++;                           /* 增加连续丢线检测次数 */
            }                                           /* 丢线计数限幅处理结束 */
        } else {                                        /* 当前仍有至少一路检测到黑线 */
            lost_count = 0U;                            /* 清零不连续的丢线检测计数 */
        }                                               /* 弧线段丢线防抖处理结束 */

        if (lost_count >= lost_confirm_count) {         /* 连续丢线后确认当前黑色弧线结束 */
            line_count = 0U;                            /* 清零下一直线段的黑线确认计数 */
            lost_count = 0U;                            /* 清零已经完成的丢线确认计数 */
            if (state == TASK6_STATE_TRACK_BC) {        /* 判断是否刚完成B到C弧线 */
                if (g_imu_data_valid == 0U) {           /* 切换C到D直线前检查陀螺仪 */
                    Motor_Disable();                    /* 陀螺仪异常时立即停车 */
                    lost_count = lost_confirm_count;    /* 保留切段条件以便数据恢复后继续 */
                    return;                             /* 等待姿态数据恢复 */
                }                                       /* C点陀螺仪有效性检查结束 */
                g_yaw_target = g_imu_ypr[0];            /* 把C点出弧方向保存为C到D目标角度 */
                left_position_origin = Motor_GetLeftEncoderPosition(); /* 记录C点左轮编码器读数作为新位置零点 */
                right_position_origin = Motor_GetRightEncoderPosition(); /* 记录C点右轮编码器读数作为新位置零点 */
                PID_Init(&g_yaw_pid, 0.35f, 0.0f, 0.08f, 100.0f, -20.0f, 20.0f); /* 清除角度环上一段历史量 */
                PID_Init(&g_speed_pid, 0.45f, 0.10f, 0.01f, 50.0f, 0.0f, 55.0f); /* 清除位置环上一段历史量 */
                state = TASK6_STATE_STRAIGHT_CD;        /* 转入C到D双环直行状态 */
                printf("task6: straight CD\r\n");     /* 通过串口提示开始C到D直行 */
            } else {                                    /* 当前刚完成D到A弧线 */
                Motor_Disable();                        /* 回到A点后立即关闭电机 */
                state = TASK6_STATE_FINISHED;           /* 转入任务完成停车状态 */
                printf("task6: arrived A\r\n");       /* 通过串口提示整圈任务完成 */
            }                                           /* 弧线结束后的状态切换完成 */
            return;                                     /* 状态切换后结束本控制周期 */
        }                                               /* 弧线结束判断处理完成 */

        line_error = IR_GetError(ir_value);             /* 用八路加权结果计算黑线左右位置误差 */
        line_out = PID_Calc(&g_ir_pid, line_error, 0.0f); /* 用灰度位置误差计算巡线PID输出 */
        track_diff = line_diff_gain * line_out * fabsf(line_out); /* 对大误差进行非线性差速增强 */
        if (track_diff > max_track_diff) track_diff = max_track_diff; /* 限制向一个方向的最大差速 */
        if (track_diff < -max_track_diff) track_diff = -max_track_diff; /* 限制向另一方向的最大差速 */
        left_cmd = track_base_speed + (int16_t)track_diff; /* 按当前实测符号计算正面所见左轮速度 */
        right_cmd = track_base_speed - (int16_t)track_diff; /* 按当前实测符号计算正面所见右轮速度 */
        if (left_cmd < motor_min_speed) left_cmd = motor_min_speed; /* 限制左轮最低前进速度 */
        if (left_cmd > motor_max_speed) left_cmd = motor_max_speed; /* 限制左轮最高前进速度 */
        if (right_cmd < motor_min_speed) right_cmd = motor_min_speed; /* 限制右轮最低前进速度 */
        if (right_cmd > motor_max_speed) right_cmd = motor_max_speed; /* 限制右轮最高前进速度 */
        Motor_Enable();                                 /* 确保巡线期间电机驱动保持使能 */
        Motor_SetSpeed(left_cmd, right_cmd);             /* 按正面所见左轮和右轮顺序输出巡线差速 */

        if (++print_div >= 20U) {                       /* 每二百毫秒输出一次巡线调试数据 */
            print_div = 0U;                             /* 清零串口输出分频计数 */
            printf("task6:S%u line=%.2f diff=%.2f pwm=%d,%d\r\n", (unsigned int)state, line_error, track_diff, left_cmd, right_cmd); /* 输出灰度误差和正面所见左右轮命令 */
        }                                               /* 巡线调试输出处理结束 */
    } else {                                            /* 捕获理论上不应出现的异常状态 */
        Motor_Disable();                                /* 异常状态下立即关闭电机 */
        state = TASK6_STATE_FINISHED;                   /* 锁定到安全停车状态 */
    }                                                   /* task6运行状态分支结束 */
}                                                       /* A到B到C到D再回A的控制状态机结束 */

void task6_reset(void)                                  /* 定义主程序调用的task6复位接口 */
{                                                       /* task6复位接口开始 */
    g_task6_reset_request = 1U;                         /* 请求task6在下一次调用时完整初始化 */
    Motor_Disable();                                    /* 状态切换瞬间先关闭电机确保安全 */
}                                                       /* task6复位接口结束 */



void task7(void)
{
    //角度环PID控制
    static uint8_t task1_state = 0;
    const float target_dist   = 100.0f;
    static float turn_out = 0;
    static uint8_t print_div = 0;
     static float speed_out = 0.0f;
    const float wheel_c       = PI * 6.5f;
    const float pulse_per_cm  = (ENCODER_PPR * 28.0f) / wheel_c;


    //初始化
    if(task1_state == 0){
        Motor_ResetLeftEncoder();
        Motor_ResetRightEncoder();
        Motor_Enable();
        IMU_init();

        if(g_imu_data_valid){//如果已经初始化陀螺仪，则闭环目标为当前角度
            g_yaw_target = g_imu_ypr[0];
        } else {
            g_yaw_target = 0.0f;
        }
        PID_Init(&g_speed_pid, 0.45f, 0.1f, 0.01f, 50, 5, 60);//初始化速度环PID
        PID_Init(&g_yaw_pid, 0.45f, 0.0f, 0.1f, 100, -30, 30);
        task1_state = 1;
    }

    if(!g_imu_data_valid)return;

    float current_yaw = g_imu_ypr[0];
    turn_out = PID_Calc(&g_yaw_pid, g_yaw_target, current_yaw);


    float current_position = Motor_GetLeftEncoderPosition() / pulse_per_cm;
    speed_out = PID_Calc(&g_speed_pid, target_dist, current_position);

    int16_t left_ctrl  = speed_out - (int16_t)turn_out;
    int16_t right_ctrl = speed_out + (int16_t)turn_out;

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

    
    if(current_position >= 100.0f)Motor_Disable();

  

}

