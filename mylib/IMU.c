#include "IMU.h"
#include "icm42688.h"
#include <stdio.h>

/* ============================================
 * AHRS (Attitude and Heading Reference System) 姿态解算算法
 * ============================================
 * 本算法采用四元数法进行姿态解算,融合加速度计和陀螺仪数据
 * 核心原理:
 *   - 陀螺仪积分: 高频动态响应好,但存在漂移
 *   - 加速度计: 低频稳定,但对震动敏感
 *   - 互补融合: 用加速度计修正陀螺仪的漂移
 */

/* ============================================
 * 地理坐标系定义变量
 * north: 地理北向向量(由加速度计推算)
 * west:  西向向量(正交于north和重力方向)
 * 说明: 用于计算载体坐标系与地理坐标系的转换矩阵
 * ============================================ */
xyz_f_t north, west;

/* ============================================
 * PI控制器变量 - 用于修正陀螺仪漂移
 * exInt, eyInt, ezInt: 积分项累计误差
 * 说明: 当加速度计检测到姿态误差时,通过PI控制器进行补偿
 *       P项提供即时响应,I项消除稳态误差
 * ============================================ */
volatile float exInt, eyInt, ezInt;

/* ============================================
 * 四元数变量 - 表示载体姿态
 * q0, q1, q2, q3: 四元数的四个分量
 * 说明: 四元数q = q0 + q1*i + q2*j + q3*k
 *       用于表示三维旋转,避免欧拉角的万向节锁问题
 * ============================================ */
volatile float q0=1.0f, q1, q2, q3;

/* ============================================
 * 时间相关变量
 * lastUpdate: 上次更新时刻(本项目未使用)
 * now: 当前时刻(本项目未使用)
 * 说明: 用于计算积分时间步长
 * ============================================ */
volatile uint32_t lastUpdate, now;

/* ============================================
 * Yaw角度平滑滤波
 * yaw[5]: 滑动窗口滤波数组
 * 说明: 存储最近5次的yaw值,通过平均减少噪声
 * ============================================ */
volatile float yaw[5]={0,0,0,0,0};

/* ============================================
 * 加速度计零偏
 * Ax_offset, Ay_offset: X/Y轴加速度零偏(本项目未使用)
 * 说明: 用于校准传感器零位误差
 * ============================================ */
int16_t Ax_offset=0, Ay_offset=0;

/* ============================================
 * 陀螺仪数据缓存与统计
 * TTangles_gyro[7]: 陀螺仪数据缓存[ax,ay,az,gx,gy,gz,reserve]
 * Angle_Final[3]: 最终角度(本项目未使用)
 * Kp: PI控制器比例系数 - 控制误差修正强度
 * 说明: Kp越大,对加速度计的信任度越高,响应越快但噪声越大
 * ============================================ */
float TTangles_gyro[7];
float Angle_Final[3];
float Kp = 11.0f;

/* ============================================
 * invSqrt1: 快速计算1/sqrt(x) (牛顿-拉弗森法)
 * 参数: x - 输入值(必须为正数)
 * 返回值: 1/sqrt(x)的近似值
 * 说明:
 *   1. 相比标准库sqrt(),此函数速度快4-5倍
 *   2. 适用于嵌入式系统的实时计算
 *   3. 原理: 通过位操作快速逼近,再用一次牛顿迭代精化
 *   4. 精度误差<0.001%,完全满足姿态解算需求
 * 数学原理:
 *   IEEE754浮点数表示: y = 2^e * m
 *   1/sqrt(y) ≈ 2^(-e/2) * (1/sqrt(m))
 *   关键近似: 1/sqrt(m) ≈ 1.5 - 0.5*m (对于m∈[0.5,1])
 *   常数0x5f3759df是这个近似的最优偏移量
 */
static float invSqrt1(float x) {
    float h=0.5f*x, y=x;
    long i=*(long*)&y;
    i=0x5f3759df-(i>>1);
    y=*(float*)&i;
    return y*(1.5f-(h*y*y));
}

/* IMU_init: IMU模块初始化
 * 流程:
 *   1. 初始化ICM42688传感器硬件
 *   2. 重置四元数初值为单位四元数(表示零姿态)
 *   3. 重置PI控制器积分项为0
 * 返回值: 0=成功, 非0=失败
 */
void IMU_init(void) {
    if (0==ICM42688_Init()) {
        q0=1.0f; q1=q2=q3=0.0f;  /* 单位四元数: q0=1表示无旋转 */
        exInt=eyInt=ezInt=0.0f;  /* 重置积分项 */
        return;
    }
    printf("IMU Init ERROR!!\r\n");
}

/* ============================================
 * 陀螺仪方差计算(用于静止检测)
 * Gf[3][300]: 滑动窗口存储历史陀螺仪数据
 * Gt[3]: 窗口内数据和
 * sqGt[3]: 窗口内数据和的平方(用于计算方差)
 * Gif: 窗口是否已填满标志
 * Gc: 当前数据位置索引
 * go[3]: 静止时的陀螺仪零偏估计
 * Cc: 静止检测计数器
 * 说明: 当陀螺仪数据方差很小时,认为传感器静止
 *       此时可以准确估计陀螺仪零偏
 * ============================================ */
static double Gf[3][500], Gt[3], sqGt[3];
static int Gif=0, Gc=0;
static float go[3]={0};
static int Cc=0;
static uint8_t gyroBiasReady = 0;

/* Board-level gyro gain calibration.  A controlled 90 degree Z-axis turn
 * measured 25.94 degrees before this correction, so 90 / 25.94 = 3.469.
 * Keep the sensor range conversion in icm42688.c unchanged; change only
 * this value after repeating the mechanical reference test. */
#define GYRO_X_SCALE_CAL  1.0f
#define GYRO_Y_SCALE_CAL  1.0f
#define GYRO_Z_SCALE_CAL  3.469f

/* calGyroVar: 计算陀螺仪方差并检测静止状态
 * 参数:
 *   d[]  - 当前陀螺仪数据[gx, gy, gz]
 *   len  - 滑动窗口长度(本项目用100)
 *   sq[] - 输出方差
 *   avg[] - 输出平均值
 * 算法:
 *   1. 滑动窗口更新数据
 *   2. 计算窗口内数据的均值和方差
 *   3. 方差小于阈值时认为静止,更新零偏估计
 * 方差公式: Var = E[X²] - E[X]² = (Σx² - (Σx)²/n) / n
 */
static void calGyroVar(float d[], int len, float sq[], float avg[]) {
    int i; double tl;
    /* 首次填充窗口 */
    if(!Gif){for(i=0;i<3;i++){Gf[i][Gc]=d[i];Gt[i]+=d[i];sqGt[i]+=d[i]*d[i];sq[i]=100;avg[i]=0;}}
    /* 滑动窗口更新 */
    else{for(i=0;i<3;i++){Gt[i]-=Gf[i][Gc];sqGt[i]-=Gf[i][Gc]*Gf[i][Gc];Gf[i][Gc]=d[i];Gt[i]+=d[i];sqGt[i]+=d[i]*d[i];}}
    if(++Gc>=len){Gc=0;Gif=1;Kp=0.5f;}  /* 窗口填满后降低Kp */
    if(!Gif)return;
    tl=len;
    /* 计算均值和方差 */
    for(i=0;i<3;i++){avg[i]=(float)(Gt[i]/tl);sq[i]=(float)((sqGt[i]-Gt[i]*Gt[i]/tl)/tl);}
}

/* getVals: 获取并预处理传感器数据
 * 参数: v[] - 输出数组[ax,ay,az,gx,gy,gz,mx,my,mz]
 * 说明:
 *   1. 读取加速度计和陀螺仪原始数据
 *   2. 进行零偏补偿
 *   3. 静止检测并更新零偏估计
 *   4. 注意: 本项目未使用磁力计,所以mx,my,mz为0
 */

static int8_t getVals(float *v) {
    icm42688_real_data_t av,gv;
    float sqr[3],avgr[3];

    /* 读取6轴传感器数据 */
    int8_t status = ICM42688_ReadMotion6(&av,&gv);//读取数据
    if (status != 0) return status;

    /* 缓存数据到TTangles_gyro数组 */
    TTangles_gyro[0]=av.x; TTangles_gyro[1]=av.y; TTangles_gyro[2]=av.z;
    TTangles_gyro[3]=gv.x; TTangles_gyro[4]=gv.y; TTangles_gyro[5]=gv.z;
    TTangles_gyro[6]=0;


    /* 计算陀螺仪方差,检测静止状态 */
    calGyroVar(&TTangles_gyro[3],100,sqr,avgr);

    /* 静止检测: 连续100次方差<0.02认为静止,更新零偏 */
    if(sqr[0]<0.02f&&sqr[1]<0.02f&&sqr[2]<0.02f&&Cc>=99){
        go[0]=avgr[0]; go[1]=avgr[1]; go[2]=avgr[2];  /* 更新零偏估计 */
        exInt=eyInt=ezInt=0;  /* 静止时重置积分项 */
        if (!gyroBiasReady) {
            q0=1.0f; q1=q2=q3=0.0f;
            gyroBiasReady = 1;
        }
        Cc=0;
    }else if(Cc<100)Cc++;

    if (!gyroBiasReady) return -2;

    /* 输出加速度计原始数据 */
    v[0]=av.x; v[1]=av.y; v[2]=av.z;
    /* 输出陀螺仪零偏补偿后的数据 */
    v[3]=(gv.x-go[0]) * GYRO_X_SCALE_CAL;
    v[4]=(gv.y-go[1]) * GYRO_Y_SCALE_CAL;
    v[5]=(gv.z-go[2]) * GYRO_Z_SCALE_CAL;
    return 0;
}

/* ============================================
 * AHRS算法常量
 * ============================================ */
#define AKi  0.005f    /* 积分系数(Accel Ki) - 积分项的增益 */

/* AHRSupdate: AHRS姿态更新核心算法
 * 参数:
 *   gx, gy, gz - 陀螺仪角速度(°/s)
 *   ax, ay, az - 加速度计数据(g)
 *   mx, my, mz - 磁力计数据(本项目未使用,全为0)
 * 算法步骤:
 *   1. 加速度计规范化 - 将加速度向量单位化
 *   2. 计算参考方向向量 - 从四元数推算的理论加速度方向
 *   3. 计算误差 - 实际加速度与理论的差异
 *   4. PI控制补偿 - 用误差修正陀螺仪
 *   5. 四元数更新 - 梯度下降法积分
 *   6. 四元数规范化 - 保持四元数为单位四元数
 *
 * 核心公式:
 *   四元数乘法: q_new = q ⊗ (1, gx*dt/2, gy*dt/2, gz*dt/2)
 *   其中⊗表示四元数乘法
 *
 *   误差计算(加速度计):
 *   期望重力方向(载体->地理): [0, 0, 1] (假设载体水平)
 *   由四元数推算的重力方向: vx,vy,vz
 *   误差 = 实际加速度叉乘期望重力方向
 *
 *   PI补偿:
 *   gx_corrected = gx + Kp*ex + Ki*∫ex*dt
 */
static void AHRSupdate(float gx,float gy,float gz,float ax,float ay,float az,
                       float mx,float my,float mz,float dt_seconds) {
    float n,vx,vy,vz,ex,ey,ez,tq0,tq1,tq2,tq3;
    float halfDt = 0.5f * dt_seconds;

    /* 预计算四元数分量乘积(用于后续计算) */
    float q0q0=q0*q0,q0q1=q0*q1,q0q2=q0*q2;
    float q1q1=q1*q1,q1q3=q1*q3;
    float q2q2=q2*q2,q2q3=q2*q3,q3q3=q3*q3;

    /* 规范化加速度计向量(转换为单位向量)
     * n = |a| = sqrt(ax²+ay²+az²)
     * a_normalized = a / |a| */
    n=invSqrt1(ax*ax+ay*ay+az*az); ax*=n; ay*=n; az*=n;

    /* 规范化磁力计向量(本项目未使用,始终为0) */
    if ((mx*mx+my*my+mz*mz) > 0.0f) {
        n=invSqrt1(mx*mx+my*my+mz*mz); mx*=n; my*=n; mz*=n;
    }

    /* 从四元数计算重力向量在载体坐标系的投影(北向)
     * 地理坐标系中,重力方向为[0, 0, 1]
     * 通过旋转矩阵转换到载体坐标系
     * vx,vy,vz 表示重力向量在载体X,Y,Z轴的分量 */
    vx=2*(q1q3-q0q2); vy=2*(q0q1+q2q3); vz=q0q0-q1q1-q2q2+q3q3;

    /* 计算北向向量(地理坐标系X轴在载体坐标系的投影) */
    north.x=1-2*(q3*q3+q2*q2); north.y=2*(-q0*q3+q1*q2); north.z=2*(q0*q2-q1*q3);

    /* 计算西向向量(地理坐标系Y轴在载体坐标系的投影) */
    west.x=2*(q0*q3+q1*q2); west.y=1-2*(q3*q3+q1*q1); west.z=2*(-q0*q1+q2*q3);

    /* 计算误差向量 - 实际加速度与四元数推算的差异
     * 误差 = ay*vz - az*vy (叉乘的Y分量)
     *       = az*vx - ax*vz (叉乘的Z分量)
     *       = ax*vy - ay*vx (叉乘的X分量)
     * 物理意义: 如果载体静止,加速度计只测到重力
     *          任何与理论值的偏差就是传感器误差或外部运动 */
    ex=(ay*vz-az*vy); ey=(az*vx-ax*vz); ez=(ax*vy-ay*vx);

    /* PI控制器补偿
     * 如果误差不为零,说明陀螺仪积分结果与加速度计不符
     * 需要用加速度计的测量来修正陀螺仪的漂移
     *
     * 积分项: exInt += ex * AKi * AHT
     * 含义: 累积误差,用于消除稳态误差
     *
     * 比例项: Kp * ex
     * 含义: 当前误差的直接补偿,提供即时响应
     *
     * 总补偿: gx += Kp*ex + exInt
     *
     * 快速转动保护: 当角速度超过300°/s(约5.24rad/s)时,
     * 加速度计会测到向心加速度,此时跳过PI修正,
     * 纯靠陀螺仪积分,避免加速度计"帮倒忙" */
    float gyro_mag = gx*gx + gy*gy + gz*gz;
    if (gyro_mag < 27.5f && (ex!=0||ey!=0||ez!=0)) {
        exInt+=ex*AKi*dt_seconds; eyInt+=ey*AKi*dt_seconds; ezInt+=ez*AKi*dt_seconds;
        gx+=Kp*ex+exInt; gy+=Kp*ey+eyInt; gz+=Kp*ez+ezInt;
    }

    /* 四元数更新(半步长积分法)
     * 公式: q(t+dt) = q(t) + dt/2 * Ω(t) * q(t)
     * 其中Ω是角速度四元数[0, gx, gy, gz]
     *
     * 四元数乘法结果:
     * tq0 = q0 - (q1*gx + q2*gy + q3*gz) * AHT
     * tq1 = q1 + (q0*gx + q2*gz - q3*gy) * AHT
     * tq2 = q2 + (q0*gy - q1*gz + q3*gx) * AHT
     * tq3 = q3 + (q0*gz + q1*gy - q2*gx) * AHT */
    tq0=q0+(-q1*gx-q2*gy-q3*gz)*halfDt;
    tq1=q1+(q0*gx+q2*gz-q3*gy)*halfDt;
    tq2=q2+(q0*gy-q1*gz+q3*gx)*halfDt;
    tq3=q3+(q0*gz+q1*gy-q2*gx)*halfDt;

    /* 规范化四元数(确保四元数为单位长度)
     * 原因: 积分过程中可能出现数值误差累积
     *       规范化可以保证四元数表示的旋转是正确的
     * 公式: q_normalized = q / |q| = q / sqrt(q0²+q1²+q2²+q3²) */
    n=invSqrt1(tq0*tq0+tq1*tq1+tq2*tq2+tq3*tq3);
    q0=tq0*n; q1=tq1*n; q2=tq2*n; q3=tq3*n;
}

/* ============================================
 * 传感器数据缓存
 * mqv[9]: [ax, ay, az, gx, gy, gz, mx, my, mz]
 * 说明: 9个float存储完整的9轴传感器数据
 * ============================================ */
static float mqv[9];

/* getQ: 获取当前姿态四元数
 * 参数: q[] - 输出四元数[q0, q1, q2, q3]
 * 流程:
 *   1. 获取传感器原始数据
 *   2. 将角度/秒转换为弧度/秒(乘以π/180)
 *   3. 调用AHRS更新算法
 *   4. 返回更新后的四元数
 */
static int8_t getQ(float *q, float dt_seconds) {
    int8_t status = getVals(mqv);
    if (status != 0) return status;
    /* 将dps转换为rad/s: 角度制×π/180 */
    AHRSupdate(mqv[3]*3.1415926535f/180,mqv[4]*3.1415926535f/180,mqv[5]*3.1415926535f/180,
               mqv[0],mqv[1],mqv[2],mqv[6],mqv[7],mqv[8],dt_seconds);
    q[0]=q0; q[1]=q1; q[2]=q2; q[3]=q3;
    return 0;
}

/* IMU_getYawPitchRoll: 获取姿态角(欧拉角)
 * 参数: a[] - 输出数组[yaw, pitch, roll], 单位:度
 * 说明:
 *   Yaw (偏航角): 绕Z轴旋转 [-180°, 180°]
 *   Pitch (俯仰角): 绕X轴旋转 [-90°, 90°]
 *   Roll (翻滚角): 绕Y轴旋转 [-180°, 180°]
 *
 * 欧拉角转四元数公式(已知四元数求欧拉角):
 *
 *   Roll = atan2(2*(q2*q3+q0*q1), 1-2*(q1²+q2²))
 *   Pitch = -asin(-2*q1*q3+2*q0*q2)
 *   Yaw = atan2(2*(q1*q2+q0*q3), 1-2*(q2²+q3²))
 *
 * 注意: 不同坐标系定义会导致公式略有差异
 *       本代码使用了特定的坐标转换约定
 */
int8_t IMU_getYawPitchRoll(float *a, float dt_seconds) {
    float q[4];
    int8_t status;
    if (a == 0 || dt_seconds <= 0.0f) return -3;
    status = getQ(q, dt_seconds);
    if (status != 0) return status;

    /* 计算Yaw (偏航角) - 绕Z轴旋转
     * atan2(2*(q1*q2+q0*q3), -2*q2²-2*q3²+1)
     * 转换为度: 弧度值 × 180/π */
    a[0]=-atan2f(2*q[1]*q[2]+2*q[0]*q[3],
                 -2*q[2]*q[2]-2*q[3]*q[3]+1)*180/3.1415926535f;

    /* 计算Pitch (俯仰角) - 绕X轴旋转
     * asin(-2*q1*q3+2*q0*q2)
     * 使用asin而不是atan2,因为俯仰角范围是[-90°,90°] */
    a[1]=-asinf(-2*q[1]*q[3]+2*q[0]*q[2])*180/3.1415926535f;

    /* 计算Roll (翻滚角) - 绕Y轴旋转
     * atan2(2*(q2*q3+q0*q1), -2*q1²-2*q2²+1) */
    a[2]=atan2f(2*q[2]*q[3]+2*q[0]*q[1],-2*q[1]*q[1]-2*q[2]*q[2]+1)*180/3.1415926535f;
    return 0;
}

/* IMU_TT_getgyro: 获取原始陀螺仪数据(调试用)
 * 参数: z[] - 输出数组[ax,ay,az,gx,gy,gz,0]
 * 说明: 直接返回传感器缓存数据,不做任何处理
 */
void IMU_TT_getgyro(float *z) {
    z[0]=TTangles_gyro[0]; z[1]=TTangles_gyro[1]; z[2]=TTangles_gyro[2];
    z[3]=TTangles_gyro[3]; z[4]=TTangles_gyro[4]; z[5]=TTangles_gyro[5];
    z[6]=TTangles_gyro[6];
}

void IMU_GetCorrectedGyro(float *gyro) {
    if (gyro == 0) return;
    gyro[0] = mqv[3];
    gyro[1] = mqv[4];
    gyro[2] = mqv[5];
}

/* MPU6050_InitAng_Offset: 传感器零偏初始化(兼容函数)
 * 说明: 本项目ICM42688不需要额外零偏初始化
 *       此函数仅保持与MPU6050接口的兼容性
 */
void MPU6050_InitAng_Offset(void) {}

 float Yaw_Error(float target, float current)
{
    float err = target - current;
    while (err > 180.0f) err -= 360.0f;
    while (err < -180.0f) err += 360.0f;
    return err;
}
